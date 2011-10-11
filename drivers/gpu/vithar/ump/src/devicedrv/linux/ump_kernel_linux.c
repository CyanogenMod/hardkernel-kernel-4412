/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

#define UMP_LICENSE_IS_GPL 1 /* hardcoded for now */
#define UMP_KERNEL_LINUX_LICENSE "GPL"

#include <uk/mali_ukk.h>
#include <common/ump_kernel_uk.h>

#include <ump/ump_kernel_interface.h>

#include <linux/module.h>            /* kernel module definitions */
#include <linux/fs.h>                /* file system operations */
#include <linux/cdev.h>              /* character device definitions */
#include <linux/ioport.h>            /* request_mem_region */

#if UMP_LICENSE_IS_GPL
#include <linux/device.h>            /* class registration support */
#endif

#include <common/ump_kernel_core.h>

#include "ump_kernel_linux_mem.h"
#include <ump_arch.h>

struct ump_linux_device
{
	struct cdev cdev;
#if UMP_LICENSE_IS_GPL
	struct class * ump_class;
#endif
};

/* Name of the UMP device driver */
static char ump_dev_name[] = "ump"; /* should be const, but the functions we call requires non-cost */

/* Module parameter to control log level */
int ump_debug_level = 2;
module_param(ump_debug_level, int, S_IRUSR | S_IWUSR | S_IWGRP | S_IRGRP | S_IROTH); /* rw-rw-r-- */
MODULE_PARM_DESC(ump_debug_level, "Higher number, more dmesg output");

/* By default the module uses any available major, but it's possible to set it at load time to a specific number */
#ifdef CONFIG_VITHAR
int ump_major = 240;
#else
int ump_major = 0;
#endif

module_param(ump_major, int, S_IRUGO); /* r--r--r-- */
MODULE_PARM_DESC(ump_major, "Device major number");

char * ump_revision = UMP_SVN_REV_STRING;
module_param(ump_revision, charp, S_IRUGO); /* r--r--r-- */
MODULE_PARM_DESC(ump_revision, "Revision info");

static int umpp_linux_open(struct inode *inode, struct file *filp);
static int umpp_linux_release(struct inode *inode, struct file *filp);
#ifdef HAVE_UNLOCKED_IOCTL
static long umpp_linux_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int umpp_linux_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif

/* This variable defines the file operations this UMP device driver offers */
static struct file_operations ump_fops =
{
	.owner   = THIS_MODULE,
	.open    = umpp_linux_open,
	.release = umpp_linux_release,
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl   = umpp_linux_ioctl,
#else
	.ioctl   = umpp_linux_ioctl,
#endif
	.compat_ioctl = umpp_linux_ioctl,
	.mmap = umpp_linux_mmap
};


/* The global variable containing the global device data */
static struct ump_linux_device ump_linux_device;

#define DBG_MSG(level, ...) do { \
if ((level) <=  ump_debug_level)\
{\
OSK_PRINT_INFO(OSK_UMP, "UMP<" #level ">:\n" __VA_ARGS__ );\
} \
} while (0)

#define MSG_ERR(...) do{ \
OSK_PRINT_ERROR(OSK_UMP, "UMP: ERR: %s\n           %s()%4d\n", __FILE__, __func__  , __LINE__) ; \
OSK_PRINT_ERROR(OSK_UMP, __VA_ARGS__); \
OSK_PRINT_ERROR(OSK_UMP, "\n"); \
} while(0)

#define MSG(...) do{ \
OSK_PRINT_INFO(OSK_UMP, "UMP: " __VA_ARGS__);\
} while (0)

/*
 * This function is called by Linux to initialize this module.
 * All we do is initialize the UMP device driver.
 */
static int __init umpp_linux_initialize_module(void)
{
	ump_result err;

	DBG_MSG(2, "Inserting UMP device driver. Compiled: %s, time: %s\n", __DATE__, __TIME__);

	err = umpp_core_constructor();
	if (UMP_OK != err)
	{
		MSG_ERR(("UMP device driver init failed\n"));
		return -ENOTTY;
	}

	MSG("UMP device driver %s loaded\n", UMP_SVN_REV_STRING);

	return 0;
}



/*
 * This function is called by Linux to unload/terminate/exit/cleanup this module.
 * All we do is terminate the UMP device driver.
 */
static void __exit umpp_linux_cleanup_module(void)
{
	DBG_MSG(2, "Unloading UMP device driver\n");
	umpp_core_destructor();
	DBG_MSG(2, "Module unloaded\n");
}



/*
 * Initialize the UMP device driver.
 */
ump_result umpp_device_initialize(void)
{
	int err;
	dev_t dev = 0;

	if (0 == ump_major)
	{
		/* auto select a major */
		err = alloc_chrdev_region(&dev, 0, 1, ump_dev_name);
		ump_major = MAJOR(dev);
	}
	else
	{
		/* use load time defined major number */
		dev = MKDEV(ump_major, 0);
		err = register_chrdev_region(dev, 1, ump_dev_name);
	}

	if (0 == err)
	{
		memset(&ump_linux_device, 0, sizeof(ump_linux_device));

		/* initialize our char dev data */
		cdev_init(&ump_linux_device.cdev, &ump_fops);
		ump_linux_device.cdev.owner = THIS_MODULE;
		ump_linux_device.cdev.ops = &ump_fops;

		/* register char dev with the kernel */
		err = cdev_add(&ump_linux_device.cdev, dev, 1/*count*/);
		if (0 == err)
		{

#if UMP_LICENSE_IS_GPL
			ump_linux_device.ump_class = class_create(THIS_MODULE, ump_dev_name);
			if (IS_ERR(ump_linux_device.ump_class))
			{
				err = PTR_ERR(ump_linux_device.ump_class);
			}
			else
			{
				struct device * mdev;
				mdev = device_create(ump_linux_device.ump_class, NULL, dev, NULL, ump_dev_name);
				if (!IS_ERR(mdev))
				{
					return UMP_OK;
				}

				err = PTR_ERR(mdev);
				class_destroy(ump_linux_device.ump_class);
			}
			cdev_del(&ump_linux_device.cdev);
#else
			return UMP_OK;
#endif
		}

		unregister_chrdev_region(dev, 1);
	}

	return UMP_ERROR;
}



/*
 * Terminate the UMP device driver
 */
void umpp_device_terminate(void)
{
	dev_t dev = MKDEV(ump_major, 0);

#if UMP_LICENSE_IS_GPL
	device_destroy(ump_linux_device.ump_class, dev);
	class_destroy(ump_linux_device.ump_class);
#endif

	/* unregister char device */
	cdev_del(&ump_linux_device.cdev);

	/* free major */
	unregister_chrdev_region(dev, 1);
}


static int umpp_linux_open(struct inode *inode, struct file *filp)
{
	umpp_session *session;
	
	session = umpp_core_session_start();
	if (NULL == session) 
	{
		return -EFAULT;
	}
	
	filp->private_data = session;

	return 0;
}

static int umpp_linux_release(struct inode *inode, struct file *filp)
{
	umpp_session *session;
	
	session = filp->private_data;

	umpp_core_session_end(session);

	filp->private_data = NULL;

	return 0;
}

#ifdef HAVE_UNLOCKED_IOCTL
static long umpp_linux_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int umpp_linux_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
	u64 msg[(UKK_CALL_MAX_SIZE+7)>>3]; /* alignment fixup */
	u32 size = _IOC_SIZE(cmd);
	ukk_call_context ukk_ctx;
	struct umpp_session *session = filp->private_data;

#ifndef HAVE_UNLOCKED_IOCTL
	(void)inode; /* unused arg */
#endif

	if (size > UKK_CALL_MAX_SIZE) return -ENOTTY;

	if (0 != copy_from_user(&msg, (void *)arg, size))
	{
		return -EFAULT;
	}

	ukk_call_prepare(&ukk_ctx, &session->ukk_session, NULL);

	if (MALI_ERROR_NONE != ukk_dispatch(&ukk_ctx, &msg, size))
	{
		return -EFAULT;
	}

	if (0 != copy_to_user((void *)arg, &msg, size))
	{
		MSG_ERR("failed to copy results of UK call back to user space\n");
		return -EFAULT;
	}
	return 0;
}


/* Export UMP kernel space API functions */
EXPORT_SYMBOL(ump_dd_allocate);
EXPORT_SYMBOL(ump_dd_allocation_flags_get);
EXPORT_SYMBOL(ump_dd_resize);
EXPORT_SYMBOL(ump_dd_secure_id_get);
EXPORT_SYMBOL(ump_dd_from_secure_id);
EXPORT_SYMBOL(ump_dd_pin_size);
EXPORT_SYMBOL(ump_dd_unpin_size);
EXPORT_SYMBOL(ump_dd_phys_blocks_get);
EXPORT_SYMBOL(ump_dd_size_get);
EXPORT_SYMBOL(ump_dd_retain);
EXPORT_SYMBOL(ump_dd_release);
EXPORT_SYMBOL(ump_dd_create_from_phys_blocks);

/* Setup init and exit functions for this module */
module_init(umpp_linux_initialize_module);
module_exit(umpp_linux_cleanup_module);

/* And some module informatio */
MODULE_LICENSE(UMP_KERNEL_LINUX_LICENSE);
MODULE_AUTHOR("ARM Ltd.");
MODULE_VERSION(UMP_SVN_REV_STRING);
