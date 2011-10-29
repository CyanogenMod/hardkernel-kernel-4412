/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2010-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file mali_kbase_core.c
 * Base kernel driver init.
 */

#include <osk/mali_osk.h>
#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_uku.h>
#include <kbase/src/common/mali_kbase_mem.h>
#include <kbase/src/common/mali_midg_regmap.h>
#include <kbase/src/linux/mali_kbase_mem_linux.h>
#include <uk/mali_ukk.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/miscdevice.h>
#include <linux/list.h>
#include <linux/semaphore.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#ifdef CONFIG_VITHAR
#include <mach/map.h>
#include <kbase/src/platform/mali_kbase_platform.h>
#include <kbase/src/platform/mali_kbase_runtime_pm.h>
#endif

#ifdef CONFIG_ARM
#define FAKE_PLATFORM_DEVICE	1
#ifdef CONFIG_ARCH_VEXPRESS
#define FAKE_PLATFORM_TYPE "mali-t6f1"
#else

#ifdef CONFIG_VITHAR
#define FAKE_PLATFORM_TYPE "mali-t604"
#else
#define FAKE_PLATFORM_TYPE "mali-t6xm"
#endif
#endif
#endif

#define	JOB_IRQ_TAG	0
#define MMU_IRQ_TAG	1
#define GPU_IRQ_TAG	2

struct kbase_irq_table
{
	u32		tag;
	irq_handler_t	handler;
};

static const char kbase_drv_name[] = KBASE_DRV_NAME;

static int kbase_dev_nr;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
static DEFINE_SEMAPHORE(kbase_dev_list_lock);
#else
static DECLARE_MUTEX(kbase_dev_list_lock);
#endif
static LIST_HEAD(kbase_dev_list);

static mali_error kbase_dispatch(ukk_call_context * const ukk_ctx, void * const args, u32 args_size)
{
	struct kbase_context *kctx;
	struct kbase_device *kbdev;
	uk_header *ukh = args;
	u32 id;

	OSKP_ASSERT( ukh != NULL );

	kctx = CONTAINER_OF(ukk_session_get(ukk_ctx), kbase_context, ukk_session);
	kbdev = kctx->kbdev;
	id = ukh->id;
	ukh->ret = MALI_ERROR_NONE; /* Be optimistic */

	switch(id)
	{
		case KBASE_FUNC_TMEM_ALLOC:
		{
			kbase_uk_tmem_alloc *tmem = args;
			struct kbase_va_region *reg;

			if (sizeof(*tmem) != args_size)
				goto bad_size;

			reg = kbase_tmem_alloc(kctx, tmem->vsize, tmem->psize,
					       tmem->extent, tmem->flags);
			if (reg)
			{
				tmem->gpu_addr	= reg->start_pfn << 12;
			}
			else
				ukh->ret = MALI_ERROR_FUNCTION_FAILED;
			break;
		}

		case KBASE_FUNC_TMEM_FROM_UMP:
#if MALI_KBASE_USE_UMP
		{
			kbase_uk_tmem_from_ump * tmem_ump = args;
			struct kbase_va_region *reg;

			if (sizeof(*tmem_ump) != args_size)
				goto bad_size;
			reg = kbase_tmem_from_ump(kctx, tmem_ump->id, &tmem_ump->pages);
			if (reg)
			{
				tmem_ump->gpu_addr = reg->start_pfn << 12;
			}
			else
			{
				ukh->ret = MALI_ERROR_FUNCTION_FAILED;
			}
			break;
		}
#else
		{
			ukh->ret = MALI_ERROR_FUNCTION_FAILED;
			break;
		}
#endif

		case KBASE_FUNC_PMEM_ALLOC:
		{
			kbase_uk_pmem_alloc *pmem = args;
			struct kbase_va_region *reg;

			if (sizeof(*pmem) != args_size)
				goto bad_size;
			reg = kbase_pmem_alloc(kctx, pmem->vsize, pmem->flags,
					       &pmem->cookie);
			if (!reg)
				ukh->ret = MALI_ERROR_FUNCTION_FAILED;
			break;
		}

		case KBASE_FUNC_MEM_FREE:
		{
			kbase_uk_mem_free *mem = args;

			if (sizeof(*mem) != args_size)
				goto bad_size;
			if (kbase_mem_free(kctx, mem->gpu_addr))
				ukh->ret = MALI_ERROR_FUNCTION_FAILED;
			break;
		}

		case KBASE_FUNC_JOB_SUBMIT:
		{
			kbase_uk_job_submit * job = args;
			
			if (sizeof(*job) != args_size)
				goto bad_size;

			if (MALI_ERROR_NONE != kbase_jd_submit(kctx, job))
				ukh->ret = MALI_ERROR_FUNCTION_FAILED;
			break;
		}

		case KBASE_FUNC_SYNC:
		{
			kbase_uk_sync_now *sn = args;

			if (sizeof(*sn) != args_size)
				goto bad_size;

			kbase_sync_now(kctx, &sn->sset);
			break;
		}

		case KBASE_FUNC_POST_TERM:
		{
			kbase_event_close(kctx);
			break;
		}

		case KBASE_FUNC_HWCNT_SETUP:
		{
			kbase_uk_hwcnt_setup * setup = args;

			if (sizeof(*setup) != args_size)
				goto bad_size;

			if (MALI_ERROR_NONE != kbase_instr_hwcnt_setup(kctx, setup))
			{
				ukh->ret = MALI_ERROR_FUNCTION_FAILED;
			}
			break;
		}

		case KBASE_FUNC_HWCNT_DUMP:
		{
			/* args ignored */
			if (MALI_ERROR_NONE != kbase_instr_hwcnt_dump(kctx))
			{
				ukh->ret = MALI_ERROR_FUNCTION_FAILED;
			}
			break;
		}

		case KBASE_FUNC_TMEM_RESIZE:
		{
			kbase_uk_tmem_resize *resize = args;
			
			if (sizeof(*resize) != args_size)
				goto bad_size;

			ukh->ret = kbase_tmem_resize(kctx, resize->gpu_addr, resize->delta, &resize->size, &resize->result_subcode);
			break;
		}

		case KBASE_FUNC_FIND_CPU_MAPPING:
		{
			kbase_uk_find_cpu_mapping *find = args;
			struct kbase_cpu_mapping *map;

			if (sizeof(*find) != args_size)
			{
				goto bad_size;
			}

			OSKP_ASSERT( find != NULL );
			if ( find->size > SIZE_MAX || find->cpu_addr > UINTPTR_MAX )
			{
				map = NULL;
			}
			else
			{
				map = kbasep_find_enclosing_cpu_mapping( kctx,
				                                         find->gpu_addr,
				                                         (osk_virt_addr)(uintptr_t)find->cpu_addr,
				                                         (size_t)find->size );
			}

			if ( NULL != map )
			{
				find->uaddr = PTR_TO_U64( map->uaddr );
				find->nr_pages = map->nr_pages;
				find->page_off = map->page_off;
			}
			else
			{
				ukh->ret = MALI_ERROR_FUNCTION_FAILED;
			}
			break;
		}

		default:
			dev_err(kbdev->osdev.dev, "unknown syscall %08x", ukh->id);
			goto out_bad;
	}

	return MALI_ERROR_NONE;

bad_size:
	dev_err(kbdev->osdev.dev, "Wrong syscall size (%d) for %08x\n", args_size, ukh->id);
out_bad:
	return MALI_ERROR_FUNCTION_FAILED;
}

static struct kbase_device *to_kbase_device(struct device *dev)
{
	return dev_get_drvdata(dev);
}

static int kbase_open(struct inode *inode, struct file *filp)
{
	struct list_head *entry;
	struct kbase_device *kbdev = NULL;
	struct kbase_context *kctx;

	down(&kbase_dev_list_lock);
	list_for_each(entry, &kbase_dev_list)
	{
		struct kbase_device *tmp;

		tmp = list_entry(entry, struct kbase_device, osdev.entry);
		if (tmp->osdev.mdev.minor == iminor(inode))
		{
			kbdev = tmp;
			break;
		}
	}
	up(&kbase_dev_list_lock);

	if (!kbdev)
		return -ENODEV;

	get_device(kbdev->osdev.dev);

	kctx = kbase_create_context(kbdev);
	if (!kctx)
	{
		put_device(kbdev->osdev.dev);
		return -ENOMEM;
	}
    
	if (MALI_ERROR_NONE != ukk_session_init(&kctx->ukk_session, kbase_dispatch, BASE_UK_VERSION_MAJOR, BASE_UK_VERSION_MINOR))
	{
		kbase_destroy_context(kctx);
		put_device(kbdev->osdev.dev);
	        return -EFAULT;
	}

	init_waitqueue_head(&kctx->osctx.event_queue);
	filp->private_data = kctx;

	dev_dbg(kbdev->osdev.dev, "created base context\n");
	return 0;
}

static int kbase_release(struct inode *inode, struct file *filp)
{
	struct kbase_context *kctx = filp->private_data;
	struct kbase_device *kbdev = kctx->kbdev;

	ukk_session_term(&kctx->ukk_session);
	filp->private_data = NULL;
	kbase_destroy_context(kctx);
	dev_dbg(kbdev->osdev.dev, "deleted base context\n");
	put_device(kbdev->osdev.dev);

	return 0;
}

static long kbase_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	u64 msg[(UKK_CALL_MAX_SIZE+7)>>3]; /* alignment fixup */
	u32 size = _IOC_SIZE(cmd);
	ukk_call_context ukk_ctx;
	struct kbase_context *kctx = filp->private_data;

	if (size > UKK_CALL_MAX_SIZE) return -ENOTTY;

	if (0 != copy_from_user(&msg, (void *)arg, size))
	{
		return -EFAULT;
	}

	ukk_call_prepare(&ukk_ctx, &kctx->ukk_session, NULL);

	if (MALI_ERROR_NONE != ukk_dispatch(&ukk_ctx, &msg, size))
	{
		return -EFAULT;
	}

	if (0 != copy_to_user((void *)arg, &msg, size))
	{
		pr_err("failed to copy results of UK call back to user space\n");
		return -EFAULT;
	}
	return 0;
}

static ssize_t kbase_read(struct file *filp, char __user *buf,
			  size_t count, loff_t *f_pos)
{
	struct kbase_context *kctx = filp->private_data;
	base_jd_event uevent;

	if (count < sizeof(uevent))
		return -ENOBUFS;

	while (kbase_event_dequeue(kctx, &uevent))
	{
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(kctx->osctx.event_queue,
					     kbase_event_pending(kctx)))
			return -ERESTARTSYS;
	}

	if (copy_to_user(buf, &uevent, sizeof(uevent)))
	{
		return -EFAULT;
	}

	return sizeof(uevent);
}

static unsigned int kbase_poll(struct file *filp, poll_table *wait)
{
	struct kbase_context *kctx = filp->private_data;

	poll_wait(filp, &kctx->osctx.event_queue, wait);
	if (kbase_event_pending(kctx))
		return POLLIN | POLLRDNORM;

	return 0;
}

void kbase_event_wakeup(kbase_context *kctx)
{
	wake_up_interruptible(&kctx->osctx.event_queue);
}

static const struct file_operations kbase_fops =
{
	.owner		= THIS_MODULE,
	.open		= kbase_open,
	.release	= kbase_release,
	.read		= kbase_read,
	.poll		= kbase_poll,
	.unlocked_ioctl	= kbase_ioctl,
	.mmap		= kbase_mmap,
};

void kbase_os_reg_write(kbase_device *kbdev, u16 offset, u32 value)
{
	writel(value, kbdev->osdev.reg + offset);
}

u32 kbase_os_reg_read(kbase_device *kbdev, u16 offset)
{
	return readl(kbdev->osdev.reg + offset);
}

static void *kbase_tag(void *ptr, u32 tag)
{
	return (void *)(((uintptr_t) ptr) | tag);
}

static void *kbase_untag(void *ptr)
{
	return (void *)(((uintptr_t) ptr) & ~3);
}

static irqreturn_t kbase_job_irq_handler(int irq, void *data)
{
	struct kbase_device *kbdev	= kbase_untag(data);
	u32 val;

	val = kbase_reg_read(kbdev, JOB_CONTROL_REG(JOB_IRQ_RAWSTAT), NULL);
	if (!val)
		return IRQ_NONE;

	dev_dbg(kbdev->osdev.dev, "%s: irq %d rawstat 0x%x\n", __func__, irq, val);

	kbase_job_done(kbdev, val);

	return IRQ_HANDLED;
}

static irqreturn_t kbase_mmu_irq_handler(int irq, void *data)
{
	struct kbase_device *kbdev	= kbase_untag(data);
	u32 val;

	val = kbase_reg_read(kbdev, MMU_REG(MMU_IRQ_STATUS), NULL);
	if (!val)
		return IRQ_NONE;

	dev_dbg(kbdev->osdev.dev, "%s: irq %d irqstatus 0x%x\n", __func__, irq, val);

	kbase_mmu_interrupt(kbdev, val);

	return IRQ_HANDLED;
}

static irqreturn_t kbase_gpu_irq_handler(int irq, void *data)
{
	struct kbase_device *kbdev	= kbase_untag(data);
	u32 val = 0;
	u32 tval;

	while (1)
	{
		tval = kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_IRQ_RAWSTAT), NULL);
		if (!tval)
			break;

		if (val & GPU_FAULT)
		{
			kbase_report_gpu_fault(kbdev, val & MULTIPLE_GPU_FAULTS);
		}

		kbase_reg_write(kbdev, GPU_CONTROL_REG(GPU_IRQ_CLEAR), tval, NULL);
		val |= tval;
	}

	if (!val)
		return IRQ_NONE;

	dev_dbg(kbdev->osdev.dev, "%s: irq %d rawstat 0x%x\n", __func__, irq, val);
	
	if (val & (POWER_CHANGED_ALL | POWER_CHANGED_SINGLE))
	{
		kbase_pm_send_event(kbdev, KBASE_PM_EVENT_STATE_CHANGED);
	}
	if (val & RESET_COMPLETED)
	{
		kbase_pm_reset_done(kbdev);
	}

	return IRQ_HANDLED;
}

static irq_handler_t kbase_handler_table[] = {
	[JOB_IRQ_TAG] = kbase_job_irq_handler,
	[MMU_IRQ_TAG] = kbase_mmu_irq_handler,
	[GPU_IRQ_TAG] = kbase_gpu_irq_handler,
};

static int kbase_install_interrupts(kbase_device *kbdev)
{
	struct kbase_os_device *osdev = &kbdev->osdev;
	int nr = ARRAY_SIZE(kbase_handler_table);
	int err;
	u32 i;

	BUG_ON(nr >= 4);	/* Only 3 interrupts! */

	for (i = 0; i < nr; i++)
	{
		err = request_irq(osdev->irqs[i].irq,
				  kbase_handler_table[i],
				  osdev->irqs[i].flags | IRQF_SHARED,
				  dev_name(osdev->dev),
				  kbase_tag(kbdev, i));
		if (err)
		{
			dev_err(osdev->dev, "Can't request interrupt %d (index %d)\n", osdev->irqs[i].irq, i);
			goto release;
		}
	}

	return 0;

release:
	while (i-- > 0)
	{
		free_irq(osdev->irqs[i].irq, kbase_tag(kbdev, i));
	}

	return err;
}

static void kbase_release_interrupts(kbase_device *kbdev)
{
	struct kbase_os_device *osdev = &kbdev->osdev;
	int nr = ARRAY_SIZE(kbase_handler_table);
	u32 i;

	for (i = 0; i < nr; i++)
	{
		if (osdev->irqs[i].irq)
			free_irq(osdev->irqs[i].irq, kbase_tag(kbdev, i));
	}
}

/** Show callback for the @c power_policy sysfs file.
 *
 * This function is called to get the contents of the @c power_policy sysfs
 * file. This is a list of the available policies with the currently active one
 * surrounded by square brackets.
 *
 * @param dev	The device this sysfs file is for
 * @param attr	The attributes of the sysfs file
 * @param buf	The output buffer for the sysfs file contents
 *
 * @return The number of bytes output to @c buf.
 */
static ssize_t show_policy(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev;
	const struct kbase_pm_policy *current_policy;
	const struct kbase_pm_policy **policy_list;
	int policy_count;
	int i;
	ssize_t ret = 0;

	kbdev = to_kbase_device(dev);

	if (!kbdev)
		return -ENODEV;

	current_policy = kbase_pm_get_policy(kbdev);

	policy_count = kbase_pm_list_policies(&policy_list);

	for(i=0; i<policy_count && ret<PAGE_SIZE; i++)
	{
		if (policy_list[i] == current_policy)
			ret += snprintf(buf+ret, PAGE_SIZE - ret, "[%s] ", policy_list[i]->name);
		else
			ret += snprintf(buf+ret, PAGE_SIZE - ret, "%s ", policy_list[i]->name);
	}

	if (ret < PAGE_SIZE - 1)
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "\n");
	else
	{
		buf[PAGE_SIZE-2] = '\n';
		buf[PAGE_SIZE-1] = '\0';
		ret = PAGE_SIZE-1;
	}

	return ret;
}

/** Store callback for the @c power_policy sysfs file.
 *
 * This function is called when the @c power_policy sysfs file is written to.
 * It matches the requested policy against the available policies and if a
 * matching policy is found calls @ref kbase_pm_set_policy to change the
 * policy.
 *
 * @param dev	The device with sysfs file is for
 * @param attr	The attributes of the sysfs file
 * @param buf	The value written to the sysfs file
 * @param count	The number of bytes written to the sysfs file
 *
 * @return @c count if the function succeeded. An error code on failure.
 */
static ssize_t set_policy(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct kbase_device *kbdev;
	const struct kbase_pm_policy *new_policy = NULL;
	const struct kbase_pm_policy **policy_list;
	int policy_count;
	int i;

	kbdev = to_kbase_device(dev);

	if (!kbdev)
		return -ENODEV;

	policy_count = kbase_pm_list_policies(&policy_list);

	for(i=0; i<policy_count; i++)
	{
		if (sysfs_streq(policy_list[i]->name, buf))
		{
			new_policy = policy_list[i];
			break;
		}
	}

	if (!new_policy) {
		dev_err(dev, "power_policy: policy not found\n");
		return -EINVAL;
	}

	kbase_pm_set_policy(kbdev, new_policy);
	return count;
}

/** The sysfs file @c power_policy.
 *
 * This is used for obtaining information about the available policies,
 * determining which policy is currently active, and changing the active
 * policy.
 */
DEVICE_ATTR(power_policy, S_IRUGO|S_IWUSR, show_policy, set_policy);

static int kbase_common_device_init(kbase_device *kbdev)
{
	struct kbase_os_device	*osdev = &kbdev->osdev;
	int err;
	mali_error mali_err;

	/* kbase_device_create uses calloc - no need to call kbasep_js_devdata_zeroinit( kbdev ) */

	osdev->reg_res = request_mem_region(osdev->reg_start,
					    osdev->reg_size,
					    dev_name(osdev->dev));
	if (!osdev->reg_res) {
		dev_err(osdev->dev, "Register window unavailable\n");
		err = -EIO;
		goto out;
	}

	osdev->reg = ioremap(osdev->reg_start, osdev->reg_size);
	if (!osdev->reg) {
		dev_err(osdev->dev, "Can't remap register window\n");
		err = -EINVAL;
		goto out_free_dev;
	}

	err = kbase_install_interrupts(kbdev);
	if (err)
		goto out_unmap;

	dev_set_drvdata(osdev->dev, kbdev);
#ifdef CONFIG_VITHAR_DEVICE_NODE_CREATION_IN_RUNTIME
	osdev->mdev.minor	= MISC_DYNAMIC_MINOR;
#else
	/* FPGA use ROM filesystem, so we cannot MISC device node */
	osdev->mdev.minor	= 77;
#endif
	osdev->mdev.name	= osdev->devname;
	osdev->mdev.fops	= &kbase_fops;
	osdev->mdev.parent	= get_device(osdev->dev);
	scnprintf(osdev->devname, DEVNAME_SIZE, "%s%d", kbase_drv_name, kbase_dev_nr++);

	if (misc_register(&osdev->mdev))
	{
		dev_err(osdev->dev, "Couldn't register misc dev %s\n", osdev->devname);
		err = -EINVAL;
		goto out_interrupts;
	}

	if (device_create_file(osdev->dev, &dev_attr_power_policy))
	{
		dev_err(osdev->dev, "Couldn't create sysfs file\n");
		goto out_register;
	}

#ifdef CONFIG_VITHAR
	if(kbase_platform_create_sysfs_file(osdev->dev))
		goto out_register;
#endif

	down(&kbase_dev_list_lock);
	list_add(&osdev->entry, &kbase_dev_list);
	up(&kbase_dev_list_lock);

	dev_info(osdev->dev, "Probed as %s\n", dev_name(osdev->mdev.this_device));
	mali_err = kbase_job_slot_init(kbdev);
	if (MALI_ERROR_NONE != mali_err)
	{
		goto out_create_file;
	}

	mali_err = kbase_pm_init(kbdev);
	if (MALI_ERROR_NONE != mali_err)
	{
		goto out_job_slot;
	}

	mali_err = kbasep_js_devdata_init(kbdev);
	if (MALI_ERROR_NONE != mali_err)
	{
		goto out_pm;
	}

	return 0;
out_pm:
	kbasep_js_devdata_term(kbdev); /* Safe to call even when didn't initialize */
	kbase_pm_term(kbdev);
out_job_slot:
	kbase_job_slot_exit(kbdev);
out_create_file:
	device_remove_file(kbdev->osdev.dev, &dev_attr_power_policy);
#ifdef CONFIG_VITHAR
	kbase_platform_remove_sysfs_file(kbdev->osdev.dev);
#endif
out_register:
	misc_deregister(&kbdev->osdev.mdev);
out_interrupts:
	kbase_release_interrupts(kbdev);
	put_device(osdev->dev);
out_unmap:
	iounmap(osdev->reg);
out_free_dev:
	if (NULL != osdev->reg_res)
	{
		release_resource(osdev->reg_res);
	}
	kfree(osdev->reg_res);
out:
	return err;
}

static int kbase_pci_device_probe(struct pci_dev *pdev,
				  const struct pci_device_id *pci_id)
{
	const kbase_device_info	*dev_info;
	kbase_device		*kbdev;
	kbase_os_device		*osdev;
	int err;

	dev_info = &kbase_dev_info[pci_id->driver_data];
	kbdev = kbase_device_create(dev_info);
	if (!kbdev) {
		dev_err(&pdev->dev, "Can't allocate device\n");
		err = -ENOMEM;
		goto out;
	}

	osdev = &kbdev->osdev;
	osdev->dev = &pdev->dev;

	err = pci_enable_device(pdev);
	if (err)
		goto out_free_dev;;

	osdev->reg_start = pci_resource_start(pdev, 0);
	osdev->reg_size = pci_resource_len(pdev, 0);
	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM)) {
		err = -EINVAL;
		goto out_disable;
	}

	osdev->irqs[0].irq = pdev->irq;
	osdev->irqs[1].irq = pdev->irq;
	osdev->irqs[2].irq = pdev->irq;

	pci_set_master(pdev);

	err = kbase_common_device_init(kbdev);
	if (err)
		goto out_disable;

	return 0;

out_disable:
	pci_disable_device(pdev);
out_free_dev:
	kbase_device_destroy(kbdev);
out:
	return err;
}

static int kbase_platform_device_probe(struct platform_device *pdev)
{
	struct kbase_device	*kbdev;
	kbase_device_info	*dev_info;
	struct kbase_os_device	*osdev;
	struct resource		*reg_res;
	int			err;
	int			i;

	dev_info = (kbase_device_info *)pdev->id_entry->driver_data;
	kbdev = kbase_device_create(dev_info);
	if (!kbdev) {
		dev_err(&pdev->dev, "Can't allocate device\n");
		err = -ENOMEM;
		goto out;
	}

	osdev = &kbdev->osdev;
	osdev->dev = &pdev->dev;

	for (i = 0; i < 3; i++)
	{
		struct resource	*irq_res;

		irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
		if (!irq_res)
		{
			dev_err(osdev->dev, "No IRQ resource at index %d\n", i);
			err = -ENOENT;
			goto out_free_dev;
		}

		osdev->irqs[i].irq = irq_res->start;
		osdev->irqs[i].flags = (irq_res->flags & IRQF_TRIGGER_MASK);
	}

	reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!reg_res) {
		dev_err(&pdev->dev, "Invalid register resource\n");
		err = -ENOENT;
		goto out_free_dev;
	}

	osdev->reg_start = reg_res->start;
	osdev->reg_size = resource_size(reg_res);

#ifdef CONFIG_VITHAR
	if(kbase_platform_init(osdev->dev))
	{
	    err = -ENOENT;
	    goto out_free_dev;
	}
#endif

	err = kbase_common_device_init(kbdev);
	if (err)
		goto out_free_dev;

	return 0;

out_free_dev:
	kbase_device_destroy(kbdev);
out:
	return err;
}

static int kbase_common_device_remove(struct kbase_device *kbdev)
{
	/* Remove the sys power policy file */
	device_remove_file(kbdev->osdev.dev, &dev_attr_power_policy);
#ifdef CONFIG_VITHAR
	kbase_platform_remove_sysfs_file(kbdev->osdev.dev);
#endif
	kbasep_js_devdata_term(kbdev);
	kbase_pm_term(kbdev);
	kbase_job_slot_exit(kbdev);
	list_del(&kbdev->osdev.entry);
	misc_deregister(&kbdev->osdev.mdev);
	kbase_release_interrupts(kbdev);
	put_device(kbdev->osdev.dev);
	iounmap(kbdev->osdev.reg);
	release_resource(kbdev->osdev.reg_res);
	kfree(kbdev->osdev.reg_res);
	kbase_device_destroy(kbdev);

	return 0;
}

static void kbase_pci_device_remove(struct pci_dev *pdev)
{
	struct kbase_device *kbdev = to_kbase_device(&pdev->dev);

	if (!kbdev)
		return;

	kbase_common_device_remove(kbdev);
	pci_disable_device(pdev);
}

static int kbase_platform_device_remove(struct platform_device *pdev)
{
	struct kbase_device *kbdev = to_kbase_device(&pdev->dev);

	if (!kbdev)
		return -ENODEV;

	return kbase_common_device_remove(kbdev);
}

/** Suspend callback from the OS.
 *
 * This is called by Linux when the device should suspend.
 *
 * @param dev	The device to suspend
 *
 * @return A standard Linux error code
 */
static int kbase_device_suspend(struct device *dev)
{
	struct kbase_device *kbdev = to_kbase_device(dev);

	if (!kbdev)
		return -ENODEV;

	/* Send the event to the power policy */
	kbase_pm_send_event(kbdev, KBASE_PM_EVENT_SUSPEND);

	/* Wait for the policy to suspend the device */
	kbase_pm_wait_for_power_down(kbdev);

#ifdef CONFIG_VITHAR
	/* Turn off Host clock & power to Vithar */
	kbase_platform_clock_off(dev);
	kbase_platform_power_off(dev);
#endif

	return 0;
}

/** Resume callback from the OS.
 *
 * This is called by Linux when the device should resume from suspension.
 *
 * @param dev	The device to resume
 *
 * @return A standard Linux error code
 */
static int kbase_device_resume(struct device *dev)
{
	struct kbase_device *kbdev = to_kbase_device(dev);

	if (!kbdev)
		return -ENODEV;

#ifdef CONFIG_VITHAR
	/* Turn on Host clock & power to Vithar */
	kbase_platform_power_on(dev);
	kbase_platform_clock_on(dev);
#endif

	/* Send the event to the power policy */
	kbase_pm_send_event(kbdev, KBASE_PM_EVENT_RESUME);

	/* Wait for the policy to resume the device */
	kbase_pm_wait_for_power_up(kbdev);

    return 0;
}

#define kbdev_info(x) ((kernel_ulong_t)&kbase_dev_info[(x)])

static struct platform_device_id kbase_platform_id_table[] =
{
	{
		.name		= "mali-t6xm",
		.driver_data	= kbdev_info(KBASE_MALI_T6XM),
	},
	{
		.name		= "mali-t6f1",
		.driver_data	= kbdev_info(KBASE_MALI_T6F1),
	},
	{
		.name		= "mali-t601",
		.driver_data	= kbdev_info(KBASE_MALI_T601),
	},
	{
		.name		= "mali-t604",
		.driver_data	= kbdev_info(KBASE_MALI_T604),
	},
	{
		.name		= "mali-t608",
		.driver_data	= kbdev_info(KBASE_MALI_T608),
	},
	{},
};

MODULE_DEVICE_TABLE(platform, kbase_platform_id_table);

static DEFINE_PCI_DEVICE_TABLE(kbase_pci_id_table) = {
	{ PCI_DEVICE(0x13b5, 0x6956), 0, 0, KBASE_MALI_T6XM },
	{},
};
MODULE_DEVICE_TABLE(pci, kbase_pci_id_table);

/** The power management operations for the platform driver.
 */
static struct dev_pm_ops kbase_pm_ops =
{
	.suspend	= kbase_device_suspend,
	.resume		= kbase_device_resume,
#ifdef CONFIG_VITHAR_RT_PM
	.runtime_suspend	= kbase_device_runtime_suspend,
	.runtime_resume		= kbase_device_runtime_resume,
#endif
};

static struct platform_driver kbase_platform_driver =
{
	.probe		= kbase_platform_device_probe,
	.remove		= kbase_platform_device_remove,
	.driver		=
	{
		.name		= kbase_drv_name,
		.owner		= THIS_MODULE,
		.pm 		= &kbase_pm_ops,
	},
	.id_table	= kbase_platform_id_table,
};

static struct pci_driver kbase_pci_driver =
{
	.name		= KBASE_DRV_NAME,
	.probe		= kbase_pci_device_probe,
	.remove		= kbase_pci_device_remove,
	.id_table	= kbase_pci_id_table,
};

#ifdef FAKE_PLATFORM_DEVICE
static struct platform_device *mali_device;
#if defined(CONFIG_ARCH_VEXPRESS)
#define JOB_IRQ_NUMBER  68
#define MMU_IRQ_NUMBER  69
#define GPU_IRQ_NUMBER  70
#elif defined(CONFIG_MACH_REALVIEW_PBX)
#define JOB_IRQ_NUMBER  70
#define MMU_IRQ_NUMBER  70
#define GPU_IRQ_NUMBER  70
#elif MALI_ANDROID
#define JOB_IRQ_NUMBER  20
#define MMU_IRQ_NUMBER  20
#define GPU_IRQ_NUMBER  20
#elif defined(CONFIG_ARCH_EXYNOS5)
/* IRQs are defined in irqs-exynos5.h */
#else
#error FAKE_PLATFORM_DEVICE defined and no known IRQ assignments
#endif


static struct resource mali_resource[] = {
	[0] = {
#ifdef CONFIG_VITHAR
		.start	= EXYNOS5_PA_G3D,
		.end	= EXYNOS5_PA_G3D + (4096 * 5) - 1,
#else
		.start= 0xFC010000,
		.end= 0xFC010000 + (4096 * 5) - 1,
#endif
		.flags	= IORESOURCE_MEM,
	},
	[JOB_IRQ_TAG + 1] = {
		.start	= JOB_IRQ_NUMBER,
		.end	= JOB_IRQ_NUMBER,
		.flags	= (IORESOURCE_IRQ |
			   IORESOURCE_IRQ_HIGHLEVEL),
	},
	[MMU_IRQ_TAG + 1] = {
		.start	= MMU_IRQ_NUMBER,
		.end	= MMU_IRQ_NUMBER,
		.flags	= (IORESOURCE_IRQ |
			   IORESOURCE_IRQ_HIGHLEVEL),
	},
	[GPU_IRQ_TAG + 1] = {
		.start	= GPU_IRQ_NUMBER,
		.end	= GPU_IRQ_NUMBER,
		.flags	= (IORESOURCE_IRQ |
			   IORESOURCE_IRQ_HIGHLEVEL),
	},
};
#endif

static int __init kbase_driver_init(void)
{
	int err;
#ifdef FAKE_PLATFORM_DEVICE
	mali_device = platform_device_alloc(FAKE_PLATFORM_TYPE, 0);
	platform_device_add_resources(mali_device, mali_resource, 4);
	platform_device_add(mali_device);
#endif
	err = platform_driver_register(&kbase_platform_driver);
	if (err)
		return err;
	err = pci_register_driver(&kbase_pci_driver);
	if (err) {
		platform_driver_unregister(&kbase_platform_driver);
		return err;
	}
	return 0;
}

static void __exit kbase_driver_exit(void)
{
	pci_unregister_driver(&kbase_pci_driver);
	platform_driver_unregister(&kbase_platform_driver);
#ifdef FAKE_PLATFORM_DEVICE
	if (mali_device)
		platform_device_unregister(mali_device);
#endif
}

module_init(kbase_driver_init);
module_exit(kbase_driver_exit);

MODULE_LICENSE("GPL");
