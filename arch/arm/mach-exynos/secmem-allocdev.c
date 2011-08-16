/*
 * arch/arm/mach-exynos/secmem-allocdev.c
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/cma.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

struct secchunk_info {
	int		index;
	phys_addr_t	base;
	size_t		size;
};

static struct miscdevice thisdev;

static char *secmem_info[] = {
	"mfc",	/* 0 */
	"fimc",	/* 1 */
	NULL
};

#define SECMEM_IOC_CHUNKINFO	_IOWR('S', 1, struct secchunk_info)

static long secmem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case SECMEM_IOC_CHUNKINFO:
	{
		struct cma_info info;
		struct secchunk_info minfo;
		char **mname;
		int nbufs = 0;

		for (mname = secmem_info; *mname != NULL; mname++)
			nbufs++;

		if (nbufs == 0)
			return -ENOMEM;

		if (copy_from_user(&minfo, (void __user *)arg, sizeof(minfo)))
			return -EFAULT;

		if (minfo.index < 0)
			return -EINVAL;

		if (minfo.index >= nbufs) {
			minfo.index = -1; /* No more memory region */
		} else {

			if (cma_info(&info, thisdev.this_device,
					secmem_info[minfo.index]))
				return -EINVAL;

			minfo.base = info.lower_bound;
			minfo.size = info.total_size;
		}

		if (copy_to_user((void __user *)arg, &minfo, sizeof(minfo)))
			return -EFAULT;
	}
		break;
	default:
		return -ENOTTY;
	}

	return 0;
}

static struct file_operations secmem_fops = {
	.unlocked_ioctl = &secmem_ioctl,
};

static int __init secmem_init(void)
{
	thisdev.minor = MISC_DYNAMIC_MINOR;
	thisdev.name = "s5p-smem";
	thisdev.fops = &secmem_fops;
	thisdev.parent = NULL;

	return misc_register(&thisdev);
}

static void __exit secmem_exit(void)
{
	misc_deregister(&thisdev);
}

module_init(secmem_init);
module_exit(secmem_exit);
