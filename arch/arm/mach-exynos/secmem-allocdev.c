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
#include <linux/cma.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/pm_runtime.h>

#include <plat/devs.h>
#include <plat/pd.h>

#include <mach/secmem.h>

struct miscdevice secmem;

static char *secmem_info[] = {
	"mfc",		/* 0 */
	"fimc",		/* 1 */
	"mfc-shm",	/* 2 */
	"sectbl",	/* 3 */
	NULL
};

static bool drm_onoff = false;

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

			if (cma_info(&info, secmem.this_device,
					secmem_info[minfo.index]))
				return -EINVAL;

			minfo.base = info.lower_bound;
			minfo.size = info.total_size;
		}

		if (copy_to_user((void __user *)arg, &minfo, sizeof(minfo)))
			return -EFAULT;
	}
		break;
	case SECMEM_IOC_GET_DRM_ONOFF:
		if (copy_to_user((void __user *)arg, &drm_onoff, sizeof(int)))
			return -EFAULT;
		break;
	case SECMEM_IOC_SET_DRM_ONOFF:
	{
		int val = 0;
		if (copy_from_user(&val, (int __user *)arg, sizeof(int)))
			return -EFAULT;

		if (val) {
			if (drm_onoff == false) {
				drm_onoff = true;
				pm_runtime_forbid((*(secmem.this_device)).parent);
				exynos_pd_enable(&exynos4_device_pd[PD_MFC].dev);
			} else
				printk(KERN_ERR "%s: DRM is already on\n", __func__);
		} else {
			if (drm_onoff == true) {
				drm_onoff = false;
				pm_runtime_allow((*(secmem.this_device)).parent);
			} else
				printk(KERN_ERR "%s: DRM is already off\n", __func__);
		}
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
	int ret;
	secmem.minor = MISC_DYNAMIC_MINOR;
	secmem.name = "s5p-smem";
	secmem.fops = &secmem_fops;

	ret = misc_register(&secmem);
	if (ret)
		return ret;

	pm_runtime_enable(secmem.this_device);

	return 0;
}

static void __exit secmem_exit(void)
{
	__pm_runtime_disable(secmem.this_device, false);
	misc_deregister(&secmem);
}

module_init(secmem_init);
module_exit(secmem_exit);
