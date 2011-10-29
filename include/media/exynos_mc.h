/* linux/inclue/media/exynos_mc.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * header file for exynos media device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef GSC_MDEVICE_H_
#define GSC_MDEVICE_H_

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define err(fmt, args...) \
	printk(KERN_ERR "%s:%d: " fmt "\n", __func__, __LINE__, ##args)

#define MDEV_MODULE_NAME "exynos5-mdev"
#define MAX_GSC_SUBDEV		4

#define GSC_OUT_PAD_SINK	0
#define GSC_OUT_PAD_SOURCE	1

enum mdev_node {
	MDEV_OUTPUT,
	MDEV_CAPTURE,
};

/**
 * struct exynos_md - Exynos media device information
 * @media_dev: top level media device
 * @v4l2_dev: top level v4l2_device holding up the subdevs
 * @pdev: platform device this media device is hooked up into
 * @slock: spinlock protecting @sensor array
 * @id: media device IDs
 * @gsc_sd: each pointer of g-scaler's subdev array
 */
struct exynos_md {
	struct media_device	media_dev;
	struct v4l2_device	v4l2_dev;
	struct platform_device	*pdev;
	struct v4l2_subdev	*gsc_sd[MAX_GSC_SUBDEV];
	u16			id;
	spinlock_t slock;
};

static int dummy_callback(struct device *dev, void *md)
{
	/* non-zero return stops iteration */
	return -1;
}

static inline void *module_name_to_driver_data(char *module_name)
{
	struct device_driver *drv;
	struct device *dev;
	void *driver_data;

	drv = driver_find(module_name, &platform_bus_type);
	if (drv) {
		dev = driver_find_device(drv, NULL, NULL, dummy_callback);
		driver_data = dev_get_drvdata(dev);
		put_driver(drv);
		return driver_data;
	} else
		return NULL;
}

#endif
