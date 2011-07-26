/*
 * Register interface file for Samsung Camera Interface (FIMC-Lite) driver
 *
 * Copyright (c) 2011 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include "fimc-lite-core.h"

#define MODULE_NAME		"exynos-fimc-lite"
static struct flite_fmt flite_formats[] = {
	{
		.name		= "YUV422 8-bit 1 plane(UYVY)",
		.code		= V4L2_MBUS_FMT_UYVY8_2X8,
		.fmt_reg	= FLITE_REG_CIGCTRL_YUV422_1P,
	},{
		.name		= "YUV422 8-bit 1 plane(VYUY)",
		.code		= V4L2_MBUS_FMT_VYUY8_2X8,
		.fmt_reg	= FLITE_REG_CIGCTRL_YUV422_1P,
	},{
		.name		= "YUV422 8-bit 1 plane(YUYV)",
		.code		= V4L2_MBUS_FMT_YUYV8_2X8,
		.fmt_reg	= FLITE_REG_CIGCTRL_YUV422_1P,
	},{
		.name		= "YUV422 8-bit 1 plane(YVYU)",
		.code		= V4L2_MBUS_FMT_YVYU8_2X8,
		.fmt_reg	= FLITE_REG_CIGCTRL_YUV422_1P,
	},{
		/* ISP supports only GRBG order in 4212 */
		.name		= "RAW8(GRBG)",
		.code		= V4L2_MBUS_FMT_SGRBG8_1X8,
		.fmt_reg	= FLITE_REG_CIGCTRL_RAW8,
	},{
		/* ISP supports only GRBG order in 4212 */
		.name		= "RAW10(GRBG)",
		.code		= V4L2_MBUS_FMT_SGRBG10_1X10,
		.fmt_reg	= FLITE_REG_CIGCTRL_RAW10,
	},{
		/* ISP supports only GRBG order in 4212 */
		.name		= "RAW12(GRBG)",
		.code		= V4L2_MBUS_FMT_SGRBG12_1X12,
		.fmt_reg	= FLITE_REG_CIGCTRL_RAW12,
	},{
		.name		= "User Defined(JPEG)",
		.code		= V4L2_MBUS_FMT_JPEG_1X8,
		.fmt_reg	= FLITE_REG_CIGCTRL_USER(1),
	},
};

static struct flite_dev *to_flite_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct flite_dev, sd);
}

inline struct flite_fmt const *find_flite_format(struct
		v4l2_mbus_framefmt *mf)
{
	int num_fmt = ARRAY_SIZE(flite_formats);

	while (num_fmt--)
		if (mf->code == flite_formats[num_fmt].code)
			break;
	if (num_fmt < 0)
		return NULL;

	return &flite_formats[num_fmt];
}

static int flite_s_mbus_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct flite_dev *flite = to_flite_dev(sd);
	struct flite_fmt const *f_fmt = find_flite_format(mf);
	struct flite_frame *f_frame = &flite->source_frame;

	v4l2_dbg(1, debug, sd, "%s: w: %d, h: %d\n", __func__,
		 mf->width, mf->height);

	if (unlikely(!f_fmt)) {
		v4l2_err(sd, "f_fmt is null\n");
		return -EINVAL;
	}

	flite->mbus_fmt = *mf;

	/* These are the datas from fimc
	 * If you want to crop the image, you can use s_crop
	 */
	f_frame->o_width = mf->width;
	f_frame->o_height = mf->height;
	f_frame->width = mf->width;
	f_frame->height = mf->height;
	f_frame->offs_h = 0;
	f_frame->offs_v = 0;

	return 0;
}

static int flite_g_mbus_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct flite_dev *flite = to_flite_dev(sd);

	mf = &flite->mbus_fmt;

	return 0;
}

static int flite_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *cc)
{
	struct flite_dev *flite = to_flite_dev(sd);
	struct flite_frame *f;

	f = &flite->source_frame;

	cc->bounds.left		= 0;
	cc->bounds.top		= 0;
	cc->bounds.width	= f->o_width;
	cc->bounds.height	= f->o_height;
	cc->defrect		= cc->bounds;

	return 0;
}

static int flite_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	struct flite_dev *flite = to_flite_dev(sd);
	struct flite_frame *f;

	f = &flite->source_frame;

	crop->c.left	= f->offs_h;
	crop->c.top	= f->offs_v;
	crop->c.width	= f->width;
	crop->c.height	= f->height;

	return 0;
}

static int flite_s_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	struct flite_dev *flite = to_flite_dev(sd);
	struct flite_frame *f;

	f = &flite->source_frame;

	if (crop->c.left + crop->c.width > f->o_width)
		return -EINVAL;
	if (crop->c.top + crop->c.height > f->o_height)
		return -EINVAL;

	f->width = crop->c.width;
	f->height = crop->c.height;
	f->offs_h = crop->c.left;
	f->offs_v = crop->c.top;

	v4l2_dbg(1, debug, sd,
		"%s : width : %d, height : %d, offs_h : %d, off_v : %d\n",
		__func__, f->width, f->height, f->offs_h, f->offs_v);

	return 0;
}

static int flite_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct flite_dev *flite = to_flite_dev(sd);
	struct s3c_platform_camera *cam = flite->pdata->cam;
	int ret = 0;
	u32 int_src = FLITE_REG_CIGCTRL_IRQ_STARTEN0_ENABLE |
		FLITE_REG_CIGCTRL_IRQ_LASTEN0_ENABLE |
		FLITE_REG_CIGCTRL_IRQ_ENDEN0_DISABLE;
	unsigned long flags;

	spin_lock_irqsave(&flite->slock, flags);

	if (test_bit(FLITE_ST_SUSPENDED, &flite->state))
		goto s_stream_unlock;
	if (enable) {
		ret = flite_hw_set_cam_source_size(flite);
		if (unlikely(ret < 0))
			goto s_stream_unlock;
		flite_hw_set_camera_type(flite, cam);
		ret = flite_hw_set_source_format(flite);
		if (unlikely(ret < 0))
			goto s_stream_unlock;
		if (cam->use_isp)
			flite_hw_set_output_dma(flite, false);

		flite_hw_set_interrupt_source(flite, int_src);
		flite_hw_set_config_irq(flite, cam);
		flite_hw_set_window_offset(flite);
		flite_hw_set_capture_start(flite);

		set_bit(FLITE_ST_STREAMING, &flite->state);
	} else {
		if (test_and_clear_bit(FLITE_ST_STREAMING, &flite->state)) {
			flite_hw_set_capture_stop(flite);
			ret = wait_event_timeout(flite->irq_queue,
			test_bit(FLITE_ST_STREAMING, &flite->state), HZ/20); /* 50 ms */
			if (unlikely(!ret)) {
				v4l2_err(sd, "wait timeout\n");
				ret = -EBUSY;
			}
		}
	}
s_stream_unlock:
	spin_unlock_irqrestore(&flite->slock, flags);
	return ret;
}

static irqreturn_t flite_irq_handler(int irq, void *priv)
{
	struct flite_dev *flite = priv;
	u32 int_src = 0;

	flite_hw_get_int_src(flite, &int_src);

	spin_lock(&flite->slock);

	switch (int_src & FLITE_REG_CISTATUS_IRQ_MASK) {
	case FLITE_REG_CISTATUS_IRQ_SRC_OVERFLOW:
		v4l2_dbg(1, debug, &flite->sd, "overflow generated\n");
		break;
	case FLITE_REG_CISTATUS_IRQ_SRC_LASTCAPEND:
		v4l2_dbg(1, debug, &flite->sd, "last capture end\n");
		clear_bit(FLITE_ST_STREAMING, &flite->state);
		wake_up(&flite->irq_queue);
		break;
	case FLITE_REG_CISTATUS_IRQ_SRC_FRMSTART:
		v4l2_dbg(1, debug, &flite->sd, "frame start\n");
		break;
	case FLITE_REG_CISTATUS_IRQ_SRC_FRMEND:
		v4l2_dbg(1, debug, &flite->sd, "frame end\n");
		break;
	}
	flite_hw_clear_irq(flite);

	spin_unlock(&flite->slock);

	return IRQ_HANDLED;
}

static int flite_s_power(struct v4l2_subdev *sd, int on)
{
	struct flite_dev *flite = to_flite_dev(sd);
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&flite->slock, flags);

	if (on) {
		set_bit(FLITE_ST_POWERED, &flite->state);
		ret = pm_runtime_get_sync(&flite->pdev->dev);
	} else {
		ret = pm_runtime_put_sync(&flite->pdev->dev);
	}

	if ((!ret && !on) || (ret && on))
		clear_bit(FLITE_ST_POWERED, &flite->state);

	spin_unlock_irqrestore(&flite->slock, flags);

	return ret;
}

static struct v4l2_subdev_core_ops flite_core_ops = {
	.s_power = flite_s_power,
};

static struct v4l2_subdev_video_ops flite_video_ops = {
	.g_mbus_fmt	= flite_g_mbus_fmt,
	.s_mbus_fmt	= flite_s_mbus_fmt,
	.s_stream	= flite_s_stream,
	.cropcap	= flite_cropcap,
	.g_crop		= flite_g_crop,
	.s_crop		= flite_s_crop,
};

static struct v4l2_subdev_ops flite_subdev_ops = {
	.core	= &flite_core_ops,
	.video	= &flite_video_ops,
};

static int flite_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct flite_dev *flite = to_flite_dev(sd);
	unsigned long flags;

	spin_lock_irqsave(&flite->slock, flags);

	if (test_bit(FLITE_ST_STREAMING, &flite->state))
		flite_s_stream(sd, false);
	if (test_bit(FLITE_ST_POWERED, &flite->state))
		flite_s_power(sd, false);

	set_bit(FLITE_ST_SUSPENDED, &flite->state);

	spin_unlock_irqrestore(&flite->slock, flags);

	return 0;
}

static int flite_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct flite_dev *flite = to_flite_dev(sd);
	unsigned long flags;

	spin_lock_irqsave(&flite->slock, flags);

	if (test_bit(FLITE_ST_POWERED, &flite->state))
		flite_s_power(sd, true);
	if (test_bit(FLITE_ST_STREAMING, &flite->state))
		flite_s_stream(sd, true);

	clear_bit(FLITE_ST_SUSPENDED, &flite->state);

	spin_unlock_irqrestore(&flite->slock, flags);

	return 0;
}

static int flite_probe(struct platform_device *pdev)
{
	struct resource *mem_res;
	struct resource *regs_res;
	struct flite_dev *flite;
	int ret = -ENODEV;

	flite = kzalloc(sizeof(struct flite_dev), GFP_KERNEL);
	if (!flite)
		return -ENOMEM;

	flite->pdev = pdev;
	flite->pdata = pdev->dev.platform_data;

	init_waitqueue_head(&flite->irq_queue);
	spin_lock_init(&flite->slock);

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		dev_err(&pdev->dev, "Failed to get io memory region\n");
		goto p_err1;
	}

	regs_res = request_mem_region(mem_res->start, resource_size(mem_res),
				      pdev->name);
	if (!regs_res) {
		dev_err(&pdev->dev, "Failed to request io memory region\n");
		goto p_err1;
	}

	flite->regs_res = regs_res;
	flite->regs = ioremap(mem_res->start, resource_size(mem_res));
	if (!flite->regs) {
		dev_err(&pdev->dev, "Failed to remap io region\n");
		goto p_err2;
	}

	flite->irq = platform_get_irq(pdev, 0);
	if (flite->irq < 0) {
		dev_err(&pdev->dev, "Failed to get irq\n");
		goto p_err3;
	}

	ret = request_irq(flite->irq, flite_irq_handler, 0, dev_name(&pdev->dev), flite);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed\n");
		goto p_err3;
	}

	v4l2_subdev_init(&flite->sd, &flite_subdev_ops);
	flite->sd.owner = THIS_MODULE;
	strcpy(flite->sd.name, MODULE_NAME);

	/* This allows to retrieve the platform device id by the host driver */
	v4l2_set_subdevdata(&flite->sd, pdev);

	/* .. and a pointer to the subdev. */
	platform_set_drvdata(pdev, &flite->sd);

	pm_runtime_enable(&pdev->dev);

	set_bit(FLITE_ST_SUSPENDED, &flite->state);

	return 0;

p_err3:
	iounmap(flite->regs);
p_err2:
	release_mem_region(regs_res->start, resource_size(regs_res));
p_err1:
	kfree(flite);
	return ret;
}

static int flite_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct flite_dev *flite = to_flite_dev(sd);
	struct resource *res = flite->regs_res;

	flite_s_power(&flite->sd, 0);
	pm_runtime_disable(&pdev->dev);

	free_irq(flite->irq, flite);
	iounmap(flite->regs);
	release_mem_region(res->start, resource_size(res));
	kfree(flite);

	return 0;
}

static const struct dev_pm_ops flite_pm_ops = {
	.suspend		= flite_suspend,
	.resume			= flite_resume,
};

static struct platform_driver flite_driver = {
	.probe		= flite_probe,
	.remove	= __devexit_p(flite_remove),
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &flite_pm_ops,
	}
};

static int __init flite_init(void)
{
	int ret = platform_driver_register(&flite_driver);
	if (ret)
		err("platform_driver_register failed: %d\n", ret);
	return ret;
}

static void __exit flite_exit(void)
{
	platform_driver_unregister(&flite_driver);
}
module_init(flite_init);
module_exit(flite_exit);

MODULE_AUTHOR("Sky Kang<sungchun.kang@samsung.com>");
MODULE_DESCRIPTION("Exynos FIMC-Lite driver");
MODULE_LICENSE("GPL");
