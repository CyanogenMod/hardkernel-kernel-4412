/*
 * Samsung Exynos4 SoC series FIMC-IS slave interface driver
 *
 * main platform driver interface
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
 * Contact: Younghwan Joo, <yhwan.joo@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/memory.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>

#include <linux/videodev2.h>
#include <linux/videodev2_samsung.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>

#ifdef CONFIG_CMA
#include <linux/cma.h>
#endif
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>

#include "fimc-is-core.h"
#include "fimc-is-regs.h"
#include "fimc-is-param.h"
#include "fimc-is-cmd.h"

struct fimc_is_dev *to_fimc_is_dev(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct fimc_is_dev, sd);
}

void fimc_is_mem_cache_clean(const void *start_addr, unsigned long size)
{
	unsigned long paddr;

	dmac_map_area(start_addr, size, DMA_TO_DEVICE);
	/*
	 * virtual & phsical addrees mapped directly, so we can convert
	 * the address just using offset
	 */
	paddr = __pa((unsigned long)start_addr);
	outer_clean_range(paddr, paddr + size);
}

void fimc_is_mem_cache_inv(const void *start_addr, unsigned long size)
{
	unsigned long paddr;
	paddr = __pa((unsigned long)start_addr);
	outer_inv_range(paddr, paddr + size);
	dmac_unmap_area(start_addr, size, DMA_FROM_DEVICE);
}

int fimc_is_init_mem_mgr(struct fimc_is_dev *dev)
{
#ifdef CONFIG_CMA
	struct cma_info mem_info;
	int err;

	sprintf(dev->cma_name, "%s%d", "fimc_is", 0);
	err = cma_info(&mem_info, &dev->pdev->dev, 0);
	printk(KERN_INFO "%s : [cma_info] start_addr : 0x%x, end_addr : 0x%x, "
			"total_size : 0x%x, free_size : 0x%x\n",
			__func__, mem_info.lower_bound, mem_info.upper_bound,
			mem_info.total_size, mem_info.free_size);
	if (err) {
		dev_err(&dev->pdev->dev, "%s: get cma info failed\n", __func__);
		return -EINVAL;
	}
	dev->mem.size = mem_info.total_size;
	dev->mem.base = (dma_addr_t)cma_alloc
		(&dev->pdev->dev, dev->cma_name, (size_t)dev->mem.size, 0);
	dev->is_p_region =
		(struct is_region *)(phys_to_virt(dev->mem.base +
				FIMC_IS_A5_MEM_SIZE - FIMC_IS_REGION_SIZE));
	memset((void *)dev->is_p_region, 0,
		(unsigned long)sizeof(struct is_region));
	fimc_is_mem_cache_clean((void *)dev->is_p_region, IS_PARAM_SIZE);

	printk(KERN_INFO "ctrl->mem.size = 0x%x\n", dev->mem.size);
	printk(KERN_INFO "ctrl->mem.base = 0x%x\n", dev->mem.base);
#endif
	return 0;
}

static irqreturn_t fimc_is_irq_handler1(int irq, void *dev_id)
{
	struct fimc_is_dev *dev = dev_id;

	/* Read ISSR10 ~ ISSR15 */
	dev->i2h_cmd.cmd = readl(dev->regs + ISSR10);

	switch (dev->i2h_cmd.cmd) {
	case IHC_GET_SENSOR_NUMBER:
		fimc_is_hw_set_sensor_num(dev);
		break;
	case IHC_LOAD_SET_FILE:
		fimc_is_hw_get_param(dev, 2);
	case IHC_SET_SHOT_MARK:
		fimc_is_hw_get_param(dev, 3);
		break;
	case IHC_SET_FACE_MARK:
		fimc_is_hw_get_param(dev, 2);
		break;
	case IHC_FRAME_DONE:
		fimc_is_hw_get_param(dev, 2);
		break;
	case IHC_NOT_READY:
		break;
	case ISR_DONE:
		fimc_is_hw_get_param(dev, 3);
		break;
	case ISR_NDONE:
		fimc_is_hw_get_param(dev, 4);
		break;
	}
	/* Just clear the interrupt pending bits. */
	fimc_is_fw_clear_irq1(dev);

	switch (dev->i2h_cmd.cmd) {
	case IHC_GET_SENSOR_NUMBER:
		printk(KERN_INFO "IHC_GET_SENSOR_NUMBER\n");
		set_bit(IS_ST_FW_DOWNLOADED, &dev->state);
		fimc_is_hw_wait_intsr0_intsd0(dev);
		fimc_is_hw_set_intgr0_gd0(dev);
		break;
	case IHC_LOAD_SET_FILE:
		printk(KERN_INFO "IHC_GET_SENSOR_NUMBER\n");
		printk(KERN_INFO "Param1 = 0x%08x\n", dev->i2h_cmd.arg[0]);
		printk(KERN_INFO "Param2 = 0x%08x\n", dev->i2h_cmd.arg[1]);
		dev->setfile.base = dev->i2h_cmd.arg[0];
		dev->setfile.size = dev->i2h_cmd.arg[1];
		set_bit(IS_ST_SET_FILE, &dev->state);
		break;
	case IHC_SET_SHOT_MARK:
		break;
	case IHC_SET_FACE_MARK:
		printk(KERN_INFO "Count = %d\n", dev->i2h_cmd.arg[0]);
		printk(KERN_INFO "FN = %d\n",
		dev->is_p_region->face[dev->i2h_cmd.arg[0]].frame_number);
		printk(KERN_INFO "Confidence = %d\n",
		dev->is_p_region->face[dev->i2h_cmd.arg[0]].confidence);
		printk(KERN_INFO "Size = %d,%d\n",
		dev->is_p_region->face[dev->i2h_cmd.arg[0]].width,
		dev->is_p_region->face[dev->i2h_cmd.arg[0]].height);
		printk(KERN_INFO "Offset = %d,%d\n",
		dev->is_p_region->face[dev->i2h_cmd.arg[0]].offset_x,
		dev->is_p_region->face[dev->i2h_cmd.arg[0]].offset_y);
		break;
	case IHC_FRAME_DONE:
		break;
	case IHC_LOCK_DONE:
		break;
	case ISR_DONE:
		printk(KERN_INFO "ISR_DONE - %d\n", dev->i2h_cmd.arg[0]);
		switch (dev->i2h_cmd.arg[0]) {
		case HIC_PREVIEW_STILL:
		case HIC_PREVIEW_VIDEO:
		case HIC_CAPTURE_STILL:
		case HIC_CAPTURE_VIDEO:
			if (test_and_clear_bit(IS_ST_CHANGE_MODE,
				&dev->state)) {
				dev->sensor.offset_x = dev->i2h_cmd.arg[1];
				dev->sensor.offset_y = dev->i2h_cmd.arg[2];
				set_bit(IS_ST_RUN, &dev->state);
			}
			break;
		case HIC_STREAM_ON:
			clear_bit(IS_ST_RUN, &dev->state);
			set_bit(IS_ST_STREAM_ON, &dev->state);
			break;
		case HIC_STREAM_OFF:
			set_bit(IS_ST_STREAM_OFF, &dev->state);
			set_bit(IS_ST_RUN, &dev->state);
			break;
		case HIC_SET_PARAMETER:
			dev->p_region_index1 = 0;
			dev->p_region_index2 = 0;
			atomic_set(&dev->p_region_num, 0);
			if (test_bit(IS_ST_INIT_CAPTURE_VIDEO, &dev->state))
				set_bit(IS_ST_RUN, &dev->state);
			else if (test_bit(IS_ST_INIT_CAPTURE_STILL,
				&dev->state))
				set_bit(IS_ST_INIT_CAPTURE_VIDEO, &dev->state);
			else if (test_bit(IS_ST_INIT_PREVIEW_VIDEO,
				&dev->state))
				set_bit(IS_ST_INIT_CAPTURE_STILL, &dev->state);
			else if (test_bit(IS_ST_INIT_PREVIEW_STILL,
				&dev->state))
				set_bit(IS_ST_INIT_PREVIEW_VIDEO, &dev->state);
			else {
				clear_bit(IS_ST_SET_PARAM, &dev->state);
				set_bit(IS_ST_RUN, &dev->state);
			}
			break;
		case HIC_GET_PARAMETER:
			break;
		case HIC_SET_TUNE:
			break;
		case HIC_GET_STATE:
			break;
		case HIC_OPEN_SENSOR:
			set_bit(IS_ST_INIT_PREVIEW_STILL, &dev->state);
			break;
		case HIC_CLOSE_SENSOR:
			set_bit(IS_ST_INIT_CAPTURE_VIDEO, &dev->state);
			dev->sensor.id = 0;
			break;
		case HIC_POWER_DOWN:
			set_bit(FIMC_IS_PWR_ST_POWEROFF, &dev->power);
			break;
		}
		break;
	case ISR_NDONE:
		printk(KERN_ERR "ISR_NDONE - %d: %d\n",
			dev->i2h_cmd.arg[0], dev->i2h_cmd.arg[1]);
		switch (dev->i2h_cmd.arg[1]) {
		case HIC_SET_PARAMETER:
			printk(KERN_ERR "SET_PARAMETER ERR : %d\n",
				dev->i2h_cmd.arg[2]);
			printk(KERN_ERR "SET_PARAMETER ERR : %d\n",
				dev->i2h_cmd.arg[3]);
			break;
		}
		break;
	}
	wake_up(&dev->irq_queue1);
	return IRQ_HANDLED;
}

static int fimc_is_probe(struct platform_device *pdev)
{
	struct exynos4_platform_fimc_is *pdata;
	struct resource *mem_res;
	struct resource *regs_res;
	struct fimc_is_dev *dev;
	int ret = -ENODEV;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Not enough memory for FIMC-IS device.\n");
		return -ENOMEM;
	}
	mutex_init(&dev->lock);
	spin_lock_init(&dev->slock);
	init_waitqueue_head(&dev->irq_queue1);
	dev->pdev = pdev;
	if (!dev->pdev) {
		dev_err(&pdev->dev, "No platform data specified\n");
		goto p_err1;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "Platform data not set\n");
		goto p_err1;
	}
	dev->pdata = pdata;
	/*
	 * I/O remap
	*/
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		dev_err(&pdev->dev, "Failed to get io memory region\n");
		goto p_err1;
	}

	regs_res = request_mem_region(mem_res->start,
		resource_size(mem_res), pdev->name);
	if (!regs_res) {
		dev_err(&pdev->dev, "Failed to request io memory region\n");
		goto p_err1;
	}
	dev->regs_res = regs_res;

	dev->regs = ioremap(mem_res->start, resource_size(mem_res));
	if (!dev->regs) {
		dev_err(&pdev->dev, "Failed to remap io region\n");
		goto p_err2;
	}

	/*
	 * initialize IRQ , FIMC-IS IRQ : ISP[0] -> SPI[90] , ISP[1] -> SPI[95]
	*/
	dev->irq1 = platform_get_irq(pdev, 0);
	if (dev->irq1 < 0) {
		ret = dev->irq1;
		dev_err(&pdev->dev, "Failed to get irq\n");
		goto p_err2;
	}

	ret = request_irq(dev->irq1, fimc_is_irq_handler1,
		IRQF_DISABLED, dev_name(&pdev->dev), dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to allocate irq (%d)\n", ret);
		goto e_irqfree1;
	}

	/*
	 * initialize memory manager
	*/
	ret = fimc_is_init_mem_mgr(dev);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to fimc_is_init_mem_mgr (%d)\n", ret);
		goto e_irqfree1;
	}

	/* Init v4l2 sub device */
	v4l2_subdev_init(&dev->sd, &fimc_is_subdev_ops);
	dev->sd.owner = THIS_MODULE;
	strcpy(dev->sd.name, MODULE_NAME);
	v4l2_set_subdevdata(&dev->sd, pdev);

	platform_set_drvdata(pdev, &dev->sd);

	pm_runtime_enable(&pdev->dev);
	set_bit(FIMC_IS_PWR_ST_SUSPENDED, &dev->power);

	dev->sensor_num = FIMC_IS_SENSOR_NUM;
	dev->sensor.id = 0;
	dev->p_region_index1 = 0;
	dev->p_region_index2 = 0;
	atomic_set(&dev->p_region_num, 0);

	set_bit(IS_ST_IDLE, &dev->state);
	dbg("FIMC-IS probe completed\n");
	return 0;

e_irqfree1:
	free_irq(dev->irq1, dev);
p_err2:
	release_mem_region(regs_res->start, resource_size(regs_res));
p_err1:
	kfree(dev);
	dev_err(&dev->pdev->dev, "failed to install\n");
	return ret;
}

static int fimc_is_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);
	kfree(dev);
	return 0;
}

static int fimc_is_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct fimc_is_dev *is_dev = to_fimc_is_dev(sd);

	mutex_lock(&is_dev->lock);
	set_bit(FIMC_IS_PWR_ST_SUSPENDED, &is_dev->power);
	mutex_unlock(&is_dev->lock);
	return 0;
}

static int fimc_is_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct fimc_is_dev *is_dev = to_fimc_is_dev(sd);

	mutex_lock(&is_dev->lock);
	clear_bit(FIMC_IS_PWR_ST_SUSPENDED, &is_dev->power);
	mutex_unlock(&is_dev->lock);
	return 0;
}

static int fimc_is_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct fimc_is_dev *is_dev = to_fimc_is_dev(sd);

	dbg("fimc_is_runtime_suspend\n");
	mutex_lock(&is_dev->lock);
	set_bit(FIMC_IS_PWR_ST_SUSPENDED, &is_dev->power);
	if (is_dev->pdata->clk_off) {
		is_dev->pdata->clk_off(pdev);
	} else {
		printk(KERN_ERR "#### failed to Clock OFF ####\n");
		return -EINVAL;
	}
	mutex_unlock(&is_dev->lock);
	return 0;
}

static int fimc_is_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct fimc_is_dev *is_dev = to_fimc_is_dev(sd);

	dbg("fimc_is_runtime_resume\n");
	mutex_lock(&is_dev->lock);
	clear_bit(FIMC_IS_PWR_ST_SUSPENDED, &is_dev->power);
	if (is_dev->pdata->clk_cfg) {
		is_dev->pdata->clk_cfg(pdev);
	} else {
		printk(KERN_ERR "#### failed to Clock CONFIG ####\n");
		return -EINVAL;
	}
	if (is_dev->pdata->clk_on) {
		is_dev->pdata->clk_on(pdev);
	} else {
		printk(KERN_ERR "#### failed to Clock On ####\n");
		return -EINVAL;
	}

	is_dev->frame_count = 0;
	mutex_unlock(&is_dev->lock);
	return 0;
}

static const struct dev_pm_ops fimc_is_pm_ops = {
	.suspend	 = fimc_is_suspend,
	.resume		 = fimc_is_resume,
	.runtime_suspend = fimc_is_runtime_suspend,
	.runtime_resume	 = fimc_is_runtime_resume,
};

static struct platform_driver fimc_is_driver = {
	.probe		= fimc_is_probe,
	.remove		= fimc_is_remove,
	.driver		= {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &fimc_is_pm_ops,
	},
};

static int __init fimc_is_init(void)
{
	int ret;
	ret = platform_driver_register(&fimc_is_driver);
	if (ret)
		err("platform_driver_register failed: %d\n", ret);
	return ret;
}

static void __exit fimc_is_exit(void)
{
	platform_driver_unregister(&fimc_is_driver);
}

module_init(fimc_is_init);
module_exit(fimc_is_exit);

MODULE_AUTHOR("Younghwan Joo, <yhwan.joo@samsung.com>");
MODULE_DESCRIPTION("Exynos4 series FIMC-IS slave driver");
MODULE_LICENSE("GPL");
