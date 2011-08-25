/*
 * Samsung Exynos4 SoC series FIMC-IS slave interface driver
 *
 * v4l2 subdev driver interface
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
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/memory.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>

#include <linux/videodev2.h>
#include <linux/videodev2_samsung.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-mediabus.h>

#include "fimc-is-core.h"
#include "fimc-is-regs.h"
#include "fimc-is-param.h"
#include "fimc-is-cmd.h"

/* v4l2 subdev core operations
*/
static int fimc_is_s_fimc_lite(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);
	fimc_is_hw_set_lite(dev,
		dev->is_p_region->parameter.isp.otf_input.width,
		dev->is_p_region->parameter.isp.otf_input.height);
	dbg("lite set : w= %d, h = %d\n",
		dev->is_p_region->parameter.isp.otf_input.width+
		dev->sensor.offset_x,
		dev->is_p_region->parameter.isp.otf_input.height+
		dev->sensor.offset_y);

	return ret;
}

static int fimc_is_load_fw(struct v4l2_subdev *sd)
{
	int ret;
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);
	dbg("fimc_is_load_fw\n");
	if (test_bit(IS_ST_IDLE, &dev->state)) {
		set_bit(IS_ST_PWR_ON, &dev->state);
		fimc_is_hw_io_init(dev);
		fimc_is_hw_reset(dev);
		ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_FW_DOWNLOADED, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
		dbg("fimc_is_load_fw end\n");
	} else {
		dbg("IS FW was loaded before\n");
	}
	return 0;
}

static int fimc_is_reset(struct v4l2_subdev *sd, u32 val)
{
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);
	dbg("fimc_is_reset\n");
	if (val)
		dbg("hard reset start\n");
	else
		return -EINVAL;
	return 0;
}

static int fimc_is_init_set(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);
	fimc_is_hw_diable_wdt(dev);
	dbg("fimc_is_init\n");
	if (test_bit(IS_ST_FW_DOWNLOADED, &dev->state)) {
		dbg("v4l2 : open sensor : %d\n", val);
		fimc_is_hw_open_sensor(dev, val, ISS_PREVIEW_STILL);
		ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_INIT_PREVIEW_STILL, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout:%s\n", __func__);
			/* FIX ME */
			set_bit(IS_ST_INIT_PREVIEW_STILL, &dev->state);
			return -EBUSY;
		}
		/* Debug only */
		dbg("Header addr = 0x%x\n", dev->is_p_region->header);
		dbg("FN[0] addr = 0x%x\n",
			virt_to_phys(&dev->is_p_region->header[0]
							.frame_number));
		dbg("FN[1] addr = 0x%x\n",
			virt_to_phys(&dev->is_p_region->header[1]
							.frame_number));
		dbg("FN[2] addr = 0x%x\n",
			virt_to_phys(&dev->is_p_region->header[2]
							.frame_number));
		dbg("FN[3] addr = 0x%x\n",
			virt_to_phys(&dev->is_p_region->header[3]
							.frame_number));
		dev->frame_count = 0;
		/* 1. */
		dbg("Default setting : preview_still\n");
		dev->scenario_id = ISS_PREVIEW_STILL;
		fimc_is_hw_set_init(dev);
		fimc_is_mem_cache_clean((void *)dev->is_p_region,
			(unsigned long)sizeof(IS_PARAM));
		fimc_is_hw_set_param(dev);
		ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_INIT_PREVIEW_VIDEO, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
		/* 2. */
		dbg("Default setting : preview_video\n");
		dev->scenario_id = ISS_PREVIEW_VIDEO;
		fimc_is_hw_set_init(dev);
		fimc_is_mem_cache_clean((void *)dev->is_p_region,
			(unsigned long)sizeof(IS_PARAM));
		fimc_is_hw_set_param(dev);
		ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_INIT_CAPTURE_STILL, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
		/* 3. */
		dbg("Default setting : capture_still\n");
		dev->scenario_id = ISS_CAPTURE_STILL;
		fimc_is_hw_set_init(dev);
		fimc_is_mem_cache_clean((void *)dev->is_p_region,
			(unsigned long)sizeof(IS_PARAM));
		fimc_is_hw_set_param(dev);
		ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_INIT_CAPTURE_VIDEO, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
		/* 4. */
		dbg("Default setting : capture_video\n");
		dev->scenario_id = ISS_CAPTURE_VIDEO;
		fimc_is_hw_set_init(dev);
		fimc_is_mem_cache_clean((void *)dev->is_p_region,
			(unsigned long)sizeof(IS_PARAM));
		fimc_is_hw_set_param(dev);
		ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_RUN, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}

		dbg("Stream Off\n");
		clear_bit(IS_ST_RUN, &dev->state); /* FIX ME */
		set_bit(IS_ST_STREAM, &dev->state); /* FIX ME */
		fimc_is_hw_set_stream(dev, 0); /*stream off */
		ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_RUN, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
	}

	return 0;
}

static int fimc_is_s_power(struct v4l2_subdev *sd, int on)
{
	struct fimc_is_dev *is_dev = to_fimc_is_dev(sd);
	struct device *dev = &is_dev->pdev->dev;
	int ret = 0;

	dbg("fimc_is_s_power\n");
	if (on) {
		set_bit(FIMC_IS_PWR_ST_POWERED, &is_dev->power);
		ret = pm_runtime_get_sync(dev);
	} else {
		ret = pm_runtime_put_sync(dev);
	}

	if ((!ret && !on) || (ret && on))
		clear_bit(FIMC_IS_PWR_ST_POWERED, &is_dev->power);

	return ret;
}

static int fimc_is_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	int index;
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);

	dbg("fimc_is_g_ctrl\n");
	switch (ctrl->id) {
	case V4L2_CID_IS_CAMERA_EXIF_EXPTIME:
		ctrl->value = 0;
		break;
	case V4L2_CID_IS_CAMERA_EXIF_FLASH:
		ctrl->value = 0;
		break;
	case V4L2_CID_IS_CAMERA_EXIF_ISO:
		ctrl->value = 0;
		break;
	case V4L2_CID_IS_CAMERA_EXIF_SHUTTERSPEED:
		ctrl->value = 0;
		break;
	case V4L2_CID_IS_CAMERA_EXIF_BRIGHTNESS:
		ctrl->value = 0;
		break;
	case V4L2_CID_IS_GET_SENSOR_OFFSET_X:
		ctrl->value = dev->sensor.offset_x;
		break;
	case V4L2_CID_IS_GET_SENSOR_OFFSET_Y:
		ctrl->value = dev->sensor.offset_y;
		break;
	case V4L2_CID_IS_GET_FRAME_VALID:
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)sizeof(IS_HEADER));
		ctrl->value = dev->is_p_region->header
			[dev->frame_count%MAX_FRAME_COUNT_PREVIEW].valid;
		printk(KERN_INFO "Valid %d = %d\n",
			(dev->frame_count%MAX_FRAME_COUNT_PREVIEW),
							ctrl->value);
		dev->is_p_region->header
			[dev->frame_count%MAX_FRAME_COUNT_PREVIEW].valid = 0;
		fimc_is_mem_cache_clean((void *)IS_HEADER,
			(unsigned long)sizeof(IS_HEADER));
		dev->frame_count++;
		break;
	case V4L2_CID_IS_GET_FRAME_BADMARK:
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)sizeof(IS_HEADER));
		index = ctrl->value;
		ctrl->value = dev->is_p_region->header
			[index%MAX_FRAME_COUNT_PREVIEW].captured;
		dev->is_p_region->header
			[index%MAX_FRAME_COUNT_PREVIEW].valid = 0;
		dev->is_p_region->header
			[index%MAX_FRAME_COUNT_PREVIEW].captured = 0;
		fimc_is_mem_cache_clean((void *)IS_HEADER,
			(unsigned long)sizeof(IS_HEADER));
		break;
	case V4L2_CID_IS_GET_FRAME_NUMBER:
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)sizeof(IS_HEADER));
		ctrl->value =
			dev->is_p_region->header
			[dev->frame_count%MAX_FRAME_COUNT_PREVIEW].frame_number;
		printk(KERN_INFO "FN = %d\n", ctrl->value);
		break;
	default:
		return -EINVAL;
	}
	dbg("ctrl->value= %d\n", ctrl->value);
	return ret;
}

static int fimc_is_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);

	dbg("fimc_is_s_ctrl\n");
	switch (ctrl->id) {
	case V4L2_CID_IS_S_SCENARIO_MODE:
		if (test_bit(IS_ST_RUN, &dev->state)) {
			clear_bit(IS_ST_RUN, &dev->state);
			set_bit(IS_ST_CHANGE_MODE, &dev->state);
			fimc_is_hw_change_mode(dev, ctrl->value);
			ret = wait_event_timeout(dev->irq_queue1,
					test_bit(IS_ST_RUN, &dev->state),
					FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
						"wait timeout:%s\n", __func__);
					return -EBUSY;
				}
		}
		break;
	case V4L2_CID_IS_CAMERA_SHOT_MODE_NORMAL:
		IS_SET_PARAM_GLOBAL_SHOTMODE_CMD(dev, ctrl->value);
		IS_SET_PARAM_GLOBAL_SHOTMODE_SMILE(dev,
			GLOBAL_SHOTMODE_SMILE_DISABLE);
		IS_SET_PARAM_GLOBAL_SHOTMODE_EYEBLINK(dev,
			GLOBAL_SHOTMODE_SMILE_DISABLE);
		IS_SET_PARAM_BIT(dev, PARAM_GLOBAL_SHOTMODE);
		IS_INC_PARAM_NUM(dev);
		fimc_is_mem_cache_clean((void *)dev->is_p_region,
			(unsigned long)sizeof(IS_PARAM));
		fimc_is_hw_set_param(dev);
		break;
	case V4L2_CID_IS_CAMERA_SHOT_MODE_SMILE:
		IS_SET_PARAM_GLOBAL_SHOTMODE_CMD(dev, ctrl->value);
		IS_SET_PARAM_GLOBAL_SHOTMODE_SMILE(dev,
			GLOBAL_SHOTMODE_SMILE_ENABLE);
		IS_SET_PARAM_GLOBAL_SHOTMODE_EYEBLINK(dev,
			GLOBAL_SHOTMODE_EYEBLINK_DISABLE);
		IS_SET_PARAM_BIT(dev, PARAM_GLOBAL_SHOTMODE);
		IS_INC_PARAM_NUM(dev);
		fimc_is_mem_cache_clean((void *)dev->is_p_region,
			(unsigned long)sizeof(IS_PARAM));
		fimc_is_hw_set_param(dev);
		break;
	case V4L2_CID_IS_CAMERA_SHOT_MODE_EYEBLINK:
		IS_SET_PARAM_GLOBAL_SHOTMODE_CMD(dev, ctrl->value);
		IS_SET_PARAM_GLOBAL_SHOTMODE_SMILE(dev,
			GLOBAL_SHOTMODE_SMILE_DISABLE);
		IS_SET_PARAM_GLOBAL_SHOTMODE_EYEBLINK(dev,
			GLOBAL_SHOTMODE_EYEBLINK_ENABLE);
		IS_SET_PARAM_BIT(dev, PARAM_GLOBAL_SHOTMODE);
		IS_INC_PARAM_NUM(dev);
		fimc_is_mem_cache_clean((void *)dev->is_p_region,
			(unsigned long)sizeof(IS_PARAM));
		fimc_is_hw_set_param(dev);
		break;
	case V4L2_CID_IS_CAMERA_OBJECT_POSITION_X:
		IS_ISP_SET_PARAM_AF_WIN_POS_X(dev, ctrl->value);
		IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, ctrl->value);
		break;
	case V4L2_CID_IS_CAMERA_OBJECT_POSITION_Y:
		IS_ISP_SET_PARAM_AF_WIN_POS_Y(dev, ctrl->value);
		IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, ctrl->value);
		break;
	case V4L2_CID_IS_CAMERA_WINDOW_SIZE_X:
		IS_ISP_SET_PARAM_AF_WIN_WIDTH(dev, ctrl->value);
		IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, ctrl->value);
		break;
	case V4L2_CID_IS_CAMERA_WINDOW_SIZE_Y:
		IS_ISP_SET_PARAM_AF_WIN_HEIGHT(dev, ctrl->value);
		IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, ctrl->value);
		break;
	case V4L2_CID_IS_CAMERA_FOCUS_MODE:
		switch (ctrl->value) {
		case IS_FOCUS_MODE_AUTO:
			IS_ISP_SET_PARAM_AF_CMD(dev,
				ISP_AF_COMMAND_SET_FOCUSMODE);
			IS_ISP_SET_PARAM_AF_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AF_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AF_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AF_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_HEIGHT(dev, 0);
			break;
		case IS_FOCUS_MODE_MACRO:
			IS_ISP_SET_PARAM_AF_CMD(dev,
				ISP_AF_COMMAND_SET_FOCUSMODE);
			IS_ISP_SET_PARAM_AF_MODE(dev, ISP_AF_MODE_MACRO);
			IS_ISP_SET_PARAM_AF_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AF_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AF_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_HEIGHT(dev, 0);
			break;
		case IS_FOCUS_MODE_INFINITY:
			IS_ISP_SET_PARAM_AF_CMD(dev,
				ISP_AF_COMMAND_SET_FOCUSMODE);
			IS_ISP_SET_PARAM_AF_MODE(dev, ISP_AF_MODE_INFINITY);
			IS_ISP_SET_PARAM_AF_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AF_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AF_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_HEIGHT(dev, 0);
			break;
		case IS_FOCUS_MODE_CONTINUOUS:
			IS_ISP_SET_PARAM_AF_CMD(dev,
				ISP_AF_COMMAND_SET_FOCUSMODE);
			IS_ISP_SET_PARAM_AF_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AF_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AF_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_ENABLE);
			IS_ISP_SET_PARAM_AF_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_HEIGHT(dev, 0);
			break;
		case IS_FOCUS_MODE_TOUCH:
			IS_ISP_SET_PARAM_AF_CMD(dev, ISP_AF_COMMAND_TOUCH);
			IS_ISP_SET_PARAM_AF_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AF_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AF_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_ENABLE);
			break;
		case IS_FOCUS_MODE_HALFSHUTTER:
			IS_ISP_SET_PARAM_AF_CMD(dev,
				ISP_AF_COMMAND_HALFSHUTTER);
			IS_ISP_SET_PARAM_AF_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AF_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AF_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AF_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_HEIGHT(dev, 0);
			break;
		case IS_FOCUS_MODE_FACEDETECT:
			IS_ISP_SET_PARAM_AF_CMD(dev,
				ISP_AF_COMMAND_SET_FOCUSMODE);
			IS_ISP_SET_PARAM_AF_MODE(dev,
				ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AF_FACE(dev,
				ISP_AF_FACE_ENABLE);
			IS_ISP_SET_PARAM_AF_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AF_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_HEIGHT(dev, 0);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_FOCUS_MODE_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AF);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_FLASH_MODE:
		switch (ctrl->value) {
		case IS_FLASH_MODE_OFF:
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			break;
		case IS_FLASH_MODE_AUTO:
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_AUTO);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			break;
		case IS_FLASH_MODE_AUTO_REDEYE:
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_AUTO);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_ENABLE);
			break;
		case IS_FLASH_MODE_ON:
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_MANUALON);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			break;
		case IS_FLASH_MODE_TORCH:
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_TORCH);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_FLASH_MODE_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_AWB_MODE:
		switch (ctrl->value) {
		case IS_AWB_AUTO:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev, 0);
			break;
		case IS_AWB_DAYLIGHT:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			break;
		case IS_AWB_CLOUDY:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_CLOUDY);
			break;
		case IS_AWB_TUNGSTEN:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_TUNGSTEN);
			break;
		case IS_AWB_FLUORESCENT:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_FLUORESCENT);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_AWB_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_IMAGE_EFFECT:
		switch (ctrl->value) {
		case IS_IMAGE_EFFECT_DISABLE:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_DISABLE);
			break;
		case IS_IMAGE_EFFECT_BLACK_WHITE:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_BLACK_WHITE);
			break;
		case IS_IMAGE_EFFECT_ANTIQUE:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_ANTIQUE);
			break;
		case IS_IMAGE_EFFECT_GREEN:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_GREEN);
			break;
		case IS_IMAGE_EFFECT_BLUE:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_BLUE);
			break;
		}
		/* only ISP effect in Pegasus */
		if (ctrl->value >= 0 && ctrl->value < 5) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_IMAGE_EFFECT);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_ISO:
		switch (ctrl->value) {
		case IS_ISO_AUTO:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_AUTO);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			break;
		case IS_ISO_50:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 50);
			break;
		case IS_ISO_100:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 100);
			dev->is_p_region->parameter.isp.iso.cmd =
				ISP_ISO_COMMAND_MANUAL;
			dev->is_p_region->parameter.isp.iso.value = 100;
			break;
		case IS_ISO_200:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 200);
			break;
		case IS_ISO_400:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 400);
			break;
		case IS_ISO_800:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 800);
			break;
		case IS_ISO_1600:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 1600);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_ISO_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_CONTRAST:
		switch (ctrl->value) {
		case IS_CONTRAST_AUTO:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_AUTOCONTRAST);
			break;
		case IS_CONTRAST_MINUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, -2);
			break;
		case IS_CONTRAST_MINUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, -1);
			break;
		case IS_CONTRAST_DEFAULT:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 0);
			break;
		case IS_CONTRAST_PLUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 1);
			break;
		case IS_CONTRAST_PLUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 2);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_CONTRAST_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_SATURATION:
		switch (ctrl->value) {
		case IS_SATURATION_MINUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, -2);
			break;
		case IS_SATURATION_MINUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, -1);
			break;
		case IS_SATURATION_DEFAULT:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, 0);
			break;
		case IS_SATURATION_PLUS1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, 1);
			break;
		case IS_SATURATION_PLUS2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, 2);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_SATURATION_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_SHARPNESS:
		switch (ctrl->value) {
		case IS_SHARPNESS_MINUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, -2);
			break;
		case IS_SHARPNESS_MINUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, -1);
			break;
		case IS_SHARPNESS_DEFAULT:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, 0);
			break;
		case IS_SHARPNESS_PLUS1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, 1);
			break;
		case IS_SHARPNESS_PLUS2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, 2);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_SHARPNESS_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_EXPOSURE:
		switch (ctrl->value) {
		case IS_EXPOSURE_MINUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_EXPOSURE(dev, -2);
			break;
		case IS_EXPOSURE_MINUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_EXPOSURE(dev, -1);
			break;
		case IS_EXPOSURE_DEFAULT:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_EXPOSURE(dev, 0);
			break;
		case IS_EXPOSURE_PLUS1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_EXPOSURE(dev, 1);
			break;
		case IS_EXPOSURE_PLUS2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_EXPOSURE(dev, 2);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_EXPOSURE_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_BRIGHTNESS:
		if (ctrl->value >= 0) {
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_HUE:
		if (ctrl->value >= 0) {
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_HUE(dev, ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_METERING:
		switch (ctrl->value) {
		case IS_METERING_CENTER:
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_CENTER);
			break;
		case IS_METERING_SPOT:
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_SPOT);
			break;
		case IS_METERING_MATRIX:
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_MATRIX);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_METERING_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_AFC_MODE:
		switch (ctrl->value) {
		case IS_AFC_DISABLE:
			IS_ISP_SET_PARAM_AFC_CMD(dev, ISP_AFC_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_AFC_MANUAL(dev, 0);
			break;
		case IS_AFC_AUTO:
			IS_ISP_SET_PARAM_AFC_CMD(dev, ISP_AFC_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AFC_MANUAL(dev, 0);
			break;
		case IS_AFC_MANUAL_50HZ:
			IS_ISP_SET_PARAM_AFC_CMD(dev, ISP_AFC_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_AFC_MANUAL(dev, ISP_AFC_MANUAL_50HZ);
			break;
		case IS_AFC_MANUAL_60HZ:
			IS_ISP_SET_PARAM_AFC_CMD(dev, ISP_AFC_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_AFC_MANUAL(dev, ISP_AFC_MANUAL_60HZ);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_AFC_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AFC);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_FD_NUM_FACE:
		if (ctrl->value >= 0) {
			IS_FD_SET_PARAM_FDCONTROL_MAX_NUMBER(dev, ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_FD_FD);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_SET_ISP:
		switch (ctrl->value) {
		case IS_ISP_BYPASS_DISABLE:
			IS_ISP_SET_PARAM_CONTROL_BYPASS(dev,
				CONTROL_BYPASS_DISABLE);
			break;
		case IS_ISP_BYPASS_ENABLE:
			IS_ISP_SET_PARAM_CONTROL_BYPASS(dev,
				CONTROL_BYPASS_ENABLE);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_ISP_BYPASS_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_CONTROL);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_SET_DRC:
		switch (ctrl->value) {
		case IS_DRC_BYPASS_DISABLE:
			IS_DRC_SET_PARAM_CONTROL_BYPASS(dev,
				CONTROL_BYPASS_DISABLE);
			IS_DRC_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_START);
			break;
		case IS_DRC_BYPASS_ENABLE:
			IS_DRC_SET_PARAM_CONTROL_BYPASS(dev,
				CONTROL_BYPASS_ENABLE);
			IS_DRC_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_START);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_DRC_BYPASS_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_DRC_CONTROL);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_SET_FD:
		switch (ctrl->value) {
		case IS_FD_BYPASS_DISABLE:
			IS_FD_SET_PARAM_CONTROL_BYPASS(dev,
				CONTROL_BYPASS_DISABLE);
			IS_FD_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_START);
			break;
		case IS_FD_BYPASS_ENABLE:
			IS_FD_SET_PARAM_CONTROL_BYPASS(dev,
				CONTROL_BYPASS_ENABLE);
			IS_FD_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_STOP);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_FD_BYPASS_ENABLE) {
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONTROL);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CMD_ISP:
		switch (ctrl->value) {
		case IS_ISP_COMMAND_STOP:
			IS_ISP_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_STOP);
			break;
		case IS_ISP_COMMAND_START:
			IS_ISP_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_START);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_ISP_COMMAND_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_CONTROL);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CMD_DRC:
		switch (ctrl->value) {
		case IS_DRC_COMMAND_STOP:
			IS_DRC_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_STOP);
			break;
		case IS_DRC_COMMAND_START:
			IS_DRC_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_START);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_ISP_COMMAND_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_DRC_CONTROL);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CMD_FD:
		switch (ctrl->value) {
		case IS_FD_COMMAND_STOP:
			IS_FD_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_STOP);
			break;
		case IS_FD_COMMAND_START:
			IS_FD_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_START);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_ISP_COMMAND_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONTROL);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				(unsigned long)sizeof(IS_PARAM));
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_SET_FRAME_NUMBER:
		dev->frame_count = ctrl->value + 1;
		dev->is_p_region->header[0].valid = 0;
		dev->is_p_region->header[1].valid = 0;
		dev->is_p_region->header[2].valid = 0;
		dev->is_p_region->header[3].valid = 0;
		fimc_is_mem_cache_clean((void *)IS_HEADER,
			(unsigned long)sizeof(IS_HEADER));
		break;
	case V4L2_CID_IS_SET_FRAME_VALID:
		dev->is_p_region->header[0].valid = ctrl->value;
		dev->is_p_region->header[1].valid = ctrl->value;
		dev->is_p_region->header[2].valid = ctrl->value;
		dev->is_p_region->header[3].valid = ctrl->value;
		fimc_is_mem_cache_clean((void *)IS_HEADER,
			(unsigned long)sizeof(IS_HEADER));
		break;
	default:
		dbg("Invalid control\n");
		return -EINVAL;
	}

	return ret;
}

static int fimc_is_g_ext_ctrls(struct v4l2_subdev *sd,
	struct v4l2_ext_controls *ctrls)
{
	struct v4l2_ext_control *ctrl;
	int i, ret = 0;

	dbg("fimc_is_g_ext_ctrls\n");
	ctrl = ctrls->controls;
	if (!ctrls->ctrl_class == V4L2_CTRL_CLASS_CAMERA)
		return -EINVAL;

	for (i = 0; i < ctrls->count; i++, ctrl++) {
		/* ret = fimc_is_g_ctrl(sd, ctrl); */
		if (ret) {
			ctrls->error_idx = i;
			break;
		}
	}
	return ret;
}

static int fimc_is_s_ext_ctrls_handler(struct fimc_is_dev *dev,
	struct v4l2_ext_control *ctrl)
{
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_IS_S_SCENARIO_MODE:
		switch (ctrl->value) {
		case IS_MODE_PREVIEW_STILL:
			dev->scenario_id = ISS_PREVIEW_STILL;
			break;
		case IS_MODE_PREVIEW_VIDEO:
			dev->scenario_id = ISS_PREVIEW_VIDEO;
			break;
		case IS_MODE_CAPTURE_STILL:
			dev->scenario_id = ISS_CAPTURE_STILL;
			break;
		case IS_MODE_CAPTURE_VIDEO:
			dev->scenario_id = ISS_CAPTURE_VIDEO;
			break;
		}
		break;
	case V4L2_CID_IS_CMD_ISP:
		switch (ctrl->value) {
		case IS_ISP_COMMAND_STOP:
			IS_ISP_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_STOP);
			break;
		case IS_ISP_COMMAND_START:
			IS_ISP_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_START);
			break;
		}
		break;
	case V4L2_CID_IS_SET_ISP:
		switch (ctrl->value) {
		case IS_ISP_BYPASS_DISABLE:
			IS_ISP_SET_PARAM_CONTROL_BYPASS(dev,
				CONTROL_BYPASS_DISABLE);
			IS_ISP_SET_PARAM_CONTROL_ERR(dev, CONTROL_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_CONTROL);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_ISP_BYPASS_ENABLE:
			IS_ISP_SET_PARAM_CONTROL_BYPASS(dev,
				CONTROL_BYPASS_ENABLE);
			IS_ISP_SET_PARAM_CONTROL_ERR(dev, CONTROL_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_CONTROL);
			IS_INC_PARAM_NUM(dev);
			break;
		}
		break;
	case V4L2_CID_IS_CMD_DRC:
		switch (ctrl->value) {
		case IS_DRC_COMMAND_STOP:
			IS_ISP_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_STOP);
			break;
		case IS_DRC_COMMAND_START:
			IS_ISP_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_START);
			break;
		}
		break;
	case V4L2_CID_IS_SET_DRC:
		switch (ctrl->value) {
		case IS_DRC_BYPASS_DISABLE:
			IS_DRC_SET_PARAM_CONTROL_BYPASS(dev,
				CONTROL_BYPASS_DISABLE);
			IS_DRC_SET_PARAM_CONTROL_ERR(dev, CONTROL_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_DRC_CONTROL);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_DRC_BYPASS_ENABLE:
			IS_DRC_SET_PARAM_CONTROL_BYPASS(dev,
				CONTROL_BYPASS_ENABLE);
			IS_DRC_SET_PARAM_CONTROL_ERR(dev, CONTROL_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_DRC_CONTROL);
			IS_INC_PARAM_NUM(dev);
			break;
		}
		break;
	case V4L2_CID_IS_CMD_FD:
		switch (ctrl->value) {
		case IS_FD_COMMAND_STOP:
			IS_FD_SET_PARAM_CONTROL_CMD(dev, CONTROL_COMMAND_STOP);
			break;
		case IS_FD_COMMAND_START:
			IS_FD_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_START);
			break;
		}
		break;
	case V4L2_CID_IS_SET_FD:
		switch (ctrl->value) {
		case IS_FD_BYPASS_DISABLE:
			IS_DRC_SET_PARAM_CONTROL_BYPASS(dev,
				CONTROL_BYPASS_DISABLE);
			IS_DRC_SET_PARAM_CONTROL_ERR(dev, CONTROL_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONTROL);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_FD_BYPASS_ENABLE:
			IS_DRC_SET_PARAM_CONTROL_BYPASS(dev,
				CONTROL_BYPASS_ENABLE);
			IS_DRC_SET_PARAM_CONTROL_ERR(dev, CONTROL_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONTROL);
			IS_INC_PARAM_NUM(dev);
			break;
		}
		break;
	case V4L2_CID_IS_CAMERA_ISP_SEL_INPUT:
		switch (ctrl->value) {
		case IS_ISP_INPUT_OTF:
			IS_ISP_SET_PARAM_OTF_INPUT_CMD(dev,
				OTF_INPUT_COMMAND_ENABLE);
			IS_ISP_SET_PARAM_DMA_INPUT1_CMD(dev,
				DMA_INPUT_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_DMA_INPUT2_CMD(dev,
				DMA_INPUT_COMMAND_DISABLE);
			break;
		case IS_ISP_INPUT_DMA1:
			IS_ISP_SET_PARAM_OTF_INPUT_CMD(dev,
				DMA_INPUT_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_DMA_INPUT1_CMD(dev,
				OTF_INPUT_COMMAND_ENABLE);
			IS_ISP_SET_PARAM_DMA_INPUT2_CMD(dev,
				DMA_INPUT_COMMAND_DISABLE);
			break;
		case IS_ISP_INPUT_DMA2:
			IS_ISP_SET_PARAM_OTF_INPUT_CMD(dev,
				DMA_INPUT_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_DMA_INPUT1_CMD(dev,
				DMA_INPUT_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_DMA_INPUT2_CMD(dev,
				OTF_INPUT_COMMAND_ENABLE);
			break;
		}
		break;
	case V4L2_CID_IS_CAMERA_ISP_SEL_OUTPUT:
		switch (ctrl->value) {
		case IS_ISP_OUTPUT_OTF:
			IS_ISP_SET_PARAM_OTF_OUTPUT_CMD(dev,
				OTF_OUTPUT_COMMAND_ENABLE);
			IS_ISP_SET_PARAM_DMA_OUTPUT1_CMD(dev,
				DMA_OUTPUT_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_DMA_OUTPUT2_CMD(dev,
				DMA_OUTPUT_COMMAND_DISABLE);
			break;
		case IS_ISP_OUTPUT_DMA1:
			IS_ISP_SET_PARAM_OTF_OUTPUT_CMD(dev,
				DMA_OUTPUT_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_DMA_OUTPUT1_CMD(dev,
				OTF_OUTPUT_COMMAND_ENABLE);
			IS_ISP_SET_PARAM_DMA_OUTPUT2_CMD(dev,
				DMA_OUTPUT_COMMAND_DISABLE);
			break;
		case IS_ISP_OUTPUT_DMA2:
			IS_ISP_SET_PARAM_OTF_OUTPUT_CMD(dev,
				DMA_OUTPUT_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_DMA_OUTPUT1_CMD(dev,
				DMA_OUTPUT_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_DMA_OUTPUT2_CMD(dev,
				OTF_OUTPUT_COMMAND_ENABLE);
			break;
		case IS_ISP_OUTPUT_OTF_DMA1:
			IS_ISP_SET_PARAM_OTF_OUTPUT_CMD(dev,
				OTF_OUTPUT_COMMAND_ENABLE);
			IS_ISP_SET_PARAM_DMA_OUTPUT1_CMD(dev,
				OTF_OUTPUT_COMMAND_ENABLE);
			IS_ISP_SET_PARAM_DMA_OUTPUT2_CMD(dev,
				DMA_OUTPUT_COMMAND_DISABLE);
			break;
		case IS_ISP_OUTPUT_OTF_DMA2:
			IS_ISP_SET_PARAM_OTF_OUTPUT_CMD(dev,
				OTF_OUTPUT_COMMAND_ENABLE);
			IS_ISP_SET_PARAM_DMA_OUTPUT1_CMD(dev,
				DMA_OUTPUT_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_DMA_OUTPUT2_CMD(dev,
				OTF_OUTPUT_COMMAND_ENABLE);
			break;
		}
		break;
	case V4L2_CID_IS_CAMERA_DRC_SEL_INPUT:
		switch (ctrl->value) {
		case IS_DRC_INPUT_OTF:
			IS_DRC_SET_PARAM_OTF_INPUT_CMD(dev,
				OTF_INPUT_COMMAND_ENABLE);
			IS_DRC_SET_PARAM_DMA_INPUT_CMD(dev,
				DMA_INPUT_COMMAND_DISABLE);
			/* The output type of DRC is fixed at OTF */
			IS_DRC_SET_PARAM_OTF_OUTPUT_CMD(dev,
			OTF_OUTPUT_COMMAND_ENABLE);
			break;
		case IS_DRC_INPUT_DMA:
			IS_DRC_SET_PARAM_OTF_INPUT_CMD(dev,
				DMA_INPUT_COMMAND_DISABLE);
			IS_DRC_SET_PARAM_DMA_INPUT_CMD(dev,
				OTF_INPUT_COMMAND_ENABLE);
			/* The output type of DRC is fixed at OTF */
			IS_DRC_SET_PARAM_OTF_OUTPUT_CMD(dev,
			OTF_OUTPUT_COMMAND_ENABLE);
			break;
		}
		break;
	case V4L2_CID_IS_CAMERA_FD_SEL_INPUT:
		switch (ctrl->value) {
		case IS_FD_INPUT_OTF:
			IS_FD_SET_PARAM_OTF_INPUT_CMD(dev,
				OTF_INPUT_COMMAND_ENABLE);
			IS_FD_SET_PARAM_DMA_INPUT_CMD(dev,
				DMA_INPUT_COMMAND_DISABLE);
			break;
		case IS_FD_INPUT_DMA:
			IS_FD_SET_PARAM_OTF_INPUT_CMD(dev,
				DMA_INPUT_COMMAND_DISABLE);
			IS_FD_SET_PARAM_DMA_INPUT_CMD(dev,
				OTF_INPUT_COMMAND_ENABLE);
			break;
		}
		break;
	case V4L2_CID_IS_CAMERA_INIT_WIDTH:
		dev->sensor.width = ctrl->value;
		/* Setting init width of ISP */
		IS_ISP_SET_PARAM_OTF_INPUT_WIDTH(dev, ctrl->value);
		IS_ISP_SET_PARAM_DMA_INPUT1_WIDTH(dev, ctrl->value);
		IS_ISP_SET_PARAM_DMA_INPUT2_WIDTH(dev, ctrl->value);
		IS_ISP_SET_PARAM_OTF_OUTPUT_WIDTH(dev, ctrl->value);
		IS_ISP_SET_PARAM_DMA_OUTPUT1_WIDTH(dev, ctrl->value);
		IS_ISP_SET_PARAM_DMA_OUTPUT2_WIDTH(dev, ctrl->value);
		/* Setting init width of DRC */
		IS_DRC_SET_PARAM_OTF_INPUT_WIDTH(dev, ctrl->value);
		IS_DRC_SET_PARAM_DMA_INPUT_WIDTH(dev, ctrl->value);
		IS_DRC_SET_PARAM_OTF_OUTPUT_WIDTH(dev, ctrl->value);
		/* Setting init width of FD */
		IS_FD_SET_PARAM_OTF_INPUT_WIDTH(dev, ctrl->value);
		IS_FD_SET_PARAM_DMA_INPUT_WIDTH(dev, ctrl->value);
		break;
	case V4L2_CID_IS_CAMERA_INIT_HEIGHT:
		dev->sensor.height = ctrl->value;
		/* Setting init height and format of ISP */
		IS_ISP_SET_PARAM_OTF_INPUT_HEIGHT(dev, ctrl->value);
		IS_ISP_SET_PARAM_OTF_INPUT_FORMAT(dev,
			OTF_INPUT_FORMAT_BAYER);
		IS_ISP_SET_PARAM_OTF_INPUT_BITWIDTH(dev,
			OTF_INPUT_BIT_WIDTH_10BIT);
		IS_ISP_SET_PARAM_OTF_INPUT_ORDER(dev,
			OTF_INPUT_ORDER_BAYER_GR_BG);
		IS_ISP_SET_PARAM_OTF_INPUT_ERR(dev,
			OTF_INPUT_ERROR_NO);
		IS_SET_PARAM_BIT(dev, PARAM_ISP_OTF_INPUT);
		IS_INC_PARAM_NUM(dev);
		IS_ISP_SET_PARAM_DMA_INPUT1_HEIGHT(dev, ctrl->value);
		IS_ISP_SET_PARAM_DMA_INPUT1_FORMAT(dev, 0);
		IS_ISP_SET_PARAM_DMA_INPUT1_BITWIDTH(dev, 0);
		IS_ISP_SET_PARAM_DMA_INPUT1_PLANE(dev, 0);
		IS_ISP_SET_PARAM_DMA_INPUT1_ORDER(dev, 0);
		IS_ISP_SET_PARAM_DMA_INPUT1_ERR(dev, 0);
		IS_SET_PARAM_BIT(dev, PARAM_ISP_DMA1_INPUT);
		IS_INC_PARAM_NUM(dev);
		IS_ISP_SET_PARAM_DMA_INPUT2_HEIGHT(dev, ctrl->value);
		IS_ISP_SET_PARAM_DMA_INPUT2_FORMAT(dev, 0);
		IS_ISP_SET_PARAM_DMA_INPUT2_BITWIDTH(dev, 0);
		IS_ISP_SET_PARAM_DMA_INPUT2_PLANE(dev, 0);
		IS_ISP_SET_PARAM_DMA_INPUT2_ORDER(dev, 0);
		IS_ISP_SET_PARAM_DMA_INPUT2_ERR(dev, 0);
		IS_SET_PARAM_BIT(dev, PARAM_ISP_DMA2_INPUT);
		IS_INC_PARAM_NUM(dev);
		IS_ISP_SET_PARAM_OTF_OUTPUT_HEIGHT(dev, ctrl->value);
		IS_ISP_SET_PARAM_OTF_OUTPUT_FORMAT(dev,
			OTF_OUTPUT_FORMAT_YUV444);
		IS_ISP_SET_PARAM_OTF_OUTPUT_BITWIDTH(dev,
			OTF_OUTPUT_BIT_WIDTH_12BIT);
		IS_ISP_SET_PARAM_OTF_OUTPUT_ORDER(dev,
			OTF_OUTPUT_ORDER_BAYER_GR_BG);
		IS_ISP_SET_PARAM_OTF_OUTPUT_ERR(dev, OTF_OUTPUT_ERROR_NO);
		IS_SET_PARAM_BIT(dev, PARAM_ISP_OTF_OUTPUT);
		IS_INC_PARAM_NUM(dev);
		IS_ISP_SET_PARAM_DMA_OUTPUT1_HEIGHT(dev, ctrl->value);
		IS_ISP_SET_PARAM_DMA_OUTPUT1_FORMAT(dev, 0);
		IS_ISP_SET_PARAM_DMA_OUTPUT1_BITWIDTH(dev, 0);
		IS_ISP_SET_PARAM_DMA_OUTPUT1_PLANE(dev, 0);
		IS_ISP_SET_PARAM_DMA_OUTPUT1_ORDER(dev, 0);
		IS_ISP_SET_PARAM_DMA_OUTPUT1_BUFFER_NUMBER(dev, 0);
		IS_ISP_SET_PARAM_DMA_OUTPUT1_BUFFER_ADDRESS(dev, 0);
		IS_ISP_SET_PARAM_DMA_OUTPUT1_ERR(dev, DMA_OUTPUT_ERROR_NO);
		IS_SET_PARAM_BIT(dev, PARAM_ISP_DMA1_OUTPUT);
		IS_INC_PARAM_NUM(dev);
		IS_ISP_SET_PARAM_DMA_OUTPUT2_HEIGHT(dev, ctrl->value);
		IS_ISP_SET_PARAM_DMA_OUTPUT2_FORMAT(dev, 0);
		IS_ISP_SET_PARAM_DMA_OUTPUT2_BITWIDTH(dev, 0);
		IS_ISP_SET_PARAM_DMA_OUTPUT2_PLANE(dev, 0);
		IS_ISP_SET_PARAM_DMA_OUTPUT2_ORDER(dev, 0);
		IS_ISP_SET_PARAM_DMA_OUTPUT2_BUFFER_NUMBER(dev, 0);
		IS_ISP_SET_PARAM_DMA_OUTPUT2_BUFFER_ADDRESS(dev, 0);
		IS_ISP_SET_PARAM_DMA_OUTPUT2_ERR(dev, DMA_OUTPUT_ERROR_NO);
		IS_SET_PARAM_BIT(dev, PARAM_ISP_DMA2_OUTPUT);
		IS_INC_PARAM_NUM(dev);
		/* Setting init height and format of DRC */
		IS_DRC_SET_PARAM_OTF_INPUT_HEIGHT(dev, ctrl->value);
		IS_DRC_SET_PARAM_OTF_INPUT_FORMAT(dev,
			OTF_INPUT_FORMAT_YUV444);
		IS_DRC_SET_PARAM_OTF_INPUT_BITWIDTH(dev,
			OTF_INPUT_BIT_WIDTH_12BIT);
		IS_DRC_SET_PARAM_OTF_INPUT_ORDER(dev,
			OTF_INPUT_ORDER_BAYER_GR_BG);
		IS_DRC_SET_PARAM_OTF_INPUT_ERR(dev, OTF_INPUT_ERROR_NO);
		IS_SET_PARAM_BIT(dev, PARAM_DRC_OTF_INPUT);
		IS_INC_PARAM_NUM(dev);
		IS_DRC_SET_PARAM_DMA_INPUT_HEIGHT(dev, ctrl->value);
		IS_DRC_SET_PARAM_DMA_INPUT_FORMAT(dev, 0);
		IS_DRC_SET_PARAM_DMA_INPUT_BITWIDTH(dev, 0);
		IS_DRC_SET_PARAM_DMA_INPUT_PLANE(dev, 0);
		IS_DRC_SET_PARAM_DMA_INPUT_ORDER(dev, 0);
		IS_DRC_SET_PARAM_DMA_INPUT_ERR(dev, 0);
		IS_SET_PARAM_BIT(dev, PARAM_DRC_DMA_INPUT);
		IS_INC_PARAM_NUM(dev);
		IS_DRC_SET_PARAM_OTF_OUTPUT_HEIGHT(dev, ctrl->value);
		IS_DRC_SET_PARAM_OTF_OUTPUT_FORMAT(dev,
			OTF_OUTPUT_FORMAT_YUV444);
		IS_DRC_SET_PARAM_OTF_OUTPUT_BITWIDTH(dev,
			OTF_OUTPUT_BIT_WIDTH_12BIT);
		IS_DRC_SET_PARAM_OTF_OUTPUT_ORDER(dev,
			OTF_OUTPUT_ORDER_BAYER_GR_BG);
		IS_DRC_SET_PARAM_OTF_OUTPUT_ERR(dev, OTF_OUTPUT_ERROR_NO);
		IS_SET_PARAM_BIT(dev, PARAM_DRC_OTF_OUTPUT);
		IS_INC_PARAM_NUM(dev);
		/* Setting init height and format of FD */
		IS_FD_SET_PARAM_OTF_INPUT_HEIGHT(dev, ctrl->value);
		IS_FD_SET_PARAM_OTF_INPUT_FORMAT(dev,
			OTF_INPUT_FORMAT_YUV444);
		IS_FD_SET_PARAM_OTF_INPUT_BITWIDTH(dev,
			OTF_INPUT_BIT_WIDTH_12BIT);
		IS_FD_SET_PARAM_OTF_INPUT_ORDER(dev,
			OTF_INPUT_ORDER_BAYER_GR_BG);
		IS_FD_SET_PARAM_OTF_INPUT_ERR(dev, OTF_INPUT_ERROR_NO);
		IS_SET_PARAM_BIT(dev, PARAM_FD_OTF_INPUT);
		IS_INC_PARAM_NUM(dev);
		IS_FD_SET_PARAM_DMA_INPUT_HEIGHT(dev, ctrl->value);
		IS_FD_SET_PARAM_DMA_INPUT_FORMAT(dev, 0);
		IS_FD_SET_PARAM_DMA_INPUT_BITWIDTH(dev, 0);
		IS_FD_SET_PARAM_DMA_INPUT_PLANE(dev, 0);
		IS_FD_SET_PARAM_DMA_INPUT_ORDER(dev, 0);
		IS_FD_SET_PARAM_DMA_INPUT_ERR(dev, 0);
		IS_SET_PARAM_BIT(dev, PARAM_FD_DMA_INPUT);
		IS_INC_PARAM_NUM(dev);
		break;
	/* ISP - AF mode */
	case V4L2_CID_IS_CAMERA_FOCUS_MODE:
		switch (ctrl->value) {
		case IS_FOCUS_MODE_AUTO:
			IS_ISP_SET_PARAM_AF_CMD(dev,
				ISP_AF_COMMAND_SET_FOCUSMODE);
			IS_ISP_SET_PARAM_AF_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AF_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AF_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AF_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_AF_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AF);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_FOCUS_MODE_MACRO:
			IS_ISP_SET_PARAM_AF_CMD(dev,
				ISP_AF_COMMAND_SET_FOCUSMODE);
			IS_ISP_SET_PARAM_AF_MODE(dev, ISP_AF_MODE_MACRO);
			IS_ISP_SET_PARAM_AF_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AF_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AF_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_AF_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AF);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_FOCUS_MODE_INFINITY:
			IS_ISP_SET_PARAM_AF_CMD(dev,
				ISP_AF_COMMAND_SET_FOCUSMODE);
			IS_ISP_SET_PARAM_AF_MODE(dev, ISP_AF_MODE_INFINITY);
			IS_ISP_SET_PARAM_AF_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AF_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AF_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AF_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_AF_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AF);
			IS_INC_PARAM_NUM(dev);
			break;
		}
		break;
	/* ISP - FLASH mode */
	case V4L2_CID_IS_CAMERA_FLASH_MODE:
		switch (ctrl->value) {
		case IS_FLASH_MODE_OFF:
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev, ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_FLASH_MODE_AUTO:
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_AUTO);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev, ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_FLASH_MODE_AUTO_REDEYE:
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_AUTO);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_ENABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev, ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			break;
		}
		break;
	/* ISP - AWB mode */
	case V4L2_CID_IS_CAMERA_AWB_MODE:
		switch (ctrl->value) {
		case IS_AWB_AUTO:
			IS_ISP_SET_PARAM_AWB_CMD(dev, ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev, 0);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_AWB_DAYLIGHT:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_AWB_CLOUDY:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_CLOUDY);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_AWB_TUNGSTEN:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_TUNGSTEN);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_AWB_FLUORESCENT:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_FLUORESCENT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			break;
		}
		break;
	/* ISP - EFFECT mode */
	case V4L2_CID_IS_CAMERA_IMAGE_EFFECT:
		switch (ctrl->value) {
		case IS_IMAGE_EFFECT_DISABLE:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_DISABLE);
			IS_ISP_SET_PARAM_EFFECT_ERR(dev,
				ISP_IMAGE_EFFECT_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_IMAGE_EFFECT);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_IMAGE_EFFECT_BLACK_WHITE:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_BLACK_WHITE);
			IS_ISP_SET_PARAM_EFFECT_ERR(dev,
				ISP_IMAGE_EFFECT_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_IMAGE_EFFECT);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_IMAGE_EFFECT_ANTIQUE:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_ANTIQUE);
			IS_ISP_SET_PARAM_EFFECT_ERR(dev,
				ISP_IMAGE_EFFECT_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_IMAGE_EFFECT);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_IMAGE_EFFECT_GREEN:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_GREEN);
			IS_ISP_SET_PARAM_EFFECT_ERR(dev,
				ISP_IMAGE_EFFECT_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_IMAGE_EFFECT);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_IMAGE_EFFECT_BLUE:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_BLUE);
			IS_ISP_SET_PARAM_EFFECT_ERR(dev,
				ISP_IMAGE_EFFECT_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_IMAGE_EFFECT);
			IS_INC_PARAM_NUM(dev);
			break;
		}
		break;
	/* ISP - ISO mode */
	case V4L2_CID_IS_CAMERA_ISO:
		switch (ctrl->value) {
		case IS_ISO_AUTO:
			IS_ISP_SET_PARAM_ISO_CMD(dev, ISP_ISO_COMMAND_AUTO);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			break;
		}
		break;
	/* ISP - ADJUST mode */
	case V4L2_CID_IS_CAMERA_CONTRAST:
		switch (ctrl->value) {
		case IS_CONTRAST_AUTO:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_AUTOCONTRAST);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_EXPOSURE(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_HUE(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_ERR(dev,
				ISP_ADJUST_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			break;
		}
		break;
	/* ISP - METERING mode */
	case V4L2_CID_IS_CAMERA_METERING:
		switch (ctrl->value) {
		case IS_METERING_CENTER:
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_CENTER);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_METERING_SPOT:
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_SPOT);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev,
				dev->sensor.width/2);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev,
				dev->sensor.height/2);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev,
				dev->sensor.width);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev,
				dev->sensor.height);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_METERING_MATRIX:
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_MATRIX);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev,
				dev->sensor.width);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev,
				dev->sensor.height);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			break;
		}
		break;
	/* ISP - AFC mode */
	case V4L2_CID_IS_CAMERA_AFC_MODE:
		switch (ctrl->value) {
		case IS_AFC_DISABLE:
			IS_ISP_SET_PARAM_AFC_CMD(dev, ISP_AFC_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_AFC_MANUAL(dev, 0);
			IS_ISP_SET_PARAM_AFC_ERR(dev, ISP_AFC_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AFC);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_AFC_AUTO:
			IS_ISP_SET_PARAM_AFC_CMD(dev, ISP_AFC_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AFC_MANUAL(dev, 0);
			IS_ISP_SET_PARAM_AFC_ERR(dev, ISP_AFC_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AFC);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_AFC_MANUAL_50HZ:
			IS_ISP_SET_PARAM_AFC_CMD(dev, ISP_AFC_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_AFC_MANUAL(dev, ISP_AFC_MANUAL_50HZ);
			IS_ISP_SET_PARAM_AFC_ERR(dev, ISP_AFC_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AFC);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_AFC_MANUAL_60HZ:
			IS_ISP_SET_PARAM_AFC_CMD(dev, ISP_AFC_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_AFC_MANUAL(dev, ISP_AFC_MANUAL_60HZ);
			IS_ISP_SET_PARAM_AFC_ERR(dev, ISP_AFC_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AFC);
			IS_INC_PARAM_NUM(dev);
			break;
		}
		break;
	/* FD  - Max face number */
	case V4L2_CID_IS_FD_NUM_FACE:
		IS_FD_SET_PARAM_FDCONTROL_MAX_NUMBER(dev, ctrl->value);
		IS_FD_SET_PARAM_FDCONTROL_ERR(dev, FD_ERROR_NO);
		IS_SET_PARAM_BIT(dev, PARAM_FD_FD);
		IS_INC_PARAM_NUM(dev);
		break;
	default:
		dbg("Invalid control\n");
		return -EINVAL;
	}
	return ret;
}

static int fimc_is_s_ext_ctrls(struct v4l2_subdev *sd,
	struct v4l2_ext_controls *ctrls)
{
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);
	struct v4l2_ext_control *ctrl = ctrls->controls;
	int i, ret = 0;

	dbg("fimc_is_s_ext_ctrls\n");
	if (!ctrls->ctrl_class == V4L2_CTRL_CLASS_CAMERA)
		return -EINVAL;

	for (i = 0; i < ctrls->count; i++, ctrl++) {
		ret = fimc_is_s_ext_ctrls_handler(dev, ctrl);
		if (ret) {
			ctrls->error_idx = i;
			break;
		}
	}
	fimc_is_mem_cache_clean((void *)dev->is_p_region,
		(unsigned long)sizeof(IS_PARAM));
	fimc_is_hw_set_param(dev);
	return ret;
}

/* v4l2 subdev video operations
*/
static int fimc_is_try_mbus_fmt(struct v4l2_subdev *sd,
struct v4l2_mbus_framefmt *mf)
{
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);
	dbg("----fimc_is_try_mbus_fmt----\n");
	/* for otf, only one image format is available */
	IS_ISP_SET_PARAM_OTF_INPUT_WIDTH(dev, mf->width);
	IS_ISP_SET_PARAM_OTF_INPUT_HEIGHT(dev, mf->height);
	IS_ISP_SET_PARAM_OTF_OUTPUT_WIDTH(dev, mf->width);
	IS_ISP_SET_PARAM_OTF_OUTPUT_HEIGHT(dev, mf->height);

	IS_DRC_SET_PARAM_OTF_INPUT_WIDTH(dev, mf->width);
	IS_DRC_SET_PARAM_OTF_INPUT_HEIGHT(dev, mf->height);
	IS_DRC_SET_PARAM_OTF_OUTPUT_WIDTH(dev, mf->width);
	IS_DRC_SET_PARAM_OTF_OUTPUT_HEIGHT(dev, mf->height);

	IS_FD_SET_PARAM_OTF_INPUT_WIDTH(dev, mf->width);
	IS_FD_SET_PARAM_OTF_INPUT_HEIGHT(dev, mf->height);
	return 0;
}

static int fimc_is_g_mbus_fmt(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *mf)
{
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);
	dbg("fimc_is_g_mbus_fmt\n");
	/* for otf, only one image format is available */
	IS_DRC_GET_PARAM_OTF_OUTPUT_WIDTH(dev, mf->width);
	IS_DRC_GET_PARAM_OTF_OUTPUT_HEIGHT(dev, mf->height);
	mf->code = V4L2_MBUS_FMT_YUYV8_2X8;
	mf->field = 0;
	return 0;
}

static int fimc_is_s_mbus_fmt(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *mf)
{
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);

	dbg("fimc_is_s_mbus_fmt");
	/* scenario ID setting */
	switch (mf->field) {
	case 0:
		dev->scenario_id = ISS_PREVIEW_STILL;
		break;
	case 1:
		dev->scenario_id = ISS_PREVIEW_VIDEO;
		break;
	case 2:
		dev->scenario_id = ISS_CAPTURE_STILL;
		break;
	case 3:
		dev->scenario_id = ISS_CAPTURE_VIDEO;
		break;
	}
	/* 1. ISP input / output*/
	IS_ISP_SET_PARAM_CONTROL_CMD(dev, CONTROL_COMMAND_START);
	IS_SET_PARAM_BIT(dev, PARAM_ISP_CONTROL);
	IS_INC_PARAM_NUM(dev);
	IS_ISP_SET_PARAM_OTF_INPUT_CMD(dev, OTF_INPUT_COMMAND_ENABLE);
	IS_ISP_SET_PARAM_OTF_INPUT_WIDTH(dev, mf->width);
	IS_ISP_SET_PARAM_OTF_INPUT_HEIGHT(dev, mf->height);
	IS_ISP_SET_PARAM_OTF_INPUT_FORMAT(dev, OTF_INPUT_FORMAT_BAYER);
	IS_ISP_SET_PARAM_OTF_INPUT_BITWIDTH(dev, OTF_INPUT_BIT_WIDTH_10BIT);
	IS_ISP_SET_PARAM_OTF_INPUT_ORDER(dev, OTF_INPUT_ORDER_BAYER_GR_BG);
	IS_ISP_SET_PARAM_OTF_INPUT_ERR(dev, OTF_INPUT_ERROR_NO);
	IS_SET_PARAM_BIT(dev, PARAM_ISP_OTF_INPUT);
	IS_INC_PARAM_NUM(dev);
	IS_ISP_SET_PARAM_OTF_OUTPUT_CMD(dev, OTF_OUTPUT_COMMAND_ENABLE);
	IS_ISP_SET_PARAM_OTF_OUTPUT_WIDTH(dev, mf->width);
	IS_ISP_SET_PARAM_OTF_OUTPUT_HEIGHT(dev, mf->height);
	IS_ISP_SET_PARAM_OTF_OUTPUT_FORMAT(dev, OTF_OUTPUT_FORMAT_YUV444);
	IS_ISP_SET_PARAM_OTF_OUTPUT_BITWIDTH(dev, OTF_OUTPUT_BIT_WIDTH_12BIT);
	IS_ISP_SET_PARAM_OTF_OUTPUT_ORDER(dev, OTF_OUTPUT_ORDER_BAYER_GR_BG);
	IS_ISP_SET_PARAM_OTF_OUTPUT_ERR(dev, OTF_OUTPUT_ERROR_NO);
	IS_SET_PARAM_BIT(dev, PARAM_ISP_OTF_OUTPUT);
	IS_INC_PARAM_NUM(dev);

	/* 2. DRC input / output*/
	IS_DRC_SET_PARAM_CONTROL_CMD(dev, CONTROL_COMMAND_START);
	IS_SET_PARAM_BIT(dev, PARAM_DRC_CONTROL);
	IS_INC_PARAM_NUM(dev);
	IS_DRC_SET_PARAM_OTF_INPUT_CMD(dev, OTF_INPUT_COMMAND_ENABLE);
	IS_DRC_SET_PARAM_OTF_INPUT_WIDTH(dev, mf->width);
	IS_DRC_SET_PARAM_OTF_INPUT_HEIGHT(dev, mf->height);
	IS_DRC_SET_PARAM_OTF_INPUT_FORMAT(dev, OTF_INPUT_FORMAT_YUV444);
	IS_DRC_SET_PARAM_OTF_INPUT_BITWIDTH(dev, OTF_INPUT_BIT_WIDTH_12BIT);
	IS_DRC_SET_PARAM_OTF_INPUT_ORDER(dev, OTF_INPUT_ORDER_BAYER_GR_BG);
	IS_DRC_SET_PARAM_OTF_INPUT_ERR(dev, OTF_INPUT_ERROR_NO);
	IS_SET_PARAM_BIT(dev, PARAM_DRC_OTF_INPUT);
	IS_INC_PARAM_NUM(dev);
	IS_DRC_SET_PARAM_OTF_OUTPUT_CMD(dev, OTF_OUTPUT_COMMAND_ENABLE);
	IS_DRC_SET_PARAM_OTF_OUTPUT_WIDTH(dev, mf->width);
	IS_DRC_SET_PARAM_OTF_OUTPUT_HEIGHT(dev, mf->height);
	IS_DRC_SET_PARAM_OTF_OUTPUT_FORMAT(dev, OTF_OUTPUT_FORMAT_YUV444);
	IS_DRC_SET_PARAM_OTF_OUTPUT_BITWIDTH(dev, OTF_OUTPUT_BIT_WIDTH_8BIT);
	IS_DRC_SET_PARAM_OTF_OUTPUT_ORDER(dev, OTF_OUTPUT_ORDER_BAYER_GR_BG);
	IS_DRC_SET_PARAM_OTF_OUTPUT_ERR(dev, OTF_OUTPUT_ERROR_NO);
	IS_SET_PARAM_BIT(dev, PARAM_DRC_OTF_OUTPUT);
	IS_INC_PARAM_NUM(dev);
	/* 3. FD input / output*/
	IS_FD_SET_PARAM_CONTROL_CMD(dev, CONTROL_COMMAND_STOP);
	IS_SET_PARAM_BIT(dev, PARAM_FD_CONTROL);
	IS_INC_PARAM_NUM(dev);
	IS_FD_SET_PARAM_OTF_INPUT_CMD(dev, OTF_INPUT_COMMAND_ENABLE);
	IS_FD_SET_PARAM_OTF_INPUT_WIDTH(dev, mf->width);
	IS_FD_SET_PARAM_OTF_INPUT_HEIGHT(dev, mf->height);
	IS_FD_SET_PARAM_OTF_INPUT_FORMAT(dev, OTF_INPUT_FORMAT_YUV444);
	IS_FD_SET_PARAM_OTF_INPUT_BITWIDTH(dev, OTF_INPUT_BIT_WIDTH_8BIT);
	IS_FD_SET_PARAM_OTF_INPUT_ORDER(dev, OTF_INPUT_ORDER_BAYER_GR_BG);
	IS_FD_SET_PARAM_OTF_INPUT_ERR(dev, OTF_INPUT_ERROR_NO);
	IS_SET_PARAM_BIT(dev, PARAM_FD_OTF_INPUT);
	IS_INC_PARAM_NUM(dev);

	fimc_is_mem_cache_clean((void *)dev->is_p_region,
		(unsigned long)sizeof(IS_PARAM));
	fimc_is_hw_set_param(dev);

	return 0;
}

static int fimc_is_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);

	if (enable) {
		if (test_bit(IS_ST_RUN, &dev->state)) {
			dbg("IS Stream On\n");
			clear_bit(IS_ST_RUN, &dev->state);
			fimc_is_hw_set_stream(dev, enable);
			ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_STREAM, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&dev->pdev->dev,
					"wait timeout : %s\n", __func__);
				return -EBUSY;
			}
		} else {
			dev_err(&dev->pdev->dev, "not stream-on condition\n");
			return -EINVAL;
		}
	} else {
		dbg("IS Stream Off\n");
		if (test_and_clear_bit(IS_ST_STREAM, &dev->state))
			clear_bit(IS_ST_RUN, &dev->state);
		fimc_is_hw_set_stream(dev, enable);
		ret = wait_event_timeout(dev->irq_queue1,
		test_bit(IS_ST_RUN, &dev->state),
		FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
	}
	return 0;
}

const struct v4l2_subdev_core_ops fimc_is_core_ops = {
	.load_fw = fimc_is_load_fw,
	.init = fimc_is_init_set,
	.reset = fimc_is_reset,
	.s_power = fimc_is_s_power,
	.g_ctrl = fimc_is_g_ctrl,
	.s_ctrl = fimc_is_s_ctrl,
	.g_ext_ctrls = fimc_is_g_ext_ctrls,
	.s_ext_ctrls = fimc_is_s_ext_ctrls,
	.s_gpio = fimc_is_s_fimc_lite,
};

const struct v4l2_subdev_video_ops fimc_is_video_ops = {
	.try_mbus_fmt	= fimc_is_try_mbus_fmt,
	.g_mbus_fmt	= fimc_is_g_mbus_fmt,
	.s_mbus_fmt	= fimc_is_s_mbus_fmt,
	.s_stream	= fimc_is_s_stream,
};

const struct v4l2_subdev_ops fimc_is_subdev_ops = {
	.core	= &fimc_is_core_ops,
	.video	= &fimc_is_video_ops,
};
