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

#include <linux/firmware.h>
#include <linux/dma-mapping.h>

#include <linux/vmalloc.h>

#include "fimc-is-core.h"
#include "fimc-is-regs.h"
#include "fimc-is-param.h"
#include "fimc-is-cmd.h"
#include "fimc-is-err.h"

/* Binary load functions */
static int fimc_is_request_firmware(struct fimc_is_dev *dev)
{
	int ret;
	struct firmware *fw_blob;
	u8 *buf = NULL;
#ifdef SDCARD_FW
	struct file *fp;
	mm_segment_t old_fs;
	long fsize, nread;
	int fw_requested = 1;

	ret = 0;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(FIMC_IS_FW_SDCARD, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		dbg("failed to open %s\n", FIMC_IS_FW_SDCARD);
		goto request_fw;
	}
	fw_requested = 0;
	fsize = fp->f_path.dentry->d_inode->i_size;
	dbg("start, file path %s, size %ld Bytes\n", FIMC_IS_FW_SDCARD, fsize);
	buf = vmalloc(fsize);
	if (!buf) {
		err("failed to allocate memory\n");
		ret = -ENOMEM;
		goto out;
	}
	nread = vfs_read(fp, (char __user *)buf, fsize, &fp->f_pos);
	if (nread != fsize) {
		err("failed to read firmware file, %ld Bytes\n", nread);
		ret = -EIO;
		goto out;
	}

#if defined(CONFIG_VIDEOBUF2_CMA_PHYS)
	memcpy((void *)phys_to_virt(dev->mem.base), (void *)buf, fsize);
	fimc_is_mem_cache_clean((void *)phys_to_virt(dev->mem.base),
		fsize + 1);
#elif defined(CONFIG_VIDEOBUF2_ION)
	if (dev->mem.bitproc_buf == 0) {
		err("failed to load FIMC-IS F/W, FIMC-IS will not working\n");
	} else {
		memcpy(dev->mem.kvaddr, (void *)buf, fsize);
		fimc_is_mem_cache_clean((void *)dev->mem.kvaddr, fsize + 1);
		dev->fw.state = 1;
		dbg("FIMC_IS F/W loaded successfully - size:%ld\n", fsize);
	}
#endif
	dev->fw.state = 1;
request_fw:
	if (fw_requested) {
		set_fs(old_fs);
#endif
		ret = request_firmware((const struct firmware **)&fw_blob,
					FIMC_IS_FW, &dev->pdev->dev);
		if (ret) {
			dev_err(&dev->pdev->dev,
				"could not load firmware (err=%d)\n", ret);
			return -EINVAL;
		}
#if defined(CONFIG_VIDEOBUF2_CMA_PHYS)
		memcpy((void *)phys_to_virt(dev->mem.base),
				fw_blob->data, fw_blob->size);
		fimc_is_mem_cache_clean((void *)phys_to_virt(dev->mem.base),
							fw_blob->size + 1);
#elif defined(CONFIG_VIDEOBUF2_ION)
		if (dev->mem.bitproc_buf == 0) {
			err("failed to load FIMC-IS F/W\n");
		} else {
			memcpy(dev->mem.kvaddr, fw_blob->data, fw_blob->size);
			fimc_is_mem_cache_clean(
				(void *)dev->mem.kvaddr, fw_blob->size + 1);
			dbg(
		"FIMC_IS F/W loaded successfully - size:%d\n", fw_blob->size);
		}
#endif
		dev->fw.state = 1;
		dbg("FIMC_IS F/W loaded successfully - size:%d\n",
							fw_blob->size);
		release_firmware(fw_blob);
#ifdef SDCARD_FW
	}
#endif

out:
#ifdef SDCARD_FW
	if (!fw_requested) {
		vfree(buf);
		filp_close(fp, current->files);
		set_fs(old_fs);
	}
#endif
	return ret;
}

static int fimc_is_load_setfile(struct fimc_is_dev *dev)
{
	int ret;
	struct firmware *fw_blob;
	u8 *buf = NULL;
#ifdef SDCARD_FW
	struct file *fp;
	mm_segment_t old_fs;
	long fsize, nread;
	int fw_requested = 1;

	ret = 0;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(FIMC_IS_SETFILE_SDCARD, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		dbg("failed to open %s\n", FIMC_IS_SETFILE_SDCARD);
		goto request_fw;
	}
	fw_requested = 0;
	fsize = fp->f_path.dentry->d_inode->i_size;
	dbg("start, file path %s, size %ld Bytes\n",
		FIMC_IS_SETFILE_SDCARD, fsize);
	buf = vmalloc(fsize);
	if (!buf) {
		err("failed to allocate memory\n");
		ret = -ENOMEM;
		goto out;
	}
	nread = vfs_read(fp, (char __user *)buf, fsize, &fp->f_pos);
	if (nread != fsize) {
		err("failed to read firmware file, %ld Bytes\n", nread);
		ret = -EIO;
		goto out;
	}
#if defined(CONFIG_VIDEOBUF2_CMA_PHYS)
	memcpy((void *)phys_to_virt(dev->mem.base + dev->setfile.base),
							(void *)buf, fsize);
	fimc_is_mem_cache_clean(
		(void *)phys_to_virt(dev->mem.base + dev->setfile.base),
		fsize + 1);
#elif defined(CONFIG_VIDEOBUF2_ION)
	if (dev->mem.bitproc_buf == 0) {
		err("failed to load FIMC-IS F/W, FIMC-IS will not working\n");
	} else {
		memcpy((dev->mem.kvaddr + dev->setfile.base),
						(void *)buf, fsize);
		fimc_is_mem_cache_clean((void *)dev->mem.kvaddr, fsize + 1);
		dev->fw.state = 1;
		dbg("FIMC_IS F/W loaded successfully - size:%ld\n", fsize);
	}
#endif
	dev->fw.state = 1;
request_fw:
	if (fw_requested) {
		set_fs(old_fs);
#endif
		ret = request_firmware((const struct firmware **)&fw_blob,
					FIMC_IS_SETFILE, &dev->pdev->dev);
		if (ret) {
			dev_err(&dev->pdev->dev,
				"could not load firmware (err=%d)\n", ret);
			return -EINVAL;
		}
#if defined(CONFIG_VIDEOBUF2_CMA_PHYS)
		memcpy((void *)phys_to_virt(dev->mem.base + dev->setfile.base),
			fw_blob->data, fw_blob->size);
		fimc_is_mem_cache_clean(
			(void *)phys_to_virt(dev->mem.base + dev->setfile.base),
			fw_blob->size + 1);
#elif defined(CONFIG_VIDEOBUF2_ION)
		if (dev->mem.bitproc_buf == 0) {
			err("failed to load FIMC-IS F/W\n");
		} else {
			memcpy((dev->mem.kvaddr + dev->setfile.base),
						fw_blob->data, fw_blob->size);
			fimc_is_mem_cache_clean((void *)dev->mem.kvaddr,
						fw_blob->size + 1);
			dbg(
		"FIMC_IS F/W loaded successfully - size:%d\n", fw_blob->size);
		}
#endif
		dev->setfile.state = 1;
		dbg("FIMC_IS setfile loaded successfully - size:%d\n",
								fw_blob->size);
		release_firmware(fw_blob);
#ifdef SDCARD_FW
	}
#endif

	dbg("A5 mem base  = 0x%08x\n", dev->mem.base);
	dbg("Setfile base  = 0x%08x\n", dev->setfile.base);
out:
#ifdef SDCARD_FW
	if (!fw_requested) {
		vfree(buf);
		filp_close(fp, current->files);
		set_fs(old_fs);
	}
#endif
	return ret;
}

/* v4l2 subdev core operations
*/
static int fimc_is_load_fw(struct v4l2_subdev *sd)
{
	u32 timeout;
	int ret;
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);
	dbg("fimc_is_load_fw\n");
	if (test_bit(IS_ST_IDLE, &dev->state)) {
		/* 1. Load IS firmware */
		dev->fw.state = 0;
		ret = fimc_is_request_firmware(dev);
		if (ret) {
			dev_err(&dev->pdev->dev,
			"failed to fimc_is_request_firmware (%d)\n", ret);
			return -EINVAL;
		}
		timeout = 30;
		while (!dev->fw.state) {
			if (timeout == 0) {
				dev_err(&dev->pdev->dev,
				"Load firmware failed : %s\n", __func__);
			}
			timeout--;
			mdelay(1);
		}
		set_bit(IS_ST_PWR_ON, &dev->state);
		/* 2. Init GPIO (UART) */
		ret = fimc_is_hw_io_init(dev);
		if (ret) {
			dev_err(&dev->pdev->dev,
				"failed to init GPIO config\n");
			return -EINVAL;
		}
		/* 3. A5 power on */
		clear_bit(IS_ST_FW_DOWNLOADED, &dev->state);
		fimc_is_hw_a5_power(dev, 1);
		ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_FW_DOWNLOADED, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout A5 power on: %s\n", __func__);
			return -EINVAL;
		}
		dbg("fimc_is_load_fw end\n");
	} else {
		dbg("IS FW was loaded before\n");
	}
	return 0;
}

static int fimc_is_reset(struct v4l2_subdev *sd, u32 val)
{
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
	dev->sensor.sensor_type = val;
	dev->sensor.id = 0;
	dbg("fimc_is_init\n");
	if (test_bit(IS_ST_FW_DOWNLOADED, &dev->state)) {
		/* Init sequence 1: Open sensor */
		dbg("v4l2 : open sensor : %d\n", val);
		clear_bit(IS_ST_INIT_PREVIEW_STILL, &dev->state);
		fimc_is_hw_open_sensor(dev, dev->sensor.id, val);
		ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_INIT_PREVIEW_STILL, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT_SENSOR);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout:%s\n", __func__);
			return -EBUSY;
		}
		/* Init sequence 2: Load setfile */
		clear_bit(IS_ST_SET_FILE, &dev->state);
		ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_SET_FILE, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout:%s\n", __func__);
			return -EBUSY;
		}
		fimc_is_load_setfile(dev);
		fimc_is_hw_wait_intmsr0_intmsd0(dev);
		fimc_is_hw_set_load_setfile(dev);
		fimc_is_hw_set_intgr0_gd0(dev);
		dbg("v4l2 : Load set file end\n");
		/* Debug only */
		dbg("Parameter region addr = 0x%08x\n",
			virt_to_phys(dev->is_p_region));
		dbg("ISP region addr = 0x%08x\n",
			virt_to_phys(&dev->is_p_region->parameter.isp));
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

		dbg("Stream Off\n");
		clear_bit(IS_ST_STREAM_OFF, &dev->state);
		fimc_is_hw_set_stream(dev, 0); /*stream off */
		ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_STREAM_OFF, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
		clear_bit(IS_ST_RUN, &dev->state);

		/* 1. */
		dbg("Default setting : preview_still\n");
		dev->scenario_id = ISS_PREVIEW_STILL;
		fimc_is_hw_set_init(dev);

		fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
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
			IS_PARAM_SIZE);
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
			IS_PARAM_SIZE);
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
			IS_PARAM_SIZE);
		fimc_is_hw_set_param(dev);
		ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_RUN, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&dev->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
		clear_bit(IS_ST_STREAM_OFF, &dev->state);
		set_bit(IS_ST_RUN, &dev->state);
		dbg("Init sequence completed!! Ready to use\n");
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
		clear_bit(FIMC_IS_PWR_ST_POWEROFF, &is_dev->power);
		set_bit(FIMC_IS_PWR_ST_POWERED, &is_dev->power);
		ret = pm_runtime_get_sync(dev);
	} else {
		clear_bit(FIMC_IS_PWR_ST_POWERED, &is_dev->power);
		fimc_is_hw_subip_poweroff(is_dev);
		ret = wait_event_timeout(is_dev->irq_queue1,
			test_bit(FIMC_IS_PWR_ST_POWEROFF, &is_dev->power),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&is_dev->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
		fimc_is_hw_a5_power(is_dev, 0);
		dbg("A5 power off\n");
		ret = pm_runtime_put_sync(dev);

		is_dev->sensor.id = 0;
		is_dev->p_region_index1 = 0;
		is_dev->p_region_index2 = 0;
		atomic_set(&is_dev->p_region_num, 0);
		is_dev->state = 0;
		set_bit(IS_ST_IDLE, &is_dev->state);
		is_dev->power = 0;
		is_dev->af.af_state = FIMC_IS_AF_IDLE;
		set_bit(FIMC_IS_PWR_ST_POWEROFF, &is_dev->power);
	}

	if ((!ret && !on) || (ret && on))
		clear_bit(FIMC_IS_PWR_ST_POWERED, &is_dev->power);

	return ret;
}

static int fimc_is_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	int i, max;
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);

	switch (ctrl->id) {
	/* EXIF information */
	case V4L2_CID_IS_CAMERA_EXIF_EXPTIME:
	case V4L2_CID_CAMERA_EXIF_EXPTIME: /* Exposure Time */
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)(sizeof(struct is_frame_header)*4));
		ctrl->value = dev->is_p_region->header[0].
			exif.exposure_time.num;
		break;
	case V4L2_CID_IS_CAMERA_EXIF_FLASH:
	case V4L2_CID_CAMERA_EXIF_FLASH: /* Flash */
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)(sizeof(struct is_frame_header)*4));
		ctrl->value = dev->is_p_region->header[0].exif.flash;
		break;
	case V4L2_CID_IS_CAMERA_EXIF_ISO:
	case V4L2_CID_CAMERA_EXIF_ISO: /* ISO Speed Rating */
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)(sizeof(struct is_frame_header)*4));
		ctrl->value = dev->is_p_region->header[0].
			exif.iso_speed_rating;
		break;
	case V4L2_CID_IS_CAMERA_EXIF_SHUTTERSPEED:
	case V4L2_CID_CAMERA_EXIF_TV: /* Shutter Speed */
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)(sizeof(struct is_frame_header)*4));
		ctrl->value = dev->is_p_region->header[0].exif.
			shutter_speed.num;
		break;
	case V4L2_CID_IS_CAMERA_EXIF_BRIGHTNESS:
	case V4L2_CID_CAMERA_EXIF_BV: /* Brightness */
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)(sizeof(struct is_frame_header)*4));
		ctrl->value = dev->is_p_region->header[0].exif.brightness.num;
		break;
	case V4L2_CID_CAMERA_EXIF_EBV: /* exposure bias */
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)(sizeof(struct is_frame_header)*4));
		ctrl->value = dev->is_p_region->header[0].exif.brightness.den;
		break;
	/* Get x and y offset of sensor  */
	case V4L2_CID_IS_GET_SENSOR_OFFSET_X:
		ctrl->value = dev->sensor.offset_x;
		break;
	case V4L2_CID_IS_GET_SENSOR_OFFSET_Y:
		ctrl->value = dev->sensor.offset_y;
		break;
	/* Get current sensor size  */
	case V4L2_CID_IS_GET_SENSOR_WIDTH:
		switch (dev->scenario_id) {
		case ISS_PREVIEW_STILL:
			ctrl->value = dev->sensor.width_prev;
			break;
		case ISS_PREVIEW_VIDEO:
			ctrl->value = dev->sensor.width_prev_cam;
			break;
		case ISS_CAPTURE_STILL:
			ctrl->value = dev->sensor.width_cap;
			break;
		case ISS_CAPTURE_VIDEO:
			ctrl->value = dev->sensor.width_cam;
			break;
		}
		break;
	case V4L2_CID_IS_GET_SENSOR_HEIGHT:
		switch (dev->scenario_id) {
		case ISS_PREVIEW_STILL:
			ctrl->value = dev->sensor.height_prev;
			break;
		case ISS_PREVIEW_VIDEO:
			ctrl->value = dev->sensor.height_prev_cam;
			break;
		case ISS_CAPTURE_STILL:
			ctrl->value = dev->sensor.height_cap;
			break;
		case ISS_CAPTURE_VIDEO:
			ctrl->value = dev->sensor.height_cam;
			break;
		}
		break;
	/* Get information related to frame management  */
	case V4L2_CID_IS_GET_FRAME_VALID:
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)(sizeof(struct is_frame_header)*4));
		if ((dev->scenario_id == ISS_PREVIEW_STILL) ||
			(dev->scenario_id == ISS_PREVIEW_VIDEO)) {
			ctrl->value = dev->is_p_region->header
			[dev->frame_count%MAX_FRAME_COUNT_PREVIEW].valid;
		} else {
			ctrl->value = dev->is_p_region->header[0].valid;
		}
		break;
	case V4L2_CID_IS_GET_FRAME_BADMARK:
		break;
	case V4L2_CID_IS_GET_FRAME_NUMBER:
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)(sizeof(struct is_frame_header)*4));
		if ((dev->scenario_id == ISS_PREVIEW_STILL) ||
			(dev->scenario_id == ISS_PREVIEW_VIDEO)) {
			ctrl->value =
				dev->is_p_region->header
				[dev->frame_count%MAX_FRAME_COUNT_PREVIEW].
				frame_number;
		} else {
			ctrl->value =
				dev->is_p_region->header[0].frame_number;
		}
		break;
	case V4L2_CID_IS_GET_LOSTED_FRAME_NUMBER:
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)(sizeof(struct is_frame_header)*4));
		if (dev->scenario_id == ISS_CAPTURE_STILL) {
			ctrl->value =
				dev->is_p_region->header[0].frame_number;
		} else if (dev->scenario_id == ISS_CAPTURE_VIDEO) {
			ctrl->value =
				dev->is_p_region->header[0].frame_number + 1;
		} else {
			max = dev->is_p_region->header[0].frame_number;
			for (i = 1; i < MAX_FRAME_COUNT_PREVIEW; i++) {
				if (max <
				dev->is_p_region->header[i].frame_number)
					max =
				dev->is_p_region->header[i].frame_number;
			}
			ctrl->value = max;
		}
		dev->frame_count = ctrl->value;
		break;
	case V4L2_CID_IS_GET_FRAME_CAPTURED:
		fimc_is_mem_cache_inv((void *)IS_HEADER,
			(unsigned long)(sizeof(struct is_frame_header)*4));
		ctrl->value =
			dev->is_p_region->header
			[dev->frame_count%MAX_FRAME_COUNT_PREVIEW].captured;
		break;
	case V4L2_CID_IS_FD_GET_DATA:
		ctrl->value = dev->fd_header.count;
		fimc_is_mem_cache_inv((void *)IS_FACE,
		(unsigned long)(sizeof(struct is_face_marker)*MAX_FACE_COUNT));
		memcpy((void *)dev->fd_header.target_addr,
			&dev->is_p_region->face[dev->fd_header.index],
			(sizeof(struct is_face_marker)*dev->fd_header.count));
		break;
	/* AF result */
	case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT:
		if (!is_af_use(dev))
			ctrl->value = 0x02;
		else
			ctrl->value = dev->af.af_lock_state;
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int fimc_is_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int ret = 0;
	int i;
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);

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
					"Mode change timeout:%s\n", __func__);
				return -EBUSY;
			}
		}
		break;
	case V4L2_CID_IS_S_FORMAT_SCENARIO:
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
		default:
			return -EBUSY;
		}
		break;
	case V4L2_CID_IS_CAMERA_SHOT_MODE_NORMAL:
		IS_SET_PARAM_GLOBAL_SHOTMODE_CMD(dev, ctrl->value);
		IS_SET_PARAM_BIT(dev, PARAM_GLOBAL_SHOTMODE);
		IS_INC_PARAM_NUM(dev);
		fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
		fimc_is_hw_set_param(dev);
		break;
	case V4L2_CID_CAMERA_FRAME_RATE:
		IS_SENSOR_SET_FRAME_RATE(dev, ctrl->value);
		IS_SET_PARAM_BIT(dev, PARAM_SENSOR_FRAME_RATE);
		IS_INC_PARAM_NUM(dev);
		fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
		fimc_is_hw_set_param(dev);
		break;
	/* Focus */
	case V4L2_CID_IS_CAMERA_OBJECT_POSITION_X:
	case V4L2_CID_CAMERA_OBJECT_POSITION_X:
		dev->af.pos_x = ctrl->value;
		/* default window size setting */
		dev->af.width = dev->sensor.width_prev;
		break;
	case V4L2_CID_IS_CAMERA_OBJECT_POSITION_Y:
	case V4L2_CID_CAMERA_OBJECT_POSITION_Y:
		dev->af.pos_y = ctrl->value;
		/* default window size setting */
		dev->af.height = dev->sensor.height_prev;
		break;
	case V4L2_CID_IS_CAMERA_WINDOW_SIZE_X:
		dev->af.width = ctrl->value;
		break;
	case V4L2_CID_IS_CAMERA_WINDOW_SIZE_Y:
		dev->af.height = ctrl->value;
		break;
	case V4L2_CID_CAMERA_FOCUS_MODE:
		switch (ctrl->value) {
		case FOCUS_MODE_AUTO:
			dev->af.mode = IS_FOCUS_MODE_AUTO;
			break;
		case FOCUS_MODE_MACRO:
			dev->af.mode = IS_FOCUS_MODE_MACRO;
			break;
		case FOCUS_MODE_INFINITY:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* In inifinity focus mode,
			Focus starts after mode change */
			dev->af.mode = IS_FOCUS_MODE_INFINITY;
			IS_ISP_SET_PARAM_AA_CMD(dev,
				ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
				ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_INFINITY);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_DISABLE);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case FOCUS_MODE_CONTINOUS:
			dev->af.mode = IS_FOCUS_MODE_CONTINUOUS;
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* In continuous focus mode,
			Focus starts after mode change */
			IS_ISP_SET_PARAM_AA_CMD(dev, ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev, ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
						ISP_AF_CONTINUOUS_ENABLE);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			dev->af.af_lock_state = 0;
			dev->af.ae_lock_state = 0;
			dev->af.awb_lock_state = 0;
			break;
		case FOCUS_MODE_TOUCH:
			dev->af.mode = IS_FOCUS_MODE_TOUCH;
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* In touch focus mode,
			Focus starts after mode change */
			IS_ISP_SET_PARAM_AA_CMD(dev, ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev, ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_TOUCH);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
						ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_WIN_POS_X(dev, dev->af.pos_x);
			IS_ISP_SET_PARAM_AA_WIN_POS_Y(dev, dev->af.pos_y);
			IS_ISP_SET_PARAM_AA_WIN_WIDTH(dev, dev->af.width);
			IS_ISP_SET_PARAM_AA_WIN_HEIGHT(dev, dev->af.height);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			dev->af.af_lock_state = 0;
			dev->af.ae_lock_state = 0;
			dev->af.awb_lock_state = 0;
			break;
		case FOCUS_MODE_FACEDETECT:
			dev->af.mode = IS_FOCUS_MODE_FACEDETECT;
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			break;
		}
		break;
	case V4L2_CID_CAMERA_SET_AUTO_FOCUS:
		switch (ctrl->value) {
		case AUTO_FOCUS_OFF:
			if (!is_af_use(dev)) {
				/* 6A3 can't support AF */
				dev->af.af_state = FIMC_IS_AF_IDLE;
			} else {
				/* Abort or lock AF */
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_ISP_SET_PARAM_AA_MODE(dev,
					ISP_AF_MODE_AUTO);
				IS_ISP_SET_PARAM_AA_FACE(dev,
					ISP_AF_FACE_DISABLE);
				IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			break;
		case AUTO_FOCUS_ON:
			if (!is_af_use(dev)) {
				/* 6A3 can't support AF */
				dev->af.af_state = FIMC_IS_AF_LOCK;
				dev->af.af_lock_state = FIMC_IS_AF_LOCKED;
			} else {
				/* ABORT first */
				if (dev->af.af_state != FIMC_IS_AF_IDLE) {
					dev->af.af_state = FIMC_IS_AF_ABORT;
					IS_ISP_SET_PARAM_AA_CMD(dev,
						ISP_AA_COMMAND_STOP);
					IS_ISP_SET_PARAM_AA_TARGET(dev,
						ISP_AA_TARGET_AF);
					IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
					IS_INC_PARAM_NUM(dev);
					fimc_is_mem_cache_clean(
						(void *)dev->is_p_region,
						IS_PARAM_SIZE);
					fimc_is_hw_set_param(dev);
					ret = wait_event_timeout(
						dev->irq_queue1,
					(dev->af.af_state == FIMC_IS_AF_IDLE),
					FIMC_IS_SHUTDOWN_TIMEOUT);
					if (!ret) {
						dev_err(&dev->pdev->dev,
						"Focus change timeout:%s\n",
						__func__);
						return -EBUSY;
					}
				}
				IS_ISP_SET_PARAM_AA_CMD(dev,
				ISP_AA_COMMAND_START);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				switch (dev->af.mode) {
				case IS_FOCUS_MODE_AUTO:
					IS_ISP_SET_PARAM_AA_MODE(dev,
							ISP_AF_MODE_AUTO);
					IS_ISP_SET_PARAM_AA_FACE(dev,
							ISP_AF_FACE_DISABLE);
					IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
						ISP_AF_CONTINUOUS_DISABLE);
					IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
					IS_INC_PARAM_NUM(dev);
					dev->af.af_state =
							FIMC_IS_AF_SETCONFIG;
					fimc_is_mem_cache_clean(
						(void *)dev->is_p_region,
						IS_PARAM_SIZE);
					fimc_is_hw_set_param(dev);
					break;
				case IS_FOCUS_MODE_MACRO:
					IS_ISP_SET_PARAM_AA_MODE(dev,
							ISP_AF_MODE_MACRO);
					IS_ISP_SET_PARAM_AA_FACE(dev,
							ISP_AF_FACE_DISABLE);
					IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
						ISP_AF_CONTINUOUS_DISABLE);
					IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
					IS_INC_PARAM_NUM(dev);
					dev->af.af_state =
							FIMC_IS_AF_SETCONFIG;
					fimc_is_mem_cache_clean(
						(void *)dev->is_p_region,
						IS_PARAM_SIZE);
					fimc_is_hw_set_param(dev);
					break;
				default:
					break;
				}
				dev->af.af_lock_state = 0;
				dev->af.ae_lock_state = 0;
				dev->af.awb_lock_state = 0;
			}
			break;
		}
		break;
	case V4L2_CID_CAMERA_TOUCH_AF_START_STOP:
		switch (ctrl->value) {
		case TOUCH_AF_STOP:
			break;
		case TOUCH_AF_START:
			break;
		default:
			break;
		}
		break;
	case V4L2_CID_CAMERA_CAF_START_STOP:
		switch (ctrl->value) {
		case CAF_STOP:
			break;
		case CAF_START:
			break;
		default:
			break;
		}
		break;
	/* AWB, AE Lock/Unlock */
	case V4L2_CID_CAMERA_AEAWB_LOCK_UNLOCK:
		switch (ctrl->value) {
		case AE_UNLOCK_AWB_UNLOCK:
			IS_ISP_SET_PARAM_AA_CMD(dev,
				ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
				ISP_AA_TARGET_AE | ISP_AA_TARGET_AWB);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean(
				(void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case AE_LOCK_AWB_UNLOCK:
			IS_ISP_SET_PARAM_AA_CMD(dev,
				ISP_AA_COMMAND_STOP);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
				ISP_AA_TARGET_AE);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean(
				(void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			IS_ISP_SET_PARAM_AA_CMD(dev,
				ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
				ISP_AA_TARGET_AWB);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean(
				(void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case AE_UNLOCK_AWB_LOCK:
			IS_ISP_SET_PARAM_AA_CMD(dev,
				ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
				ISP_AA_TARGET_AE);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean(
				(void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			IS_ISP_SET_PARAM_AA_CMD(dev,
				ISP_AA_COMMAND_STOP);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
				ISP_AA_TARGET_AWB);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean(
				(void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case AE_LOCK_AWB_LOCK:
			IS_ISP_SET_PARAM_AA_CMD(dev,
				ISP_AA_COMMAND_STOP);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
				ISP_AA_TARGET_AE | ISP_AA_TARGET_AWB);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean(
				(void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		default:
			break;
		}
		break;
	/* FLASH */
	case V4L2_CID_CAMERA_FLASH_MODE:
		switch (ctrl->value) {
		case FLASH_MODE_OFF:
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			break;
		case FLASH_MODE_AUTO:
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_AUTO);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_ENABLE);
			break;
		case FLASH_MODE_ON:
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_MANUALON);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			break;
		case FLASH_MODE_TORCH:
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_TORCH);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			break;
		}
		if (ctrl->value > FLASH_MODE_BASE
					&& ctrl->value < FLASH_MODE_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_CAMERA_WHITE_BALANCE:
		switch (ctrl->value) {
		case WHITE_BALANCE_AUTO:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev, 0);
			break;
		case WHITE_BALANCE_SUNNY:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			break;
		case WHITE_BALANCE_CLOUDY:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_CLOUDY);
			break;
		case WHITE_BALANCE_TUNGSTEN:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_TUNGSTEN);
			break;
		case WHITE_BALANCE_FLUORESCENT:
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_FLUORESCENT);
			break;
		}
		if (ctrl->value > WHITE_BALANCE_BASE
				&& ctrl->value < WHITE_BALANCE_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
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
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_CAMERA_EFFECT:
		switch (ctrl->value) {
		case IMAGE_EFFECT_NONE:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_DISABLE);
			break;
		case IMAGE_EFFECT_BNW:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_MONOCHROME);
			break;
		case IMAGE_EFFECT_NEGATIVE:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_NEGATIVE_MONO);
			break;
		case IMAGE_EFFECT_SEPIA:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_SEPIA);
			break;
		}
		/* only ISP effect in Pegasus */
		if (ctrl->value > IMAGE_EFFECT_BASE
				&& ctrl->value < IMAGE_EFFECT_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_IMAGE_EFFECT);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_IMAGE_EFFECT:
		switch (ctrl->value) {
		case IS_IMAGE_EFFECT_DISABLE:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_DISABLE);
			break;
		case IS_IMAGE_EFFECT_MONOCHROME:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_MONOCHROME);
			break;
		case IS_IMAGE_EFFECT_NEGATIVE_MONO:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_NEGATIVE_MONO);
			break;
		case IS_IMAGE_EFFECT_NEGATIVE_COLOR:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_NEGATIVE_COLOR);
			break;
		case IS_IMAGE_EFFECT_SEPIA:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_SEPIA);
			break;
		}
		/* only ISP effect in Pegasus */
		if (ctrl->value >= 0 && ctrl->value < 5) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_IMAGE_EFFECT);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_CAMERA_ISO:
		switch (ctrl->value) {
		case ISO_AUTO:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_AUTO);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			break;
		case ISO_50:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 50);
			break;
		case ISO_100:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 100);
			break;
		case ISO_200:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 200);
			break;
		case ISO_400:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 400);
			break;
		case ISO_800:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 800);
			break;
		case ISO_1600:
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 1600);
			break;
		}
		if (ctrl->value >= ISO_AUTO && ctrl->value < ISO_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
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
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_CAMERA_CONTRAST:
		switch (ctrl->value) {
		case CONTRAST_MINUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, -2);
			break;
		case CONTRAST_MINUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, -1);
			break;
		case CONTRAST_DEFAULT:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 0);
			break;
		case CONTRAST_PLUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 1);
			break;
		case CONTRAST_PLUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 2);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < CONTRAST_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
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
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_CAMERA_SATURATION:
		switch (ctrl->value) {
		case SATURATION_MINUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, -2);
			break;
		case SATURATION_MINUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, -1);
			break;
		case SATURATION_DEFAULT:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, 0);
			break;
		case SATURATION_PLUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, 1);
			break;
		case SATURATION_PLUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, 2);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < SATURATION_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
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
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_CAMERA_SHARPNESS:
		switch (ctrl->value) {
		case SHARPNESS_MINUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, -2);
			break;
		case SHARPNESS_MINUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, -1);
			break;
		case SHARPNESS_DEFAULT:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, 0);
			break;
		case SHARPNESS_PLUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, 1);
			break;
		case SHARPNESS_PLUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, 2);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < SHARPNESS_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
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
				IS_PARAM_SIZE);
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
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_CAMERA_BRIGHTNESS:
		switch (ctrl->value) {
		case EV_MINUS_4:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, EV_MINUS_4);
			break;
		case EV_MINUS_3:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, EV_MINUS_4);
			break;
		case EV_MINUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, EV_MINUS_4);
			break;
		case EV_MINUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, EV_MINUS_4);
			break;
		case EV_DEFAULT:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, EV_MINUS_4);
			break;
		case EV_PLUS_1:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, EV_MINUS_4);
			break;
		case EV_PLUS_2:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, EV_MINUS_4);
			break;
		case EV_PLUS_3:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, EV_MINUS_4);
			break;
		case EV_PLUS_4:
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, EV_MINUS_4);
			break;
		}
		if (ctrl->value >= EV_MINUS_4 && ctrl->value < EV_MAX) {
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_BRIGHTNESS:
		if (ctrl->value >= -4 && ctrl->value < 5) {
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_HUE:
		if (ctrl->value >= -4 && ctrl->value < 5) {
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_HUE(dev, ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_CAMERA_METERING:
		switch (ctrl->value) {
		case METERING_CENTER:
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			break;
		case METERING_SPOT:
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_SPOT);
			break;
		case METERING_MATRIX:
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_MATRIX);
			break;
		}
		if (ctrl->value > METERING_BASE && ctrl->value < METERING_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CAMERA_METERING:
		switch (ctrl->value) {
		case IS_METERING_AVERAGE:
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
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
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	/* Ony valid at SPOT Mode */
	case V4L2_CID_IS_CAMERA_METERING_POSITION_X:
		IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, ctrl->value);
		IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 10);
		break;
	case V4L2_CID_IS_CAMERA_METERING_POSITION_Y:
		IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, ctrl->value);
		IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 10);
		break;
	case V4L2_CID_IS_CAMERA_METERING_WINDOW_X:
		IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, ctrl->value);
		break;
	case V4L2_CID_IS_CAMERA_METERING_WINDOW_Y:
		IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, ctrl->value);
		break;
	case V4L2_CID_CAMERA_ANTI_BANDING:
		switch (ctrl->value) {
		case ANTI_BANDING_OFF:
			IS_ISP_SET_PARAM_AFC_CMD(dev, ISP_AFC_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_AFC_MANUAL(dev, 0);
			break;
		case ANTI_BANDING_AUTO:
			IS_ISP_SET_PARAM_AFC_CMD(dev, ISP_AFC_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AFC_MANUAL(dev, 0);
			break;
		case ANTI_BANDING_50HZ:
			IS_ISP_SET_PARAM_AFC_CMD(dev, ISP_AFC_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_AFC_MANUAL(dev, ISP_AFC_MANUAL_50HZ);
			break;
		case ANTI_BANDING_60HZ:
			IS_ISP_SET_PARAM_AFC_CMD(dev, ISP_AFC_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_AFC_MANUAL(dev, ISP_AFC_MANUAL_60HZ);
			break;
		}
		if (ctrl->value >= ANTI_BANDING_OFF
				&& ctrl->value <= ANTI_BANDING_60HZ) {
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AFC);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
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
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_FD_SET_MAX_FACE_NUMBER:
		if (ctrl->value >= 0) {
			IS_FD_SET_PARAM_FD_CONFIG_CMD(dev,
				FD_CONFIG_COMMAND_MAXIMUM_NUMBER);
			IS_FD_SET_PARAM_FD_CONFIG_MAX_NUMBER(dev, ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONFIG);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_FD_SET_ROLL_ANGLE:
		switch (ctrl->value) {
		case IS_FD_ROLL_ANGLE_BASIC:
			IS_FD_SET_PARAM_FD_CONFIG_CMD(dev,
				FD_CONFIG_COMMAND_ROLL_ANGLE);
			IS_FD_SET_PARAM_FD_CONFIG_ROLL_ANGLE(dev, ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONFIG);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case V4L2_CID_IS_FD_SET_YAW_ANGLE:
			IS_FD_SET_PARAM_FD_CONFIG_CMD(dev,
				FD_CONFIG_COMMAND_YAW_ANGLE);
			IS_FD_SET_PARAM_FD_CONFIG_YAW_ANGLE(dev, ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONFIG);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case V4L2_CID_IS_FD_SET_SMILE_MODE:
			IS_FD_SET_PARAM_FD_CONFIG_CMD(dev,
				FD_CONFIG_COMMAND_SMILE_MODE);
			IS_FD_SET_PARAM_FD_CONFIG_SMILE_MODE(dev, ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONFIG);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case V4L2_CID_IS_FD_SET_BLINK_MODE:
			IS_FD_SET_PARAM_FD_CONFIG_CMD(dev,
				FD_CONFIG_COMMAND_BLINK_MODE);
			IS_FD_SET_PARAM_FD_CONFIG_BLINK_MODE(dev, ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONFIG);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case V4L2_CID_IS_FD_SET_EYE_DETECT_MODE:
			IS_FD_SET_PARAM_FD_CONFIG_CMD(dev,
				FD_CONFIG_COMMAND_EYES_DETECT);
			IS_FD_SET_PARAM_FD_CONFIG_EYE_DETECT(dev, ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONFIG);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case V4L2_CID_IS_FD_SET_MOUTH_DETECT_MODE:
			IS_FD_SET_PARAM_FD_CONFIG_CMD(dev,
				FD_CONFIG_COMMAND_MOUTH_DETECT);
			IS_FD_SET_PARAM_FD_CONFIG_MOUTH_DETECT(dev,
							ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONFIG);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case V4L2_CID_IS_FD_SET_ORIENTATION_MODE:
			IS_FD_SET_PARAM_FD_CONFIG_CMD(dev,
				FD_CONFIG_COMMAND_ORIENTATION);
			IS_FD_SET_PARAM_FD_CONFIG_ORIENTATION(dev,
								ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONFIG);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case V4L2_CID_IS_FD_SET_ORIENTATION:
			IS_FD_SET_PARAM_FD_CONFIG_CMD(dev,
				FD_CONFIG_COMMAND_ORIENTATION_VALUE);
			IS_FD_SET_PARAM_FD_CONFIG_ORIENTATION_VALUE(dev,
								ctrl->value);
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONFIG);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		}
		break;
	case V4L2_CID_IS_FD_SET_DATA_ADDRESS:
		dev->fd_header.target_addr = ctrl->value;
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
				IS_PARAM_SIZE);
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
				IS_PARAM_SIZE);
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
				IS_PARAM_SIZE);
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
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_CMD_FD:
		switch (ctrl->value) {
		case IS_FD_COMMAND_STOP:
			dbg("IS_FD_COMMAND_STOP\n");
			IS_FD_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_STOP);
			break;
		case IS_FD_COMMAND_START:
			dbg("IS_FD_COMMAND_START\n");
			IS_FD_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_START);
			break;
		}
		if (ctrl->value >= 0 && ctrl->value < IS_ISP_COMMAND_MAX) {
			IS_SET_PARAM_BIT(dev, PARAM_FD_CONTROL);
			IS_INC_PARAM_NUM(dev);
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
				IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
		}
		break;
	case V4L2_CID_IS_SET_FRAME_NUMBER:
		dev->frame_count = ctrl->value + 1;
		dev->is_p_region->header[0].valid = 0;
		dev->is_p_region->header[1].valid = 0;
		dev->is_p_region->header[2].valid = 0;
		dev->is_p_region->header[3].valid = 0;
		fimc_is_mem_cache_clean((void *)IS_HEADER, IS_PARAM_SIZE);
		break;
	case V4L2_CID_IS_SET_FRAME_VALID:
		if ((dev->scenario_id == ISS_CAPTURE_STILL)
			|| (dev->scenario_id == ISS_CAPTURE_VIDEO)) {
			dev->is_p_region->header[0].valid = ctrl->value;
			dev->is_p_region->header[0].bad_mark = ctrl->value;
			dev->is_p_region->header[0].captured = ctrl->value;
		} else {
			dev->is_p_region->header[dev->frame_count%
				MAX_FRAME_COUNT_PREVIEW].valid = ctrl->value;
			dev->is_p_region->header[dev->frame_count%
			MAX_FRAME_COUNT_PREVIEW].bad_mark = ctrl->value;
			dev->is_p_region->header[dev->frame_count%
			MAX_FRAME_COUNT_PREVIEW].captured = ctrl->value;
		}
		dev->frame_count++;
		fimc_is_mem_cache_clean((void *)IS_HEADER, IS_PARAM_SIZE);
		break;
	case V4L2_CID_IS_SET_FRAME_BADMARK:
		break;
	case V4L2_CID_IS_SET_FRAME_CAPTURED:
		break;
	case V4L2_CID_IS_CLEAR_FRAME_NUMBER:
		if (dev->scenario_id == ISS_CAPTURE_STILL) {
			dev->is_p_region->header[0].valid = 0;
			dev->is_p_region->header[0].bad_mark = 0;
			dev->is_p_region->header[0].captured = 0;
		} else if (dev->scenario_id == ISS_CAPTURE_VIDEO) {
			dev->is_p_region->header[0].valid = 0;
			dev->is_p_region->header[0].bad_mark = 0;
			dev->is_p_region->header[0].captured = 0;
		} else {
			for (i = 0; i < MAX_FRAME_COUNT_PREVIEW; i++) {
				if (dev->is_p_region->header[i].frame_number <
					dev->frame_count) {
					dev->is_p_region->header[i].valid = 0;
					dev->is_p_region->header[i].
						bad_mark = 0;
					dev->is_p_region->header[i].
						captured = 0;
				}
			}
		}
		fimc_is_mem_cache_clean((void *)IS_HEADER, IS_PARAM_SIZE);
		break;
	case V4L2_CID_IS_ISP_DMA_BUFFER_NUM:
		dbg("Debug mode - ISP DMA write\n");
		dev->is_p_region->parameter.isp.dma1_output.cmd
			= DMA_OUTPUT_COMMAND_ENABLE;
		dev->is_p_region->parameter.isp.dma1_output.buffer_number
			= ctrl->value;
		IS_SET_PARAM_BIT(dev, PARAM_ISP_DMA1_OUTPUT);
		IS_INC_PARAM_NUM(dev);
		dev->is_p_region->parameter.isp.dma2_output.cmd
			= DMA_OUTPUT_COMMAND_ENABLE;
		dev->is_p_region->parameter.isp.dma2_output.buffer_number
			= ctrl->value;
		IS_SET_PARAM_BIT(dev, PARAM_ISP_DMA2_OUTPUT);
		IS_INC_PARAM_NUM(dev);
		fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
		fimc_is_hw_set_param(dev);
		break;
	case V4L2_CID_IS_ISP_DMA_BUFFER_ADDRESS:
		dev->is_p_region->parameter.isp.dma1_output.buffer_address
			= ctrl->value;
		fimc_is_mem_cache_clean((void *)IS_HEADER, IS_PARAM_SIZE);
		break;
	case V4L2_CID_CAMERA_SCENE_MODE:
		switch (ctrl->value) {
		case SCENE_MODE_NONE:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_AUTO);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev, ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
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
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_AUTO);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_ENABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AA_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AA_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AA_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case SCENE_MODE_PORTRAIT:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_AUTO);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev, ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, -1);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, -1);
			IS_ISP_SET_PARAM_ADJUST_EXPOSURE(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_HUE(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_ERR(dev,
				ISP_ADJUST_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_AUTO);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_ENABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case SCENE_MODE_LANDSCAPE:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_AUTO);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_MATRIX);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev, ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, 1);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, 1);
			IS_ISP_SET_PARAM_ADJUST_EXPOSURE(dev, 1);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_HUE(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_ERR(dev,
				ISP_ADJUST_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case SCENE_MODE_SPORTS:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 800);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev, ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
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
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case SCENE_MODE_PARTY_INDOOR:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 200);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev, ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_EXPOSURE(dev, 1);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_HUE(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_ERR(dev,
				ISP_ADJUST_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_AUTO);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_ENABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case SCENE_MODE_BEACH_SNOW:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 50);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev, ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 1);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_EXPOSURE(dev, 1);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_HUE(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_ERR(dev,
				ISP_ADJUST_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case SCENE_MODE_SUNSET:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
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
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case SCENE_MODE_DUSK_DAWN:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_FLUORESCENT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
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
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case SCENE_MODE_FALL_COLOR:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev, ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_EXPOSURE(dev, 2);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_HUE(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_ERR(dev,
				ISP_ADJUST_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case SCENE_MODE_NIGHTSHOT:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev, ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
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
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case SCENE_MODE_BACK_LIGHT:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev, ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
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
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		/* FIXME add with SCENE_MODE_BACK_LIGHT (FLASH mode) */
		case SCENE_MODE_FIREWORKS:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 50);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev, ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
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
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case SCENE_MODE_TEXT:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_AUTO);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev, ISP_AWB_COMMAND_AUTO);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
			IS_ISP_SET_PARAM_ADJUST_CONTRAST(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_SATURATION(dev, 2);
			IS_ISP_SET_PARAM_ADJUST_SHARPNESS(dev, 2);
			IS_ISP_SET_PARAM_ADJUST_EXPOSURE(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_BRIGHTNESS(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_HUE(dev, 0);
			IS_ISP_SET_PARAM_ADJUST_ERR(dev,
				ISP_ADJUST_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ADJUST);
			IS_INC_PARAM_NUM(dev);
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_MACRO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		case SCENE_MODE_CANDLE_LIGHT:
			/* ABORT first */
			if (dev->af.af_state != FIMC_IS_AF_IDLE) {
				dev->af.af_state = FIMC_IS_AF_ABORT;
				IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_STOP);
				IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
				IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
				IS_INC_PARAM_NUM(dev);
				fimc_is_mem_cache_clean(
					(void *)dev->is_p_region,
					IS_PARAM_SIZE);
				fimc_is_hw_set_param(dev);
				ret = wait_event_timeout(dev->irq_queue1,
				(dev->af.af_state == FIMC_IS_AF_IDLE),
				FIMC_IS_SHUTDOWN_TIMEOUT);
				if (!ret) {
					dev_err(&dev->pdev->dev,
					"Focus change timeout:%s\n", __func__);
					return -EBUSY;
				}
			}
			/* ISO */
			IS_ISP_SET_PARAM_ISO_CMD(dev,
				ISP_ISO_COMMAND_AUTO);
			IS_ISP_SET_PARAM_ISO_VALUE(dev, 0);
			IS_ISP_SET_PARAM_ISO_ERR(dev, ISP_ISO_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_ISO);
			IS_INC_PARAM_NUM(dev);
			/* Metering */
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
			IS_ISP_SET_PARAM_METERING_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_METERING_ERR(dev,
				ISP_METERING_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_METERING);
			IS_INC_PARAM_NUM(dev);
			/* AWB */
			IS_ISP_SET_PARAM_AWB_CMD(dev,
				ISP_AWB_COMMAND_ILLUMINATION);
			IS_ISP_SET_PARAM_AWB_ILLUMINATION(dev,
				ISP_AWB_ILLUMINATION_DAYLIGHT);
			IS_ISP_SET_PARAM_AWB_ERR(dev, ISP_AWB_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AWB);
			IS_INC_PARAM_NUM(dev);
			/* Adjust */
			IS_ISP_SET_PARAM_ADJUST_CMD(dev,
				ISP_ADJUST_COMMAND_MANUAL);
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
			/* Flash */
			IS_ISP_SET_PARAM_FLASH_CMD(dev,
				ISP_FLASH_COMMAND_DISABLE);
			IS_ISP_SET_PARAM_FLASH_REDEYE(dev,
				ISP_FLASH_REDEYE_DISABLE);
			IS_ISP_SET_PARAM_FLASH_ERR(dev,
				ISP_FLASH_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_FLASH);
			IS_INC_PARAM_NUM(dev);
			/* AF */
			IS_ISP_SET_PARAM_AA_CMD(dev,
					ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev,
					ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_MACRO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
					ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			dev->af.af_state = FIMC_IS_AF_SETCONFIG;
			fimc_is_mem_cache_clean((void *)dev->is_p_region,
			IS_PARAM_SIZE);
			fimc_is_hw_set_param(dev);
			break;
		default:
			break;
		}
		break;
	case V4L2_CID_CAMERA_VT_MODE:
		break;
	case V4L2_CID_CAMERA_VGA_BLUR:
		break;
	default:
		dbg("Invalid control\n");
		return -EINVAL;
	}

	return ret;
}

static int fimc_is_g_ext_ctrls_handler(struct fimc_is_dev *dev,
	struct v4l2_ext_control *ctrl, int index)
{
	int ret = 0;
	u32 tmp = 0;
	switch (ctrl->id) {
	/* Face Detection CID handler */
	/* 1. Overall information */
	case V4L2_CID_IS_FD_GET_FACE_COUNT:
		ctrl->value = dev->fd_header.count;
		break;
	case V4L2_CID_IS_FD_GET_FACE_FRAME_NUMBER:
		if (dev->fd_header.offset < dev->fd_header.count) {
			ctrl->value =
				dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].frame_number;
		} else {
			ctrl->value = 0;
			return -255;
		}
		break;
	case V4L2_CID_IS_FD_GET_FACE_CONFIDENCE:
		ctrl->value = dev->is_p_region->face[dev->fd_header.index
					+ dev->fd_header.offset].confidence;
		break;
	case V4L2_CID_IS_FD_GET_FACE_SMILE_LEVEL:
		ctrl->value = dev->is_p_region->face[dev->fd_header.index
					+ dev->fd_header.offset].smile_level;
		break;
	case V4L2_CID_IS_FD_GET_FACE_BLINK_LEVEL:
		ctrl->value = dev->is_p_region->face[dev->fd_header.index
					+ dev->fd_header.offset].blink_level;
		break;
	/* 2. Face information */
	case V4L2_CID_IS_FD_GET_FACE_TOPLEFT_X:
		tmp = dev->is_p_region->face[dev->fd_header.index
					+ dev->fd_header.offset].face.offset_x;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.width;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	case V4L2_CID_IS_FD_GET_FACE_TOPLEFT_Y:
		tmp = dev->is_p_region->face[dev->fd_header.index
					+ dev->fd_header.offset].face.offset_y;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.height;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	case V4L2_CID_IS_FD_GET_FACE_BOTTOMRIGHT_X:
		tmp = dev->is_p_region->face[dev->fd_header.index
					+ dev->fd_header.offset].face.offset_x
			+ dev->is_p_region->face[dev->fd_header.index
					+ dev->fd_header.offset].face.width;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.width;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	case V4L2_CID_IS_FD_GET_FACE_BOTTOMRIGHT_Y:
		tmp = dev->is_p_region->face[dev->fd_header.index
					+ dev->fd_header.offset].face.offset_y
			+ dev->is_p_region->face[dev->fd_header.index
					+ dev->fd_header.offset].face.height;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.height;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	/* 3. Left eye information */
	case V4L2_CID_IS_FD_GET_LEFT_EYE_TOPLEFT_X:
		tmp = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].left_eye.offset_x;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.width;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	case V4L2_CID_IS_FD_GET_LEFT_EYE_TOPLEFT_Y:
		tmp = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].left_eye.offset_y;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.height;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	case V4L2_CID_IS_FD_GET_LEFT_EYE_BOTTOMRIGHT_X:
		tmp = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].left_eye.offset_x
			+ dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].left_eye.width;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.width;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	case V4L2_CID_IS_FD_GET_LEFT_EYE_BOTTOMRIGHT_Y:
		tmp = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].left_eye.offset_y
			+ dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].left_eye.height;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.height;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	/* 4. Right eye information */
	case V4L2_CID_IS_FD_GET_RIGHT_EYE_TOPLEFT_X:
		tmp = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].right_eye.offset_x;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.width;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	case V4L2_CID_IS_FD_GET_RIGHT_EYE_TOPLEFT_Y:
		tmp = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].right_eye.offset_y;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.height;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	case V4L2_CID_IS_FD_GET_RIGHT_EYE_BOTTOMRIGHT_X:
		tmp = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].right_eye.offset_x
			+ dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].right_eye.width;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.width;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	case V4L2_CID_IS_FD_GET_RIGHT_EYE_BOTTOMRIGHT_Y:
		tmp = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].right_eye.offset_y
			+ dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].right_eye.height;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.height;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	/* 5. Mouth eye information */
	case V4L2_CID_IS_FD_GET_MOUTH_TOPLEFT_X:
		tmp = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].mouth.offset_x;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.width;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	case V4L2_CID_IS_FD_GET_MOUTH_TOPLEFT_Y:
		tmp = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].mouth.offset_y;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.height;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	case V4L2_CID_IS_FD_GET_MOUTH_BOTTOMRIGHT_X:
		tmp = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].mouth.offset_x
			+ dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].mouth.width;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.width;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	case V4L2_CID_IS_FD_GET_MOUTH_BOTTOMRIGHT_Y:
		tmp = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].mouth.offset_y
			+ dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].mouth.height;
		tmp = (tmp * 2 * GED_FD_RANGE) /  dev->fd_header.height;
		ctrl->value = (s32)tmp - GED_FD_RANGE;
		break;
	/* 6. Angle information */
	case V4L2_CID_IS_FD_GET_ANGLE:
		ctrl->value = dev->is_p_region->face[dev->fd_header.index
				+ dev->fd_header.offset].roll_angle;
		break;
	/* 7. Update next face information */
	case V4L2_CID_IS_FD_GET_NEXT:
		dev->fd_header.offset++;
		break;
	default:
		return 255;
		break;
	}
	return ret;
}

static int fimc_is_g_ext_ctrls(struct v4l2_subdev *sd,
	struct v4l2_ext_controls *ctrls)
{
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);
	struct v4l2_ext_control *ctrl;
	int i, ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&dev->slock, flags);
	ctrl = ctrls->controls;
	if (!ctrls->ctrl_class == V4L2_CTRL_CLASS_CAMERA)
		return -EINVAL;

	fimc_is_mem_cache_inv((void *)IS_FACE,
		(unsigned long)(sizeof(struct is_face_marker)*MAX_FACE_COUNT));

	dev->fd_header.offset = 0;
	/* get width and height at the current scenario */
	switch (dev->scenario_id) {
	case ISS_PREVIEW_STILL:
		dev->fd_header.width = (s32)dev->sensor.width_prev;
		dev->fd_header.height = (s32)dev->sensor.height_prev;
		break;
	case ISS_PREVIEW_VIDEO:
		dev->fd_header.width = (s32)dev->sensor.width_prev_cam;
		dev->fd_header.height = (s32)dev->sensor.height_prev_cam;
		break;
	case ISS_CAPTURE_STILL:
		dev->fd_header.width = (s32)dev->sensor.width_cap;
		dev->fd_header.height = (s32)dev->sensor.height_cap;
		break;
	case ISS_CAPTURE_VIDEO:
		dev->fd_header.width = (s32)dev->sensor.width_cam;
		dev->fd_header.height = (s32)dev->sensor.height_cam;
		break;
	}
	for (i = 0; i < ctrls->count; i++) {
		ctrl = ctrls->controls + i;
		ret = fimc_is_g_ext_ctrls_handler(dev, ctrl, i);
		if (ret > 0) {
			ctrls->error_idx = i;
			break;
		} else if (ret < 0) {
			ret = 0;
			break;
		}
	}

	dev->fd_header.index = 0;
	dev->fd_header.count = 0;
	spin_unlock_irqrestore(&dev->slock, flags);
	return ret;
}

static int fimc_is_s_ext_ctrls_handler(struct fimc_is_dev *dev,
	struct v4l2_ext_control *ctrl)
{
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
	case V4L2_CID_CAMERA_FACE_DETECTION:
		switch (ctrl->value) {
		case FACE_DETECTION_OFF:
			IS_FD_SET_PARAM_CONTROL_CMD(dev, CONTROL_COMMAND_STOP);
			break;
		case FACE_DETECTION_ON:
			IS_FD_SET_PARAM_CONTROL_CMD(dev,
				CONTROL_COMMAND_START);
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
		switch (dev->scenario_id) {
		case ISS_PREVIEW_STILL:
			dev->sensor.width_prev = ctrl->value;
			break;
		case ISS_PREVIEW_VIDEO:
			dev->sensor.width_prev_cam = ctrl->value;
			break;
		case ISS_CAPTURE_STILL:
			dev->sensor.width_cap = ctrl->value;
			break;
		case ISS_CAPTURE_VIDEO:
			dev->sensor.width_cam = ctrl->value;
			break;
		}
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
		switch (dev->scenario_id) {
		case ISS_PREVIEW_STILL:
			dev->sensor.height_prev = ctrl->value;
			break;
		case ISS_PREVIEW_VIDEO:
			dev->sensor.height_prev_cam = ctrl->value;
			break;
		case ISS_CAPTURE_STILL:
			dev->sensor.height_cap = ctrl->value;
			break;
		case ISS_CAPTURE_VIDEO:
			dev->sensor.height_cam = ctrl->value;
			break;
		}
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
			IS_ISP_SET_PARAM_AA_CMD(dev,
				ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev, ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_AUTO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AA_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AA_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AA_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_FOCUS_MODE_MACRO:
			IS_ISP_SET_PARAM_AA_CMD(dev,
				ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev, ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_MACRO);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AA_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AA_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AA_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_FOCUS_MODE_INFINITY:
			IS_ISP_SET_PARAM_AA_CMD(dev,
				ISP_AA_COMMAND_START);
			IS_ISP_SET_PARAM_AA_TARGET(dev, ISP_AA_TARGET_AF);
			IS_ISP_SET_PARAM_AA_MODE(dev, ISP_AF_MODE_INFINITY);
			IS_ISP_SET_PARAM_AA_FACE(dev, ISP_AF_FACE_DISABLE);
			IS_ISP_SET_PARAM_AA_CONTINUOUS(dev,
				ISP_AF_CONTINUOUS_DISABLE);
			IS_ISP_SET_PARAM_AA_WIN_POS_X(dev, 0);
			IS_ISP_SET_PARAM_AA_WIN_POS_Y(dev, 0);
			IS_ISP_SET_PARAM_AA_WIN_WIDTH(dev, 0);
			IS_ISP_SET_PARAM_AA_WIN_HEIGHT(dev, 0);
			IS_ISP_SET_PARAM_AA_ERR(dev, ISP_AF_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_AA);
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
		case IS_IMAGE_EFFECT_MONOCHROME:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_MONOCHROME);
			IS_ISP_SET_PARAM_EFFECT_ERR(dev,
				ISP_IMAGE_EFFECT_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_IMAGE_EFFECT);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_IMAGE_EFFECT_NEGATIVE_MONO:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_NEGATIVE_MONO);
			IS_ISP_SET_PARAM_EFFECT_ERR(dev,
				ISP_IMAGE_EFFECT_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_IMAGE_EFFECT);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_IMAGE_EFFECT_NEGATIVE_COLOR:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_NEGATIVE_COLOR);
			IS_ISP_SET_PARAM_EFFECT_ERR(dev,
				ISP_IMAGE_EFFECT_ERROR_NO);
			IS_SET_PARAM_BIT(dev, PARAM_ISP_IMAGE_EFFECT);
			IS_INC_PARAM_NUM(dev);
			break;
		case IS_IMAGE_EFFECT_SEPIA:
			IS_ISP_SET_PARAM_EFFECT_CMD(dev,
				ISP_IMAGE_EFFECT_SEPIA);
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
		case IS_METERING_AVERAGE:
			IS_ISP_SET_PARAM_METERING_CMD(dev,
				ISP_METERING_COMMAND_AVERAGE);
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
				dev->sensor.width_prev/2);
			IS_ISP_SET_PARAM_METERING_WIN_POS_Y(dev,
				dev->sensor.height_prev/2);
			IS_ISP_SET_PARAM_METERING_WIN_WIDTH(dev,
				dev->sensor.width_prev);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev,
				dev->sensor.height_prev);
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
				dev->sensor.width_prev);
			IS_ISP_SET_PARAM_METERING_WIN_HEIGHT(dev,
				dev->sensor.height_prev);
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
	case V4L2_CID_IS_FD_SET_MAX_FACE_NUMBER:
		IS_FD_SET_PARAM_FD_CONFIG_CMD(dev,
			FD_CONFIG_COMMAND_MAXIMUM_NUMBER);
		IS_FD_SET_PARAM_FD_CONFIG_MAX_NUMBER(dev, ctrl->value);
		IS_FD_SET_PARAM_FD_CONFIG_ERR(dev, ERROR_FD_NO);
		IS_SET_PARAM_BIT(dev, PARAM_FD_CONFIG);
		IS_INC_PARAM_NUM(dev);
		break;
	default:
		dbg("Invalid control\n");
		return -EINVAL;
	}
	return 0;
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
	fimc_is_mem_cache_clean((void *)dev->is_p_region, IS_PARAM_SIZE);
	fimc_is_hw_set_param(dev);
	return ret;
}

/* v4l2 subdev video operations
*/
static int fimc_is_try_mbus_fmt(struct v4l2_subdev *sd,
struct v4l2_mbus_framefmt *mf)
{
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);
	dbg("fimc_is_try_mbus_fmt - %d, %d\n", mf->width, mf->height);
	switch (dev->scenario_id) {
	case ISS_PREVIEW_STILL:
		dev->sensor.width_prev = mf->width;
		dev->sensor.height_prev = mf->height;
		break;
	case ISS_PREVIEW_VIDEO:
		dev->sensor.width_prev_cam = mf->width;
		dev->sensor.height_prev_cam = mf->height;
		break;
	case ISS_CAPTURE_STILL:
		dev->sensor.width_cap = mf->width;
		dev->sensor.height_cap = mf->height;
		break;
	case ISS_CAPTURE_VIDEO:
		dev->sensor.width_cam = mf->width;
		dev->sensor.height_cam = mf->height;
		break;
	}
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
	int ret = 0;

	dbg("fimc_is_s_mbus_fmt- %d,%d", mf->width, mf->height);

	/* scenario ID setting */
	switch (mf->field) {
	case 0:
		dev->scenario_id = ISS_PREVIEW_STILL;
		dev->sensor.width_prev = mf->width;
		dev->sensor.height_prev = mf->height;
#ifdef FIXED_60_FPS
		IS_SENSOR_SET_FRAME_RATE(dev, 60);
		IS_SET_PARAM_BIT(dev, PARAM_SENSOR_FRAME_RATE);
		IS_INC_PARAM_NUM(dev);
		IS_ISP_SET_PARAM_OTF_INPUT_RESERVED3(dev, 0);
		IS_ISP_SET_PARAM_OTF_INPUT_RESERVED4(dev, 16666);
#endif
		break;
	case 1:
		dev->scenario_id = ISS_PREVIEW_VIDEO;
		dev->sensor.width_prev_cam = mf->width;
		dev->sensor.height_prev_cam = mf->height;
		break;
	case 2:
		dev->scenario_id = ISS_CAPTURE_STILL;
		dev->sensor.width_cap = mf->width;
		dev->sensor.height_cap = mf->height;
#ifdef FIXED_60_FPS
		IS_SENSOR_SET_FRAME_RATE(dev, 15);
		IS_SET_PARAM_BIT(dev, PARAM_SENSOR_FRAME_RATE);
		IS_INC_PARAM_NUM(dev);
		IS_ISP_SET_PARAM_OTF_INPUT_RESERVED3(dev, 0);
		IS_ISP_SET_PARAM_OTF_INPUT_RESERVED4(dev, 66666);
#endif
		break;
	case 3:
		dev->scenario_id = ISS_CAPTURE_VIDEO;
		dev->sensor.width_cam = mf->width;
		dev->sensor.height_cam = mf->height;
		break;
	}

	/* 1. ISP input / output*/
	IS_ISP_SET_PARAM_OTF_INPUT_WIDTH(dev, mf->width);
	IS_ISP_SET_PARAM_OTF_INPUT_HEIGHT(dev, mf->height);
	IS_SET_PARAM_BIT(dev, PARAM_ISP_OTF_INPUT);
	IS_INC_PARAM_NUM(dev);
	IS_ISP_SET_PARAM_OTF_OUTPUT_WIDTH(dev, mf->width);
	IS_ISP_SET_PARAM_OTF_OUTPUT_HEIGHT(dev, mf->height);
	IS_SET_PARAM_BIT(dev, PARAM_ISP_OTF_OUTPUT);
	IS_INC_PARAM_NUM(dev);
	IS_ISP_SET_PARAM_DMA_OUTPUT1_WIDTH(dev, mf->width);
	IS_ISP_SET_PARAM_DMA_OUTPUT1_HEIGHT(dev, mf->height);
	IS_SET_PARAM_BIT(dev, PARAM_ISP_DMA1_OUTPUT);
	IS_INC_PARAM_NUM(dev);
	IS_ISP_SET_PARAM_DMA_OUTPUT2_WIDTH(dev, mf->width);
	IS_ISP_SET_PARAM_DMA_OUTPUT2_HEIGHT(dev, mf->height);
	IS_SET_PARAM_BIT(dev, PARAM_ISP_DMA2_OUTPUT);
	IS_INC_PARAM_NUM(dev);
	/* 2. DRC input / output*/
	IS_DRC_SET_PARAM_OTF_INPUT_WIDTH(dev, mf->width);
	IS_DRC_SET_PARAM_OTF_INPUT_HEIGHT(dev, mf->height);
	IS_SET_PARAM_BIT(dev, PARAM_DRC_OTF_INPUT);
	IS_INC_PARAM_NUM(dev);
	IS_DRC_SET_PARAM_OTF_OUTPUT_WIDTH(dev, mf->width);
	IS_DRC_SET_PARAM_OTF_OUTPUT_HEIGHT(dev, mf->height);
	IS_SET_PARAM_BIT(dev, PARAM_DRC_OTF_OUTPUT);
	IS_INC_PARAM_NUM(dev);
	/* 3. FD input / output*/
	IS_FD_SET_PARAM_OTF_INPUT_WIDTH(dev, mf->width);
	IS_FD_SET_PARAM_OTF_INPUT_HEIGHT(dev, mf->height);
	IS_SET_PARAM_BIT(dev, PARAM_FD_OTF_INPUT);
	IS_INC_PARAM_NUM(dev);

	fimc_is_mem_cache_clean((void *)dev->is_p_region, IS_PARAM_SIZE);
	clear_bit(IS_ST_RUN, &dev->state);
	fimc_is_hw_set_param(dev);
	/* Below sequence is for preventing system hang
		due to size mis-match */
	ret = wait_event_timeout(dev->irq_queue1,
		test_bit(IS_ST_RUN, &dev->state),
		FIMC_IS_SHUTDOWN_TIMEOUT);
	if (!ret) {
		dev_err(&dev->pdev->dev, "wait timeout : %s\n", __func__);
		return -EBUSY;
	}
	set_bit(IS_ST_RUN, &dev->state);
	return 0;
}

static int fimc_is_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct fimc_is_dev *dev = to_fimc_is_dev(sd);

	if (enable) {
		if (test_bit(IS_ST_RUN, &dev->state) &&
				!test_bit(IS_ST_STREAM_ON, &dev->state)) {
			dbg("IS Stream On\n");
			clear_bit(IS_ST_RUN, &dev->state);
			fimc_is_hw_set_stream(dev, enable);
			ret = wait_event_timeout(dev->irq_queue1,
			test_bit(IS_ST_STREAM_ON, &dev->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&dev->pdev->dev,
					"wait timeout : %s\n", __func__);
				return -EBUSY;
			}
		} else {
			dev_err(&dev->pdev->dev, "ON : not stream-on condition\n");
			return -EINVAL;
		}
	} else {
		dbg("IS Stream Off\n");
		if (test_bit(IS_ST_STREAM_ON, &dev->state) &&
				!test_bit(IS_ST_STREAM_OFF, &dev->state)) {
			clear_bit(IS_ST_STREAM_ON, &dev->state);
			fimc_is_hw_set_stream(dev, enable);
			ret = wait_event_timeout(dev->irq_queue1,
				test_bit(IS_ST_STREAM_OFF, &dev->state),
						FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&dev->pdev->dev,
					"wait timeout : %s\n", __func__);
				return -EBUSY;
			}
			clear_bit(IS_ST_STREAM_OFF, &dev->state);
		} else {
			dev_err(&dev->pdev->dev, "OFF : not stream-on condition\n");
			return -EINVAL;
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
