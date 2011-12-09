/* linux/drivers/media/video/exynos/gsc/gsc-core.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung EXYNOS5 SoC series G-scaler driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <media/v4l2-ioctl.h>

#include "gsc-core.h"
#define GSC_CLOCK_GATE_NAME		"gscl"

static struct gsc_fmt gsc_formats[] = {
	{
		.name		= "RGB565",
		.pixelformat	= V4L2_PIX_FMT_RGB565X,
		.depth		= { 16 },
		.color		= GSC_RGB,
		.num_planes	= 1,
		.nr_comp	= 1,
	}, {
		.name		= "XRGB-8-8-8-8, 32 bpp",
		.pixelformat	= V4L2_PIX_FMT_RGB32,
		.depth		= { 32 },
		.color		= GSC_RGB,
		.num_planes	= 1,
		.nr_comp	= 1,
		.mbus_code	= V4L2_MBUS_FMT_XRGB8888_4X8_LE,
	}, {
		.name		= "YUV 4:2:2 packed, YCbYCr",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.depth		= { 16 },
		.color		= GSC_YUV422,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CBCR,
		.num_planes	= 1,
		.nr_comp	= 1,
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,
	}, {
		.name		= "YUV 4:2:2 packed, CbYCrY",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
		.depth		= { 16 },
		.color		= GSC_YUV422,
		.yorder		= GSC_LSB_C,
		.corder		= GSC_CBCR,
		.num_planes	= 1,
		.nr_comp	= 1,
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,
	}, {
		.name		= "YUV 4:2:2 packed, CrYCbY",
		.pixelformat	= V4L2_PIX_FMT_VYUY,
		.depth		= { 16 },
		.color		= GSC_YUV422,
		.yorder		= GSC_LSB_C,
		.corder		= GSC_CRCB,
		.num_planes	= 1,
		.nr_comp	= 1,
		.mbus_code	= V4L2_MBUS_FMT_VYUY8_2X8,
	}, {
		.name		= "YUV 4:2:2 packed, YCrYCb",
		.pixelformat	= V4L2_PIX_FMT_YVYU,
		.depth		= { 16 },
		.color		= GSC_YUV422,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CRCB,
		.num_planes	= 1,
		.nr_comp	= 1,
		.mbus_code	= V4L2_MBUS_FMT_YVYU8_2X8,
	}, {
		.name		= "YUV 4:4:4 planar, YCbYCr",
		.pixelformat	= V4L2_PIX_FMT_YUV32,
		.depth		= { 32 },
		.color		= GSC_YUV444,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CBCR,
		.num_planes	= 1,
		.nr_comp	= 1,
		.mbus_code	= V4L2_MBUS_FMT_YUV8_1X24,
	}, {
		.name		= "YUV 4:2:2 planar, Y/Cb/Cr",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.depth		= { 16 },
		.color		= GSC_YUV422,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CBCR,
		.num_planes	= 1,
		.nr_comp	= 3,
	}, {
		.name		= "YUV 4:2:2 planar, Y/CbCr",
		.pixelformat	= V4L2_PIX_FMT_NV16,
		.depth		= { 16 },
		.color		= GSC_YUV422,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CBCR,
		.num_planes	= 1,
		.nr_comp	= 2,
	}, {
		.name		= "YUV 4:2:2 planar, Y/CrCb",
		.pixelformat	= V4L2_PIX_FMT_NV61,
		.depth		= { 16 },
		.color		= GSC_YUV422,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CRCB,
		.num_planes	= 1,
		.nr_comp	= 2,
	}, {
		.name		= "YUV 4:2:0 planar, YCbCr",
		.pixelformat	= V4L2_PIX_FMT_YUV420,
		.depth		= { 12 },
		.color		= GSC_YUV420,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CBCR,
		.num_planes	= 1,
		.nr_comp	= 3,
	},{
		.name		= "YUV 4:2:0 planar, YCbCr",
		.pixelformat	= V4L2_PIX_FMT_YVU420,
		.depth		= { 12 },
		.color		= GSC_YUV420,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CBCR,
		.num_planes	= 1,
		.nr_comp	= 3,

	}, {
		.name		= "YUV 4:2:0 planar, Y/CbCr",
		.pixelformat	= V4L2_PIX_FMT_NV12,
		.depth		= { 12 },
		.color		= GSC_YUV420,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CBCR,
		.num_planes	= 1,
		.nr_comp	= 2,
	}, {
		.name		= "YUV 4:2:0 planar, Y/CrCb",
		.pixelformat	= V4L2_PIX_FMT_NV21,
		.depth		= { 12 },
		.color		= GSC_YUV420,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CRCB,
		.num_planes	= 1,
		.nr_comp	= 2,
	}, {
		.name		= "YUV 4:2:0 non-contiguous 2-planar, Y/CbCr",
		.pixelformat	= V4L2_PIX_FMT_NV12M,
		.depth		= { 8, 4 },
		.color		= GSC_YUV420,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CBCR,
		.num_planes	= 2,
		.nr_comp	= 2,
	}, {
		.name		= "YUV 4:2:0 non-contiguous 3-planar, Y/Cb/Cr",
		.pixelformat	= V4L2_PIX_FMT_YUV420M,
		.depth		= { 8, 2, 2 },
		.color		= GSC_YUV420,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CBCR,
		.num_planes	= 3,
		.nr_comp	= 3,
	}, {
		.name		= "YUV 4:2:0 non-contiguous 3-planar, Y/Cr/Cb",
		.pixelformat	= V4L2_PIX_FMT_YVU420M,
		.depth		= { 8, 2, 2 },
		.color		= GSC_YUV420,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CRCB,
		.num_planes	= 3,
		.nr_comp	= 3,
	}, {
		.name		= "YUV 4:2:0 non-contiguous 2-planar, Y/CbCr, tiled",
		.pixelformat	= V4L2_PIX_FMT_NV12MT_16X16,
		.depth		= { 8, 4 },
		.color		= GSC_YUV420,
		.yorder		= GSC_LSB_Y,
		.corder		= GSC_CBCR,
		.num_planes	= 2,
		.nr_comp	= 2,
	},
};

struct gsc_fmt *get_format(int index)
{
	return &gsc_formats[index];
}

struct gsc_fmt *find_format(u32 *pixelformat, u32 *mbus_code, int index)
{
	struct gsc_fmt *fmt, *def_fmt = NULL;
	unsigned int i;

	if (index >= ARRAY_SIZE(gsc_formats))
		return NULL;

	for (i = 0; i < ARRAY_SIZE(gsc_formats); ++i) {
		fmt = get_format(i);
		if (pixelformat && fmt->pixelformat == *pixelformat)
			return fmt;
		if (mbus_code && fmt->mbus_code == *mbus_code)
			return fmt;
		if (index == i)
			def_fmt = fmt;
	}
	return def_fmt;

}

void gsc_set_frame_size(struct gsc_frame *frame, int width, int height)
{
	frame->f_width	= width;
	frame->f_height	= height;
	frame->crop.width = width;
	frame->crop.height = height;
	frame->crop.left = 0;
	frame->crop.top = 0;
}

int gsc_check_scale_size(struct gsc_ctx *ctx)
{
	struct gsc_pix_max *pix_max = ctx->gsc_dev->variant->pix_max;
	struct gsc_pix_min *pix_min = ctx->gsc_dev->variant->pix_min;

	if (ctx->s_frame.f_width < pix_min->org_w ||
	    ctx->s_frame.f_height < pix_min->org_h ||
	    ctx->d_frame.f_width < pix_min->org_w ||
	    ctx->d_frame.f_height < pix_min->org_h) {
		gsc_err("Original image size is under minimun size");
		return -EINVAL;
	}

	if (ctx->s_frame.f_width > pix_max->org_scaler_input_w ||
	    ctx->s_frame.f_height > pix_max->org_scaler_input_h ||
	    ctx->d_frame.f_width > pix_max->target_w ||
	    ctx->d_frame.f_height > pix_max->target_h) {
		gsc_err("Orginal image size exceeds the maximun size");
		return -EINVAL;
	}

	return 0;
}

int gsc_cal_prescaler_ratio(struct gsc_variant *var, u32 src, u32 dst, u32 *ratio)
{
	if ((dst > src) || (dst >= src / var->poly_sc_down_max)) {
		*ratio = 1;
		return 0;
	}

	if ((src / var->poly_sc_down_max / var->pre_sc_down_max) > dst) {
		gsc_err("scale ratio exceeded maximun scale down ratio(1/16)");
		return -EINVAL;
	}

	*ratio = (dst > (src / 8)) ? 2 : 4;

	return 0;
}

void gsc_get_prescaler_shfactor(u32 hratio, u32 vratio, u32 *sh)
{
	if (hratio == 4 && vratio == 4)
		*sh = 4;
	else if ((hratio == 4 && vratio == 2) ||
		 (hratio == 2 && vratio == 4))
		*sh = 3;
	else if ((hratio == 4 && vratio == 1) ||
		 (hratio == 1 && vratio == 4) ||
		 (hratio == 2 && vratio == 2))
		*sh = 2;
	else if (hratio == 1 && vratio == 1)
		*sh = 0;
	else
		*sh = 1;
}

int gsc_enum_fmt_mplane(struct v4l2_fmtdesc *f)
{
	struct gsc_fmt *fmt;

	fmt = find_format(NULL, NULL, f->index);
	if (!fmt)
		return -EINVAL;

	strncpy(f->description, fmt->name, sizeof(f->description) - 1);
	f->pixelformat = fmt->pixelformat;

	return 0;
}

u32 get_plane_size(struct gsc_frame *frame, unsigned int plane)
{
	if (!frame || plane >= frame->fmt->num_planes) {
		gsc_err("Invalid argument");
		return 0;
	}

	return frame->payload[plane];
}

int gsc_try_fmt_mplane(struct gsc_dev *gsc, struct v4l2_format *f)
{
	struct gsc_variant *variant = gsc->variant;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct gsc_fmt *fmt;
	u32 max_w, max_h, mod_x, mod_y;
	u32 min_w, min_h, tmp_w, tmp_h;
	int i;

	gsc_dbg("user put w: %d, h: %d", pix_mp->width, pix_mp->height);

	fmt = find_format(&pix_mp->pixelformat, NULL, 0);
	if (!fmt) {
		gsc_err("pixelformat format (0x%X) invalid\n", pix_mp->pixelformat);
		return -EINVAL;
	}

	if (pix_mp->field == V4L2_FIELD_ANY)
		pix_mp->field = V4L2_FIELD_NONE;
	else if (pix_mp->field != V4L2_FIELD_NONE) {
		gsc_err("Not supported field order(%d)\n", pix_mp->field);
		return -EINVAL;
	}

	max_w = variant->pix_max->target_w;
	max_h = variant->pix_max->target_h;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		mod_x = ffs(variant->pix_align->org_w) - 1;
		if (is_yuv420(fmt->color))
			mod_y = ffs(variant->pix_align->org_h) - 1;
		else
			mod_y = ffs(variant->pix_align->org_h) - 2;
		min_w = variant->pix_min->org_w;
		min_h = variant->pix_min->org_h;
	} else {
		mod_x = ffs(variant->pix_align->org_w) - 1;
		if (is_yuv420(fmt->color))
			mod_y = ffs(variant->pix_align->org_h) - 1;
		else
			mod_y = ffs(variant->pix_align->org_h) - 2;
		min_w = variant->pix_min->target_w;
		min_h = variant->pix_min->target_h;
	}
	gsc_dbg("mod_x: %d, mod_y: %d, max_w: %d, max_h = %d",
	     mod_x, mod_y, max_w, max_h);
	/* To check if image size is modified to adjust parameter against
	   hardware abilities */
	tmp_w = pix_mp->width;
	tmp_h = pix_mp->height;

	v4l_bound_align_image(&pix_mp->width, min_w, max_w, mod_x,
		&pix_mp->height, min_h, max_h, mod_y, 0);
	if (tmp_w != pix_mp->width || tmp_h != pix_mp->height)
		gsc_info("Image size has been modified from %dx%d to %dx%d",
			 tmp_w, tmp_h, pix_mp->width, pix_mp->height);

	pix_mp->num_planes = fmt->num_planes;
	pix_mp->colorspace = V4L2_COLORSPACE_JPEG;

	for (i = 0; i < pix_mp->num_planes; ++i) {
		int bpl = (pix_mp->width * fmt->depth[i]) >> 3;
		pix_mp->plane_fmt[i].bytesperline = bpl;
		pix_mp->plane_fmt[i].sizeimage = bpl * pix_mp->height;

		gsc_dbg("[%d]: bpl: %d, sizeimage: %d",
		    i, bpl, pix_mp->plane_fmt[i].sizeimage);
	}

	return 0;
}

int gsc_g_fmt_mplane(struct gsc_ctx *ctx, struct v4l2_format *f)
{
	struct gsc_frame *frame;
	struct v4l2_pix_format_mplane *pix_mp;
	int i;

	frame = ctx_get_frame(ctx, f->type);
	if (IS_ERR(frame))
		return PTR_ERR(frame);

	pix_mp = &f->fmt.pix_mp;

	pix_mp->width		= frame->f_width;
	pix_mp->height		= frame->f_height;
	pix_mp->field		= V4L2_FIELD_NONE;
	pix_mp->pixelformat	= frame->fmt->pixelformat;
	pix_mp->colorspace	= V4L2_COLORSPACE_JPEG;
	pix_mp->num_planes	= frame->fmt->num_planes;

	for (i = 0; i < pix_mp->num_planes; ++i) {
		pix_mp->plane_fmt[i].bytesperline = (frame->f_width *
			frame->fmt->depth[i]) / 8;
		pix_mp->plane_fmt[i].sizeimage = pix_mp->plane_fmt[i].bytesperline *
			frame->f_height;
	}

	return 0;
}

int gsc_g_crop(struct gsc_ctx *ctx, struct v4l2_crop *cr)
{
	struct gsc_frame *frame;

	frame = ctx_get_frame(ctx, cr->type);
	if (IS_ERR(frame))
		return PTR_ERR(frame);

	memcpy(&cr->c, &frame->crop, sizeof(struct v4l2_rect));

	return 0;
}

int gsc_try_crop(struct gsc_ctx *ctx, struct v4l2_crop *cr)
{
	struct gsc_frame *f;
	struct gsc_dev *gsc = ctx->gsc_dev;
	struct gsc_variant *variant = gsc->variant;
	u32 mod_x = 0, mod_y = 0, tmp_w, tmp_h;
	u32 min_w, min_h, max_w, max_h;

	if (cr->c.top < 0 || cr->c.left < 0) {
		gsc_err("doesn't support negative values for top & left\n");
		return -EINVAL;
	}
	gsc_dbg("user put w: %d, h: %d", cr->c.width, cr->c.height);

	if (cr->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		f = &ctx->d_frame;
	else if (cr->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		f = &ctx->s_frame;
	else
		return -EINVAL;

	max_w = f->f_width;
	max_h = f->f_height;

	if (V4L2_TYPE_IS_OUTPUT(cr->type)) {
		if ((is_yuv422(f->fmt->color) && f->fmt->nr_comp == 1) ||
		    is_rgb(f->fmt->color))
			min_w = 32;
		else
			min_w = 64;
		if ((is_yuv422(f->fmt->color) && f->fmt->nr_comp == 3) ||
		    is_yuv420(f->fmt->color))
			min_h = 32;
		else
			min_h = 16;
	} else {
		if (is_yuv420(f->fmt->color) || is_yuv422(f->fmt->color))
			mod_x = ffs(variant->pix_align->target_w) - 1;
		if (is_yuv420(f->fmt->color))
			mod_y = ffs(variant->pix_align->target_h) - 1;
		min_w = variant->pix_min->target_w;
		min_h = variant->pix_min->target_h;
		if (ctx->ctrl_val.rot == 90 || ctx->ctrl_val.rot == 270) {
			max_w = f->f_height;
			max_h = f->f_width;
		}
	}
	gsc_dbg("mod_x: %d, mod_y: %d, min_w: %d, min_h = %d",
	     mod_x, mod_y, min_w, min_h);
	/* To check if croppd image size is modified to adjust parameter\
	   against hardware abilities */
	tmp_w = cr->c.width;
	tmp_h = cr->c.height;

	v4l_bound_align_image(&cr->c.width, min_w, max_w, mod_x,
			      &cr->c.height, min_h, max_h, mod_y, 0);
	if (tmp_w != cr->c.width || tmp_h != cr->c.height)
		gsc_info("Image cropped size has been modified from %dx%d to %dx%d",
			 tmp_w, tmp_h, cr->c.width, cr->c.height);

	/* adjust left/top if cropping rectangle is out of bounds */
	/* Need to add code to algin left value with 2's multiple */
	if (cr->c.left + cr->c.width > max_w)
		cr->c.left = max_w - cr->c.width;
	if (cr->c.top + cr->c.height > max_h)
		cr->c.top = max_h - cr->c.height;

	if (is_yuv420(f->fmt->color) || is_yuv422(f->fmt->color))
		if (cr->c.left % 2)
			cr->c.left -= 1;

	gsc_dbg("Aligned l:%d, t:%d, w:%d, h:%d, f_w: %d, f_h: %d",
	    cr->c.left, cr->c.top, cr->c.width, cr->c.height, max_w, max_h);

	return 0;
}

int gsc_check_scaler_ratio(struct gsc_variant *var, int sw, int sh, int dw, int dh)
{
	if ((sw / dw) > var->sc_down_max ||
	    (sh / dh) > var->sc_down_max ||
	    (dw / sw) > var->sc_up_max ||
	    (dh / sh) > var->sc_up_max)
		return -EINVAL;

	return 0;
}

int gsc_pipeline_s_stream(struct gsc_dev *gsc, int on)
{
	struct gsc_pipeline *p = &gsc->pipeline;
	int ret = 0;

	ret = v4l2_subdev_call(p->disp, video, s_stream, on);
	if (ret)
		gsc_err("Display s_stream on failed\n");

	return ret;
}

int gsc_out_link_validate(const struct media_pad *source,
			  const struct media_pad *sink)
{
	struct v4l2_subdev_format src_fmt;
	struct v4l2_subdev_crop dst_crop;
	struct v4l2_subdev *sd;
	struct gsc_dev *gsc;
	struct gsc_frame *f;
	int dst_width, dst_height;
	int ret;

	if (media_entity_type(source->entity) != MEDIA_ENT_T_V4L2_SUBDEV ||
	    media_entity_type(sink->entity) != MEDIA_ENT_T_V4L2_SUBDEV) {
		gsc_err("media entity type isn't subdev\n");
		return 0;
	}

	sd = media_entity_to_v4l2_subdev(source->entity);
	gsc = entity_data_to_gsc(v4l2_get_subdevdata(sd));
	f = &gsc->out.ctx->d_frame;

	src_fmt.format.width = f->crop.width;
	src_fmt.format.height = f->crop.height;
	src_fmt.format.code = f->fmt->mbus_code;

	sd = media_entity_to_v4l2_subdev(sink->entity);
	/* To check if G-Scaler destination size and Mixer destinatin size
	   are the same */
	dst_crop.pad = sink->index;
	dst_crop.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(sd, pad, get_crop, NULL, &dst_crop);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		gsc_err("subdev get_fmt is failed\n");
		return -EPIPE;
	}

	if (gsc->out.ctx->ctrl_val.rot == 90 || gsc->out.ctx->ctrl_val.rot == 270) {
		dst_width = dst_crop.rect.height;
		dst_height = dst_crop.rect.width;
	} else {
		dst_width = dst_crop.rect.width;
		dst_height = dst_crop.rect.height;
	}

	if (src_fmt.format.width != dst_width ||
	    src_fmt.format.height != dst_height) {
		gsc_err("sink and source format is different\
			src_fmt.w = %d, src_fmt.h = %d,\
			dst_crop.w = %d, dst_crop.h = %d, rotation = %d",
			src_fmt.format.width, src_fmt.format.height,
			dst_crop.rect.width, dst_crop.rect.height,
			gsc->out.ctx->ctrl_val.rot);
		return -EINVAL;
	}

	return 0;
}

/*
 * V4L2 controls handling
 */
static int gsc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gsc_ctx *ctx = ctrl_to_ctx(ctrl);
	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			ctx->ctrl_val.hflip = true;
		else
			ctx->ctrl_val.hflip = false;
		break;

	case V4L2_CID_VFLIP:
		if (ctrl->val)
			ctx->ctrl_val.vflip = true;
		else
			ctx->ctrl_val.vflip = false;
		break;

	case V4L2_CID_ROTATE:
		ctx->ctrl_val.rot = ctrl->val;
		break;

	case V4L2_CID_GLOBAL_ALPHA:
		ctx->ctrl_val.global_alpha = ctrl->val;
		break;

	case V4L2_CID_CACHEABLE:
		ctx->ctrl_val.cacheable = ctrl->val;
		break;

	default:
		gsc_err("Invalid control\n");
		return -EINVAL;
	}

	if (gsc_m2m_opened(ctx->gsc_dev))
		gsc_ctx_state_lock_set(GSC_PARAMS, ctx);

	return 0;
}

const struct v4l2_ctrl_ops gsc_ctrl_ops = {
	.s_ctrl = gsc_s_ctrl,
};

static const struct v4l2_ctrl_config gsc_custom_ctrl[] = {
	{
		.ops = &gsc_ctrl_ops,
		.id = V4L2_CID_GLOBAL_ALPHA,
		.name = "Set RGB alpha",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.max = 255,
		.step = 1,
	}, {
		.ops = &gsc_ctrl_ops,
		.id = V4L2_CID_CACHEABLE,
		.name = "Set cacheable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

int gsc_ctrls_create(struct gsc_ctx *ctx)
{
	if (ctx->ctrls_rdy) {
		gsc_err("Control handler of this context was created already");
		return 0;
	}

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, GSC_MAX_CTRL_NUM);

	ctx->ctrl_rotate = v4l2_ctrl_new_std(&ctx->ctrl_handler, &gsc_ctrl_ops,
					V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctx->ctrl_hflip = v4l2_ctrl_new_std(&ctx->ctrl_handler, &gsc_ctrl_ops,
					V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctx->ctrl_vflip = v4l2_ctrl_new_std(&ctx->ctrl_handler, &gsc_ctrl_ops,
					V4L2_CID_ROTATE, 0, 270, 90, 0);
	ctx->ctrl_global_alpha = v4l2_ctrl_new_custom(&ctx->ctrl_handler,
					&gsc_custom_ctrl[0], NULL);
	ctx->ctrl_cacheable = v4l2_ctrl_new_custom(&ctx->ctrl_handler,
					&gsc_custom_ctrl[1], NULL);
	ctx->ctrls_rdy = ctx->ctrl_handler.error == 0;

	if (ctx->ctrl_handler.error) {
		int err = ctx->ctrl_handler.error;

		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		return err;
	}

	return 0;
}

void gsc_ctrls_delete(struct gsc_ctx *ctx)
{
	if (ctx->ctrls_rdy) {
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		ctx->ctrls_rdy = false;
	}
}

/* The color format (nr_comp, num_planes) must be already configured. */
int gsc_prepare_addr(struct gsc_ctx *ctx, struct vb2_buffer *vb,
		     struct gsc_frame *frame, struct gsc_addr *addr)
{
	struct gsc_dev *gsc = ctx->gsc_dev;
	int ret = 0;
	u32 pix_size;

	if (IS_ERR(vb) || IS_ERR(frame)) {
		gsc_err("Invalid argument");
		return -EINVAL;
	}

	pix_size = frame->f_width * frame->f_height;

	gsc_dbg("num_planes= %d, nr_comp= %d, pix_size= %d",
		frame->fmt->num_planes, frame->fmt->nr_comp, pix_size);

	addr->y = gsc->vb2->plane_addr(vb, 0);

	if (frame->fmt->num_planes == 1) {
		switch (frame->fmt->nr_comp) {
		case 1:
			addr->cb = 0;
			addr->cr = 0;
			break;
		case 2:
			/* decompose Y into Y/Cb */
			addr->cb = (dma_addr_t)(addr->y + pix_size);
			addr->cr = 0;
			break;
		case 3:
			addr->cb = (dma_addr_t)(addr->y + pix_size);
			addr->cr = (dma_addr_t)(addr->cb + (pix_size >> 2));
			break;
		default:
			gsc_err("Invalid the number of color planes");
			return -EINVAL;
		}
	} else {
		if (frame->fmt->num_planes >= 2)
			addr->cb = gsc->vb2->plane_addr(vb, 1);

		if (frame->fmt->num_planes == 3)
			addr->cr = gsc->vb2->plane_addr(vb, 2);
	}

	gsc_dbg("ADDR: y= 0x%X  cb= 0x%X cr= 0x%X ret= %d",
		addr->y, addr->cb, addr->cr, ret);

	return ret;
}

void gsc_wq_suspend(struct work_struct *work)
{
	struct gsc_dev *gsc = container_of(work, struct gsc_dev,
					     work_struct);
	pm_runtime_put_sync(&gsc->pdev->dev);
}

void gsc_cap_irq_handler(struct gsc_dev *gsc)
{
	int done_index;

	done_index = gsc_hw_get_done_output_buf_index(gsc);
	gsc_info("done_index : %d", done_index);
	if (done_index < 0)
		gsc_err("All buffers are masked\n");
	test_bit(ST_CAPT_RUN, &gsc->state) ? :
		set_bit(ST_CAPT_RUN, &gsc->state);
	vb2_buffer_done(gsc->cap.vbq.bufs[done_index], VB2_BUF_STATE_DONE);
}

static irqreturn_t gsc_irq_handler(int irq, void *priv)
{
	struct gsc_dev *gsc = priv;
	int gsc_irq;

	gsc_irq = gsc_hw_get_irq_status(gsc);
	gsc_hw_clear_irq(gsc, gsc_irq);

	if (gsc_irq == GSC_OR_IRQ) {
		gsc_err("Local path input over-run interrupt has occurred!\n");
		return IRQ_HANDLED;
	}

	spin_lock(&gsc->slock);

	if (test_and_clear_bit(ST_M2M_RUN, &gsc->state)) {
		struct vb2_buffer *src_vb, *dst_vb;
		struct gsc_ctx *ctx =
			v4l2_m2m_get_curr_priv(gsc->m2m.m2m_dev);

		if (!ctx || !ctx->m2m_ctx)
			goto isr_unlock;

		src_vb = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
		dst_vb = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
		if (src_vb && dst_vb) {
			v4l2_m2m_buf_done(src_vb, VB2_BUF_STATE_DONE);
			v4l2_m2m_buf_done(dst_vb, VB2_BUF_STATE_DONE);

			if (test_and_clear_bit(ST_STOP_REQ, &gsc->state))
				wake_up(&gsc->irq_queue);
			else
				v4l2_m2m_job_finish(gsc->m2m.m2m_dev, ctx->m2m_ctx);

			/* wake_up job_abort, stop_streaming */
			spin_lock(&ctx->slock);
			if (ctx->state & GSC_CTX_STOP_REQ) {
				ctx->state &= ~GSC_CTX_STOP_REQ;
				wake_up(&gsc->irq_queue);
			}
			spin_unlock(&ctx->slock);
		}
		/* schedule pm_runtime_put_sync */
		queue_work(gsc->irq_workqueue, &gsc->work_struct);
	} else if (test_bit(ST_OUTPUT_STREAMON, &gsc->state)) {
		if (!list_empty(&gsc->out.active_buf_q)) {
			struct gsc_input_buf *done_buf;
			done_buf = active_queue_pop(&gsc->out, gsc);
			gsc_hw_set_input_buf_masking(gsc, done_buf->idx, true);
			vb2_buffer_done(&done_buf->vb, VB2_BUF_STATE_DONE);
			list_del(&done_buf->list);
		}
	} else if (test_bit(ST_CAPT_PEND, &gsc->state)) {
		gsc_cap_irq_handler(gsc);
	}

isr_unlock:
	spin_unlock(&gsc->slock);
	return IRQ_HANDLED;
}

static int gsc_get_media_info(struct device *dev, void *p)
{
	struct exynos_md **mdev = p;
	struct platform_device *pdev = to_platform_device(dev);

	mdev[pdev->id] = dev_get_drvdata(dev);
	if (!mdev[pdev->id])
		return -ENODEV;

	return 0;
}

static int gsc_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gsc_dev *gsc = (struct gsc_dev *)platform_get_drvdata(pdev);

	if (gsc_m2m_opened(gsc))
		gsc->m2m.ctx = NULL;

	gsc->vb2->suspend(gsc->alloc_ctx);
	clk_disable(gsc->clock);
	clear_bit(ST_PWR_ON, &gsc->state);

	return 0;
}

static int gsc_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gsc_dev *gsc = (struct gsc_dev *)platform_get_drvdata(pdev);

	clk_enable(gsc->clock);
	gsc->vb2->resume(gsc->alloc_ctx);
	set_bit(ST_PWR_ON, &gsc->state);
	return 0;
}

static int gsc_probe(struct platform_device *pdev)
{
	struct gsc_dev *gsc;
	struct resource *res;
	struct gsc_driverdata *drv_data;
	struct device_driver *driver;
	struct exynos_md *mdev[2] = {NULL, NULL};
	int ret = 0;
	char workqueue_name[WORKQUEUE_NAME_SIZE];

	dev_dbg(&pdev->dev, "%s():\n", __func__);
	drv_data = (struct gsc_driverdata *)
		platform_get_device_id(pdev)->driver_data;

	if (pdev->id >= drv_data->num_entities) {
		dev_err(&pdev->dev, "Invalid platform device id: %d\n",
			pdev->id);
		return -EINVAL;
	}

	gsc = kzalloc(sizeof(struct gsc_dev), GFP_KERNEL);
	if (!gsc)
		return -ENOMEM;

	gsc->id = pdev->id;
	gsc->variant = drv_data->variant[gsc->id];
	gsc->pdev = pdev;
	gsc->pdata = pdev->dev.platform_data;

	init_waitqueue_head(&gsc->irq_queue);
	spin_lock_init(&gsc->slock);
	mutex_init(&gsc->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to find the registers\n");
		ret = -ENOENT;
		goto err_info;
	}

	gsc->regs_res = request_mem_region(res->start, resource_size(res),
			dev_name(&pdev->dev));
	if (!gsc->regs_res) {
		dev_err(&pdev->dev, "failed to obtain register region\n");
		ret = -ENOENT;
		goto err_info;
	}

	gsc->regs = ioremap(res->start, resource_size(res));
	if (!gsc->regs) {
		dev_err(&pdev->dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_req_region;
	}

	/* Get Gscaler clock */
	gsc->clock = clk_get(&gsc->pdev->dev, GSC_CLOCK_GATE_NAME);
	if (IS_ERR(gsc->clock)) {
		gsc_err("failed to get gscaler.%d clock", gsc->id);
		goto err_regs_unmap;
	}
	clk_put(gsc->clock);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get IRQ resource\n");
		ret = -ENXIO;
		goto err_regs_unmap;
	}
	gsc->irq = res->start;

	ret = request_irq(gsc->irq, gsc_irq_handler, 0, pdev->name, gsc);
	if (ret) {
		dev_err(&pdev->dev, "failed to install irq (%d)\n", ret);
		goto err_regs_unmap;
	}

#if defined(CONFIG_VIDEOBUF2_CMA_PHYS)
	gsc->vb2 = &gsc_vb2_cma;
#elif defined(CONFIG_VIDEOBUF2_ION)
	gsc->vb2 = &gsc_vb2_ion;
#endif

	platform_set_drvdata(pdev, gsc);

	ret = gsc_register_m2m_device(gsc);
	if (ret)
		goto err_irq;

	/* find media device */
	driver = driver_find(MDEV_MODULE_NAME, &platform_bus_type);
	if (!driver)
		goto err_irq;

	ret = driver_for_each_device(driver, NULL, &mdev[0],
			gsc_get_media_info);
	put_driver(driver);
	if (ret)
		goto err_irq;

	gsc->mdev[MDEV_OUTPUT] = mdev[MDEV_OUTPUT];
	gsc->mdev[MDEV_CAPTURE] = mdev[MDEV_CAPTURE];

	gsc_info("mdev->mdev[%d] = 0x%08x, mdev->mdev[%d] = 0x%08x",
		 MDEV_OUTPUT, (u32)gsc->mdev[MDEV_OUTPUT], MDEV_CAPTURE,
		 (u32)gsc->mdev[MDEV_CAPTURE]);

	ret = gsc_register_output_device(gsc);
	if (ret)
		goto err_irq;

	if (gsc->pdata)	{
		ret = gsc_register_capture_device(gsc);
		if (ret)
			goto err_irq;
	}
	pm_runtime_enable(&pdev->dev);

	sprintf(workqueue_name, "gsc%d_irq_wq_name", gsc->id);
	gsc->irq_workqueue = create_singlethread_workqueue(workqueue_name);
	if (gsc->irq_workqueue == NULL) {
		dev_err(&pdev->dev, "failed to create workqueue for gsc\n");
		goto err_irq;
	}
	INIT_WORK(&gsc->work_struct, gsc_wq_suspend);

	gsc->alloc_ctx = gsc->vb2->init(gsc);
	if (IS_ERR(gsc->alloc_ctx)) {
		ret = PTR_ERR(gsc->alloc_ctx);
		goto err_wq;
	}

	gsc_info("gsc-%d registered successfully", gsc->id);

	return 0;

err_wq:
	destroy_workqueue(gsc->irq_workqueue);
err_irq:
	free_irq(gsc->irq, gsc);
err_regs_unmap:
	iounmap(gsc->regs);
err_req_region:
	release_resource(gsc->regs_res);
	kfree(gsc->regs_res);
err_info:
	kfree(gsc);

	return ret;
}

static int __devexit gsc_remove(struct platform_device *pdev)
{
	struct gsc_dev *gsc =
		(struct gsc_dev *)platform_get_drvdata(pdev);

	free_irq(gsc->irq, gsc);

	gsc_unregister_m2m_device(gsc);
	gsc_unregister_output_device(gsc);
	gsc_unregister_capture_device(gsc);

	/* Add clock disable function */

	gsc->vb2->cleanup(gsc->alloc_ctx);

	clk_disable(gsc->clock);
	clk_put(gsc->clock);
	iounmap(gsc->regs);
	release_resource(gsc->regs_res);
	kfree(gsc->regs_res);
	kfree(gsc);

	dev_info(&pdev->dev, "%s driver unloaded\n", pdev->name);
	return 0;
}

static int gsc_suspend(struct device *dev)
{
	struct platform_device *pdev;
	struct gsc_dev *gsc;
	int ret = 0;

	pdev = to_platform_device(dev);
	gsc = (struct gsc_dev *)platform_get_drvdata(pdev);

	if (gsc_m2m_run(gsc)) {
		set_bit(ST_STOP_REQ, &gsc->state);
		ret = wait_event_timeout(gsc->irq_queue,
				!test_bit(ST_STOP_REQ, &gsc->state),
				GSC_SHUTDOWN_TIMEOUT);
		if (ret == 0)
			dev_err(&gsc->pdev->dev, "wait timeout : %s\n",
				__func__);
	}
	pm_runtime_put_sync(dev);

	return ret;
}

static int gsc_resume(struct device *dev)
{
	struct platform_device *pdev;
	struct gsc_driverdata *drv_data;
	struct gsc_dev *gsc;
	struct gsc_ctx *ctx;

	pdev = to_platform_device(dev);
	gsc = (struct gsc_dev *)platform_get_drvdata(pdev);
	drv_data = (struct gsc_driverdata *)
		platform_get_device_id(pdev)->driver_data;

	pm_runtime_get_sync(dev);
	if (gsc_m2m_opened(gsc)) {
		ctx = v4l2_m2m_get_curr_priv(gsc->m2m.m2m_dev);
		if (ctx != NULL) {
			gsc->m2m.ctx = NULL;
			v4l2_m2m_job_finish(gsc->m2m.m2m_dev, ctx->m2m_ctx);
		}
	}

	return 0;
}

static const struct dev_pm_ops gsc_pm_ops = {
	.suspend		= gsc_suspend,
	.resume			= gsc_resume,
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend	= gsc_runtime_suspend,
	.runtime_resume		= gsc_runtime_resume,
#endif
};

struct gsc_pix_max gsc_max_exynos5210 = {
	.org_scaler_bypass_w	= 8192,
	.org_scaler_bypass_h	= 8192,
	.org_scaler_input_w	= 4800,
	.org_scaler_input_h	= 3344,
	.real_rot_dis_w		= 4800,
	.real_rot_dis_h		= 3344,
	.real_rot_en_w		= 2047,
	.real_rot_en_h		= 2047,
	.target_w		= 4800,
	.target_h		= 3344,
};

struct gsc_pix_min gsc_min_exynos5210 = {
	.org_w			= 64,
	.org_h			= 32,
	.real_w			= 64,
	.real_h			= 32,
	.target_w		= 64,
	.target_h		= 32,
};

struct gsc_pix_align gsc_align_exynos5210 = {
	.org_h			= 16,
	.org_w			= 16, /* yuv420 : 16, others : 8 */
	.offset_h		= 2,  /* yuv420/422 : 2, others : 1 */
	.real_w			= 16, /* yuv420/422 : 4~16, others : 2~8 */
	.real_h			= 16, /* yuv420 : 4~16, others : 1 */
	.target_w		= 2,  /* yuv420/422 : 2, others : 1 */
	.target_h		= 2,  /* yuv420 : 2, others : 1 */
};

struct gsc_variant gsc_variant_exynos5210 = {
	.pix_max		= &gsc_max_exynos5210,
	.pix_min		= &gsc_min_exynos5210,
	.pix_align		= &gsc_align_exynos5210,
	.in_buf_cnt		= 8,
	.out_buf_cnt		= 16,
	.sc_up_max		= 8,
	.sc_down_max		= 16,
	.poly_sc_down_max	= 4,
	.pre_sc_down_max	= 4,
	.local_sc_down		= 2,
};

static struct gsc_driverdata gsc_drvdata_exynos5210 = {
	.variant = {
		[0] = &gsc_variant_exynos5210,
		[1] = &gsc_variant_exynos5210,
		[2] = &gsc_variant_exynos5210,
		[3] = &gsc_variant_exynos5210,
	},
	.num_entities = 4,
	.lclk_frequency = 266000000UL,
};

static struct platform_device_id gsc_driver_ids[] = {
	{
		.name		= "exynos-gsc",
		.driver_data	= (unsigned long)&gsc_drvdata_exynos5210,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, gsc_driver_ids);

static struct platform_driver gsc_driver = {
	.probe		= gsc_probe,
	.remove	= __devexit_p(gsc_remove),
	.id_table	= gsc_driver_ids,
	.driver = {
		.name	= GSC_MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &gsc_pm_ops,
	}
};

static int __init gsc_init(void)
{
	int ret = platform_driver_register(&gsc_driver);
	if (ret)
		gsc_err("platform_driver_register failed: %d\n", ret);
	return ret;
}

static void __exit gsc_exit(void)
{
	platform_driver_unregister(&gsc_driver);
}

module_init(gsc_init);
module_exit(gsc_exit);

MODULE_AUTHOR("Hyunwong Kim <khw0178.kim@samsung.com>");
MODULE_DESCRIPTION("Samsung EXYNOS5 Soc series G-Scaler driver");
MODULE_LICENSE("GPL");
