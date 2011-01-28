/*
 * linux/drivers/media/video/s5p-mfc/s5p_mfc_dec.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/io.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/videodev2.h>
#include <media/videobuf2-core.h>

#include "regs-mfc.h"

#include "s5p_mfc_opr.h"
#include "s5p_mfc_intr.h"
#include "s5p_mfc_mem.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_reg.h"
#include "s5p_mfc_dec.h"
#include "s5p_mfc_common.h"

#define DEF_SRC_FMT	2
#define DEF_DST_FMT	0

static struct s5p_mfc_fmt formats[] = {
	{
		.name = "4:2:0 2 Planes 64x32 Tiles",
		.fourcc = V4L2_PIX_FMT_NV12MT,
		.codec_mode = MFC_FORMATS_NO_CODEC,
		.type = MFC_FMT_RAW,
		.num_planes = 2,
	 },
	{
		.name = "4:2:0 2 Planes",
		.fourcc = V4L2_PIX_FMT_NV12M,
		.codec_mode = MFC_FORMATS_NO_CODEC,
		.type = MFC_FMT_RAW,
		.num_planes = 2,
	},
	{
		.name = "H264 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_H264,
		.codec_mode = S5P_FIMV_CODEC_H264_DEC,
		.type = MFC_FMT_DEC,
		.num_planes = 1,
	},
	{
		.name = "H263 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_H263,
		.codec_mode = S5P_FIMV_CODEC_H263_DEC,
		.type = MFC_FMT_DEC,
		.num_planes = 1,
	},
	{
		.name = "MPEG1/MPEG2 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_MPEG12,
		.codec_mode = S5P_FIMV_CODEC_MPEG2_DEC,
		.type = MFC_FMT_DEC,
		.num_planes = 1,
	},
	{
		.name = "MPEG4 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_MPEG4,
		.codec_mode = S5P_FIMV_CODEC_MPEG4_DEC,
		.type = MFC_FMT_DEC,
		.num_planes = 1,
	},
	{
		.name = "DivX Encoded Stream",
		.fourcc = V4L2_PIX_FMT_DIVX,
		.codec_mode = S5P_FIMV_CODEC_MPEG4_DEC,
		.type = MFC_FMT_DEC,
		.num_planes = 1,
	},
	{
		.name = "DivX 3.11 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_DIVX3,
		.codec_mode = S5P_FIMV_CODEC_DIVX311_DEC,
		.type = MFC_FMT_DEC,
		.num_planes = 1,
	},
	{
		.name = "DivX 4.12 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_DIVX4,
		.codec_mode = S5P_FIMV_CODEC_DIVX412_DEC,
		.type = MFC_FMT_DEC,
		.num_planes = 1,
	},
	{
		.name = "DivX 5.00-5.02 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_DIVX500,
		.codec_mode = S5P_FIMV_CODEC_DIVX502_DEC,
		.type = MFC_FMT_DEC,
		.num_planes = 1,
	},
	{
		.name = "DivX 5.03 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_DIVX503,
		.codec_mode = S5P_FIMV_CODEC_DIVX503_DEC,
		.type = MFC_FMT_DEC,
		.num_planes = 1,
	},
	{
		.name = "XviD Encoded Stream",
		.fourcc = V4L2_PIX_FMT_XVID,
		.codec_mode = S5P_FIMV_CODEC_MPEG4_DEC,
		.type = MFC_FMT_DEC,
		.num_planes = 1,
	},
	{
		.name = "VC1 Encoded Stream",
		.fourcc = V4L2_PIX_FMT_VC1,
		.codec_mode = S5P_FIMV_CODEC_VC1_DEC,
		.type = MFC_FMT_DEC,
		.num_planes = 1,
	},
	{
		.name = "VC1 RCV Encoded Stream",
		.fourcc = V4L2_PIX_FMT_VC1_RCV,
		.codec_mode = S5P_FIMV_CODEC_VC1RCV_DEC,
		.type = MFC_FMT_DEC,
		.num_planes = 1,
	},
};

#define NUM_FORMATS ARRAY_SIZE(formats)

/* Find selected format description */
static struct s5p_mfc_fmt *find_format(struct v4l2_format *f, unsigned int t)
{
	unsigned int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].fourcc == f->fmt.pix_mp.pixelformat &&
		    formats[i].type == t)
			return (struct s5p_mfc_fmt *)&formats[i];
	}

	return NULL;
}

static struct v4l2_queryctrl controls[] = {
	{
		.id = V4L2_CID_CODEC_DISPLAY_DELAY,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "",
		.minimum = 0,
		.maximum = 16383,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_CODEC_LOOP_FILTER_MPEG4_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Mpeg4 Loop Filter Enable",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_CODEC_SLICE_INTERFACE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Slice Interface Enable",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
};

#define NUM_CTRLS ARRAY_SIZE(controls)

static struct v4l2_queryctrl *get_ctrl(int id)
{
	int i;

	for (i = 0; i < NUM_CTRLS; ++i)
		if (id == controls[i].id)
			return &controls[i];
	return NULL;
}

/* Check whether a ctrl value if correct */
static int check_ctrl_val(struct s5p_mfc_ctx *ctx, struct v4l2_control *ctrl)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	struct v4l2_queryctrl *c;

	c = get_ctrl(ctrl->id);
	if (!c)
		return -EINVAL;
	if (ctrl->value < c->minimum || ctrl->value > c->maximum
	    || (c->step != 0 && ctrl->value % c->step != 0)) {
		v4l2_err(&dev->v4l2_dev, "Invalid control value\n");
		return -ERANGE;
	}
	return 0;
}

/* Check whether a context should be run on hardware */
static int s5p_mfc_ctx_ready(struct s5p_mfc_ctx *ctx)
{
	mfc_debug("src=%d, dst=%d, state=%d capstat=%d\n",
		  ctx->src_queue_cnt, ctx->dst_queue_cnt,
		  ctx->state, ctx->capture_state);

	/* Context is to parse header */
	if (ctx->src_queue_cnt >= 1 && ctx->state == MFCINST_GOT_INST)
		return 1;
	/* Context is to decode a frame */
	if (ctx->src_queue_cnt >= 1 &&
	    ctx->state == MFCINST_RUNNING &&
	    ctx->dst_queue_cnt >= ctx->dpb_count)
		return 1;
	/* Context is to return last frame */
	if (ctx->state == MFCINST_FINISHING &&
	    ctx->dst_queue_cnt >= ctx->dpb_count)
		return 1;
	/* Context is to set buffers */
	if (ctx->src_queue_cnt >= 1 &&
	    ctx->state == MFCINST_HEAD_PARSED &&
	    ctx->capture_state == QUEUE_BUFS_MMAPED)
		return 1;

	mfc_debug("s5p_mfc_ctx_ready: ctx is not ready.\n");

	return 0;
}

static struct s5p_mfc_codec_ops decoder_codec_ops = {
	.pre_seq_start		= NULL,
	.post_seq_start		= NULL,
	.pre_frame_start	= NULL,
	.post_frame_start	= NULL,
};

/* Query capabilities of the device */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);

	strncpy(cap->driver, dev->plat_dev->name, sizeof(cap->driver) - 1);
	strncpy(cap->card, dev->plat_dev->name, sizeof(cap->card) - 1);
	cap->bus_info[0] = 0;
	cap->version = KERNEL_VERSION(1, 0, 0);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT
						    | V4L2_CAP_STREAMING;
	return 0;
}

/* Enumerate format */
static int vidioc_enum_fmt(struct v4l2_fmtdesc *f, bool mplane, bool out)
{
	struct s5p_mfc_fmt *fmt;
	int i, j = 0;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (mplane && formats[i].num_planes == 1)
			continue;
		else if (!mplane && formats[i].num_planes > 1)
			continue;
		/* FIXME: to Kamil */
		/*
		if (out && formats[i].type != MFC_FMT_RAW)
			continue;
		else if (!out && formats[i].type != MFC_FMT_DEC)
			continue;
		*/
		if (out && formats[i].type != MFC_FMT_DEC)
			continue;
		else if (!out && formats[i].type != MFC_FMT_RAW)
			continue;

		if (j == f->index)
			break;
		++j;
	}
	if (i == ARRAY_SIZE(formats))
		return -EINVAL;
	fmt = &formats[i];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *pirv,
							struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(f, false, false);
}

static int vidioc_enum_fmt_vid_cap_mplane(struct file *file, void *pirv,
							struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(f, true, false);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *prov,
							struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(f, false, true);
}

static int vidioc_enum_fmt_vid_out_mplane(struct file *file, void *prov,
							struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(f, true, true);
}

/* Get format */
static int vidioc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_ctx *ctx = priv;
	struct v4l2_pix_format_mplane *pix_mp;

	mfc_debug_enter();
	pix_mp = &f->fmt.pix_mp;
	mfc_debug("f->type = %d ctx->state = %d\n", f->type, ctx->state);
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    ctx->state == MFCINST_GOT_INST) {
		/* If the MFC is parsing the header,
		 * so wait until it is finished */
		s5p_mfc_clean_ctx_int_flags(ctx);
		s5p_mfc_wait_for_done_ctx(ctx, S5P_FIMV_R2H_CMD_SEQ_DONE_RET,
									1);
	}
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    ctx->state >= MFCINST_HEAD_PARSED &&
	    ctx->state < MFCINST_ABORT) {
		/* This is run on CAPTURE (deocde output) */
		/* Width and height are set to the dimensions
		   of the movie, the buffer is bigger and
		   further processing stages should crop to this
		   rectangle. */
		pix_mp->width = ctx->buf_width;
		pix_mp->height = ctx->buf_height;
		pix_mp->field = V4L2_FIELD_NONE;
		pix_mp->num_planes = 2;
		/* Set pixelformat to the format in which MFC
		   outputs the decoded frame */
		pix_mp->pixelformat = V4L2_PIX_FMT_NV12MT;
		pix_mp->plane_fmt[0].bytesperline = ctx->buf_width;
		pix_mp->plane_fmt[0].sizeimage = ctx->luma_size;
		pix_mp->plane_fmt[1].bytesperline = ctx->buf_width;
		pix_mp->plane_fmt[1].sizeimage = ctx->chroma_size;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* This is run on OUTPUT
		   The buffer contains compressed image
		   so width and height have no meaning */
		pix_mp->width = 0;
		pix_mp->height = 0;
		pix_mp->field = V4L2_FIELD_NONE;
		pix_mp->plane_fmt[0].bytesperline = ctx->dec_src_buf_size;
		pix_mp->plane_fmt[0].sizeimage = ctx->dec_src_buf_size;
		pix_mp->pixelformat = ctx->src_fmt->fourcc;
		pix_mp->num_planes = ctx->src_fmt->num_planes;
	} else {
		mfc_err("Format could not be read\n");
		mfc_debug("%s-- with error\n", __func__);
		return -EINVAL;
	}
	mfc_debug_leave();
	return 0;
}

/* Try format */
static int vidioc_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_fmt *fmt;
	struct v4l2_pix_format_mplane *pix_mp;

	mfc_debug("Type is %d\n", f->type);
	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_err("Currently only decoding is supported.\n");
		return -EINVAL;
	}
	fmt = find_format(f, MFC_FMT_DEC);
	if (!fmt) {
		mfc_err("Unsupported format.\n");
		return -EINVAL;
	}
	if (fmt->type != MFC_FMT_DEC) {
		mfc_err("\n");
		return -EINVAL;
	}
	pix_mp = &f->fmt.pix_mp;
	if (pix_mp->plane_fmt[0].sizeimage == 0) {
		mfc_err("Application is required to "
			"specify input buffer size (via sizeimage)\n");
		return -EINVAL;
	}
	/* As this buffer will contain compressed data, the size is set
	 * to the maximum size.
	 * Width and height are left intact as they may be relevant for
	 * DivX 3.11 decoding. */
	pix_mp->plane_fmt[0].bytesperline = pix_mp->plane_fmt[0].sizeimage;
	return 0;
}

/* Set format */
static int vidioc_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = priv;
	unsigned long flags;
	int ret = 0;
	struct s5p_mfc_fmt *fmt;
	struct v4l2_pix_format_mplane *pix_mp;

	mfc_debug_enter();
	ret = vidioc_try_fmt(file, priv, f);
	pix_mp = &f->fmt.pix_mp;
	if (ret)
		return ret;
	if (ctx->vq_src.streaming || ctx->vq_dst.streaming) {
		v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
		ret = -EBUSY;
		goto out;
	}
	fmt = find_format(f, MFC_FMT_DEC);
	if (!fmt || fmt->codec_mode == MFC_FORMATS_NO_CODEC) {
		mfc_err("Unknown codec.\n");
		ret = -EINVAL;
		goto out;
	}
	if (fmt->type != MFC_FMT_DEC) {
		mfc_err("Wrong format selected, you should choose "
					"format for decoding.\n");
		ret = -EINVAL;
		goto out;
	}
	ctx->src_fmt = fmt;
	ctx->codec_mode = fmt->codec_mode;
	mfc_debug("The codec number is: %d\n", ctx->codec_mode);
	ctx->pix_format = pix_mp->pixelformat;
	if (pix_mp->pixelformat != V4L2_PIX_FMT_DIVX3) {
		pix_mp->height = 0;
		pix_mp->width = 0;
	} else {
		ctx->img_height = pix_mp->height;
		ctx->img_width = pix_mp->width;
	}
	mfc_debug("s_fmt w/h: %dx%d, ctx: %dx%d\n", pix_mp->width,
		pix_mp->height, ctx->img_width, ctx->img_height);
	ctx->dec_src_buf_size =	pix_mp->plane_fmt[0].sizeimage;
	pix_mp->plane_fmt[0].bytesperline = 0;
	ctx->state = MFCINST_INIT;
	ctx->dst_bufs_cnt = 0;
	ctx->src_bufs_cnt = 0;
	ctx->capture_state = QUEUE_FREE;
	ctx->output_state = QUEUE_FREE;
	s5p_mfc_alloc_instance_buffer(ctx);
	s5p_mfc_alloc_dec_temp_buffers(ctx);
	spin_lock_irqsave(&dev->condlock, flags);
	set_bit(ctx->num, &dev->ctx_work_bits);
	spin_unlock_irqrestore(&dev->condlock, flags);
	s5p_mfc_clean_ctx_int_flags(ctx);
	s5p_mfc_try_run(dev);
	if (s5p_mfc_wait_for_done_ctx(ctx,
			S5P_FIMV_R2H_CMD_OPEN_INSTANCE_RET, 1)) {
		/* Error or timeout */
		mfc_err("Error getting instance from hardware.\n");
		s5p_mfc_release_instance_buffer(ctx);
		s5p_mfc_release_dec_desc_buffer(ctx);
		ret = -EIO;
		goto out;
	}
	mfc_debug("Got instance number: %d\n", ctx->inst_no);
out:
	mfc_debug_leave();
	return ret;
}

/* Reqeust buffers */
static int vidioc_reqbufs(struct file *file, void *priv,
					  struct v4l2_requestbuffers *reqbufs)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = priv;
	int ret = 0;
	unsigned long flags;

	mfc_debug_enter();
	mfc_debug("Memory type: %d\n", reqbufs->memory);
	if (reqbufs->memory != V4L2_MEMORY_MMAP) {
		mfc_err("Only V4L2_MEMORY_MAP is supported.\n");
		return -EINVAL;
	}
	if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* Can only request buffers after an instance has been opened.*/
		if (ctx->state == MFCINST_GOT_INST) {
			/* Decoding */
			if (ctx->output_state != QUEUE_FREE) {
				mfc_err("Bufs have already been requested.\n");
				return -EINVAL;
			}
			ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
			if (ret) {
				mfc_err("vb2_reqbufs on output failed.\n");
				return ret;
			}
			mfc_debug("vb2_reqbufs: %d\n", ret);
			ctx->output_state = QUEUE_BUFS_REQUESTED;
		}
	} else if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (ctx->capture_state != QUEUE_FREE) {
			mfc_err("Bufs have already been requested.\n");
			return -EINVAL;
		}
		ctx->capture_state = QUEUE_BUFS_REQUESTED;
		ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
		if (ret) {
			mfc_err("vb2_reqbufs on capture failed.\n");
			return ret;
		}
		if (reqbufs->count < ctx->dpb_count) {
			mfc_err("Not enough buffers allocated.\n");
			reqbufs->count = 0;
			ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
			return -ENOMEM;
		}
		ctx->total_dpb_count = reqbufs->count;
		ret = s5p_mfc_alloc_codec_buffers(ctx);
		if (ret) {
			mfc_err("Failed to allocate decoding buffers.\n");
			reqbufs->count = 0;
			ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
			return -ENOMEM;
		}
		if (ctx->dst_bufs_cnt == ctx->total_dpb_count) {
			ctx->capture_state = QUEUE_BUFS_MMAPED;
		} else {
			mfc_err("Not all buffers passed to buf_init.\n");
			reqbufs->count = 0;
			ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
			s5p_mfc_release_codec_buffers(ctx);
			return -ENOMEM;
		}
		if (s5p_mfc_ctx_ready(ctx)) {
			spin_lock_irqsave(&dev->condlock, flags);
			set_bit(ctx->num, &dev->ctx_work_bits);
			spin_unlock_irqrestore(&dev->condlock, flags);
		}
		s5p_mfc_try_run(dev);
		s5p_mfc_wait_for_done_ctx(ctx,
					 S5P_FIMV_R2H_CMD_INIT_BUFFERS_RET, 1);
	}
	mfc_debug_leave();
	return ret;
}

/* Query buffer */
static int vidioc_querybuf(struct file *file, void *priv,
						   struct v4l2_buffer *buf)
{
	struct s5p_mfc_ctx *ctx = priv;
	int ret;
	int i;

	mfc_debug_enter();
	if (buf->memory != V4L2_MEMORY_MMAP) {
		mfc_err("Only mmaped buffers can be used.\n");
		return -EINVAL;
	}
	mfc_debug("State: %d, buf->type: %d\n", ctx->state, buf->type);
	if (ctx->state == MFCINST_GOT_INST &&
			buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = vb2_querybuf(&ctx->vq_src, buf);
	} else if (ctx->state == MFCINST_RUNNING &&
			buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = vb2_querybuf(&ctx->vq_dst, buf);
		for (i = 0; i < buf->length; i++)
			buf->m.planes[i].m.mem_offset += DST_QUEUE_OFF_BASE;
	} else {
		mfc_err("vidioc_querybuf called in an inappropriate state.\n");
		ret = -EINVAL;
	}
	mfc_debug_leave();
	return ret;
}

/* Queue a buffer */
static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct s5p_mfc_ctx *ctx = priv;

	mfc_debug_enter();
	mfc_debug("Enqueued buf: %d (type = %d)\n", buf->index, buf->type);
	if (ctx->state == MFCINST_ERROR) {
		mfc_err("Call on QBUF after unrecoverable error.\n");
		return -EIO;
	}
	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return vb2_qbuf(&ctx->vq_src, buf);
	else
		return vb2_qbuf(&ctx->vq_dst, buf);
	mfc_debug_leave();
	return -EINVAL;
}

/* Dequeue a buffer */
static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct s5p_mfc_ctx *ctx = priv;
	int ret;

	mfc_debug_enter();
	mfc_debug("Addr: %p %p %p Type: %d\n", &ctx->vq_src, buf, buf->m.planes,
								buf->type);
	if (ctx->state == MFCINST_ERROR) {
		mfc_err("Call on DQBUF after unrecoverable error.\n");
		return -EIO;
	}
	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		ret = vb2_dqbuf(&ctx->vq_src, buf, file->f_flags & O_NONBLOCK);
	else
		ret = vb2_dqbuf(&ctx->vq_dst, buf, file->f_flags & O_NONBLOCK);
	mfc_debug_leave();
	return ret;
}

/* Stream on */
static int vidioc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct s5p_mfc_ctx *ctx = priv;
	int ret = -EINVAL;

	mfc_debug_enter();
	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		ret = vb2_streamon(&ctx->vq_src, type);
	else
		ret = vb2_streamon(&ctx->vq_dst, type);
	mfc_debug("ctx->src_queue_cnt = %d ctx->state = %d "
		  "ctx->dst_queue_cnt = %d ctx->dpb_count = %d\n",
		  ctx->src_queue_cnt, ctx->state, ctx->dst_queue_cnt,
		  ctx->dpb_count);
	mfc_debug_leave();
	return ret;
}

/* Stream off, which equals to a pause */
static int vidioc_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct s5p_mfc_ctx *ctx = priv;
	int ret;

	mfc_debug_enter();
	ret = -EINVAL;
	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		ret = vb2_streamoff(&ctx->vq_src, type);
	else
		ret = vb2_streamoff(&ctx->vq_dst, type);
	mfc_debug_leave();
	return ret;
}

/* Query a ctrl */
static int vidioc_queryctrl(struct file *file, void *priv,
			    struct v4l2_queryctrl *qc)
{
	struct v4l2_queryctrl *c;

	c = get_ctrl(qc->id);
	if (!c)
		return -EINVAL;
	*qc = *c;
	return 0;
}

/* Get ctrl */
static int vidioc_g_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = priv;

	mfc_debug_enter();
	switch (ctrl->id) {
	case V4L2_CID_CODEC_LOOP_FILTER_MPEG4_ENABLE:
		ctrl->value = ctx->loop_filter_mpeg4;
		break;
	case V4L2_CID_CODEC_DISPLAY_DELAY:
		ctrl->value = ctx->display_delay;
		break;
	case V4L2_CID_CODEC_REQ_NUM_BUFS:
		if (ctx->state >= MFCINST_HEAD_PARSED &&
		    ctx->state < MFCINST_ABORT) {
			ctrl->value = ctx->dpb_count;
			break;
		} else if (ctx->state != MFCINST_INIT) {
			v4l2_err(&dev->v4l2_dev, "Decoding not initialised.\n");
			return -EINVAL;
		}

		/* Should wait for the header to be parsed */
		s5p_mfc_clean_ctx_int_flags(ctx);
		s5p_mfc_wait_for_done_ctx(ctx,
				S5P_FIMV_R2H_CMD_SEQ_DONE_RET, 1);
		if (ctx->state >= MFCINST_HEAD_PARSED &&
		    ctx->state < MFCINST_ABORT) {
			ctrl->value = ctx->dpb_count;
		} else {
			v4l2_err(&dev->v4l2_dev,
					 "Decoding not initialised.\n");
			return -EINVAL;
		}
		break;
	case V4L2_CID_CODEC_SLICE_INTERFACE:
		ctrl->value = ctx->slice_interface;
		break;
	default:
		v4l2_err(&dev->v4l2_dev, "Invalid control\n");
		return -EINVAL;
	}
	mfc_debug_leave();
	return 0;
}

/* Set a ctrl */
static int vidioc_s_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = priv;
	int ret = 0;
	int stream_on;

	mfc_debug_enter();
	stream_on = ctx->vq_src.streaming || ctx->vq_dst.streaming;
	ret = check_ctrl_val(ctx, ctrl);
	if (ret != 0)
		return ret;
	switch (ctrl->id) {
	case V4L2_CID_CODEC_LOOP_FILTER_MPEG4_ENABLE:
		if (stream_on)
			return -EBUSY;
		ctx->loop_filter_mpeg4 = ctrl->value;
		break;
	case V4L2_CID_CODEC_DISPLAY_DELAY:
		if (stream_on)
			return -EBUSY;
		ctx->display_delay = ctrl->value;
		break;
	case V4L2_CID_CODEC_SLICE_INTERFACE:
		if (stream_on)
			return -EBUSY;
		ctx->slice_interface = ctrl->value;
		break;
	default:
		v4l2_err(&dev->v4l2_dev, "Invalid control\n");
		return -EINVAL;
	}
	mfc_debug_leave();
	return 0;
}

/* Get cropping information */
static int vidioc_g_crop(struct file *file, void *priv,
		struct v4l2_crop *cr)
{
	struct s5p_mfc_ctx *ctx = priv;
	u32 left, right, top, bottom;

	mfc_debug_enter();
	if (ctx->state != MFCINST_HEAD_PARSED &&
	ctx->state != MFCINST_RUNNING && ctx->state != MFCINST_FINISHING
					&& ctx->state != MFCINST_FINISHED) {
			mfc_debug("%s-- with error\n", __func__);
			return -EINVAL;
		}
	if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_H264) {
		left = s5p_mfc_get_h_crop(ctx);
		right = left >> S5P_FIMV_SHARED_CROP_RIGHT_SHIFT;
		left = left & S5P_FIMV_SHARED_CROP_LEFT_MASK;
		top = s5p_mfc_get_v_crop(ctx);
		bottom = top >> S5P_FIMV_SHARED_CROP_BOTTOM_SHIFT;
		top = top & S5P_FIMV_SHARED_CROP_TOP_MASK;
		cr->c.left = left;
		cr->c.top = top;
		cr->c.width = ctx->img_width - left - right;
		cr->c.height = ctx->img_height - top - bottom;
		mfc_debug("Cropping info [h264]: l=%d t=%d "
			"w=%d h=%d (r=%d b=%d fw=%d fh=%d\n", left, top,
			cr->c.width, cr->c.height, right, bottom,
			ctx->buf_width, ctx->buf_height);
	} else {
		cr->c.left = 0;
		cr->c.top = 0;
		cr->c.width = ctx->img_width;
		cr->c.height = ctx->img_height;
		mfc_debug("Cropping info: w=%d h=%d fw=%d "
			"fh=%d\n", cr->c.width,	cr->c.height, ctx->buf_width,
							ctx->buf_height);
	}
	mfc_debug_leave();
	return 0;
}

/* v4l2_ioctl_ops */
static const struct v4l2_ioctl_ops s5p_mfc_dec_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap_mplane = vidioc_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt_vid_out,
	.vidioc_enum_fmt_vid_out_mplane = vidioc_enum_fmt_vid_out_mplane,
	.vidioc_g_fmt_vid_cap_mplane = vidioc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vidioc_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vidioc_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vidioc_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vidioc_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vidioc_s_fmt,
	.vidioc_reqbufs = vidioc_reqbufs,
	.vidioc_querybuf = vidioc_querybuf,
	.vidioc_qbuf = vidioc_qbuf,
	.vidioc_dqbuf = vidioc_dqbuf,
	.vidioc_streamon = vidioc_streamon,
	.vidioc_streamoff = vidioc_streamoff,
	.vidioc_queryctrl = vidioc_queryctrl,
	.vidioc_g_ctrl = vidioc_g_ctrl,
	.vidioc_s_ctrl = vidioc_s_ctrl,
	.vidioc_g_crop = vidioc_g_crop,
};

static int s5p_mfc_queue_setup(struct vb2_queue *vq, unsigned int *buf_count,
			       unsigned int *plane_count, unsigned long psize[],
			       void *allocators[])
{
	struct s5p_mfc_ctx *ctx = vq->drv_priv;

	mfc_debug_enter();

	/* Video output for decoding (source)
	 * this can be set after getting an instance */
	if (ctx->state == MFCINST_GOT_INST &&
	    vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_debug("setting for VIDEO output\n");
		/* A single plane is required for input */
		*plane_count = 1;
		if (*buf_count < 1)
			*buf_count = 1;
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;
	/* Video capture for decoding (destination)
	 * this can be set after the header was parsed */
	} else if (ctx->state == MFCINST_HEAD_PARSED &&
		   vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_debug("setting for VIDEO capture\n");
		/* Output plane count is 2 - one for Y and one for CbCr */
		*plane_count = 2;
		/* Setup buffer count */
		if (*buf_count < ctx->dpb_count)
			*buf_count = ctx->dpb_count;
		if (*buf_count > ctx->dpb_count + MFC_MAX_EXTRA_DPB)
			*buf_count = ctx->dpb_count + MFC_MAX_EXTRA_DPB;
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;
	} else {
		mfc_err("State seems invalid. State = %d, vq->type = %d\n",
							ctx->state, vq->type);
		return -EINVAL;
	}
	mfc_debug("%s, buffer count=%d, plane count=%d type=0x%x\n", __func__,
					*buf_count, *plane_count, vq->type);

	if (ctx->state == MFCINST_HEAD_PARSED &&
	    vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		psize[0] = ctx->luma_size;
		psize[1] = ctx->chroma_size;
		/* FIXME: */
		allocators[0] = ctx->dev->alloc_ctx[MFC_CMA_BANK2_ALLOC_CTX];
		allocators[1] = ctx->dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX];
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
		   ctx->state == MFCINST_GOT_INST) {
		psize[0] = ctx->dec_src_buf_size;
		allocators[0] = ctx->dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX];
	} else {
		mfc_err("Currently only decoding is supported. Decoding not initalised.\n");
		return -EINVAL;
	}

	mfc_debug("%s, plane=0, size=%lu\n", __func__, psize[0]);
	mfc_debug("%s, plane=1, size=%lu\n", __func__, psize[1]);

	mfc_debug_leave();

	return 0;
}

static void s5p_mfc_unlock(struct vb2_queue *q)
{
	struct s5p_mfc_ctx *ctx = q->drv_priv;
	struct s5p_mfc_dev *dev = ctx->dev;

	mutex_unlock(&dev->mfc_mutex);
}

static void s5p_mfc_lock(struct vb2_queue *q)
{
	struct s5p_mfc_ctx *ctx = q->drv_priv;
	struct s5p_mfc_dev *dev = ctx->dev;

	mutex_lock(&dev->mfc_mutex);
}

static int s5p_mfc_buf_init(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vq->drv_priv;
	unsigned int i;

	mfc_debug_enter();
	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (ctx->capture_state == QUEUE_BUFS_MMAPED) {
			mfc_debug_leave();
			return 0;
		}
		for (i = 0; i <= ctx->src_fmt->num_planes ; i++) {
			if (mfc_plane_cookie(vb, i) == 0) {
				mfc_err("Plane mem not allocated.\n");
				return -EINVAL;
			}
		}
		if (vb2_plane_size(vb, 0) < ctx->luma_size ||
			vb2_plane_size(vb, 1) < ctx->chroma_size) {
			mfc_err("Plane buffer (CAPTURE) is too small.\n");
			return -EINVAL;
		}
		mfc_debug("Size: 0=%lu 2=%lu\n", vb2_plane_size(vb, 0),
							vb2_plane_size(vb, 1));
		i = vb->v4l2_buf.index;
		ctx->dst_bufs[i].b = vb;
		ctx->dst_bufs[i].cookie.raw.luma = mfc_plane_cookie(vb, 0);
		ctx->dst_bufs[i].cookie.raw.chroma = mfc_plane_cookie(vb, 1);
		ctx->dst_bufs_cnt++;

	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (mfc_plane_cookie(vb, 0)  == 0) {
			mfc_err("Plane memory not allocated.\n");
			return -EINVAL;
		}
		mfc_debug("Plane size: %ld, ctx->dec_src_buf_size: %d\n",
				vb2_plane_size(vb, 0), ctx->dec_src_buf_size);
		if (vb2_plane_size(vb, 0) < ctx->dec_src_buf_size) {
			mfc_err("Plane buffer (OUTPUT) is too small.\n");
			return -EINVAL;
		}
		i = vb->v4l2_buf.index;
		ctx->src_bufs[i].b = vb;
		ctx->src_bufs[i].cookie.stream = mfc_plane_cookie(vb, 0);
		ctx->src_bufs_cnt++;
	} else {
		mfc_err("s5p_mfc_buf_init: unknown queue type.\n");
		return -EINVAL;
	}
	mfc_debug_leave();
	return 0;
}

static int s5p_mfc_start_streaming(struct vb2_queue *q)
{
	struct s5p_mfc_ctx *ctx = q->drv_priv;
	struct s5p_mfc_dev *dev = ctx->dev;
	unsigned long flags;

	/* If context is ready then dev = work->data;schedule it to run */
	if (s5p_mfc_ctx_ready(ctx)) {
		spin_lock_irqsave(&dev->condlock, flags);
		set_bit(ctx->num, &dev->ctx_work_bits);
		spin_unlock_irqrestore(&dev->condlock, flags);
	}

	s5p_mfc_try_run(dev);

	return 0;
}

static int s5p_mfc_stop_streaming(struct vb2_queue *q)
{
	unsigned long flags;
	struct s5p_mfc_ctx *ctx = q->drv_priv;
	struct s5p_mfc_dev *dev = ctx->dev;

	if ((ctx->state == MFCINST_FINISHING ||
		ctx->state ==  MFCINST_RUNNING) &&
		dev->curr_ctx == ctx->num && dev->hw_lock) {
		ctx->state = MFCINST_ABORT;
		s5p_mfc_wait_for_done_ctx(ctx, S5P_FIMV_R2H_CMD_FRAME_DONE_RET,
					  0);
	}
	ctx->state = MFCINST_FINISHED;

	spin_lock_irqsave(&dev->irqlock, flags);
	s5p_mfc_cleanup_queue(&ctx->dst_queue,
	        &ctx->vq_dst);
	s5p_mfc_cleanup_queue(&ctx->src_queue,
	        &ctx->vq_src);
	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		INIT_LIST_HEAD(&ctx->dst_queue);
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		INIT_LIST_HEAD(&ctx->src_queue);
	spin_unlock_irqrestore(&dev->irqlock, flags);

	return 0;
}


static void s5p_mfc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vq->drv_priv;
	struct s5p_mfc_dev *dev = ctx->dev;
	unsigned long flags;
	struct s5p_mfc_buf *mfc_buf;

	mfc_debug_enter();

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		mfc_buf = &ctx->src_bufs[vb->v4l2_buf.index];
		mfc_debug("Src queue: %p\n", &ctx->src_queue);
		mfc_debug("Adding to src: %p (%08lx, %08x)\n", vb,
				mfc_plane_cookie(vb, 0),
				ctx->src_bufs[vb->v4l2_buf.index].cookie.stream);
		spin_lock_irqsave(&dev->irqlock, flags);
		list_add_tail(&mfc_buf->list, &ctx->src_queue);
		ctx->src_queue_cnt++;
		spin_unlock_irqrestore(&dev->irqlock, flags);
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_buf = &ctx->dst_bufs[vb->v4l2_buf.index];
		mfc_debug("Dst queue: %p\n", &ctx->dst_queue);
		mfc_debug("Adding to dst: %p (%lx)\n", vb,
						  mfc_plane_cookie(vb, 0));
		mfc_debug("ADDING Flag before: %lx (%d)\n",
					ctx->dec_dst_flag, vb->v4l2_buf.index);
		/* Mark destination as available for use by MFC */
		spin_lock_irqsave(&dev->irqlock, flags);
		set_bit(vb->v4l2_buf.index, &ctx->dec_dst_flag);
		mfc_debug("ADDING Flag after: %lx\n", ctx->dec_dst_flag);
		list_add_tail(&mfc_buf->list, &ctx->dst_queue);
		ctx->dst_queue_cnt++;
		spin_unlock_irqrestore(&dev->irqlock, flags);
	} else {
		mfc_err("Unsupported buffer type (%d)\n", vq->type);
	}

	if (s5p_mfc_ctx_ready(ctx)) {
		spin_lock_irqsave(&dev->condlock, flags);
		set_bit(ctx->num, &dev->ctx_work_bits);
		spin_unlock_irqrestore(&dev->condlock, flags);
	}
	s5p_mfc_try_run(dev);

	mfc_debug_leave();
}

static struct vb2_ops s5p_mfc_dec_qops = {
	.queue_setup	= s5p_mfc_queue_setup,
	.wait_prepare	= s5p_mfc_unlock,
	.wait_finish	= s5p_mfc_lock,
	.buf_init	= s5p_mfc_buf_init,
	.start_streaming= s5p_mfc_start_streaming,
	.stop_streaming = s5p_mfc_stop_streaming,
	.buf_queue	= s5p_mfc_buf_queue,
};

struct s5p_mfc_codec_ops *get_dec_codec_ops(void)
{
	return &decoder_codec_ops;
}

struct vb2_ops *get_dec_queue_ops(void)
{
	return &s5p_mfc_dec_qops;
}

const struct v4l2_ioctl_ops *get_dec_v4l2_ioctl_ops(void)
{
	return &s5p_mfc_dec_ioctl_ops;
}

struct s5p_mfc_fmt *get_dec_def_fmt(bool src)
{
	if (src)
		return &formats[DEF_SRC_FMT];
	else
		return &formats[DEF_DST_FMT];
}

