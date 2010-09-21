/*
 * Samsung S5P Multi Format Codec v 5.0
 *
 * This file contains description of formats used by MFC and cotrols
 * used by the driver.
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 * Kamil Debski, <k.debski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version
 */

#ifndef S5P_MFC_CTRLS_H_
#define S5P_MFC_CTRLS_H_

#include <media/v4l2-ioctl.h>
#include "regs-mfc5.h"

#define MFC_FMT_DEC	0
#define MFC_FMT_ENC	1
#define MFC_FMT_RAW	2

struct s5p_mfc_fmt {
	char *name;
	u32 fourcc;
	u32 codec_mode;
	u32 type;
	u32 num_planes;
};

#define MFC_FORMATS_NO_CODEC -1

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
	.fourcc = V4L2_PIX_FMT_NV12,
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

static struct v4l2_queryctrl s5p_mfc_ctrls[] = {
/* For decoding */
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

#define NUM_CTRLS ARRAY_SIZE(s5p_mfc_ctrls)

#endif /* S5P_MFC_CTRLS_H_ */

