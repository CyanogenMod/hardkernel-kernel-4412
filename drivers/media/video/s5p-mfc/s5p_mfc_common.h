/*
 * Samsung S5P Multi Format Codec v 5.0
 *
 * This file contains definitions of enums and structs used by the codec
 * driver.
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 * Kamil Debski, <k.debski@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version
 */

#ifndef S5P_MFC_COMMON_H_
#define S5P_MFC_COMMON_H_

#include "regs-mfc5.h"
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include <media/videobuf2-core.h>

#define MFC_MAX_EXTRA_DPB       5
#define MFC_MAX_BUFFERS		32
#define MFC_FRAME_PLANES	2

#define MFC_NUM_CONTEXTS	4
/* Interrupt timeout */
#define MFC_INT_TIMEOUT		2000
/* Busy wait timeout */
#define MFC_BW_TIMEOUT		500
/* Watchdog interval */
#define MFC_WATCHDOG_INTERVAL   1000
/* After how many executions watchdog should assume lock up */
#define MFC_WATCHDOG_CNT        10

#define MFC_NO_INSTANCE_SET	-1

/**
 * enum s5p_mfc_inst_type - The type of an MFC device node.
 */
enum s5p_mfc_node_type {
	MFCNODE_INVALID = -1,
	MFCNODE_DECODER = 0,
	MFCNODE_ENCODER = 1,
};

/**
 * enum s5p_mfc_inst_type - The type of an MFC instance.
 */
enum s5p_mfc_inst_type {
	MFCINST_INVALID = 0,
	MFCINST_DECODER = 1,
	MFCINST_ENCODER = 2,
};

/**
 * enum s5p_mfc_inst_state - The state of an MFC instance.
 */
enum s5p_mfc_inst_state {
	MFCINST_FREE = 0,
	MFCINST_INIT = 100,
	MFCINST_GOT_INST,
	MFCINST_HEAD_PARSED,
	MFCINST_BUFS_SET,
	MFCINST_RUNNING,
	MFCINST_FINISHING,
	MFCINST_FINISHED,
	MFCINST_RETURN_INST,
	MFCINST_ERROR,
	MFCINST_ABORT,
};

/**
 * enum s5p_mfc_queue_state - The state of buffer queue.
 */
enum s5p_mfc_queue_state {
	QUEUE_FREE = 0,
	QUEUE_BUFS_REQUESTED,
	QUEUE_BUFS_QUERIED,
	QUEUE_BUFS_MMAPED,
};

struct s5p_mfc_ctx;

/**
 * struct s5p_mfc_buf - MFC buffer
 *
 */
struct s5p_mfc_buf {
	struct list_head list;
	struct vb2_buffer *b;
	union {
		struct {
			size_t luma;
			size_t chroma;
		} raw;
		size_t stream;
	} paddr;
};

/**
 * struct s5p_mfc_dev - The struct containing driver internal parameters.
 */
struct s5p_mfc_dev {
	struct v4l2_device v4l2_dev;
	struct video_device *vfd_dec;
	struct video_device *vfd_enc;
	struct platform_device *plat_dev;

	int num_inst;
	spinlock_t irqlock;
	spinlock_t condlock;

	void __iomem *regs_base;
	int irq;

	struct resource *mfc_mem;
	void __iomem *base_virt_addr;

	struct mutex *mfc_mutex;

	int int_cond;
	int int_type;
	unsigned int int_err;
	wait_queue_head_t queue;

	size_t port_a;
	size_t port_b;

	unsigned long hw_lock;

	struct clk *clock1;
	struct clk *clock2;

	struct s5p_mfc_ctx *ctx[MFC_NUM_CONTEXTS];
	int curr_ctx;
	unsigned long ctx_work_bits;

	atomic_t watchdog_cnt;
	struct timer_list watchdog_timer;
	struct workqueue_struct *watchdog_workqueue;
	struct work_struct watchdog_work;

	struct vb2_alloc_ctx **alloc_ctx;
};

/**
 *
 */
struct s5p_mfc_h264_enc_params {
	u8 num_b_frame;
	enum v4l2_codec_mfc5x_enc_h264_profile profile;
	u8 level;
	enum v4l2_codec_mfc5x_enc_switch interlace;
	enum v4l2_codec_mfc5x_enc_h264_loop_filter loop_filter_mode;
	s8 loop_filter_alpha;
	s8 loop_filter_beta;
	enum v4l2_codec_mfc5x_enc_h264_entropy_mode entropy_mode;
	u8 max_ref_pic;
	u8 num_ref_pic_4p;
	enum v4l2_codec_mfc5x_enc_switch _8x8_transform;
	enum v4l2_codec_mfc5x_enc_switch rc_mb;
	u32 rc_framerate;
	u8 rc_frame_qp;
	u8 rc_min_qp;
	u8 rc_max_qp;
	enum v4l2_codec_mfc5x_enc_switch_inv rc_mb_dark;
	enum v4l2_codec_mfc5x_enc_switch_inv rc_mb_smooth;
	enum v4l2_codec_mfc5x_enc_switch_inv rc_mb_static;
	enum v4l2_codec_mfc5x_enc_switch_inv rc_mb_activity;
	u8 rc_p_frame_qp;
	u8 rc_b_frame_qp;
	enum v4l2_codec_mfc5x_enc_switch ar_vui;
	u8 ar_vui_idc;
	u16 ext_sar_width;
	u16 ext_sar_height;
	enum v4l2_codec_mfc5x_enc_switch open_gop;
	u16 open_gop_size;
};

/**
 *
 */
struct s5p_mfc_enc_params {
	u16 width;
	u16 height;

	u16 gop_size;
	enum v4l2_codec_mfc5x_enc_multi_slice_mode slice_mode;
	u16 slice_mb;
	u32 slice_bit;
	u16 intra_refresh_mb;
	enum v4l2_codec_mfc5x_enc_switch pad;
	u8 pad_luma;
	u8 pad_cb;
	u8 pad_cr;
	enum v4l2_codec_mfc5x_enc_switch rc_frame;
	u32 rc_bitrate;
	u16 rc_reaction_coeff;

	u16 vbv_buf_size;
	enum v4l2_codec_mfc5x_enc_seq_hdr_mode seq_hdr_mode;
	enum v4l2_codec_mfc5x_enc_frame_skip_mode frame_skip_mode;
	enum v4l2_codec_mfc5x_enc_switch fixed_target_bit;

	union {
		struct s5p_mfc_h264_enc_params h264;
	} codec;
};

struct s5p_mfc_codec_ops {
	/* initialization routines */
	int (*alloc_ctx_buf) (struct s5p_mfc_ctx *ctx);
	int (*alloc_desc_buf) (struct s5p_mfc_ctx *ctx);
	int (*get_init_arg) (struct s5p_mfc_ctx *ctx, void *arg);
	int (*pre_seq_start) (struct s5p_mfc_ctx *ctx);
	int (*post_seq_start) (struct s5p_mfc_ctx *ctx);
	int (*set_init_arg) (struct s5p_mfc_ctx *ctx, void *arg);
	int (*set_codec_bufs) (struct s5p_mfc_ctx *ctx);
	int (*set_dpbs) (struct s5p_mfc_ctx *ctx);		/* decoder */
	/* execution routines */
	int (*get_exe_arg) (struct s5p_mfc_ctx *ctx, void *arg);
	int (*pre_frame_start) (struct s5p_mfc_ctx *ctx);
	int (*post_frame_start) (struct s5p_mfc_ctx *ctx);
	int (*multi_data_frame) (struct s5p_mfc_ctx *ctx);
	int (*set_exe_arg) (struct s5p_mfc_ctx *ctx, void *arg);
	/* configuration routines */
	int (*get_codec_cfg) (struct s5p_mfc_ctx *ctx, unsigned int type, int *value);
	int (*set_codec_cfg) (struct s5p_mfc_ctx *ctx, unsigned int type, int *value);
};

/**
 * struct s5p_mfc_ctx - This struct contains the instance context
 */
struct s5p_mfc_ctx {
	struct s5p_mfc_dev *dev;
	int num;

	int int_cond;
	int int_type;
	unsigned int int_err;
	wait_queue_head_t queue;

	struct s5p_mfc_fmt *src_fmt;
	struct s5p_mfc_fmt *dst_fmt;

	struct vb2_queue vq_src;
	struct vb2_queue vq_dst;

	struct list_head src_queue;
	struct list_head dst_queue;

	unsigned int src_queue_cnt;
	unsigned int dst_queue_cnt;

	enum s5p_mfc_inst_type type;
	enum s5p_mfc_inst_state state;
	int inst_no;

	/* Decoder parameters */
	int img_width;
	int img_height;
	int buf_width;
	int buf_height;
	int dpb_count;
	int total_dpb_count;

	int luma_size;
	int chroma_size;
	int mv_size;

	unsigned long consumed_stream;
	int slice_interface;

	/* Buffers */
	void *port_a_buf;
	size_t port_a_phys;
	size_t port_a_size;

	void *port_b_buf;
	size_t port_b_phys;
	size_t port_b_size;


	enum s5p_mfc_queue_state capture_state;
	enum s5p_mfc_queue_state output_state;

//	size_t dec_dst_buf_luma[MFC_MAX_BUFFERS];
//	size_t dec_dst_buf_chroma[MFC_MAX_BUFFERS];

	struct s5p_mfc_buf src_bufs[MFC_MAX_BUFFERS];
	int src_bufs_cnt;
	struct s5p_mfc_buf dst_bufs[MFC_MAX_BUFFERS];
	int dst_bufs_cnt;

//	int dec_dst_buf_cnt;
	unsigned int sequence;
	unsigned long dec_dst_flag;
	size_t dec_src_buf_size;

	/* Control values */
	int codec_mode;
	__u32 pix_format;
	int loop_filter_mpeg4;
	int display_delay;

	/* Buffers */

	void *instance_buf;
	size_t instance_phys;
	size_t instance_size;

	void *desc_buf;
	size_t desc_phys;

	void *shared_buf;
	size_t shared_phys;
	void *shared_virt;

	struct s5p_mfc_enc_params enc_params;

	size_t enc_dst_buf_size;

	int frame_count;
	enum v4l2_codec_mfc5x_enc_frame_type frame_type;
	enum v4l2_codec_mfc5x_enc_force_frame_type force_frame_type;

	struct s5p_mfc_codec_ops *c_ops;
};

#endif /* S5P_MFC_COMMON_H_ */
