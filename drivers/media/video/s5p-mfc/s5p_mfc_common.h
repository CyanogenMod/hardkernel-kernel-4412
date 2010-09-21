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
 * enum s5p_mfc_inst_state - The state of an MFC instance.
 */
enum s5p_mfc_inst_state {
	MFCINST_FREE = 0,
	MFCINST_DEC_INIT = 100,
	MFCINST_DEC_GOT_INST,
	MFCINST_DEC_HEAD_PARSED,
	MFCINST_DEC_BUFS_SET,
	MFCINST_DEC_RUNNING,
	MFCINST_DEC_FINISHING,
	MFCINST_DEC_FINISHED,
	MFCINST_DEC_RETURN_INST,
	MFCINST_DEC_ERROR,
	MFCINST_DEC_ABORT,
	MFCINST_ENC_INIT = 200,
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
	struct video_device *vfd;
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
 * struct s5p_mfc_ctx - This struct contains the instance context
 */
struct s5p_mfc_ctx {
	struct s5p_mfc_dev *dev;
	int num;

	int int_cond;
	int int_type;
	unsigned int int_err;
	wait_queue_head_t queue;

	struct s5p_mfc_fmt *fmt;

	struct vb2_queue vq_src;
	struct vb2_queue vq_dst;

	struct list_head src_queue;
	struct list_head dst_queue;

	unsigned int src_queue_cnt;
	unsigned int dst_queue_cnt;

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

};

#endif /* S5P_MFC_COMMON_H_ */
