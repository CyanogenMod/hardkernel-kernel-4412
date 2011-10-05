/*
 * drivers/media/video/samsung/mfc5/s5p_mfc_opr.h
 *
 * Header file for Samsung MFC (Multi Function Codec - FIMV) driver
 * Contains declarations of hw related functions.
 *
 * Kamil Debski, Copyright (c) 2010 Samsung Electronics
 * http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef S5P_MFC_OPR_V6_H_
#define S5P_MFC_OPR_V6_H_

#include "s5p_mfc_common.h"
#include "s5p_mfc_mem.h"

#define MFC_CTRL_MODE_CUSTOM	MFC_CTRL_MODE_SFR

/*
int s5p_mfc_release_firmware(struct s5p_mfc_dev *dev);
int s5p_mfc_alloc_firmware(struct s5p_mfc_dev *dev);
int s5p_mfc_load_firmware(struct s5p_mfc_dev *dev);
int s5p_mfc_init_hw(struct s5p_mfc_dev *dev);
*/

int s5p_mfc_init_decode(struct s5p_mfc_ctx *ctx);
int s5p_mfc_init_encode(struct s5p_mfc_ctx *mfc_ctx);
/*
void s5p_mfc_deinit_hw(struct s5p_mfc_dev *dev);
int s5p_mfc_set_sleep(struct s5p_mfc_ctx *ctx);
int s5p_mfc_set_wakeup(struct s5p_mfc_ctx *ctx);
*/

int s5p_mfc_set_dec_frame_buffer(struct s5p_mfc_ctx *ctx);
int s5p_mfc_set_dec_stream_buffer(struct s5p_mfc_ctx *ctx, int buf_addr,
						  unsigned int start_num_byte,
						  unsigned int buf_size);

void s5p_mfc_set_enc_frame_buffer(struct s5p_mfc_ctx *ctx,
		unsigned long y_addr, unsigned long c_addr);
int s5p_mfc_set_enc_stream_buffer(struct s5p_mfc_ctx *ctx,
		unsigned long addr, unsigned int size);
void s5p_mfc_get_enc_frame_buffer(struct s5p_mfc_ctx *ctx,
		unsigned long *y_addr, unsigned long *c_addr);
int s5p_mfc_set_enc_ref_buffer(struct s5p_mfc_ctx *mfc_ctx);

int s5p_mfc_decode_one_frame(struct s5p_mfc_ctx *ctx, int last_frame);
int s5p_mfc_encode_one_frame(struct s5p_mfc_ctx *mfc_ctx);

/* Instance handling */
/*
int s5p_mfc_open_inst(struct s5p_mfc_ctx *ctx);
int s5p_mfc_return_inst_no(struct s5p_mfc_ctx *ctx);
*/

/* Memory allocation */
int s5p_mfc_alloc_dec_temp_buffers(struct s5p_mfc_ctx *ctx);
void s5p_mfc_set_dec_desc_buffer(struct s5p_mfc_ctx *ctx);
void s5p_mfc_release_dec_desc_buffer(struct s5p_mfc_ctx *ctx);

int s5p_mfc_alloc_codec_buffers(struct s5p_mfc_ctx *ctx);
void s5p_mfc_release_codec_buffers(struct s5p_mfc_ctx *ctx);

int s5p_mfc_alloc_instance_buffer(struct s5p_mfc_ctx *ctx);
void s5p_mfc_release_instance_buffer(struct s5p_mfc_ctx *ctx);

#define s5p_mfc_get_dspl_y_adr()	(readl(dev->regs_base + \
					S5P_FIMV_SI_DISPLAY_Y_ADR) << 11)
#define s5p_mfc_get_dspl_status()	readl(dev->regs_base + \
						S5P_FIMV_SI_DISPLAY_STATUS)
#define s5p_mfc_get_frame_type()	(readl(dev->regs_base + \
						S5P_FIMV_DECODE_FRAME_TYPE) \
					& S5P_FIMV_DECODE_FRAME_MASK)
#define s5p_mfc_get_consumed_stream()	readl(dev->regs_base + \
						S5P_FIMV_SI_CONSUMED_BYTES)
#define s5p_mfc_get_int_reason()	(readl(dev->regs_base + \
					S5P_FIMV_RISC2HOST_CMD) & \
					S5P_FIMV_RISC2HOST_CMD_MASK)
#define s5p_mfc_get_int_err()		readl(dev->regs_base + \
						S5P_FIMV_RISC2HOST_ARG2)
#define s5p_mfc_err_dec(x)		(((x) & S5P_FIMV_ERR_DEC_MASK) >> \
						S5P_FIMV_ERR_DEC_SHIFT)
#define s5p_mfc_err_dspl(x)		(((x) & S5P_FIMV_ERR_DSPL_MASK) >> \
						S5P_FIMV_ERR_DSPL_SHIFT)
#define s5p_mfc_get_img_width()		readl(dev->regs_base + \
						S5P_FIMV_SI_HRESOL)
#define s5p_mfc_get_img_height()	readl(dev->regs_base + \
						S5P_FIMV_SI_VRESOL)
#define s5p_mfc_get_dpb_count()		readl(dev->regs_base + \
						S5P_FIMV_SI_BUF_NUMBER)
#define s5p_mfc_get_inst_no()		readl(dev->regs_base + \
						S5P_FIMV_RISC2HOST_ARG1)
#define s5p_mfc_get_enc_strm_size()	readl(dev->regs_base + \
						S5P_FIMV_ENC_SI_STRM_SIZE)
#define s5p_mfc_get_enc_slice_type()	readl(dev->regs_base + \
						S5P_FIMV_ENC_SI_SLICE_TYPE)

#define s5p_mfc_clear_int_flags()				\
	do {							\
		s5p_mfc_write_reg(0, S5P_FIMV_RISC2HOST_CMD);	\
		s5p_mfc_write_reg(0, S5P_FIMV_RISC2HOST_INT);	\
	} while (0)

/* FIXME: temporal definition to avoid compile error */
enum MFC_SHM_OFS
{
	PIC_TIME_TOP		= 0x10, /* D */
	PIC_TIME_BOT		= 0x14, /* D */
	START_BYTE_NUM		= 0x18, /* D */

	CROP_INFO_H		= 0x20, /* D */
	CROP_INFO_V		= 0x24, /* D */
	EXT_ENC_CONTROL		= 0x28,	/* E */
	RC_VOP_TIMING		= 0x30,	/* E, MPEG4 */

	ALLOC_LUMA_DPB_SIZE	= 0x64,	/* D */
	ALLOC_CHROMA_DPB_SIZE	= 0x68,	/* D */
	ALLOC_MV_SIZE		= 0x6C,	/* D */
	P_B_FRAME_QP		= 0x70,	/* E */
	ASPECT_RATIO_IDC	= 0x74, /* E, H.264, depend on ASPECT_RATIO_VUI_ENABLE in EXT_ENC_CONTROL */
	EXTENDED_SAR		= 0x78, /* E, H.264, depned on ASPECT_RATIO_VUI_ENABLE in EXT_ENC_CONTROL */

	H264_I_PERIOD		= 0x9C, /* E, H.264, open GOP */
	RC_CONTROL_CONFIG	= 0xA0, /* E */
};

void s5p_mfc_try_run(struct s5p_mfc_dev *dev);

void s5p_mfc_cleanup_queue(struct list_head *lh, struct vb2_queue *vq);

void s5p_mfc_write_info(struct s5p_mfc_ctx *ctx, unsigned int data, unsigned int ofs);
unsigned int s5p_mfc_read_info(struct s5p_mfc_ctx *ctx, unsigned int ofs);

#endif /* S5P_MFC_OPR_V6_H_ */
