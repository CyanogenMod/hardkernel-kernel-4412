/*
 * drivers/media/video/samsung/mfc5/s5p_mfc_opr.c
 *
 * Samsung MFC (Multi Function Codec - FIMV) driver
 * This file contains hw related functions.
 *
 * Kamil Debski, Copyright (c) 2010 Samsung Electronics
 * http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include "regs-mfc5.h"

#include "s5p_mfc_opr.h"
#include "s5p_mfc_common.h"
#include "s5p_mfc_memory.h"
#include "s5p_mfc_intr.h"
#include "s5p_mfc_debug.h"

#include <linux/firmware.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/cma.h>

#include <asm/cacheflush.h>

#include <media/videobuf2-cma.h>

static void *s5p_mfc_bitproc_buf;
static size_t s5p_mfc_bitproc_phys;
static unsigned char *s5p_mfc_bitproc_virt;

/* #define S5P_MFC_DEBUG_REGWRITE  */
#ifdef S5P_MFC_DEBUG_REGWRITE
#undef writel
#define writel(v, r) do { \
	printk(KERN_ERR "MFCWRITE(%p): %08x\n", r, (unsigned int)v); \
	__raw_writel(v, r); } while (0)
#endif /* S5P_MFC_DEBUG_REGWRITE */

#define READL(offset)		readl(dev->regs_base + (offset))
#define WRITEL(data, offset)	writel((data), dev->regs_base + (offset))
#define OFFSETA(x)		(((x) - dev->port_a) >> S5P_FIMV_MEM_OFFSET)
#define OFFSETB(x)		(((x) - dev->port_b) >> S5P_FIMV_MEM_OFFSET)

static inline void *s5p_mfc_mem_alloc(void *a, unsigned int s)
{
	return vb2_cma_memops.alloc(a, s);
}

static inline size_t s5p_mfc_mem_paddr(void *a, void *b)
{
	return (size_t)vb2_cma_memops.cookie(b);
}

static inline void s5p_mfc_mem_put(void *a, void *b)
{
	vb2_cma_memops.put(b);
}

static inline void *s5p_mfc_mem_vaddr(void *a, void *b)
{
	return vb2_cma_memops.vaddr(b);
}

/* Reset the device */
static int s5p_mfc_cmd_reset(struct s5p_mfc_dev *dev)
{
	unsigned int mc_status;
	unsigned long timeout;

	mfc_debug_enter();
	/* Stop procedure */
	WRITEL(0x3f6, S5P_FIMV_SW_RESET);	/*  reset RISC */
	WRITEL(0x3e2, S5P_FIMV_SW_RESET);	/*  All reset except for MC */
	mdelay(10);
	timeout = jiffies + msecs_to_jiffies(MFC_BW_TIMEOUT);
	/* Check MC status */
	do {
		if (time_after(jiffies, timeout)) {
			mfc_err("Timeout while resetting MFC.\n");
			return -EIO;
		}
		mc_status = READL(S5P_FIMV_MC_STATUS);
	} while (mc_status & 0x3);
	WRITEL(0x0, S5P_FIMV_SW_RESET);
	WRITEL(0x3fe, S5P_FIMV_SW_RESET);
	mfc_debug_leave();
	return 0;
}

/* Send a command to the MFC */
static int s5p_mfc_cmd_host2risc(struct s5p_mfc_dev *dev,
				struct s5p_mfc_ctx *ctx, int cmd, int arg)
{
	int cur_cmd;
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(MFC_BW_TIMEOUT);
	/* wait until host to risc command register becomes 'H2R_CMD_EMPTY' */
	do {
		if (time_after(jiffies, timeout)) {
			mfc_err("Timeout while waiting for hardware.\n");
			return -EIO;
		}
		cur_cmd = READL(S5P_FIMV_HOST2RISC_CMD);
	} while (cur_cmd != S5P_FIMV_H2R_CMD_EMPTY);
	WRITEL(arg, S5P_FIMV_HOST2RISC_ARG1);
	if (cmd == S5P_FIMV_H2R_CMD_OPEN_INSTANCE) {
		/* No CRC calculation (slow!) */
		WRITEL(0, S5P_FIMV_HOST2RISC_ARG2);
		/* Physical addr of the instance buffer */
		WRITEL(OFFSETA(ctx->instance_phys),
		       S5P_FIMV_HOST2RISC_ARG3);
		/* Size of the instance buffer */
		WRITEL(ctx->instance_size, S5P_FIMV_HOST2RISC_ARG4);
	}
	/* Issue the command */
	WRITEL(cmd, S5P_FIMV_HOST2RISC_CMD);
	return 0;
}
/*
static void s5p_mfc_cmd_sleep()
{
	WRITEL(-1, S5P_FIMV_CH_ID);
	WRITEL(MFC_SLEEP, S5P_FIMV_COMMAND_TYPE);
}
*/

/*
static void s5p_mfc_cmd_wakeup()
{
	WRITEL(-1, S5P_FIMV_CH_ID);
	WRITEL(MFC_WAKEUP, S5P_FIMV_COMMAND_TYPE);
	mdelay(100);
}
*/


/* Allocate temporary buffers for decoding */
int s5p_mfc_alloc_dec_temp_buffers(struct s5p_mfc_ctx *ctx)
{
	void *desc_virt;
	struct s5p_mfc_dev *dev = ctx->dev;
	mfc_debug_enter();
	ctx->desc_buf = s5p_mfc_mem_alloc(
			dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX], DESC_BUF_SIZE);
	if (IS_ERR_VALUE((int)ctx->desc_buf)) {
		ctx->desc_buf = 0;
		mfc_err("Allocating DESC buffer failed.\n");
		return -ENOMEM;
	}
	ctx->desc_phys = s5p_mfc_mem_paddr(
			dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX], ctx->desc_buf);
	desc_virt = s5p_mfc_mem_vaddr(
			dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX], ctx->desc_buf);
	if (desc_virt == NULL) {
		s5p_mfc_mem_put(
			dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX], ctx->desc_buf);
		ctx->desc_phys = 0;
		ctx->desc_buf = 0;
		mfc_err("Remapping DESC buffer failed.\n");
		return -ENOMEM;
	}
	/* Zero content of the allocated memory */
	memset(desc_virt, 0, DESC_BUF_SIZE);
	flush_cache_all();
	mfc_debug_leave();
	return 0;
}

/* Release temproary buffers for decoding */
void s5p_mfc_release_dec_desc_buffer(struct s5p_mfc_ctx *ctx)
{
	if (ctx->desc_phys) {
		s5p_mfc_mem_put(ctx->dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX],
								ctx->desc_buf);
		ctx->desc_phys = 0;
		ctx->desc_buf = 0;
	}
}

/* Allocate decoding buffers */
int s5p_mfc_alloc_dec_buffers(struct s5p_mfc_ctx *ctx)
{
	unsigned int luma_size, chroma_size, mv_size;
	struct s5p_mfc_dev *dev = ctx->dev;

	mfc_debug_enter();
	luma_size = ctx->luma_size;
	chroma_size = ctx->chroma_size;
	mv_size = ctx->mv_size;
	mfc_debug("Luma size:%d Chroma size:%d MV size:%d Totals bufs: %d\n",
		  luma_size, chroma_size, mv_size, ctx->total_dpb_count);
	/* Codecs have different memory requirements */
	switch (ctx->codec_mode) {
	case S5P_FIMV_CODEC_H264_DEC:
		ctx->port_a_size =
		    ALIGN(S5P_FIMV_DEC_NB_IP_SIZE +
					S5P_FIMV_DEC_VERT_NB_MV_SIZE,
					S5P_FIMV_DEC_BUF_ALIGN);
		/* TODO, when merged with FIMC then test will it work without
		 * alignment to 8192. For all codecs. */
		ctx->port_b_size =
		    ctx->total_dpb_count * mv_size;
		break;
	case S5P_FIMV_CODEC_MPEG4_DEC:
	case S5P_FIMV_CODEC_DIVX412_DEC:
	case S5P_FIMV_CODEC_DIVX311_DEC:
	case S5P_FIMV_CODEC_DIVX502_DEC:
	case S5P_FIMV_CODEC_DIVX503_DEC:
		ctx->port_a_size =
		    ALIGN(S5P_FIMV_DEC_NB_DCAC_SIZE +
				     S5P_FIMV_DEC_UPNB_MV_SIZE +
				     S5P_FIMV_DEC_SUB_ANCHOR_MV_SIZE +
				     S5P_FIMV_DEC_STX_PARSER_SIZE +
				     S5P_FIMV_DEC_OVERLAP_TRANSFORM_SIZE,
				     S5P_FIMV_DEC_BUF_ALIGN);
		ctx->port_b_size = 0;
		break;

	case S5P_FIMV_CODEC_VC1RCV_DEC:
	case S5P_FIMV_CODEC_VC1_DEC:
		ctx->port_a_size =
		    ALIGN(S5P_FIMV_DEC_OVERLAP_TRANSFORM_SIZE +
			     S5P_FIMV_DEC_UPNB_MV_SIZE +
			     S5P_FIMV_DEC_SUB_ANCHOR_MV_SIZE +
			     S5P_FIMV_DEC_NB_DCAC_SIZE +
			     3 * S5P_FIMV_DEC_VC1_BITPLANE_SIZE,
			     S5P_FIMV_DEC_BUF_ALIGN);
		ctx->port_b_size = 0;
		break;

	case S5P_FIMV_CODEC_MPEG2_DEC:
		ctx->port_a_size = 0;
		ctx->port_b_size = 0;
		break;
	case S5P_FIMV_CODEC_H263_DEC:
		ctx->port_a_size =
		    ALIGN(S5P_FIMV_DEC_OVERLAP_TRANSFORM_SIZE +
			     S5P_FIMV_DEC_UPNB_MV_SIZE +
			     S5P_FIMV_DEC_SUB_ANCHOR_MV_SIZE +
			     S5P_FIMV_DEC_NB_DCAC_SIZE,
			     S5P_FIMV_DEC_BUF_ALIGN);
		ctx->port_b_size = 0;
		break;
	default:
		break;
	}

	/* Allocate only if memory from bank 1 is necessary */
	if (ctx->port_a_size > 0) {
		ctx->port_a_buf = s5p_mfc_mem_alloc(
		dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX], ctx->port_a_size);
		if (IS_ERR(ctx->port_a_buf)) {
			ctx->port_a_buf = 0;
			printk(KERN_ERR
			       "Buf alloc for decoding failed (port A).\n");
			return -ENOMEM;
		}
		ctx->port_a_phys = s5p_mfc_mem_paddr(
		dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX], ctx->port_a_buf);
	}

	/* Allocate only if memory from bank 2 is necessary */
	if (ctx->port_b_size > 0) {
		ctx->port_b_buf = s5p_mfc_mem_alloc(
		dev->alloc_ctx[MFC_CMA_BANK2_ALLOC_CTX], ctx->port_b_size);
		if (IS_ERR(ctx->port_b_buf)) {
			ctx->port_b_buf = 0;
			mfc_err("Buf alloc for decoding failed (port B).\n");
			return -ENOMEM;
		}
		ctx->port_b_phys = s5p_mfc_mem_paddr(
		dev->alloc_ctx[MFC_CMA_BANK2_ALLOC_CTX], ctx->port_b_buf);
	}
	mfc_debug_leave();

	return 0;
}

/* Release buffers allocated for decoding */
void s5p_mfc_release_dec_buffers(struct s5p_mfc_ctx *ctx)
{
	if (ctx->port_a_buf) {
		s5p_mfc_mem_put(ctx->dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX],
							ctx->port_a_buf);
		ctx->port_a_buf = 0;
		ctx->port_a_phys = 0;
		ctx->port_a_size = 0;
	}
	if (ctx->port_b_buf) {
		s5p_mfc_mem_put(ctx->dev->alloc_ctx[MFC_CMA_BANK2_ALLOC_CTX],
							ctx->port_b_buf);
		ctx->port_b_buf = 0;
		ctx->port_b_phys = 0;
		ctx->port_b_size = 0;
	}
}

/* Allocate memory for instance data buffer */
int s5p_mfc_alloc_instance_buffer(struct s5p_mfc_ctx *ctx)
{
	void *instance_virt;
	struct s5p_mfc_dev *dev = ctx->dev;

	mfc_debug_enter();
	if (ctx->codec_mode == S5P_FIMV_CODEC_H264_DEC ||
		ctx->codec_mode == S5P_FIMV_CODEC_H264_ENC)
		ctx->instance_size = MFC_H264_INSTANCE_BUF_SIZE;
	else
		ctx->instance_size = MFC_INSTANCE_BUF_SIZE;
	ctx->instance_buf = s5p_mfc_mem_alloc(
		dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX], ctx->instance_size);
	if (IS_ERR(ctx->instance_buf)) {
		mfc_err("Allocating instance buffer failed.\n");
		ctx->instance_phys = 0;
		ctx->instance_buf = 0;
		return -ENOMEM;
	}
	ctx->instance_phys = s5p_mfc_mem_paddr(
		dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX], ctx->instance_buf);
	instance_virt = s5p_mfc_mem_vaddr(
		dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX], ctx->instance_buf);
	if (instance_virt == NULL) {
		mfc_err("Remapping instance buffer failed.\n");
		s5p_mfc_mem_put(dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX],
							ctx->instance_buf);
		ctx->instance_phys = 0;
		ctx->instance_buf = 0;
		return -ENOMEM;
	}
	/* Zero content of the allocated memory */
	memset(instance_virt, 0, ctx->instance_size);
	flush_cache_all();
	ctx->shared_buf = s5p_mfc_mem_alloc(
		dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX], SHARED_BUF_SIZE);
	if (IS_ERR(ctx->shared_buf)) {
		mfc_err("Allocating shared buffer failed\n");
		ctx->shared_buf = 0;
		s5p_mfc_mem_put(dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX],
							ctx->instance_buf);
		ctx->instance_phys = 0;
		ctx->instance_buf = 0;
		return -ENOMEM;
	}
	ctx->shared_phys = s5p_mfc_mem_paddr(
		dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX], ctx->shared_buf);
	ctx->shared_virt = s5p_mfc_mem_vaddr(
		dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX], ctx->shared_buf);
	if (!ctx->shared_virt) {
		mfc_err("Remapping shared buffer failed\n");
		s5p_mfc_mem_put(dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX],
							ctx->shared_buf);
		ctx->shared_phys = 0;
		ctx->shared_buf = 0;
		s5p_mfc_mem_put(dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX],
							ctx->instance_buf);
		ctx->instance_phys = 0;
		ctx->instance_buf = 0;
		return -ENOMEM;
	}
	/* Zero content of the allocated memory */
	memset((void *)ctx->shared_virt, 0, SHARED_BUF_SIZE);
	flush_cache_all();
	mfc_debug_leave();
	return 0;
}

/* Release instance buffer */
void s5p_mfc_release_instance_buffer(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;

	mfc_debug_enter();
	if (ctx->instance_buf) {
		s5p_mfc_mem_put(dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX],
							ctx->instance_buf);
		ctx->instance_phys = 0;
		ctx->instance_buf = 0;
	}
	if (ctx->shared_phys) {
		s5p_mfc_mem_put(dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX],
							ctx->shared_buf);
		ctx->shared_phys = 0;
		ctx->shared_buf = 0;
	}
	mfc_debug_leave();
}

/* Set registers for decoding temporary buffers */
void s5p_mfc_set_dec_desc_buffer(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;

	WRITEL(OFFSETA(ctx->desc_phys), S5P_FIMV_SI_CH0_DESC_ADR);
	WRITEL(DESC_BUF_SIZE, S5P_FIMV_SI_CH0_DESC_SIZE);
}

/* Set registers for shared buffer */
void s5p_mfc_set_shared_buffer(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;

	WRITEL(ctx->shared_phys - ctx->dev->port_a,
	       S5P_FIMV_SI_CH0_HOST_WR_ADR);
}

/* Set registers for decoding stream buffer */
int s5p_mfc_set_dec_stream_buffer(struct s5p_mfc_ctx *ctx, int buf_addr,
		  unsigned int start_num_byte, unsigned int buf_size)
{
	struct s5p_mfc_dev *dev = ctx->dev;

	mfc_debug_enter();
	mfc_debug("inst_no: %d, buf_addr: 0x%08x, buf_size: 0x"
		"%08x (%d)\n",  ctx->inst_no, buf_addr, buf_size, buf_size);
	WRITEL(OFFSETA(buf_addr), S5P_FIMV_SI_CH0_SB_ST_ADR);
	WRITEL(CPB_BUF_SIZE, S5P_FIMV_SI_CH0_CPB_SIZE);
	WRITEL(buf_size, S5P_FIMV_SI_CH0_SB_FRM_SIZE);
	mfc_debug("Shared_virt: %p (start offset: %d)\n",
					ctx->shared_virt, start_num_byte);
	s5p_mfc_set_start_num(ctx, start_num_byte);
	mfc_debug_leave();
	return 0;
}

/* Set decoding frame buffer */
int s5p_mfc_set_dec_frame_buffer(struct s5p_mfc_ctx *ctx)
{
	unsigned int frame_size, i;
	unsigned int frame_size_ch, frame_size_mv;
	struct s5p_mfc_dev *dev = ctx->dev;
	unsigned int dpb;
	size_t buf_addr1, buf_addr2;
	int buf_size1, buf_size2;

	buf_addr1 = ctx->port_a_phys;
	buf_size1 = ctx->port_a_size;
	buf_addr2 = ctx->port_b_phys;
	buf_size2 = ctx->port_b_size;
	mfc_debug("Buf1: %p (%d) Buf2: %p (%d)\n", (void *)buf_addr1, buf_size1,
						(void *)buf_addr2, buf_size2);
	mfc_debug("Total DPB COUNT: %d\n", ctx->total_dpb_count);
	mfc_debug("Setting display delay to %d\n", ctx->display_delay);
	dpb = READL(S5P_FIMV_SI_CH0_DPB_CONF_CTRL) & ~S5P_FIMV_DPB_COUNT_MASK;
	WRITEL(ctx->total_dpb_count | dpb, S5P_FIMV_SI_CH0_DPB_CONF_CTRL);
	s5p_mfc_set_shared_buffer(ctx);
	switch (ctx->codec_mode) {
	case S5P_FIMV_CODEC_H264_DEC:
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_VERT_NB_MV_ADR);
		buf_addr1 += S5P_FIMV_DEC_VERT_NB_MV_SIZE;
		buf_size1 -= S5P_FIMV_DEC_VERT_NB_MV_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_VERT_NB_IP_ADR);
		buf_addr1 += S5P_FIMV_DEC_NB_IP_SIZE;
		buf_size1 -= S5P_FIMV_DEC_NB_IP_SIZE;
		break;
	case S5P_FIMV_CODEC_MPEG4_DEC:
	case S5P_FIMV_CODEC_DIVX311_DEC:
	case S5P_FIMV_CODEC_DIVX412_DEC:
	case S5P_FIMV_CODEC_DIVX502_DEC:
	case S5P_FIMV_CODEC_DIVX503_DEC:
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_NB_DCAC_ADR);
		buf_addr1 += S5P_FIMV_DEC_NB_DCAC_SIZE;
		buf_size1 -= S5P_FIMV_DEC_NB_DCAC_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_UP_NB_MV_ADR);
		buf_addr1 += S5P_FIMV_DEC_UPNB_MV_SIZE;
		buf_size1 -= S5P_FIMV_DEC_UPNB_MV_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_SA_MV_ADR);
		buf_addr1 += S5P_FIMV_DEC_SUB_ANCHOR_MV_SIZE;
		buf_size1 -= S5P_FIMV_DEC_SUB_ANCHOR_MV_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_SP_ADR);
		buf_addr1 += S5P_FIMV_DEC_STX_PARSER_SIZE;
		buf_size1 -= S5P_FIMV_DEC_STX_PARSER_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_OT_LINE_ADR);
		buf_addr1 += S5P_FIMV_DEC_OVERLAP_TRANSFORM_SIZE;
		buf_size1 -= S5P_FIMV_DEC_OVERLAP_TRANSFORM_SIZE;
		break;
	case S5P_FIMV_CODEC_H263_DEC:
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_OT_LINE_ADR);
		buf_addr1 += S5P_FIMV_DEC_OVERLAP_TRANSFORM_SIZE;
		buf_size1 -= S5P_FIMV_DEC_OVERLAP_TRANSFORM_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_UP_NB_MV_ADR);
		buf_addr1 += S5P_FIMV_DEC_UPNB_MV_SIZE;
		buf_size1 -= S5P_FIMV_DEC_UPNB_MV_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_SA_MV_ADR);
		buf_addr1 += S5P_FIMV_DEC_SUB_ANCHOR_MV_SIZE;
		buf_size1 -= S5P_FIMV_DEC_SUB_ANCHOR_MV_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_NB_DCAC_ADR);
		buf_addr1 += S5P_FIMV_DEC_NB_DCAC_SIZE;
		buf_size1 -= S5P_FIMV_DEC_NB_DCAC_SIZE;
		break;
	case S5P_FIMV_CODEC_VC1_DEC:
	case S5P_FIMV_CODEC_VC1RCV_DEC:
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_NB_DCAC_ADR);
		buf_addr1 += S5P_FIMV_DEC_NB_DCAC_SIZE;
		buf_size1 -= S5P_FIMV_DEC_NB_DCAC_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_OT_LINE_ADR);
		buf_addr1 += S5P_FIMV_DEC_OVERLAP_TRANSFORM_SIZE;
		buf_size1 -= S5P_FIMV_DEC_OVERLAP_TRANSFORM_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_UP_NB_MV_ADR);
		buf_addr1 += S5P_FIMV_DEC_UPNB_MV_SIZE;
		buf_size1 -= S5P_FIMV_DEC_UPNB_MV_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_SA_MV_ADR);
		buf_addr1 += S5P_FIMV_DEC_SUB_ANCHOR_MV_SIZE;
		buf_size1 -= S5P_FIMV_DEC_SUB_ANCHOR_MV_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_BITPLANE3_ADR);
		buf_addr1 += S5P_FIMV_DEC_VC1_BITPLANE_SIZE;
		buf_size1 -= S5P_FIMV_DEC_VC1_BITPLANE_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_BITPLANE2_ADR);
		buf_addr1 += S5P_FIMV_DEC_VC1_BITPLANE_SIZE;
		buf_size1 -= S5P_FIMV_DEC_VC1_BITPLANE_SIZE;
		WRITEL(OFFSETA(buf_addr1), S5P_FIMV_BITPLANE1_ADR);
		buf_addr1 += S5P_FIMV_DEC_VC1_BITPLANE_SIZE;
		buf_size1 -= S5P_FIMV_DEC_VC1_BITPLANE_SIZE;
		break;
	case S5P_FIMV_CODEC_MPEG2_DEC:
		break;
	default:
		mfc_err("Unknown codec for decoding (%x).\n",
			ctx->codec_mode);
		return -EINVAL;
		break;
	}
	frame_size = ctx->luma_size;
	frame_size_ch = ctx->chroma_size;
	frame_size_mv = ctx->mv_size;
	mfc_debug("Frame size: %d ch: %d mv: %d\n", frame_size, frame_size_ch,
								frame_size_mv);
	for (i = 0; i < ctx->total_dpb_count; i++) {
		/* Port B */
		mfc_debug("Luma %d: %x\n", i, ctx->dst_bufs[i].paddr.raw.luma);
		WRITEL(OFFSETB(ctx->dst_bufs[i].paddr.raw.luma),
						S5P_FIMV_LUMA_ADR + i * 4);
		mfc_debug("\tChroma %d: %x\n", i,
					ctx->dst_bufs[i].paddr.raw.chroma);
		WRITEL(OFFSETA(ctx->dst_bufs[i].paddr.raw.chroma),
					       S5P_FIMV_CHROMA_ADR + i * 4);
		if (ctx->codec_mode == S5P_FIMV_CODEC_H264_DEC) {
			mfc_debug("\tBuf2: %x, size: %d\n",
							buf_addr2, buf_size2);
			WRITEL(OFFSETB(buf_addr2), S5P_FIMV_MV_ADR + i * 4);
			buf_addr2 += frame_size_mv;
			buf_size2 -= frame_size_mv;
		}
	}
	mfc_debug("Buf1: %u, buf_size1: %d\n", buf_addr1, buf_size1);
	mfc_debug("Buf 1/2 size after: %d/%d (frames %d)\n",
			buf_size1,  buf_size2, ctx->total_dpb_count);
	if (buf_size1 < 0 || buf_size2 < 0) {
		mfc_debug("Not enough memory has been allocated.\n");
		return -ENOMEM;
	}

	s5p_mfc_set_luma_size(ctx, frame_size);
	s5p_mfc_set_chroma_size(ctx, frame_size_ch);

	if (ctx->codec_mode == S5P_FIMV_CODEC_H264_DEC) {
		s5p_mfc_set_mv_size(ctx, frame_size_mv);
	}
	WRITEL(((S5P_FIMV_CH_INIT_BUFS & S5P_FIMV_CH_MASK) << S5P_FIMV_CH_SHIFT)
				| (ctx->inst_no), S5P_FIMV_SI_CH0_INST_ID);

	mfc_debug("After setting buffers.\n");
	return 0;
}

/* Allocate firmware */
int s5p_mfc_alloc_firmware(struct s5p_mfc_dev *dev)
{
	int err;
	struct cma_info mem_info_f, mem_info_a, mem_info_b;
	mfc_debug_enter();
	if (s5p_mfc_bitproc_buf) {
		mfc_err("Attempting to allocate firmware when it seems that it is already loaded.\n");
		return -ENOMEM;
	}
	/* Get memory region information and check if it is correct */
	err = cma_info(&mem_info_f, dev->v4l2_dev.dev, MFC_CMA_FW);
	mfc_debug("Area \"%s\" is from %08x to %08x and has size %08x", "f",
				mem_info_f.lower_bound, mem_info_f.upper_bound,
							mem_info_f.total_size);
	if (err) {
		mfc_err("Couldn't get memory information from CMA.\n");
		return -EINVAL;
	}
	err = cma_info(&mem_info_a, dev->v4l2_dev.dev, MFC_CMA_BANK1);
	mfc_debug("Area \"%s\" is from %08x to %08x and has size %08x", "a",
			mem_info_a.lower_bound, mem_info_a.upper_bound,
						mem_info_a.total_size);
	if (err) {
		mfc_err("Couldn't get memory information from CMA.\n");
		return -EINVAL;
	}
	err = cma_info(&mem_info_b, dev->v4l2_dev.dev, MFC_CMA_BANK2);
	mfc_debug("Area \"%s\" is from %08x to %08x and has size %08x", "b",
		mem_info_b.lower_bound, mem_info_b.upper_bound,
					mem_info_b.total_size);
	if (err) {
		mfc_err("Couldn't get memory information from CMA.\n");
		return -EINVAL;
	}
	if (mem_info_f.upper_bound > mem_info_a.lower_bound) {
			mfc_err("Firmware has to be "
			"allocated before  memory for buffers (bank A).\n");
		return -EINVAL;
	}
	mfc_debug("Allocating memory for firmware.\n");
	s5p_mfc_bitproc_buf = s5p_mfc_mem_alloc(
		dev->alloc_ctx[MFC_CMA_FW_ALLOC_CTX], FIRMWARE_CODE_SIZE);
	if (IS_ERR(s5p_mfc_bitproc_buf)) {
		s5p_mfc_bitproc_buf = 0;
		printk(KERN_ERR "Allocating bitprocessor buffer failed\n");
		return -ENOMEM;
	}
	s5p_mfc_bitproc_phys = s5p_mfc_mem_paddr(
		dev->alloc_ctx[MFC_CMA_FW_ALLOC_CTX], s5p_mfc_bitproc_buf);
	if (s5p_mfc_bitproc_phys & (128 << 10)) {
		mfc_err("The base memory is not aligned to 128KB.\n");
		s5p_mfc_mem_put(dev->alloc_ctx[MFC_CMA_FW_ALLOC_CTX],
							s5p_mfc_bitproc_buf);
		s5p_mfc_bitproc_phys = 0;
		s5p_mfc_bitproc_buf = 0;
		return -EIO;
	}
	dev->port_a = s5p_mfc_bitproc_phys;
	dev->port_b = mem_info_b.lower_bound;
	mfc_debug("Port A: %08x Port B: %08x (FW: %08x size: %08x)\n",
				dev->port_a, dev->port_b, s5p_mfc_bitproc_phys,
							FIRMWARE_CODE_SIZE);
	s5p_mfc_bitproc_virt = s5p_mfc_mem_vaddr(
		dev->alloc_ctx[MFC_CMA_FW_ALLOC_CTX], s5p_mfc_bitproc_buf);
	mfc_debug("Virtual address for FW: %08lx\n",
				(long unsigned int)s5p_mfc_bitproc_virt);
	if (!s5p_mfc_bitproc_virt) {
		mfc_err("Bitprocessor memory remap failed\n");
		s5p_mfc_mem_put(dev->alloc_ctx[MFC_CMA_FW_ALLOC_CTX],
							s5p_mfc_bitproc_buf);
		s5p_mfc_bitproc_phys = 0;
		s5p_mfc_bitproc_buf = 0;
		return -EIO;
	}
	mfc_debug_leave();
	return 0;
}

/* Load firmware to MFC */
int s5p_mfc_load_firmware(struct s5p_mfc_dev *dev)
{
	struct firmware *fw_blob;
	int err;

	/* Firmare has to be present as a separate file or compiled
	 * into kernel. */
	mfc_debug_enter();
	mfc_debug("Requesting fw\n");
	err = request_firmware((const struct firmware **)&fw_blob,
				     "s5pc110-mfc.fw", dev->v4l2_dev.dev);
	mfc_debug("Ret of request_firmware: %d Size: %d\n", err, fw_blob->size);
	if (err != 0) {
		mfc_err("Firmware is not present in the /lib/firmware directory nor compiled in kernel.\n");
		return -EINVAL;
	}
	if (fw_blob->size > FIRMWARE_CODE_SIZE) {
		mfc_err("MFC firmware is too big to be loaded.\n");
		release_firmware(fw_blob);
		return -ENOMEM;
	}
	if (s5p_mfc_bitproc_buf == 0 || s5p_mfc_bitproc_phys == 0) {
		mfc_err("MFC firmware is not allocated or was not mapped correctly.\n");
		release_firmware(fw_blob);
		return -EINVAL;
	}
	memcpy(s5p_mfc_bitproc_virt, fw_blob->data, fw_blob->size);
	flush_cache_all();
	release_firmware(fw_blob);
	mfc_debug_leave();
	return 0;
}

/* Release firmware memory */
int s5p_mfc_release_firmware(struct s5p_mfc_dev *dev)
{
	/* Before calling this function one has to make sure
	 * that MFC is no longer processing */
	if (!s5p_mfc_bitproc_buf)
		return -EINVAL;
	s5p_mfc_mem_put(dev->alloc_ctx[MFC_CMA_FW_ALLOC_CTX],
							s5p_mfc_bitproc_buf);
	s5p_mfc_bitproc_virt =  0;
	s5p_mfc_bitproc_phys = 0;
	s5p_mfc_bitproc_buf = 0;
	return 0;
}

/* Initialize hardware */
int s5p_mfc_init_hw(struct s5p_mfc_dev *dev)
{
	int fw_buf_size;
	unsigned int fw_version;
	int ret;

	mfc_debug_enter();
	mfc_debug("Device pointer: %p\n", dev);
	if (!s5p_mfc_bitproc_buf)
		return -EINVAL;
	/* 0. MFC reset */
	mfc_debug("MFC reset...\n");
	ret = s5p_mfc_cmd_reset(dev);
	if (ret) {
		mfc_err("Failed to reset MFC - timeout.\n");
		return ret;
	}
	mfc_debug("Done MFC reset...\n");
	/* 1. Set DRAM base Addr */
	WRITEL(dev->port_a, S5P_FIMV_MC_DRAMBASE_ADR_A); /* channelA, port0 */
	WRITEL(dev->port_b, S5P_FIMV_MC_DRAMBASE_ADR_B); /* channelB, port1 */
	mfc_debug("Port A: %08x, Port B: %08x\n", dev->port_a, dev->port_b);
	/* 2. Initialize registers of stream I/F for decoder */
	WRITEL(0xffffffff, S5P_FIMV_SI_CH0_INST_ID);
	WRITEL(0xffffffff, S5P_FIMV_SI_CH1_INST_ID);
	WRITEL(0, S5P_FIMV_RISC2HOST_CMD);
	WRITEL(0, S5P_FIMV_HOST2RISC_CMD);
	/* 3. Release reset signal to the RISC.  */
	WRITEL(0x3ff, S5P_FIMV_SW_RESET);
	mfc_debug("Will now wait for completion of firmware transfer.\n");
	if (s5p_mfc_wait_for_done_dev(dev, S5P_FIMV_R2H_CMD_FW_STATUS_RET)) {
		mfc_err("Failed to load firmware.\n");
		s5p_mfc_clean_dev_int_flags(dev);
		return -EIO;
	}
	s5p_mfc_clean_dev_int_flags(dev);
	/* 4. Initialize firmware */
	fw_buf_size = FIRMWARE_CODE_SIZE;
	mfc_debug("Writing a command\n");
	ret = s5p_mfc_cmd_host2risc(dev, 0, S5P_FIMV_H2R_CMD_SYS_INIT,
								fw_buf_size);
	if (ret) {
		mfc_err("Failed to send command to MFC - timeout.\n");
		return ret;
	}
	mfc_debug("Ok, now will write a command to init the system\n");
	if (s5p_mfc_wait_for_done_dev(dev, S5P_FIMV_R2H_CMD_SYS_INIT_RET)) {
		mfc_err("Failed to load firmware\n");
		return -EIO;
	}
	dev->int_cond = 0;
	if (dev->int_err != 0 || dev->int_type !=
						S5P_FIMV_R2H_CMD_SYS_INIT_RET) {
		/* Failure. */
		mfc_err("Failed to init firmware - error: %d"
				" int: %d.\n",dev->int_err, dev->int_type);
		return -EIO;
	}
	fw_version = READL(S5P_FIMV_FW_VERSION);
	mfc_info("MFC FW version : %02xyy, %02xmm, %02xdd\n",
		 (fw_version >> S5P_FIMV_FW_Y_SHIFT) & S5P_FIMV_FW_MASK,
		 (fw_version >> S5P_FIMV_FW_M_SHIFT) & S5P_FIMV_FW_MASK,
		 (fw_version >> S5P_FIMV_FW_D_SHIFT) & S5P_FIMV_FW_MASK);
	mfc_debug_leave();
	return 0;
}

/* Open a new instance and get its number */
int s5p_mfc_open_inst(struct s5p_mfc_ctx *ctx)
{
	int ret;
	struct s5p_mfc_dev *dev = ctx->dev;

	mfc_debug_enter();
	mfc_debug("Requested codec mode: %d\n", ctx->codec_mode);
	ret = s5p_mfc_cmd_host2risc(ctx->dev, ctx, \
			S5P_FIMV_H2R_CMD_OPEN_INSTANCE, ctx->codec_mode);
	mfc_debug_leave();
	return ret;
}

/* Close instance */
int s5p_mfc_return_inst_no(struct s5p_mfc_ctx *ctx)
{
	int ret = 0;
	struct s5p_mfc_dev *dev = ctx->dev;

	mfc_debug_enter();
	if (ctx->state != MFCINST_FREE) {
		ret = s5p_mfc_cmd_host2risc(dev, ctx,
			S5P_FIMV_H2R_CMD_CLOSE_INSTANCE, ctx->inst_no);
	} else {
		ret = -EINVAL;
	}
	mfc_debug_leave();
	return ret;
}

/* Initialize decoding */
int s5p_mfc_init_decode(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;

	mfc_debug_enter();
	mfc_debug("InstNo: %d/%d\n", ctx->inst_no, S5P_FIMV_CH_SEQ_HEADER);
	s5p_mfc_set_shared_buffer(ctx);
	mfc_debug("BUFs: %08x %08x %08x %08x %08x\n",
		  READL(S5P_FIMV_SI_CH0_DESC_ADR),
		  READL(S5P_FIMV_SI_CH0_CPB_SIZE),
		  READL(S5P_FIMV_SI_CH0_DESC_SIZE),
		  READL(S5P_FIMV_SI_CH0_SB_ST_ADR),
		  READL(S5P_FIMV_SI_CH0_SB_FRM_SIZE));
	/* Setup loop filter, for decoding this is only valid for MPEG4 */
	if (ctx->codec_mode == S5P_FIMV_CODEC_MPEG4_DEC) {
		mfc_debug("Set loop filter to: %d\n", ctx->loop_filter_mpeg4);
		WRITEL(ctx->loop_filter_mpeg4, S5P_FIMV_ENC_LF_CTRL);
	} else {
		WRITEL(0, S5P_FIMV_ENC_LF_CTRL);
	}
	WRITEL(((ctx->slice_interface & S5P_FIMV_SLICE_INT_MASK) <<
		S5P_FIMV_SLICE_INT_SHIFT) | ((ctx->display_delay > 0 ? 1 : 0) <<
		S5P_FIMV_DDELAY_ENA_SHIFT) | ((ctx->display_delay &
		S5P_FIMV_DDELAY_VAL_MASK) << S5P_FIMV_DDELAY_VAL_SHIFT),
		S5P_FIMV_SI_CH0_DPB_CONF_CTRL);
	if (ctx->codec_mode == S5P_FIMV_CODEC_DIVX311_DEC) {
		mfc_debug("Setting DivX 3.11 resolution to %dx%d\n",
					ctx->img_width, ctx->img_height);
		WRITEL(ctx->img_width, S5P_FIMV_SI_DIVX311_HRESOL);
		WRITEL(ctx->img_height, S5P_FIMV_SI_DIVX311_VRESOL);
	}
	WRITEL(
	((S5P_FIMV_CH_SEQ_HEADER & S5P_FIMV_CH_MASK) << S5P_FIMV_CH_SHIFT)
				| (ctx->inst_no), S5P_FIMV_SI_CH0_INST_ID);
	mfc_debug_leave();
	return 0;
}

/* Decode a single frame */
int s5p_mfc_decode_one_frame(struct s5p_mfc_ctx *ctx, int last_frame)
{
	struct s5p_mfc_dev *dev = ctx->dev;

	mfc_debug("Setting flags to %08lx (free:%d WTF:%d)\n",
				ctx->dec_dst_flag, ctx->dst_queue_cnt,
						ctx->dst_bufs_cnt);
	WRITEL(ctx->dec_dst_flag, S5P_FIMV_SI_CH0_RELEASE_BUF);
	s5p_mfc_set_shared_buffer(ctx);
	/* Issue different commands to instance basing on whether it
	 * is the last frame or not. */
	if (!last_frame)
		WRITEL(((S5P_FIMV_CH_FRAME_START & S5P_FIMV_CH_MASK) <<
		S5P_FIMV_CH_SHIFT ) | (ctx->inst_no), S5P_FIMV_SI_CH0_INST_ID);
	else
		WRITEL(((S5P_FIMV_CH_LAST_FRAME & S5P_FIMV_CH_MASK) <<
		S5P_FIMV_CH_SHIFT) | (ctx->inst_no), S5P_FIMV_SI_CH0_INST_ID);
	mfc_debug("Decoding a usual frame.\n");
	return 0;
}

/* Deinitialize hardware */
void s5p_mfc_deinit_hw(struct s5p_mfc_dev *dev)
{
	s5p_mfc_cmd_reset(dev);
}

