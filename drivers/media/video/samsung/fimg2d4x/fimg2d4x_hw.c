/* linux/drivers/media/video/samsung/fimg2d4x/fimg2d4x_hw.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *	http://www.samsung.com/
 *
 * Samsung Graphics 2D driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/io.h>

#include "fimg2d.h"
#include "fimg2d4x.h"

static int a8_rgb_888 = (int)DEFAULT_A8_RGB_888;
static int msk_opr = (int)DEFAULT_MSK_OPR;
static int alpha_opr = (int)DEFAULT_ALPHA_OPR;
static int premult_round = (int)DEFAULT_PREMULT_ROUND_MODE;
static int blend_round = (int)DEFAULT_BLEND_ROUND_MODE;

void fimg2d4x_reset(struct fimg2d_control *info)
{
	writel(FIMG2D_SOFT_RESET, info->regs + FIMG2D_SOFT_RESET_REG);

	/* remove wince option */
	writel(0x0, info->regs + FIMG2D_BLEND_FUNCTION_REG);
}

void fimg2d4x_enable_irq(struct fimg2d_control *info)
{
	writel(FIMG2D_BLIT_INT_ENABLE, info->regs + FIMG2D_INTEN_REG);
}

void fimg2d4x_disable_irq(struct fimg2d_control *info)
{
	writel(0, info->regs + FIMG2D_INTEN_REG);
}

void fimg2d4x_clear_irq(struct fimg2d_control *info)
{
	writel(FIMG2D_BLIT_INT_FLAG, info->regs + FIMG2D_INTC_PEND_REG);
}

int fimg2d4x_is_blit_done(struct fimg2d_control *info)
{
	return readl(info->regs + FIMG2D_INTC_PEND_REG) & FIMG2D_BLIT_INT_FLAG;
}

int fimg2d4x_blit_done_status(struct fimg2d_control *info)
{
	volatile unsigned long sts;

	/* read twice */
	sts = readl(info->regs + FIMG2D_FIFO_STAT_REG);
	sts = readl(info->regs + FIMG2D_FIFO_STAT_REG);

	return (int)(sts & FIMG2D_BLIT_FINISHED);
}

void fimg2d4x_start_blit(struct fimg2d_control *info)
{
	writel(FIMG2D_START_BITBLT, info->regs + FIMG2D_BITBLT_START_REG);
}

void fimg2d4x_set_max_burst_length(struct fimg2d_control *info, enum max_burst_len len)
{
	unsigned long cfg;

	if (len == MAX_BURST_8)	/* initial value */
		return;

	cfg = readl(info->regs + FIMG2D_AXI_MODE_REG);

	cfg &= ~FIMG2D_MAX_BURST_LEN_MASK;
	cfg |= len << FIMG2D_MAX_BURST_LEN_SHIFT;
}

void fimg2d4x_set_src_type(struct fimg2d_control *info, enum image_sel type)
{
	unsigned long cfg;

	if (type == IMG_FGCOLOR)
		return;

	if (type == IMG_MEMORY)
		cfg = FIMG2D_IMAGE_TYPE_MEMORY;
	else if (type == IMG_FGCOLOR)	/* initial value */
		return;
	else
		cfg = FIMG2D_IMAGE_TYPE_BGCOLOR;

	writel(cfg, info->regs + FIMG2D_SRC_SELECT_REG);
}

void fimg2d4x_set_src_image(struct fimg2d_control *info, struct fimg2d_image *s)
{
	unsigned long cfg;

	writel(FIMG2D_ADDR(s->addr.start), info->regs + FIMG2D_SRC_BASE_ADDR_REG);
	writel(FIMG2D_STRIDE(s->stride), info->regs + FIMG2D_SRC_STRIDE_REG);

	if (s->order < ARGB_ORDER_END) {	/* argb */
		cfg = s->order << FIMG2D_RGB_ORDER_SHIFT;
		if (s->fmt == CF_A8 && a8_rgb_888)
			writel(a8_rgb_888, info->regs + FIMG2D_SRC_A8_RGB_EXT_REG);
	} else if (s->order < P1_ORDER_END) {	/* YCbC1 1plane */
		cfg = (s->order - P1_CRY1CBY0) << FIMG2D_YCBCR_ORDER_SHIFT;
	} else {	/* YCbCr 2plane */
		cfg = (s->order - P2_CRCB) << FIMG2D_YCBCR_ORDER_SHIFT;
		cfg |= FIMG2D_YCBCR_2PLANE;

		writel(FIMG2D_ADDR(s->plane2.start),
				info->regs + FIMG2D_SRC_PLANE2_BASE_ADDR_REG);
	}

	cfg |= s->fmt << FIMG2D_COLOR_FORMAT_SHIFT;

	writel(cfg, info->regs + FIMG2D_SRC_COLOR_MODE_REG);
}

void fimg2d4x_set_src_rect(struct fimg2d_control *info, struct fimg2d_rect *r)
{
	writel(FIMG2D_OFFSET(r->x1, r->y1), info->regs + FIMG2D_SRC_LEFT_TOP_REG);
	writel(FIMG2D_OFFSET(r->x2, r->y2), info->regs + FIMG2D_SRC_RIGHT_BOTTOM_REG);
}

void fimg2d4x_set_dst_type(struct fimg2d_control *info, enum image_sel type)
{
	unsigned long cfg;

	if (type == IMG_FGCOLOR)	/* initial value */
		return;

	if (type == IMG_MEMORY)
		cfg = FIMG2D_IMAGE_TYPE_MEMORY;
	else
		cfg = FIMG2D_IMAGE_TYPE_BGCOLOR;

	writel(cfg, info->regs + FIMG2D_DST_SELECT_REG);
}

/**
 * @d: set base address, stride, color format, order
*/
void fimg2d4x_set_dst_image(struct fimg2d_control *info, struct fimg2d_image *d)
{
	unsigned long cfg;

	writel(FIMG2D_ADDR(d->addr.start), info->regs + FIMG2D_DST_BASE_ADDR_REG);
	writel(FIMG2D_STRIDE(d->stride), info->regs + FIMG2D_DST_STRIDE_REG);

	if (d->order < ARGB_ORDER_END) {
		cfg = d->order << FIMG2D_RGB_ORDER_SHIFT;
		if (d->fmt == CF_A8 && a8_rgb_888)
			writel(a8_rgb_888, info->regs + FIMG2D_DST_A8_RGB_EXT_REG);
	} else if (d->order < P1_ORDER_END) {
		cfg = (d->order - P1_CRY1CBY0) << FIMG2D_YCBCR_ORDER_SHIFT;
	} else {
		cfg = (d->order - P2_CRCB) << FIMG2D_YCBCR_ORDER_SHIFT;
		cfg |= FIMG2D_YCBCR_2PLANE;

		writel(FIMG2D_ADDR(d->plane2.start),
				info->regs + FIMG2D_DST_PLANE2_BASE_ADDR_REG);
	}

	cfg |= d->fmt << FIMG2D_COLOR_FORMAT_SHIFT;

	writel(cfg, info->regs + FIMG2D_DST_COLOR_MODE_REG);
}

void fimg2d4x_set_dst_rect(struct fimg2d_control *info, struct fimg2d_rect *r)
{
	writel(FIMG2D_OFFSET(r->x1, r->y1), info->regs + FIMG2D_DST_LEFT_TOP_REG);
	writel(FIMG2D_OFFSET(r->x2, r->y2), info->regs + FIMG2D_DST_RIGHT_BOTTOM_REG);
}

void fimg2d4x_enable_msk(struct fimg2d_control *info)
{
	unsigned long cfg;

	cfg = readl(info->regs + FIMG2D_BITBLT_COMMAND_REG);
	cfg |= FIMG2D_ENABLE_NORMAL_MSK;

	writel(cfg, info->regs + FIMG2D_BITBLT_COMMAND_REG);
}

void fimg2d4x_set_msk_image(struct fimg2d_control *info, struct fimg2d_image *m)
{
	unsigned long cfg;

	writel(FIMG2D_ADDR(m->addr.start), info->regs + FIMG2D_MSK_BASE_ADDR_REG);
	writel(FIMG2D_STRIDE(m->stride), info->regs + FIMG2D_MSK_STRIDE_REG);

	cfg = m->order << FIMG2D_MSK_ORDER_SHIFT;
	cfg |= (m->fmt - CF_MSK_1BIT) << FIMG2D_MSK_FORMAT_SHIFT;

	/* 16, 32bit mask only */
	if (m->fmt >= CF_MSK_16BIT_565) {
		if (msk_opr == MSK_ALPHA)
			cfg |= FIMG2D_MSK_TYPE_ALPHA;
		else if (msk_opr == MSK_ARGB)
			cfg |= FIMG2D_MSK_TYPE_ARGB;
		else
			cfg |= FIMG2D_MSK_TYPE_MIXED;
	}

	writel(cfg, info->regs + FIMG2D_MSK_MODE_REG);
}

void fimg2d4x_set_msk_rect(struct fimg2d_control *info, struct fimg2d_rect *r)
{
	writel(FIMG2D_OFFSET(r->x1, r->y1), info->regs + FIMG2D_MSK_LEFT_TOP_REG);
	writel(FIMG2D_OFFSET(r->x2, r->y2), info->regs + FIMG2D_MSK_RIGHT_BOTTOM_REG);
}

/**
 * If solid color fill is enabled, other blit command is ignored.
 * Color format of solid color is considered to be
 *	the same as destination color format
 * Channel order of solid color is A-R-G-B or Y-Cb-Cr
 */
void fimg2d4x_set_color_fill(struct fimg2d_control *info, unsigned long color)
{
	writel(FIMG2D_SOLID_FILL, info->regs + FIMG2D_BITBLT_COMMAND_REG);

	/* sf color */
	if (color)
		writel(color, info->regs + FIMG2D_SF_COLOR_REG);
}

/**
 * premultiply src, dst, pat for read
 * depremultiply dst for write
 */
void fimg2d4x_set_premultiplied(struct fimg2d_control *info)
{
	unsigned long cfg;

	cfg = readl(info->regs + FIMG2D_BITBLT_COMMAND_REG);
	cfg |= FIMG2D_PREMULT_ALL;

	writel(cfg, info->regs + FIMG2D_BITBLT_COMMAND_REG);
}

void fimg2d4x_src_premultiply(struct fimg2d_control *info)
{
	unsigned long cfg;

	cfg = readl(info->regs + FIMG2D_BITBLT_COMMAND_REG);
	cfg |= FIMG2D_SRC_PREMULT;

	writel(cfg, info->regs + FIMG2D_BITBLT_COMMAND_REG);
}

void fimg2d4x_dst_premultiply(struct fimg2d_control *info)
{
	unsigned long cfg;

	cfg = readl(info->regs + FIMG2D_BITBLT_COMMAND_REG);
	cfg |= FIMG2D_DST_RD_PREMULT;

	writel(cfg, info->regs + FIMG2D_BITBLT_COMMAND_REG);
}

void fimg2d4x_dst_depremultiply(struct fimg2d_control *info)
{
	unsigned long cfg;

	cfg = readl(info->regs + FIMG2D_BITBLT_COMMAND_REG);
	cfg |= FIMG2D_DST_WR_DEPREMULT;

	writel(cfg, info->regs + FIMG2D_BITBLT_COMMAND_REG);
}

/**
 * set transp/bluscr mode, bs color, bg color
 */
void fimg2d4x_set_bluescreen(struct fimg2d_control *info,
		struct fimg2d_bluscr *bluscr)
{
	unsigned long cfg;

	cfg = readl(info->regs + FIMG2D_BITBLT_COMMAND_REG);

	if (bluscr->mode == TRANSP)
		cfg |= FIMG2D_TRANSP_MODE;
	else if (bluscr->mode == BLUSCR)
		cfg |= FIMG2D_BLUSCR_MODE;
	else	/* opaque: initial value */
		return;

	writel(cfg, info->regs + FIMG2D_BITBLT_COMMAND_REG);

	/* bs color */
	if (bluscr->bs_color)
		writel(bluscr->bs_color, info->regs + FIMG2D_BS_COLOR_REG);

	/* bg color */
	if (bluscr->mode == BLUSCR && bluscr->bg_color)
		writel(bluscr->bg_color, info->regs + FIMG2D_BG_COLOR_REG);
}

/**
 * @c: destination clipping region
 */
void fimg2d4x_enable_clipping(struct fimg2d_control *info, struct fimg2d_clip *c)
{
	unsigned long cfg;

	cfg = readl(info->regs + FIMG2D_BITBLT_COMMAND_REG);
	cfg |= FIMG2D_ENABLE_CW;

	writel(cfg, info->regs + FIMG2D_BITBLT_COMMAND_REG);

	writel(FIMG2D_OFFSET(c->x1, c->y1), info->regs + FIMG2D_CW_LT_REG);
	writel(FIMG2D_OFFSET(c->x2, c->y2), info->regs + FIMG2D_CW_RB_REG);
}

void fimg2d4x_enable_dithering(struct fimg2d_control *info)
{
	unsigned long cfg;

	cfg = readl(info->regs + FIMG2D_BITBLT_COMMAND_REG);
	cfg |= FIMG2D_ENABLE_DITHER;

	writel(cfg, info->regs + FIMG2D_BITBLT_COMMAND_REG);
}


#define MAX_PRECISION 16

/**
 * scale_factor_to_fixed16 - convert scaling factor to fixed pint 16
 */
static unsigned long scale_factor_to_fixed16(int percental_factor)
{
	int i;
	unsigned long fixed16;
	unsigned long intg;
	unsigned short frac;
	unsigned short temp;

	intg = percental_factor/100;
	frac = 0;
	temp = percental_factor%100;

	for (i = 0; i < MAX_PRECISION; i++) {
		if (temp == 0 || temp == 100)
			break;

		if (temp >= 50) {
			frac |= 1 << (15-i);
			temp -= 50;
		}
		temp <<= 1;
	}

	fixed16 = (intg << 16) | frac;

	return fixed16;
}

void fimg2d4x_set_src_scaling(struct fimg2d_control *info, struct fimg2d_scale *s)
{
	int xinv, yinv;
	unsigned long xcfg, ycfg;
	unsigned long mode;

	/* scaling algorithm */
	if (s->mode == SCALING_NEAREST)
		mode = FIMG2D_SCALE_MODE_NEAREST;
	else
		mode = FIMG2D_SCALE_MODE_BILINEAR;

	writel(mode, info->regs + FIMG2D_SRC_SCALE_CTRL_REG);

	if (s->factor == SCALING_PERCENTAGE) {
		/* inversed percental scaling factor:
		 * scale-up 200%->50, scale-down 50%->200 */
		xinv = 10000/s->scale_w;
		yinv = 10000/s->scale_h;
	} else {
		/* inversed percental scaling factor */
		xinv = (s->src_w*100)/s->dst_w;
		yinv = (s->src_h*100)/s->dst_h;
	}

	/* convert inversed scaling factor to fixed point 16 */
	xcfg = scale_factor_to_fixed16(xinv);
	ycfg = scale_factor_to_fixed16(yinv);

	writel(xcfg, info->regs + FIMG2D_SRC_XSCALE_REG);
	writel(ycfg, info->regs + FIMG2D_SRC_YSCALE_REG);
}

void fimg2d4x_set_msk_scaling(struct fimg2d_control *info, struct fimg2d_scale *s)
{
	int xinv, yinv;
	unsigned long xcfg, ycfg;
	unsigned long mode;

	/* scaling algorithm */
	if (s->mode == SCALING_NEAREST)
		mode = FIMG2D_SCALE_MODE_NEAREST;
	else
		mode = FIMG2D_SCALE_MODE_BILINEAR;

	writel(mode, info->regs + FIMG2D_MSK_SCALE_CTRL_REG);

	if (s->factor == SCALING_PERCENTAGE) {
		/* inversed percental scaling factor:
		 * scale-up 200%->50, scale-down 50%->200 */
		xinv = 10000/s->scale_w;
		yinv = 10000/s->scale_h;
	} else {
		/* inversed percental scaling factor */
		xinv = (s->src_w*100)/s->dst_w;
		yinv = (s->src_h*100)/s->dst_h;
	}

	/* convert inversed percental scaling factor to fixed point 16 */
	xcfg = scale_factor_to_fixed16(xinv);
	ycfg = scale_factor_to_fixed16(yinv);

	writel(xcfg, info->regs + FIMG2D_MSK_XSCALE_REG);
	writel(ycfg, info->regs + FIMG2D_MSK_YSCALE_REG);
}

void fimg2d4x_set_src_repeat(struct fimg2d_control *info, struct fimg2d_repeat *r)
{
	unsigned long cfg;

	if (r->mode == REPEAT_NORMAL)	/* initial value */
		return;

	if (r->mode == NO_REPEAT)
		cfg = FIMG2D_SRC_REPEAT_NONE;
	else
		cfg = (r->mode - REPEAT_NORMAL) << FIMG2D_SRC_REPEAT_SHIFT;

	writel(cfg, info->regs + FIMG2D_SRC_REPEAT_MODE_REG);

	/* src pad color */
	if (r->mode == REPEAT_PAD)
		writel(r->pad_color, info->regs + FIMG2D_SRC_PAD_VALUE_REG);
}

void fimg2d4x_set_msk_repeat(struct fimg2d_control *info, struct fimg2d_repeat *r)
{
	unsigned long cfg;

	/* initial value is normal, no sfr exist for norepeat */
	if (r->mode == REPEAT_NORMAL || r->mode == NO_REPEAT)
		return;

	cfg = (r->mode - REPEAT_NORMAL) << FIMG2D_MSK_REPEAT_SHIFT;

	writel(cfg, info->regs + FIMG2D_MSK_REPEAT_MODE_REG);

	/* mask pad color */
	if (r->mode == REPEAT_PAD)
		writel(r->pad_color, info->regs + FIMG2D_MSK_PAD_VALUE_REG);
}

void fimg2d4x_set_rotation(struct fimg2d_control *info, enum rotation rot)
{
	int rev_rot90;	/* counter clockwise, 4.1 specific */
	unsigned long cfg;
	enum addressing dirx, diry;

	rev_rot90 = 0;
	dirx = diry = FORWARD_ADDRESSING;

	switch (rot) {
	case ROT_90:	/* -270 degree */
		rev_rot90 = 1;	/* fall through */
	case ROT_180:
		dirx = REVERSE_ADDRESSING;
		diry = REVERSE_ADDRESSING;
		break;
	case ROT_270:	/* -90 degree */
		rev_rot90 = 1;
		break;
	case XFLIP:
		diry = REVERSE_ADDRESSING;
		break;
	case YFLIP:
		dirx = REVERSE_ADDRESSING;
		break;
	case ORIGIN:
	default:
		break;
	}

	/* destination direction */
	if (dirx == REVERSE_ADDRESSING || diry == REVERSE_ADDRESSING) {
		cfg = readl(info->regs + FIMG2D_DST_PAT_DIRECT_REG);

		if (dirx == REVERSE_ADDRESSING)
			cfg |= FIMG2D_DST_X_DIR_NEGATIVE;

		if (diry == REVERSE_ADDRESSING)
			cfg |= FIMG2D_DST_Y_DIR_NEGATIVE;

		writel(cfg, info->regs + FIMG2D_DST_PAT_DIRECT_REG);
	}

	/* rotation -90 */
	if (rev_rot90) {
		cfg = readl(info->regs + FIMG2D_ROTATE_REG);
		cfg |= FIMG2D_SRC_ROTATE_90;
		cfg |= FIMG2D_MSK_ROTATE_90;

		writel(cfg, info->regs + FIMG2D_ROTATE_REG);
	}
}

void fimg2d4x_set_fgcolor(struct fimg2d_control *info, unsigned long fg)
{
	if (fg)
		writel(fg, info->regs + FIMG2D_FG_COLOR_REG);
}

void fimg2d4x_set_bgcolor(struct fimg2d_control *info, unsigned long bg)
{
	if (bg)
		writel(bg, info->regs + FIMG2D_BG_COLOR_REG);
}

void fimg2d4x_enable_alpha(struct fimg2d_control *info, unsigned char g_alpha)
{
	unsigned long cfg;

	/* enable alpha */
	cfg = readl(info->regs + FIMG2D_BITBLT_COMMAND_REG);
	cfg |= FIMG2D_ALPHA_BLEND_MODE;

	writel(cfg, info->regs + FIMG2D_BITBLT_COMMAND_REG);

	/* set global(constant) alpha */
	if (g_alpha < 0xff) {	/* initial value */
		cfg = readl(info->regs + FIMG2D_ALPHA_REG);
		cfg &= ~FIMG2D_GALPHA_MASK;
		cfg |= g_alpha << FIMG2D_GALPHA_SHIFT;
		writel(cfg, info->regs + FIMG2D_ALPHA_REG);
	}
}

/**
 * Four channels of the image are computed with:
 *	R = [ coeff(S)*S  + coeff(D)*D ]
 *	where
 *	R is result color or alpha
 *	S is source color or alpha
 *	D is destination color or alpha
 *
 * Caution: supposed that Sc and Dc are alpha-premultiplied value
 *
 * MODE:             coeff(S)               coeff(D)
 * ----------------------------------------------------------------------------
 * FILL:
 * CLEAR:	     0,                     0
 * SRC:		     1,                     0
 * DST:		     0,                     1
 * SRC_OVER:         1,                     1-Sa
 * DST_OVER:         1-Da                   1
 * SRC_IN:	     Da                     0
 * DST_IN:           0                      Sa
 * SRC_OUT:          1-Da                   0
 * DST_OUT:          0                      1-Sa
 * SRC_ATOP:         Da                     1-Sa
 * DST_ATOP:         1-Da                   Sa
 * XOR:              1-Da                   1-Sa
 * ADD:              1                      1
 * MULTIPLY:         Dc                     0
 * SCREEN:           1                      1-Sc
 * DARKEN:
 *    [ Ra = Sa + (1-Sa)*Da, Rc = (Da*Sc<Sa*Dc)? Sc+(1-Sa)*Dc : (1-Da)*Sc+Dc ]
 * LIGHTEN:
 *    [ Ra = Sa + (1-Sa)*Da, Rc = (Da*Sc>Sa*Dc)? Sc+(1-Sa)*Dc : (1-Da)*Sc+Dc ]
 * DISJ_SRC_OVER:    1                      min(1,(1-Sa)/Da)
 * DISJ_DST_OVER:    min(1,(1-Da)/Sa)       1
 * DISJ_SRC_IN:      max(1-(1-Da)/Sa,0)     0
 * DISJ_DST_IN:      0                      max(1-(1-Sa)/Da,0)
 * DISJ_SRC_OUT:     min(1,(1-Da)/Sa)       0
 * DISJ_DST_OUT:     0                      min(1,(1-Sa)/Da)
 * DISJ_SRC_ATOP:    max(1-(1-Da)/Sa,0)     min(1,(1-Sa)/Da)
 * DISJ_DST_ATOP:    min(1,(1-Da)/Sa)       max(1-(1-Sa)/Da,0)
 * DISJ_XOR:         min(1,(1-Da)/Sa)       min(1,(1-Sa)/Da)
 * CONJ_SRC_OVER:    1                      max(1-Sa/Da,0)
 * CONJ_DST_OVER:    max(1-Da/Sa,0)         1
 * CONJ_SRC_IN:      min(1,Da/Sa)           0
 * CONJ_DST_IN:      0                      min(1,Sa/Da)
 * CONJ_SRC_OUT:     max(1-Da/Sa,0)         0
 * CONJ_DST_OUT:     0                      max(1-Sa/Da,0)
 * CONJ_SRC_ATOP:    min(1,Da/Sa)           max(1-Sa/Da,0)
 * CONJ_DST_ATOP:    max(1-Da/Sa,0)         min(1,Sa/Da)
 * CONJ_XOR:         max(1-Da/Sa,0)         max(1-Sa/Da,0)
 */
static struct fimg2d_blend_coeff const coeff_table[MAX_FIMG2D_BLIT_OP] = {
	{ 0, 0, 0, 0 },		/* FILL */
	{ 0, COEFF_ZERO,	0, COEFF_ZERO },	/* CLEAR */
	{ 0, COEFF_ONE,		0, COEFF_ZERO },	/* SRC */
	{ 0, COEFF_ZERO,	0, COEFF_ONE },		/* DST */
	{ 0, COEFF_ONE,		1, COEFF_SA },		/* SRC_OVER */
	{ 1, COEFF_DA,		0, COEFF_ONE },		/* DST_OVER */
	{ 0, COEFF_DA,		0, COEFF_ZERO },	/* SRC_IN */
	{ 0, COEFF_ZERO,	0, COEFF_SA },		/* DST_IN */
	{ 1, COEFF_DA,		0, COEFF_ZERO },	/* SRC_OUT */
	{ 0, COEFF_ZERO,	1, COEFF_SA },		/* DST_OUT */
	{ 0, COEFF_DA,		1, COEFF_SA },		/* SRC_ATOP */
	{ 1, COEFF_DA,		0, COEFF_SA },		/* DST_ATOP */
	{ 1, COEFF_DA,		1, COEFF_SA },		/* XOR */
	{ 0, COEFF_ONE,		0, COEFF_ONE },		/* ADD */
	{ 0, COEFF_DC,		0, COEFF_ZERO },	/* MULTIPLY */
	{ 0, COEFF_ONE,		1, COEFF_SC },		/* SCREEN */
	{ 0, 0, 0, 0 },		/* DARKEN */
	{ 0, 0, 0, 0 },		/* LIGHTEN */
	{ 0, COEFF_ONE,		0, COEFF_DISJ_S },	/* DISJ_SRC_OVER */
	{ 0, COEFF_DISJ_D,	0, COEFF_ONE },		/* DISJ_DST_OVER */
	{ 1, COEFF_DISJ_D,	0, COEFF_ZERO },	/* DISJ_SRC_IN */
	{ 0, COEFF_ZERO,	1, COEFF_DISJ_S },	/* DISJ_DST_IN */
	{ 0, COEFF_DISJ_D,	0, COEFF_ONE },		/* DISJ_SRC_OUT */
	{ 0, COEFF_ZERO,	0, COEFF_DISJ_S },	/* DISJ_DST_OUT */
	{ 1, COEFF_DISJ_D,	0, COEFF_DISJ_S },	/* DISJ_SRC_ATOP */
	{ 0, COEFF_DISJ_D,	1, COEFF_DISJ_S },	/* DISJ_DST_ATOP */
	{ 0, COEFF_DISJ_D,	0, COEFF_DISJ_S },	/* DISJ_XOR */
	{ 0, COEFF_ONE,		1, COEFF_DISJ_S },	/* CONJ_SRC_OVER */
	{ 1, COEFF_DISJ_D,	0, COEFF_ONE },		/* CONJ_DST_OVER */
	{ 0, COEFF_CONJ_D,	0, COEFF_ONE },		/* CONJ_SRC_IN */
	{ 0, COEFF_ZERO,	0, COEFF_CONJ_S },	/* CONJ_DST_IN */
	{ 1, COEFF_CONJ_D,	0, COEFF_ZERO },	/* CONJ_SRC_OUT */
	{ 0, COEFF_ZERO,	1, COEFF_CONJ_S },	/* CONJ_DST_OUT */
	{ 0, COEFF_CONJ_D,	1, COEFF_CONJ_S },	/* CONJ_SRC_ATOP */
	{ 1, COEFF_CONJ_D,	0, COEFF_CONJ_D },	/* CONJ_DST_ATOP */
	{ 1, COEFF_CONJ_D,	1, COEFF_CONJ_S },	/* CONJ_XOR */
	{ 0, 0, 0, 0 },		/* USER */
};

void fimg2d4x_set_alpha_composite(struct fimg2d_control *info,
		enum blit_op op, unsigned char g_alpha)
{
	unsigned long cfg = 0;
	struct fimg2d_blend_coeff const *tbl;

	switch (op) {
	case BLIT_OP_SOLID_FILL:
	case BLIT_OP_CLR:
	case BLIT_OP_SRC:
	case BLIT_OP_DST:
		/* nop */
		break;
	case BLIT_OP_DARKEN:
		cfg |= FIMG2D_DARKEN;
		break;
	case BLIT_OP_LIGHTEN:
		cfg |= FIMG2D_LIGHTEN;
		break;
	case BLIT_OP_USER_COEFF:
		/* TODO */
		return;
	default:
		tbl = &coeff_table[op];

		/* src coeff */
		cfg |= tbl->s_coeff << FIMG2D_SRC_COEFF_SHIFT;

		if (tbl->s_coeff_inv)
			cfg |= FIMG2D_INV_SRC_COEFF;

		if (alpha_opr != ALPHA_PERPIXEL && g_alpha < 0xff) {
			cfg |= alpha_opr << FIMG2D_SRC_COEFF_DA_SHIFT;
			cfg |= alpha_opr << FIMG2D_SRC_COEFF_SA_SHIFT;
		}

		/* dst coeff */
		cfg |= tbl->d_coeff << FIMG2D_DST_COEFF_SHIFT;

		if (tbl->d_coeff_inv)
			cfg |= FIMG2D_INV_DST_COEFF;

		if (alpha_opr != ALPHA_PERPIXEL && g_alpha < 0xff) {
			cfg |= alpha_opr << FIMG2D_DST_COEFF_DA_SHIFT;
			cfg |= alpha_opr << FIMG2D_DST_COEFF_SA_SHIFT;
		}
		break;
	}

	writel(cfg, info->regs + FIMG2D_BLEND_FUNCTION_REG);

	/* round mode */
	if (premult_round != PREMULT_ROUND_3 || blend_round != BLEND_ROUND_3) {
		cfg = readl(info->regs + FIMG2D_ROUND_MODE_REG);

		cfg &= ~FIMG2D_PREMULT_ROUND_MASK;
		cfg |= (premult_round << FIMG2D_PREMULT_ROUND_SHIFT);

		cfg &= ~FIMG2D_BLEND_ROUND_MASK;
		cfg |= (blend_round << FIMG2D_BLEND_ROUND_SHIFT);

		writel(cfg, info->regs + FIMG2D_ROUND_MODE_REG);
	}
}


