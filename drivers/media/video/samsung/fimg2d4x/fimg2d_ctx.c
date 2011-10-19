/* linux/drivers/media/video/samsung/fimg2d4x/fimg2d_ctx.c
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

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <plat/fimg2d.h>

#include "fimg2d.h"

#ifdef CONFIG_VIDEO_FIMG2D_DEBUG
static inline void fimg2d_print_params(struct fimg2d_blit __user *u)
{
	fimg2d_debug("op: %d\n", u->op);
	fimg2d_debug("solid color: 0x%lx\n", u->solid_color);
	fimg2d_debug("g_alpha: 0x%x\n", u->g_alpha);
	fimg2d_debug("premultiplied: %d\n", u->premult);
	fimg2d_debug("dither: %d\n", u->dither);
	fimg2d_debug("rotate: %d\n", u->rotate);

	if (u->scaling) {
		fimg2d_debug("scaling mode: %d, factor: %d, percent(w:%d, h:%d) "
				"pixel(s:%d,%d d:%d,%d)\n",
				u->scaling->mode, u->scaling->factor,
				u->scaling->scale_w, u->scaling->scale_h,
				u->scaling->src_w, u->scaling->src_h,
				u->scaling->dst_w, u->scaling->dst_h);
	}

	if (u->repeat) {
		fimg2d_debug("repeat mode: %d, pad color: 0x%lx\n",
				u->repeat->mode, u->repeat->pad_color);
	}

	if (u->bluscr) {
		fimg2d_debug("bluescreen mode: %d, bs_color: 0x%lx bg_color: 0x%lx\n",
				u->bluscr->mode, u->bluscr->bs_color, u->bluscr->bg_color);
	}

	if (u->clipping) {
		fimg2d_debug("clipping mode: %d, LT(%d,%d) RB(%d,%d)\n",
				u->clipping->enable, u->clipping->x1, u->clipping->y1,
				u->clipping->x2, u->clipping->y2);
	}

	if (u->src) {
		fimg2d_debug("src type: %d addr: 0x%lx size: %d cacheable: %d\n",
				u->src->addr.type, u->src->addr.start, u->src->addr.size,
				u->src->addr.cacheable);
		fimg2d_debug("src width: %d height: %d stride: %d order: %d format: %d\n",
				u->src->width, u->src->height, u->src->stride,
				u->src->order, u->src->fmt);
	}

	if (u->dst) {
		fimg2d_debug("dst type: %d addr: 0x%lx size: %d cacheable: %d\n",
				u->dst->addr.type, u->dst->addr.start, u->dst->addr.size,
				u->dst->addr.cacheable);
		fimg2d_debug("dst width: %d height: %d stride: %d order: %d format: %d\n",
				u->dst->width, u->dst->height, u->dst->stride,
				u->dst->order, u->dst->fmt);
	}

	if (u->msk) {
		fimg2d_debug("msk type: %d addr: 0x%lx size: %d cacheable: %d\n",
				u->msk->addr.type, u->msk->addr.start, u->msk->addr.size,
				u->msk->addr.cacheable);
		fimg2d_debug("msk width: %d height: %d stride: %d order: %d format: %d\n",
				u->msk->width, u->msk->height, u->msk->stride,
				u->msk->order, u->msk->fmt);
	}

	if (u->src_rect) {
		fimg2d_debug("src rect LT(%d,%d) RB(%d,%d)\n",
				u->src_rect->x1, u->src_rect->y1,
				u->src_rect->x2, u->src_rect->y2);
	}

	if (u->dst_rect) {
		fimg2d_debug("dst rect LT(%d,%d) RB(%d,%d)\n",
				u->dst_rect->x1, u->dst_rect->y1,
				u->dst_rect->x2, u->dst_rect->y2);
	}

	if (u->msk_rect) {
		fimg2d_debug("msk rect LT(%d,%d) RB(%d,%d)\n",
				u->msk_rect->x1, u->msk_rect->y1,
				u->msk_rect->x2, u->msk_rect->y2);
	}
}
#endif

void fimg2d_print_current_command(struct fimg2d_control *info,
		unsigned long pgtable_base)
{
	struct fimg2d_context *ctx;
	struct fimg2d_bltcmd *cmd;

	cmd = fimg2d_get_first_command(info);
	if (!cmd)
		return;

	ctx = cmd->ctx;
	if (ctx->mm->pgd != phys_to_virt(pgtable_base))
		return;

	printk(KERN_INFO " op: %d\n", cmd->op);
	printk(KERN_INFO " solid color: 0x%lx\n", cmd->solid_color);
	printk(KERN_INFO " g_alpha: 0x%x\n", cmd->g_alpha);
	printk(KERN_INFO " premultiplied: %d\n", cmd->premult);
	printk(KERN_INFO " dither: %d\n", cmd->dither);
	printk(KERN_INFO " rotate: %d\n", cmd->rotate);
	printk(KERN_INFO " repeat mode: %d, pad color: 0x%lx\n",
			cmd->repeat.mode, cmd->repeat.pad_color);
	printk(KERN_INFO " bluescreen mode: %d, bs_color: 0x%lx bg_color: 0x%lx\n",
			cmd->bluscr.mode, cmd->bluscr.bs_color, cmd->bluscr.bg_color);

	if (cmd->scaling.mode != NO_SCALING) {
		printk(KERN_INFO " scaling mode: %d, factor: %d, percent(w:%d, h:%d) "
				" pixel(s:%d,%d d:%d,%d)\n",
				cmd->scaling.mode, cmd->scaling.factor,
				cmd->scaling.scale_w, cmd->scaling.scale_h,
				cmd->scaling.src_w, cmd->scaling.src_h,
				cmd->scaling.dst_w, cmd->scaling.dst_h);
	}

	if (cmd->clipping.enable) {
		printk(KERN_INFO " clip rect LT(%d,%d) RB(%d,%d)\n",
				cmd->clipping.x1, cmd->clipping.y1,
				cmd->clipping.x2, cmd->clipping.y2);
	}

	if (cmd->srcen) {
		printk(KERN_INFO " src type: %d addr: 0x%lx size: %d cacheable: %d\n",
				cmd->src.addr.type, cmd->src.addr.start, cmd->src.addr.size,
				cmd->src.addr.cacheable);
		printk(KERN_INFO " src width: %d height: %d stride: %d order: %d format: %d\n",
				cmd->src.width, cmd->src.height, cmd->src.stride,
				cmd->src.order, cmd->src.fmt);
		printk(KERN_INFO " src rect LT(%d,%d) RB(%d,%d)\n",
				cmd->src_rect.x1, cmd->src_rect.y1,
				cmd->src_rect.x2, cmd->src_rect.y2);
		printk(KERN_INFO " src cache addr: 0x%lx size: %d\n",
				cmd->src_cache.addr, cmd->src_cache.size);
	}

	if (cmd->dsten) {
		printk(KERN_INFO " dst type: %d addr: 0x%lx size: %d cacheable: %d\n",
				cmd->dst.addr.type, cmd->dst.addr.start, cmd->dst.addr.size,
				cmd->dst.addr.cacheable);
		printk(KERN_INFO " dst width: %d height: %d stride: %d order: %d format: %d\n",
				cmd->dst.width, cmd->dst.height, cmd->dst.stride,
				cmd->dst.order, cmd->dst.fmt);
		printk(KERN_INFO " dst rect LT(%d,%d) RB(%d,%d)\n",
				cmd->dst_rect.x1, cmd->dst_rect.y1,
				cmd->dst_rect.x2, cmd->dst_rect.y2);
		printk(KERN_INFO " dst cache addr: 0x%lx size: %d\n",
				cmd->dst_cache.addr, cmd->dst_cache.size);
	}

	if (cmd->msken) {
		printk(KERN_INFO " msk type: %d addr: 0x%lx size: %d cacheable: %d\n",
				cmd->msk.addr.type, cmd->msk.addr.start, cmd->msk.addr.size,
				cmd->msk.addr.cacheable);
		printk(KERN_INFO " msk width: %d height: %d stride: %d order: %d format: %d\n",
				cmd->msk.width, cmd->msk.height, cmd->msk.stride,
				cmd->msk.order, cmd->msk.fmt);
		printk(KERN_INFO " msk rect LT(%d,%d) RB(%d,%d)\n",
				cmd->msk_rect.x1, cmd->msk_rect.y1,
				cmd->msk_rect.x2, cmd->msk_rect.y2);
		printk(KERN_INFO " msk cache addr: 0x%lx size: %d\n",
				cmd->msk_cache.addr, cmd->msk_cache.size);
	}

	printk(KERN_INFO " cache size all: %d bytes\n", cmd->size_all);
	printk(KERN_INFO " seq_no: %u\n", cmd->seq_no);
	printk(KERN_INFO " ctx: 0x%lx pgd (ka:0x%lx pa:0x%lx)\n",
			(unsigned long)ctx, (unsigned long)ctx->mm->pgd, pgtable_base);
}

static int fimg2d_check_params(struct fimg2d_blit __user *u)
{
	int w, h;
	struct fimg2d_rect *sr, *dr, *mr;
	struct fimg2d_clip *ur;

	if (u->op < 0 || u->op >= BLIT_OP_END)
		goto err_op;

	if (u->src) {
		w = u->src->width;
		h = u->src->height;
		sr = u->src_rect;

		if (!sr ||
			sr->x1 < 0 || sr->x2 > w ||
			sr->y1 < 0 || sr->y2 > h ||
			sr->x1 == sr->x2 || sr->y1 == sr->y2)
			goto err_src_rect;

		/* 8000: max width & height */
		if (w > 8000 || h > 8000)
			goto err_src_rect;
	}

	if (u->msk) {
		w = u->msk->width;
		h = u->msk->height;
		mr = u->msk_rect;

		if (!mr ||
			mr->x1 < 0 || mr->x2 > w ||
			mr->y1 < 0 || mr->y2 > h ||
			mr->x1 == mr->x2 || mr->y1 == mr->y2)
			goto err_msk_rect;

		/* 8000: max width & height */
		if (w > 8000 || h > 8000)
			goto err_msk_rect;
	}

	if (u->dst) {
		w = u->dst->width;
		h = u->dst->height;
		dr = u->dst_rect;

		if (!dr ||
			dr->x1 < 0 || dr->x1 >= w ||
			dr->y1 < 0 || dr->y1 >= h ||
			dr->x1 == dr->x2 || dr->y1 == dr->y2)
			goto err_dst_rect;

		/* 8000: max width & height */
		if (w > 8000 || h > 8000)
			goto err_src_rect;

		/* out of dst_rect */
		if (u->clipping && u->clipping->enable) {
			ur = u->clipping;
			if (ur->x1 >= dr->x2 || ur->x2 <= dr->x1 ||
				ur->y1 >= dr->y2 || ur->y2 <= dr->y1)
				goto err_clip_rect;
		}
	}

	return 0;

err_op:
	printk(KERN_ERR "%s: invalid op\n", __func__);
	return -1;
err_src_rect:
	printk(KERN_ERR "%s: invalid src rect LT(%d,%d) RB(%d,%d)\n",
			__func__, sr->x1, sr->y1, sr->x2, sr->y2);
	return -1;
err_msk_rect:
	printk(KERN_ERR "%s: invalid msk rect, LT(%d,%d) RB(%d,%d)\n",
			__func__, mr->x1, mr->y1, mr->x2, mr->y2);
	return -1;
err_dst_rect:
	printk(KERN_ERR "%s: invalid dst rect, LT(%d,%d) RB(%d,%d)\n",
			__func__, dr->x1, dr->y1, dr->x2, dr->y2);
	return -1;
err_clip_rect:
	printk(KERN_ERR "%s: invalid clip rect, LT(%d,%d) RB(%d,%d)\n",
			__func__, ur->x1, ur->y1, ur->x2, ur->y2);
	return -1;
}

static void fimg2d_fixup_params(struct fimg2d_bltcmd *cmd)
{
	/* fix up scaling */
	if (cmd->scaling.mode) {
		if (cmd->scaling.factor == SCALING_PERCENTAGE) {
			if ((!cmd->scaling.scale_w && !cmd->scaling.scale_h) ||
				(cmd->scaling.scale_w == 100 && cmd->scaling.scale_h == 100)) {
				cmd->scaling.mode = NO_SCALING;
			}
		} else if (cmd->scaling.factor == SCALING_PIXELS) {
			if ((cmd->scaling.src_w == cmd->scaling.dst_w) &&
				(cmd->scaling.src_h == cmd->scaling.dst_h)) {
				cmd->scaling.mode = NO_SCALING;
			}
		}
	}

	/* fix up dst rect */
	if (cmd->dst_rect.x2 > cmd->dst.width) {
		fimg2d_debug("fixing up dst coord x2: %d --> %d\n",
				cmd->dst_rect.x2, cmd->dst.width);
		cmd->dst_rect.x2 = cmd->dst.width;
	}
	if (cmd->dst_rect.y2 > cmd->dst.height) {
		fimg2d_debug("fixing up dst coord y2: %d --> %d\n",
				cmd->dst_rect.y2, cmd->dst.height);
		cmd->dst_rect.y2 = cmd->dst.height;
	}

	/* fix up clip rect */
	if (cmd->clipping.enable) {
		/* fit to smaller dst region  as a clip rect */
		if (cmd->clipping.x1 < cmd->dst_rect.x1) {
			fimg2d_debug("fixing up cipping coord x1: %d --> %d\n",
					cmd->clipping.x1, cmd->dst_rect.x1);
			cmd->clipping.x1 = cmd->dst_rect.x1;
		}
		if (cmd->clipping.y1 < cmd->dst_rect.y1) {
			fimg2d_debug("fixing up cipping coord y1: %d --> %d\n",
					cmd->clipping.y1, cmd->dst_rect.y1);
			cmd->clipping.y1 = cmd->dst_rect.y1;
		}
		if (cmd->clipping.x2 > cmd->dst_rect.x2) {
			fimg2d_debug("fixing up cipping coord x2: %d --> %d\n",
					cmd->clipping.x2, cmd->dst_rect.x2);
			cmd->clipping.x2 = cmd->dst_rect.x2;
		}
		if (cmd->clipping.y2 > cmd->dst_rect.y2) {
			fimg2d_debug("fixing up cipping coord y2: %d --> %d\n",
					cmd->clipping.y2, cmd->dst_rect.y2);
			cmd->clipping.y2 = cmd->dst_rect.y2;
		}
	}
}

static int fimg2d_check_dma_sync(struct fimg2d_bltcmd *cmd)
{
	struct fimg2d_cache *csrc, *cdst, *cmsk;

	csrc = &cmd->src_cache;
	cdst = &cmd->dst_cache;
	cmsk = &cmd->msk_cache;

	if (cmd->srcen) {
		csrc->addr = cmd->src.addr.start +
				(cmd->src.stride * cmd->src_rect.y1);
		csrc->size = cmd->src.stride *
				(cmd->src_rect.y2 - cmd->src_rect.y1);
	}

	if (cmd->msken) {
		cmsk->addr = cmd->msk.addr.start +
				(cmd->msk.stride * cmd->msk_rect.y1);
		cmsk->size = cmd->msk.stride *
				(cmd->msk_rect.y2 - cmd->msk_rect.y1);
	}

	/* caculate horizontally clipped region */
	if (cmd->dsten) {
		if (cmd->clipping.enable) {
			cdst->addr = cmd->dst.addr.start +
					(cmd->dst.stride * cmd->clipping.y1);
			cdst->size = cmd->dst.stride *
					(cmd->clipping.y2 - cmd->clipping.y1);
		} else {
			cdst->addr = cmd->dst.addr.start +
					(cmd->dst.stride * cmd->dst_rect.y1);
			cdst->size = cmd->dst.stride *
					(cmd->dst_rect.y2 - cmd->dst_rect.y1);
		}
	}

#ifdef CONFIG_OUTER_CACHE
	if (cmd->srcen && fimg2d_check_pagetable(cmd->ctx->mm,
				csrc->addr, csrc->size) == PT_FAULT)
		return -1;

	if (cmd->msken && fimg2d_check_pagetable(cmd->ctx->mm,
				cmsk->addr, cmsk->size) == PT_FAULT)
		return -1;

	if (cmd->dsten && fimg2d_check_pagetable(cmd->ctx->mm,
				cdst->addr, cdst->size) == PT_FAULT)
		return -1;
#endif

	cmd->size_all = 0;

	if (cmd->srcen && cmd->src.addr.cacheable)
		cmd->size_all += csrc->size;
	if (cmd->msken && cmd->msk.addr.cacheable)
		cmd->size_all += cmsk->size;
	if (cmd->dsten && cmd->dst.addr.cacheable)
		cmd->size_all += cdst->size;

	fimg2d_debug("cached size all = %d\n", cmd->size_all);

	/* FIXME: L1 cache size = (num_possible_cpus()*SZ_32K) */
	if (cmd->size_all < L1_CACHE_SIZE) {
		fimg2d_debug("innercache range\n");
		if (cmd->srcen && cmd->src.addr.cacheable)
			fimg2d_dma_sync_inner(csrc->addr, csrc->size, DMA_TO_DEVICE);

		if (cmd->msken && cmd->msk.addr.cacheable)
			fimg2d_dma_sync_inner(cmsk->addr, cmsk->size, DMA_TO_DEVICE);

		if (cmd->dsten && cmd->dst.addr.cacheable)
			fimg2d_dma_sync_inner(cdst->addr, cdst->size, DMA_BIDIRECTIONAL);
	}

	return 0;
}

int fimg2d_add_command(struct fimg2d_control *info, struct fimg2d_context *ctx,
			struct fimg2d_blit __user *u)
{
	struct fimg2d_bltcmd *cmd;

#ifdef CONFIG_VIDEO_FIMG2D_DEBUG
	fimg2d_print_params(u);
#endif

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		printk(KERN_ERR "failed to create bitblt command header\n");
		return -ENOMEM;
	}

	if (info->err) {
		printk(KERN_ERR "%s: previous unrecoverable hardware error\n", __func__);
		goto err_user;
	}

	if (fimg2d_check_params(u))
		goto err_user;

	cmd->ctx = ctx;
	cmd->seq_no = u->seq_no;

	cmd->op = u->op;
	cmd->premult = u->premult;
	cmd->g_alpha = u->g_alpha;
	cmd->dither = u->dither;
	cmd->rotate = u->rotate;
	cmd->solid_color = u->solid_color;

	if (u->scaling && u->scaling->mode) {
		if (copy_from_user(&cmd->scaling, u->scaling, sizeof(cmd->scaling)))
			goto err_user;
	}

	if (u->repeat && u->repeat->mode) {
		if (copy_from_user(&cmd->repeat, u->repeat, sizeof(cmd->repeat)))
			goto err_user;
	}

	if (u->bluscr && u->bluscr->mode) {
		if (copy_from_user(&cmd->bluscr, u->bluscr, sizeof(cmd->bluscr)))
			goto err_user;
	}

	if (u->clipping && u->clipping->enable) {
		if (copy_from_user(&cmd->clipping, u->clipping, sizeof(cmd->clipping)))
			goto err_user;
	}

	if (u->src) {
		cmd->srcen = true;
		if (copy_from_user(&cmd->src, u->src, sizeof(cmd->src)))
			goto err_user;
	}

	if (u->dst) {
		cmd->dsten = true;
		if (copy_from_user(&cmd->dst, u->dst, sizeof(cmd->dst)))
			goto err_user;
	}

	if (u->msk) {
		cmd->msken = true;
		if (copy_from_user(&cmd->msk, u->msk, sizeof(cmd->msk)))
			goto err_user;
	}

	if (u->src_rect) {
		if (copy_from_user(&cmd->src_rect, u->src_rect, sizeof(cmd->src_rect)))
			goto err_user;
	}

	if (u->dst_rect) {
		if (copy_from_user(&cmd->dst_rect, u->dst_rect, sizeof(cmd->dst_rect)))
			goto err_user;
	}

	if (u->msk_rect) {
		if (copy_from_user(&cmd->msk_rect, u->msk_rect, sizeof(cmd->msk_rect)))
			goto err_user;
	}

	fimg2d_fixup_params(cmd);

	if (fimg2d_check_dma_sync(cmd))
		goto err_user;

	/* add command node and increase ncmd */
	spin_lock(&info->bltlock);
	atomic_inc(&ctx->ncmd);
	fimg2d_enqueue(&cmd->node, &info->cmd_q);
	fimg2d_debug("ctx %p pgd (ka:0x%lx,pa:0x%lx) ncmd(%d) seq_no(%u)\n",
			cmd->ctx,
			(unsigned long)cmd->ctx->mm->pgd,
			(unsigned long)virt_to_phys((unsigned long *)cmd->ctx->mm->pgd),
			atomic_read(&ctx->ncmd), cmd->seq_no);
	spin_unlock(&info->bltlock);

	return 0;

err_user:
	kfree(cmd);
	return -EFAULT;
}

inline void fimg2d_add_context(struct fimg2d_control *info, struct fimg2d_context *ctx)
{
	atomic_set(&ctx->ncmd, 0);
	init_waitqueue_head(&ctx->wait_q);

	atomic_inc(&info->nctx);
	fimg2d_debug("ctx %p nctx(%d)\n", ctx, atomic_read(&info->nctx));
}

inline void fimg2d_del_context(struct fimg2d_control *info, struct fimg2d_context *ctx)
{
	atomic_dec(&info->nctx);
	fimg2d_debug("ctx %p nctx(%d)\n", ctx, atomic_read(&info->nctx));
}
