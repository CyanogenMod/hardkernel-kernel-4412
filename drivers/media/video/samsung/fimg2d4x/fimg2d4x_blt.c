/* linux/drivers/media/video/samsung/fimg2d4x/fimg2d4x_blt.c
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
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>

#include "fimg2d.h"
#include "fimg2d4x.h"

#ifdef CONFIG_S5P_SYSTEM_MMU
#include <plat/sysmmu.h>
#include <mach/sysmmu.h>
#endif

#ifdef CONFIG_PM_RUNTIME
#include <plat/devs.h>
#include <linux/pm_runtime.h>
#endif

#define L1_CACHE_SIZE	SZ_64K
#define L2_CACHE_SIZE	SZ_1M

#undef PERF
#undef POST_BLIT

#ifdef PERF
static long get_blit_perf(struct timeval *start, struct timeval *end)
{
	long sec, usec, time;

	sec = end->tv_sec - start->tv_sec;
	if (end->tv_usec >= start->tv_usec) {
		usec = end->tv_usec - start->tv_usec;
	} else {
		usec = end->tv_usec + 1000000 - start->tv_usec;
		sec--;
	}
	time = sec * 1000000 + usec;
	printk(KERN_INFO "[%s] bitblt perf: %ld usec elapsed\n", __func__, time);

	return time; /* microseconds */
}
#endif

#ifdef CONFIG_OUTER_CACHE
static void fimg2d4x_pre_bitblt(struct fimg2d_control *info, struct fimg2d_bltcmd *cmd)
{
	int size_all, size;
	unsigned long addr;
	struct mm_struct *mm;

	size_all = 0;

	if (cmd->srcen && cmd->src.addr.cacheable)
		size_all += cmd->src.addr.size;

	if (cmd->msken && cmd->msk.addr.cacheable)
		size_all += cmd->msk.addr.size;

	if (cmd->dsten && cmd->dst.addr.cacheable)
		size_all += cmd->dst.addr.size;

	spin_lock(&info->bltlock);

	if (size_all >= L2_CACHE_SIZE) {
		flush_all_cpu_caches();	/* inner all */
		outer_flush_all();	/* outer all */
		goto cacheflush_end;
	} else if (size_all >= L1_CACHE_SIZE) {
		flush_all_cpu_caches();	/* inner all */
	}

	mm = cmd->ctx->mm;

	if (cmd->srcen) {
		addr = cmd->src.addr.start + cmd->src.stride * cmd->src_rect.y1;
		/* horizontally clipped region */
		size = cmd->src.stride * (cmd->src_rect.y2 - cmd->src_rect.y1);

		if (size_all < L1_CACHE_SIZE)
			fimg2d_dma_sync_inner(addr, size, DMA_TO_DEVICE);

		fimg2d_clean_outer_pagetable(mm, addr, size);
		fimg2d_dma_sync_outer(mm, addr, size, CACHE_CLEAN);
	}

	if (cmd->msken) {
		addr = cmd->msk.addr.start + cmd->msk.stride * cmd->msk_rect.y1;
		size = cmd->msk.stride * (cmd->msk_rect.y2 - cmd->msk_rect.y1);

		if (size_all < L1_CACHE_SIZE)
			fimg2d_dma_sync_inner(addr, size, DMA_TO_DEVICE);

		fimg2d_clean_outer_pagetable(mm, addr, size);
		fimg2d_dma_sync_outer(mm, addr, size, CACHE_CLEAN);
	}

	if (cmd->dsten) {
		addr = cmd->dst.addr.start + cmd->dst.stride * cmd->dst_rect.y1;
		size = cmd->dst.stride * (cmd->dst_rect.y2 - cmd->dst_rect.y1);

		if (size_all < L1_CACHE_SIZE)
			fimg2d_dma_sync_inner(addr, size, DMA_BIDIRECTIONAL);

		fimg2d_clean_outer_pagetable(mm, addr, size);
		fimg2d_dma_sync_outer(mm, addr, size, CACHE_FLUSH);
	}

cacheflush_end:
	spin_unlock(&info->bltlock);
}
#endif

#ifdef POST_BLIT
static void fimg2d4x_post_bitblt(struct fimg2d_control *info, struct fimg2d_bltcmd *cmd)
{
	/* TODO */
	return;
}
#endif

/**
 * blitter
 */
void fimg2d4x_bitblt(struct fimg2d_control *info)
{
	struct fimg2d_context *ctx;
	struct fimg2d_bltcmd *cmd;
	unsigned long saddr, daddr, maddr;
	unsigned long ssize, dsize, msize;
	unsigned long pgd;
#ifdef PERF
	struct timeval start, end;
#endif

	fimg2d_debug("enter blitter\n");

#ifdef CONFIG_PM_RUNTIME
	if (atomic_read(&info->pwron) == 0) {
		fimg2d_debug("pm_runtime_get_sync\n");
		pm_runtime_get_sync(&(s5p_device_fimg2d.dev));
	}
#endif

	while ((cmd = fimg2d_get_first_command(info))) {
		ctx = cmd->ctx;
		pgd = virt_to_phys(ctx->mm->pgd);

		if (info->err) {
			printk(KERN_ERR "%s: unrecoverable hardware error\n", __func__);
			goto blitend;
		}

		atomic_set(&info->busy, 1);

#ifdef CONFIG_OUTER_CACHE
		fimg2d4x_pre_bitblt(info, cmd);
#endif

		/* set sfr */
		info->configure(info, cmd);

		/* set sysmmu */
		if (cmd->dst.addr.type == ADDR_USER) {
#ifdef CONFIG_S5P_SYSTEM_MMU
			s5p_sysmmu_set_tablebase_pgd(SYSMMU_MDMA, pgd);
			fimg2d_debug("set sysmmu table base: ctx %p pgd %p seq_no(%u)\n",
					ctx, (void *)pgd, cmd->seq_no);
#endif
		}

#ifdef PERF
		do_gettimeofday(&start);
#endif

		/* start bitblt */
		info->run(info);

		if (!wait_event_timeout(info->wait_q, !atomic_read(&info->busy),
				msecs_to_jiffies(1000))) {
			if (!fimg2d4x_blit_done_status(info)) {
				saddr = daddr = maddr = 0;
				ssize = dsize = msize = 0;
				if (cmd->srcen) {
					saddr = cmd->src.addr.start;
					ssize = cmd->src.addr.size;
				}
				if (cmd->dsten) {
					daddr = cmd->dst.addr.start;
					dsize = cmd->dst.addr.size;
				}
				if (cmd->msken) {
					maddr = cmd->msk.addr.start;
					msize = cmd->msk.addr.size;
				}
				printk(KERN_ERR "[%s] fatal error, ctx %p pgd %p seq_no(%u)\n"
						"\t src addr %p end %p size %ld\n"
						"\t dst addr %p end %p size %ld\n"
						"\t msk addr %p end %p size %ld\n",
						__func__, ctx, (void *)pgd, cmd->seq_no,
						(void *)saddr, (void *)saddr+ssize, ssize,
						(void *)daddr, (void *)daddr+dsize, dsize,
						(void *)maddr, (void *)maddr+msize, msize);

				/* fatal h/w error */
				info->err = true;
			} else {
				printk(KERN_ERR "[%s] ctx %p pgd %p seq_no(%u) wait timeout\n",
						__func__, ctx, (void *)pgd, cmd->seq_no);
			}
		} else {
			fimg2d_debug("blitter wake up\n");
		}

#ifdef PERF
		do_gettimeofday(&end);
		get_blit_perf(&start, &end);
#endif

blitend:
		spin_lock(&info->bltlock);
		fimg2d_dequeue(&cmd->node);
		kfree(cmd);
		atomic_dec(&ctx->ncmd);

		/* wake up context */
		if (!atomic_read(&ctx->ncmd)) {
			fimg2d_debug("no more blit jobs for ctx %p\n", ctx);
			wake_up(&ctx->wait_q);
		}
		spin_unlock(&info->bltlock);
	}

	atomic_set(&info->active, 0);

#ifdef CONFIG_PM_RUNTIME
	if (atomic_read(&info->pwron) == 1) {
		pm_runtime_put_sync(&(s5p_device_fimg2d.dev));
		fimg2d_debug("pm_runtime_put_sync\n");
	}
#endif

	fimg2d_debug("exit blitter\n");
}

/**
 * configure hardware
 */
static void fimg2d4x_configure(struct fimg2d_control *info, struct fimg2d_bltcmd *cmd)
{
	enum image_sel srcsel, dstsel;

	fimg2d_debug("ctx %p seq_no(%u)\n", cmd->ctx, cmd->seq_no);

	/* TODO: batch blit */
	fimg2d4x_reset(info);

	/* src and dst select */
	srcsel = dstsel = IMG_MEMORY;

	switch (cmd->op) {
	case BLIT_OP_SOLID_FILL:
		srcsel = dstsel = IMG_FGCOLOR;
		fimg2d4x_set_color_fill(info, cmd->fillcolor);
		break;
	case BLIT_OP_CLR:
		srcsel = dstsel = IMG_FGCOLOR;
		fimg2d4x_set_color_fill(info, 0);
		break;
	case BLIT_OP_SRC:
		dstsel = IMG_FGCOLOR;
		if (!cmd->srcen)
			srcsel = IMG_FGCOLOR;
		break;
	case BLIT_OP_DST:
		srcsel = IMG_FGCOLOR;
		if (!cmd->dsten)
			dstsel = IMG_FGCOLOR;
		break;
	default:	/* alpha blending */
		fimg2d4x_enable_alpha(info, cmd->g_alpha);
		fimg2d4x_set_alpha_composite(info, cmd->op, cmd->g_alpha);
		if (cmd->premult == NON_PREMULTIPLIED)
			fimg2d4x_set_premultiplied(info);
		break;
	}

	fimg2d4x_set_src_type(info, srcsel);
	fimg2d4x_set_dst_type(info, dstsel);

	/* src */
	if (cmd->srcen) {
		fimg2d4x_set_src_image(info, &cmd->src);
		fimg2d4x_set_src_rect(info, &cmd->src_rect);
	}

	/* dst */
	if (cmd->dsten) {
		fimg2d4x_set_dst_image(info, &cmd->dst);
		fimg2d4x_set_dst_rect(info, &cmd->dst_rect);
	}

	/* mask */
	if (cmd->msken) {
		fimg2d4x_enable_msk(info);
		fimg2d4x_set_msk_image(info, &cmd->msk);
		fimg2d4x_set_msk_rect(info, &cmd->msk_rect);
	}

	/* bluescreen */
	if (cmd->bluscr.mode != OPAQUE)
		fimg2d4x_set_bluescreen(info, &cmd->bluscr);

	/* scaling */
	if (cmd->scaling.mode != NO_SCALING) {
		fimg2d4x_set_src_scaling(info, &cmd->scaling);
		if (cmd->msken)
			fimg2d4x_set_msk_scaling(info, &cmd->scaling);
	}

	/* repeat */
	if (cmd->repeat.mode != NO_REPEAT) {
		fimg2d4x_set_src_repeat(info, &cmd->repeat);
		if (cmd->msken)
			fimg2d4x_set_msk_repeat(info, &cmd->repeat);
	}

	/* rotation */
	if (cmd->rotate != ORIGIN)
		fimg2d4x_set_rotation(info, cmd->rotate);

	/* clipping */
	if (cmd->clipping.enable)
		fimg2d4x_enable_clipping(info, &cmd->clipping);

	/* dithering */
	if (cmd->dither)
		fimg2d4x_enable_dithering(info);
}

/**
 * enable irq and start bitblt
 */
static void fimg2d4x_run(struct fimg2d_control *info)
{
	fimg2d_debug("start bitblt\n");
	fimg2d4x_enable_irq(info);
	fimg2d4x_clear_irq(info);
	fimg2d4x_start_blit(info);
}

/**
 * disable irq and wake up thread
 */
static void fimg2d4x_stop(struct fimg2d_control *info)
{
	if (fimg2d4x_is_blit_done(info)) {
		fimg2d_debug("bitblt done\n");
		fimg2d4x_disable_irq(info);
		fimg2d4x_clear_irq(info);
		atomic_set(&info->busy, 0);
		wake_up(&info->wait_q);
	}
}

int fimg2d_register_ops(struct fimg2d_control *info)
{
	info->blit = fimg2d4x_bitblt;
	info->configure = fimg2d4x_configure;
	info->run = fimg2d4x_run;
	info->stop = fimg2d4x_stop;

	return 0;
}
