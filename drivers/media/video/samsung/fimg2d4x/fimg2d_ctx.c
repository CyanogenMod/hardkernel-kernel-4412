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

inline void fimg2d_enqueue(struct fimg2d_control *info,
			struct list_head *node, struct list_head *q)
{
	list_add_tail(node, q);
}

inline void fimg2d_dequeue(struct fimg2d_control *info, struct list_head *node)
{
	list_del(node);
}

inline int fimg2d_queue_is_empty(struct list_head *q)
{
	return list_empty(q);
}

inline struct fimg2d_bltcmd *fimg2d_get_first_command(struct fimg2d_control *info)
{
	if (list_empty(&info->cmd_q))
		return NULL;
	else
		return list_first_entry(&info->cmd_q, struct fimg2d_bltcmd, node);
}

int fimg2d_add_command(struct fimg2d_control *info, struct fimg2d_context *ctx,
			struct fimg2d_blit __user *u)
{
	struct fimg2d_bltcmd *cmd;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		printk(KERN_ERR "failed to create bitblt command header\n");
		return -ENOMEM;
	}

	cmd->ctx = ctx;
	cmd->seq_no = u->seq_no;

	fimg2d_debug("context(%p), seq_no(%u)\n", cmd->ctx, cmd->seq_no);

	cmd->op = u->op;
	cmd->fillcolor = u->fillcolor;
	cmd->g_alpha = u->g_alpha;
	cmd->premult = u->premult;
	cmd->dither = u->dither;
	cmd->rotate = u->rotate;

	if (u->scaling) {
		if (copy_from_user(&cmd->scaling, u->scaling, sizeof(cmd->scaling)))
			goto err_user;
	}

	if (u->repeat) {
		if (copy_from_user(&cmd->repeat, u->repeat, sizeof(cmd->repeat)))
			goto err_user;
	}

	if (u->bluscr) {
		if (copy_from_user(&cmd->bluscr, u->bluscr, sizeof(cmd->bluscr)))
			goto err_user;
	}

	if (u->clipping) {
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

	/* add command node and increase ncmd */
	spin_lock(&info->bltlock);
	atomic_inc(&ctx->ncmd);
	fimg2d_enqueue(info, &cmd->node, &info->cmd_q);
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
}

inline void fimg2d_del_context(struct fimg2d_control *info, struct fimg2d_context *ctx)
{
	atomic_dec(&info->nctx);
}

