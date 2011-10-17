/* linux/drivers/media/video/exynos/gsc-vb2.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Videobuf2 allocator operations file
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include "gsc-core.h"

void *gsc_cma_init(struct gsc_dev *gsc)
{
	return vb2_cma_phys_init(&gsc->pdev->dev, NULL, 0, false);
}

void gsc_cma_resume(void *alloc_ctx){}
void gsc_cma_suspend(void *alloc_ctx){}
void gsc_cma_set_cacheable(void *alloc_ctx, bool cacheable){}

int gsc_cma_cache_flush(struct vb2_buffer *vb, u32 plane_no)
{
	return 0;
}

const struct gsc_vb2 gsc_vb2_cma = {
	.ops		= &vb2_cma_phys_memops,
	.init		= gsc_cma_init,
	.cleanup	= vb2_cma_phys_cleanup,
	.plane_addr	= vb2_cma_phys_plane_paddr,
	.resume		= gsc_cma_resume,
	.suspend	= gsc_cma_suspend,
	.cache_flush	= gsc_cma_cache_flush,
	.set_cacheable	= gsc_cma_set_cacheable,
};
