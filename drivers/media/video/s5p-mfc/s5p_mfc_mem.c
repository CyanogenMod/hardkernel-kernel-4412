/*
 * linux/drivers/media/video/s5p-mfc/s5p_mfc_mem.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>

#include "s5p_mfc_mem.h"

#if defined(CONFIG_S5P_MFC_VB2_CMA)
static const char *s5p_mem_types[] = {
	MFC_CMA_BANK2,
	MFC_CMA_BANK1,
	MFC_CMA_FW
};

static unsigned long s5p_mem_alignments[] = {
	MFC_CMA_BANK2_ALIGN,
	MFC_CMA_BANK1_ALIGN,
	MFC_CMA_FW_ALIGN
};
#elif defined(CONFIG_S5P_MFC_VB2_DMA_POOL)
static unsigned long s5p_mem_base_align[] = {
	MFC_BASE_ALIGN_ORDER,
	MFC_BASE_ALIGN_ORDER,
};
static unsigned long s5p_mem_bank_align[] = {
	MFC_BANK_A_ALIGN_ORDER,
	MFC_BANK_B_ALIGN_ORDER,
};

static unsigned long s5p_mem_sizes[] = {
	3 << 20,
	3 << 20,
};
#endif

struct vb2_mem_ops *s5p_mfc_mem_ops(void)
{
#if defined(CONFIG_S5P_MFC_VB2_CMA)
	return (struct vb2_mem_ops *)&vb2_cma_memops;
#elif defined(CONFIG_S5P_MFC_VB2_DMA_POOL)
	return (struct vb2_mem_ops *)&vb2_dma_pool_memops;
#endif
}

void **s5p_mfc_mem_init_multi(struct device *dev)
{
#if defined(CONFIG_S5P_MFC_VB2_CMA)
	return (void **)vb2_cma_init_multi(dev, MFC_CMA_ALLOC_CTX_NUM,
					   s5p_mem_types, s5p_mem_alignments);

#elif defined(CONFIG_S5P_MFC_VB2_DMA_POOL)
	return (void **)vb2_dma_pool_init_multi(dev, MFC_ALLOC_CTX_NUM,
						s5p_mem_base_align,
						s5p_mem_bank_align,
						s5p_mem_sizes);
#endif
}

void s5p_mfc_mem_cleanup_multi(void **alloc_ctxes)
{
#if defined(CONFIG_S5P_MFC_VB2_CMA)
	vb2_cma_cleanup_multi((struct vb2_alloc_ctx **)alloc_ctxes);
#elif defined(CONFIG_S5P_MFC_VB2_DMA_POOL)
	vb2_dma_pool_cleanup_multi(alloc_ctxes, MFC_ALLOC_CTX_NUM);
#endif
}

void s5p_mfc_cache_clean(const void *start_addr, unsigned long size)
{
	unsigned long paddr;

	dmac_map_area(start_addr, size, DMA_TO_DEVICE);
	paddr = __pa((unsigned long)start_addr);
	outer_clean_range(paddr, paddr + size);
}

void s5p_mfc_cache_inv(const void *start_addr, unsigned long size)
{
	unsigned long paddr;

	paddr = __pa((unsigned long)start_addr);
	outer_inv_range(paddr, paddr + size);
	dmac_unmap_area(start_addr, size, DMA_FROM_DEVICE);
}
