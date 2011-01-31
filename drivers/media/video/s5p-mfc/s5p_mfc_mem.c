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

#include "s5p_mfc_common.h"
#include "s5p_mfc_mem.h"
#include "s5p_mfc_pm.h"
#include "s5p_mfc_debug.h"

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

struct vb2_mem_ops *s5p_mfc_mem_ops(void)
{
	return (struct vb2_mem_ops *)&vb2_cma_memops;
}

void **s5p_mfc_mem_init_multi(struct device *dev)
{
	return (void **)vb2_cma_init_multi(dev, MFC_CMA_ALLOC_CTX_NUM,
					   s5p_mem_types, s5p_mem_alignments);
}

void s5p_mfc_mem_cleanup_multi(void **alloc_ctxes)
{
	vb2_cma_cleanup_multi((struct vb2_alloc_ctx **)alloc_ctxes);
}
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

struct vb2_mem_ops *s5p_mfc_mem_ops(void)
{
	return (struct vb2_mem_ops *)&vb2_dma_pool_memops;
}

void **s5p_mfc_mem_init_multi(struct device *dev)
{
	return (void **)vb2_dma_pool_init_multi(dev, MFC_ALLOC_CTX_NUM,
						s5p_mem_base_align,
						s5p_mem_bank_align,
						s5p_mem_sizes);
}

void s5p_mfc_mem_cleanup_multi(void **alloc_ctxes)
{
	vb2_dma_pool_cleanup_multi(alloc_ctxes, MFC_ALLOC_CTX_NUM);
}
#elif defined(CONFIG_S5P_MFC_VB2_SDVMM)
static void s5p_mfc_tlb_invalidate(enum vcm_dev_id id)
{
	//if (s5p_mfc_power_chk()) {
		s5p_mfc_clock_on();

		sysmmu_tlb_invalidate(SYSMMU_MFC_L);
		sysmmu_tlb_invalidate(SYSMMU_MFC_R);

		s5p_mfc_clock_off();
	//}
}

static void s5p_mfc_set_pagetable(enum vcm_dev_id id, unsigned long base)
{
	mfc_debug("base: 0x%08lx\n", base);

	//if (s5p_mfc_power_chk()) {
		s5p_mfc_clock_on();

		sysmmu_set_tablebase_pgd(SYSMMU_MFC_L, base);
		sysmmu_set_tablebase_pgd(SYSMMU_MFC_R, base);

		s5p_mfc_clock_off();
	//}
}

const static struct s5p_vcm_driver s5p_mfc_vcm_driver = {
	.tlb_invalidator = &s5p_mfc_tlb_invalidate,
	.pgd_base_specifier = &s5p_mfc_set_pagetable,
	.phys_alloc = NULL,
	.phys_free = NULL,
};

struct vb2_mem_ops *s5p_mfc_mem_ops(void)
{
	return (struct vb2_mem_ops *)&vb2_sdvmm_memops;
}

void **s5p_mfc_mem_init_multi(struct device *dev)
{
	struct vb2_vcm vcm;
	
	vcm.vcm_id = VCM_DEV_MFC;
	vcm.driver = &s5p_mfc_vcm_driver;
	/* FIXME: check port count */
	vcm.size = SZ_256M;

	sysmmu_on(SYSMMU_MFC_L);
	sysmmu_on(SYSMMU_MFC_R);
	
	//vcm_set_pgtable_base(VCM_DEV_MFC);
	
	return (void **)vb2_sdvmm_init_multi(MFC_ALLOC_CTX_NUM, &vcm, NULL);
}

void s5p_mfc_mem_cleanup_multi(void **alloc_ctxes)
{
	vb2_sdvmm_cleanup_multi(alloc_ctxes);

	sysmmu_off(SYSMMU_MFC_L);
	sysmmu_off(SYSMMU_MFC_R);
}
#endif

#if defined(CONFIG_S5P_MFC_VB2_SDVMM)
void s5p_mfc_cache_clean(const void *start_addr, unsigned long size)
{
	unsigned long paddr;
	void *cur_addr, *end_addr;

	dmac_map_area(start_addr, size, DMA_TO_DEVICE);

	cur_addr = (void *)((unsigned long)start_addr & PAGE_MASK);
	end_addr = cur_addr + PAGE_ALIGN(size);

	while (cur_addr < end_addr) {
		paddr = page_to_pfn(vmalloc_to_page(cur_addr));
		paddr <<= PAGE_SHIFT;
		if (paddr)
			outer_clean_range(paddr, paddr + PAGE_SIZE);
		cur_addr += PAGE_SIZE;
	}

	/* FIXME: L2 operation optimization */
	/*
	unsigned long start, end, unitsize;
	unsigned long cur_addr, remain;

	dmac_map_area(start_addr, size, DMA_TO_DEVICE);

	cur_addr = (unsigned long)start_addr;
	remain = size;

	start = page_to_pfn(vmalloc_to_page(cur_addr));
	start <<= PAGE_SHIFT;
	if (start & PAGE_MASK) {
		unitsize = min((start | PAGE_MASK) - start + 1, remain);
		end = start + unitsize;
		outer_clean_range(start, end);
		remain -= unitsize;
		cur_addr += unitsize;
	}

	while (remain >= PAGE_SIZE) {
		start = page_to_pfn(vmalloc_to_page(cur_addr));
		start <<= PAGE_SHIFT;
		end = start + PAGE_SIZE;
		outer_clean_range(start, end);
		remain -= PAGE_SIZE;
		cur_addr += PAGE_SIZE;
	}

	if (remain) {
		start = page_to_pfn(vmalloc_to_page(cur_addr));
		start <<= PAGE_SHIFT;
		end = start + remain;
		outer_clean_range(start, end);
	}
	*/

}

void s5p_mfc_cache_inv(const void *start_addr, unsigned long size)
{
	unsigned long paddr;
	void *cur_addr, *end_addr;

	cur_addr = (void *)((unsigned long)start_addr & PAGE_MASK);
	end_addr = cur_addr + PAGE_ALIGN(size);

	while (cur_addr < end_addr) {
		paddr = page_to_pfn(vmalloc_to_page(cur_addr));
		paddr <<= PAGE_SHIFT;
		if (paddr)
			outer_inv_range(paddr, paddr + PAGE_SIZE);
		cur_addr += PAGE_SIZE;
	}

	dmac_unmap_area(start_addr, size, DMA_FROM_DEVICE);

	/* FIXME: L2 operation optimization */
	/*
	unsigned long start, end, unitsize;
	unsigned long cur_addr, remain;

	cur_addr = (unsigned long)start_addr;
	remain = size;

	start = page_to_pfn(vmalloc_to_page(cur_addr));
	start <<= PAGE_SHIFT;
	if (start & PAGE_MASK) {
		unitsize = min((start | PAGE_MASK) - start + 1, remain);
		end = start + unitsize;
		outer_inv_range(start, end);
		remain -= unitsize;
		cur_addr += unitsize;
	}

	while (remain >= PAGE_SIZE) {
		start = page_to_pfn(vmalloc_to_page(cur_addr));
		start <<= PAGE_SHIFT;
		end = start + PAGE_SIZE;
		outer_inv_range(start, end);
		remain -= PAGE_SIZE;
		cur_addr += PAGE_SIZE;
	}

	if (remain) {
		start = page_to_pfn(vmalloc_to_page(cur_addr));
		start <<= PAGE_SHIFT;
		end = start + remain;
		outer_inv_range(start, end);
	}

	dmac_unmap_area(start_addr, size, DMA_FROM_DEVICE);
	*/
}
#else
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
#endif
