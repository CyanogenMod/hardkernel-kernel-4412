/* linux/drivers/media/video/samsung/fimg2d4x/fimg2d_cache.c
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

#include <asm/pgtable.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>

#include "fimg2d.h"

static void inner_cache_opr(unsigned long addr, unsigned long size, int dir)
{
	if (dir == DMA_TO_DEVICE || dir == DMA_FROM_DEVICE)
		dmac_map_area((void *)addr, size, dir);
	else /* DMA_BIDIRECTIONAL */
		dmac_flush_range((void *)addr, (void *)(addr + size));
}

#if defined(CONFIG_OUTER_CACHE) && defined(CONFIG_ARM)
static void outer_clean_pagetable(struct mm_struct *mm, unsigned long addr,
                                        unsigned long size)
{
	unsigned long end;
	pgd_t *pgd, *pgd_end;
	pmd_t *pmd;
	pte_t *pte, *pte_end;
	unsigned long next;

	end = addr + size;
	pgd = pgd_offset(mm, addr);
	pgd_end = pgd_offset(mm, end + PGDIR_SIZE - 1);

	/* Clean L1 page table entries */
	outer_flush_range(virt_to_phys(pgd), virt_to_phys(pgd_end));

	/* clean L2 page table entries */
	/* this regards pgd == pmd and no pud */
	do {
		next = pgd_addr_end(addr, end);
		pgd = pgd_offset(mm, addr);
		pmd = pmd_offset(pgd, addr);
		pte = pte_offset_map(pmd, addr) - PTRS_PER_PTE;
		pte_end = pte_offset_map(pmd, next-4) - PTRS_PER_PTE + 1;
		outer_flush_range(virt_to_phys(pte), virt_to_phys(pte_end));
		pte_unmap(pte);
		pte_unmap(pte_end);
		addr = next;
	} while (addr < end);
}

static unsigned long virt2phys(struct mm_struct *mm, unsigned long addr)
{
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *pte;

	pgd = pgd_offset(mm, addr);
	pmd = pmd_offset(pgd, addr);
	pte = pte_offset_map(pmd, addr);

	return (pte_val(*pte) & PAGE_MASK) | (addr & (PAGE_SIZE-1));
}

static void outer_cache_opr(struct mm_struct *mm, unsigned long addr,
					unsigned long size, enum cache_opr opr)
{
	u32 paddr, cur_addr, end_addr;

	cur_addr = addr & PAGE_MASK;
	end_addr = cur_addr + PAGE_ALIGN(size);

	if (opr == CACHE_INVAL) {
		do {
			paddr = virt2phys(mm, cur_addr);
			if (paddr)
				outer_inv_range(paddr, paddr + PAGE_SIZE);
			cur_addr += PAGE_SIZE;
		} while (cur_addr < end_addr);
	} else if (opr == CACHE_CLEAN) {
		do {
			paddr = virt2phys(mm, cur_addr);
			if (paddr)
				outer_clean_range(paddr, paddr + PAGE_SIZE);
			cur_addr += PAGE_SIZE;
		} while (cur_addr < end_addr);
	} else if (opr == CACHE_FLUSH) {
		do {
			paddr = virt2phys(mm, cur_addr);
			if (paddr)
				outer_flush_range(paddr, paddr + PAGE_SIZE);
			cur_addr += PAGE_SIZE;
		} while (cur_addr < end_addr);
	}
}
#else
#define outer_clean_pagetable(mm, addr, size) do {} while(0)
#define outer_cache_opr(mm, addr, size, opr) do {} while(0)
#endif /* CONFIG_OUTER_CACHE && CONFIG_ARM */

void fimg2d_dma_sync_pagetable(struct mm_struct *mm, unsigned long addr,
                                        unsigned long size, enum addr_space type)
{
	if (type != ADDR_USER)
		return;

	outer_clean_pagetable(mm, addr, size);
}

void fimg2d_dma_sync_image(struct mm_struct *mm, unsigned long addr,
			unsigned long size, enum addr_space type, enum cache_opr opr)
{
	int dir;

	if (type != ADDR_USER)
		return;

	if (opr == CACHE_INVAL)
		dir = DMA_FROM_DEVICE;
	else if (opr == CACHE_CLEAN)
		dir = DMA_TO_DEVICE;
	else
		dir = DMA_BIDIRECTIONAL;

	/* flush_cache_user_range(addr, (addr + size)); */
	inner_cache_opr(addr, size, dir);
	outer_cache_opr(mm, addr, size, opr);
}

void fimg2d_dma_sync_all(void)
{
	flush_all_cpu_caches(); /* inner */
	outer_flush_all(); /* outer */
}
