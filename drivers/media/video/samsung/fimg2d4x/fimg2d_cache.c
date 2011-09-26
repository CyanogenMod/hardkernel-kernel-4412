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

#define LV1_SHIFT		20
#define LV1_PT_SIZE		SZ_1M
#define LV2_PT_SIZE		SZ_1K
#define LV2_BASE_MASK		0x3ff
#define LV2_PT_MASK		0xff000
#define LV2_SHIFT		12
#define DESC_MASK		0x3

static inline unsigned long virt2phys(struct mm_struct *mm, unsigned long addr)
{
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *pte;

	pgd = pgd_offset(mm, addr);
	pmd = pmd_offset(pgd, addr);
	pte = pte_offset_map(pmd, addr);

	return (pte_val(*pte) & PAGE_MASK) | (addr & (PAGE_SIZE-1));
}

#ifdef CONFIG_OUTER_CACHE
void fimg2d_clean_outer_pagetable(struct mm_struct *mm, unsigned long addr,
                                        size_t size)
{
	unsigned long *lv1, *lv1end;
	unsigned long lv2pa;
	unsigned long pgd;

	pgd = (unsigned long)mm->pgd;

	lv1 = (unsigned long *)pgd + (addr >> LV1_SHIFT);
	lv1end = (unsigned long *)pgd + ((addr + size + LV1_PT_SIZE-1) >> LV1_SHIFT);

	/* clean level1 page table */
	outer_clean_range(virt_to_phys(lv1), virt_to_phys(lv1end));

	do {
		lv2pa = *lv1 & ~LV2_BASE_MASK;	/* lv2 pt base */
		/* clean level2 page table */
		outer_clean_range(lv2pa, lv2pa + LV2_PT_SIZE);
		lv1++;
	} while (lv1 != lv1end);
}

void fimg2d_dma_sync_outer(struct mm_struct *mm, unsigned long addr,
					size_t size, enum cache_opr opr)
{
	unsigned long paddr, cur_addr, end_addr;

	cur_addr = addr & PAGE_MASK;
	end_addr = cur_addr + PAGE_ALIGN(size);

	if (opr == CACHE_CLEAN) {
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

enum pt_status fimg2d_check_pagetable(struct mm_struct *mm, unsigned long addr,
					size_t size)
{
	unsigned long *lv1d, *lv2d;
	unsigned long pgd;

	pgd = (unsigned long)mm->pgd;

	do {
		lv1d = (unsigned long *)pgd + (addr >> LV1_SHIFT);

		/* check level 1 descriptor */
		if ((*lv1d & DESC_MASK) == 0x0 || ((*lv1d & DESC_MASK) == 0x3)) {
			printk(KERN_ERR "[%s] LV1 PT fault, pgd 0x%lx addr 0x%lx"
					"lv1 descriptor 0x%lx\n",
					__func__, pgd, addr, *lv1d);
			return PT_FAULT;
		}

		/* check level 2 descriptor */
		lv2d = (unsigned long *)phys_to_virt(*lv1d & ~LV2_BASE_MASK) +
				((addr & LV2_PT_MASK) >> LV2_SHIFT);

		if ((*lv2d & DESC_MASK) == 0x0) {
			printk(KERN_ERR "[%s] LV2 PT fault, pgd 0x%lx addr 0x%lx "
					"lv2 descriptor 0x%lx\n",
					__func__, pgd, addr, *lv2d);
			return PT_FAULT;
		}

		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	} while ((long)size > 0);

	return PT_NORMAL;
}
#endif /* CONFIG_OUTER_CACHE */
