/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file
 * Implementation of the OS abstraction layer for the kernel device driver
 */

#ifndef _OSK_ARCH_LOW_LEVEL_MEM_H_
#define _OSK_ARCH_LOW_LEVEL_MEM_H_

#ifndef _OSK_H_
#error "Include mali_osk.h directly"
#endif

#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/dma-mapping.h>

struct osk_phy_allocator
{
};

OSK_STATIC_INLINE osk_error osk_phy_allocator_init(osk_phy_allocator * const allocator, osk_phy_addr mem, u32 nr_pages)
{
	return OSK_ERR_NONE;
}

OSK_STATIC_INLINE void osk_phy_allocator_term(osk_phy_allocator *allocator)
{
}

OSK_STATIC_INLINE osk_error osk_phy_pages_alloc(osk_phy_allocator *allocator,
					     u32 nr_pages, osk_phy_addr *pages)
{
	int i;

	for (i = 0; i < nr_pages; i++)
	{
		struct page *p;
		void * mp;

		p = alloc_page(GFP_HIGHUSER);
		if (NULL == p)
		{
			goto out;
		}

		mp = kmap(p);
		if (NULL == mp)
		{
			__free_page(p);
			goto out;
		}

		memset(mp, 0x00, PAGE_SIZE); /* instead of __GFP_ZERO, so we can do cache maintenance */
		osk_sync_to_memory(PFN_PHYS(page_to_pfn(p)), mp, PAGE_SIZE);
		kunmap(p);

		pages[i] = PFN_PHYS(page_to_pfn(p));
	}

	return OSK_ERR_NONE;

out:
	if (i)
	{
		osk_phy_pages_free(allocator, i, pages);
	}

	return OSK_ERR_ALLOC;
}

static inline void osk_phy_pages_free(osk_phy_allocator *allocator,
				      u32 nr_pages, osk_phy_addr *pages)
{
	int i;

	for (i = 0; i < nr_pages; i++)
	{
		if (0 != pages[i])
		{
			__free_page(pfn_to_page(PFN_DOWN(pages[i])));
			pages[i] = (osk_phy_addr)0;
		}
	}
}

static inline void *osk_kmap(osk_phy_allocator *allocator, osk_phy_addr page)
{
	return kmap(pfn_to_page(PFN_DOWN(page)));
}

static inline void osk_kunmap(osk_phy_allocator *allocator, osk_phy_addr page)
{
	kunmap(pfn_to_page(PFN_DOWN(page)));
}

static inline void osk_sync_to_memory(osk_phy_addr paddr, osk_virt_addr vaddr, size_t sz)
{
#ifdef CONFIG_ARM
	dmac_flush_range(vaddr, vaddr+sz-1);
	outer_flush_range(paddr, paddr+sz-1);
#elif defined(CONFIG_X86)
	struct scatterlist scl = {0, };
	sg_set_page(&scl, pfn_to_page(PFN_DOWN(paddr)), sz,
			paddr & (PAGE_SIZE -1 ));
	dma_sync_sg_for_cpu(NULL, &scl, 1, DMA_TO_DEVICE);
	mb(); /* for outer_sync (if needed) */
#else
#error Implement cache maintenance for your architecture here
#endif
}

static inline void osk_sync_to_cpu(osk_phy_addr paddr, osk_virt_addr vaddr, size_t sz)
{
#ifdef CONFIG_ARM
	dmac_flush_range(vaddr, vaddr+sz-1);
	outer_flush_range(paddr, paddr+sz-1);
#elif defined(CONFIG_X86)
	struct scatterlist scl = {0, };
	sg_set_page(&scl, pfn_to_page(PFN_DOWN(paddr)), sz,
			paddr & (PAGE_SIZE -1 ));
	dma_sync_sg_for_cpu(NULL, &scl, 1, DMA_FROM_DEVICE);
#else
#error Implement cache maintenance for your architecture here
#endif
}

#endif /* _OSK_ARCH_LOW_LEVEL_MEM_H_ */
