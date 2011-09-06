/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2010-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file mali_kbase_mmmu.c
 * Base kernel MMU management.
 */

/* #define DEBUG    1 */
#include <osk/mali_osk.h>
#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_midg_regmap.h>

#define beenthere(f, a...)  OSK_PRINT_INFO(OSK_BASE_MMU, "%s:" f, __func__, ##a)

#include <kbase/src/common/mali_kbase_mem.h>
#include <kbase/src/common/mali_kbase_defs.h>


/*
 * Definitions:
 * - PGD: Page Directory.
 * - PTE: Page Table Entry. A 64bit value pointing to the next
 *        level of translation
 * - ATE: Address Transation Entry. A 64bit value pointing to
 *        a 4kB physical page.
 */

static void kbase_mmu_report_fault_and_kill(kbase_context *kctx, kbase_as * as, mali_addr64 fault_addr);
static void fault_worker(osk_workq_work *data);
static u8 region_size(u64 pfn_width);

static void ksync_kern_vrange_gpu(osk_phy_addr paddr, osk_virt_addr vaddr, size_t size)
{
	osk_sync_to_memory(paddr, vaddr, size);
}

static u64 make_multiple(u64 minimum, u32 multiple)
{
	/* Note: due to a ARM gcc compiler issue this had to be marked as volatile to avoid gcc ignoring my copy and overwriting the minimum variable */
	volatile u64 copy = minimum;
	u32 remainder = osk_divmod6432((u64*)&copy, multiple);
	if (remainder == 0)
	{
		return minimum;
	}
	else
	{
		return minimum + multiple - remainder;
	}
}

static void mmu_mask_reenable(kbase_device * kbdev, kbase_context *kctx, kbase_as * as)
{
	u32 mask;

	osk_spinlock_irq_lock(&kbdev->mmu_mask_change);
	mask = kbase_reg_read(kbdev, MMU_REG(MMU_IRQ_MASK), kctx);
	mask |= ((1UL << as->number) | (1UL << (16 + as->number)));
	kbase_reg_write(kbdev, MMU_REG(MMU_IRQ_MASK), mask, kctx);
	osk_spinlock_irq_unlock(&kbdev->mmu_mask_change);
}

static void fault_worker(osk_workq_work *data)
{
	u64 fault_pfn;
	u64 new_pages;
	u64 fault_rel_pfn;
	kbase_as * faulting_as;
	int as_no;
	kbase_context * kctx;
	kbase_device * kbdev;
	kbasep_js_device_data *js_devdata;
	kbase_va_region *region;
	kbasep_js_kctx_info *js_kctx_info;

	faulting_as = CONTAINER_OF(data, kbase_as, work);
	fault_pfn = faulting_as->fault_addr >> OSK_PAGE_SHIFT;
	as_no = faulting_as->number;

	kbdev = CONTAINER_OF( faulting_as, kbase_device, as[as_no] );
	js_devdata = &kbdev->js_data;

	osk_spinlock_irq_lock( &js_devdata->runpool_lock );
	kctx = kbasep_js_policy_runpool_lookup_ctx( &js_devdata->policy, as_no );
	osk_spinlock_irq_unlock( &js_devdata->runpool_lock );
	/* Context cannot be descheduled after we release the lock, because we've
	 * ref-counted the pf_busy_count */

	OSK_ASSERT( kctx != NULL );
	OSK_ASSERT( kctx->kbdev == kbdev );

	js_kctx_info = &kctx->jctx.sched_info;

	kbase_gpu_vm_lock(kctx);

	/* find the region object for this VA */
	region = kbase_region_lookup(kctx, faulting_as->fault_addr);
	if (NULL == region || (GROWABLE_FLAGS_REQUIRED != (region->flags & GROWABLE_FLAGS_MASK)))
	{
		kbase_gpu_vm_unlock(kctx);
		/* failed to find the region or mismatch of the flags */
		kbase_mmu_report_fault_and_kill(kctx, faulting_as, faulting_as->fault_addr);
		goto fault_done;
	}

	/* find the size we need to grow it by */
	fault_rel_pfn = fault_pfn - region->start_pfn;
	new_pages = make_multiple(fault_rel_pfn - region->nr_alloc_pages + 1, region->extent);
	if (new_pages + region->nr_alloc_pages > region->nr_pages)
	{
		/* cap to max vsize */
		new_pages = region->nr_pages - region->nr_alloc_pages;
	}

	if (OSK_ERR_NONE == osk_phy_pages_alloc(&region->phy_allocator, new_pages, &region->phy_pages[region->nr_alloc_pages]))
	{
		/* alloc success */
		mali_addr64 lock_addr;

		/* AS transaction begin */
		osk_spinlock_irq_lock(&faulting_as->transaction_lock);

		/* Lock the VA region we're about to update */
		lock_addr = (faulting_as->fault_addr & OSK_PAGE_MASK) | region_size(new_pages);
		kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_LOCKADDR_LO), lock_addr & 0xFFFFFFFFUL, kctx);
		kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_LOCKADDR_HI), lock_addr >> 32, kctx);
		kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_COMMAND), 2/*LOCK*/, kctx);

		/* set up the new pages */
		kbase_mmu_insert_pages(kctx, region->start_pfn + region->nr_alloc_pages,
				&region->phy_pages[region->nr_alloc_pages],
				new_pages, region->flags);

		/* clear the irq */
		/* MUST BE BEFORE THE FLUSH/UNLOCK */
		kbase_reg_write(kbdev, MMU_REG(MMU_IRQ_CLEAR), (1UL << as_no), NULL);

		/* flush L2 and unlock the VA (resumes the MMU) */
		kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_COMMAND), 4/*FLUSH*/, kctx);

		/* wait for the flush to complete */
		while (kbase_reg_read(kbdev, MMU_AS_REG(as_no, ASn_STATUS), kctx) & 1);
	
		osk_spinlock_irq_unlock(&faulting_as->transaction_lock);
		/* AS transaction end */

		/* reenable this in the mask */
		mmu_mask_reenable(kbdev, kctx, faulting_as);

		/* track the new count */
		region->nr_alloc_pages += new_pages;
		OSK_ASSERT(region->nr_alloc_pages <= region->nr_pages);
		kbase_gpu_vm_unlock(kctx);
	}
	else
	{
		/* failed to extend, handle as a normal PF */
		kbase_gpu_vm_unlock(kctx);
		kbase_mmu_report_fault_and_kill(kctx, faulting_as, faulting_as->fault_addr);
	}


fault_done:
	/* By this point, the fault was handled in some way, so release the lock count */
	{
		int new_lock_count;
		mali_bool need_to_try_schedule_context = MALI_FALSE;

		/* AS transaction begin */
		osk_spinlock_irq_lock(&faulting_as->transaction_lock);
		new_lock_count = --(faulting_as->pf_busy_count);
		osk_spinlock_irq_unlock( &faulting_as->transaction_lock );
		/* AS transaction end */

		if ( new_lock_count != 0 )
		{
			/* No need to check and de-schedule the context if others pfs are busy */
			return;
		}

		/* If this was the last busy page fault, check and deschedule (since
		 * that would've been cancelled if the page fault was busy the last
		 * time it happened. */
		osk_mutex_lock( &js_kctx_info->ctx.jsctx_lock  );
		if ( js_kctx_info->ctx.is_scheduled != MALI_FALSE )
		{
			need_to_try_schedule_context = kbasep_js_check_and_deschedule_ctx( kbdev, kctx );
		}
		osk_mutex_unlock( &js_kctx_info->ctx.jsctx_lock );

		/* Handle any queued actions from the lock */
		if ( need_to_try_schedule_context != MALI_FALSE )
		{
			kbasep_js_try_schedule_head_ctx( kbdev );
		}
	}
}

osk_phy_addr kbase_mmu_alloc_pgd(kbase_context *kctx)
{
	osk_phy_addr pgd;
	osk_error err;
	u64 *page;
	int i;

	err = osk_phy_pages_alloc(&kctx->pgd_allocator, 1, &pgd);
	if (OSK_ERR_NONE != err)
		return 0;

	page = osk_kmap(&kctx->pgd_allocator, pgd);
	for (i = 0; i < 512; i++)
		page[i] = ENTRY_IS_INVAL;

	/* Clean the full page */
	ksync_kern_vrange_gpu(pgd, page, 512 * sizeof(u64));
	osk_kunmap(&kctx->pgd_allocator, pgd);
	return pgd;
}

static osk_phy_addr mmu_pte_to_phy_addr(u64 entry)
{
	if (!(entry & 1))
		return 0;

	return entry & ~0xFFF;
}

static u64 mmu_phyaddr_to_pte(osk_phy_addr phy)
{
	return (phy & ~0xFFF) | ENTRY_IS_PTE;
}

static u64 mmu_phyaddr_to_ate(osk_phy_addr phy, u64 flags)
{
	return (phy & ~0xFFF) | (flags & ENTRY_FLAGS_MASK) | ENTRY_IS_ATE;
}

/* Given PGD PFN for level N, return PGD PFN for level N+1 */
static osk_phy_addr mmu_get_next_pgd(struct kbase_context *kctx,
                                     osk_phy_addr pgd, u64 vpfn, int level)
{
	u64 *page;
	osk_phy_addr target_pgd;

	/*
	 * Architecture spec defines level-0 as being the top-most.
	 * This is a bit unfortunate here, but we keep the same convention.
	 */
	vpfn >>= (3 - level) * 9;
	vpfn &= 0x1FF;

	page = osk_kmap(&kctx->pgd_allocator, pgd);
	target_pgd = mmu_pte_to_phy_addr(page[vpfn]);

	if (!target_pgd) {
		target_pgd = kbase_mmu_alloc_pgd(kctx);
		OSK_ASSERT(target_pgd);
		
		page[vpfn] = mmu_phyaddr_to_pte(target_pgd);
		ksync_kern_vrange_gpu(pgd + (vpfn * sizeof(u64)), page + vpfn, sizeof(u64));
		/* Rely on the caller to update the address space flags. */
	}

	osk_kunmap(&kctx->pgd_allocator, pgd);
	return target_pgd;
}

static osk_phy_addr mmu_get_bottom_pgd(struct kbase_context *kctx, u64 vpfn)
{
	osk_phy_addr pgd;
	int l;

	pgd = kctx->pgd;

	for (l = MIDGARD_MMU_TOPLEVEL; l < 3; l++) {
		pgd = mmu_get_next_pgd(kctx, pgd, vpfn, l);
		OSK_ASSERT(pgd);
	}

	return pgd;
}

/*
 * Map 'nr' pages pointed to by 'phys' at GPU PFN 'vpfn'
 */
void kbase_mmu_insert_pages(struct kbase_context *kctx, u64 vpfn,
                            osk_phy_addr *phys, u32 nr, u16 flags)
{
	osk_phy_addr pgd;
	u64 *pgd_page;
	u64 mmu_flags;

	mmu_flags = ENTRY_RD_BIT; /* GPU read always given */
	mmu_flags |= (flags & KBASE_REG_GPU_RW) ? ENTRY_WR_BIT : 0; /* write perm if requested */
	mmu_flags |= (flags & KBASE_REG_GPU_NX) ? ENTRY_NX_BIT : 0; /* nx if requested */

	while (nr) {
		u32 i;
		u32 index = vpfn & 0x1FF;
		u32 count = 512 - index;
		
		if (count > nr)
			count = nr;

		/*
		 * Repeatedly calling mmu_get_bottom_pte() is clearly
		 * suboptimal. We don't have to re-parse the whole tree
		 * each time (just cache the l0-l2 sequence).
		 * On the other hand, it's only a gain when we map more than
		 * 256 pages at once (on average). Do we really care?
		 */
		pgd = mmu_get_bottom_pgd(kctx, vpfn);
		OSK_ASSERT(pgd);

		pgd_page = osk_kmap(&kctx->pgd_allocator, pgd);

		for (i = 0; i < count; i++) {
			OSK_ASSERT(0 == (pgd_page[index + i] & 1UL));
			pgd_page[index + i] = mmu_phyaddr_to_ate(phys[i], mmu_flags);
		}

		phys += count;
		vpfn += count;
		nr -= count;

		ksync_kern_vrange_gpu(pgd + (index * sizeof(u64)), pgd_page + index, count * sizeof(u64));

		osk_kunmap(&kctx->pgd_allocator, pgd);
	}
}

/*
 * We actually only discard the ATE, and not the page table
 * pages. There is a potential DoS here, as we'll leak memory by
 * having PTEs that are potentially unused.  Will require physical
 * page accounting, so MMU pages are part of the process allocation.
 */
void kbase_mmu_teardown_pages(struct kbase_context *kctx, u64 vpfn, u32 nr)
{
	osk_phy_addr pgd;
	u64 *pgd_page;
	kbase_device *kbdev;

	beenthere("kctx %p vpfn %lx nr %d", (void *)kctx, (unsigned long)vpfn, nr);

	kbdev = kctx->kbdev;

	while (nr)
	{
		u32 i;
		u32 index = vpfn & 0x1FF;
		u32 count = 512 - index;
		if (count > nr)
			count = nr;

		pgd = mmu_get_bottom_pgd(kctx, vpfn);
		OSK_ASSERT(pgd);

		pgd_page = osk_kmap(&kctx->pgd_allocator, pgd);

		for (i = 0; i < count; i++) {
			/*
			 * Possible micro-optimisation: only write to the
			 * low 32bits. That's enough to invalidate the mapping.
			 */
			pgd_page[index + i] = ENTRY_IS_INVAL;
		}

		vpfn += count;
		nr -= count;

		ksync_kern_vrange_gpu(pgd + (index * sizeof(u64)), pgd_page + index, count * sizeof(u64));

		osk_kunmap(&kctx->pgd_allocator, pgd);
	}

	/* lock the runpool whilst deciding on whether to flush based on number of jobs running */
	osk_spinlock_irq_lock( &kbdev->js_data.runpool_lock );
	if (kctx->jctx.sched_info.runpool.nr_running_jobs > 0)
	{
		/* Lock the VA region we're about to update */
		u64 lock_addr = (vpfn << OSK_PAGE_SHIFT ) | region_size(nr);

		OSK_ASSERT( kctx->as_nr >= 0 );

		/* AS transaction begin */
		osk_spinlock_irq_lock(&kbdev->as[kctx->as_nr].transaction_lock);
		kbase_reg_write(kctx->kbdev, MMU_AS_REG(kctx->as_nr, ASn_LOCKADDR_LO), lock_addr & 0xFFFFFFFFUL, kctx);
		kbase_reg_write(kctx->kbdev, MMU_AS_REG(kctx->as_nr, ASn_LOCKADDR_HI), lock_addr >> 32, kctx);
		kbase_reg_write(kctx->kbdev, MMU_AS_REG(kctx->as_nr, ASn_COMMAND), 2/*LOCK*/, kctx);

		/* flush L2 and unlock the VA */
		kbase_reg_write(kctx->kbdev, MMU_AS_REG(kctx->as_nr, ASn_COMMAND), 4/*FLUSH*/, kctx);

		/* wait for the flush to complete */
		while (kbase_reg_read(kctx->kbdev, MMU_AS_REG(kctx->as_nr, ASn_STATUS), kctx) & 1);

		osk_spinlock_irq_unlock(&kbdev->as[kctx->as_nr].transaction_lock);
		/* AS transaction end */
	}
	osk_spinlock_irq_unlock( &kbdev->js_data.runpool_lock );
}

static int mmu_pte_is_valid(u64 pte)
{
	return ((pte & 3) == ENTRY_IS_ATE);
}

/* This is a debug feature only */
static void mmu_check_unused(kbase_context *kctx, osk_phy_addr pgd)
{
	u64 *page;
	int i;

	page = osk_kmap(&kctx->pgd_allocator, pgd);
	for (i = 0; i < 512; i++)
		if (mmu_pte_is_valid(page[i]))
			beenthere("live pte %016lx", (unsigned long)page[i]);
	osk_kunmap(&kctx->pgd_allocator, pgd);
}

static void mmu_teardown_level(kbase_context *kctx, osk_phy_addr pgd, int level, int zap)
{
	osk_phy_addr target_pgd;
	u64 *pgd_page;
	int i;

	pgd_page = osk_kmap(&kctx->pgd_allocator, pgd);

	for (i = 0; i < 512; i++) {
		target_pgd = mmu_pte_to_phy_addr(pgd_page[i]);

		if (target_pgd) {
			if (level < 2)
				mmu_teardown_level(kctx, target_pgd, level + 1, zap);
			else {
				/*
				 * So target_pte is a level-3 page.
				 * As a leaf, it is safe to free it.
				 * Unless we have live pages attached to it!
				 */
				mmu_check_unused(kctx, target_pgd);
			}

			beenthere("pte %lx level %d", (unsigned long)target_pgd, level + 1);
			if (zap)
				osk_phy_pages_free(&kctx->pgd_allocator, 1, &target_pgd);
		}
	}

	osk_kunmap(&kctx->pgd_allocator, pgd);
}

void kbase_mmu_free_pgd(struct kbase_context *kctx)
{
	mmu_teardown_level(kctx, kctx->pgd, MIDGARD_MMU_TOPLEVEL, 1);
	beenthere("pgd %lx", (unsigned long)kctx->pgd);
	osk_phy_pages_free(&kctx->pgd_allocator, 1, &kctx->pgd);
}

static size_t kbasep_mmu_dump_level(kbase_context *kctx, osk_phy_addr pgd, int level, char **buffer, size_t *size_left)
{
	osk_phy_addr target_pgd;
	u64 *pgd_page;
	int i;
	size_t size = 512*sizeof(u64)+sizeof(u64);
	
	pgd_page = osk_kmap(&kctx->pgd_allocator, pgd);
	
	if (*size_left >= size)
	{
		/* A modified physical address that contains the page table level */
		u64 m_pgd = pgd | level;

		/* Put the modified physical address in the output buffer */
		memcpy(*buffer, &m_pgd, sizeof(u64));
		*buffer += sizeof(osk_phy_addr);

		/* Followed by the page table itself */
		memcpy(*buffer, pgd_page, sizeof(u64)*512);
		*buffer += sizeof(u64)*512;

		*size_left -= size;
	}
	
	for (i = 0; i < 512; i++) {
		if ((pgd_page[i] & ENTRY_IS_PTE) == ENTRY_IS_PTE) {
			target_pgd = mmu_pte_to_phy_addr(pgd_page[i]);

			size += kbasep_mmu_dump_level(kctx, target_pgd, level + 1, buffer, size_left);
		}
	}

	osk_kunmap(&kctx->pgd_allocator, pgd);

	return size;
}

void *kbase_mmu_dump(struct kbase_context *kctx,int nr_pages)
{
	void *kaddr;
	size_t size_left;

	if (0 == nr_pages)
	{
		/* can't find in a 0 sized buffer, early out */
		return NULL;
	}

	size_left = nr_pages * OSK_PAGE_SIZE;

	kaddr = osk_vmalloc(size_left);

	if (kaddr)
	{
		u64 end_marker = 0xFFULL;
		char *buffer = (char*)kaddr;

		size_t size = kbasep_mmu_dump_level(kctx, kctx->pgd, MIDGARD_MMU_TOPLEVEL, &buffer, &size_left);

		/* Add on the size for the end marker */
		size += sizeof(u64);

		if (size > nr_pages * OSK_PAGE_SIZE || size_left < sizeof(u64)) {
			/* The buffer isn't big enough - free the memory and return failure */
			osk_vfree(kaddr);
			return 0;
		}

		/* Add the end marker */
		memcpy(buffer, &end_marker, sizeof(u64));
	}

	return kaddr;
}

static u8 region_size(u64 pfn_width)
{
	u32 lz;

	lz = osk_clz(pfn_width>>32);
	if (lz == sizeof(u32)*8)
	{
		lz += osk_clz(pfn_width & 0xFFFFFFFFUL);
	}

	/* check if log 2 */
	if (pfn_width == (1ull << (63 - lz)))
		return 2 * (63 - lz + OSK_PAGE_SHIFT);
	else
		return 2 * (63 - lz + 1 + OSK_PAGE_SHIFT);
}

void kbase_mmu_interrupt(kbase_device * kbdev, u32 irq_stat)
{
	const int num_as = 16;
	kbasep_js_device_data *js_devdata;
	const int busfault_shift = 16;
	const int pf_shift = 0;
	const unsigned long mask = (1UL << num_as) - 1;
	
	u64 fault_addr;
	u32 new_mask;
	u32 tmp;

	u32 bf_bits = (irq_stat >> busfault_shift) & mask; /* bus faults */
	/* Ignore ASes with both pf and bf */
	u32 pf_bits = ((irq_stat >> pf_shift) & mask) & ~bf_bits;  /* page faults */

	js_devdata = &kbdev->js_data;

	/* remember current mask */
	osk_spinlock_irq_lock(&kbdev->mmu_mask_change);
	new_mask = kbase_reg_read(kbdev, MMU_REG(MMU_IRQ_MASK), NULL);
	/* mask interrupts for now */
	kbase_reg_write(kbdev, MMU_REG(MMU_IRQ_MASK), 0, NULL);
	osk_spinlock_irq_unlock(&kbdev->mmu_mask_change);

	while (bf_bits)
	{
		int as_no;
		u32 reg;
		kbase_context * kctx;

		/* the while logic ensures we have a bit set, no need to check for not-found here */
		as_no = osk_find_first_set_bit(bf_bits);
		/* mark as handled */
		bf_bits &= ~(1UL << as_no);

		/* find faulting address */
		fault_addr = kbase_reg_read(kbdev, MMU_AS_REG(as_no, ASn_FAULTADDRESS_HI), NULL);
		fault_addr <<= 32;
		fault_addr |= kbase_reg_read(kbdev, MMU_AS_REG(as_no, ASn_FAULTADDRESS_LO), NULL);

		/* Keep the runpool lock whilst the obtained context is being worked on */
		osk_spinlock_irq_lock( &js_devdata->runpool_lock );
		kctx = kbasep_js_policy_runpool_lookup_ctx( &js_devdata->policy, as_no );

		if (kctx)
		{
			/* hw counters dumping in progress, signal the other thread that it failed */
			if (kbdev->hwcnt_context == kctx && kbdev->hwcnt_in_progress == MALI_TRUE)
			{
				kbdev->hwcnt_in_progress = MALI_FALSE;
			}

			/* Stop the kctx from submitting more jobs and cause it to be scheduled out/rescheduled */
			kbasep_js_clear_submit_allowed( js_devdata, kctx );
		}

		OSK_PRINT_WARN(OSK_BASE_MMU, "Bus error in AS%d at 0x%016llx\n", as_no, fault_addr);

		/* switch to UNMAPPED mode, will abort all jobs and stop any hw counter dumping */
		/* AS transaction begin */
		osk_spinlock_irq_lock(&kbdev->as[as_no].transaction_lock);
		reg = kbase_reg_read(kbdev, MMU_AS_REG(as_no, ASn_TRANSTAB_LO), kctx);
		reg &= ~3;
		kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_TRANSTAB_LO), reg, kctx);
		kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_COMMAND), 1, kctx);

		kbase_reg_write(kbdev, MMU_REG(MMU_IRQ_CLEAR), (1UL << as_no) | (1UL << (as_no + num_as))  , NULL);
		osk_spinlock_irq_unlock(&kbdev->as[as_no].transaction_lock);
		/* AS transaction end */

		osk_spinlock_irq_unlock( &js_devdata->runpool_lock );

	}

	/*
	 * pf_bits is non-zero if we have at least one AS with a page fault and no bus fault.
	 * Handle the PFs in our worker thread.
	 */
	while (pf_bits)
	{
		kbase_as * as;
		/* the while logic ensures we have a bit set, no need to check for not-found here */
		int as_no = osk_find_first_set_bit(pf_bits);
		kbase_context * kctx;

		/* mark as handled */
		pf_bits &= ~(1UL << as_no);

		/* find faulting address */
		fault_addr = kbase_reg_read(kbdev, MMU_AS_REG(as_no, ASn_FAULTADDRESS_HI), NULL);
		fault_addr <<= 32;
		fault_addr |= kbase_reg_read(kbdev, MMU_AS_REG(as_no, ASn_FAULTADDRESS_LO), NULL);

		as = &kbdev->as[as_no];
		/* Run pool lock held to prevent the context being scheduled out
		 * as->transaction_lock held so we can update the pf_busy_count, to stop the ctx being
		 * scheduled out whilst we're waiting for the fault worker to run, and to protect the UPDATE+IRQ_CLEAR transaction */
		osk_spinlock_irq_lock( &js_devdata->runpool_lock );
		/* AS transaction begin */
		osk_spinlock_irq_lock(&as->transaction_lock);

		kctx = kbasep_js_policy_runpool_lookup_ctx( &js_devdata->policy, as_no );

		/* if the AS has no context we terminate the work here */
		if (!kctx)
		{
			u32 reg;
			reg = kbase_reg_read(kbdev, MMU_AS_REG(as_no, ASn_TRANSTAB_LO), NULL);
			reg &= ~3;
			kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_TRANSTAB_LO), reg, NULL);
			kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_COMMAND), 1/*UPDATE*/, NULL);
			kbase_reg_write(kbdev, MMU_REG(MMU_IRQ_CLEAR), (1UL << as_no), NULL);
			osk_spinlock_irq_unlock(&as->transaction_lock);
			/* AS transaction end */

			osk_spinlock_irq_unlock( &js_devdata->runpool_lock );
			continue; /* done, on to the next AS */
		}
		/* Prevent ctx from being scheduled out whilst we're handling a page fault on it */
		++(as->pf_busy_count);

		/* remove the queued PFs from the mask */
		new_mask &= ~((1UL << as_no) | (1UL << (as_no + num_as)));

		/* queue work pending for this AS */
		as->fault_addr = fault_addr;

		osk_spinlock_irq_unlock( &as->transaction_lock );
		/* AS transaction end */
		osk_spinlock_irq_unlock( &js_devdata->runpool_lock );

		osk_workq_submit(&as->pf_wq, fault_worker, &as->work);
	}

	/* reenable interrupts */
	osk_spinlock_irq_lock(&kbdev->mmu_mask_change);
	tmp = kbase_reg_read(kbdev, MMU_REG(MMU_IRQ_MASK), NULL);
	new_mask |= tmp;
	kbase_reg_write(kbdev, MMU_REG(MMU_IRQ_MASK), new_mask, NULL);
	osk_spinlock_irq_unlock(&kbdev->mmu_mask_change);
}

static void kbase_mmu_report_fault_and_kill(kbase_context *kctx, kbase_as * as, mali_addr64 fault_addr)
{
	u32 fault_status;
	int exception_type;
	int access_type;
	int source_id;
	int as_no;
	kbase_device * kbdev;
	kbasep_js_device_data *js_devdata;

	OSK_ASSERT(as);
	OSK_ASSERT(kctx);

	as_no = as->number;
	kbdev = kctx->kbdev;
	js_devdata = &kbdev->js_data;

	/* Lock the context in the RunPool whilst we're working on it.
	 * This is needed so we can have it scheduled out after killing it. */
	osk_spinlock_irq_lock( &js_devdata->runpool_lock );

	fault_status = kbase_reg_read(kbdev, MMU_AS_REG(as_no, ASn_FAULTSTATUS), kctx);

	/* decode the fault status */
	exception_type = fault_status       & 0xFF;
	access_type =   (fault_status >> 8) & 0x3;
	source_id =     (fault_status >> 16);

	/* terminal fault, print info about the fault */
	OSK_PRINT_WARN(OSK_BASE_MMU, "Fault in AS%d at VA 0x%016llX\n", as_no, fault_addr);
	OSK_PRINT_WARN(OSK_BASE_MMU, "raw fault status 0x%X\n", fault_status);
	OSK_PRINT_WARN(OSK_BASE_MMU, "decoded fault status:\n");
	OSK_PRINT_WARN(OSK_BASE_MMU, "exception type 0x%X\n", exception_type);
	OSK_PRINT_WARN(OSK_BASE_MMU, "access type 0x%X\n", access_type);
	OSK_PRINT_WARN(OSK_BASE_MMU, "source id 0x%X\n", source_id);

	/* hardware counters dump fault handling */
	if (kbdev->hwcnt_context && kbdev->hwcnt_context->as_nr == as_no && kbdev->hwcnt_in_progress == MALI_TRUE)
	{
#if 0
		PRLAM-5311 blocks any address checking
		u32 num_core_groups = 1; /* See MIDBASE-701 */
		if ((fault_addr >= kbdev->hwcnt_addr) && (fault_addr < (kbdev->hwcnt_addr + (num_core_groups * 2048))))
		{
#endif
			kbdev->hwcnt_in_progress = MALI_FALSE;
#if 0
		}
#endif
	}

	/* Stop the kctx from submitting more jobs and cause it to be scheduled
	 * out/rescheduled - but only once the MMU IRQ has been handled (to stop
	 * the AS being reused in the mean time) */
	kbasep_js_clear_submit_allowed( js_devdata, kctx );
	
	/* Kill any running jobs from the context */
	kbase_job_kill_jobs_from_context(kctx);

	/* AS transaction begin */
	osk_spinlock_irq_lock(&as->transaction_lock);

#ifdef BASE_HW_ISSUE_6035
	kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_TRANSTAB_LO), kbdev->mmu_fault_pages[0], kctx);
	kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_TRANSTAB_HI), 0, kctx);

	kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_COMMAND), 1, kctx);
#else
	{
		u32 reg;
		/* switch to UNMAPPED mode, will abort all jobs and stop any hw counter dumping */
		reg = kbase_reg_read(kbdev, MMU_AS_REG(as_no, ASn_TRANSTAB_LO), kctx);
		reg &= ~3;
		kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_TRANSTAB_LO), reg, kctx);
		kbase_reg_write(kbdev, MMU_AS_REG(as_no, ASn_COMMAND), 1, kctx);
	}
#endif

	kbase_reg_write(kbdev, MMU_REG(MMU_IRQ_CLEAR), (1UL << as_no), NULL);

	osk_spinlock_irq_unlock(&as->transaction_lock);
	/* AS transaction end */

	mmu_mask_reenable(kbdev, kctx, as);
	osk_spinlock_irq_unlock( &js_devdata->runpool_lock );
}

