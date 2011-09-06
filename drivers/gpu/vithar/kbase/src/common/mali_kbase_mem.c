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
 * @file mali_kbase_mem.c
 * Base kernel memory APIs
 */

#include <osk/mali_osk.h>
#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_mem.h>
#include <kbase/src/common/mali_midg_regmap.h>

/**
 * @brief Check the zone compatibility of two regions.
 */
static int kbase_match_zone(struct kbase_va_region *reg1, struct kbase_va_region *reg2)
{
	return ((reg1->flags & KBASE_REG_ZONE_MASK) == (reg2->flags & KBASE_REG_ZONE_MASK));
}

/**
 * @brief Allocate a free region object.
 *
 * The allocated object is not part of any list yet, and is flagged as
 * KBASE_REG_FREE. No mapping is allocated yet.
 */
struct kbase_va_region *kbase_alloc_free_region(struct kbase_context *kctx, u64 start_pfn, u32 nr_pages, u32 zone)
{
	struct kbase_va_region *new_reg = osk_calloc(sizeof(*new_reg));

	if (!new_reg) {
		OSK_PRINT_WARN(OSK_BASE_MEM,
			       "calloc failed at %d in kbase_alloc_free_region",
			       __LINE__);
		return NULL;
	}

	new_reg->kctx = kctx;
	new_reg->flags = zone | KBASE_REG_FREE;
	new_reg->start_pfn = start_pfn;
	new_reg->nr_pages = nr_pages;
	OSK_DLIST_INIT(&new_reg->map_list);

	return new_reg;
}

/**
 * @brief Free a region object.
 *
 * The described region must be freed of any mapping.
 *
 * If the region is not flagged as KBASE_REG_FREE, the destructor
 * kbase_free_phy_pages() will be called.
 */
void kbase_free_alloced_region(struct kbase_va_region *reg)
{
	OSK_ASSERT(OSK_DLIST_IS_EMPTY(&reg->map_list));
	if (!(reg->flags & KBASE_REG_FREE))
	{
		kbase_free_phy_pages(reg, reg->nr_alloc_pages);
		reg->flags |= KBASE_REG_FREE;
	}
	osk_free(reg);
}

/**
 * @brief Insert a region object in the global list.
 *
 * The region new_reg is inserted at start_pfn by replacing at_reg
 * partially or completely. at_reg must be a KBASE_REG_FREE region
 * that contains start_pfn and at least nr_pages from start_pfn.  It
 * must be called with the context region lock held. Internal use
 * only.
 */
static mali_error kbase_insert_va_region(struct kbase_context *kctx,
					 struct kbase_va_region *new_reg,
					 struct kbase_va_region *at_reg,
					 u64 start_pfn, u32 nr_pages)
{
	struct kbase_va_region *new_front_reg;
	mali_error err = MALI_ERROR_NONE;

	new_reg->start_pfn = start_pfn;
	new_reg->nr_pages = nr_pages;

	/* Trivial replacement case */
	if (at_reg->start_pfn == start_pfn && at_reg->nr_pages == nr_pages)
	{
		OSK_DLIST_INSERT_BEFORE(&kctx->reg_list, new_reg, at_reg, struct kbase_va_region, link);
		OSK_DLIST_REMOVE(&kctx->reg_list, at_reg, link);
		kbase_free_alloced_region(at_reg);
		goto out;
	}

	/* Begin case */
	if (at_reg->start_pfn == start_pfn)
	{
		at_reg->start_pfn += nr_pages;
		at_reg->nr_pages -= nr_pages;

		OSK_DLIST_INSERT_BEFORE(&kctx->reg_list, new_reg, at_reg, struct kbase_va_region, link);
		goto out;
	}

	/* End case */
	if ((at_reg->start_pfn + at_reg->nr_pages) == (start_pfn + nr_pages))
	{
		at_reg->nr_pages -= nr_pages;

		OSK_DLIST_INSERT_AFTER(&kctx->reg_list, new_reg, at_reg, struct kbase_va_region, link);
		goto out;
	}

	/* Middle of the road... */
	new_front_reg = kbase_alloc_free_region(kctx, at_reg->start_pfn,
						start_pfn - at_reg->start_pfn,
						at_reg->flags & KBASE_REG_ZONE_MASK);
	if (!new_front_reg)
	{
		err = MALI_ERROR_OUT_OF_MEMORY;
		goto out;
	}

	at_reg->nr_pages -= nr_pages + new_front_reg->nr_pages;
	at_reg->start_pfn = start_pfn + nr_pages;

	OSK_DLIST_INSERT_BEFORE(&kctx->reg_list, new_front_reg, at_reg, struct kbase_va_region, link);
	OSK_DLIST_INSERT_BEFORE(&kctx->reg_list, new_reg, at_reg, struct kbase_va_region, link);

out:
	return err;
}

/**
 * @brief Remove a region object from the global list.
 *
 * The region reg is removed, possibly by merging with other free and
 * compatible adjacent regions.  It must be called with the context
 * region lock held. The associated memory is not realeased (see
 * kbase_free_alloced_region). Internal use only.
 */
static mali_error kbase_free_va_region(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	struct kbase_va_region *prev;
	struct kbase_va_region *next;
	int merged_front = 0;
	int merged_back = 0;
	mali_error err = MALI_ERROR_NONE;

	prev = OSK_DLIST_PREV(reg, struct kbase_va_region, link);
	if (!OSK_DLIST_IS_VALID(prev, link))
		prev = NULL;
	
	next = OSK_DLIST_NEXT(reg, struct kbase_va_region, link);
	if (!OSK_DLIST_IS_VALID(next, link))
		next = NULL;

	/* Try to merge with front first */
	if (prev && (prev->flags & KBASE_REG_FREE) && kbase_match_zone(prev, reg))
	{
		/* We're compatible with the previous VMA, merge with it */
		OSK_DLIST_REMOVE(&kctx->reg_list, reg, link);
		prev->nr_pages += reg->nr_pages;
		reg = prev;
		merged_front = 1;
	}

	/* Try to merge with back next */
	if (next && (next->flags & KBASE_REG_FREE) && kbase_match_zone(next, reg))
	{
		/* We're compatible with the next VMA, merge with it */
		next->start_pfn = reg->start_pfn;
		next->nr_pages += reg->nr_pages;
		OSK_DLIST_REMOVE(&kctx->reg_list, reg, link);

		if (merged_front)
		{
			/* we already merged with prev, free it */
			kbase_free_alloced_region(prev);
		}

		merged_back = 1;
	}

	if (!(merged_front || merged_back))
	{
		/*
		 * We didn't merge anything. Add a new free
		 * placeholder and remove the original one.
		 */
		struct kbase_va_region *free_reg;

		free_reg = kbase_alloc_free_region(kctx, reg->start_pfn, reg->nr_pages, reg->flags & KBASE_REG_ZONE_MASK);
		if (!free_reg)
		{
			err = MALI_ERROR_OUT_OF_MEMORY;
			goto out;
		}

		OSK_DLIST_INSERT_BEFORE(&kctx->reg_list, free_reg, reg, struct kbase_va_region, link);
		OSK_DLIST_REMOVE(&kctx->reg_list, reg, link);
	}

out:
	return err;
}

/**
 * @brief Add a region to the global list.
 *
 * Add reg to the global list, according to its zone. If addr is
 * non-null, this address is used directly (as in the PMEM
 * case). Alignment can be enforced by specifying a number of pages
 * (which *must* but power of 2).
 *
 * Context region list lock must be held.
 *
 * Mostly used by kbase_gpu_mmap(), but also useful to register the
 * ring-buffer region.
 */
mali_error kbase_add_va_region(struct kbase_context *kctx,
			       struct kbase_va_region *reg,
			       mali_addr64 addr, u32 nr_pages,
			       u32 align)
{
	struct kbase_va_region *tmp;
	u64 gpu_pfn = addr >> OSK_PAGE_SHIFT;
	mali_error err = MALI_ERROR_NONE;

	if (!align)
		align = 1;

	if (gpu_pfn)
	{
		OSK_ASSERT(!(gpu_pfn & (align - 1)));

		/*
		 * So we want a specific address. Parse the list until
		 * we find the enclosing region, which *must* be free.
		 */
		OSK_DLIST_FOREACH(&kctx->reg_list, struct kbase_va_region, link, tmp)
		{
			if (tmp->start_pfn <= gpu_pfn &&
			    (tmp->start_pfn + tmp->nr_pages) >= (gpu_pfn + nr_pages))
			{
				/* We have the candidate */
				if (!kbase_match_zone(tmp, reg))
				{
					/* Wrong zone, fail */
					err = MALI_ERROR_OUT_OF_GPU_MEMORY;
					OSK_PRINT_WARN(OSK_BASE_MEM, "Zone mismatch: %d != %d", tmp->flags & KBASE_REG_ZONE_MASK, reg->flags & KBASE_REG_ZONE_MASK);
					goto out;
				}

				if (!(tmp->flags & KBASE_REG_FREE))
				{
					OSK_PRINT_WARN(OSK_BASE_MEM, "!(tmp->flags & KBASE_REG_FREE): tmp->start_pfn=0x%llx tmp->flags=0x%x tmp->nr_pages=0x%x gpu_pfn=0x%llx nr_pages=0x%x\n",
							tmp->start_pfn, tmp->flags, tmp->nr_pages, gpu_pfn, nr_pages);
					OSK_PRINT_WARN(OSK_BASE_MEM, "in function %s (%p, %p, 0x%llx, 0x%x, 0x%x)\n", __func__,
							kctx,reg,addr, nr_pages, align);
					/* Busy, fail */
					err = MALI_ERROR_OUT_OF_GPU_MEMORY;
					goto out;
				}

				err = kbase_insert_va_region(kctx, reg, tmp, gpu_pfn, nr_pages);
				if (err) OSK_PRINT_WARN(OSK_BASE_MEM, "Failed to insert va region");
				goto out;
			}
		}

		err = MALI_ERROR_OUT_OF_GPU_MEMORY;
		OSK_PRINT_WARN(OSK_BASE_MEM, "Out of mem");
		goto out;
	}

	/* Find the first free region that accomodates our requirements */
	OSK_DLIST_FOREACH(&kctx->reg_list, struct kbase_va_region, link, tmp)
	{
		if (tmp->nr_pages >= nr_pages &&
		    (tmp->flags & KBASE_REG_FREE) &&
		    kbase_match_zone(tmp, reg))
		{
			/* Check alignement */
			u64 start_pfn;

			start_pfn = (tmp->start_pfn + align) & ~(align - 1);
			if ((tmp->start_pfn + tmp->nr_pages - start_pfn) < nr_pages)
				continue;

			/* It fits, let's use it */
			
			err = kbase_insert_va_region(kctx, reg, tmp, start_pfn, nr_pages);
			
			goto out;
		}
	}

	err = MALI_ERROR_OUT_OF_GPU_MEMORY;
out:
	return err;
}

void kbase_mmu_update(struct kbase_context *kctx)
{
	/* ASSERT that the context has a valid as_nr, which is only the case
	 * when it's scheduled in. This may require you hold the jsctx_lock and
	 * runpool_lock if the context is in danger of being scheduled out. */
	OSK_ASSERT( kctx->as_nr >= 0 );

	/* lock the AS for a transaction */
	osk_spinlock_irq_lock(&kctx->kbdev->as[kctx->as_nr].transaction_lock);

	kbase_reg_write(kctx->kbdev,
					MMU_AS_REG(kctx->as_nr, ASn_TRANSTAB_LO),
					(kctx->pgd & 0xfffff000) | (1ul << 2) | 3, kctx);
	kbase_reg_write(kctx->kbdev,
					MMU_AS_REG(kctx->as_nr, ASn_TRANSTAB_HI),
					kctx->pgd >> 32, kctx);
	kbase_reg_write(kctx->kbdev,
					MMU_AS_REG(kctx->as_nr, ASn_MEMATTR_LO),
					0x48484848, kctx);
	kbase_reg_write(kctx->kbdev,
					MMU_AS_REG(kctx->as_nr, ASn_MEMATTR_HI),
					0x48484848, kctx);
	kbase_reg_write(kctx->kbdev,
					MMU_AS_REG(kctx->as_nr, ASn_COMMAND),
					1, kctx);

	osk_spinlock_irq_unlock(&kctx->kbdev->as[kctx->as_nr].transaction_lock);
}

/* the AS transaction lock must be held by the caller */
void kbase_mmu_disable (kbase_context *kctx)
{
	/* ASSERT that the context has a valid as_nr, which is only the case
	 * when it's scheduled in. This may require you hold the jsctx_lock and
	 * runpool_lock if the context is in danger of being scheduled out. */
	OSK_ASSERT( kctx->as_nr >= 0 );

	kbase_reg_write(kctx->kbdev,
			MMU_AS_REG(kctx->as_nr, ASn_TRANSTAB_LO),
			0, kctx);
	kbase_reg_write(kctx->kbdev,
			MMU_AS_REG(kctx->as_nr, ASn_TRANSTAB_HI),
			0, kctx);
	kbase_reg_write(kctx->kbdev,
			MMU_AS_REG(kctx->as_nr, ASn_COMMAND),
			1, kctx);
}

/**
 * @brief Register region and map it on the GPU.
 *
 * Call kbase_add_va_region() and map the region on the GPU.
 */
mali_error kbase_gpu_mmap(struct kbase_context *kctx,
			  struct kbase_va_region *reg,
			  mali_addr64 addr, u32 nr_pages,
			  u32 align)
{
	mali_error err;

	if ((err = kbase_add_va_region(kctx, reg, addr, nr_pages, align)))
		goto out;

	kbase_mmu_insert_pages(kctx, reg->start_pfn,
			       kbase_get_phy_pages(reg),
			       reg->nr_alloc_pages, reg->flags);
out:
	return err;
}

/**
 * @brief Remove the region from the GPU and unregister it.
 *
 * Not exported for the time being. Must be called with context lock
 * held.
 */
static mali_error kbase_gpu_munmap(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	mali_error err;

	kbase_mmu_teardown_pages(kctx, reg->start_pfn, reg->nr_alloc_pages);
	err = kbase_free_va_region(kctx, reg);

	return err;
}

kbase_va_region *kbase_region_lookup(kbase_context *kctx, mali_addr64 gpu_addr)
{
	kbase_va_region *tmp;
	u64 gpu_pfn = gpu_addr >> OSK_PAGE_SHIFT;

	OSK_DLIST_FOREACH(&kctx->reg_list, kbase_va_region, link, tmp)
	{
		if (gpu_pfn >= tmp->start_pfn && (gpu_pfn < tmp->start_pfn + tmp->nr_pages))
			return tmp;
	}

	return NULL;
}


/**
 * @brief Check that a pointer is actually a valid region.
 *
 * Not exported for the time being. Must be called with context lock
 * held.
 */
static struct kbase_va_region *kbase_validate_region(struct kbase_context *kctx, mali_addr64 gpu_addr)
{
	struct kbase_va_region *tmp;
	u64 gpu_pfn = gpu_addr >> OSK_PAGE_SHIFT;

	OSK_DLIST_FOREACH(&kctx->reg_list, struct kbase_va_region, link, tmp)
	{
		if (tmp->start_pfn == gpu_pfn)
			return tmp;
	}

	return NULL;
}

/**
 * @brief Find a mapping keyed with ptr in region reg
 */
static struct kbase_cpu_mapping *kbase_find_cpu_mapping(struct kbase_va_region *reg,
							const void *ptr)
{
	struct kbase_cpu_mapping *map;

	OSK_DLIST_FOREACH(&reg->map_list, struct kbase_cpu_mapping, link, map)
	{
		if (map->private == ptr)
			return map;
	}

	return NULL;
}

static struct kbase_cpu_mapping *kbasep_find_enclosing_cpu_mapping_of_region(
                                      const struct kbase_va_region *reg,
                                      osk_virt_addr                 uaddr,
                                      size_t                        size)
{
	struct kbase_cpu_mapping *map;

	if ((uintptr_t)uaddr + size < (uintptr_t)uaddr) /* overflow check */
	{
		return NULL;
	}

	OSK_DLIST_FOREACH(&reg->map_list, struct kbase_cpu_mapping, link, map)
	{
		if (map->uaddr <= uaddr &&
		    ((uintptr_t)map->uaddr + (map->nr_pages << OSK_PAGE_SHIFT)) >= ((uintptr_t)uaddr + size))
			return map;
	}

	return NULL;
}

static void kbase_dump_mappings(struct kbase_va_region *reg)
{
	struct kbase_cpu_mapping *map;

	OSK_DLIST_FOREACH(&reg->map_list, struct kbase_cpu_mapping, link, map)
	{
		OSK_PRINT_WARN(OSK_BASE_MEM, "uaddr %p nr_pages %d page_off %016llx vma %p\n",
		       map->uaddr, map->nr_pages,
		       map->page_off, map->private);
	}
}

/**
 * @brief Delete a mapping keyed with ptr in region reg
 */
mali_error kbase_cpu_free_mapping(struct kbase_va_region *reg, const void *ptr)
{
	struct kbase_cpu_mapping *map;
	mali_error err = MALI_ERROR_NONE;

	map = kbase_find_cpu_mapping(reg, ptr);
	if (!map)
	{
		OSK_PRINT_WARN(OSK_BASE_MEM, "Freeing unknown mapping %p in region %p\n", ptr, (void*)reg);
		kbase_dump_mappings(reg);
		err = MALI_ERROR_FUNCTION_FAILED;
		goto out;
	}

	OSK_DLIST_REMOVE(&reg->map_list, map, link);
	osk_free(map);
out:
	return err;
}

struct kbase_cpu_mapping *kbasep_find_enclosing_cpu_mapping(
                               struct kbase_context *kctx,
                               mali_addr64           gpu_addr,
                               osk_virt_addr         uaddr,
                               size_t                size )
{
	struct kbase_cpu_mapping     *map = NULL;
	const struct kbase_va_region *reg;

	OSKP_ASSERT( kctx != NULL );
	kbase_gpu_vm_lock(kctx);

	reg = kbase_validate_region( kctx, gpu_addr );
	if ( NULL != reg )
	{
		map = kbasep_find_enclosing_cpu_mapping_of_region( reg, uaddr, size);
	}

	kbase_gpu_vm_unlock(kctx);

	return map;
}

static void kbase_do_syncset(struct kbase_context *kctx, base_syncset *set,
			     osk_sync_kmem_fn sync_fn)
{
	struct basep_syncset *sset = &set->basep_sset;
	struct kbase_va_region *reg;
	struct kbase_cpu_mapping *map;
	osk_phy_addr *pa;
	u64 page_off, page_count, size_in_pages;
	osk_virt_addr start;
	size_t size;
	int i;
	u32 offset_within_page;

	kbase_gpu_vm_lock(kctx);

	reg = kbase_validate_region(kctx, sset->mem_handle);
	if (!reg)
		goto out_unlock;

	if (!(reg->flags & KBASE_REG_CPU_CACHED))
	{
		goto out_unlock;
	}

	start = (osk_virt_addr)(uintptr_t)sset->user_addr;
	size = sset->size;

	map = kbasep_find_enclosing_cpu_mapping_of_region(reg, start, size);
	if (!map)
		goto out_unlock;

	offset_within_page = (uintptr_t)start & (OSK_PAGE_SIZE - 1);
	size_in_pages = (size + offset_within_page + (OSK_PAGE_SIZE - 1)) & OSK_PAGE_MASK;
	page_off = map->page_off + (((uintptr_t)start - (uintptr_t)map->uaddr) >> OSK_PAGE_SHIFT);
	page_count = (size_in_pages >> OSK_PAGE_SHIFT);
	pa = kbase_get_phy_pages(reg);

	for (i = 0; i < page_count; i++)
	{
		u32 offset = (uintptr_t)start & (OSK_PAGE_SIZE - 1);
		osk_phy_addr paddr = pa[page_off + i] + offset;
		size_t sz = OSK_MIN(((size_t)OSK_PAGE_SIZE - offset), size);

		sync_fn(paddr, start, sz);

		start = (void*)((uintptr_t)start + sz);
		size -= sz;
	}

	OSK_ASSERT(size == 0);

out_unlock:
	kbase_gpu_vm_unlock(kctx);
}

static void kbase_sync_to_memory(kbase_context *kctx, base_syncset *syncset)
{
	kbase_do_syncset(kctx, syncset, osk_sync_to_memory);
}

static void kbase_sync_to_cpu(kbase_context *kctx, base_syncset *syncset)
{
	kbase_do_syncset(kctx, syncset, osk_sync_to_cpu);
}

void kbase_sync_now(kbase_context *kctx, base_syncset *syncset)
{
	struct basep_syncset *sset = &syncset->basep_sset;
	switch(sset->type)
	{
	case BASE_SYNCSET_OP_MSYNC:
		kbase_sync_to_memory(kctx, syncset);
		break;
		
	case BASE_SYNCSET_OP_CSYNC:
		kbase_sync_to_cpu(kctx, syncset);
		break;

	default:
		OSK_PRINT_WARN(OSK_BASE_MEM, "Unknown msync op %d\n", sset->type);
		break;
	}
}

void kbase_pre_job_sync(kbase_context *kctx, base_syncset *syncsets, u32 nr)
{
	int i;

	for (i = 0; i < nr; i++) {
		struct basep_syncset *sset = &syncsets[i].basep_sset;
		switch(sset->type) {
		case BASE_SYNCSET_OP_MSYNC:
			kbase_sync_to_memory(kctx, &syncsets[i]);
			break;

		case BASE_SYNCSET_OP_CSYNC:
			continue;

		default:
			OSK_PRINT_WARN(OSK_BASE_MEM, "Unknown msync op %d\n", sset->type);
			break;
		}
	}
}

void kbase_post_job_sync(kbase_context *kctx, base_syncset *syncsets, u32 nr)
{
	int i;

	for (i = 0; i < nr; i++) {
		struct basep_syncset *sset = &syncsets[i].basep_sset;
		switch(sset->type) {
		case BASE_SYNCSET_OP_CSYNC:
			kbase_sync_to_cpu(kctx, &syncsets[i]);
			break;

		case BASE_SYNCSET_OP_MSYNC:
			continue;

		default:
			OSK_PRINT_WARN(OSK_BASE_MEM, "Unknown msync op %d\n", sset->type);
			break;
		}
	}
}

/* vm lock must be held */
mali_error kbase_mem_free_region(struct kbase_context *kctx,
				 kbase_va_region *reg)
{
	mali_error err;

	if (!OSK_DLIST_IS_EMPTY(&reg->map_list))
	{
		/*
		 * We still have mappings, can't free
		 * memory. This also handles the race
		 * condition with the unmap code (see
		 * kbase_cpu_vm_close()).
		 */
		OSK_PRINT_WARN(OSK_BASE_MEM, "Pending CPU mappings, not freeing memory!\n");
		err = MALI_ERROR_FUNCTION_FAILED;
		goto out;
	}

	err = kbase_gpu_munmap(kctx, reg);
	if (err)
	{
		OSK_PRINT_WARN(OSK_BASE_MEM, "Could not unmap from the GPU...\n");
		goto out;
	}

	/* This will also free the physical pages */
	kbase_free_alloced_region(reg);

out:
	return err;
}

/**
 * @brief Free the region from the GPU and unregister it.
 *
 * This function implements the free operation on a memory segment.
 * It will loudly fail if called with outstanding mappings.
 */
mali_error kbase_mem_free(struct kbase_context *kctx, mali_addr64 gpu_addr)
{
	mali_error err;
	struct kbase_va_region *reg;
	
	if (0 == gpu_addr)
	{
		OSK_PRINT_WARN(OSK_BASE_MEM, "gpu_addr 0 is reserved for the ringbuffer and it's an error to try to free it using kbase_mem_free\n");
		return MALI_ERROR_FUNCTION_FAILED;
	}

	kbase_gpu_vm_lock(kctx);

	if (gpu_addr < OSK_PAGE_SIZE)
	{
		/* an OS specific cookie, ask the OS specific code to validate it */
		reg = kbase_lookup_cookie(kctx, gpu_addr);
		if (!reg)
		{
			err = MALI_ERROR_FUNCTION_FAILED;
			goto out_unlock;
		}

		/* ask to unlink the cookie as we'll free it */
		kbase_unlink_cookie(kctx, gpu_addr, reg);
	}
	else
	{
		/* A real GPU va */

		/* Validate the region */
		reg = kbase_validate_region(kctx, gpu_addr);
		if (!reg)
		{
			err = MALI_ERROR_FUNCTION_FAILED;
			goto out_unlock;
		}
	}

	err = kbase_mem_free_region(kctx, reg);

out_unlock:
	kbase_gpu_vm_unlock(kctx);
	return err;
}

#define KBASE_CPU_CACHED_HINT	(BASE_MEM_HINT_CPU_RD | BASE_MEM_HINT_CPU_WR)
#define KBASE_GPU_CACHED_HINT	(BASE_MEM_HINT_GPU_RD | BASE_MEM_HINT_GPU_WR)

void kbase_update_region_flags(struct kbase_va_region *reg, u32 flags)
{
	if ((flags & KBASE_CPU_CACHED_HINT) == KBASE_CPU_CACHED_HINT)
		reg->flags |= KBASE_REG_CPU_CACHED;

	if ((flags & KBASE_GPU_CACHED_HINT) == KBASE_GPU_CACHED_HINT)
		reg->flags |= KBASE_REG_GPU_CACHED;

	if (flags & BASE_MEM_GROW_ON_GPF)
		reg->flags |= KBASE_REG_PF_GROW;

	if (flags & BASE_MEM_PROT_CPU_WR)
		reg->flags |= KBASE_REG_CPU_RW;

	if (flags & BASE_MEM_PROT_GPU_WR)
		reg->flags |= KBASE_REG_GPU_RW;

	if (0 == (flags & BASE_MEM_PROT_GPU_EX))
		reg->flags |= KBASE_REG_GPU_NX;
}

void kbase_free_phy_pages(struct kbase_va_region *reg, u32 vsize)
{
	osk_phy_addr *page_array = kbase_get_phy_pages(reg);

#if MALI_KBASE_USE_UMP
	if (reg->flags & KBASE_REG_IS_UMP)
	{
		ump_dd_handle umph;
		umph = (ump_dd_handle)reg->ump_handle;
		ump_dd_unpin_size(umph);
		ump_dd_release(umph);
	}
	else
	{
#endif /* MALI_KBASE_USE_UMP */
		if (reg->flags & KBASE_REG_IS_TB)
		{
			/* trace buffer being freed. Disconnect, then use osk_vfree */
			/* save tb so we can free it after the disconnect call */
			void * tb;
			tb = reg->kctx->jctx.tb;
			kbase_device_trace_buffer_uninstall(reg->kctx);
			osk_vfree(tb);
		}
		else
		{
			osk_phy_pages_free(&reg->phy_allocator, vsize, page_array);
		}
#if MALI_KBASE_USE_UMP
	}
#endif /* MALI_KBASE_USE_UMP */

	kbase_set_phy_pages(reg, NULL);
	osk_free(page_array);
	osk_phy_allocator_term(&reg->phy_allocator);
}

int kbase_alloc_phy_pages(struct kbase_va_region *reg, u32 vsize, u32 size)
{
	osk_phy_addr *page_array;

	if (OSK_ERR_NONE != osk_phy_allocator_init(&reg->phy_allocator, 0, vsize))
		goto out;

	page_array = osk_calloc(vsize * sizeof(*page_array));
	if (!page_array)
		goto out_term;

	if (OSK_ERR_NONE != osk_phy_pages_alloc(&reg->phy_allocator, size, page_array))
		goto out_free;

	kbase_set_phy_pages(reg, page_array);

	return 0;

out_free:
	osk_free(page_array);
out_term:
	osk_phy_allocator_term(&reg->phy_allocator);
out:
	return -1;
}

struct kbase_va_region *kbase_tmem_alloc(struct kbase_context *kctx,
					 u32 vsize, u32 psize,
					 u32 extent, u32 flags)
{
	struct kbase_va_region *reg;
	mali_error err;
	u32 align = 1;

	if (flags & BASE_MEM_PROT_GPU_EX)
	{
		if (kbase_device_has_feature(kctx->kbdev, KBASE_FEATURE_HAS_16BIT_PC))
		{
			/* A 16-bit PC gives us a 64kB max size and a 64kB alignment req */
			align = (1ul << (16 - OSK_PAGE_SHIFT));
		}
		else
		{
			/* A 32-bit PC is the default, giving a 4GB/4GB max size/alignment */
			align = (1ul << (32 - OSK_PAGE_SHIFT));
		}

		if (vsize > align)
		{
			OSK_PRINT_WARN(OSK_BASE_MEM, "Executable tmem virtual size %lx is larger than the pc (%lx) (in pages)", (unsigned long)vsize, (unsigned long)align);
			return NULL;
		}
	}

	reg = kbase_alloc_free_region(kctx, 0, vsize, KBASE_REG_ZONE_TMEM);
	if (!reg)
		goto out1;

	reg->flags &= ~KBASE_REG_FREE;

	kbase_update_region_flags(reg, flags);

	if (kbase_alloc_phy_pages(reg, vsize, psize))
		goto out2;

	reg->nr_alloc_pages	= psize;
	reg->extent		= extent;

	kbase_gpu_vm_lock(kctx);
	err = kbase_gpu_mmap(kctx, reg, 0, vsize, align);
	kbase_gpu_vm_unlock(kctx);

	if (err)
	{
		OSK_PRINT_WARN(OSK_BASE_MEM, "kbase_gpu_mmap failed\n");
		goto out3;
	}

	return reg;

out3:
	kbase_free_phy_pages(reg, psize);
out2:
	osk_free(reg);
out1:
	return NULL;
}

mali_error kbase_tmem_resize(struct kbase_context *kctx, mali_addr64 gpu_addr, s32 delta, u32 *size, base_backing_threshold_status * failure_reason)
{
	kbase_va_region *reg;
	mali_error ret = MALI_ERROR_FUNCTION_FAILED;
	u32 desired_size_pages;
	osk_phy_addr *phy_pages;
	
	OSK_ASSERT(size);
	OSK_ASSERT(failure_reason);

	kbase_gpu_vm_lock(kctx);

	/* Validate the region */
	reg = kbase_validate_region(kctx, gpu_addr);
	if (!reg || (reg->flags & KBASE_REG_FREE) || ((KBASE_REG_ZONE_MASK & reg->flags) != KBASE_REG_ZONE_TMEM))
	{
		/* not a valid region, is free memory or not a tmem */
		*failure_reason = BASE_BACKING_THRESHOLD_ERROR_INVALID_ARGUMENTS;
		goto out_unlock;
	}

	if (0 == (reg->flags & KBASE_REG_PF_GROW))
	{
		/* not growable */
		*failure_reason = BASE_BACKING_THRESHOLD_ERROR_NOT_GROWABLE;
		goto out_unlock;
	}

	if (!OSK_DLIST_IS_EMPTY(&reg->map_list))
	{
		/* We still have mappings */
		*failure_reason = BASE_BACKING_THRESHOLD_ERROR_MAPPED;
		goto out_unlock;
	}


	/* Calculate the desired size */
	desired_size_pages = reg->nr_alloc_pages + delta;

	if (desired_size_pages > reg->nr_alloc_pages && delta < 0)
	{
		/* Underflow */
		*failure_reason = BASE_BACKING_THRESHOLD_ERROR_INVALID_ARGUMENTS;
		goto out_unlock;
	}
	if (desired_size_pages > reg->nr_pages)
	{
		/* Would overflow the VA region */
		*failure_reason = BASE_BACKING_THRESHOLD_ERROR_INVALID_ARGUMENTS;
		goto out_unlock;
	}

	phy_pages = kbase_get_phy_pages(reg);

	if (delta > 0)
	{
		/* Allocate some more pages */
		if (OSK_ERR_NONE != osk_phy_pages_alloc(&reg->phy_allocator,
		                                        delta,
		                                        phy_pages + reg->nr_alloc_pages))
		{
			*failure_reason = BASE_BACKING_THRESHOLD_ERROR_OOM;
			goto out_unlock;
		}
		kbase_mmu_insert_pages(kctx, reg->start_pfn+reg->nr_alloc_pages,
		                       phy_pages + reg->nr_alloc_pages,
		                       delta, reg->flags);
		reg->nr_alloc_pages += delta;
	}
	else if (delta < 0)
	{
		/* Free some pages */
		
		/* Get the absolute value of delta. Note that we have to add one before and after the negation to avoid
		 * overflowing when delta is INT_MIN */
		u32 num_pages = (u32)(-(delta+1))+1;

		reg->nr_alloc_pages -= num_pages;
		
		kbase_mmu_teardown_pages(kctx, reg->start_pfn + reg->nr_alloc_pages, num_pages);

		osk_phy_pages_free(&reg->phy_allocator,
		                   num_pages,
		                   phy_pages + reg->nr_alloc_pages);
	}
	
	*size = reg->nr_alloc_pages;

	ret = MALI_ERROR_NONE;

out_unlock:
	kbase_gpu_vm_unlock(kctx);
	return ret;
}

#if MALI_KBASE_USE_UMP

#define KBASE_DEVICE_SHIFT UMP_DEVICE_Z_SHIFT

struct kbase_va_region *kbase_tmem_from_ump(struct kbase_context *kctx, ump_secure_id id, u64 * const pages)
{
	struct kbase_va_region *reg;
	mali_error err;
	ump_dd_handle umph;
	u64 vsize;
	u64 block_count;
	const ump_dd_physical_block * block_array;
	osk_phy_addr *page_array;
	int i;
	int j;
	int page = 0;
	ump_alloc_flags ump_flags;
	ump_alloc_flags cpu_flags;
	ump_alloc_flags gpu_flags;

	OSK_ASSERT(NULL != pages);

	umph = ump_dd_from_secure_id(id);
	if (UMP_DD_INVALID_MEMORY_HANDLE == umph)
	{
		return NULL;
	}

	ump_dd_pin_size(umph);

	ump_flags = ump_dd_allocation_flags_get(umph);
	cpu_flags = (ump_flags >> UMP_DEVICE_CPU_SHIFT) & UMP_DEVICE_MASK;
	gpu_flags = (ump_flags >> KBASE_DEVICE_SHIFT) & UMP_DEVICE_MASK;

	vsize = ump_dd_size_get(umph);
	vsize >>= OSK_PAGE_SHIFT;

	reg = kbase_alloc_free_region(kctx, 0, vsize, KBASE_REG_ZONE_TMEM);
	if (!reg)
		goto out1;

	reg->flags &= ~KBASE_REG_FREE;
	reg->flags |= KBASE_REG_IS_UMP;
	reg->flags |= KBASE_REG_GPU_NX; /* UMP is always No eXecute */

	reg->ump_handle = umph;

	if ((cpu_flags & (UMP_HINT_DEVICE_RD|UMP_HINT_DEVICE_WR)) == (UMP_HINT_DEVICE_RD|UMP_HINT_DEVICE_WR))
		reg->flags |= KBASE_REG_CPU_CACHED;

	if (cpu_flags & UMP_PROT_DEVICE_WR)
		reg->flags |= KBASE_REG_CPU_RW;

	if ((gpu_flags & (UMP_HINT_DEVICE_RD|UMP_HINT_DEVICE_WR)) == (UMP_HINT_DEVICE_RD|UMP_HINT_DEVICE_WR))
		reg->flags |= KBASE_REG_GPU_CACHED;

	if (gpu_flags & UMP_PROT_DEVICE_WR)
		reg->flags |= KBASE_REG_GPU_RW;

	/* ump phys block query */
	ump_dd_phys_blocks_get(umph, &block_count, &block_array);

	page_array = osk_calloc(vsize * sizeof(*page_array));
	if (!page_array)
		goto out2;

	for (i = 0; i < block_count; i++)
	{
		for (j = 0; j < (block_array[i].size >> OSK_PAGE_SHIFT); j++)
		{
			page_array[page] = block_array[i].addr + (j << OSK_PAGE_SHIFT);
			page++;
		}
	}

	kbase_set_phy_pages(reg, page_array);

	reg->nr_alloc_pages	= vsize;
	reg->extent		= vsize;

	kbase_gpu_vm_lock(kctx);
	err = kbase_gpu_mmap(kctx, reg, 0, vsize, 1/* no alignment */);
	kbase_gpu_vm_unlock(kctx);

	if (err)
	{
		OSK_PRINT_WARN(OSK_BASE_MEM, "kbase_gpu_mmap failed\n");
		goto out3;
	}

	*pages = vsize;

	return reg;

out3:
	osk_free(page_array);
out2:
	osk_free(reg);
out1:
	ump_dd_unpin_size(umph);
	ump_dd_release(umph);
	return NULL;


}

#endif /* MALI_KBASE_USE_UMP */

/**
 * @brief Acquire the per-context region list lock
 */
void kbase_gpu_vm_lock(struct kbase_context *kctx)
{
	osk_mutex_lock(&kctx->reg_lock);
}

/**
 * @brief Release the per-context region list lock
 */
void kbase_gpu_vm_unlock(struct kbase_context *kctx)
{
	osk_mutex_unlock(&kctx->reg_lock);
}
