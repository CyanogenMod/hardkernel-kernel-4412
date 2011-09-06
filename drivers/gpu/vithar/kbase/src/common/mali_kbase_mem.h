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
 * @file mali_kbase_mem.h
 * Base kernel memory APIs
 */

#ifndef _KBASE_MEM_H_
#define _KBASE_MEM_H_

#include <malisw/mali_malisw.h>
#include <osk/mali_osk.h>
#if MALI_KBASE_USE_UMP
#include <ump/ump_kernel_interface.h>
#endif /* MALI_KBASE_USE_UMP */
#include <kbase/src/common/mali_kbase.h>
#include <base/mali_base_kernel.h>

/**
 * A CPU mapping
 */
typedef struct kbase_cpu_mapping
{
	osk_dlist_item      link;
	osk_virt_addr       uaddr;
	u32                 nr_pages;
	mali_size64         page_off;
	void                *private; /* Use for VMA */
} kbase_cpu_mapping;

/**
 * A GPU memory region, and attributes for CPU mappings.
 */  
typedef struct kbase_va_region
{
	osk_dlist_item          link;

	struct kbase_context    *kctx; /* Backlink to base context */

	u64                     start_pfn;  /* The PFN in GPU space */
	u32                     nr_pages;   /* VA size */

#define KBASE_REG_FREE       (1ul << 0) /* Free region */
#define KBASE_REG_CPU_RW     (1ul << 1) /* CPU write access */
#define KBASE_REG_GPU_RW     (1ul << 2) /* GPU write access */
#define KBASE_REG_GPU_NX     (1ul << 3) /* No eXectue flag */
#define KBASE_REG_CPU_CACHED (1ul << 4) /* Is CPU cached? */
#define KBASE_REG_GPU_CACHED (1ul << 5) /* Is GPU cached? */
#define KBASE_REG_PF_GROW    (1ul << 6) /* Can grow on pf? */
#define KBASE_REG_IS_RB      (1ul << 7) /* Is ringbuffer? */
#define KBASE_REG_IS_UMP     (1ul << 8) /* Is UMP? */
#define KBASE_REG_IS_MMU_DUMP (1ul << 9) /* Is an MMU dump */
#define KBASE_REG_IS_TB      (1ul << 10) /* Is register trace buffer? */

#define KBASE_REG_ZONE_MASK  (3ul << 11) /* Space for 4 different zones. Only use 3 for now */
#define KBASE_REG_ZONE(x)    (((x) & 3) << 11)

#define KBASE_REG_ZONE_PMEM  KBASE_REG_ZONE(0)

#ifndef KBASE_REG_ZONE_TMEM  /* To become 0 on a 64bit platform */
/*
 * On a 32bit platform, TMEM should be wired from 4GB to the VA limit
 * of the GPU, which is currently hardcoded at 48 bits. Unfortunately,
 * the Linux mmap() interface limits us to 2^32 pages (2^44 bytes, see
 * mmap64 man page for reference).
 */
#define KBASE_REG_ZONE_TMEM         KBASE_REG_ZONE(1)
#define KBASE_REG_ZONE_TMEM_BASE    ((1ULL << 32) >> OSK_PAGE_SHIFT)
#define KBASE_REG_ZONE_TMEM_SIZE    (((1ULL << 44) >> OSK_PAGE_SHIFT) - \
                                    KBASE_REG_ZONE_TMEM_BASE)
#endif

#define KBASE_REG_COOKIE_MASK       (0xFFFF << 16)
#define KBASE_REG_COOKIE(x)         (((x) & 0xFFFF) << 16)
/* Bit mask of cookies that not used for PMEM but reserved for other uses */
#define KBASE_REG_RESERVED_COOKIES  7ULL
/* The reserved cookie values */
#define KBASE_REG_COOKIE_RB         0
#define KBASE_REG_COOKIE_MMU_DUMP   1
#define KBASE_REG_COOKIE_TB         2

	u32                 flags;

	u32                 nr_alloc_pages; /* nr of pages allocated */
	u32                 extent;         /* nr of pages alloc'd on PF */

	osk_phy_allocator   phy_allocator;

	void                *ump_handle;
	osk_phy_addr        *phy_pages;

	osk_dlist           map_list;
} kbase_va_region;

/* Common functions */
static INLINE osk_phy_addr *kbase_get_phy_pages(struct kbase_va_region *reg)
{
	return reg->phy_pages;
}

static INLINE void kbase_set_phy_pages(struct kbase_va_region *reg, osk_phy_addr *phy_pages)
{
	reg->phy_pages = phy_pages;
}

struct kbase_va_region *kbase_alloc_free_region(struct kbase_context *kctx, u64 start_pfn, u32 nr_pages, u32 zone);
void kbase_free_alloced_region(struct kbase_va_region *reg);
mali_error kbase_add_va_region(struct kbase_context *kctx,
                               struct kbase_va_region *reg,
                               mali_addr64 addr, u32 nr_pages,
                               u32 align);
kbase_va_region *kbase_region_lookup(kbase_context *kctx, mali_addr64 gpu_addr);

mali_error kbase_gpu_mmap(struct kbase_context *kctx,
                          struct kbase_va_region *reg,
                          mali_addr64 addr, u32 nr_pages,
                          u32 align);
void kbase_update_region_flags(struct kbase_va_region *reg, u32 flags);

void kbase_gpu_vm_lock(struct kbase_context *kctx);
void kbase_gpu_vm_unlock(struct kbase_context *kctx);

void kbase_free_phy_pages(struct kbase_va_region *reg, u32 vsize);
int kbase_alloc_phy_pages(struct kbase_va_region *reg, u32 vsize, u32 size);

mali_error kbase_cpu_free_mapping(struct kbase_va_region *reg, const void *ptr);

osk_phy_addr kbase_mmu_alloc_pgd(kbase_context *kctx);
void kbase_mmu_free_pgd(struct kbase_context *kctx);
void kbase_mmu_insert_pages(struct kbase_context *kctx, u64 vpfn,
                            osk_phy_addr *phys, u32 nr, u16 flags);
void kbase_mmu_teardown_pages(struct kbase_context *kctx, u64 vpfn, u32 nr);
void kbase_mmu_update(struct kbase_context *kctx);
void kbase_mmu_disable (kbase_context *kctx);

void kbase_mmu_interrupt(kbase_device * kbdev, u32 irq_stat);

/** Dump the MMU tables to a buffer
 *
 * This function allocates a buffer (of @c nr_pages pages) to hold a dump of the MMU tables and fills it. If the 
 * buffer is too small then the return value will be NULL.
 *
 * The GPU vm lock must be held when calling this function.
 *
 * The buffer returned should be freed with @ref osk_vfree when it is no longer required.
 *
 * @param[in]   kctx        The kbase context to dump
 * @param[in]   nr_pages    The number of pages to allocate for the buffer.
 *
 * @return The address of the buffer containing the MMU dump or NULL on error (including if the @c nr_pages is too 
 * small)
 */
void *kbase_mmu_dump(struct kbase_context *kctx,int nr_pages);

void kbase_sync_now(kbase_context *kctx, base_syncset *syncset);
void kbase_pre_job_sync(kbase_context *kctx, base_syncset *syncsets, u32 nr);
void kbase_post_job_sync(kbase_context *kctx, base_syncset *syncsets, u32 nr);

struct kbase_va_region *kbase_tmem_alloc(struct kbase_context *kctx,
                                         u32 vsize, u32 psize,
                                         u32 extent, u32 flags);

/** Resize a tmem region
 *
 * This function changes the number of physical pages committed to a tmem region.
 *
 * @param[in]   kctx        The kbase context which the tmem belongs to
 * @param[in]   gpu_addr    The base address of the tmem region
 * @param[in]   delta       The number of pages to grow or shrink by
 * @param[out]  size        The number of pages of memory committed after growing/shrinking
 *
 * @return MALI_ERROR_NONE on success
 */
mali_error kbase_tmem_resize(struct kbase_context *kctx, mali_addr64 gpu_addr, s32 delta, u32 *size, base_backing_threshold_status * failure_reason);

#if MALI_KBASE_USE_UMP
struct kbase_va_region *kbase_tmem_from_ump(struct kbase_context *kctx, ump_secure_id id, u64 * const pages);
#endif /* MALI_KBASE_USE_UMP */


/* OS specific functions */
struct kbase_va_region * kbase_lookup_cookie(struct kbase_context * kctx, mali_addr64 cookie);
void kbase_unlink_cookie(struct kbase_context * kctx, mali_addr64 cookie, struct kbase_va_region * reg);
mali_error kbase_mem_free(struct kbase_context *kctx, mali_addr64 gpu_addr);
mali_error kbase_mem_free_region(struct kbase_context *kctx,
                                 struct kbase_va_region *reg);

/**
 * @brief Find a CPU mapping of a memory allocation containing a given address range
 *
 * Searches for a CPU mapping of any part of the region starting at @p gpu_addr that
 * fully encloses the CPU virtual address range specified by @p uaddr and @p size.
 * Returns a failure indication if only part of the address range lies within a
 * CPU mapping, or the address range lies within a CPU mapping of a different region.
 *
 * @param[in,out] kctx      The kernel base context used for the allocation.
 * @param[in]     gpu_addr  GPU address of the start of the allocated region
 *                          within which to search.
 * @param[in]     uaddr     Start of the CPU virtual address range.
 * @param[in]     size      Size of the CPU virtual address range (in bytes).
 *
 * @return A pointer to a descriptor of the CPU mapping that fully encloses
 *         the specified address range, or NULL if none was found.
 */
struct kbase_cpu_mapping *kbasep_find_enclosing_cpu_mapping(
                               struct kbase_context *kctx,
                               mali_addr64           gpu_addr,
                               osk_virt_addr         uaddr,
                               size_t                size );

#endif /* _KBASE_MEM_H_ */
