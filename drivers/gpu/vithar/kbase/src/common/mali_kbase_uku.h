/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

#ifndef _KBASE_UKU_H_
#define _KBASE_UKU_H_

#include <uk/mali_uk.h>
#include <uk/mali_uku.h>
#include <ump/ump.h>
#include <malisw/mali_malisw.h>
#include <base/mali_base_kernel.h>

#define BASE_UK_VERSION_MAJOR 0
#define BASE_UK_VERSION_MINOR 0

typedef struct kbase_uk_tmem_alloc
{
	uk_header   header;
	/* IN */
	u32         vsize;
	u32         psize;
	u32         extent;
	u32         flags;
	/* OUT */
	mali_addr64 gpu_addr;
} kbase_uk_tmem_alloc;

typedef struct kbase_uk_tmem_from_ump
{
	uk_header   header;
	/* IN */
	ump_secure_id id;
	/* OUT */
	mali_addr64 gpu_addr;
	u64 pages;
} kbase_uk_tmem_from_ump;

typedef struct kbase_uk_pmem_alloc
{
	uk_header   header;
	/* IN */
	u32         vsize;
	u32         flags;
	/* OUT */
	u16         cookie;
} kbase_uk_pmem_alloc;

typedef struct kbase_uk_mem_free
{
	uk_header   header;
	/* IN */
	mali_addr64 gpu_addr;
	/* OUT */
} kbase_uk_mem_free;

typedef struct kbase_uk_job_submit
{
	uk_header   header;
	/* IN */
	u64         bag_uaddr;
	u64         core_restriction;
	u32         offset;
	u32         size;
	u32         nr_atoms;
	/* OUT */
} kbase_uk_job_submit;

typedef struct kbase_uk_post_term
{
	uk_header   header;
} kbase_uk_post_term;

typedef struct kbase_uk_sync_now
{
	uk_header   header;

	/* IN */
	base_syncset sset;

	/* OUT */
} kbase_uk_sync_now;

typedef struct kbase_uk_hwcnt_setup
{
	uk_header   header;

	/* IN */
	mali_addr64 dump_buffer;
	u32         jm_bm;
	u32         shader_bm;
	u32         tiler_bm;
	u32         l3_cache_bm;
	u32         mmu_l2_bm;
	/* OUT */
} kbase_uk_hwcnt_setup;

typedef struct kbase_uk_hwcnt_dump
{
	uk_header   header;
} kbase_uk_hwcnt_dump;

typedef struct kbase_uk_tmem_resize
{
	uk_header   header;
	/* IN */
	mali_addr64 gpu_addr;
	s32         delta;
	/* OUT */
	u32         size;
	base_backing_threshold_status result_subcode;
} kbase_uk_tmem_resize;

typedef struct kbase_uk_find_cpu_mapping
{
	uk_header     header;
	/* IN */
	mali_addr64   gpu_addr;
	u64           cpu_addr;
	u64           size;
	/* OUT */
	u64           uaddr;
	u32           nr_pages;
	mali_size64   page_off;
} kbase_uk_find_cpu_mapping;

typedef enum ump_uk_function_id
{
	KBASE_FUNC_TMEM_ALLOC = (UK_FUNC_ID + 0),
	KBASE_FUNC_TMEM_FROM_UMP,
	KBASE_FUNC_PMEM_ALLOC,
	KBASE_FUNC_MEM_FREE,

	KBASE_FUNC_JOB_SUBMIT,

	KBASE_FUNC_SYNC,

	KBASE_FUNC_POST_TERM,

	KBASE_FUNC_HWCNT_SETUP,
	KBASE_FUNC_HWCNT_DUMP,

	KBASE_FUNC_TMEM_RESIZE,

	KBASE_FUNC_FIND_CPU_MAPPING

} ump_uk_function_id;

#endif /* _KBASE_UKU_H_ */
