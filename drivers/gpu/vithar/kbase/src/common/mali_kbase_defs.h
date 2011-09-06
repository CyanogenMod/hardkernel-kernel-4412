/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file mali_kbase_defs.h
 *
 * Defintions (types, #defines, etcs) common to Kbase. They are placed here to
 * allow the hierarchy of header files to work.
 */

#ifndef _KBASE_DEFS_H_
#define _KBASE_DEFS_H_

#define KBASE_DRV_NAME  "mali"
#define KBASE_REGISTER_TRACE_ENABLED 1

#include <mali_base_hwconfig.h>

/* Number of milliseconds before resetting the GPU when a job cannot be "zapped" from the hardware.
 * Note that the time is actually 2*ZAP_TIMEOUT between the context zap starting and the GPU actually being reset to
 * give other contexts time for their jobs to be soft-stopped and removed from the hardware before resetting.
 */
#define ZAP_TIMEOUT 5000

/* Forward declarations+defintions */
typedef struct kbase_context kbase_context;
typedef struct kbase_jd_atom kbasep_jd_atom;
typedef struct kbase_device kbase_device;

/**
 * The maximum number of Job Slots to support in the Hardware.
 *
 * You can optimize this down if your target devices will only ever support a
 * small number of job slots.
 */
#define BASE_JM_MAX_NR_SLOTS        16

#ifndef UINTPTR_MAX

/**
 * @brief Maximum value representable by type uintptr_t
 */
#if CSTD_CPU_32BIT
#define UINTPTR_MAX U32_MAX
#elif CSTD_CPU_64BIT
#define UINTPTR_MAX U64_MAX
#endif /* CSTD_CPU_64BIT */

#endif /* !defined(UINTPTR_MAX) */

/* mmu */
#define ENTRY_IS_ATE        1ULL
#define ENTRY_IS_INVAL      2ULL
#define ENTRY_IS_PTE        3ULL

#define MIDGARD_MMU_VA_BITS 48

#define ENTRY_ATTR_BITS (7ULL << 2) /* bits 4:2 */
#define ENTRY_RD_BIT (1ULL << 6)
#define ENTRY_WR_BIT (1ULL << 7)
#define ENTRY_SHARE_BITS (3ULL << 8) /* bits 9:8 */
#define ENTRY_ACCESS_BIT (1ULL << 10)
#define ENTRY_NX_BIT (1ULL << 54)

#define ENTRY_FLAGS_MASK (ENTRY_ATTR_BITS | ENTRY_RD_BIT | ENTRY_WR_BIT | ENTRY_SHARE_BITS | ENTRY_ACCESS_BIT | ENTRY_NX_BIT)

#if MIDGARD_MMU_VA_BITS > 39
#define MIDGARD_MMU_TOPLEVEL    0
#else
#define MIDGARD_MMU_TOPLEVEL    1
#endif

#define GROWABLE_FLAGS_REQUIRED (KBASE_REG_PF_GROW | KBASE_REG_ZONE_TMEM)
#define GROWABLE_FLAGS_MASK     (GROWABLE_FLAGS_REQUIRED | KBASE_REG_FREE)

#include "mali_kbase_js_defs.h"

typedef struct kbase_event {
	osk_dlist_item      entry;
	const void          *data;
	base_jd_event_code  event_code;
} kbase_event;


/* Hijack the event entry field to link the struct with the different
 * queues... */
typedef struct kbase_jd_bag {
	kbase_event event;
	u64         core_restriction;
	size_t      offset;
	u32         nr_atoms;
} kbase_jd_bag;

typedef struct kbase_jd_atom {
	kbase_event     event;
	osk_workq_work  work;
	kbasep_js_tick  start_timestamp;
	base_jd_atom    *atom;
	kbase_jd_bag    *bag;
	kbase_context   *kctx;
	base_jd_dep     pre_dep;
	base_jd_dep     post_dep;
	u32             nr_syncsets;

	kbasep_js_policy_job_info sched_info;
	/** Job Slot to retry submitting to if submission from IRQ handler failed
	 *
	 * NOTE: see if this can be unified into the another member e.g. the event */
	int             retry_submit_on_slot;
} kbase_jd_atom;

/*
 * Theory of operations:
 *
 * - sem is an array of 256 bits, each bit being a semaphore
 * for a 1-1 job dependency:
 * Initially set to 0 (passing)
 * Incremented when a post_dep is queued
 * Decremented when a post_dep is completed
 * pre_dep is satisfied when value is 0
 * sem #0 is hardwired to 0 (always passing).
 *
 * - queue is an array of atoms, one per semaphore.
 * When a pre_dep is not satisfied, the atom is added to both
 * queues it depends on (except for queue 0 which is never used).
 * Each time a post_dep is signal, the corresponding bit is cleared,
 * the atoms removed from the queue, and the corresponding pre_dep
 * is cleared. The atom can be run when pre_dep[0] == pre_dep[1] == 0.
 */

typedef struct kbase_jd_dep_queue {
	kbase_jd_atom *queue[256];
	u32            sem[BASEP_JD_SEM_ARRAY_SIZE];
} kbase_jd_dep_queue;

typedef struct kbase_jd_context {
	osk_mutex           lock;
	kbasep_js_kctx_info	sched_info;
	kbase_jd_dep_queue  dep_queue;
	base_jd_atom        *pool;
	size_t              pool_size;

	/** Tracks all job-dispatch jobs.  This includes those not tracked by
	 * the scheduler: 'not ready to run' and 'dependency-only' jobs. */
	u32                 job_nr;

	osk_waitq           idle_waitq;
	osk_workq           job_done_wq;
	osk_spinlock_irq    tb_lock;
	u32                 *tb;
	size_t              tb_wrap_offset;
} kbase_jd_context;

typedef struct kbase_jm_slot {
	osk_spinlock_irq lock;

	/* The number of slots must be a power of two */
#define BASE_JM_SUBMIT_SLOTS        16
#define BASE_JM_SUBMIT_SLOTS_MASK   (BASE_JM_SUBMIT_SLOTS - 1)

	kbase_jd_atom    *submitted[BASE_JM_SUBMIT_SLOTS];

	u8               submitted_head;
	u8               submitted_nr;

#ifdef BASE_HW_ISSUE_5713
	/** Are we allowed to submit to this slot?
	 * MALI_TRUE if submission is not permitted at this time due to a soft-stop in progress.
	 */
	mali_bool8      submission_blocked_for_soft_stop;
#endif
} kbase_jm_slot;

typedef enum kbase_midgard_type
{
	KBASE_MALI_T6XM,
	KBASE_MALI_T6F1,
	KBASE_MALI_T601,
	KBASE_MALI_T604,
	KBASE_MALI_T608
} kbase_midgard_type;

#define KBASE_FEATURE_HAS_MODEL_PMU             (1U << 0)
#define KBASE_FEATURE_NEEDS_REG_DELAY           (1U << 1)
#define KBASE_FEATURE_HAS_16BIT_PC              (1U << 2)
#define KBASE_FEATURE_LACKS_RESET_INT           (1U << 3)
#define KBASE_FEATURE_DELAYED_PERF_WRITE_STATUS (1U << 4)

typedef struct kbase_device_info
{
	kbase_midgard_type  dev_type;
	u32                 features;
} kbase_device_info;


/* NOTE: Move into Property Query */
#define KBASE_JSn_FEATURE_NULL_JOB         (1u << 1)
#define KBASE_JSn_FEATURE_SET_VALUE_JOB    (1u << 2)
#define KBASE_JSn_FEATURE_CACHE_FLUSH_JOB  (1u << 3)
#define KBASE_JSn_FEATURE_COMPUTE_JOB      (1u << 4)
#define KBASE_JSn_FEATURE_VERTEX_JOB       (1u << 5)
#define KBASE_JSn_FEATURE_GEOMETRY_JOB     (1u << 6)
#define KBASE_JSn_FEATURE_TILER_JOB        (1u << 7)
#define KBASE_JSn_FEATURE_FUSED_JOB        (1u << 8)
#define KBASE_JSn_FEATURE_FRAGMENT_JOB     (1u << 9)
typedef u16 kbasep_jsn_feature;

/**
 * Important: Our code makes assumptions that a kbase_as structure is always at
 * kbase_device->as[number]. This is used to recover the containing
 * kbase_device from a kbase_as structure.
 *
 * Therefore, kbase_as structures must not be allocated anywhere else.
 */
typedef struct kbase_as
{
	int               number;
	int               pf_busy_count; /**< Ref count of whether a page fault is currently busy on this AS */
	osk_workq         pf_wq;
	osk_workq_work    work;
	mali_addr64       fault_addr;
	osk_spinlock_irq  transaction_lock;
} kbase_as;

struct kbase_device {
	const kbase_device_info *dev_info;
	kbase_jm_slot           jm_slots[BASE_JM_MAX_NR_SLOTS];
	s8                      slot_submit_count_irq[BASE_JM_MAX_NR_SLOTS];
	kbase_os_device         osdev;
	kbase_pm_device_data    pm;
	kbasep_js_device_data   js_data;

	kbase_as                as[16];

	osk_phy_allocator       mmu_fault_allocator;
	osk_phy_addr            mmu_fault_pages[4];
	osk_spinlock_irq        mmu_mask_change;

	/*
	 * NOTE: Add Property Query Here
	 *
	 * Some memebrs are optimized for packing together (by making them smaller
	 * types than u32). The operations on them should rarely involve
	 * masking. The use of signed types for arithmetic indicates to the
	 * compiler that the value will not rollover (which would be undefined
	 * behavior), and so under the Total License model, it is free to make
	 * optimizations based on that (i.e. to remove masking).
	 */
	u64                     shader_present_bitmap;
	s8                      nr_address_spaces;     /**< Number of address spaces in the GPU (constant) */
	s8                      nr_job_slots;          /**< Number of job slots in the GPU (constant) */

	/** GPU's JSn_FEATURES registers - not directly applicable to a base_jd_atom's
	 * core_req member. Instead, see the kbasep_js_device_data's js_reqs[] member */
	kbasep_jsn_feature      job_slot_features[BASE_JM_MAX_NR_SLOTS];

	osk_spinlock            hwcnt_lock;
	struct kbase_context *  hwcnt_context;
	u64                     hwcnt_addr;
	mali_bool               hwcnt_in_progress;

	mali_bool               expecting_reset_completed_irq;

	/* Set when we're about to reset the GPU */
	osk_atomic              reset_gpu;
	/* Work queue and work item for performing the reset in */
	osk_workq               reset_workq;
	osk_workq_work          reset_work;
};

struct kbase_context {
	kbase_device            *kbdev;
	osk_phy_allocator       pgd_allocator;
	osk_phy_addr            pgd;

	osk_dlist               event_list;
	osk_mutex               event_mutex;
	mali_bool               event_closed;

	osk_mutex               reg_lock; /* To be converted to a rwlock? */
	osk_dlist               reg_list; /* Ordered list of GPU regions */

	kbase_os_context        osctx;
	kbase_jd_context        jctx;
	ukk_session             ukk_session;

	/** This is effectively part of the Run Pool, because it only has a valid
	 * setting (non-negative) whilst the context is scheduled */
	int                     as_nr;
};


#if KBASE_REGISTER_TRACE_ENABLED
typedef enum kbase_reg_access_type
{
	REG_READ,
	REG_WRITE
} kbase_reg_access_type;
#endif /* KBASE_REGISTER_TRACE_ENABLED */


#endif /* _KBASE_DEFS_H_ */
