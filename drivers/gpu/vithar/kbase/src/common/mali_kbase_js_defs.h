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
 * @file mali_kbase_js.h
 * Job Scheduler Type Definitions
 */


#ifndef _KBASE_JS_DEFS_H_
#define _KBASE_JS_DEFS_H_

#include "mali_kbase_js_policy_fcfs.h"
#include "mali_kbase_js_policy_rr.h"

/**
 * @addtogroup base_api
 * @{
 */

/**
 * @addtogroup base_kbase_api
 * @{
 */

/**
 * @addtogroup kbase_js
 * @{
 */

/* Wrapper Interface - doxygen is elsewhere */
typedef union kbasep_js_policy
{
#ifdef KBASE_JS_POLICY_AVAILABLE_FCFS
	kbasep_js_policy_fcfs   fcfs;
#endif
#ifdef KBASE_JS_POLICY_AVAILABLE_RR
	kbasep_js_policy_rr     rr;
#endif
} kbasep_js_policy;

/* Wrapper Interface - doxygen is elsewhere */
typedef union kbasep_js_policy_ctx_info
{
#ifdef KBASE_JS_POLICY_AVAILABLE_FCFS
	kbasep_js_policy_fcfs_ctx   fcfs;
#endif
#ifdef KBASE_JS_POLICY_AVAILABLE_RR
	kbasep_js_policy_rr_ctx     rr;
#endif
} kbasep_js_policy_ctx_info;

/* Wrapper Interface - doxygen is elsewhere */
typedef union kbasep_js_policy_job_info
{
#ifdef KBASE_JS_POLICY_AVAILABLE_FCFS
	kbasep_js_policy_fcfs_job fcfs;
#endif
#ifdef KBASE_JS_POLICY_AVAILABLE_RR
	kbasep_js_policy_rr_job rr;
#endif
} kbasep_js_policy_job_info;

/**
 * @brief Maximum number of jobs that can be submitted to a job slot whilst
 * inside the IRQ handler.
 *
 * This is important because GPU NULL jobs can complete whilst the IRQ handler
 * is running. Otherwise, it potentially allows an unlimited number of GPU NULL
 * jobs to be submitted inside the IRQ handler, which increases IRQ latency.
 */
#define KBASE_JS_MAX_JOB_SUBMIT_PER_SLOT_PER_IRQ 2

/**
 * @brief the IRQ_THROTTLE time in microseconds
 *
 * This will be converted via the GPU's clock frequency into a cycle-count.
 *
 * @note we can make an estimate of the GPU's frequency by periodically
 * sampling its CYCLE_COUNT register
 */
#define KBASE_JS_IRQ_THROTTLE_TIME_US 20

/**
 * @brief KBase Device Data Job Scheduler sub-structure
 *
 * This encapsulates the current context of the Job Scheduler on a particular
 * device. This context is global to the device, and is not tied to any
 * particular kbase_context running on the device.
 *
 * nr_contexts_running, nr_nss_ctxs_running and as_free are optimized for
 * packing together (by making them smaller types than u32). The operations on
 * them should rarely involve masking. The use of signed types for arithmetic
 * indicates to the compiler that the value will not rollover (which would be
 * undefined behavior), and so under the Total License model, it is free to
 * make optimizations based on that (i.e. to remove masking).
 */
typedef struct kbasep_js_device_data
{
	/**
	 * Run Pool lock, which is used from IRQ context. Unless otherwise stated,
	 * you must hold this lock whilst accessing any member of this structure.
	 *
	 * Being an IRQ lock, you must not sleep whilst holding this lock, even if
	 * held outside of IRQ context.
	 *
	 * In addition, this is used to access the kbasep_js_kctx_info::runpool substructure
	 */
	osk_spinlock_irq runpool_lock;

	/**
	 * Queue Lock, used to access the Policy's queue of contexts independently
	 * of the Run Pool.
	 *
	 * Of course, you don't need the Run Pool lock to access this.
	 */
	osk_mutex queue_lock;

	u16 as_free;                            /**< Bitpattern of free Address Spaces */

	/** Bitvector indicating whether a currently scheduled context is allowed to submit jobs */
	u16 submit_allowed;

	s8 nr_contexts_running;                 /**< Number of currently scheduled contexts */
	s8 nr_nss_ctxs_running;                 /**< Number of NSS contexts */

	/**
	 * Policy-specific information.
	 *
	 * Run Pool related members must hold the runpool_lock.
	 *
	 * Queue related members must hold the queue_lock.
	 */
	kbasep_js_policy policy;

	/** Core Requirements to match up with base_js_atom's core_req memeber
	 * @note This is a write-once member, and so no locking is required to read */
	base_jd_core_req js_reqs[BASE_JM_MAX_NR_SLOTS];

	/** The initalized-flag is placed at the end, to avoid cache-pollution (we should
	 * only be using this during init/term paths).
	 * @note This is a write-once member, and so no locking is required to read */
	int init_status;
} kbasep_js_device_data;

/**
 * @brief KBase Context Job Scheduling information structure
 *
 * This is a substructure in the kbase_context that encapsulates all the
 * scheduling information.
 */

typedef struct kbasep_js_kctx_info
{
	/**
	 * Runpool substructure. This must only be accessed whilst the Run Pool
	 * lock ( kbasep_js_device_data::runpool_lock ) is held. Hence, you may
	 * access its members from IRQ context.
	 *
	 * @note some of the members could be moved into kbasep_js_device_data for
	 * improved d-cache/tlb efficiency.
	 */
	struct
	{
		u32 nr_running_jobs;                    /**< Number of jobs currently running */
		kbasep_js_policy_ctx_info policy_ctx;   /**< Policy-specific context */
	} runpool;

	/**
	 * Job Scheduler Context information sub-structure. These members are
	 * accessed regardless of whether the context is:
	 * - In the Policy's Run Pool
	 * - In the Policy's Queue
	 * - Not queued nor in the Run Pool.
	 *
	 * You must obtain the jsctx_lock before accessing any other members of
	 * this substructure.
	 *
	 * You may not access any of these members from IRQ context.
	 */
	struct
	{
		osk_mutex jsctx_lock;                   /**< Job Scheduler Context lock */

		/** Number of jobs <b>ready to run</b> - does \em not include the jobs waiting in
		 * the dispatcher, and dependency-only jobs. See kbase_jd_context::job_nr
		 * for such jobs*/
		u32 nr_jobs;

		u32 nr_nss_jobs;                        /**< Number of NSS jobs (BASE_JD_REQ_NSS bit set) */

		/* NOTE: Unify the flags together */
		/**
		 * Is the context scheduled on the Run Pool? 
		 *
		 * This is only ever updated whilst the kbase_gpu_vm_lock() and is
		 * held, because the MMU registers must be updated whenever this is
		 * changed. The js_ctx_lock is also held whilst updating, of course.
		 *
		 * You must hold at least one of the kbase_gpu_vm_lock() or jsctx_lock
		 * when checking this and making changes based upon it. That is, the
		 * context won't get scheduled out/scheduled back in whilst holding
		 * either/both of these locks.
		 */
		mali_bool is_scheduled;
		mali_bool is_dying;                     /**< Is the context in the process of being evicted? */
	} ctx;

	/* The initalized-flag is placed at the end, to avoid cache-pollution (we should
	 * only be using this during init/term paths) */
	int init_status;
} kbasep_js_kctx_info;


/**
 * @brief The JS timer resolution, in microseconds
 *
 * Any non-zero difference in time will be at least this size.
 */
#define KBASEP_JS_TICK_RESOLUTION_US (1000000u/osk_time_mstoticks(1000))

/**
 * @note MIDBASE-769: OSK to add high resolution timer
 *
 * The underlying tick is an unsigned integral type
 */
typedef osk_ticks kbasep_js_tick;


#endif /* _KBASE_JS_DEFS_H_ */


/** @} */ /* end group kbase_js */
/** @} */ /* end group base_kbase_api */
/** @} */ /* end group base_api */
