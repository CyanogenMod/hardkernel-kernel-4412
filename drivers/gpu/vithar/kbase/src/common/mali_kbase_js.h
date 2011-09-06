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
 * Job Scheduler APIs.
 */

#ifndef _KBASE_JS_H_
#define _KBASE_JS_H_

#include <malisw/mali_malisw.h>
#include <osk/mali_osk.h>

#include "mali_kbase_js_defs.h"
#include "mali_kbase_js_policy.h"

/**
 * @addtogroup base_api
 * @{
 */

/**
 * @addtogroup base_kbase_api
 * @{
 */

/**
 * @addtogroup kbase_js Job Scheduler Internal APIs
 * @{
 *
 * These APIs are Internal to KBase and are available for use by the
 * @ref kbase_js_policy "Job Scheduler Policy APIs"
 */



/**
 * @brief Efficient Zero-initializer for kbasep_js_device_data sub-structure of
 * a kbase_device.
 *
 * This only zero-initializes the parts of the structure required for
 * subsequent initialization/termination. If the memory backing the
 * kbasep_js_device_data sub-structure of \a kbdev has already been
 * zero-initialized, then you need not call this function on that structure.
 */
static INLINE void kbasep_js_devdata_zeroinit( kbase_device *kbdev )
{
	kbasep_js_device_data *js_devdata;
	OSK_ASSERT( kbdev != NULL );

	js_devdata = &kbdev->js_data;

	js_devdata->init_status = 0;
}

/**
 * @brief Initialize the Job Scheduler
 *
 * The kbasep_js_device_data sub-structure of \a kbdev must be zero
 * initialized before passing to the kbasep_js_devdata_init() function. This is
 * to give efficient error path code.
 *
 * If your memory allocator has _not_ already zero-initialized the memory
 * backing \a kbdev, then you may efficiently zero-initialized the
 * kbasep_js_device_data sub-structure with kbasep_js_devdata_zeroinit().
 */
mali_error kbasep_js_devdata_init( kbase_device *kbdev );

/**
 * @brief Terminate the Job Scheduler
 *
 * It is safe to call this on \a kbdev even if it the kbasep_js_device_data
 * sub-structure was never initialized/failed initialization, to give efficient
 * error-path code.
 *
 * For this to work, the kbasep_js_device_data sub-structure of \a kbdev must
 * be zero initialized before passing to the kbasep_js_devdata_init()
 * function. This is to give efficient error path code.
 *
 * If your memory allocator has _not_ already zero-initialized the memory
 * backing \a kbdev, then you may efficiently zero-initialized the
 * kbasep_js_device_data sub-structure with kbasep_js_devdata_zeroinit().
 *
 * It is a Programming Error to call this whilst there are still kbase_context
 * structures registered with this scheduler.
 */
void kbasep_js_devdata_term( kbase_device *kbdev );



/**
 * @brief Efficient Zero-initializer for the kbasep_js_kctx_info sub-structure
 * within a kbase_context
 *
 * This only zero-initializes the parts of the structure required for
 * subsequent initialization/termination. If the memory backing
 * kbasep_js_kctx_info has already been zero-initialize, then you need not
 * call this function on that structure.
 */
static INLINE void kbasep_js_kctx_info_zeroinit( kbase_context *kctx )
{
	OSK_ASSERT( kctx != NULL );
	kctx->jctx.sched_info.init_status = 0;
}

/**
 * @brief Initialize the Scheduling Component of a kbase_context on the Job Scheduler.
 *
 * This effectively registers a kbase_context with a Job Scheduler.
 *
 * It does not register any jobs owned by the kbase_context with the scheduler.
 * Those must be separately registered by kbasep_js_add_job().
 *
 * The kbase_context must be zero intitialized before passing to the
 * kbase_js_init() function. This is to give efficient error path code.
 *
 * If your memory allocator has not already zero-initialized the memory backing
 * kbase_context, then you may efficiently zero-initialize the jctx.sched_info
 * member with kbasep_js_kctx_info_zeroinit().
 */
mali_error kbasep_js_kctx_init( kbase_context *kctx );

/**
 * @brief Terminate the Scheduling Component of a kbase_context on the Job Scheduler
 *
 * This effectively de-registers a kbase_context from its Job Scheduler
 *
 * It is safe to call this on a kbase_context that has never had or failed
 * initialization of its jctx.sched_info member, to give efficient error-path
 * code.
 *
 * For this to work, the kbase_context must be zero intitialized before passing
 * to the kbase_js_init() function.
 *
 * If your memory allocator has not already zero-initialized the memory backing
 * kbase_context, then you may efficiently zero-initialize the jctx.sched_info
 * member with kbasep_js_kctx_info_zeroinit().
 *
 * It is a Programming Error to call this whilst there are still jobs
 * registered with this context.
 */
void kbasep_js_kctx_term( kbase_context *kctx );

/**
 * @brief Add a job chain to the Job Scheduler, and take necessary actions to
 * schedule the context/run the job.
 *
 * The Policy's Queue can be updated by this in the following ways:
 * - If previously there were no jobs on the context, then this context is
 * enqueued onto the Policy Queue
 * - If the job is high priority, then it could cause the Policy to schedule out
 * a low-priority context, allowing this context to be scheduled in.
 *
 * It is a programming error to have more than U32_MAX jobs in flight at a time.
 *
 * The following locking conditions are made on the caller:
 * - it must hold kbasep_js_kctx_info::ctx::jsctx_lock.
 * - it must \em not hold kbasep_js_device_data::runpool_lock (as this will be
 * obtained internally)
 * - it must \em not hold kbasep_jd_device_data::queue_lock (again, it's used internally).
 *
 * @return MALI_TRUE indicates that the Policy Queue was updated, and so the
 * caller will need to try scheduling a context onto the Run Pool.
 * @return MALI_FALSE indicates that no updates were made to the Policy Queue,
 * so no further action is required from the caller.
 */
mali_bool kbasep_js_add_job( kbase_context *kctx, kbase_jd_atom *atom );

/**
 * @brief Remove a job chain from the Job Scheduler
 *
 * It is a programming error to call this when:
 * - \a atom is not a job belonging to kctx.
 * - \a atom has already been removed from the Job Scheduler.
 *
 * The following locking conditions are made on the caller:
 * - it must hold kbasep_js_kctx_info::ctx::jsctx_lock.
 * - it must hold hold kbasep_js_device_data::runpool_lock
 *
 */
void kbasep_js_remove_job_nolock( kbase_context *kctx, kbase_jd_atom *atom );

/**
 * @brief See if a context should be removed from the Run Pool, and take the
 * appropriate actions thereafter.
 *
 * The following actions will be taken as part of deschduling a context:
 * - For the context being descheduled:
 *  - If the context is in the processing of dying (all the jobs are being
 * removed from it), then descheduling also kills off any jobs remaining in the
 * context.
 *  - Regardless of whether the context is dying or not, if any jobs remain
 * after descheduling the context then it is re-enqueued to the Policy's
 * Queue.
 *  - Otherwise, the context is still known to the scheduler, but remains absent
 * from the Policy Queue until a job is next added to it.
 * - Once the context is descheduled, this also handles scheduling in a new
 * context (if one is available), and if necessary, running a job from that new
 * context.
 *
 * It is a programming error to call this on a \a kctx that is not currently scheduled.
 *
 * The locking conditions on the caller are as follows:
 * - it must hold kbasep_js_kctx_info::ctx::jsctx_lock.
 * - it must \em not hold kbasep_js_device_data::runpool_lock (as this will be
 * obtained internally)
 * - it must \em not hold the kbase_gpu_vm_lock() (as this will be obtained internally)
 * - it must \em not hold the kbase_device::as[n].transaction_lock (as this will be obtained internally)
 *
 * @return MALI_TRUE indicates that the context was descheduled. The caller
 * should take appropriate actions to ensure the Run Pool is kept full.
 * @return MALI_FALSE indicates that the context stayed scheduled in. The
 * caller need not take any more actions.
 */
mali_bool kbasep_js_check_and_deschedule_ctx( kbase_device *kbdev, kbase_context *kctx );

/**
 * @brief Try to submit the next job on a \b particular slot whilst in IRQ
 * context, and whilst the caller already holds the job-slot IRQ spinlock.
 *
 * \a *submit_count will be checked against
 * KBASE_JS_MAX_JOB_SUBMIT_PER_SLOT_PER_IRQ to see whether too many jobs have
 * been submitted. This is to prevent the IRQ handler looping over lots of GPU
 * NULL jobs, which may complete whilst the IRQ handler is still processing. \a
 * submit_count itself should point to kbase_device::slot_submit_count_irq[ \a js ],
 * which is initialized to zero on entry to the IRQ handler.
 *
 * The following locks must be held by the caller:
 * - kbasep_js_device_data::runpool_lock
 * - kbdev->jm_slots[ \a js ].lock
 *
 * @return truthful (i.e. != MALI_FALSE) if there was space to submit in the
 * GPU, but we couldn't get a job from the Run Pool. This may be because the
 * Run Pool needs maintenence outside of IRQ context. Therefore, this indicates
 * that submission should be retried from a work-queue, by using
 * kbasep_js_try_run_next_job_on_slot().
 * @return MALI_FALSE if submission had no problems: the GPU is either already
 * full of jobs in the HEAD and NEXT registers, or we were able to get enough
 * jobs from the Run Pool to fill the GPU's HEAD and NEXT registers.
 */
mali_bool kbasep_js_try_run_next_job_on_slot_irq_nolock( kbase_device *kbdev, int js, s8 *submit_count );

/**
 * @brief Try to submit the next job on a particular slot, outside of IRQ context
 *
 * This obtains the Job Slot lock for the duration of the call only.
 *
 * Unlike kbasep_js_try_run_next_job_on_slot_irq_nolock(), there is no limit on
 * submission, because eventually IRQ_THROTTLE will kick in to prevent us
 * getting stuck in a loop of submitting GPU NULL jobs. This is because the IRQ
 * handler will be delayed, and so this function will eventually fill up the
 * space in our software 'submitted' slot (kbase_jm_slot::submitted).
 *
 * In addition, there's no return value - we'll run the maintenence functions
 * on the Policy's Run Pool, but if there's nothing there after that, then the
 * Run Pool is truely empty, and so no more action need be taken.
 *
 * The following locking conditions are made on the caller:
 * - it must \em not hold kbasep_js_kctx_info::ctx::jsctx_lock (as this will be
 * obtained internally)
 * - it must \em not hold kbasep_js_device_data::runpool_lock (as this will be
 * obtained internally)
 * - it must \em not hold kbasep_jd_device_data::queue_lock (again, it's used internally).
 * - it must \em not hold kbdev->jm_slots[ \a js ].lock (as this will be
 * obtained internally)
 *
 * @note The caller \em might be holding one of the
 * kbasep_js_kctx_info::ctx::jsctx_lock locks.
 *
 */
void kbasep_js_try_run_next_job_on_slot( kbase_device *kbdev, int js );

/**
 * @brief Try to schedule the next context onto the Run Pool
 *
 * This checks whether there's space in the Run Pool to accommodate a new
 * context. If so, it attempts to dequeue a context from the Policy Queue, and
 * submit this to the Run Pool.
 *
 * If the scheduling succeeds, then it also makes a call to
 * kbasep_js_try_run_next_job(), in case the new context has jobs matching the
 * job slot requirements, but no other currently scheduled context has such
 * jobs.
 *
 * If any of these actions fail (Run Pool Full, Policy Queue empty, etc) then
 * the function just returns normally.
 * 
 * The following locking conditions are made on the caller:
 * - it must \em not hold kbasep_js_device_data::runpool_lock (as this will be
 * obtained internally)
 * - it must \em not hold kbdev->jm_slots[ \a js ].lock (as this will be
 * obtained internally)
 * - it must \em not hold the kbase_gpu_vm_lock() (as this will be obtained internally)
 *
 */
void kbasep_js_try_schedule_head_ctx( kbase_device *kbdev );

/*
 * Helpers follow
 */

/**
 * @brief Check that a context is allowed to submit jobs on this policy
 *
 * The purpose of this abstraction is to hide the underlying data size, and wrap up
 * the long repeated line of code.
 *
 * As with any mali_bool, never test the return value with MALI_TRUE.
 *
 * The caller must hold kbasep_js_device_data::runpool_lock.
 */
static INLINE mali_bool kbasep_js_is_submit_allowed( kbasep_js_device_data *js_devdata, kbase_context *kctx )
{
	u16 test_bit = (u16)(1u << kctx->as_nr);

	OSK_ASSERT( kctx->jctx.sched_info.ctx.is_scheduled != MALI_FALSE );

	return (mali_bool)(js_devdata->submit_allowed & test_bit);
}

/**
 * @brief Allow a context to submit jobs on this policy
 *
 * The purpose of this abstraction is to hide the underlying data size, and wrap up
 * the long repeated line of code.
 *
 * The caller must hold kbasep_js_device_data::runpool_lock.
 */
static INLINE void kbasep_js_set_submit_allowed( kbasep_js_device_data *js_devdata, kbase_context *kctx )
{
	u16 set_bit = (u16)(1u << kctx->as_nr);

	OSK_ASSERT( kctx->jctx.sched_info.ctx.is_scheduled != MALI_FALSE );

	OSK_PRINT_INFO(OSK_BASE_JM, "JS: Setting Submit Allowed on %p (as=%d)", kctx, kctx->as_nr );

	js_devdata->submit_allowed |= set_bit;
}

/**
 * @brief Prevent a context from submitting more jobs on this policy
 *
 * The purpose of this abstraction is to hide the underlying data size, and wrap up
 * the long repeated line of code.
 *
 * The caller must hold kbasep_js_device_data::runpool_lock.
 */
static INLINE void kbasep_js_clear_submit_allowed( kbasep_js_device_data *js_devdata, kbase_context *kctx )
{
	u16 clear_bit = (u16)(1u << kctx->as_nr);
	u16 clear_mask = ~clear_bit;

	OSK_ASSERT( kctx->jctx.sched_info.ctx.is_scheduled != MALI_FALSE );

	OSK_PRINT_INFO(OSK_BASE_JM, "JS: Clearing Submit Allowed on %p (as=%d)", kctx, kctx->as_nr );

	js_devdata->submit_allowed &= clear_mask;
}

/**
 * @brief Manage the 'retry_submit_on_slot' part of a kbase_jd_atom
 */
static INLINE void kbasep_js_clear_job_retry_submit( kbase_jd_atom *atom )
{
	atom->retry_submit_on_slot = -1;
}

static INLINE mali_bool kbasep_js_get_job_retry_submit_slot( kbase_jd_atom *atom, int *res )
{
	int js = atom->retry_submit_on_slot;
	*res = js;
	return (mali_bool)( js >= 0 );
}

static INLINE void kbasep_js_set_job_retry_submit_slot( kbase_jd_atom *atom, int js )
{
	OSK_ASSERT( 0 <= js && js <= BASE_JM_MAX_NR_SLOTS );

	atom->retry_submit_on_slot = js;
}

/**
 * @note MIDBASE-769: OSK to add high resolution timer
 */
static INLINE kbasep_js_tick kbasep_js_get_js_ticks( void )
{
	return osk_time_now();
}

/**
 * Supports about an hour worth of time difference, allows the underlying
 * clock to be more/less accurate than microseconds
 *
 * @note MIDBASE-769: OSK to add high resolution timer
 */
static INLINE u32 kbasep_js_convert_js_ticks_to_us( kbasep_js_tick js_tick )
{
	return js_tick*(1000000u/osk_time_mstoticks(1000u));
}

/**
 * Supports about an hour worth of time difference, allows the underlying
 * clock to be more/less accurate than microseconds
 *
 * @note MIDBASE-769: OSK to add high resolution timer
 */
static INLINE kbasep_js_tick kbasep_js_convert_js_us_to_ticks( u32 us )
{
	return (us*osk_time_mstoticks(1000u))/1000000u;
}
/**
 * Determine if ticka comes after tickb
 *
 * @note MIDBASE-769: OSK to add high resolution timer
 */
static INLINE mali_bool kbasep_js_ticks_after( kbasep_js_tick ticka, kbasep_js_tick tickb )
{
	kbasep_js_tick tick_diff = ticka - tickb;
	const kbasep_js_tick wrapvalue = ((kbasep_js_tick)1u) << ((sizeof(kbasep_js_tick)*8)-1);

	return (mali_bool)(tick_diff < wrapvalue);
}

/** @} */ /* end group kbase_js */
/** @} */ /* end group base_kbase_api */
/** @} */ /* end group base_api */

#endif /* _KBASE_JS_H_ */
