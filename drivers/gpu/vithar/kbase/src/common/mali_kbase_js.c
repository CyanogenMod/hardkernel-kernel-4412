/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/*
 * Job Scheduler Implementation
 */
#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_js.h>

#include "mali_kbase_jm.h"

 /* For kbase_mmu_update(), kbase_mmu_disable(), kbase_gpu_vm_lock(), kbase_gpu_vm_unlock() */
#include <kbase/src/common/mali_kbase_mem.h>

/*
 * Private function prototypes
 */

STATIC INLINE void kbasep_js_check_and_ref_nss_running_ctx( kbasep_js_device_data *js_devdata, kbase_context *kctx );
STATIC INLINE void kbasep_js_check_and_deref_nss_running_ctx( kbasep_js_device_data *js_devdata, kbase_context *kctx );

STATIC void kbasep_js_check_and_ref_nss_job( kbasep_js_device_data *js_devdata,
											 kbase_context *kctx,
											 kbase_jd_atom *atom );
STATIC void kbasep_js_check_and_deref_nss_job( kbasep_js_device_data *js_devdata,
											   kbase_context *kctx,
											   kbase_jd_atom *atom );


/*
 * Private types
 */
enum
{
	JS_DEVDATA_INIT_NONE        =0,
	JS_DEVDATA_INIT_CONSTANTS   =(1 << 0),
	JS_DEVDATA_INIT_RUNPOOL_LOCK=(1 << 1),
	JS_DEVDATA_INIT_QUEUE_LOCK  =(1 << 2),
	JS_DEVDATA_INIT_POLICY      =(1 << 3),
	JS_DEVDATA_INIT_ALL         =((1 << 4)-1)
};

enum
{
	JS_KCTX_INIT_NONE        =0,
	JS_KCTX_INIT_CONSTANTS   =(1 << 0),
	JS_KCTX_INIT_JSCTX_LOCK  =(1 << 1),
	JS_KCTX_INIT_POLICY      =(1 << 2),
	JS_KCTX_INIT_ALL         =((1 << 3)-1)
};

/*
 * Private functions
 */
STATIC INLINE void kbasep_js_check_and_ref_nss_running_ctx( kbasep_js_device_data *js_devdata, kbase_context *kctx )
{
	kbasep_js_kctx_info *js_kctx_info;

	OSK_ASSERT( kctx != NULL );
	js_kctx_info = &kctx->jctx.sched_info;

	OSK_ASSERT( js_kctx_info->ctx.is_scheduled != MALI_FALSE );

	if ( js_kctx_info->ctx.nr_nss_jobs > 0 )
	{
		OSK_ASSERT( js_devdata->nr_nss_ctxs_running < S8_MAX );
		++(js_devdata->nr_nss_ctxs_running);

		if ( js_devdata->nr_nss_ctxs_running == 1 )
		{
			OSK_PRINT_INFO(OSK_BASE_JM, "JS: First NSS Context %p scheduled (switched to NSS state)", kctx );
		}
	}
}

STATIC INLINE void kbasep_js_check_and_deref_nss_running_ctx( kbasep_js_device_data *js_devdata, kbase_context *kctx )
{
	kbasep_js_kctx_info *js_kctx_info;

	OSK_ASSERT( kctx != NULL );
	js_kctx_info = &kctx->jctx.sched_info;

	OSK_ASSERT( js_kctx_info->ctx.is_scheduled != MALI_FALSE );

	if ( js_kctx_info->ctx.nr_nss_jobs > 0 )
	{
		OSK_ASSERT( js_devdata->nr_nss_ctxs_running > 0 );
		--(js_devdata->nr_nss_ctxs_running);

		if ( js_devdata->nr_nss_ctxs_running == 0 )
		{
			OSK_PRINT_INFO(OSK_BASE_JM, "JS: Last NSS Context %p descheduled (switched to SS state)", kctx );
		}
	}
}

STATIC void kbasep_js_check_and_ref_nss_job( kbasep_js_device_data *js_devdata,
											 kbase_context *kctx,
											 kbase_jd_atom *atom )
{
	kbasep_js_kctx_info *js_kctx_info;

	OSK_ASSERT( kctx != NULL );
	js_kctx_info = &kctx->jctx.sched_info;

	if ( atom->atom->core_req & BASE_JD_REQ_NSS )
	{
		OSK_ASSERT( js_kctx_info->ctx.nr_nss_jobs < U32_MAX );
		++(js_kctx_info->ctx.nr_nss_jobs);

		if ( js_kctx_info->ctx.is_scheduled != MALI_FALSE
			 && js_kctx_info->ctx.nr_nss_jobs == 1 )
		{
			/* Only NSS ref-count a running ctx on the first nss job */
			kbasep_js_check_and_ref_nss_running_ctx( js_devdata, kctx );
		}
	}
}

STATIC void kbasep_js_check_and_deref_nss_job( kbasep_js_device_data *js_devdata,
											   kbase_context *kctx,
											   kbase_jd_atom *atom )
{
	kbasep_js_kctx_info *js_kctx_info;

	OSK_ASSERT( kctx != NULL );
	js_kctx_info = &kctx->jctx.sched_info;

	if ( atom->atom->core_req & BASE_JD_REQ_NSS )
	{
		OSK_ASSERT( js_kctx_info->ctx.nr_nss_jobs > 0 );

		if ( js_kctx_info->ctx.is_scheduled != MALI_FALSE
			 && js_kctx_info->ctx.nr_nss_jobs == 1 )
		{
			/* Only NSS deref-count a running ctx on the last nss job */
			kbasep_js_check_and_deref_nss_running_ctx( js_devdata, kctx );
		}

		--(js_kctx_info->ctx.nr_nss_jobs);
	}
}


STATIC base_jd_core_req core_reqs_from_jsn_features( kbasep_jsn_feature features )
{
	base_jd_core_req core_req = 0u;

	if ( (features & KBASE_JSn_FEATURE_SET_VALUE_JOB) != 0 )
	{
		core_req |= BASE_JD_REQ_V;
	}
	if ( (features & KBASE_JSn_FEATURE_CACHE_FLUSH_JOB) != 0 )
	{
		core_req |= BASE_JD_REQ_CF;
	}
	if ( (features & KBASE_JSn_FEATURE_COMPUTE_JOB) != 0 )
	{
		core_req |= BASE_JD_REQ_CS;
	}
	if ( (features & KBASE_JSn_FEATURE_TILER_JOB) != 0 )
	{
		core_req |= BASE_JD_REQ_T;
	}
	if ( (features & KBASE_JSn_FEATURE_FRAGMENT_JOB) != 0 )
	{
		core_req |= BASE_JD_REQ_FS;
	}

	return core_req;
}

/**
 * You MUST hold the kbase_gpu_vm_lock before calling this
 */
STATIC void pick_and_assign_kctx_addr_space( kbase_device *kbdev, kbase_context *kctx )
{
	kbasep_js_device_data *js_devdata;
	long ffs_result;

	OSK_ASSERT( kbdev != NULL );
	OSK_ASSERT( kctx != NULL );

	js_devdata = &kbdev->js_data;

	/* Find the free address space */
	ffs_result = osk_find_first_set_bit( js_devdata->as_free );
	/* ASSERT that we should've found a free one */
	OSK_ASSERT( 0 <= ffs_result && ffs_result < kbdev->nr_address_spaces );

	/* Assign addr space */
	kctx->as_nr = (int)ffs_result;
	js_devdata->as_free &= ~((u16)(1u << ffs_result));

	/* The address space might be different from a previous use: update the GPU's MMU */
	kbase_mmu_update( kctx );
}

STATIC void kbasep_js_try_run_next_job( kbase_device *kbdev )
{
	int js;

	OSK_ASSERT( kbdev != NULL );

	for ( js = 0; js < kbdev->nr_job_slots ; ++js )
	{
		kbasep_js_try_run_next_job_on_slot( kbdev, js );
	}
}



/*
 * Functions private to KBase ('Protected' functions)
 */
mali_error kbasep_js_devdata_init( kbase_device *kbdev )
{
	kbasep_js_device_data *js_devdata;
	mali_error err;
	int i;
	u16 as_present;
	osk_error osk_err;

	OSK_ASSERT( kbdev != NULL );

	js_devdata = &kbdev->js_data;
	as_present = (1U << kbdev->nr_address_spaces) - 1;

	OSK_ASSERT( js_devdata->init_status == JS_DEVDATA_INIT_NONE );

	js_devdata->nr_contexts_running = 0;
	js_devdata->nr_nss_ctxs_running = 0;
	js_devdata->as_free = as_present; /* All ASs initially free */
	js_devdata->submit_allowed = 0u; /* No ctx allowed to submit */

	for ( i = 0; i < kbdev->nr_job_slots; ++i )
	{
		js_devdata->js_reqs[i] = core_reqs_from_jsn_features( kbdev->job_slot_features[i] );
	}
	js_devdata->init_status |= JS_DEVDATA_INIT_CONSTANTS;

	/* On error, we could continue on: providing none of the below resources
	 * rely on the ones above */

	osk_err = osk_spinlock_irq_init( &js_devdata->runpool_lock, OSK_LOCK_ORDER_JS_RUNPOOL );
	if ( osk_err == OSK_ERR_NONE )
	{
		js_devdata->init_status |= JS_DEVDATA_INIT_RUNPOOL_LOCK;
	}

	osk_err = osk_mutex_init( &js_devdata->queue_lock, OSK_LOCK_ORDER_JS_QUEUE );
	if ( osk_err == OSK_ERR_NONE )
	{
		js_devdata->init_status |= JS_DEVDATA_INIT_QUEUE_LOCK;
	}

	err = kbasep_js_policy_init( kbdev );
	if ( err == MALI_ERROR_NONE)
	{
		js_devdata->init_status |= JS_DEVDATA_INIT_POLICY;
	}

	/* On error, do no cleanup; this will be handled by the caller(s), since
	 * we've designed this resource to be safe to terminate on init-fail */
	if ( js_devdata->init_status != JS_DEVDATA_INIT_ALL)
	{
		return MALI_ERROR_FUNCTION_FAILED;
	}

	return MALI_ERROR_NONE;
}

void kbasep_js_devdata_term( kbase_device *kbdev )
{
	kbasep_js_device_data *js_devdata;

	OSK_ASSERT( kbdev != NULL );

	js_devdata = &kbdev->js_data;

	if ( (js_devdata->init_status & JS_DEVDATA_INIT_CONSTANTS) )
	{
		/* The caller must de-register all contexts before calling this */
		OSK_ASSERT( js_devdata->nr_contexts_running == 0 );
		OSK_ASSERT( js_devdata->nr_nss_ctxs_running == 0 );
	}
	if ( (js_devdata->init_status & JS_DEVDATA_INIT_POLICY) )
	{
		kbasep_js_policy_term( &js_devdata->policy );
	}
	if ( (js_devdata->init_status & JS_DEVDATA_INIT_QUEUE_LOCK) )
	{
		osk_mutex_term( &js_devdata->queue_lock );
	}
	if ( (js_devdata->init_status & JS_DEVDATA_INIT_RUNPOOL_LOCK) )
	{
		osk_spinlock_irq_term( &js_devdata->runpool_lock );
	}

	js_devdata->init_status = JS_DEVDATA_INIT_NONE;
}


mali_error kbasep_js_kctx_init( kbase_context *kctx )
{
	kbase_device *kbdev;
	kbasep_js_kctx_info *js_kctx_info;
	kbasep_js_policy    *js_policy;
	mali_error err;
	osk_error osk_err;

	OSK_ASSERT( kctx != NULL );

	kbdev = kctx->kbdev;
	OSK_ASSERT( kbdev != NULL );

	js_policy = &kbdev->js_data.policy;

	js_kctx_info = &kctx->jctx.sched_info;
	OSK_ASSERT( js_kctx_info->init_status == JS_KCTX_INIT_NONE );

	js_kctx_info->ctx.nr_jobs = 0;
	js_kctx_info->ctx.nr_nss_jobs = 0;

	js_kctx_info->ctx.is_scheduled = MALI_FALSE;
	js_kctx_info->ctx.is_dying = MALI_FALSE;

	js_kctx_info->init_status |= JS_KCTX_INIT_CONSTANTS;

	/* On error, we could continue on: providing none of the below resources
	 * rely on the ones above */
	osk_err = osk_mutex_init( &js_kctx_info->ctx.jsctx_lock, OSK_LOCK_ORDER_JS_CTX );
	if ( osk_err == OSK_ERR_NONE )
	{
		js_kctx_info->init_status |= JS_KCTX_INIT_JSCTX_LOCK;
	}

	err = kbasep_js_policy_init_ctx( js_policy, kctx );
	if ( err == MALI_ERROR_NONE )
	{
		js_kctx_info->init_status |= JS_KCTX_INIT_POLICY;
	}

	/* On error, do no cleanup; this will be handled by the caller(s), since
	 * we've designed this resource to be safe to terminate on init-fail */
	if ( js_kctx_info->init_status != JS_KCTX_INIT_ALL)
	{
		return MALI_ERROR_FUNCTION_FAILED;
	}

	return MALI_ERROR_NONE;
}

void kbasep_js_kctx_term( kbase_context *kctx )
{
	kbase_device *kbdev;
	kbasep_js_kctx_info *js_kctx_info;
	kbasep_js_policy    *js_policy;

	OSK_ASSERT( kctx != NULL );

	kbdev = kctx->kbdev;
	OSK_ASSERT( kbdev != NULL );

	js_policy = &kbdev->js_data.policy;
	js_kctx_info = &kctx->jctx.sched_info;

	if ( (js_kctx_info->init_status & JS_KCTX_INIT_CONSTANTS) )
	{
		/* The caller must de-register all jobs before calling this */
		OSK_ASSERT( js_kctx_info->ctx.is_scheduled == MALI_FALSE );
		OSK_ASSERT( js_kctx_info->ctx.nr_jobs == 0 );
		OSK_ASSERT( js_kctx_info->ctx.nr_nss_jobs == 0 );
	}

	if ( (js_kctx_info->init_status & JS_KCTX_INIT_JSCTX_LOCK) )
	{
		osk_mutex_term( &js_kctx_info->ctx.jsctx_lock );
	}

	if ( (js_kctx_info->init_status & JS_KCTX_INIT_POLICY) )
	{
		kbasep_js_policy_term_ctx( js_policy, kctx );
	}

	js_kctx_info->init_status = JS_DEVDATA_INIT_NONE;
}

mali_bool kbasep_js_add_job( kbase_context *kctx, kbase_jd_atom *atom )
{
	kbasep_js_kctx_info *js_kctx_info;
	kbase_device *kbdev;
	kbasep_js_device_data *js_devdata;
	kbasep_js_policy    *js_policy;

	mali_bool policy_queue_updated = MALI_FALSE;

	OSK_ASSERT( kctx != NULL );
	OSK_ASSERT( atom != NULL );

	kbdev = kctx->kbdev;
	js_devdata = &kbdev->js_data;
	js_policy = &kbdev->js_data.policy;
	js_kctx_info = &kctx->jctx.sched_info;

	/*
	 * Atomically do the following:
	 * - Update the numbers of jobs information
	 * - Add the job to the run pool if necessary (part of init_job)
	 */
	osk_spinlock_irq_lock( &js_devdata->runpool_lock );
	OSK_PRINT_INFO( OSK_BASE_JM, "JS: job enqueue %p", (void *)atom);

	/* Refcount ctx.nr_jobs */
	OSK_ASSERT( js_kctx_info->ctx.nr_jobs < U32_MAX );
	++(js_kctx_info->ctx.nr_jobs);
	kbasep_js_check_and_ref_nss_job( js_devdata, kctx, atom );

	/* Setup any scheduling information */
	kbasep_js_clear_job_retry_submit( atom );

	/* Register the job with the system, causing it to be scheduled if the
	 * parent context gets scheduled */
	kbasep_js_policy_init_job( js_policy, atom );

	osk_spinlock_irq_unlock( &js_devdata->runpool_lock );

	if ( js_kctx_info->ctx.is_scheduled != MALI_FALSE )
	{
		/* Handle an already running context - try to run the new job, in case it
		 * matches requirements that aren't matched by any other job in the Run
		 * Pool */
		kbasep_js_try_run_next_job( kbdev );
	}
	else if ( js_kctx_info->ctx.nr_jobs == 1 )
	{
		/* Handle Refcount going from 0 to 1: schedule the context on the Policy Queue */
		OSK_ASSERT( js_kctx_info->ctx.is_scheduled == MALI_FALSE );

		OSK_PRINT_INFO(OSK_BASE_JM, "JS: Enqueue Context %p", kctx );


		/* If the policy knows about 'Priority', it could evict a context that
		 * has no jobs currently running, which allows kctx to be subsequently
		 * dequeued from the head */
		osk_mutex_lock( &js_devdata->queue_lock );
		kbasep_js_policy_enqueue_ctx( js_policy, kctx );
		osk_mutex_unlock( &js_devdata->queue_lock );

		/* This context is becoming active */
		kbase_pm_context_active(kctx->kbdev);

		/* NOTE: Potentially, we can make the scheduling of the head context
		 * happen in a work-queue if we need to wait for the PM to power
		 * up. Also need logic to submit nothing until PM really has completed
		 * powering up. */

		/* Policy Queue was updated - caller must try to schedule the head context */
		policy_queue_updated = MALI_TRUE;
	}

	return policy_queue_updated;
}

void kbasep_js_remove_job_nolock( kbase_context *kctx, kbase_jd_atom *atom )
{
	kbasep_js_kctx_info *js_kctx_info;
	kbase_device *kbdev;
	kbasep_js_device_data *js_devdata;
	kbasep_js_policy    *js_policy;

	OSK_ASSERT( kctx != NULL );
	OSK_ASSERT( atom != NULL );

	kbdev = kctx->kbdev;
	js_devdata = &kbdev->js_data;
	js_policy = &kbdev->js_data.policy;
	js_kctx_info = &kctx->jctx.sched_info;

	/* De-refcount ctx.nr_jobs */
	OSK_ASSERT( js_kctx_info->ctx.nr_jobs > 0 );
	--(js_kctx_info->ctx.nr_jobs);
	kbasep_js_check_and_deref_nss_job( js_devdata, kctx, atom );

	/* Note: runpool.nr_running_jobs already updated elsewhere */

	/* De-register the job from the system */
	kbasep_js_policy_term_job( js_policy, atom );
}

mali_bool kbasep_js_check_and_deschedule_ctx( kbase_device *kbdev, kbase_context *kctx )
{
	kbasep_js_device_data *js_devdata;
	kbasep_js_kctx_info *js_kctx_info;
	kbasep_js_policy    *js_policy;
	
	mali_bool was_descheduled = MALI_FALSE;

	int as_nr;
	kbase_as *current_as;

	OSK_ASSERT( kbdev != NULL );
	OSK_ASSERT( kctx != NULL );

	js_kctx_info = &kctx->jctx.sched_info;
	js_devdata = &kbdev->js_data;
	js_policy = &kbdev->js_data.policy;

	OSK_ASSERT( js_kctx_info->ctx.is_scheduled != MALI_FALSE );

	/* Atomically do the following:
	 * - Check if the context should be scheduled out, and if so:
	 *  - Remove it from the Run Pool
	 *  - Free up the address space
	 *  - Update contexts running information
	 *  - mark as "not scheduled"
	 *  - Kill remaining jobs if the context is being zapped
	 *
	 * In addition, we need the VM lock for updating Address Space registers
	 *
	 * The AS Lock is also required to check the as refcount.
	 */
	kbase_gpu_vm_lock(kctx);
	osk_spinlock_irq_lock( &js_devdata->runpool_lock );

	as_nr = kctx->as_nr;
	OSK_ASSERT( as_nr >= 0 );

	current_as = &kbdev->as[as_nr];

	osk_spinlock_irq_lock( &current_as->transaction_lock );

	/* Make a set of checks to see if the context should be scheduled out
	 *
	 * Important: We don't schedule out whilst a PF handler is busy working on it.
	 * That handler must take responsibility to ensure this context gets checked for
	 * descheduling
	 */
	if ( current_as->pf_busy_count == 0
		 && kctx->jctx.sched_info.runpool.nr_running_jobs == 0
		 && ( kctx->jctx.sched_info.ctx.nr_jobs == 0
			  || kbasep_js_is_submit_allowed( js_devdata, kctx ) == MALI_FALSE ) )
	{
		/* Remove the context from the Run Pool */
		OSK_PRINT_INFO(OSK_BASE_JM, "JS: RunPool Remove Context %p because running_jobs=%d, jobs=%d, allowed=%d",
					   kctx,
					   kctx->jctx.sched_info.runpool.nr_running_jobs,
					   kctx->jctx.sched_info.ctx.nr_jobs,
					   kbasep_js_is_submit_allowed( js_devdata, kctx ) );

		kbasep_js_policy_runpool_remove_ctx( js_policy, kctx );

		/* Disable the MMU on the affected address space
		 *
		 * The kbase_gpu_vm_lock MUST be held by this point */
		kbase_mmu_disable( kctx );

		/* Free up the address space */
		js_devdata->as_free |= ((u16)(1u << kctx->as_nr));

		/* Clear the as_nr, for debugging */
		kctx->as_nr = -1;
	
		/* update book-keeping info */
		--(js_devdata->nr_contexts_running);
		kbasep_js_check_and_deref_nss_running_ctx( js_devdata, kctx );
		js_kctx_info->ctx.is_scheduled = MALI_FALSE;

		/* Handle dying contexts */
		if ( js_kctx_info->ctx.is_dying != MALI_FALSE )
		{
			/* This happens asynchronously */
			OSK_PRINT_INFO(OSK_BASE_JM, "JS: ** Killing Context %p on RunPool Remove **", kctx );
			kbasep_js_policy_kill_all_ctx_jobs( js_policy, kctx );
		}

		/* Queue an action to occur after we've dropped the lock */
		was_descheduled = MALI_TRUE;
	}
	osk_spinlock_irq_unlock( &current_as->transaction_lock );
	osk_spinlock_irq_unlock( &js_devdata->runpool_lock );
	kbase_gpu_vm_unlock(kctx);


	/* Do we have an action queued whilst the lock was held? */
	if ( was_descheduled != MALI_FALSE )
	{
		/* Determine whether this context should be requeued on the policy queue */
		if ( js_kctx_info->ctx.nr_jobs > 0 && js_kctx_info->ctx.is_dying == MALI_FALSE )
		{
			OSK_PRINT_INFO(OSK_BASE_JM, "JS: Requeue Context %p", kctx );
			osk_mutex_lock( &js_devdata->queue_lock );
			kbasep_js_policy_enqueue_ctx( js_policy, kctx );
			osk_mutex_unlock( &js_devdata->queue_lock );
		}
		else
		{
			OSK_PRINT_INFO(OSK_BASE_JM, "JS: Idling Context %p (not requeued)", kctx );
			/* Notify PM that a context has gone idle */
			kbase_pm_context_idle(kctx->kbdev);
		}
	}
	return was_descheduled;
}

/*
 * Note: this function is quite similar to kbasep_js_try_run_next_job_on_slot()
 */
mali_bool kbasep_js_try_run_next_job_on_slot_irq_nolock( kbase_device *kbdev, int js, s8 *submit_count )
{
	kbasep_js_device_data *js_devdata;
	mali_bool has_job = MALI_TRUE;

	OSK_ASSERT( kbdev != NULL );

	js_devdata = &kbdev->js_data;

	/* Keep submitting while there's space to run a job on this job-slot,
	 * and there are jobs to get that match its requirements (see 'break'
	 * statement below) */
	while ( *submit_count < KBASE_JS_MAX_JOB_SUBMIT_PER_SLOT_PER_IRQ
	        && kbasep_jm_is_submit_slots_free( kbdev, js, NULL ) != MALI_FALSE )
	{
		kbase_jd_atom *dequeued_atom;

		/* Dequeue a job that matches the requirements */
		has_job = kbasep_js_policy_dequeue_job_irq( kbdev, js, &dequeued_atom );

		if ( has_job != MALI_FALSE )
		{
			kbase_context *parent_ctx = dequeued_atom->kctx;
			kbasep_js_kctx_info *js_kctx_info = &parent_ctx->jctx.sched_info;

			/* ASSERT that the Policy picked a job from an allowed context */
			OSK_ASSERT( kbasep_js_is_submit_allowed( js_devdata, parent_ctx) );

			/* Update book-keeping */
			++(js_kctx_info->runpool.nr_running_jobs);

			/* Submit the job */
			kbase_job_submit_nolock( kbdev, dequeued_atom, js );

			++(*submit_count);
		}
		else
		{
			/* No more jobs - stop submitting for this slot */
			break;
		}
	}

	/* Indicate whether a retry in submission should be tried on a different dequeue function */
	return (mali_bool)(has_job == MALI_FALSE);
}

void kbasep_js_try_run_next_job_on_slot( kbase_device *kbdev, int js )
{
	kbasep_js_device_data *js_devdata;
	kbase_jm_slot *jm_slots;
	mali_bool has_job;

	OSK_ASSERT( kbdev != NULL );

	js_devdata = &kbdev->js_data;
	jm_slots = kbdev->jm_slots;

	osk_spinlock_irq_lock(&jm_slots[js].lock);

	/* Keep submitting while there's space to run a job on this job-slot,
	 * and there are jobs to get that match its requirements (see 'break'
	 * statement below) */
	if (  kbasep_jm_is_submit_slots_free( kbdev, js, NULL ) != MALI_FALSE )
	{
		/* Only lock the Run Pool whilst there's work worth doing */
		osk_spinlock_irq_lock( &js_devdata->runpool_lock );

		do {
			kbase_jd_atom *dequeued_atom;

			/* Dequeue a job that matches the requirements */
			has_job = kbasep_js_policy_dequeue_job( kbdev, js, &dequeued_atom );

			if ( has_job != MALI_FALSE )
			{
				kbase_context *parent_ctx = dequeued_atom->kctx;
				kbasep_js_kctx_info *js_kctx_info = &parent_ctx->jctx.sched_info;

				/* ASSERT that the Policy picked a job from an allowed context */
				OSK_ASSERT( kbasep_js_is_submit_allowed( js_devdata, parent_ctx) );

				/* Update book-keeping */
				++(js_kctx_info->runpool.nr_running_jobs);

				/* Submit the job */
				kbase_job_submit_nolock( kbdev, dequeued_atom, js );
			}

		} while ( kbasep_jm_is_submit_slots_free( kbdev, js, NULL ) != MALI_FALSE
		          && has_job != MALI_FALSE );
		
		osk_spinlock_irq_unlock( &js_devdata->runpool_lock );
	}
	osk_spinlock_irq_unlock(&jm_slots[js].lock);
}

void kbasep_js_try_schedule_head_ctx( kbase_device *kbdev )
{
	kbasep_js_device_data *js_devdata;
	mali_bool has_kctx;
	kbase_context *head_kctx;
	kbasep_js_kctx_info *js_kctx_info;
	mali_bool is_runpool_full;

	OSK_ASSERT( kbdev != NULL );

	js_devdata = &kbdev->js_data;

	/* Make a speculative check on the Run Pool - this MUST be repeated once
	 * we've obtained a context from the queue and reobtained the Run Pool
	 * lock */
	osk_spinlock_irq_lock( &js_devdata->runpool_lock );
	is_runpool_full = (mali_bool)( js_devdata->nr_contexts_running >= kbdev->nr_address_spaces );
	osk_spinlock_irq_unlock( &js_devdata->runpool_lock );

	if ( is_runpool_full != MALI_FALSE )
	{
		/* No free address spaces - nothing to do */
		return;
	}

	/* Grab the context off head of queue - if there is one */
	osk_mutex_lock( &js_devdata->queue_lock );
	has_kctx = kbasep_js_policy_dequeue_head_ctx( &js_devdata->policy, &head_kctx );
	osk_mutex_unlock( &js_devdata->queue_lock );

	if ( has_kctx == MALI_FALSE )
	{
		/* No ctxs to run - nothing to do */
		return;
	}
	js_kctx_info = &head_kctx->jctx.sched_info;

	OSK_PRINT_INFO(OSK_BASE_JM, "JS: Dequeue Context %p", head_kctx );

	/*
	 * Atomic transaction on the Context and Run Pool begins
	 *
	 * Also, the VM lock must be held for accessing the MMU registers
	 */
	osk_mutex_lock( &js_kctx_info->ctx.jsctx_lock );
	kbase_gpu_vm_lock( head_kctx );
	osk_spinlock_irq_lock( &js_devdata->runpool_lock );

	/* Re-check to see if the Run Pool is full */
	is_runpool_full = (mali_bool)( js_devdata->nr_contexts_running >= kbdev->nr_address_spaces );
	if ( is_runpool_full != MALI_FALSE )
	{
		/* No free address spaces - roll back the transaction so far and return */
		osk_spinlock_irq_unlock( &js_devdata->runpool_lock );

		/* Only requeue if not dying - which might occur through zapping-whilst-scheduling */
		if ( js_kctx_info->ctx.is_dying == MALI_FALSE )
		{
			OSK_PRINT_INFO(OSK_BASE_JM, "JS: Transaction rollback: Requeue Context %p", head_kctx );

			osk_mutex_lock( &js_devdata->queue_lock );
			kbasep_js_policy_enqueue_ctx( &js_devdata->policy, head_kctx );
			osk_mutex_unlock( &js_devdata->queue_lock );
		}
		else
		{
			OSK_PRINT_INFO(OSK_BASE_JM, "JS: Transaction rollback: Context %p is dying, leave it", head_kctx );
			/* Rest of transaction handled in kbase_job_zap_context() */
		}

		kbase_gpu_vm_unlock( head_kctx );
		osk_mutex_unlock( &js_kctx_info->ctx.jsctx_lock );
		return;
	}

	/* Do all the necessaries to pick the address space (inc. update book-keeping info) */
	pick_and_assign_kctx_addr_space( kbdev, head_kctx );

	/* Add the context to the Run Pool */
	OSK_PRINT_INFO(OSK_BASE_JM, "JS: RunPool Add Context %p", head_kctx );
	kbasep_js_policy_runpool_add_ctx( &js_devdata->policy, head_kctx );

	/* update book-keeping info */
	js_kctx_info->ctx.is_scheduled = MALI_TRUE;
	++(js_devdata->nr_contexts_running);
	kbasep_js_check_and_ref_nss_running_ctx( js_devdata, head_kctx );

	/* Allow it to run jobs */
	kbasep_js_set_submit_allowed( js_devdata, head_kctx );

	/* Transaction complete */
	osk_spinlock_irq_unlock( &js_devdata->runpool_lock );
	kbase_gpu_vm_unlock( head_kctx );
	osk_mutex_unlock( &js_kctx_info->ctx.jsctx_lock );


	/* Try to run the next job, in case this context has jobs that match the
	 * job slot requirements, but none of the other currently running contexts
	 * do */
	kbasep_js_try_run_next_job( kbdev );
}
