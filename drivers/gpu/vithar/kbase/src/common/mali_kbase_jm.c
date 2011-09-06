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
 * @file mali_kbase_jm.c
 * Base kernel job manager APIs
 */

#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_mem.h>
#include <kbase/src/common/mali_midg_regmap.h>

#include "mali_kbase_jm.h"

#define beenthere(f, a...)  OSK_PRINT_INFO(OSK_BASE_JM, "%s:" f, __func__, ##a)

static void kbase_job_hw_submit(kbase_device *kbdev, kbase_jd_atom *katom, int js)
{
	kbase_context *kctx = katom->kctx;
	u64 affinity = 0;
	u32 cfg;

	/* Command register must be available */
	OSK_ASSERT(kbasep_jm_is_js_free(kbdev, js, katom->kctx));

	kbase_reg_write(kbdev, JOB_SLOT_REG(js, JSn_HEAD_NEXT_LO), katom->atom->jc & 0xFFFFFFFF, katom->kctx);
	kbase_reg_write(kbdev, JOB_SLOT_REG(js, JSn_HEAD_NEXT_HI), katom->atom->jc >> 32, katom->kctx);

	switch(js)
	{
		case 0: /* Fragment shaders are the lucky ones... */
		affinity = kbdev->shader_present_bitmap;
		break;

		case 1:
		if (kbdev->shader_present_bitmap == 1ULL)
		{
			affinity = 1ULL;
			break;
		}
		/* Fall through */
		case 2:
		{
			/*
			 * Statically partition the cores for JS1/JS2.
			 * Assume we'll never have more that 2^32
			 * cores... Actually, there's no need in
			 * computing this for every JC until this is
			 * controlled by PM/job-scheduler.
			 */
			u32 order = 32 - osk_clz(kbdev->shader_present_bitmap
			                         & 0xFFFFFFFF);
			order += 1;
			order >>= 1;
			affinity = ((1 << order) -1) << (order * (js - 1));
			break;
		}
	}

	OSK_ASSERT(affinity != 0);

	kbase_reg_write(kbdev, JOB_SLOT_REG(js, JSn_AFFINITY_NEXT_LO), affinity & 0xFFFFFFFF, katom->kctx);
	kbase_reg_write(kbdev, JOB_SLOT_REG(js, JSn_AFFINITY_NEXT_HI), affinity >> 32, katom->kctx);

	/* start MMU, medium priority, cache clean/flush on end, clean/flush on start */
	cfg = kctx->as_nr | (3 << 12) | (1 << 10) | (8 << 16) | (3 << 8);
	kbase_reg_write(kbdev, JOB_SLOT_REG(js, JSn_CONFIG_NEXT), cfg, katom->kctx);

	/* Write an approximate start timestamp.
	 * It's approximate because there might be a job in the HEAD register. In
	 * such cases, we'll try to make a better approximation in the IRQ handler
	 * (up to the KBASE_JS_IRQ_THROTTLE_TIME_US). */
	katom->start_timestamp = kbasep_js_get_js_ticks();

	/* GO ! */
	OSK_PRINT_INFO(OSK_BASE_JM, "JS: Submitting atom %p from ctx %p to js[%d] with head=0x%llx, affinity=0x%llx",
				   katom, kctx, js, katom->atom->jc, affinity );

	kbase_reg_write(kbdev, JOB_SLOT_REG(js, JSn_COMMAND_NEXT), 1, katom->kctx);
}

void kbase_job_submit_nolock(kbase_device *kbdev, kbase_jd_atom *katom, int js)
{
	kbase_jm_slot *jm_slots = kbdev->jm_slots;

#ifdef BASE_HW_ISSUE_5713
	OSK_ASSERT(!jm_slots[js].submission_blocked_for_soft_stop);
#endif

	/*
	 * We can have:
	 * - one job already done (pending interrupt),
	 * - one running,
	 * - one ready to be run.
	 * Hence a maximum of 3 inflight jobs. We have a 4 job
	 * queue, which I hope will be enough...
	 */
	kbasep_jm_enqueue_submit_slot( &jm_slots[js], katom );
	kbase_job_hw_submit(kbdev, katom, js);
}

void kbase_job_done_slot(kbase_device *kbdev, int s, u32 completion_code, u64 job_tail, kbasep_js_tick *end_timestamp)
{
	kbase_jm_slot *slot = &kbdev->jm_slots[s];
	kbase_jd_atom *katom;
	kbasep_js_policy *js_policy;
	kbasep_js_device_data *js_devdata;
	mali_bool submit_retry_needed;

	js_devdata = &kbdev->js_data;
	js_policy = &kbdev->js_data.policy;

	katom = kbasep_jm_dequeue_submit_slot( slot );
	
	if (job_tail != 0)
	{
		/* Some of the job has been executed, so we update the job chain address to where we should resume from */
		katom->atom->jc = job_tail;
	}

#ifdef BASE_HW_ISSUE_5713
	if (kbasep_jm_nr_jobs_submitted( slot ) == 0)
	{
		slot->submission_blocked_for_soft_stop = MALI_FALSE;
	}
#endif

	/* Only update the event code for jobs that weren't cancelled */
	if ( katom->event.event_code != BASE_JD_EVENT_JOB_CANCELLED )
	{
		katom->event.event_code = completion_code;
	}

	/*
	 * This atomically does the following:
	 * - updates the runpool's notion of time spent by a running ctx
	 * - determines whether a context should be marked for scheduling out
	 * - tries to submit the next job on the slot (picking from all ctxs in the runpool)
	 *
	 * NOTE: This should be moved this into the scheduling core
	 */
	osk_spinlock_irq_lock( &js_devdata->runpool_lock );

	{
		kbasep_js_tick tick_diff;
		u32 microseconds_spent;
		kbase_context *parent_ctx;

		parent_ctx = katom->kctx;

		/* Calculate the job's time used */
		tick_diff = *end_timestamp - katom->start_timestamp;
		microseconds_spent = kbasep_js_convert_js_ticks_to_us( tick_diff );
		/* Round up time spent to the minimum timer resolution */
		if (microseconds_spent < KBASEP_JS_TICK_RESOLUTION_US)
		{
			microseconds_spent = KBASEP_JS_TICK_RESOLUTION_US;
		}

		/* Log the result of the job (completion status, and time spent). */
		kbasep_js_policy_log_job_result( js_policy, katom, microseconds_spent );

		/* Determine whether the parent context's timeslice is up */
		if ( kbasep_js_policy_should_remove_ctx( js_policy, parent_ctx ) != MALI_FALSE )
		{
			kbasep_js_clear_submit_allowed( js_devdata, parent_ctx );
		}
	}

	/* Submit a new job (if there is one) to help keep the GPU's HEAD and NEXT registers full */
	submit_retry_needed = kbasep_js_try_run_next_job_on_slot_irq_nolock(
		kbdev,
		s,
		&kbdev->slot_submit_count_irq[s] );

	if ( submit_retry_needed != MALI_FALSE )
	{
		kbasep_js_set_job_retry_submit_slot( katom, s );
	}

	kbase_device_trace_register_access(katom->kctx, REG_WRITE , JOB_CONTROL_REG(JOB_IRQ_CLEAR), 1 << s);

	/* Defer the remaining work onto the workqueue:
	 * - Re-queue Soft-stopped jobs
	 * - For any other jobs, queue the job back into the dependency system
	 * - Schedule out the parent context if necessary, and schedule a new one in.
	 * - Try submitting jobs from outside of IRQ context if we failed here.
	 */
	kbase_jd_done(katom);

	osk_spinlock_irq_unlock( &js_devdata->runpool_lock );

}

/**
 * Update the start_timestamp of the job currently in the HEAD, based on the
 * fact that we got an IRQ for the previous set of completed jobs.
 *
 * The estimate also takes into account the KBASE_JS_IRQ_THROTTLE_TIME_US and
 * the time the job was submitted, to work out the best estimate (which might
 * still result in an over-estimate to the calculated time spent)
 */
STATIC void kbasep_job_slot_update_head_start_timestamp( kbase_jm_slot *slot, kbasep_js_tick end_timestamp )
{
	if ( kbasep_jm_nr_jobs_submitted( slot ) > 0 )
	{
		kbase_jd_atom *katom;
		kbasep_js_tick new_timestamp;
		katom = kbasep_jm_peek_idx_submit_slot( slot, 0 ); /* The atom in the HEAD */

		OSK_ASSERT( katom != NULL );

		/* Account for any IRQ Throttle time - makes an overestimate of the time spent by the job */
		new_timestamp = end_timestamp - kbasep_js_convert_js_us_to_ticks(KBASE_JS_IRQ_THROTTLE_TIME_US);
		if ( kbasep_js_ticks_after( new_timestamp, katom->start_timestamp ) )
		{
			/* Only update the timestamp if it's a better estimate than what's currently stored.
			 * This is because our estimate that accounts for the throttle time may be too much
			 * of an overestimate */
			katom->start_timestamp = new_timestamp;
		}
	}
}

void kbase_job_done(kbase_device *kbdev, u32 done)
{
	int i;
	u32 count = 0;
	kbasep_js_tick end_timestamp = kbasep_js_get_js_ticks();
 
	OSK_MEMSET( &kbdev->slot_submit_count_irq[0], 0, sizeof(kbdev->slot_submit_count_irq) );

	while (done) {
		kbase_jm_slot *slot;
		u32 failed = done >> 16;

		/* treat failed slots as finished slots */
		u32 finished = (done & 0xFFFF) | failed;

		/* Note: This is inherently unfair, as we always check
		 * for lower numbered interrupts before the higher
		 * numbered ones.*/
		i = osk_find_first_set_bit(finished);
		OSK_ASSERT(i >= 0);

		slot = &kbdev->jm_slots[i];
		osk_spinlock_irq_lock(&slot->lock);

		do {
			int nr_done;
			u32 active;
			u32 completion_code = BASE_JD_EVENT_DONE; /* assume OK */
			u64 job_tail = 0;

#ifdef BASE_HW_ISSUE_5713
			if (failed & (1u << i) || slot->submission_blocked_for_soft_stop)
#else
			if (failed & (1u << i))
#endif
			{
				/* read out the job slot status code if the job slot reported failure */
				completion_code = kbase_reg_read(kbdev, JOB_SLOT_REG(i, JSn_STATUS), NULL);
				
				switch(completion_code)
				{
					case BASE_JD_EVENT_NOT_STARTED:
						/* NOT_STARTED means that the job slot is idle - so the previous job must have completed */
						completion_code = BASE_JD_EVENT_DONE;
						break;
					case BASE_JD_EVENT_STOPPED:
						/* Soft-stopped job - read the value of JS<n>_TAIL so that the job chain can be resumed */
						job_tail = (u64)kbase_reg_read(kbdev, JOB_SLOT_REG(i, JSn_TAIL_LO), NULL) |
						           ((u64)kbase_reg_read(kbdev, JOB_SLOT_REG(i, JSn_TAIL_HI), NULL) << 32);
						break;
#ifdef BASE_HW_ISSUE_5713
					case BASE_JD_EVENT_ACTIVE:
						if (slot->submission_blocked_for_soft_stop)
						{
							/* We're still waiting for the job to soft-stop, but a previous job has completed */
							completion_code = BASE_JD_EVENT_DONE;
							break;
						}
#endif
					default:
						OSK_PRINT_WARN(OSK_BASE_JD, "error detected from slot %d, job status 0x%08x",
						               i, completion_code);
				}
			}

			kbase_reg_write(kbdev, JOB_CONTROL_REG(JOB_IRQ_CLEAR), done & ((1 << i) | (1 << (i + 16))), NULL);
			active = kbase_reg_read(kbdev, JOB_CONTROL_REG(JOB_IRQ_JS_STATE), NULL);

			if (((active >> i) & 1) == 0)
			{
				/* Work around a hardware issue. If a job fails after we last read RAWSTAT but
				 * before our call to IRQ_CLEAR then it will not be present in JS_STATE */
				u32 rawstat = kbase_reg_read(kbdev, JOB_CONTROL_REG(JOB_IRQ_RAWSTAT), NULL);

				if ((rawstat >> (i+16)) & 1)
				{
					/* There is a failed job that we've missed - add it back to active */
					active |= (1u << i);
				}
			}

			OSK_PRINT_INFO(OSK_BASE_JM, "Job ended with status 0x%08X\n", completion_code);

			nr_done = kbasep_jm_nr_jobs_submitted( slot );
			nr_done -= (active >> i) & 1;
			nr_done -= (active >> (i + 16)) & 1;

			if (nr_done <= 0)
			{
				OSK_PRINT_INFO(OSK_BASE_JM,
				               "Spurious interrupt on slot %d",
				               i);
				goto spurious;
			}

			count += nr_done;

			while (nr_done) {
				if (nr_done == 1)
				{
					kbase_job_done_slot(kbdev, i, completion_code, job_tail, &end_timestamp);
				}
				else
				{
					/* More than one job has completed. Since this is not the last job being reported this time it
					 * must have passed. This is because the hardware will not allow further jobs in a job slot to
					 * complete until the faile job is cleared from the IRQ status.
					 */
					kbase_job_done_slot(kbdev, i, BASE_JD_EVENT_DONE, 0, &end_timestamp);
				}
				nr_done--;
			}

spurious:
			done = kbase_reg_read(kbdev, JOB_CONTROL_REG(JOB_IRQ_RAWSTAT), NULL);
			
			failed = done >> 16;
			finished = (done & 0xFFFF) | failed;
		} while (finished & (1 << i));

		kbasep_job_slot_update_head_start_timestamp( slot, end_timestamp );


		osk_spinlock_irq_unlock(&slot->lock);
	}

	beenthere("irq efficiency = %d", count);
}

/**
 * Remove all jobs queued on a given job slot and belonging to a given
 * context. Job slot lock must be held so we don't race against the
 * interrupt.
 */
static void kbase_zap_context_slot(kbase_context *kctx, kbase_jm_slot *slot, int js)
{
	kbase_device *kbdev = kctx->kbdev;
	kbase_jd_atom *katom;
	u8 i;
	u8 jobs_submitted;

	jobs_submitted = kbasep_jm_nr_jobs_submitted( slot );

	/* Start with whatever we've already queued */
	for (i = 0; i < jobs_submitted; i++)
	{
		katom = kbasep_jm_peek_idx_submit_slot( slot, i );

		if (katom->kctx != kctx)
			continue;

		katom->event.event_code = BASE_JD_EVENT_JOB_CANCELLED;
		
		if (i+1 == jobs_submitted) {
			/* The last job in the slot, check if there is a job in the next register */
			if (kbase_reg_read(kbdev, JOB_SLOT_REG(js, JSn_COMMAND_NEXT), NULL) == 0)
			{
				beenthere("hard-stopping job on slot %d", js);
				/* There is no next job - so the one we want to cancel is in the current register */
				kbase_job_slot_hardstop(kctx->kbdev, js);
			}
			else
			{
				/* The job is in the next registers */
				beenthere("clearing job from next registers on slot %d", js);
				kbase_reg_write(kbdev, JOB_SLOT_REG(js, JSn_COMMAND_NEXT), JSn_COMMAND_NOP, NULL);

				/* Check to see if we did remove a job from the next registers */
				if (kbase_reg_read(kbdev, JOB_SLOT_REG(js, JSn_HEAD_NEXT_LO), NULL) != 0 ||
				    kbase_reg_read(kbdev, JOB_SLOT_REG(js, JSn_HEAD_NEXT_HI), NULL) != 0)
				{
					/* The job was successfully cleared from the next registers, requeue it */
					kbase_jd_atom *dequeued_katom = kbasep_jm_dequeue_tail_submit_slot( slot );
					OSK_ASSERT(dequeued_katom == katom);
					jobs_submitted --;

					dequeued_katom->event.event_code = BASE_JD_EVENT_REMOVED_FROM_NEXT;
					kbase_jd_done(dequeued_katom);
				}
				else
				{
					/* The job transitioned into the current registers before we managed to evict it,
					 * in this case we fall back to hard-stopping the job */
					beenthere("missed job in next register, hard-stopping slot %d", js);
					kbase_job_slot_hardstop(kctx->kbdev, js);
				}
			}
		} else if (i+2 == jobs_submitted) {
			/* There's a job after this one, check to see if that job is in the next registers */
			if (kbase_reg_read(kbdev, JOB_SLOT_REG(js, JSn_COMMAND_NEXT), NULL) != 0)
			{
				/* It is - we should remove that job and hard-stop the slot */
				beenthere("clearing job from next registers on slot %d", js);
				kbase_reg_write(kbdev, JOB_SLOT_REG(js, JSn_COMMAND_NEXT), JSn_COMMAND_NOP, NULL);

				/* Check to see if we did remove a job from the next registers */
				if (kbase_reg_read(kbdev, JOB_SLOT_REG(js, JSn_HEAD_NEXT_LO), NULL) != 0 ||
				    kbase_reg_read(kbdev, JOB_SLOT_REG(js, JSn_HEAD_NEXT_HI), NULL) != 0)
				{
					/* We did remove a job from the next registers, requeue it */
					kbase_jd_atom *dequeued_katom = kbasep_jm_dequeue_tail_submit_slot( slot );
					jobs_submitted --;

					dequeued_katom->event.event_code = BASE_JD_EVENT_REMOVED_FROM_NEXT;
					kbase_jd_done(dequeued_katom);
				}
				else
				{
					/* We missed the job, that means the job we're interested in left the hardware before
					 * we managed to do anything, so we can proceed to the next job */
					continue;
				}

				/* Next is now free, so we can hard-stop the slot */
				beenthere("hard-stopped slot %d (there was a job in next which was successfully cleared)\n", js);
				kbase_job_slot_hardstop(kctx->kbdev, js);
			}
			/* If there was no job in the next registers, then the job we were
			 * interested in has finished, so we need not take any action
			 */
		}
	}
}

void kbase_job_kill_jobs_from_context(kbase_context *kctx)
{
	kbase_device *kbdev;
	int i;

	OSK_ASSERT( kctx != NULL );
	kbdev = kctx->kbdev;
	OSK_ASSERT( kbdev != NULL );

	/* Cancel any remaining running jobs for this kctx  */
	for (i = 0; i < kbdev->nr_job_slots; i++)
	{
		kbase_jm_slot *slot = &kbdev->jm_slots[i];
		osk_spinlock_irq_lock(&slot->lock);
		kbase_zap_context_slot(kctx, slot, i);
		osk_spinlock_irq_unlock(&slot->lock);
	}
}

void kbase_job_zap_context(kbase_context *kctx)
{
	kbase_device *kbdev;
	kbasep_js_device_data *js_devdata;
	kbasep_js_kctx_info *js_kctx_info;
	int i;
	mali_bool evict_success;
	mali_bool need_to_try_schedule_context = MALI_FALSE;

	OSK_ASSERT( kctx != NULL );
	kbdev = kctx->kbdev;
	OSK_ASSERT( kbdev != NULL );
	js_devdata = &kbdev->js_data;
	js_kctx_info = &kctx->jctx.sched_info;


	/*
	 * Critical assumption: No more submission is possible outside of the
	 * workqueue. This is because the OS *must* prevent U/K calls (IOCTLs)
	 * whilst the kbase_context is terminating.
	 */


	/* First, atomically do the following:
	 * - mark the context as dying
	 * - try to evict it from the policy queue */

	osk_mutex_lock( &js_kctx_info->ctx.jsctx_lock );
	js_kctx_info->ctx.is_dying = MALI_TRUE;

	OSK_PRINT_INFO(OSK_BASE_JM, "Zap: Try Evict Ctx %p", kctx );
	osk_mutex_lock( &js_devdata->queue_lock );
	evict_success = kbasep_js_policy_try_evict_ctx( &js_devdata->policy, kctx );
	osk_mutex_unlock( &js_devdata->queue_lock );

	osk_mutex_unlock( &js_kctx_info->ctx.jsctx_lock );

	/* locks must be dropped by this point, to prevent deadlock on flush */
	OSK_PRINT_INFO(OSK_BASE_JM, "Zap: Flush Workqueue Ctx %p", kctx );
	kbase_jd_flush_workqueues( kctx );

	/*
	 * At this point we know that:
	 * - If eviction succeeded, it was in the policy queue, but now no longer is
	 * - If eviction failed, then it wasn't in the policy queue. It is one of the following:
	 *  - a. it didn't have any jobs, and so is not in the Policy Queue or the
	 * Run Pool (no work required)
	 *  - b. it was in the process of a scheduling transaction - but this can only
	 * happen as a result of the work-queue. Two options:
	 *   - i. it is now scheduled by the time of the flush - case e.
	 *   - ii. it is evicted from the Run Pool due to having to roll-back a transaction
	 *  - c. it is about to be scheduled out.
	 *   - In this case, we've marked it as dying, so the schedule-out code
	 * marks all jobs for killing, evicts it from the Run Pool, and does *not*
	 * place it back on the Policy Queue. The workqueue flush ensures this has
	 * completed
	 *  - d. it is scheduled, and may or may not be running jobs
	 */

	osk_mutex_lock( &js_kctx_info->ctx.jsctx_lock );
	if ( evict_success != MALI_FALSE || js_kctx_info->ctx.is_scheduled == MALI_FALSE )
	{
		/* The following events require us to kill off remaining jobs and
		 * update PM book-keeping:
		 * - we evicted it correctly (it must have jobs to be in the Policy Queue)
		 * - Case b-ii: scheduling transaction was partially rolled-back (again, it must've had jobs)
		 *
		 * These events need no action:
		 * - Case a: it didn't have any jobs, and was never in the Queue
		 * - Case c: scheduled out and killing of all jobs completed on the work-queue (it's not in the Run Pool)
		 */

		OSK_PRINT_INFO(OSK_BASE_JM, "Zap: Ctx %p evict_success=%d, scheduled=%d", kctx, evict_success, js_kctx_info->ctx.is_scheduled );

		if ( js_kctx_info->ctx.nr_jobs > 0 )
		{
			OSK_PRINT_INFO(OSK_BASE_JM, "Zap: Ctx %p kill remaining jobs and idle ctx", kctx );
			/* Notify PM that a context has gone idle */
			kbase_pm_context_idle(kbdev);

			/* Kill all the jobs present (call kbase_jd_cancel on all jobs) */
			kbasep_js_policy_kill_all_ctx_jobs( &js_devdata->policy, kctx );
		}
	}
	else
	{
		u32 nr_running_jobs_upon_disable;
		/* Didn't evict, but it is scheduled - it's in the Run Pool:
		 * Cases d and b(i) */
		OSK_PRINT_INFO(OSK_BASE_JM, "Zap: Ctx %p is in RunPool", kctx );

		/* Disable the ctx from submitting any more jobs */
		osk_spinlock_irq_lock( &js_devdata->runpool_lock );
		kbasep_js_clear_submit_allowed( js_devdata, kctx );
		nr_running_jobs_upon_disable = js_kctx_info->runpool.nr_running_jobs;
		osk_spinlock_irq_unlock( &js_devdata->runpool_lock );

		/* nr_running_jobs_upon_disable can't change - we've stopped any
		 * more being submitted, and hold the jctx_lock to prevent it being
		 * decremented */

		if ( nr_running_jobs_upon_disable == 0 )
		{
			OSK_PRINT_INFO(OSK_BASE_JM, "Zap: Ctx %p Deschedule Now", kctx );
			/* Deschedule the context immmediately - because there's no
			 * jobs running to cause that to happen in the workqueue */
			need_to_try_schedule_context |= kbasep_js_check_and_deschedule_ctx( kbdev, kctx );
			/* At this point, the context is evicted, and has marked all
			 * jobs for killing. No more work required. */
		}
		else
		{
			OSK_PRINT_INFO(OSK_BASE_JM, "Zap: Ctx %p Kill Running jobs and wait", kctx );
			/* Cancel any remaining running jobs for this kctx  */
			for (i = 0; i < kbdev->nr_job_slots; i++)
			{
				kbase_jm_slot *slot = &kbdev->jm_slots[i];
				osk_spinlock_irq_lock(&slot->lock);
				kbase_zap_context_slot(kctx, slot, i);
				osk_spinlock_irq_unlock(&slot->lock);
			}
			/* At this point, the context is waiting to finish jobs - when
			 * they all finish, it'll get scheduled out, and will kill all
			 * remaining jobs. And so, we must *not* attempt to kill them
			 * twice, so no more work is required */
		}
	}

	osk_mutex_unlock( &js_kctx_info->ctx.jsctx_lock );

	/* Handle any queued actions */
	if ( need_to_try_schedule_context != MALI_FALSE )
	{
		kbasep_js_try_schedule_head_ctx( kbdev );
	}

	/* Cancelling work may still be on the ctx's idle_waitq - caller must wait on it regardless */
}

mali_error kbase_job_slot_init(kbase_device *kbdev)
{
	int i;
	osk_error osk_err;


	for (i = 0; i < kbdev->nr_job_slots; i++)
	{
		osk_err = osk_spinlock_irq_init(&kbdev->jm_slots[i].lock, OSK_LOCK_ORDER_JSLOT);
		if (OSK_ERR_NONE != osk_err)
		{
			int j;
			for (j = 0; j < i; j++)
			{
				osk_spinlock_irq_term(&kbdev->jm_slots[j].lock);
			}
			return MALI_ERROR_FUNCTION_FAILED;
		}
		kbasep_jm_init_submit_slot( &kbdev->jm_slots[i] );
	}
	return MALI_ERROR_NONE;
}

void kbase_job_slot_exit(kbase_device *kbdev)
{
	int i;

	for (i = 0; i < kbdev->nr_job_slots; i++)
	{
		osk_spinlock_irq_term(&kbdev->jm_slots[i].lock);
	}
}

/*
 * The job slot lock must be held when calling this function.
 * The job slot must not already be in the process of being soft-stopped.
 *
 * Where possible any job in the next register is evicted before the soft-stop.
 */
void kbase_job_slot_softstop(kbase_device *kbdev, int js)
{
	kbase_jm_slot *slot;

	slot = &kbdev->jm_slots[js];

	if ( kbasep_jm_nr_jobs_submitted( slot ) == 0)
	{
		/* No jobs submitted */
		return;
	}

	/* Clear the next register to prevent any job in the next registers from transitioning */
	kbase_reg_write(kbdev, JOB_SLOT_REG(js, JSn_COMMAND_NEXT), JSn_COMMAND_NOP, NULL);

	/* Check to see if we did remove a job from the next registers */
	if (kbase_reg_read(kbdev, JOB_SLOT_REG(js, JSn_HEAD_NEXT_LO), NULL) ||
	    kbase_reg_read(kbdev, JOB_SLOT_REG(js, JSn_HEAD_NEXT_HI), NULL))
	{
		/* There was a job in the next registers, requeue it */
		kbase_jd_atom *katom = kbasep_jm_dequeue_tail_submit_slot( slot );

		katom->event.event_code = BASE_JD_EVENT_REMOVED_FROM_NEXT;
		kbase_jd_done(katom);
	}

#ifdef BASE_HW_ISSUE_5713
	/* PRLAM-5713 means that we can't reliably identify a job that has been soft-stopped in the interrupt handler.
	 * To work around this we ensure that the slot is not submitted to again until we've dealt with the job that is
	 * soft-stopped.
	 */
	slot->submission_blocked_for_soft_stop = MALI_TRUE;
#endif

	/* The next register should now be empty */
	OSK_ASSERT(kbase_reg_read(kbdev, JOB_SLOT_REG(js, JSn_COMMAND_NEXT), NULL) == 0);

	kbase_reg_write(kbdev, JOB_SLOT_REG(js, JSn_COMMAND), JSn_COMMAND_SOFT_STOP, NULL);
}

/*
 * The job slot lock must be held when calling this function.
 * This function unconditionally hard-stops the slot, it is up to the caller to
 * ensure that the correct job is hard-stopped.
 */
void kbase_job_slot_hardstop(kbase_device *kbdev, int js)
{
	kbase_reg_write(kbdev, JOB_SLOT_REG(js, JSn_COMMAND), JSn_COMMAND_HARD_STOP, NULL);
}
