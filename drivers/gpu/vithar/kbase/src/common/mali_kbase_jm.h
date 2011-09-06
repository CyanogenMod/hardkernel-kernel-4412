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
 * @file mali_kbase_jm.h
 * Job Manager Low-level APIs.
 */

#ifndef _KBASE_JM_H_
#define _KBASE_JM_H_

/**
 * @addtogroup base_api
 * @{
 */

/**
 * @addtogroup base_kbase_api
 * @{
 */


/**
 * @addtogroup kbase_jm Job Manager Low-level APIs
 * @{
 *
 */

static INLINE int kbasep_jm_is_js_free(kbase_device *kbdev, int js, kbase_context *kctx)
{
	OSK_ASSERT( kbdev != NULL );
	OSK_ASSERT( 0 <= js && js < kbdev->nr_job_slots  );

#ifndef BASE_HW_ISSUE_5713
	return !kbase_reg_read(kbdev, JOB_SLOT_REG(js, JSn_COMMAND_NEXT), kctx);
#else /* BASE_HW_ISSUE_5713 */
	/* On r0p0 we need to ensure that there is no job in the current registers as well
	 * because it is unsafe to soft-stop a slot when a second job is queued */
	return !kbase_reg_read(kbdev, JOB_SLOT_REG(js, JSn_COMMAND_NEXT), kctx) &&
	       !kbase_reg_read(kbdev, JOB_SLOT_REG(js, JSn_COMMAND), kctx);
#endif /* BASE_HW_ISSUE_5713 */
}

/**
 * This checks that:
 * - there is enough space in the GPU's buffers (JSn_NEXT and JSn_HEAD registers) to accomodate the job.
 * - there is enough space to track the job in a our Submit Slots. Note that we have to maintain space to
 *   requeue one job in case the next registers on the hardware need to be cleared.
 * - the slot is not blocked (due to PRLAM-5713 workaround)
 */
static INLINE mali_bool kbasep_jm_is_submit_slots_free(kbase_device *kbdev, int js, kbase_context *kctx)
{
	OSK_ASSERT( kbdev != NULL );
	OSK_ASSERT( 0 <= js && js < kbdev->nr_job_slots  );
	
	if (osk_atomic_get(&kbdev->reset_gpu) != 0)
	{
		/* The GPU is being reset - so prevent submission */
		return MALI_FALSE;
	}

#ifdef BASE_HW_ISSUE_5713
	return (mali_bool)( !kbdev->jm_slots[js].submission_blocked_for_soft_stop
	                    && kbasep_jm_is_js_free(kbdev, js, kctx)
	                    && kbdev->jm_slots[js].submitted_nr < (BASE_JM_SUBMIT_SLOTS-2) );
#else
	return (mali_bool)( kbasep_jm_is_js_free(kbdev, js, kctx)
	                    && kbdev->jm_slots[js].submitted_nr < (BASE_JM_SUBMIT_SLOTS-2) );
#endif
}

/**
 * Initialize a submit slot
 */
static INLINE void kbasep_jm_init_submit_slot(  kbase_jm_slot *slot )
{
	slot->submitted_nr = 0;
	slot->submitted_head = 0;
#ifdef BASE_HW_ISSUE_5713
	slot->submission_blocked_for_soft_stop = MALI_FALSE;
#endif
}

/**
 * Find the atom at the idx'th element in the queue without removing it, starting at the head with idx==0.
 */
static INLINE kbase_jd_atom* kbasep_jm_peek_idx_submit_slot( kbase_jm_slot *slot, u8 idx )
{
	u8 pos;
	kbase_jd_atom *katom;

	OSK_ASSERT( idx < BASE_JM_SUBMIT_SLOTS );

	pos = (slot->submitted_head + idx) & BASE_JM_SUBMIT_SLOTS_MASK;
	katom = slot->submitted[pos];

	return katom;
}

/**
 * Pop front of the submitted
 */
static INLINE kbase_jd_atom* kbasep_jm_dequeue_submit_slot( kbase_jm_slot *slot )
{
	u8 pos;
	kbase_jd_atom *katom;

	pos = slot->submitted_head & BASE_JM_SUBMIT_SLOTS_MASK;
	katom = slot->submitted[pos];
	slot->submitted[pos] = NULL; /* Just to catch bugs... */
	OSK_ASSERT(katom);

	/* rotate the buffers */
	slot->submitted_head = (slot->submitted_head + 1) & BASE_JM_SUBMIT_SLOTS_MASK;
	slot->submitted_nr--;

	OSK_PRINT_INFO( OSK_BASE_JM, "katom %p new head %u",
	                (void *)katom, (unsigned int)slot->submitted_head);

	return katom;
}

/* Pop back of the submitted queue (unsubmit a job)
 */
static INLINE kbase_jd_atom *kbasep_jm_dequeue_tail_submit_slot( kbase_jm_slot *slot )
{
	u8 pos;

	slot->submitted_nr--;

	pos = (slot->submitted_head + slot->submitted_nr) & BASE_JM_SUBMIT_SLOTS_MASK;

	return slot->submitted[pos];
}

static INLINE u8 kbasep_jm_nr_jobs_submitted( kbase_jm_slot *slot )
{
	return slot->submitted_nr;
}


/**
 * Push back of the submitted
 */
static INLINE void kbasep_jm_enqueue_submit_slot( kbase_jm_slot *slot, kbase_jd_atom *katom )
{
	u8 nr;
	u8 pos;
	nr = slot->submitted_nr++;
	OSK_ASSERT(nr < BASE_JM_SUBMIT_SLOTS);
	
	pos = (slot->submitted_head + nr) & BASE_JM_SUBMIT_SLOTS_MASK;
	slot->submitted[pos] = katom;
}

/**
 * @brief Submit a job to a certain job-slot
 *
 * The caller must check kbasep_jm_is_submit_slots_free() != MALI_FALSE before calling this.
 */
void kbase_job_submit_nolock(kbase_device *kbdev, kbase_jd_atom *katom, int js);

/**
 * @brief Complete the head job on a particular job-slot
 */
void kbase_job_done_slot(kbase_device *kbdev, int s, u32 completion_code, u64 job_tail, kbasep_js_tick *end_timestamp);

/** @} */ /* end group kbase_jm */
/** @} */ /* end group base_kbase_api */
/** @} */ /* end group base_api */

#endif /* _KBASE_JM_H_ */
