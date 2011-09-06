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
 * @file mali_kbase_pm.c
 * Base kernel power management APIs
 */

#include <osk/mali_osk.h>
#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_midg_regmap.h>

#include <kbase/src/common/mali_kbase_pm.h>

/* Functions that return the policy operation structures */
const kbase_pm_policy *kbase_pm_always_on_get_policy(void);
const kbase_pm_policy *kbase_pm_demand_get_policy(void);

/** List of functions that return the power policy structures */
static const kbase_pm_policy*(*policy_get_functions[])(void) =
{
	kbase_pm_always_on_get_policy,
	kbase_pm_demand_get_policy
};

/** The number of policies available in the system.
 * This is devired from the number of functions listed in policy_get_functions.
 */
#define POLICY_COUNT (sizeof(policy_get_functions)/sizeof(*policy_get_functions))

/** A list of the power policies available in the system */
static const kbase_pm_policy *policy_list[POLICY_COUNT];

mali_error kbase_pm_init(kbase_device *kbdev)
{
	mali_error ret = MALI_ERROR_NONE;
	int i;
	u32 lo, hi;
	osk_error osk_err;

	/* Collect a list of the available policies */
	for(i = 0; i < POLICY_COUNT; i++)
	{
		policy_list[i] = policy_get_functions[i]();
	}

	osk_err = osk_waitq_init(&kbdev->pm.power_up_waitqueue);
	if (OSK_ERR_NONE != osk_err)
	{
		return MALI_ERROR_FUNCTION_FAILED;
	}

	osk_err = osk_waitq_init(&kbdev->pm.power_down_waitqueue);
	if (OSK_ERR_NONE != osk_err)
	{
		osk_waitq_term(&kbdev->pm.power_up_waitqueue);
		return MALI_ERROR_FUNCTION_FAILED;
	}

	osk_err = osk_workq_init(&kbdev->pm.workqueue, "kbase_pm", OSK_WORKQ_NON_REENTRANT);
	if (OSK_ERR_NONE != osk_err)
	{
		osk_waitq_term(&kbdev->pm.power_down_waitqueue);
		osk_waitq_term(&kbdev->pm.power_up_waitqueue);
		return MALI_ERROR_FUNCTION_FAILED;
	}

	ret = kbase_pm_init_hw(kbdev);
	if (ret != MALI_ERROR_NONE)
	{
		osk_workq_term(&kbdev->pm.workqueue);
		osk_waitq_term(&kbdev->pm.power_down_waitqueue);
		osk_waitq_term(&kbdev->pm.power_up_waitqueue);
		return ret;
	}

	kbase_pm_power_transitioning(kbdev);

	kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_SHADER, &lo, &hi);
	kbdev->shader_present_bitmap = (((u64)hi) << 32) | lo;

	/* Pretend the GPU is active to prevent a power policy turning the GPU cores off */
	osk_atomic_set(&kbdev->pm.active_count, 1);

	osk_atomic_set(&kbdev->pm.pending_events, 0);

	osk_atomic_set(&kbdev->pm.work_active, 0);

	kbdev->pm.new_policy = NULL;
	kbdev->pm.current_policy = policy_list[0];
	kbdev->pm.current_policy->init(kbdev);

	kbase_pm_send_event(kbdev, KBASE_PM_EVENT_INIT);

	/* Idle the GPU */
	kbase_pm_context_idle(kbdev);

	return ret;
}

void kbase_pm_power_transitioning(kbase_device *kbdev)
{
	/* Clear the wait queues that are used to detect successful power up or down */
	osk_waitq_clear(&kbdev->pm.power_up_waitqueue);
	osk_waitq_clear(&kbdev->pm.power_down_waitqueue);
}

void kbase_pm_power_up_done(kbase_device *kbdev)
{
	osk_waitq_set(&kbdev->pm.power_up_waitqueue);
}

void kbase_pm_reset_done(kbase_device *kbdev)
{
	if (kbdev->expecting_reset_completed_irq)
	{
		osk_waitq_set(&kbdev->pm.power_up_waitqueue);
		kbdev->expecting_reset_completed_irq = MALI_FALSE;
	}
	else
	{
		OSK_PRINT_WARN(OSK_BASE_PM, "spurious RESET_COMPLETED IRQ; see PRLAM-4887");
	}
}

void kbase_pm_power_down_done(kbase_device *kbdev)
{
	osk_waitq_set(&kbdev->pm.power_down_waitqueue);
}

void kbase_pm_context_active(kbase_device *kbdev)
{
	int c = osk_atomic_inc(&kbdev->pm.active_count);
	
	if (c == 1)
	{
		/* First context active */
		kbase_pm_send_event(kbdev, KBASE_PM_EVENT_GPU_ACTIVE);
		kbase_pm_wait_for_power_up(kbdev);
	}
}

void kbase_pm_context_idle(kbase_device *kbdev)
{
	int c = osk_atomic_dec(&kbdev->pm.active_count);

	OSK_ASSERT(c >= 0);
	
	if (c == 0)
	{
		/* Last context has gone idle */
		kbase_pm_send_event(kbdev, KBASE_PM_EVENT_GPU_IDLE);
	}
}

void kbase_pm_term(kbase_device *kbdev)
{
	/* Turn the GPU off */
	kbase_pm_send_event(kbdev, KBASE_PM_EVENT_SUSPEND);
	/* Wait for the policy to acknowledge */
	kbase_pm_wait_for_power_down(kbdev);

	/* Destroy the workqueue - this ensures that all messages have been processed */
	osk_workq_term(&kbdev->pm.workqueue);

	/* Free any resources the policy allocated */
	kbdev->pm.current_policy->term(kbdev);

	/* Free the wait queues */
	osk_waitq_term(&kbdev->pm.power_up_waitqueue);
	osk_waitq_term(&kbdev->pm.power_down_waitqueue);
}

void kbase_pm_wait_for_power_up(kbase_device *kbdev)
{
	osk_waitq_wait(&kbdev->pm.power_up_waitqueue);
}

void kbase_pm_wait_for_power_down(kbase_device *kbdev)
{
	osk_waitq_wait(&kbdev->pm.power_down_waitqueue);
}

int kbase_pm_list_policies(const kbase_pm_policy ***list)
{
	if (!list)
		return POLICY_COUNT;

	*list = policy_list;

	return POLICY_COUNT;
}

const kbase_pm_policy *kbase_pm_get_policy(kbase_device *kbdev)
{
	return kbdev->pm.current_policy;
}

void kbase_pm_set_policy(kbase_device *kbdev, const kbase_pm_policy *new_policy)
{
	if (kbdev->pm.new_policy) {
		/* A policy change is already outstanding */
		return;
	}
	/* During a policy change we pretend the GPU is active */
	kbase_pm_context_active(kbdev);

	kbdev->pm.new_policy = new_policy;
	kbase_pm_send_event(kbdev, KBASE_PM_EVENT_CHANGE_POLICY);
}

void kbase_pm_change_policy(kbase_device *kbdev)
{
	kbdev->pm.current_policy->term(kbdev);
	kbdev->pm.current_policy = kbdev->pm.new_policy;
	kbdev->pm.current_policy->init(kbdev);
	kbase_pm_send_event(kbdev, KBASE_PM_EVENT_INIT);
	
	/* Now the policy change is finished, we release our fake context active reference */
	kbase_pm_context_idle(kbdev);
	
	kbdev->pm.new_policy = NULL;
}

/** Callback for the power management work queue.
 *
 * This function is called on the power management work queue and is responsible for delivering events to the active 
 * power policy. It manipulates the @ref kbase_pm_device_data.work_active field of @ref kbase_pm_device_data to track 
 * whether all events have been consumed.
 *
 * @param data      A pointer to the @c pm.work field of the @ref kbase_device struct
 */
static void kbase_pm_worker(osk_workq_work *data)
{
	kbase_device *kbdev = CONTAINER_OF(data, kbase_device, pm.work);
	int pending_events;
	int old_value;
	int i;

	do
	{
		osk_atomic_set(&kbdev->pm.work_active, 2);

		/* Atomically read and clear the bit mask */
		pending_events = osk_atomic_get(&kbdev->pm.pending_events);

		do
		{
			old_value = pending_events;
			pending_events = osk_atomic_compare_and_swap(&kbdev->pm.pending_events, old_value, 0);
		} while (old_value != pending_events);

		for(i = 0; pending_events; i++)
		{
			if (pending_events & (1 << i))
			{
				kbdev->pm.current_policy->event(kbdev, i);

				pending_events &= ~(1 << i);
			}
		}
		i = osk_atomic_compare_and_swap(&kbdev->pm.work_active, 2, 0);
	} while (i == 3);
}

/** Merge an event into the list of events to deliver.
 *
 * This ensures that if, for example, a GPU_IDLE is immediately followed by a GPU_ACTIVE then instead of delivering 
 * both messages to the policy the GPU_IDLE is simply discarded.
 *
 * In particular in the sequence GPU_IDLE, GPU_ACTIVE, GPU_IDLE the resultant message is GPU_IDLE and not (GPU_IDLE 
 * and GPU_ACTIVE).
 *
 * @param old_events    The bit mask of events that were previously pending
 * @param new_event     The event that should be merged into old_events
 *
 * @return The combination of old_events and the new event
 */
static int kbasep_pm_merge_event(int old_events, kbase_pm_event new_event)
{
	switch(new_event) {
		case KBASE_PM_EVENT_INIT:
		case KBASE_PM_EVENT_STATE_CHANGED:
		case KBASE_PM_EVENT_CHANGE_POLICY:
			/* Just merge these events into the list */
			return old_events | (1 << new_event);
		case KBASE_PM_EVENT_SUSPEND:
			if (old_events & (1 << KBASE_PM_EVENT_RESUME))
			{
				return old_events & ~(1 << KBASE_PM_EVENT_RESUME);
			}
			return old_events | (1 << new_event);
			break;
		case KBASE_PM_EVENT_RESUME:
			if (old_events & (1 << KBASE_PM_EVENT_SUSPEND))
			{
				return old_events & ~(1 << KBASE_PM_EVENT_SUSPEND);
			}
			return old_events | (1 << new_event);
			break;
		case KBASE_PM_EVENT_GPU_ACTIVE:
			if (old_events & (1 << KBASE_PM_EVENT_GPU_IDLE))
			{
				return old_events & ~(1 << KBASE_PM_EVENT_GPU_IDLE);
			}
			return old_events | (1 << new_event);
		case KBASE_PM_EVENT_GPU_IDLE:
			if (old_events & (1 << KBASE_PM_EVENT_GPU_ACTIVE))
			{
				return old_events & ~(1 << KBASE_PM_EVENT_GPU_ACTIVE);
			}
			return old_events | (1 << new_event);
	}

	/* Unrecognised event - this should never happen */
	OSK_ASSERT(0);
	
	return old_events | (1 << new_event);
}

void kbase_pm_send_event(kbase_device *kbdev, kbase_pm_event event)
{
	int pending_events;
	int work_active;
	int old_value, new_value;

	pending_events = osk_atomic_get(&kbdev->pm.pending_events);

	/* Atomically OR the new event into the pending_events bit mask */
	do
	{
		old_value = pending_events;
		new_value = kbasep_pm_merge_event(pending_events, event);
		if (old_value == new_value)
		{
			/* Event already pending */
			return;
		}
		pending_events = osk_atomic_compare_and_swap(&kbdev->pm.pending_events, old_value, new_value);
	} while (old_value != pending_events);

	work_active = osk_atomic_get(&kbdev->pm.work_active);
	do
	{
		old_value = work_active;
		switch(old_value)
		{
			case 0:
				/* Need to enqueue an event */
				new_value = 1;
				break;
			case 1:
				/* Event already queued */
				return;
			case 2:
				/* Event being processed, we need to ensure it checks for another event */
				new_value = 3;
				break;
			case 3:
				/* Event being processed, but another check for events is going to happen */
				return;
			default:
				OSK_ASSERT(0);
		}
		work_active = osk_atomic_compare_and_swap(&kbdev->pm.work_active, old_value, new_value);
	} while (old_value != work_active);

	if (old_value == 0)
	{
		osk_workq_submit(&kbdev->pm.workqueue, kbase_pm_worker, &kbdev->pm.work);
	}
}
