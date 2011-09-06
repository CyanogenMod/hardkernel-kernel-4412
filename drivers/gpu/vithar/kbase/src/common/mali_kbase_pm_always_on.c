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
 * @file mali_kbase_pm_always_on.c
 * "Always on" power management policy
 */

#include <osk/mali_osk.h>
#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_pm.h>

/** The states that the always_on policy can enter.
 *
 * The diagram below should the states that the always_on policy can enter and the transitions that can occur between 
 * the states:
 * 
 * @dot
 * digraph always_on_states {
 *      node [fontsize=10];
 *      edge [fontsize=10];
 *
 *      POWERING_UP     [label="STATE_POWERING_UP"      URL="\ref always_on_state.STATE_POWERING_UP"];
 *      POWERING_DOWN   [label="STATE_POWERING_DOWN"    URL="\ref always_on_state.STATE_POWERING_DOWN"];
 *      POWERED_UP      [label="STATE_POWERED_UP"       URL="\ref always_on_state.STATE_POWERED_UP"];
 *      POWERED_DOWN    [label="STATE_POWERED_DOWN"     URL="\ref always_on_state.STATE_POWERED_DOWN"];
 *      CHANGING_POLICY [label="STATE_CHANGING_POLICY"  URL="\ref always_on_state.STATE_CHANGING_POLICY"];
 *
 *      init            [label="init"                   URL="\ref KBASE_PM_EVENT_INIT"];
 *      change_policy   [label="change_policy"          URL="\ref kbase_pm_change_policy"];
 *
 *      init -> POWERING_UP [ label = "Policy init" ];
 *
 *      POWERING_UP -> POWERED_UP [label = "Power state change" URL="\ref KBASE_PM_EVENT_STATE_CHANGED"];
 *      POWERING_DOWN -> POWERED_DOWN [label = "Power state change" URL="\ref KBASE_PM_EVENT_STATE_CHANGED"];
 *      CHANGING_POLICY -> change_policy [label = "Power state change" URL="\ref KBASE_PM_EVENT_STATE_CHANGED"];
 *
 *      POWERED_UP -> POWERING_DOWN [label = "Suspend" URL="\ref KBASE_PM_EVENT_SUSPEND"];
 *
 *      POWERED_DOWN -> POWERING_UP [label = "Resume" URL="\ref KBASE_PM_EVENT_RESUME"];
 *
 *      POWERING_UP -> CHANGING_POLICY [label = "Change policy" URL="\ref KBASE_PM_EVENT_CHANGE_POLICY"];
 *      POWERING_DOWN -> CHANGING_POLICY [label = "Change policy" URL="\ref KBASE_PM_EVENT_CHANGE_POLICY"];
 *      POWERED_UP -> change_policy [label = "Change policy" URL="\ref KBASE_PM_EVENT_CHANGE_POLICY"];
 *      POWERED_DOWN -> change_policy [label = "Change policy" URL="\ref KBASE_PM_EVENT_CHANGE_POLICY"];
 * }
 * @enddot
 */
typedef enum always_on_state
{
	STATE_POWERING_UP,      /**< The GPU is powering up */
	STATE_POWERING_DOWN,    /**< The GPU is powering down */
	STATE_POWERED_UP,       /**< The GPU is powered up and jobs can execute */
	STATE_POWERED_DOWN,     /**< The GPU is powered down and the system can suspend */
	STATE_CHANGING_POLICY   /**< The power policy is about to change */
} always_on_state;

/** Private structure for policy instance data.
 *
 * This contains data that is private to the particular power policy that is active.
 */
struct kbase_pm_policy_data
{
	always_on_state state;  /**< The current state of the policy */
};

/** Function to handle a GPU state change for the always_on power policy.
 *
 * This function is called whenever the GPU has transitioned to another state. It first checks that the transition is 
 * complete and then moves the state machine to the next state.
 *
 * @param kbdev     The kbase device structure for the device
 */
static void always_on_state_changed(kbase_device *kbdev)
{
	kbase_pm_policy_data *data = kbdev->pm.policy_data;

	switch(data->state)
	{
	case STATE_POWERING_UP:
		if (kbase_pm_get_pwr_active(kbdev))
		{
			/* Cores still transitioning */
			return;
		}
		/* All cores have transitioned, inform the OS */
		kbase_pm_power_up_done(kbdev);
		data->state = STATE_POWERED_UP;

		break;
	case STATE_POWERING_DOWN:
		if (kbase_pm_get_pwr_active(kbdev))
		{
			/* Cores still transitioning */
			return;
		}
		/* All cores have transitioned, turn the clock and interrupts off */
		kbase_pm_disable_interrupts(kbdev);
		kbase_pm_clock_off(kbdev);

		/* Inform the OS */
		kbase_pm_power_down_done(kbdev);

		data->state = STATE_POWERED_DOWN;

		break;
	case STATE_CHANGING_POLICY:
		if (kbase_pm_get_pwr_active(kbdev))
		{
			/* Cores still transitioning */
			return;
		}
		/* All cores have transitioned, inform the system we can change policy*/
		kbase_pm_change_policy(kbdev);

		break;
	default:
		break;
	}
}

/** Function to handle the @ref KBASE_PM_EVENT_SUSPEND message for the always_on power policy.
 *
 * This function is called when a @ref KBASE_PM_EVENT_SUSPEND message is received. It instructs the GPU to turn off 
 * all cores.
 *
 * @param kbdev     The kbase device structure for the device
 */
static void always_on_suspend(kbase_device *kbdev)
{
	u32 lo, hi;

	/* Inform the system that the transition has started */
	kbase_pm_power_transitioning(kbdev);

	/* Turn the cores off */
	kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_L3, &lo, &hi);
	kbase_pm_invoke_power_down(kbdev, KBASE_PM_CORE_L3, lo, hi);

	kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_L2, &lo, &hi);
	kbase_pm_invoke_power_down(kbdev, KBASE_PM_CORE_L2, lo, hi);

	kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_SHADER, &lo, &hi);
	kbase_pm_invoke_power_down(kbdev, KBASE_PM_CORE_SHADER, lo, hi);

	kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_TILER, &lo, &hi);
	kbase_pm_invoke_power_down(kbdev, KBASE_PM_CORE_TILER, lo, hi);

	kbdev->pm.policy_data->state = STATE_POWERING_DOWN;

	/* Ensure that the OS is informed even if we didn't do anything */
	always_on_state_changed(kbdev);
}

/** Function to handle the @ref KBASE_PM_EVENT_RESUME message for the always_on power policy.
 *
 * This function is called when a @ref KBASE_PM_EVENT_RESUME message is received. It instructs the GPU to turn on all 
 * the cores.
 *
 * @param kbdev     The kbase device structure for the device
 */
static void always_on_resume(kbase_device *kbdev)
{
	u32 lo, hi;

	/* Inform the system that the transition has started */
	kbase_pm_power_transitioning(kbdev);

	/* Turn the clock on */
	kbase_pm_clock_on(kbdev);
	/* Enable interrupts */
	kbase_pm_enable_interrupts(kbdev);

	/* Turn the cores on */
	kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_L3, &lo, &hi);
	kbase_pm_invoke_power_up(kbdev, KBASE_PM_CORE_L3, lo, hi);

	kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_L2, &lo, &hi);
	kbase_pm_invoke_power_up(kbdev, KBASE_PM_CORE_L2, lo, hi);

	kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_SHADER, &lo, &hi);
	kbase_pm_invoke_power_up(kbdev, KBASE_PM_CORE_SHADER, lo, hi);

	kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_TILER, &lo, &hi);
	kbase_pm_invoke_power_up(kbdev, KBASE_PM_CORE_TILER, lo, hi);

	kbdev->pm.policy_data->state = STATE_POWERING_UP;

	/* Ensure that the OS is informed even if we didn't do anything */
	always_on_state_changed(kbdev);
}

/** The event callback function for the always_on power policy.
 *
 * This function is called to handle the events for the power policy. It calls the relevant handler function depending 
 * on the type of the event.
 *
 * @param kbdev     The kbase device structure for the device
 * @param event     The event that should be processed
 */
static void always_on_event(kbase_device *kbdev, kbase_pm_event event)
{
	kbase_pm_policy_data *data = kbdev->pm.policy_data;

	switch(event)
	{
	case KBASE_PM_EVENT_SUSPEND:
		always_on_suspend(kbdev);
		break;
	case KBASE_PM_EVENT_INIT: /* Init is the same as resume for this policy */
	case KBASE_PM_EVENT_RESUME:
		always_on_resume(kbdev);
		break;
	case KBASE_PM_EVENT_STATE_CHANGED:
		always_on_state_changed(kbdev);
		break;
	case KBASE_PM_EVENT_CHANGE_POLICY:
		if (data->state == STATE_POWERED_UP || data->state == STATE_POWERED_DOWN)
		{
			kbase_pm_change_policy(kbdev);
		}
		else
		{
			data->state = STATE_CHANGING_POLICY;
		}
		break;
	case KBASE_PM_EVENT_GPU_ACTIVE:
	case KBASE_PM_EVENT_GPU_IDLE:
		/* Not used - the GPU is always kept on */
		break;
	}
}

/** Initialize the always_on power policy
 *
 * This sets up the private @ref kbase_pm_device_data.policy_data field of the device for use with the always_on power 
 * policy.
 *
 * @param kbdev     The kbase device structure for the device
 */
static void always_on_init(kbase_device *kbdev)
{
	kbase_pm_policy_data *data;
#ifdef MALI_KBASE_USERSPACE
	BDBG_SET_FAIL_OFF();
#endif	
	data = osk_malloc(sizeof *data);
	OSK_ASSERT(data);

	data->state = STATE_POWERING_UP;

	kbdev->pm.policy_data = data;
#ifdef MALI_KBASE_USERSPACE
	BDBG_SET_FAIL_ON();
#endif	
}

/** Terminate the always_on power policy
 *
 * This frees the resources that were allocated by @ref always_on_init.
 *
 * @param kbdev     The kbase device structure for the device
 */
static void always_on_term(kbase_device *kbdev)
{
	osk_free(kbdev->pm.policy_data);
}

/** The @ref kbase_pm_policy structure for the always_on power policy
 *
 * This is the static structure that defines the always_on power policy's callback and name.
 */
static const kbase_pm_policy ops =
{
	"always_on",                /* name */
	always_on_init,             /* init */
	always_on_term,             /* term */
	always_on_event,            /* event */
};

/** Get the always on policy @ref kbase_pm_policy structure.
 *
 * This returns the static instance of @ref kbase_pm_policy for the 'always_on' power policy.
 *
 * @return The static instance of @ref kbase_pm_policy for the always_on power policy
 */
const kbase_pm_policy *kbase_pm_always_on_get_policy(void)
{
	return &ops;
}
