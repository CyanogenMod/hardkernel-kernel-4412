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
 * @file mali_kbase_pm_demand.c
 * A simple demand based power management policy
 */

#include <osk/mali_osk.h>
#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_pm.h>

/* Forward declaration */
static void demand_state_changed(kbase_device *kbdev);

/** The states that the demand policy can enter.
 *
 * The diagram below should the states that the demand policy can enter and the transitions that can occur between the 
 * states:
 * 
 * @dot
 * digraph demand_states {
 *      node [fontsize=10];
 *      edge [fontsize=10];
 *
 *      PREPARE_UP      [label="STATE_PREPARE_TO_POWER_UP" URL="\ref always_on_state.STATE_PREPARE_TO_POWER_UP"];
 *      PREPARE_DOWN    [label="STATE_PREPARE_TO_POWER_DOWN" URL="\ref always_on_state.STATE_PREPARE_TO_POWER_UP"];
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
 *      init -> PREPARE_UP [ label = "Policy init" ];
 *
 *      PREPARE_UP -> POWERING_UP [label = "Power state change" URL="\ref KBASE_PM_EVENT_STATE_CHANGED"];
 *      PREPARE_DOWN -> POWERING_DOWN [label = "Power state change" URL="\ref KBASE_PM_EVENT_STATE_CHANGED"];
 *
 *      POWERING_UP -> POWERED_UP [label = "Power state change" URL="\ref KBASE_PM_EVENT_STATE_CHANGED"];
 *      POWERING_DOWN -> POWERED_DOWN [label = "Power state change" URL="\ref KBASE_PM_EVENT_STATE_CHANGED"];
 *      CHANGING_POLICY -> change_policy [label = "Power state change" URL="\ref KBASE_PM_EVENT_STATE_CHANGED"];
 *
 *      POWERED_UP -> PREPARE_DOWN [label = "GPU Idle" URL="\ref KBASE_PM_EVENT_GPU_IDLE"];
 *      POWERING_UP -> PREPARE_DOWN [label = "GPU Idle" URL="\ref KBASE_PM_EVENT_GPU_IDLE"];
 *      PREPARE_UP -> PREPARE_DOWN [label = "GPU Idle" URL="\ref KBASE_PM_EVENT_GPU_IDLE"];
 *
 *      POWERED_DOWN -> PREPARE_UP [label = "GPU Active" URL="\ref KBASE_PM_EVENT_GPU_ACTIVE"];
 *      POWERING_DOWN -> PREPARE_UP [label = "GPU Active" URL="\ref KBASE_PM_EVENT_GPU_ACTIVE"];
 *      PREPARE_DOWN -> PREPARE_UP [label = "GPU Active" URL="\ref KBASE_PM_EVENT_GPU_ACTIVE"];
 *
 *      POWERING_UP -> CHANGING_POLICY [label = "Change policy" URL="\ref KBASE_PM_EVENT_CHANGE_POLICY"];
 *      POWERING_DOWN -> CHANGING_POLICY [label = "Change policy" URL="\ref KBASE_PM_EVENT_CHANGE_POLICY"];
 *      PREPARE_UP -> CHANGING_POLICY [label = "Change policy" URL="\ref KBASE_PM_EVENT_CHANGE_POLICY"];
 *      PREPARE_DOWN -> CHANGING_POLICY [label = "Change policy" URL="\ref KBASE_PM_EVENT_CHANGE_POLICY"];
 *      POWERED_UP -> change_policy [label = "Change policy" URL="\ref KBASE_PM_EVENT_CHANGE_POLICY"];
 *      POWERED_DOWN -> change_policy [label = "Change policy" URL="\ref KBASE_PM_EVENT_CHANGE_POLICY"];
 * }
 * @enddot
 *
 * NOTE: This policy does not currently handle the @ref KBASE_PM_EVENT_SUSPEND and @ref KBASE_PM_EVENT_RESUME 
 * messages. It just informs the system that the power transition is complete (even though nothing has happened).
 */
typedef enum demand_state
{
	STATE_PREPARE_TO_POWER_UP,  /**< The policy wishes to power up the GPU */
	STATE_POWERING_UP,          /**< The GPU is powering up */
	STATE_POWERED_UP,           /**< The GPU is powered up and jobs can execute */
	STATE_PREPARE_TO_POWER_DOWN,/**< The policy wishes to power down the GPU */
	STATE_POWERING_DOWN,        /**< The GPU is powering down */
	STATE_POWERED_DOWN,         /**< The GPU is powered down */
	STATE_CHANGING_POLICY       /**< The power policy is about to change */
} demand_state;

/** Private structure for policy instance data.
 *
 * This contains data that is private to the particular power policy that is active.
 */
struct kbase_pm_policy_data
{
	demand_state state;     /**< The current state of the policy */
};

/** Turns the cores on.
 *
 * This function turns all the cores of the GPU on.
 */
static void demand_power_up(kbase_device *kbdev)
{
	u32 lo, hi;

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
	demand_state_changed(kbdev);
}

/** Turn the cores off.
 *
 * This function turns all the cores of the GPU off.
 */
static void demand_power_down(kbase_device *kbdev)
{
	u32 lo, hi;

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
	demand_state_changed(kbdev);
}

/** Prepare to turn the cores off.
 *
 * This function moves the state machine to STATE_PREPARE_TO_POWER_DOWN.
 */
static void demand_prepare_power_down(kbase_device *kbdev)
{
	/* Inform the system that the transition has started */
	kbase_pm_power_transitioning(kbdev);
	
	kbdev->pm.policy_data->state = STATE_PREPARE_TO_POWER_DOWN;

	demand_state_changed(kbdev);
}

/** Prepare to turn the cores on.
 *
 * This function moves the state machine to STATE_PREPARE_TO_POWER_UP.
 */
static void demand_prepare_power_up(kbase_device *kbdev)
{
	/* Inform the system that the transition has started */
	kbase_pm_power_transitioning(kbdev);

	/* Turn the clock on */
	kbase_pm_clock_on(kbdev);
	/* Turn the interrupts on */
	kbase_pm_enable_interrupts(kbdev);

	kbdev->pm.policy_data->state = STATE_PREPARE_TO_POWER_UP;

	demand_state_changed(kbdev);
}

/** Function to handle a GPU state change for the demand power policy
 *
 * This function is called whenever the GPU has transitioned to another state. It first checks that the transition is 
 * complete and then moves the state machine to the next state.
 */
static void demand_state_changed(kbase_device *kbdev)
{
	kbase_pm_policy_data *data = kbdev->pm.policy_data;

	switch(data->state) {
		case STATE_PREPARE_TO_POWER_UP:
		case STATE_PREPARE_TO_POWER_DOWN:
		case STATE_CHANGING_POLICY:
		case STATE_POWERING_UP:
		case STATE_POWERING_DOWN:
			if (kbase_pm_get_pwr_active(kbdev)) {
				/* Cores are still transitioning - ignore the event */
				return;
			}
			break;
		default:
			/* Must not call kbase_pm_get_pwr_active here as the clock may be turned off */
			break;
	}

	switch(data->state)
	{
		case STATE_PREPARE_TO_POWER_UP:
			demand_power_up(kbdev);
			break;
		case STATE_PREPARE_TO_POWER_DOWN:
			demand_power_down(kbdev);
			break;
		case STATE_CHANGING_POLICY:
			kbase_pm_change_policy(kbdev);
			break;
		case STATE_POWERING_UP:
			data->state = STATE_POWERED_UP;
			kbase_pm_power_up_done(kbdev);
			break;
		case STATE_POWERING_DOWN:
			data->state = STATE_POWERED_DOWN;
			/* Disable interrupts and turn the clock off */
			kbase_pm_disable_interrupts(kbdev);
			kbase_pm_clock_off(kbdev);
			kbase_pm_power_down_done(kbdev);
			break;
		default:
			break;
	}
}

/** The event callback function for the demand power policy.
 *
 * This function is called to handle the events for the power policy. It calls the relevant handler function depending 
 * on the type of the event.
 *
 * @param kbdev     The kbase device structure for the device
 * @param event     The event that should be processed
 */
static void demand_event(kbase_device *kbdev, kbase_pm_event event)
{
	kbase_pm_policy_data *data = kbdev->pm.policy_data;

	switch(event)
	{
		case KBASE_PM_EVENT_INIT:
			demand_prepare_power_up(kbdev);
			break;
		case KBASE_PM_EVENT_SUSPEND:
			kbase_pm_power_down_done(kbdev);
			break;
		case KBASE_PM_EVENT_RESUME:
			kbase_pm_power_up_done(kbdev);
			break;
		case KBASE_PM_EVENT_STATE_CHANGED:
			demand_state_changed(kbdev);
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
			demand_prepare_power_up(kbdev);
			break;
		case KBASE_PM_EVENT_GPU_IDLE:
			demand_prepare_power_down(kbdev);
			break;
	}
}

/** Initialize the demand power policy.
 *
 * This sets up the private @ref kbase_pm_device_data.policy_data field of the device for use with the demand power 
 * policy.
 *
 * @param kbdev     The kbase device structure for the device
 */
static void demand_init(kbase_device *kbdev)
{
	kbase_pm_policy_data *data;

	data = osk_malloc(sizeof *data);
	OSK_ASSERT(data);

	/* Assume the GPU is powered up */
	data->state = STATE_POWERED_UP;

	kbdev->pm.policy_data = data;
}

/** Terminate the demand power policy.
 *
 * This frees the resources that were allocated by @ref demand_init.
 *
 * @param kbdev     The kbase device structure for the device
 */
static void demand_term(kbase_device *kbdev)
{
	osk_free(kbdev->pm.policy_data);
}

/** The @ref kbase_pm_policy structure for the demand power policy.
 *
 * This is the static structure that defines the demand power policy's callback and name.
 */
static const kbase_pm_policy ops =
{
	"demand",                   /* name */
	demand_init,                /* init */
	demand_term,                /* term */
	demand_event,               /* event */
};

/** Get the demand policy @ref kbase_pm_policy structure.
 *
 * This returns the static @ref kbase_pm_policy structure for the demand power policy.
 *
 * @return The static @ref kbase_pm_policy structure for the demand power policy (see @ref ops)
 */
const kbase_pm_policy *kbase_pm_demand_get_policy(void)
{
	return &ops;
}
