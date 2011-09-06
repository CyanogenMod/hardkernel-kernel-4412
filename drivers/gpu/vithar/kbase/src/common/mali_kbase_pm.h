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
 * @file mali_kbase_pm.h
 * Power management
 */

#ifndef _KBASE_PM_H_
#define _KBASE_PM_H_

#include <kbase/src/common/mali_midg_regmap.h>

/* Forward definition - see mali_kbase.h */
struct kbase_device;

/** The types of core in a GPU.
 *
 * These enumerated values are used in calls to @ref kbase_pm_invoke_power_up, @ref kbase_pm_invoke_power_down, @ref 
 * kbase_pm_get_present_cores, @ref kbase_pm_get_active_cores, @ref kbase_pm_get_trans_cores, @ref 
 * kbase_pm_get_ready_cores. The specify which type of core should be acted on.
 * These values are set in a manner that allows @ref core_type_to_reg function to be simpler and more efficient.
 */
typedef enum kbase_pm_core_type
{
	KBASE_PM_CORE_L3	 = L3_PRESENT_LO,       /**< The L3 cache */
	KBASE_PM_CORE_L2	 = L2_PRESENT_LO,       /**< The L2 cache */
	KBASE_PM_CORE_SHADER = SHADER_PRESENT_LO,   /**< Shader cores */
	KBASE_PM_CORE_TILER  = TILER_PRESENT_LO     /**< Tiler cores */
} kbase_pm_core_type;

/** Initialize the power management framework.
 *
 * Must be called before any other power management function
 *
 * @return MALI_ERROR_NONE if the power management framework was successfully initialized.
 */
mali_error kbase_pm_init(struct kbase_device *);

/** Terminate the power management framework.
 *
 * No power management functions may be called after this
 * (except @ref kbase_pm_init)
 */
void kbase_pm_term(struct kbase_device *);

/** Events that can be sent to a power policy.
 *
 * Power policies are expected to handle all these events, although they may choose to take no action.
 */
typedef enum kbase_pm_event
{
	/** Initialize the power policy.
	 *
	 * This event is sent immediately after the @ref kbase_pm_policy.init function of the policy returns.
	 *
	 * The policy may decide to transition the cores to its 'normal' state (e.g. an always on policy would turn all 
	 * the cores on). The policy should assume that the GPU is in active use (i.e. as if the @ref 
	 * KBASE_PM_EVENT_GPU_ACTIVE event had been received), if this is not the case then @ref KBASE_PM_EVENT_GPU_IDLE 
	 * will be called after this event has been handled.
	 */
	KBASE_PM_EVENT_INIT,
	/** The power state of the device has changed.
	 *
	 * This event is sent when the GPU raises an interrupt to announce that a power transition has finished. Because 
	 * there may be multiple power transitions the power policy must interrogate the state of the GPU to check whether 
	 * all expected transitions have finished. If the GPU has just turned on or off then the policy must call @ref 
	 * kbase_pm_power_up_done or @ref kbase_pm_power_down_done as appropriate.
	 */
	KBASE_PM_EVENT_STATE_CHANGED,
	/** The GPU is becoming active.
	 *
	 * This event is sent when the first context is about to use the GPU.
	 *
	 * If the core is turned off then this event must cause the core to turn on. This is done asynchronously and the 
	 * policy must call the function kbase_pm_power_up_done to signal that the core is turned on sufficiently to allow 
	 * register access.
	 */
	KBASE_PM_EVENT_GPU_ACTIVE,
	/** The GPU is becoming idle.
	 *
	 * This event is sent when the last context has finished using the GPU.
	 *
	 * The power policy may turn the GPU off entirely (e.g. turn the clocks or power off).
	 */
	KBASE_PM_EVENT_GPU_IDLE,
	/** The system has requested a change of power policy.
	 *
	 * The current policy receives this message when a request to change policy occurs. It must ensure that all active 
	 * power transitions are completed and then call the @ref kbase_pm_change_policy function.
	 * 
	 * This event is only delivered when the policy has been informed that the GPU is 'active' (the power management 
	 * code internally increments the context active counter during a policy change).
	 */
	KBASE_PM_EVENT_CHANGE_POLICY,
	/** The system is requesting to suspend the GPU.
	 *
	 * The power policy should ensure that the GPU is shut down sufficiently for the system to suspend the device.  
	 * Once the GPU is ready the policy should call @ref kbase_pm_power_down_done.
	 */
	KBASE_PM_EVENT_SUSPEND,
	/** The system is requesting to resume the GPU.
	 *
	 * The power policy should restore the GPU to the state it was before the previous @ref KBASE_PM_EVENT_SUSPEND 
	 * event. If the GPU is being powered up then it should call @ref kbase_pm_power_transitioning before changing the 
	 * state and @ref kbase_pm_power_up_done when the transition is complete.
	 */
	KBASE_PM_EVENT_RESUME
} kbase_pm_event;

typedef struct kbase_pm_policy_data kbase_pm_policy_data;

/** Power policy structure.
 *
 * Each power management policy exposes a (static) instance of this structure which contains function pointers to the 
 * policy's methods.
 */
typedef struct kbase_pm_policy
{
	/** The name of this policy */
	char *name;

	/** Function called when the policy is selected
	 *
	 * This should initialize the kbdev->pm.policy_data pointer to the policy's data structure. It should not attempt 
	 * to make any changes to hardware state.
	 *
	 * It is undefined what state the cores are in when the function is called, however no power transitions should be 
	 * occuring.
	 *
	 * @return A pointer to a private kbase_pm_policy_data object
	 */
	void (*init)(struct kbase_device *kbdev);
	/** Function called when the policy is unselected.
	 *
	 * This should free any data allocated with \c init
	 *
	 * @param[in] data The object returned by \c init. The function should free any resources associated with this 
	 * object
	 */
	void (*term)(struct kbase_device *kbdev);
	/** Function called when there is an event to process
	 *
	 * @param[in] data The object returned by \c init
	 */
	void (*event)(struct kbase_device *kbdev, kbase_pm_event event);
} kbase_pm_policy;

/** Data stored per device for power management.
 *
 * This structure contains data for the power management framework. There is one instance of this structure per device 
 * in the system.
 */
typedef struct kbase_pm_device_data
{
	/** The policy that is currently actively controlling the power state. */
	const kbase_pm_policy   *current_policy;
	/** The policy that the system is transitioning to. */
	const kbase_pm_policy   *new_policy;
	/** The data needed for the current policy. This is considered private to the policy. */
	kbase_pm_policy_data    *policy_data;
	/** The workqueue that the policy callbacks are executed on. */
	osk_workq               workqueue;
	/** A bit mask of events that are waiting to be delivered to the active policy. */
	osk_atomic              pending_events;
	/** The work unit that is enqueued onto the workqueue. */
	osk_workq_work          work;
	/** An atomic which tracks whether the work unit has been enqueued:
	 * @arg 0 - Inactive. There are no work units enqueued and @ref kbase_pm_worker is not running.
	 * @arg 1 - Enqueued. There is a work unit enqueued, but @ref kbase_pm_worker is not running.
	 * @arg 2 - Processing. @ref kbase_pm_worker is running.
	 * @arg 3 - Processing and there's an event outstanding. @ref kbase_pm_worker is running, but @ref pending_events
	 *          has been updated since it started so it should recheck the list of pending events before exiting.
	 */
	osk_atomic              work_active;
	/** The wait queue for power up events. */
	osk_waitq               power_up_waitqueue;
	/** The wait queue for power down events. */
	osk_waitq               power_down_waitqueue;
	/** The reference count of active contexts on this device. */
	osk_atomic              active_count;
} kbase_pm_device_data;

/** Get the current policy.
 * Returns the policy that is currently active.
 * 
 * @return The current policy
 */
const kbase_pm_policy *kbase_pm_get_policy(struct kbase_device *);

/** Change the policy to the one specified.
 * @param policy    The policy to change to.
 */
void kbase_pm_set_policy(struct kbase_device *, const kbase_pm_policy *policy);

/** Retrieve a statoc list of the available policies.
 * @param[out]  policies    An array pointer to take the list of policies.
 *                          The contents of this array must not be modified
 *
 * @return The number of policies
 */
int kbase_pm_list_policies(const kbase_pm_policy ***policies);

/** The current policy is ready to change to the new policy
 *
 * The current policy must ensure that all cores have finished transitioning before calling this function.
 */
void kbase_pm_change_policy(struct kbase_device *kbdev);

/** The GPU is idle.
 *
 * The OS may choose to turn off idle devices
 */
void kbase_pm_dev_idle(struct kbase_device *);

/** The GPU is active.
 *
 * The OS should avoid opportunistically turning off the GPU while it is active
 */
void kbase_pm_dev_activate(struct kbase_device *);

/** Send an event to the active power policy.
 *
 * The event is queued for sending to the active power policy. The event is merged with the current queue by the @ref 
 * kbasep_pm_merge_event function which may decide to drop events.
 *
 * Note that this function may be called in an atomic context on Linux which implies that it must not sleep.
 *
 * @param kbdev     The kbase device structure
 * @param event     The event that should be queued
 */
void kbase_pm_send_event(struct kbase_device *kbdev, kbase_pm_event event);

/** Turn one or more cores on.
 *
 * This function is called by the active power policy to turn one or more cores on.
 *
 * @param kbdev     The kbase device structure for the device
 * @param type      The type of core (see the @ref kbase_pm_core_type enumeration)
 * @param lo        A bitmask of cores to turn on (low 32 bits)
 * @param hi        A bitmask of cores to turn on (high 32 bits)
 */
void kbase_pm_invoke_power_up(struct kbase_device *kbdev, kbase_pm_core_type type, u32 lo, u32 hi);

/** Turn one or more cores off.
 *
 * This function is called by the active power policy to turn one or more core off.
 *
 * @param kbdev     The kbase device structure for the device
 * @param type      The type of core (see the @ref kbase_pm_core_type enumeration)
 * @param lo        A bitmask of cores to turn off (low 32 bits)
 * @param hi        A bitmask of cores to turn off (high 32 bits)
 */
void kbase_pm_invoke_power_down(struct kbase_device *kbdev, kbase_pm_core_type type, u32 lo, u32 hi);

/** Get details of the cores that are present in the device.
 *
 * This function can be called by the active power policy to return a bitmask of the cores (of a specified type) 
 * present in the GPU device and also a count of the number of cores.
 *
 * @param kbdev     The kbase device structure for the device
 * @param type      The type of core (see the @ref kbase_pm_core_type enumeration)
 * @param lo        The address to write the low 32 bits of the bit mask of cores present
 * @param hi        The address to write the high 32 bits of the bit mask of cores present
 *
 * @return The number of cores present of the specified type
 */
int kbase_pm_get_present_cores(struct kbase_device *kbdev, kbase_pm_core_type type, u32 *lo, u32 *hi);

/** Get details of the cores that are currently active in the device.
 *
 * This function can be called by the active power policy to return a bitmask of the cores (of a specified type) that 
 * are actively processing work (i.e. turned on *and* busy).
 *
 * @param kbdev     The kbase device structure for the device
 * @param type      The type of core (see the @ref kbase_pm_core_type enumeration)
 * @param lo        The address to write the low 32 bits of the bit mask of active cores
 * @param hi        The address to write the high 32 bits of the bit mask of active cores
 *
 * @return The number of cores that are active of the specified type
 */
int kbase_pm_get_active_cores(struct kbase_device *kbdev, kbase_pm_core_type type, u32 *lo, u32 *hi);

/** Get details of the cores that are currently transitioning between power states.
 *
 * This function can be called by the active power policy to return a bitmask of the cores (of a specified type) that 
 * are currently transitioning between power states.
 *
 * @param kbdev     The kbase device structure for the device
 * @param type      The type of core (see the @ref kbase_pm_core_type enumeration)
 * @param lo        The address to write the low 32 bits of the bit mask of transitioning cores
 * @param hi        The address to write the high 32 bits of the bit mask of transitioning cores
 *
 * @return The number of cores that are currently transitioning of the specified type
 */
int kbase_pm_get_trans_cores(struct kbase_device *kbdev, kbase_pm_core_type type, u32 *lo, u32 *hi);

/** Get details of the cores that are currently powered and ready for jobs.
 *
 * This function can be called by the active power policy to return a bitmask of the cores (of a specified type) that 
 * are powered and ready for jobs (they may or may not be currently executing jobs).
 *
 * @param kbdev     The kbase device structure for the device
 * @param type      The type of core (see the @ref kbase_pm_core_type enumeration)
 * @param lo        The address to write the low 32 bits of the bit mask of ready cores
 * @param hi        The address to write the high 32 bits of the bit mask of ready cores
 *
 * @return The number of cores that are currently ready of the specified type
 */
int kbase_pm_get_ready_cores(struct kbase_device *kbdev, kbase_pm_core_type type, u32 *lo, u32 *hi);

/** Return whether the power manager is active
 *
 * This function will return true when there are cores (of any time) that are currently transitioning between power 
 * states.
 *
 * It can be used on receipt of the @ref KBASE_PM_EVENT_STATE_CHANGED message to determine whether the requested power 
 * transitions have completely finished or not.
 * 
 * @return true when there are cores transitioning between power states, false otherwise
 */
mali_bool kbase_pm_get_pwr_active(struct kbase_device *kbdev);

/** Turn the clock for the device on.
 *
 * This function can be used by a power policy to turn the clock for the GPU on. It should be modified during 
 * integration to perform the necessary actions to ensure that the GPU is fully powered and clocked.
 *
 * @param kbdev     The kbase device structure for the device
 */
void kbase_pm_clock_on(struct kbase_device *kbdev);
/** Turn the clock for the device off.
 *
 * This function can be used by a power policy to turn the clock for the GPU off. It should be modified during 
 * integration to perform the necessary actions to turn the clock off (if this is possible in the integration).
 *
 * @param kbdev     The kbase device structure for the device
 */
void kbase_pm_clock_off(struct kbase_device *kbdev);

/** Enable interrupts on the device.
 *
 * This function should be called by the active power policy immediately after calling @ref kbase_pm_clock_on to 
 * ensure that interrupts are enabled on the device.
 *
 * @param kbdev     The kbase device structure for the device
 */
void kbase_pm_enable_interrupts(struct kbase_device *kbdev);

/** Disable interrupts on the device.
 *
 * This function should be called by the active power policy after shutting down the device (i.e. in the @ref 
 * KBASE_PM_EVENT_STATE_CHANGED handler after confirming that all cores have powered off). It prevents interrupt 
 * delivery to the CPU so no further @ref KBASE_PM_EVENT_STATE_CHANGED messages will be received until @ref 
 * kbase_pm_enable_interrupts is called.
 *
 * @param kbdev     The kbase device structure for the device
 */
void kbase_pm_disable_interrupts(struct kbase_device *kbdev);

/** Initialize the hardware
 *
 * This function checks the GPU ID register to ensure that the GPU is supported by the driver and performs a reset on 
 * the device so that it is in a known state before the device is used.
 *
 * @param kbdev     The kbase device structure for the device
 * 
 * @return MALI_ERROR_NONE if the device is supported and successfully reset.
 */
mali_error kbase_pm_init_hw(struct kbase_device *kbdev);

/** Inform the power management system that the power state of the device is transitioning.
 *
 * This function must be called by the active power policy before transitioning the core between an 'off state' and an 
 * 'on state'. It resets the wait queues that are waited on by @ref kbase_pm_wait_for_power_up and @ref 
 * kbase_pm_wait_for_power_down.
 *
 * @param kbdev     The kbase device structure for the device
 */
void kbase_pm_power_transitioning(struct kbase_device *kbdev);

/** The GPU has been powered up successfully.
 *
 * This function must be called by the active power policy when the GPU has been powered up successfully. It signals 
 * to the rest of the system that jobs can start being submitted to the device.
 *
 * @param kbdev     The kbase device structure for the device
 */
void kbase_pm_power_up_done(struct kbase_device *kbdev);

/** The GPU has been reset successfully.
 *
 * This function must be called by the GPU interrupt handler when the RESET_COMPLETED bit is set. It signals to the 
 * power management initialization code that the GPU has been successfully reset.
 *
 * @param kbdev     The kbase device structure for the device
 */
void kbase_pm_reset_done(struct kbase_device *kbdev);

/** The GPU has been powered down successfully.
 *
 * This function must be called by the active power policy when the GPU has been powered down successfully. It signals 
 * to the rest of the system that a system suspend can now take place.
 *
 * @param kbdev     The kbase device structure for the device
 */
void kbase_pm_power_down_done(struct kbase_device *kbdev);

/** Wait for the power policy to signal power up.
 *
 * This function waits for the power policy to signal power up by calling @ref kbase_pm_power_up_done. After the power 
 * policy has signalled this the function will return immediately until the power policy calls @ref 
 * kbase_pm_power_transitioning.
 *
 * @param kbdev     The kbase device structure for the device
 */
void kbase_pm_wait_for_power_up(struct kbase_device *kbdev);

/** Wait for the power policy to signal power down.
 *
 * This function waits for the power policy to signal power down by calling @ref kbase_pm_power_down_done. After the 
 * power policy has signalled this the function will return immediately until the power policy calls @ref 
 * kbase_pm_power_transitioning.
 *
 * @param kbdev     The kbase device structure for the device
 */
void kbase_pm_wait_for_power_down(struct kbase_device *kbdev);

/** Increment the count of active contexts.
 *
 * This function should be called when a context is about to submit a job. It informs the active power policy that the 
 * GPU is going to be in use shortly and the policy is expected to start turning on the GPU.
 *
 * @param kbdev     The kbase device structure for the device
 */
void kbase_pm_context_active(struct kbase_device *kbdev);

/** Decrement the reference count of active contexts.
 *
 * This function should be called when a context becomes idle. After this call the GPU may be turned off by the power 
 * policy so the calling code should ensure that it does not access the GPU's registers.
 *
 * @param kbdev     The kbase device structure for the device
 */
void kbase_pm_context_idle(struct kbase_device *kbdev);

#endif /* _KBASE_PM_H_ */
