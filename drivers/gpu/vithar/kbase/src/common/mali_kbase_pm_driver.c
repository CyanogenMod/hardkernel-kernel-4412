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
 * @file mali_kbase_pm_driver.c
 * Base kernel Power Management hardware control
 */

#include <osk/mali_osk.h>
#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_midg_regmap.h>

#include <kbase/src/common/mali_kbase_pm.h>

/** Number of milliseconds before we time out on a reset */
#define RESET_TIMEOUT   500

/** Actions that can be performed on a core.
 *
 * This enumeration is private to the file. Its values are set to allow @ref core_type_to_reg function,
 * which decodes this enumeration, to be simpler and more efficient.
 */
typedef enum kbasep_pm_action
{
	ACTION_PRESENT      = 0,
	ACTION_READY        = (SHADER_READY_LO - SHADER_PRESENT_LO),
	ACTION_PWRON        = (SHADER_PWRON_LO - SHADER_PRESENT_LO),
	ACTION_PWROFF       = (SHADER_PWROFF_LO - SHADER_PRESENT_LO),
	ACTION_PWRTRANS     = (SHADER_PWRTRANS_LO - SHADER_PRESENT_LO),
	ACTION_PWRACTIVE    = (SHADER_PWRACTIVE_LO - SHADER_PRESENT_LO)
} kbasep_pm_action;

/** Decode a core type and action to a register.
 *
 * Given a core type (defined by @ref kbase_pm_core_type) and an action (defined by @ref kbasep_pm_action) this 
 * function will return the register offset that will perform the action on the core type. The register returned is 
 * the \c _LO register and an offset must be applied to use the \c _HI register.
 * 
 * @param core_type The type of core
 * @param action    The type of action
 *
 * @return The register offset of the \c _LO register that performs an action of type \c action on a core of type \c 
 * core_type.
 */
static u32 core_type_to_reg(kbase_pm_core_type core_type, kbasep_pm_action action)
{
	return core_type + action;
}

/** Invokes an action on a core set
 *
 * This function performs the action given by \c action on a set of cores of a type given by \c core_type. It is a 
 * static function used by @ref kbase_pm_invoke_power_up and @ref kbase_pm_invoke_power_down.
 *
 * @param kbdev     The kbase device structure of the device
 * @param core_type The type of core that the action should be performed on
 * @param lo        A bit mask of cores to perform the action on (low 32 bits)
 * @param hi        A bit mask of cores to perform the action on (high 32 bits)
 * @param action    The action to perform on the cores
 */
static void kbase_pm_invoke(kbase_device *kbdev, kbase_pm_core_type core_type, u32 lo, u32 hi, kbasep_pm_action action)
{
	u32 reg;

	reg = core_type_to_reg(core_type, action);

	OSK_ASSERT(reg);

	kbase_reg_write(kbdev, GPU_CONTROL_REG(reg), lo, NULL);
	kbase_reg_write(kbdev, GPU_CONTROL_REG(reg+4), hi, NULL);
}

void kbase_pm_invoke_power_up(kbase_device *kbdev, kbase_pm_core_type type, u32 lo, u32 hi)
{
	u32 trans_lo, trans_hi;

	do {
		kbase_pm_get_trans_cores(kbdev, type, &trans_lo, &trans_hi);
	} while ((trans_lo & lo) || (trans_hi & hi));

	kbase_pm_invoke(kbdev, type, lo, hi, ACTION_PWRON);
}

void kbase_pm_invoke_power_down(kbase_device *kbdev, kbase_pm_core_type type, u32 lo, u32 hi)
{
	u32 trans_lo, trans_hi;

	do {
		kbase_pm_get_trans_cores(kbdev, type, &trans_lo, &trans_hi);
	} while ((trans_lo & lo) || (trans_hi & hi));

	kbase_pm_invoke(kbdev, type, lo, hi, ACTION_PWROFF);
}

/** Get information about a core set
 *
 * This function gets information (chosen by \c action) about a set of cores of a type given by \c core_type. It is a 
 * static function used by @ref kbase_pm_get_present_cores, @ref kbase_pm_get_active_cores, @ref 
 * kbase_pm_get_trans_cores and @ref kbase_pm_get_ready_cores.
 *
 * @param kbdev     The kbase device structure of the device
 * @param core_type The type of core that the should be queried
 * @param lo        A pointer to receive a bit mask specifying the state of the cores (low 32 bits)
 * @param hi        A pointer to receive a bit mask specifying the state of the cores (high 32 bits)
 * @param action    The property of the cores to query
 *
 * @return The number of bits set in \c lo and \c hi (i.e. the number of cores that are in the requested state).
 */
static int kbase_pm_get_state(kbase_device *kbdev, kbase_pm_core_type core_type, u32 *lo, u32 *hi,
        kbasep_pm_action action)
{
	u32 reg;
	u32 _lo, _hi;

	reg = core_type_to_reg(core_type, action);

	OSK_ASSERT(reg);

	_lo = kbase_reg_read(kbdev, GPU_CONTROL_REG(reg), NULL);
	_hi = kbase_reg_read(kbdev, GPU_CONTROL_REG(reg+4), NULL);

	if (lo) *lo = _lo;
	if (hi) *hi = _hi;

	return osk_count_set_bits(_lo) + osk_count_set_bits(_hi);
}

int kbase_pm_get_present_cores(kbase_device *kbdev, kbase_pm_core_type type, u32 *lo, u32 *hi)
{
	return kbase_pm_get_state(kbdev, type, lo, hi, ACTION_PRESENT);
}

int kbase_pm_get_active_cores(kbase_device *kbdev, kbase_pm_core_type type, u32 *lo, u32 *hi)
{
	return kbase_pm_get_state(kbdev, type, lo, hi, ACTION_PWRACTIVE);
}

int kbase_pm_get_trans_cores(kbase_device *kbdev, kbase_pm_core_type type, u32 *lo, u32 *hi)
{
	return kbase_pm_get_state(kbdev, type, lo, hi, ACTION_PWRTRANS);
}

int kbase_pm_get_ready_cores(kbase_device *kbdev, kbase_pm_core_type type, u32 *lo, u32 *hi)
{
	return kbase_pm_get_state(kbdev, type, lo, hi, ACTION_READY);
}

mali_bool kbase_pm_get_pwr_active(kbase_device *kbdev)
{
	return ((kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_STATUS), NULL) & (1<<1)) != 0);
}

void kbase_pm_enable_interrupts(kbase_device *kbdev)
{
	/*
	 * Clear all interrupts,
	 * and unmask them all.
	 */
	kbase_reg_write(kbdev, GPU_CONTROL_REG(GPU_IRQ_CLEAR), GPU_IRQ_REG_ALL, NULL);
	/* Disable the POWER_CHANGED_SINGLE interrupt as we don't use it */
	kbase_reg_write(kbdev, GPU_CONTROL_REG(GPU_IRQ_MASK), GPU_IRQ_REG_ALL & ~POWER_CHANGED_SINGLE, NULL);

	kbase_reg_write(kbdev, JOB_CONTROL_REG(JOB_IRQ_CLEAR), 0xFFFFFFFF, NULL);
	kbase_reg_write(kbdev, JOB_CONTROL_REG(JOB_IRQ_MASK), 0xFFFFFFFF, NULL);

	kbase_reg_write(kbdev, MMU_REG(MMU_IRQ_CLEAR), 0xFFFFFFFF, NULL);
	kbase_reg_write(kbdev, MMU_REG(MMU_IRQ_MASK), 0xFFFFFFFF, NULL);
}

void kbase_pm_disable_interrupts(kbase_device *kbdev)
{
	/*
	 * Mask all interrupts,
	 * and clear them all.
	 */
	kbase_reg_write(kbdev, GPU_CONTROL_REG(GPU_IRQ_MASK), 0, NULL);
	kbase_reg_write(kbdev, GPU_CONTROL_REG(GPU_IRQ_CLEAR), GPU_IRQ_REG_ALL, NULL);

	kbase_reg_write(kbdev, JOB_CONTROL_REG(JOB_IRQ_MASK), 0, NULL);
	kbase_reg_write(kbdev, JOB_CONTROL_REG(JOB_IRQ_CLEAR), 0xFFFFFFFF, NULL);

	kbase_reg_write(kbdev, MMU_REG(MMU_IRQ_MASK), 0, NULL);
	kbase_reg_write(kbdev, MMU_REG(MMU_IRQ_CLEAR), 0xFFFFFFFF, NULL);
}

/*
 * pmu layout:
 * 0x0000: PMU TAG (RO) (0xCAFECAFE)
 * 0x0004: PMU VERSION ID (RO) (0x00000000)
 * 0x0008: CLOCK ENABLE (RW) (31:1 SBZ, 0 CLOCK STATE)
 */

void kbase_pm_clock_on(kbase_device *kbdev)
{
	/* The GPU is going to transition, so unset the wait queues until the policy
	 * informs us that the transition is complete */
	osk_waitq_clear(&kbdev->pm.power_up_waitqueue);
	osk_waitq_clear(&kbdev->pm.power_down_waitqueue);

	if (kbase_device_has_feature(kbdev, KBASE_FEATURE_HAS_MODEL_PMU))
		kbase_reg_write(kbdev, 0x4008, 1, NULL);
}

void kbase_pm_clock_off(kbase_device *kbdev)
{
	if (kbase_device_has_feature(kbdev, KBASE_FEATURE_HAS_MODEL_PMU))
		kbase_reg_write(kbdev, 0x4008, 0, NULL);
}

struct kbasep_reset_timeout_data
{
	mali_bool timed_out;
	kbase_device *kbdev;
};

static void kbasep_reset_timeout(void *data)
{
	struct kbasep_reset_timeout_data *rtdata = (struct kbasep_reset_timeout_data*)data;

	rtdata->timed_out = 1;

	/* Set the wait queue to take up kbase_pm_init_hw even though the reset hasn't completed */
	kbase_pm_reset_done(rtdata->kbdev);
}

static void kbase_pm_hw_issues(kbase_device *kbdev, uint16_t revision)
{
	if (revision == 0x0000)
	{
		/* Needed due to MIDBASE-748 */
		kbase_reg_write(kbdev, GPU_CONTROL_REG(PWR_KEY), 0x2968A819, NULL);
		kbase_reg_write(kbdev, GPU_CONTROL_REG(PWR_OVERRIDE0), 0x80, NULL);
		kbase_reg_write(kbdev, GPU_CONTROL_REG(PWR_KEY), 0, NULL);
	}
}

mali_error kbase_pm_init_hw(kbase_device *kbdev)
{
	uint32_t gpu_id;
	osk_timer timer;
	struct kbasep_reset_timeout_data rtdata;
	osk_error osk_err;

	/* Ensure the clock is on before attempting to access the hardware */
	kbase_pm_clock_on(kbdev);

	/* Read the ID register */
	gpu_id = kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_ID), NULL);
	OSK_PRINT_INFO(OSK_BASE_PM, "GPU identified as '%c%c' r%dp%d",
	            (gpu_id>>16) & 0xFF,
	            (gpu_id>>24) & 0xFF,
	            (gpu_id>>8) & 0xFF,
	            (gpu_id) & 0xFF);

	if ((gpu_id >> 16) != 0x6956)
	{
		OSK_PRINT_ERROR(OSK_BASE_PM, "This GPU is not supported by this driver\n");
		return MALI_ERROR_FUNCTION_FAILED;
	}

	/* Ensure interrupts are off to begin with, this also clears any outstanding interrupts */
	kbase_pm_disable_interrupts(kbdev);

	/* Unmask the reset complete interrupt only */
	kbase_reg_write(kbdev, GPU_CONTROL_REG(GPU_IRQ_MASK), (1<<8), NULL);

	/* Soft reset the GPU */
	kbdev->expecting_reset_completed_irq = MALI_TRUE;
	kbase_reg_write(kbdev, GPU_CONTROL_REG(GPU_COMMAND), 1, NULL);

	/* If the GPU never asserts the reset interrupt we just assume that the reset has completed */
	if (kbase_device_has_feature(kbdev, KBASE_FEATURE_LACKS_RESET_INT))
	{
		goto out;
	}
	
	/* Initialize a structure for tracking the status of the reset */
	rtdata.kbdev = kbdev;
	rtdata.timed_out = 0;

	/* Create a timer to use as a timeout on the reset */
	osk_err = osk_timer_init(&timer);
	if (OSK_ERR_NONE != osk_err)
	{
		return MALI_ERROR_FUNCTION_FAILED;
	}

	osk_timer_callback_set(&timer, kbasep_reset_timeout, &rtdata);
	osk_err = osk_timer_start(&timer, RESET_TIMEOUT);
	if (OSK_ERR_NONE != osk_err)
	{
		osk_timer_term(&timer);
		return MALI_ERROR_FUNCTION_FAILED;
	}

	/* Wait for the RESET_COMPLETED interrupt to be raised,
	 * we use the "power up" waitqueue since it isn't in use yet */
	osk_waitq_wait(&kbdev->pm.power_up_waitqueue);

	if (rtdata.timed_out == 0)
	{
		/* GPU has been reset */
		osk_timer_stop(&timer);
		osk_timer_term(&timer);

		goto out;
	}

	/* No interrupt has been received - check if the RAWSTAT register says the reset has completed */
	if (kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_IRQ_RAWSTAT), NULL) & (1<<8))
	{
#if MALI_HW_TYPE != 0
		/* The interrupt is get in the RAWSTAT, this suggests that the interrupts are not getting to the CPU */
		OSK_PRINT_WARN(OSK_BASE_PM, "Reset interrupt didn't reach CPU. Check interrupt assignments. NOTE: this warning is expected on older model versions, e.g. R106639\n");
#else
		/* NOTE: The Dummy model doesn't implement the power-on-reset behavior yet. */
#endif
		/* If interrupts aren't working we can't continue. NOTE: until the old model version on trunk is updated, we do not return an error. */

		goto out;
	}

	/* The GPU doesn't seem to be responding to the reset so try a hard reset */
	OSK_PRINT_WARN(OSK_BASE_PM, "Failed to soft reset GPU, attempting a hard reset\n");
	kbase_reg_write(kbdev, GPU_CONTROL_REG(GPU_COMMAND), 2, NULL);

	/* Restart the timer to wait for the hard reset to complete */
	rtdata.timed_out = 0;
	osk_err = osk_timer_start(&timer, RESET_TIMEOUT);
	if (OSK_ERR_NONE != osk_err)
	{
		osk_timer_term(&timer);
		return MALI_ERROR_FUNCTION_FAILED;
	}

	/* Wait for the RESET_COMPLETED interrupt to be raised,
	 * we use the "power up" waitqueue since it isn't in use yet */
	osk_waitq_wait(&kbdev->pm.power_up_waitqueue);

	if (rtdata.timed_out == 0)
	{
		/* GPU has been reset */
		osk_timer_stop(&timer);
		osk_timer_term(&timer);

		goto out;
	}

	OSK_PRINT_ERROR(OSK_BASE_PM, "Failed to reset the GPU\n");

	/* The GPU still hasn't reset, give up */
	return MALI_ERROR_FUNCTION_FAILED;

out:
	kbase_pm_hw_issues(kbdev, gpu_id & 0xFFFF);
	return MALI_ERROR_NONE;
}
