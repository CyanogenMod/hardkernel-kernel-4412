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
 * @file mali_kbase_runtime_pm.c
 * Runtime PM
 */

#include <osk/mali_osk.h>
#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_uku.h>
#include <kbase/src/common/mali_kbase_mem.h>
#include <kbase/src/common/mali_midg_regmap.h>
#include <kbase/src/linux/mali_kbase_mem_linux.h>
#include <uk/mali_ukk.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/miscdevice.h>
#include <linux/list.h>
#include <linux/semaphore.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/timer.h>
#include <kbase/src/platform/mali_kbase_platform.h>
#include <linux/pm_runtime.h>

/** Suspend callback from the OS.
 *
 * This is called by Linux runtime PM when the device should suspend.
 *
 * @param dev	The device to suspend
 *
 * @return A standard Linux error code
 */
int kbase_device_runtime_suspend(struct device *dev)
{
    if(kbase_platform_clock_off(dev))
	panic("failed to turn off sclk_g3d\n");
    if(kbase_platform_power_off(dev))
	panic("failed to turn off g3d power\n");

    return 0;
}

/** Resume callback from the OS.
 *
 * This is called by Linux runtime PM when the device should resume from suspension.
 *
 * @param dev	The device to resume
 *
 * @return A standard Linux error code
 */
int kbase_device_runtime_resume(struct device *dev)
{
    if(kbase_platform_clock_on(dev))
	panic("failed to turn on sclk_g3d\n");
    if(kbase_platform_power_on(dev))
	panic("failed to turn on g3d power\n");

    return 0;
}

/** Enable runtiem pm
 *
 * @param dev	The device to enable rpm
 *
 * @return A standard Linux error code
 */
void kbase_device_runtime_enable(struct device *dev)
{
    pm_runtime_enable(dev);
}

/** Disable runtime pm
 *
 * @param dev	The device to enable rpm
 *
 * @return A standard Linux error code
 */
void kbase_device_runtime_disable(struct device *dev)
{
    pm_runtime_disable(dev);
}

/** Initialize runtiem pm fields in given device 
 *
 * @param dev	The device to initialize
 *
 * @return A standard Linux error code
 */
void kbase_device_runtime_init(struct device *dev)
{
    pm_runtime_enable(dev);
}

