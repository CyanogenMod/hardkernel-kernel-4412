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
 * @file mali_kbase_runtime_pm.h
 * Runtime PM
 */

#ifndef _KBASE_RUNTIME_PM_H_
#define _KBASE_RUNTIME_PM_H_

/* All things that are needed for the Linux port. */

int kbase_device_runtime_suspend(struct device *dev);
int kbase_device_runtime_resume(struct device *dev);
/*
void kbase_device_runtime_init_timer(void);
void kbase_device_runtime_restart_timer(void);
void kbase_device_runtime_stop_timer(void);
*/
void kbase_device_runtime_enable(struct device *dev);
void kbase_device_runtime_disable(struct device *dev);
void kbase_device_runtime_init(struct device *dev);

/* Delay time to enter into runtime-suspend */
#define RUNTIME_PM_RUNTIME_DELAY_TIME 500

#endif /* _KBASE_RUNTIME_PM_H_ */
