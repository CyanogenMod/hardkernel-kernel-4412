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
 * @file mali_kbase_platform.h
 * Platform init
 */

#ifndef _KBASE_PLATFORM_H_
#define _KBASE_PLATFORM_H_

/* All things that are needed for the Linux port. */

int kbase_platform_clock_on(struct device *dev);
int kbase_platform_clock_off(struct device *dev);
int kbase_platform_power_on(struct device *dev);
int kbase_platform_power_off(struct device *dev);
int kbase_platform_create_sysfs_file(struct device *dev);
void kbase_platform_remove_sysfs_file(struct device *dev);
int kbase_platform_init(struct device *dev);
int kbase_platform_is_power_on(void);

#endif /* _KBASE_PLATFORM_H_ */
