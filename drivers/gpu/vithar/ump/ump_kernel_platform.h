/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file ump_kernel_platform.h
 *
 * This file should define UMP_KERNEL_API_EXPORT,
 * which dictates how the UMP kernel API should be exported/imported.
 * Modify this file, if needed, to match your platform setup.
 */

#ifndef _UMP_KERNEL_PLATFORM_H_
#define _UMP_KERNEL_PLATFORM_H_

#include <malisw/mali_stdtypes.h>

/**
 * @addtogroup ump_api
 * @{
 */

/** @addtogroup ump_kernel_space_api
 * @{ */

/**
 * A define which controls how UMP kernel space API functions are imported and exported.
 *
 * Functions exported by the kernel driver is tagged with UMP_KERNEL_API_EXPORT to allow
 * the compiler/build system/OS loader to detect and handle functions which is to be exported/imported from a kernel driver.@n
 * This define should be set by the implementor of the UMP API to match their needs if needed.
 *
 * Typical usage example in the driver:
 *
 * UMP_KERNEL_API_EXPORT void my_kernel_api(void);
 */

#if defined(_WIN32)

#if defined(UMP_BUILDING_UMP_LIBRARY)
#define UMP_KERNEL_API_EXPORT __declspec(dllexport)
#else
#define UMP_KERNEL_API_EXPORT __declspec(dllimport)
#endif

#else

#define UMP_KERNEL_API_EXPORT

#endif


/** @} */ /* end group ump_kernel_space_api */

/** @} */ /* end group ump_api */

#endif /* _UMP_KERNEL_PLATFORM_H_ */
