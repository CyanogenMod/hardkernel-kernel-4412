/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2010-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

#ifndef _UMP_ARCH_H_
#define _UMP_ARCH_H_

#include <common/ump_kernel_core.h>

/**
 * Device specific setup.
 * Called by the UMP core code to to host OS/device specific setup.
 * Typical use case is device node creation for talking to user space.
 * @return UMP_OK on success, any other value on failure
 */
extern ump_result umpp_device_initialize(void);

/**
 * Device specific teardown.
 * Undo any things done by ump_device_initialize.
 */
extern void umpp_device_terminate(void);

/* These three are to be removed once OSK is updated */
extern int umpp_phys_commit(umpp_allocation * alloc);
extern ump_resize_result umpp_phys_recommit(umpp_allocation * alloc, s64 size_diff, u64 * new_size);
extern void umpp_phys_free(umpp_allocation * alloc);

#endif /* _UMP_ARCH_H_ */
