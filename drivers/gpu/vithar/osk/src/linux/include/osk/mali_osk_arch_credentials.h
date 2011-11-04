/*
 *
 * (C) COPYRIGHT 2008-2011 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * 
 */



/**
 * @file
 * Implementation of the OS abstraction layer for the kernel device driver
 */

#ifndef _OSK_ARCH_CREDENTIALS_H_
#define _OSK_ARCH_CREDENTIALS_H_

#ifndef _OSK_H_
#error "Include mali_osk.h directly"
#endif

#include <linux/cred.h>

OSK_STATIC_INLINE mali_bool osk_is_privileged(void)
{
	mali_bool is_privileged = MALI_FALSE;

	/* Check if the caller is root */
	if (current_euid() == 0)
	{
		is_privileged = MALI_TRUE;
	}

	return is_privileged;
}

#endif /* _OSK_ARCH_CREDENTIALS_H_ */
