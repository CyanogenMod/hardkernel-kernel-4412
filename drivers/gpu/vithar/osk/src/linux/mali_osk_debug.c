/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2010-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

#include <osk/mali_osk.h>

void oskp_debug_print(const char *fmt, ...)
{
	char buffer[OSK_DEBUG_MESSAGE_SIZE];
	va_list args;
	va_start(args, fmt);
	cutils_cstr_vsnprintf(buffer, OSK_DEBUG_MESSAGE_SIZE, fmt, args);
	printk(buffer);
	va_end(args);
}

