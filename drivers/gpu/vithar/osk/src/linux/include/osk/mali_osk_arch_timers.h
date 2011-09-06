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
 * @file
 * Implementation of the OS abstraction layer for the kernel device driver
 */

#ifndef _OSK_ARCH_TIMERS_H
#define _OSK_ARCH_TIMERS_H

#ifndef _OSK_H_
#error "Include mali_osk.h directly"
#endif

OSK_STATIC_INLINE osk_error osk_timer_init(osk_timer * const tim)
{
	OSK_ASSERT(NULL != tim);
	init_timer(&tim->timer);
	OSK_DEBUG_CODE(	tim->active = MALI_FALSE );
	return OSK_ERR_NONE;
}

OSK_STATIC_INLINE osk_error osk_timer_start(osk_timer *tim, u32 delay)
{
	OSK_ASSERT(NULL != tim);
	OSK_ASSERT(NULL != tim->timer.function);
	OSK_ASSERT(0 != delay);
	tim->timer.expires = jiffies + (delay * HZ / 1000);
	add_timer(&tim->timer);
	OSK_DEBUG_CODE(	tim->active = MALI_TRUE );
	return OSK_ERR_NONE;
}

OSK_STATIC_INLINE void osk_timer_stop(osk_timer *tim)
{
	OSK_ASSERT(NULL != tim);
	OSK_ASSERT(NULL != tim->timer.function);
	del_timer_sync(&tim->timer);
	OSK_DEBUG_CODE( tim->active = MALI_FALSE );
}

OSK_STATIC_INLINE void osk_timer_callback_set(osk_timer *tim, osk_timer_callback callback, void *data)
{
	OSK_ASSERT(NULL != tim);
	OSK_ASSERT(NULL != callback);
	OSK_DEBUG_CODE(
		if (MALI_FALSE == tim->active)
		{
		}
	);
	/* osk_timer_callback uses void * for the callback parameter instead of unsigned long in Linux */
	tim->timer.function = (void (*)(unsigned long))callback;
	tim->timer.data = (unsigned long)data;
}

OSK_STATIC_INLINE void osk_timer_term(osk_timer *tim)
{
	OSK_ASSERT(NULL != tim);
	OSK_DEBUG_CODE(
		if (MALI_FALSE == tim->active)
		{
		}
	);
	/* Nothing to do */
}

#endif /* _OSK_ARCH_TIMERS_H_ */
