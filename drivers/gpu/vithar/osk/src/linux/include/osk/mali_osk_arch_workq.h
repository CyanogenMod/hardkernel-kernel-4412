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

#ifndef _OSK_ARCH_WORKQ_H_
#define _OSK_ARCH_WORKQ_H_

#include <linux/version.h> /* version detection */

OSK_STATIC_INLINE osk_error osk_workq_init(osk_workq * const wq, const char *name, u32 flags)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
	unsigned int wqflags = 0;
#endif
	OSK_ASSERT(0 == (flags & ~(OSK_WORKQ_NON_REENTRANT|OSK_WORKQ_HIGH_PRIORITY|OSK_WORKQ_RESCUER)));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
	if (flags & OSK_WORKQ_NON_REENTRANT)
	{
		wqflags |= WQ_NON_REENTRANT;
	}
	if (flags & OSK_WORKQ_HIGH_PRIORITY)
	{
		wqflags |= WQ_HIGHPRI;
	}
	if (flags & OSK_WORKQ_RESCUER)
	{
		wqflags |= WQ_RESCUER;
	}

	wq->wqs = alloc_workqueue(name, wqflags, 1);
#else
	if (flags & OSK_WORKQ_NON_REENTRANT)
	{
		wq->wqs = create_singlethread_workqueue(name);
	}
	else
	{
		wq->wqs = create_workqueue(name);
	}
#endif
	if (NULL == wq->wqs)
	{
		return OSK_ERR_FAIL;
	}

	return OSK_ERR_NONE;
}

OSK_STATIC_INLINE void osk_workq_term(osk_workq *wq)
{
	destroy_workqueue(wq->wqs);
}

OSK_STATIC_INLINE void osk_workq_submit(osk_workq *wq, osk_workq_fn fn, osk_workq_work * const wk)
{
	INIT_WORK(wk, fn);
	queue_work(wq->wqs, wk);
}

#endif
