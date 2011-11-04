/*
 *
 * (C) COPYRIGHT 2010-2011 ARM Limited. All rights reserved.
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

#ifndef _OSK_ARCH_TYPES_H_
#define _OSK_ARCH_TYPES_H_

#include <linux/version.h> /* version detection */
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/mm_types.h>
#include <asm/atomic.h>
#include <linux/sched.h>

#include <malisw/mali_malisw.h>

 /* This will have to agree with the OSU definition of the CPU page size: CONFIG_CPU_PAGE_SIZE_LOG2 */
#define OSK_PAGE_SHIFT PAGE_SHIFT
#define OSK_PAGE_SIZE  PAGE_SIZE
#define OSK_PAGE_MASK  PAGE_MASK

#define OSK_MIN(x,y) min((x), (y))

typedef spinlock_t osk_spinlock;
typedef struct osk_spinlock_irq {
	spinlock_t	lock;
	unsigned long	flags;
} osk_spinlock_irq;

typedef struct mutex osk_mutex;
typedef struct rw_semaphore osk_rwlock;

typedef atomic_t osk_atomic;

typedef struct osk_waitq
{
       mali_bool signaled;    /**< set to MALI_TRUE when the waitq is signaled; set to MALI_FALSE when not signaled */
       wait_queue_head_t wq;  /**< threads waiting for flag to be signalled */
} osk_waitq;

typedef struct osk_timer {
	struct timer_list timer;
#ifdef MALI_DEBUG
	mali_bool active;
#endif
} osk_timer;

typedef struct page osk_page;
typedef struct vm_area_struct osk_vma;

typedef unsigned long osk_ticks; /* 32-bit resolution deemed to be sufficient */

typedef work_func_t		osk_workq_fn;
typedef struct work_struct	osk_workq_work;
typedef struct osk_workq
{
	struct workqueue_struct *wqs;
} osk_workq;

typedef struct device osk_power_info;


#endif /* _OSK_ARCH_TYPES_H_ */

