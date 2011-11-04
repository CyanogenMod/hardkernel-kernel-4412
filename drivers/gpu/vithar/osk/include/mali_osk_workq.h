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



#ifndef _OSK_WORKQ_H
#define _OSK_WORKQ_H

#ifndef _OSK_H_
#error "Include mali_osk.h directly"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/* pull in the arch header with the implementation  */
#include <osk/mali_osk_arch_workq.h>

/**
 * @addtogroup base_api
 * @{
 */

/**
 * @addtogroup base_osk_api
 * @{
 */

/**
 * @addtogroup oskworkqueue Work queue
 *  
 * A workqueue is a queue of functions that will be invoked by one or more worker threads
 * at some future time. Functions are invoked in FIFO order. Each function that is submitted
 * to the workqueue needs to be represented by a work unit (osk_workq_work). When a function
 * is invoked, a pointer to the work unit is passed to the invoked function. A work unit
 * needs to be embedded within the object that the invoked function needs to operate on, so
 * that the invoked function can determine a pointer to the object it needs to operate on.
 *
 * @{
 */

/**
 * @brief Initialize a work queue
 *
 * Initializes an empty work queue. One or more threads within the system will
 * be servicing the work units submitted to the work queue. 
 *
 * @param[out] wq    workqueue to initialize
 * @param[in] name   The name for the queue (may be visible in the process list)
 * @param[in] flags  flags specifying behavior of work queue, see OSK_WORKQ_ constants
 * @return OSK_ERR_NONE on success. Any other value indicates failure.
 */
OSK_STATIC_INLINE osk_error osk_workq_init(osk_workq * const wq, const char *name, u32 flags) CHECK_RESULT;

/**
 * @brief Terminate a work queue
 *
 * Stops accepting new work and waits until the work queue is empty and 
 * all work has been completed, then frees any resources allocated for the workqueue.
 *
 * @param[in] wq    intialized workqueue
 */
OSK_STATIC_INLINE void osk_workq_term(osk_workq *wq);

/**
 * @brief Submit work to a work queue
 *
 * Adds work (a work unit) to a work queue. 
 *
 * The work unit (osk_workq_work) represents a function \a fn to be invoked at some
 * future time. The invoked function \a fn is passed the pointer to the work unit \a wk.
 *
 * The work unit should be embedded within the object that the invoked function needs
 * to operate on, so that the invoked function can determine a pointer to the object
 * it needs to operate on.
 *
 * osk_workq_submit() must be callable from IRQ context (it may not block nor access user space)
 *
 * The work unit memory \a wk needs to remain allocated until the function \a fn has been invoked.
 * The work unit is an opaque storage holder and will be initialized by this function.
 *
 * The function \a fn needs to match the prototype: void fn(osk_workq_work *).
 *
 * @param[in] wq   intialized workqueue
 * @param[in] fn   function to be invoked at some future time
 * @param[out] wk  work unit to be initialized
 */
OSK_STATIC_INLINE void osk_workq_submit(osk_workq *wq, osk_workq_fn fn, osk_workq_work * const wk);

/** @} */ /* end group oskworkqueue */

/** @} */ /* end group base_osk_api */

/** @} */ /* end group base_api */


#ifdef __cplusplus
}
#endif

#endif /* _OSK_WORKQ_H */
