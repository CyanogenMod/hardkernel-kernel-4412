/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2010-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

#ifndef _KBASE_H_
#define _KBASE_H_

#include <malisw/mali_malisw.h>
#include <osk/mali_osk.h>
#include <uk/mali_ukk.h>

#include <base/mali_base_kernel.h>
#include <kbase/src/common/mali_kbase_uku.h>

#if defined(__KERNEL__) && defined(__linux__)
#include <kbase/src/linux/mali_kbase_linux.h>
#elif defined(MALI_KBASE_USERSPACE)
#include <kbase/src/userspace/mali_kbase_userspace.h>
#else
#error "Unspported OS"
#endif

#ifndef KBASE_OS_SUPPORT
#error Please fix for your platform!
#endif

#include "mali_kbase_pm.h"

#include "mali_kbase_defs.h"

#include "mali_kbase_js.h"

/**
 * @page page_base_kernel_main Kernel-side Base (KBase) APIs
 *
 * The Kernel-side Base (KBase) APIs are divided up as follows:
 * - @subpage page_kbase_js_policy
 */

/**
 * @defgroup base_kbase_api Kernel-side Base (KBase) APIs
 */


extern const kbase_device_info kbase_dev_info[];

kbase_device *kbase_device_create(const kbase_device_info *dev_info);
void kbase_device_destroy(kbase_device *kbdev);
int kbase_device_has_feature(kbase_device *kbdev, u32 feature);
kbase_midgard_type kbase_device_get_type(kbase_device *kbdev);

struct kbase_context *kbase_create_context(kbase_device *kbdev);
void kbase_destroy_context(kbase_context *kctx);

mali_error kbase_instr_hwcnt_setup(kbase_context * kctx, kbase_uk_hwcnt_setup * setup);
mali_error kbase_instr_hwcnt_dump(kbase_context * kctx);

mali_error kbase_create_os_context(kbase_os_context *osctx);
void kbase_destroy_os_context(kbase_os_context *osctx);

mali_error kbase_jd_init(struct kbase_context *kctx);
void kbase_jd_exit(struct kbase_context *kctx);
mali_error kbase_jd_submit(struct kbase_context *kctx, const kbase_uk_job_submit *user_bag);
void kbase_jd_done(kbase_jd_atom *katom);
void kbase_jd_cancel(kbase_jd_atom *katom);
void kbase_jd_flush_workqueues(kbase_context *kctx);
void kbase_jd_zap_context(kbase_context *kctx);

mali_error kbase_job_slot_init(kbase_device *kbdev);
void kbase_job_slot_exit(kbase_device *kbdev);
void kbase_job_done(kbase_device *kbdev, u32 done);
void kbase_job_submit(kbase_jd_atom *katom);
void kbase_job_submit_nolock(kbase_device *kbdev, kbase_jd_atom *katom, int js);
void kbase_job_zap_context(kbase_context *kctx);

void kbase_job_slot_softstop(kbase_device *kbdev, int js);
void kbase_job_slot_hardstop(kbase_device *kbdev, int js);

void kbase_event_post(kbase_context *ctx, kbase_event *event);
int kbase_event_dequeue(kbase_context *ctx, base_jd_event *uevent);
int kbase_event_pending(kbase_context *ctx);
mali_error kbase_event_init(kbase_context *kctx);
void kbase_event_close(kbase_context *kctx);
void kbase_event_cleanup(kbase_context *kctx);
void kbase_event_wakeup(kbase_context *kctx);

void kbase_power_on(kbase_device *kbdev);
void kbase_power_off(kbase_device *kbdev);

/* api used internally for register access. Contains validation and tracing */
void kbase_reg_write(kbase_device *kbdev, u16 offset, u32 value, kbase_context * kctx);
u32 kbase_reg_read(kbase_device *kbdev, u16 offset, kbase_context * kctx);
#if KBASE_REGISTER_TRACE_ENABLED
void kbase_device_trace_register_access(kbase_context * kctx, kbase_reg_access_type type, u16 reg_offset, u32 reg_value);
void kbase_device_trace_buffer_install(kbase_context * kctx, u32 * tb, size_t size);
void kbase_device_trace_buffer_uninstall(kbase_context * kctx);
#endif /* KBASE_REGISTER_TRACE_ENABLED */

/* api to be ported per OS, only need to do the raw register access */
void kbase_os_reg_write(kbase_device *kbdev, u16 offset, u32 value);
u32 kbase_os_reg_read(kbase_device *kbdev, u16 offset);

/** Report a GPU fault.
 *
 * This function is called from the interrupt handler when a GPU fault occurs.
 * It reports the details of the fault using OSK_PRINT_WARN.
 *
 * @param kbdev     The kbase device that the GPU fault occurred from.
 * @param multiple  Zero if only GPU_FAULT was raised, non-zero if MULTIPLE_GPU_FAULTS was also set
 */
void kbase_report_gpu_fault(kbase_device *kbdev, int multiple);

/** Kill all jobs that are currently running from a context
 *
 * This is used in response to a page fault to remove all jobs from the faulting context from the hardware.
 * 
 * @param kctx      The context to kill jobs from
 */
void kbase_job_kill_jobs_from_context(kbase_context *kctx);


#endif
