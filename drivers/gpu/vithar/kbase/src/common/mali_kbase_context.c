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
 * @file mali_kbase_context.c
 * Base kernel context APIs
 */

#include <kbase/src/common/mali_kbase_mem.h>
#include <kbase/src/common/mali_midg_regmap.h>

/**
 * @brief Create a kernel base context.
 *
 * Allocate and init a kernel base context. Calls
 * kbase_create_os_context() to setup OS specific structures.
 */
struct kbase_context *kbase_create_context(kbase_device *kbdev)
{
	struct kbase_context *kctx;
	struct kbase_va_region *pmem_reg;
	struct kbase_va_region *tmem_reg;
	osk_error osk_err;
	mali_error mali_err;

	kctx = osk_calloc(sizeof(*kctx));
	if (!kctx)
		goto out;

	kctx->kbdev = kbdev;
	kctx->as_nr = -1;

	if (kbase_jd_init(kctx))
		goto free_kctx;

	mali_err = kbasep_js_kctx_init( kctx );
	if ( MALI_ERROR_NONE != mali_err )
	{
		goto free_jd; /* safe to call kbasep_js_kctx_term  in this case */
	}

	mali_err = kbase_event_init(kctx);
	if (MALI_ERROR_NONE != mali_err)
		goto free_jd;

	osk_err = osk_mutex_init(&kctx->reg_lock, OSK_LOCK_ORDER_MEM_REG);
	if (OSK_ERR_NONE != osk_err)
		goto free_event;

	OSK_DLIST_INIT(&kctx->reg_list);

	osk_err = osk_phy_allocator_init(&kctx->pgd_allocator, 0, 0);
	if (OSK_ERR_NONE != osk_err)
		goto free_region_lock;

	kctx->pgd = kbase_mmu_alloc_pgd(kctx);
	if (!kctx->pgd)
		goto free_phy;
	
	if (kbase_create_os_context(&kctx->osctx))
		goto free_pgd;

	/* Make sure page 0 is not used... */
	pmem_reg = kbase_alloc_free_region(kctx, 1,
	                                   KBASE_REG_ZONE_TMEM_BASE - 1, KBASE_REG_ZONE_PMEM);
	tmem_reg = kbase_alloc_free_region(kctx, KBASE_REG_ZONE_TMEM_BASE,
	                                   KBASE_REG_ZONE_TMEM_SIZE, KBASE_REG_ZONE_TMEM);

	if (!pmem_reg || !tmem_reg)
	{
		if (pmem_reg)
			kbase_free_alloced_region(pmem_reg);
		if (tmem_reg)
			kbase_free_alloced_region(tmem_reg);

		kbase_destroy_context(kctx);
		return NULL;
	}

	OSK_DLIST_PUSH_FRONT(&kctx->reg_list, pmem_reg, struct kbase_va_region, link);
	OSK_DLIST_PUSH_BACK(&kctx->reg_list, tmem_reg, struct kbase_va_region, link);

	return kctx;

free_pgd:
	kbase_mmu_free_pgd(kctx);
free_phy:
	osk_phy_allocator_term(&kctx->pgd_allocator);
free_region_lock:
	osk_mutex_term(&kctx->reg_lock);
free_event:
	kbase_event_cleanup(kctx);
free_jd:
	/* Safe to call this one even when didn't initialize (assuming kctx was sufficiently zeroed) */
	kbasep_js_kctx_term(kctx);
	kbase_jd_exit(kctx);
free_kctx:
	osk_free(kctx);
out:
	return NULL;
	
}

/**
 * @brief Destroy a kernel base context.
 *
 * Destroy a kernel base context. Calls kbase_destroy_os_context() to
 * free OS specific structures. Will release all outstanding regions.
 */
void kbase_destroy_context(struct kbase_context *kctx)
{
	/* Ensure the core is powered up for the destroy process */
	kbase_pm_context_active(kctx->kbdev);

	kbase_jd_zap_context(kctx);
	kbase_event_cleanup(kctx);
	kbase_gpu_vm_lock(kctx);
	if (kctx->kbdev->hwcnt_context == kctx)
	{
		/* disable the use of the hw counters if the app didn't use the API correctly or crashed */
		kbase_uk_hwcnt_setup tmp;
		tmp.dump_buffer = 0ull;
		kbase_instr_hwcnt_setup(kctx, &tmp);
	}

	/* MMU is disabled as part of scheduling out the context */
	kbase_mmu_free_pgd(kctx);
	osk_phy_allocator_term(&kctx->pgd_allocator);
	OSK_DLIST_EMPTY_LIST(&kctx->reg_list, struct kbase_va_region,
	                     link, kbase_free_alloced_region);
	kbase_destroy_os_context(&kctx->osctx);
	kbase_gpu_vm_unlock(kctx);

	/* Safe to call this one even when didn't initialize (assuming kctx was sufficiently zeroed) */
	kbasep_js_kctx_term(kctx);

	kbase_jd_exit(kctx);
	osk_mutex_term(&kctx->reg_lock);

	kbase_pm_context_idle(kctx->kbdev);

	osk_free(kctx);
}

mali_error kbase_instr_hwcnt_setup(kbase_context * kctx, kbase_uk_hwcnt_setup * setup)
{
	mali_error err = MALI_ERROR_NONE; /* let's be optimistic */

	OSK_ASSERT(NULL != kctx);
	OSK_ASSERT(NULL != setup);

	if (setup->dump_buffer & (2048-1))
	{
		/* alignment failure */
		return MALI_ERROR_FUNCTION_FAILED;
	}

	osk_spinlock_lock(&kctx->kbdev->hwcnt_lock);

	if ((setup->dump_buffer != 0ULL) && (NULL == kctx->kbdev->hwcnt_context))
	{
		/* enable request */

		/* clean&invalidate the caches so we're sure the mmu tables for the dump buffer is valid */
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(GPU_COMMAND),        8,                               kctx);
		/* NOTE: PRLAM-5316 created as there is no way to know when the command has completed */

		/* configure */
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(PRFCNT_BASE_LO),     setup->dump_buffer & 0xFFFFFFFF, kctx);
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(PRFCNT_BASE_HI),     setup->dump_buffer >> 32,        kctx);
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(PRFCNT_JM_EN),       setup->jm_bm,                    kctx);
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(PRFCNT_SHADER_EN),   setup->shader_bm,                kctx);
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(PRFCNT_TILER_EN),    setup->tiler_bm,                 kctx);
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(PRFCNT_L3_CACHE_EN), setup->l3_cache_bm,              kctx);
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(PRFCNT_MMU_L2_EN),   setup->mmu_l2_bm,                kctx);

		/* start collecting */
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(PRFCNT_CONFIG),      (kctx->as_nr << 4) | 1,          kctx);

		/* mark as in use and that this context is the owner */
		kctx->kbdev->hwcnt_context = kctx;
		/* remember the dump address so we can reprogram it later */
		kctx->kbdev->hwcnt_addr = setup->dump_buffer;
		/* now ready to accept dump requests */
	}
	else if ((setup->dump_buffer == 0ULL) && (kctx == kctx->kbdev->hwcnt_context))
	{
		/* disable request and is the owner */
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(PRFCNT_CONFIG), 0, kctx);
		kctx->kbdev->hwcnt_context = NULL;
		kctx->kbdev->hwcnt_addr = 0ULL;
	}
	else
	{
		/* already in use or trying to disable while not owning */
		err = MALI_ERROR_FUNCTION_FAILED;
	}

	osk_spinlock_unlock(&kctx->kbdev->hwcnt_lock);

	return err;
}

mali_error kbase_instr_hwcnt_dump(kbase_context * kctx)
{
	mali_error err = MALI_ERROR_FUNCTION_FAILED;
	OSK_ASSERT(NULL != kctx);
	osk_spinlock_lock(&kctx->kbdev->hwcnt_lock);

#define WAIT_HI_LOOPS 10
#define WAIT_LOW_LOOPS 10000000ULL

	if (kctx->kbdev->hwcnt_context == kctx)
	{
		u64 i;

		/* mark that we're dumping so the PF handler can signal that we faulted */
		kctx->kbdev->hwcnt_in_progress = MALI_TRUE;

		/* dump */
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(GPU_COMMAND), 4, kctx);

		if (kbase_device_has_feature(kctx->kbdev, KBASE_FEATURE_DELAYED_PERF_WRITE_STATUS))
		{
			/* the model nees a few cycles to set the bit to indicate it's dumping */
			for (i = 0; i < WAIT_HI_LOOPS; i++)
			{
				if ((kbase_reg_read(kctx->kbdev, GPU_CONTROL_REG(GPU_STATUS), kctx) & (1<<2)))
				{
					break;
				}
			}
			if (WAIT_HI_LOOPS == i)
			{
				OSK_PRINT_WARN(OSK_BASE_CORE, "Bit didn't go high, failed to dump hardware counters\n");
				goto abort_dump;
			}
		}

		/* wait for write to complete */
		for ( i = 0; i < WAIT_LOW_LOOPS; i++)
		{
			if ( 0 == (kbase_reg_read(kctx->kbdev, GPU_CONTROL_REG(GPU_STATUS), kctx) & (1<<2)))
			{
				break;
			}
		}

		if (WAIT_LOW_LOOPS == i)
		{
			OSK_PRINT_WARN(OSK_BASE_CORE, "Dump active bit stuck high, failed to dump hardware counters\n");
			goto abort_dump;
		}

		/* if hwcnt_in_progress is now MALI_FALSE we have faulted in some way */
		if (kctx->kbdev->hwcnt_in_progress)
		{
			/* clear cache to commit the samples to main memory */
			kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(GPU_COMMAND), 7, kctx);
			/* NOTE: PRLAM-5316 created as there is no way to know that the cache clear has completed */

			/* success */
			err = MALI_ERROR_NONE;

			/* clear the mark for next time */
			kctx->kbdev->hwcnt_in_progress = MALI_FALSE;
		}

		/* reconfigure the dump address */
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(PRFCNT_BASE_LO), kctx->kbdev->hwcnt_addr & 0xFFFFFFFF, kctx);
		kbase_reg_write(kctx->kbdev, GPU_CONTROL_REG(PRFCNT_BASE_HI), kctx->kbdev->hwcnt_addr >> 32,        kctx);
	}
abort_dump:
	osk_spinlock_unlock(&kctx->kbdev->hwcnt_lock);

	return err;
}
