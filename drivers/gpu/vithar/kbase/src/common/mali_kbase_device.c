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
 * @file mali_kbase_device.c
 * Base kernel device APIs
 */

#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_defs.h>

#define GPU_NUM_ADDRESS_SPACES 4
#define GPU_NUM_JOB_SLOTS 3


/* This array is referenced at compile time, it cannot be made static... */
const kbase_device_info kbase_dev_info[] = {
	{
		KBASE_MALI_T6XM,
		(KBASE_FEATURE_HAS_MODEL_PMU),
	},
	{
		KBASE_MALI_T6F1,
		(KBASE_FEATURE_NEEDS_REG_DELAY |
		 KBASE_FEATURE_DELAYED_PERF_WRITE_STATUS |
		 KBASE_FEATURE_HAS_16BIT_PC),
	},
	{
		KBASE_MALI_T601,
	},
	{
		KBASE_MALI_T604,
	},
	{
		KBASE_MALI_T608,
	},
};


static mali_error setup_page(kbase_device * kbdev, osk_phy_addr pt, u64 value)
{
	u64 * page;
	int i;

	page = osk_kmap(&kbdev->mmu_fault_allocator, pt);
	if (NULL == page)
	{
		return MALI_ERROR_FUNCTION_FAILED;
	}

	for (i = 0; i < 512; i++)
	{
		page[i] = value;
	}
	osk_sync_to_memory(pt, page, 512 * sizeof(u64));
	osk_kunmap(&kbdev->mmu_fault_allocator, pt);

	return MALI_ERROR_NONE;
}

static mali_error build_fault_directory(kbase_device * kbdev)
{
	int i;
	int last;
	osk_error osk_err;

	osk_err = osk_phy_allocator_init(&kbdev->mmu_fault_allocator, 0, 0);

	if (OSK_ERR_NONE != osk_err)
	{
		return MALI_ERROR_OUT_OF_MEMORY;
	}

	for (i = 0; i < 4; i++)
	{
		osk_err = osk_phy_pages_alloc(&kbdev->mmu_fault_allocator, 1, &kbdev->mmu_fault_pages[i]);
		if (OSK_ERR_NONE != osk_err)
		{
			goto failed;
		}
	}

	/* L3 */
	if (MALI_ERROR_NONE != setup_page(kbdev, kbdev->mmu_fault_pages[2], kbdev->mmu_fault_pages[3] | ENTRY_WR_BIT | ENTRY_RD_BIT | ENTRY_IS_ATE))
		goto failed;

	/* L2 */
	if (MALI_ERROR_NONE != setup_page(kbdev, kbdev->mmu_fault_pages[1], kbdev->mmu_fault_pages[2] | ENTRY_IS_PTE))
		goto failed;

	/* L1 */
	if (MALI_ERROR_NONE != setup_page(kbdev, kbdev->mmu_fault_pages[0], kbdev->mmu_fault_pages[1] | ENTRY_IS_PTE))
		goto failed;

	return MALI_ERROR_NONE;

failed:

	last = i;

	for (i = 0; i < last; i++)
	{
		osk_phy_pages_free(&kbdev->mmu_fault_allocator, 1, &kbdev->mmu_fault_pages[i]);
	}

	osk_phy_allocator_term(&kbdev->mmu_fault_allocator);

	return MALI_ERROR_OUT_OF_MEMORY;
}

static void destroy_fault_directory(kbase_device * kbdev)
{
	int i;

	for (i = 0; i < 4; i++)
	{
		osk_phy_pages_free(&kbdev->mmu_fault_allocator, 1, &kbdev->mmu_fault_pages[i]);
	}

	osk_phy_allocator_term(&kbdev->mmu_fault_allocator);
}

kbase_device *kbase_device_create(const kbase_device_info *dev_info)
{
	kbase_device *kbdev;
	osk_error osk_err;
	int i; /* i used after the for loop, don't reuse ! */

	kbdev = osk_calloc(sizeof(*kbdev));
	if (!kbdev)
	{
		goto fail;
	}

	kbdev->dev_info = dev_info;

	/* NOTE: Add Property Query here */
	kbdev->nr_address_spaces = GPU_NUM_ADDRESS_SPACES;
	kbdev->nr_job_slots = GPU_NUM_JOB_SLOTS;
	kbdev->job_slot_features[0] =
		  KBASE_JSn_FEATURE_NULL_JOB
		| KBASE_JSn_FEATURE_SET_VALUE_JOB
		| KBASE_JSn_FEATURE_CACHE_FLUSH_JOB
		| KBASE_JSn_FEATURE_FRAGMENT_JOB;
	kbdev->job_slot_features[1] =
		  KBASE_JSn_FEATURE_NULL_JOB
		| KBASE_JSn_FEATURE_SET_VALUE_JOB
		| KBASE_JSn_FEATURE_CACHE_FLUSH_JOB
		| KBASE_JSn_FEATURE_COMPUTE_JOB
		| KBASE_JSn_FEATURE_VERTEX_JOB
		| KBASE_JSn_FEATURE_GEOMETRY_JOB
		| KBASE_JSn_FEATURE_TILER_JOB
		| KBASE_JSn_FEATURE_FUSED_JOB;
	kbdev->job_slot_features[2] =
		  KBASE_JSn_FEATURE_NULL_JOB
		| KBASE_JSn_FEATURE_SET_VALUE_JOB
		| KBASE_JSn_FEATURE_CACHE_FLUSH_JOB
		| KBASE_JSn_FEATURE_COMPUTE_JOB
		| KBASE_JSn_FEATURE_VERTEX_JOB
		| KBASE_JSn_FEATURE_GEOMETRY_JOB;

	osk_err = osk_spinlock_irq_init(&kbdev->mmu_mask_change, OSK_LOCK_ORDER_MMU_MASK);
	if (OSK_ERR_NONE != osk_err)
	{
		goto free_dev;
	}

	for (i = 0; i < kbdev->nr_address_spaces; i++)
	{
		const char format[] = "mali_mmu%d";
		char name[sizeof(format)];

		if (0 > cutils_cstr_snprintf(name, sizeof(name), format, i))
		{
			goto free_workqs;
		}

		kbdev->as[i].number = i;
		kbdev->as[i].pf_busy_count = 0;
		kbdev->as[i].fault_addr = 0ULL;
		osk_err = osk_workq_init(&kbdev->as[i].pf_wq, name, 0);
		if (OSK_ERR_NONE != osk_err)
		{
			goto free_workqs;
		}
		osk_err = osk_spinlock_irq_init(&kbdev->as[i].transaction_lock, OSK_LOCK_ORDER_AS);
		if (OSK_ERR_NONE != osk_err)
		{
			osk_workq_term(&kbdev->as[i].pf_wq);
			goto free_workqs;
		}
	}
	/* don't change i after this point */

	osk_err = osk_spinlock_init(&kbdev->hwcnt_lock, OSK_LOCK_ORDER_HWCNT);
	if (OSK_ERR_NONE != osk_err)
	{
		goto free_workqs;
	}

	if (MALI_ERROR_NONE != build_fault_directory(kbdev))
	{
		goto free_hwcnt_lock;
	}
	
	if (OSK_ERR_NONE != osk_workq_init(&kbdev->reset_workq, "Mali reset workqueue", 0))
	{
		goto free_fault_directory;
	}

	return kbdev;

free_fault_directory:
	destroy_fault_directory(kbdev);
free_hwcnt_lock:
	osk_spinlock_term(&kbdev->hwcnt_lock);
free_workqs:
	while (i > 0)
	{
		i--;
		osk_spinlock_irq_term(&kbdev->as[i].transaction_lock);
		osk_workq_term(&kbdev->as[i].pf_wq);
	}
	osk_spinlock_irq_term(&kbdev->mmu_mask_change);
free_dev:
	osk_free(kbdev);
fail:
	return NULL;
}

void kbase_device_destroy(kbase_device *kbdev)
{
	int i;
	
	osk_workq_term(&kbdev->reset_workq);

	for (i = 0; i < kbdev->nr_address_spaces; i++)
	{
		osk_spinlock_irq_term(&kbdev->as[i].transaction_lock);
		osk_workq_term(&kbdev->as[i].pf_wq);
	}

	destroy_fault_directory(kbdev);

	osk_spinlock_term(&kbdev->hwcnt_lock);

	osk_free(kbdev);
}

int kbase_device_has_feature(kbase_device *kbdev, u32 feature)
{
	return !!(kbdev->dev_info->features & feature);
}

kbase_midgard_type kbase_device_get_type(kbase_device *kbdev)
{
	return kbdev->dev_info->dev_type;
}

#if KBASE_REGISTER_TRACE_ENABLED

void kbase_device_trace_buffer_install(kbase_context * kctx, u32 * tb, size_t size)
{
	OSK_ASSERT(kctx);
	OSK_ASSERT(tb);

	/* set up the header */
	/* magic number in the first 4 bytes */
	tb[0] = 0x45435254;
	/* Store (write offset = 0, wrap counter = 0, transaction active = no)
	 * write offset 0 means never written.
	 * Offsets 1 to (wrap_offset - 1) used to store values when trace started
	 */
	tb[1] = 0;

	/* install trace buffer */
	osk_spinlock_irq_lock(&kctx->jctx.tb_lock);
	kctx->jctx.tb_wrap_offset = size / 8;
	kctx->jctx.tb = tb;
	osk_spinlock_irq_unlock(&kctx->jctx.tb_lock);
}

void kbase_device_trace_buffer_uninstall(kbase_context * kctx)
{
	OSK_ASSERT(kctx);

	osk_spinlock_irq_lock(&kctx->jctx.tb_lock);
	kctx->jctx.tb = NULL;
	kctx->jctx.tb_wrap_offset = 0;
	osk_spinlock_irq_unlock(&kctx->jctx.tb_lock);
}

void kbase_device_trace_register_access(kbase_context * kctx, kbase_reg_access_type type, u16 reg_offset, u32 reg_value)
{
	osk_spinlock_irq_lock(&kctx->jctx.tb_lock);
	if (kctx->jctx.tb)
	{
		u16 wrap_count;
		u16 write_offset;
		osk_atomic dummy; /* osk_atomic_set called to use memory barriers until OSK get's them */
		u32 * tb = kctx->jctx.tb;
		u32 header_word;

		header_word = tb[1];
		OSK_ASSERT(0 == (header_word & 0x1));

		wrap_count = (header_word >> 1) & 0x7FFF;
		write_offset = (header_word >> 16) & 0xFFFF;

		/* mark as transaction in progress */
		tb[1] |= 0x1;
		osk_atomic_set(&dummy, 1);

		/* calculate new offset */
		write_offset++;
		if (write_offset == kctx->jctx.tb_wrap_offset)
		{
			/* wrap */
			write_offset = 1;
			wrap_count++;
			wrap_count &= 0x7FFF; /* 15bit wrap counter */
		}

		/* store the trace entry at the selected offset */
		tb[write_offset * 2 + 0] = (reg_offset & ~0x3) | ((type == REG_WRITE) ? 0x1 : 0x0);
		tb[write_offset * 2 + 1] = reg_value;
		
		osk_atomic_set(&dummy, 1);

		/* new header word */
		header_word = (write_offset << 16) | (wrap_count << 1) | 0x0; /* transaction complete */
		tb[1] = header_word;
	}
	osk_spinlock_irq_unlock(&kctx->jctx.tb_lock);
}
#endif /* KBASE_REGISTER_TRACE_ENABLED */

void kbase_reg_write(kbase_device *kbdev, u16 offset, u32 value, kbase_context * kctx)
{
	OSK_PRINT_INFO(OSK_BASE_CORE, "w: reg %04x val %08x", offset, value);
	kbase_os_reg_write(kbdev, offset, value);
#if KBASE_REGISTER_TRACE_ENABLED
	if (kctx) kbase_device_trace_register_access(kctx, REG_WRITE, offset, value);
#endif /* KBASE_REGISTER_TRACE_ENABLED */
}

u32 kbase_reg_read(kbase_device *kbdev, u16 offset, kbase_context * kctx)
{
	u32 val;
	val = kbase_os_reg_read(kbdev, offset);
	OSK_PRINT_INFO(OSK_BASE_CORE, "r: reg %04x val %08x", offset, val);
#if KBASE_REGISTER_TRACE_ENABLED
	if (kctx) kbase_device_trace_register_access(kctx, REG_READ, offset, val);
#endif /* KBASE_REGISTER_TRACE_ENABLED */
	return val;
}

void kbase_report_gpu_fault(kbase_device *kbdev, int multiple)
{
	u32 status;
	u64 address;

	status = kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_FAULTSTATUS), NULL);
	address = (u64)kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_FAULTADDRESS_HI), NULL) << 32;
	address |= kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_FAULTADDRESS_LO), NULL);

	OSK_PRINT_WARN(OSK_BASE_CORE, "GPU Fault 0x08%x at 0x%016llx", status, address);
	if (multiple)
	{
		OSK_PRINT_WARN(OSK_BASE_CORE, "There were multiple GPU faults - some have not been reported\n");
	}
}
