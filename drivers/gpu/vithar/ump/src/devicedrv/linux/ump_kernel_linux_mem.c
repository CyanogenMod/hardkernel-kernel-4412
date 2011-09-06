/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

#include <uk/mali_ukk.h>

#include <ump/ump_kernel_interface.h>
#include <ump/src/devicedrv/linux/ump_arch_mem_common.h>

#include <linux/module.h>            /* kernel module definitions */
#include <linux/fs.h>                /* file system operations */
#include <linux/cdev.h>              /* character device definitions */
#include <linux/ioport.h>            /* request_mem_region */
#include <linux/mm.h> /* memory mananger definitions */
#include <linux/pfn.h>

#include <linux/compat.h> /* is_compat_task */

#include <common/ump_kernel_core.h>
#include <ump_arch.h>


static void umpp_vm_close(struct vm_area_struct *vma)
{
	umpp_cpu_mapping * mapping;
	umpp_session * session;
	ump_dd_handle handle;

	mapping = (umpp_cpu_mapping*)vma->vm_private_data;
	session = mapping->session;
	handle = mapping->handle;

	osk_mutex_lock(&session->session_lock);
	umpp_dd_remove_cpu_mapping(mapping->handle, mapping); /* will free the mapping object */
	osk_mutex_unlock(&session->session_lock);

	ump_dd_unpin_size(handle);
	ump_dd_release(handle);
}


static const struct vm_operations_struct umpp_vm_ops = {
	.close = umpp_vm_close
};

int umpp_phys_commit(umpp_allocation * alloc)
{
	u64 i;

	/* round up to a page boundary */
	alloc->size = (alloc->size + PAGE_SIZE - 1) & ~(PAGE_SIZE-1);
	/* calculate number of pages */
	alloc->blocksCount = alloc->size >> PAGE_SHIFT;

	alloc->block_array = kmalloc(sizeof(ump_dd_physical_block) * alloc->blocksCount, GFP_KERNEL | __GFP_NORETRY | __GFP_NOWARN);
	if (NULL == alloc->block_array)
	{
		return -ENOMEM;
	}

	for (i = 0; i < alloc->blocksCount; i++)
	{
		void * mp;
		struct page * page = alloc_page(GFP_HIGHUSER | __GFP_NORETRY | __GFP_NOWARN | __GFP_COLD);
		if (NULL == page) break;

		alloc->block_array[i].addr = page_to_pfn(page) << PAGE_SHIFT;
		alloc->block_array[i].size = PAGE_SIZE;

		mp = kmap(page);
		if (NULL == mp)
		{
			__free_page(page);
			break;
		}

		memset(mp, 0x00, PAGE_SIZE); /* instead of __GFP_ZERO, so we can do cache maintenance */
		osk_sync_to_memory(PFN_PHYS(page_to_pfn(page)), mp, PAGE_SIZE);
		kunmap(page);
	}

	if (i == alloc->blocksCount)
	{
		return 0;
	}
	else
	{
		u64 j;
		for (j = 0; j < i; j++)
		{
			struct page * page;
			page = pfn_to_page(alloc->block_array[j].addr >> PAGE_SHIFT);
			__free_page(page);
		}
		
		kfree(alloc->block_array);

		return -ENOMEM;
	}
}


ump_resize_result umpp_phys_recommit(umpp_allocation * alloc, s64 size_diff, u64 * out_new_size)
{
	ump_dd_physical_block * new_block_array;
	u64 new_count;
	s64 page_diff;
	u64 new_size;

	if (size_diff < 0)
	{
		size_diff = -size_diff;
		page_diff = -(size_diff >> PAGE_SHIFT);
	}
	else
	{
		size_diff = (size_diff + (PAGE_SIZE - 1)) & ~(PAGE_SIZE - 1);
		page_diff = size_diff >> PAGE_SHIFT;
	}

	if (0 == page_diff)
	{
		/* no change */
		*out_new_size = alloc->size;
		return UMP_RESIZE_OK;
	}

	new_count = alloc->blocksCount + page_diff;
	new_size = new_count << PAGE_SHIFT;

	new_block_array = kmalloc(sizeof(ump_dd_physical_block) * new_count, GFP_KERNEL | __GFP_NORETRY | __GFP_NOWARN);
	if (NULL == new_block_array) return UMP_RESIZE_ERROR_OOM;

	if (page_diff < 0)
	{
		u64 i;
		/* shrinking, just memcpy over the page info from the old array and free the remaining pages */
		memcpy(new_block_array, alloc->block_array, sizeof(ump_dd_physical_block) * new_count);
		for (i = new_count; i < alloc->blocksCount; i++)
		{
			struct page * page;
			page = pfn_to_page(alloc->block_array[i].addr >> PAGE_SHIFT);
			__free_page(page);
		}
		/* resize complete */
	}
	else
	{
		u64 i;

		/* copy over the existing allocation info */
		memcpy(new_block_array, alloc->block_array, sizeof(ump_dd_physical_block) * alloc->blocksCount);

		/* allocate the new pages */
		for (i = alloc->blocksCount; i < new_count; i++)
		{
			struct page * page = alloc_page(GFP_HIGHUSER | __GFP_ZERO | __GFP_NORETRY | __GFP_NOWARN | __GFP_COLD);
			if (NULL == page) break;

			new_block_array[i].addr = page_to_pfn(page) << PAGE_SHIFT;
			new_block_array[i].size = PAGE_SIZE;
		}

		if (i != new_count)
		{
			/* one of the page allocs failed, release the newly allocated pages */
			u64 j;
			for (j = i - 1; j >= alloc->blocksCount; j--)
			{
				struct page * page;
				page = pfn_to_page(new_block_array[j].addr >> PAGE_SHIFT);
				__free_page(page);
			}

			/* free the new block array */
			kfree(new_block_array);

			return UMP_RESIZE_ERROR_OOM;
		}
	}

	/* free the old block array */
	kfree(alloc->block_array);
	/* update alloc with the new information */
	alloc->block_array = new_block_array;
	alloc->blocksCount = new_count;
	alloc->size = new_size;
	*out_new_size = new_size;

	return UMP_RESIZE_OK;
}

void umpp_phys_free(umpp_allocation * alloc)
{
	u64 i;

	for (i = 0; i < alloc->blocksCount; i++)
	{
		__free_page(pfn_to_page(alloc->block_array[i].addr >> PAGE_SHIFT));
	}

	kfree(alloc->block_array);
}

int umpp_linux_mmap(struct file * filp, struct vm_area_struct * vma)
{
	ump_secure_id id;
	ump_dd_handle h;
	size_t offset;
	int err = -EINVAL;

	umpp_cpu_mapping * map = NULL;
	umpp_session *session = filp->private_data;

	map = osk_calloc(sizeof(*map));
	if (NULL == map)
	{
		WARN_ON(1);
		err = -ENOMEM;
		goto out;
	}

	/* unpack our arg */
#if defined CONFIG_64BIT && CONFIG_64BIT
	if (is_compat_task())
	{
#endif
		id = vma->vm_pgoff >> UMP_LINUX_OFFSET_BITS_32;
		offset = vma->vm_pgoff & UMP_LINUX_OFFSET_MASK_32;
#if defined CONFIG_64BIT && CONFIG_64BIT
	}
	else
	{
		id = vma->vm_pgoff >> UMP_LINUX_OFFSET_BITS_64;
		offset = vma->vm_pgoff & UMP_LINUX_OFFSET_MASK_64;
	}
#endif

	h = ump_dd_from_secure_id(id);
	if (UMP_DD_INVALID_MEMORY_HANDLE != h)
	{
		u64 i;
		u64 block_idx;
		u64 block_offset;
		osk_phy_addr paddr;
		umpp_allocation * alloc;
		u64 last_byte;

		size_t length = vma->vm_end - vma->vm_start;

		alloc = (umpp_allocation*)h;

		ump_dd_pin_size(h);

		last_byte = length + (offset << PAGE_SHIFT) - 1;
		if (last_byte >= alloc->size || last_byte < (offset << PAGE_SHIFT))
		{
			goto err_out;
		}

		if (MALI_ERROR_NONE != umpp_dd_find_start_block(alloc, offset << PAGE_SHIFT, &block_idx, &block_offset))
		{
			goto err_out;
		}

		paddr = alloc->block_array[block_idx].addr + block_offset;

		for (i = 0; i < (length >> PAGE_SHIFT); i++)
		{
			/* check if we've overrrun the current block, if so move to the next block */
			if (paddr >= (alloc->block_array[block_idx].addr + alloc->block_array[block_idx].size))
			{
				block_idx++;
				OSK_ASSERT(block_idx < alloc->blocksCount);
				paddr = alloc->block_array[block_idx].addr;
			}

			err = vm_insert_page(vma, vma->vm_start + (i << PAGE_SHIFT), pfn_to_page(paddr>>PAGE_SHIFT));
			paddr += PAGE_SIZE;
		}

		map->vaddr_start = (osk_virt_addr)vma->vm_start;
		map->nr_pages = length >> PAGE_SHIFT;
		map->page_off = offset;
		map->handle = h;
		map->session = session;

		osk_mutex_lock(&session->session_lock);
		umpp_dd_add_cpu_mapping(h, map);
		osk_mutex_unlock(&session->session_lock);

		vma->vm_flags |= VM_DONTCOPY | VM_DONTEXPAND | VM_RESERVED | VM_IO;
		vma->vm_ops = &umpp_vm_ops;
		vma->vm_private_data = map;

		return 0;

		err_out:

		ump_dd_unpin_size(h);
		ump_dd_release(h);
	}

	osk_free(map);

out:

	return err;
}

