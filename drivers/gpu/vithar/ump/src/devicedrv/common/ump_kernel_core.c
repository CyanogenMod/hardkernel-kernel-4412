/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/* module headers */
#include <osu/mali_osu_globalenums.h>
#include <osk/mali_osk.h>
#include <uk/mali_ukk.h>
#include <ump/ump_kernel_interface.h>
#include <ump/src/ump_uku.h>

/* local headers */
#include <common/ump_kernel_core.h>
#include <common/ump_kernel_uk.h>
#include <common/ump_kernel_descriptor_mapping.h>
#include <ump_arch.h>

#define UMP_EXPECTED_IDS 64
#define UMP_MAX_IDS 32768

static umpp_device device;

ump_result umpp_core_constructor(void)
{
	if (0 == osk_rwlock_init(&device.resize_lock, OSU_LOCK_ORDER_UMP_RESIZE_LOCK))
	{
		if (0 == osk_mutex_init(&device.secure_id_map_lock, OSU_LOCK_ORDER_UMP_IDMAP_LOCK))
		{
			device.secure_id_map = umpp_descriptor_mapping_create(UMP_EXPECTED_IDS, UMP_MAX_IDS);
			if (NULL != device.secure_id_map)
			{
				if (UMP_OK == umpp_device_initialize())
				{
					return UMP_OK;
				}
				umpp_descriptor_mapping_destroy(device.secure_id_map);
				}
			osk_mutex_term(&device.secure_id_map_lock);
		}
		osk_rwlock_term(&device.resize_lock);
	}
	return UMP_ERROR;
}

void umpp_core_destructor(void)
{
	umpp_device_terminate();
	umpp_descriptor_mapping_destroy(device.secure_id_map);
	osk_mutex_term(&device.secure_id_map_lock);
	osk_rwlock_term(&device.resize_lock);
}

umpp_session *umpp_core_session_start(void)
{
	umpp_session * session;

	session = osk_malloc(sizeof(*session));
	if (NULL != session)
	{
		if (OSK_ERR_NONE == osk_mutex_init(&session->session_lock, OSU_LOCK_ORDER_UMP_SESSION_LOCK))
		{
			OSK_DLIST_INIT(&session->memory_usage);

			if (MALI_ERROR_NONE == ukk_session_init(&session->ukk_session, ump_dispatch, UMP_VERSION_MAJOR, UMP_VERSION_MINOR))
			{
				return session;
			}

			osk_mutex_term(&session->session_lock);
		}

		osk_free(session);
	}
	return NULL;
}

void umpp_core_session_end(umpp_session *session)
{
	OSK_ASSERT(session);

	ukk_session_term(&session->ukk_session);

	while (!OSK_DLIST_IS_EMPTY(&session->memory_usage))
	{
		umpp_session_memory_usage * usage;
		usage = OSK_DLIST_POP_FRONT(&session->memory_usage, umpp_session_memory_usage, link);
		OSK_PRINT_WARN(OSK_UMP, "Memory usage cleanup, releasing secure ID %d\n", ump_dd_secure_id_get(usage->mem));
		ump_dd_release(usage->mem);
		osk_free(usage);
	}

	osk_mutex_term(&session->session_lock);
	osk_free(session);
}

ump_dd_handle ump_dd_allocate(u64 size, ump_alloc_flags flags, ump_dd_security_filter filter_func, ump_dd_final_release_callback final_release_func, void* callback_data)
{
	umpp_allocation * alloc;

	alloc = osk_calloc(sizeof(*alloc));

	if (NULL == alloc)
		goto out1;

	alloc->flags = flags;
	alloc->filter_func = filter_func;
	alloc->final_release_func = final_release_func;
	alloc->callback_data = callback_data;
	alloc->size = size;

	osk_atomic_set(&alloc->refcount, 1);

	if (flags & UMP_ALLOCATE_AS_SIZE_PINNED)
	{
		osk_atomic_set(&alloc->pincount, 1);
	}
	else
	{
		osk_atomic_set(&alloc->pincount, 0);
	}

	/* NOTE: Missing implementation for non-shareable allocations, see MIDBASE-522 */

	if (0 != umpp_phys_commit(alloc))
	{
		goto out2;
	}

	/* all set up, allocate an ID for it */

	osk_mutex_lock(&device.secure_id_map_lock);
	alloc->id = umpp_descriptor_mapping_allocate(device.secure_id_map, (void*)alloc);
	osk_mutex_unlock(&device.secure_id_map_lock);

	if ((int)alloc->id == 0)
	{
		/* failed to allocate a secure_id */
		goto out3;
	}

	return alloc;

out3:
	umpp_phys_free(alloc);
out2:
	osk_free(alloc);
out1:
	return UMP_DD_INVALID_MEMORY_HANDLE;
}

u64 ump_dd_size_get(const ump_dd_handle mem)
{
	umpp_allocation * alloc;

	OSK_ASSERT(mem);

	alloc = (umpp_allocation*)mem;

	return alloc->size;
}

ump_secure_id ump_dd_secure_id_get(const ump_dd_handle mem)
{
	umpp_allocation * alloc;

	OSK_ASSERT(mem);

	alloc = (umpp_allocation*)mem;

	return alloc->id;
}

ump_alloc_flags ump_dd_allocation_flags_get(const ump_dd_handle mem)
{
	const umpp_allocation * alloc;
	OSK_ASSERT(mem);
	alloc = (const umpp_allocation *)mem;

	return alloc->flags;
}

ump_resize_result ump_dd_resize(ump_dd_handle mem, s64 size_diff, u64 * new_size)
{
	umpp_allocation * alloc;
	ump_resize_result res;
	
	OSK_ASSERT(mem);

	alloc = (umpp_allocation*)mem;

	if (alloc->management_flags & UMP_MGMT_EXTERNAL)
	{
		return UMP_RESIZE_ERROR_NOT_PERMITTED;
	}

	/* check for invalid handle flags */
	if (alloc->flags & UMP_PROT_SHAREABLE)
	{
		return UMP_RESIZE_ERROR_SHARED;
	}
	if (alloc->flags & UMP_CONSTRAINT_PHYSICALLY_LINEAR)
	{
		return UMP_RESIZE_ERROR_PHYSICALLY_CONTIGUOUS;
	}

	osk_rwlock_write_lock(&device.resize_lock);

	if ( size_diff < 0 && ( alloc->size < (u64)-size_diff || size_diff == S64_MIN ))
	{
		size_diff = -alloc->size; /* don't allow the alloc size to become negative */
	}

	if (0 != osk_atomic_get(&alloc->pincount))
	{
		res = UMP_RESIZE_ERROR_PINNED;
	}
	else
	{
		res = umpp_phys_recommit(mem, size_diff, new_size);
	}

	osk_rwlock_write_unlock(&device.resize_lock);

	return res;
}

ump_dd_handle ump_dd_from_secure_id(ump_secure_id secure_id)
{
	umpp_allocation * alloc = UMP_DD_INVALID_MEMORY_HANDLE;

	osk_mutex_lock(&device.secure_id_map_lock);

	if (0 == umpp_descriptor_mapping_lookup(device.secure_id_map, secure_id, (void**)&alloc))
	{
		if (NULL != alloc->filter_func)
		{
			if (!alloc->filter_func(secure_id, alloc, alloc->callback_data))
			{
				alloc = UMP_DD_INVALID_MEMORY_HANDLE; /* the filter denied access */
			}
		}

		if (UMP_DD_INVALID_MEMORY_HANDLE != alloc)
		{
			ump_dd_retain(alloc);
		}
	}

	osk_mutex_unlock(&device.secure_id_map_lock);

	return alloc;
}

void ump_dd_retain(ump_dd_handle mem)
{
	umpp_allocation * alloc;

	OSK_ASSERT(mem);

	alloc = (umpp_allocation*)mem;

	osk_atomic_inc(&alloc->refcount);
}

void ump_dd_release(ump_dd_handle mem)
{
	umpp_allocation * alloc;
	u32 new_cnt;

	OSK_ASSERT(mem);

	alloc = (umpp_allocation*)mem;

	/* secure the id for lookup while releasing */
	osk_mutex_lock(&device.secure_id_map_lock);

	/* do the actual release */
	new_cnt = osk_atomic_dec(&alloc->refcount);
	if (0 == new_cnt)
	{
		/* remove from the table as this was the last ref */
		umpp_descriptor_mapping_remove(device.secure_id_map, alloc->id);
	}

	/* release the lock as early as possible */
	osk_mutex_unlock(&device.secure_id_map_lock);

	if (0 != new_cnt)
	{
		/* exit if still have refs */
		return;
	}

	/* cleanup */
	if (NULL != alloc->final_release_func)
	{
		alloc->final_release_func(alloc, alloc->callback_data);
	}

	if (0 == (alloc->management_flags & UMP_MGMT_EXTERNAL))
	{
		umpp_phys_free(alloc);
	}

	osk_free(alloc);
}

void ump_dd_pin_size(ump_dd_handle mem)
{
	umpp_allocation * alloc = (umpp_allocation*)mem;
	OSK_ASSERT(mem);
	osk_rwlock_read_lock(&device.resize_lock);
	osk_atomic_inc(&alloc->pincount);
	osk_rwlock_read_unlock(&device.resize_lock);
}

void ump_dd_unpin_size(ump_dd_handle mem)
{
	umpp_allocation * alloc = (umpp_allocation*)mem;
	OSK_ASSERT(mem);
	osk_rwlock_read_lock(&device.resize_lock);
	osk_atomic_dec(&alloc->pincount);
	osk_rwlock_read_unlock(&device.resize_lock);
}

void ump_dd_phys_blocks_get(const ump_dd_handle mem, u64 * pCount, const ump_dd_physical_block ** pArray)
{
	const umpp_allocation * alloc;
	OSK_ASSERT(pCount);
	OSK_ASSERT(pArray);
	OSK_ASSERT(mem);
	alloc = (const umpp_allocation *)mem;
	*pCount = alloc->blocksCount;
	*pArray = alloc->block_array;
}



umpp_cpu_mapping * umpp_dd_find_enclosing_mapping(umpp_allocation * alloc, osk_virt_addr uaddr, size_t size)
{
	umpp_cpu_mapping *map;

	osk_virt_addr target_first = uaddr;
	osk_virt_addr target_last = (osk_virt_addr)((uintptr_t)uaddr - 1 + size);

	if (target_last < target_first) /* wrapped */
	{
		return NULL;
	}

	OSK_DLIST_FOREACH(&alloc->map_list, umpp_cpu_mapping, link, map)
	{
		if ( map->vaddr_start <= target_first &&
		   (osk_virt_addr)((uintptr_t)map->vaddr_start + (map->nr_pages << OSK_PAGE_SHIFT) - 1) >= target_last)
		{
			return map;
		}
	}

	return NULL;
}

void umpp_dd_add_cpu_mapping(umpp_allocation * alloc, umpp_cpu_mapping * map)
{
	OSK_DLIST_PUSH_FRONT(&alloc->map_list, map, umpp_cpu_mapping, link);
}

void umpp_dd_remove_cpu_mapping(umpp_allocation * alloc, umpp_cpu_mapping * target)
{
	umpp_cpu_mapping * map;

	OSK_DLIST_FOREACH(&alloc->map_list, umpp_cpu_mapping, link, map)
	{
		if (map == target)
		{
			OSK_DLIST_REMOVE(&alloc->map_list, target, link);
			osk_free(target);
			return;
		}
	}

	/* not found, error */
	OSK_ASSERT(0);
}

mali_error umpp_dd_find_start_block(const umpp_allocation * alloc, u64 offset, u64 * block_index, u64 * block_internal_offset)
{
	u64 i;

	for (i = 0 ; i < alloc->blocksCount; i++)
	{
		if (offset < alloc->block_array[i].size)
		{
			/* found the block_array element containing this offset */
			*block_index = i;
			*block_internal_offset = offset;
			return MALI_ERROR_NONE;
		}
		offset -= alloc->block_array[i].size;
	}

	return MALI_ERROR_FUNCTION_FAILED;
}

void umpp_dd_cpu_msync_now(ump_dd_handle mem, ump_cpu_msync_op op, void * address, size_t size)
{
	umpp_allocation * alloc;
	osk_virt_addr vaddr;
	umpp_cpu_mapping * mapping;
	u64 virt_page_off; /* offset of given address from beginning of the virtual mapping */
	u64 phys_page_off; /* offset of the virtual mapping from the beginning of the physical buffer */
	u64 page_count; /* number of pages to sync */
	u64 i;
	u64 block_idx;
	u64 block_offset;
	osk_phy_addr  paddr;

	OSK_ASSERT((UMP_MSYNC_CLEAN == op) || (UMP_MSYNC_CLEAN_AND_INVALIDATE == op));

	alloc = (umpp_allocation*)mem;
	vaddr = (osk_virt_addr)(uintptr_t)address;

	mapping = umpp_dd_find_enclosing_mapping(alloc, vaddr, size);
	if (NULL == mapping)
	{
		OSK_PRINT_WARN(OSK_UMP, "Illegal cache sync address %llx\n", (uintptr_t)vaddr);
		return; /* invalid pointer or size causes out-of-bounds */
	}

	/* we already know that address + size don't wrap around as umpp_dd_find_enclosing_mapping didn't fail */
	page_count = ((((((uintptr_t)address + size - 1) & OSK_PAGE_MASK) - ((uintptr_t)address & OSK_PAGE_MASK))) >> OSK_PAGE_SHIFT) + 1;
	virt_page_off = (vaddr - mapping->vaddr_start) >> OSK_PAGE_SHIFT;
	phys_page_off = mapping->page_off;

	if (MALI_ERROR_NONE != umpp_dd_find_start_block(alloc, virt_page_off + phys_page_off, &block_idx, &block_offset))
	{
		/* should not fail as a valid mapping was found, so the phys mem must exists */
		OSK_PRINT_WARN(OSK_UMP, "Unable to find physical start block with offset %llx\n", virt_page_off + phys_page_off);
		OSK_ASSERT(0);
		return;
	}

	paddr = alloc->block_array[block_idx].addr + block_offset;

	for (i = 0; i < page_count; i++)
	{
		size_t offset = ((uintptr_t)vaddr) & ((1u << OSK_PAGE_SHIFT)-1);
		size_t sz = OSK_MIN((size_t)OSK_PAGE_SIZE - offset, size);

		/* check if we've overrrun the current block, if so move to the next block */
		if (paddr >= (alloc->block_array[block_idx].addr + alloc->block_array[block_idx].size))
		{
			block_idx++;
			OSK_ASSERT(block_idx < alloc->blocksCount);
			paddr = alloc->block_array[block_idx].addr;
		}

		if (UMP_MSYNC_CLEAN == op)
		{
			osk_sync_to_memory(paddr, vaddr, sz);
		}
		else /* (UMP_MSYNC_CLEAN_AND_INVALIDATE == op) already validated on entry */
		{
			osk_sync_to_cpu(paddr, vaddr, sz);
		}

		/* advance to next page  */
		vaddr = (void*)((uintptr_t)vaddr + sz);
		size -= sz;
		paddr += sz;
	}
}

UMP_KERNEL_API_EXPORT ump_dd_handle ump_dd_create_from_phys_blocks(const ump_dd_physical_block * blocks, u64 num_blocks, ump_alloc_flags flags, ump_dd_security_filter filter_func, ump_dd_final_release_callback final_release_func, void* callback_data)
{
	u64 size = 0;
	u64 i;
	umpp_allocation * alloc;

	alloc = osk_calloc(sizeof(*alloc));

	if (NULL == alloc)
	{
		goto out1;
	}

	osk_atomic_set(&alloc->refcount, 1);
	if (flags & UMP_ALLOCATE_AS_SIZE_PINNED)
	{
		osk_atomic_set(&alloc->pincount, 1);
	}
	else
	{
		osk_atomic_set(&alloc->pincount, 0);
	}
	
	/* NOTE: Missing implementation for non-shareable allocations, see MIDBASE-522 */

	alloc->block_array = osk_calloc(sizeof(ump_dd_physical_block) * num_blocks);
	if (NULL == alloc->block_array)
	{
		goto out2;
	}
	
	OSK_MEMCPY(alloc->block_array, blocks, sizeof(ump_dd_physical_block) * num_blocks);

	for (i = 0; i < num_blocks; i++)
	{
		size += blocks[i].size;
	}

	alloc->size = size;
	alloc->blocksCount = num_blocks;
	alloc->flags = flags;
	alloc->filter_func = filter_func;
	alloc->final_release_func = final_release_func;
	alloc->callback_data = callback_data;

	/* all set up, allocate an ID */

	osk_mutex_lock(&device.secure_id_map_lock);
	alloc->id = umpp_descriptor_mapping_allocate(device.secure_id_map, (void*)alloc);
	osk_mutex_unlock(&device.secure_id_map_lock);

	if ((int)alloc->id == 0)
	{
		/* failed to allocate a secure_id */
		goto out3;
	}

	alloc->management_flags |= UMP_MGMT_EXTERNAL;

	return alloc;

out3:
	osk_free(alloc->block_array);
out2:
	osk_free(alloc);
out1:
	return UMP_DD_INVALID_MEMORY_HANDLE;
}
