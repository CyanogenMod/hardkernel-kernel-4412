/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file ump_kernel_interface.h
 *
 * This file contains the kernel space part of the UMP API.
 *
 */

#ifndef _UMP_KERNEL_INTERFACE_H_
#define _UMP_KERNEL_INTERFACE_H_

/**
 * @addtogroup ump_api
 * @{
 */

/** @defgroup ump_kernel_space_api UMP Kernel Space API
 * @{ */


#include <ump/ump_kernel_platform.h>
#include <ump/ump_common.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * External representation of a UMP handle in kernel space.
 */
typedef void * ump_dd_handle;

/**
 * Value to indicate an invalid UMP memory handle.
 */
#define UMP_DD_INVALID_MEMORY_HANDLE ((ump_dd_handle)0)

/**
 * Struct used to describe a physical block used by UMP memory
 */
typedef struct ump_dd_physical_block
{
	u64 addr; /**< The physical address of the block */
	u64 size; /**< The length of the block, in bytes, typically page aligned */
} ump_dd_physical_block;

/**
 * Security filter hook.
 *
 * Each allocation can have a security filter attached to it.@n
 * The hook receives
 * @li the secure ID
 * @li a handle to the allocation
 * @li  the callback_data argument provided to @ref ump_dd_allocate or @ref ump_dd_create_from_phys_blocks
 *
 * The hook must return @a MALI_TRUE to indicate that access to the handle is allowed or @n
 * @a MALI_FALSE to state that no access is permitted.@n
 * This hook is guaranteed to be called in the context of the requesting process/address space.
 *
 * The arguments provided to the hook are;
 * @li the secure ID
 * @li handle to the allocation
 * @li the callback_data set when registering the hook
 *
 * Return value;
 * @li @a MALI_TRUE to permit access
 * @li @a MALI_FALSE to deny access
 */
typedef mali_bool (*ump_dd_security_filter)(ump_secure_id, ump_dd_handle, void *);

/**
 * Final release notify hook.
 *
 * Allocations can have a hook attached to them which is called when the last reference to the allocation is released.
 * No reference count manipulation is allowed on the provided handle, just property querying (ID get, size get, phys block get).
 * This is similar to finalizers in OO languages.
 *
 * The arguments provided to the hook are;
 * * handle to the allocation
 * * the callback_data set when registering the hook
 */
typedef void (*ump_dd_final_release_callback)(const ump_dd_handle, void *);

/**
 * Allocate a buffer.
 * The lifetime of the allocation is controlled by a reference count.
 * The reference count of the returned buffer is set to 1.
 * The memory will be freed once the reference count reaches 0.
 * Use @ref ump_dd_retain and @ref ump_dd_release to control the reference count.
 * @param size Number of bytes to allocate. Will be padded up to a multiple of the page size.
 * @param flags Bit-wise OR of zero or more of the allocation flag bits.
 * @param[in] filter_func Pointer to a function which will be called before this allocation is returned to user-space.
 * NULL permitted if no need for a callback.
 * @param[in] final_release_func Pointer to a function which will be called when the last reference is removed,
 * just before the allocation is freed. NULL permitted if no need for a callback.
 * @param[in] callback_data An opaque pointer which will be provided to @a filter_func and @a final_release_func
 * @return Handle to the new allocation, or @a UMP_DD_INVALID_MEMORY_HANDLE on allocation failure.
 */
UMP_KERNEL_API_EXPORT ump_dd_handle ump_dd_allocate(u64 size, ump_alloc_flags flags, ump_dd_security_filter filter_func, ump_dd_final_release_callback final_release_func, void* callback_data);


/**
 * Allocation bits getter.
 * Retrieves the allocation flags used when instantiating the given handle.
 * Just a copy of the flag given to @ref ump_dd_allocate and @ref ump_dd_create_from_phys_blocks
 * @param mem The handle retrieve the bits for
 * @return The allocation bits used to instantiate the allocation
 */
UMP_KERNEL_API_EXPORT ump_alloc_flags ump_dd_allocation_flags_get(const ump_dd_handle mem);


/**
 * Change or query buffer size.
 * Allow to obtain, increase or reduce the amount of physical memory allocated.
 * Pages will be added or removed at the end of the allocation, no relocation will happen.
 * @a size_diff is used to indicate if the current size is queried, asked to be increased or asked to be reduced.
 * @a size_diff is rounded up to an even page size when increasing and rounded down to an even page size when shrinking.
 *
 * @ ump_resize is not supported on allocations allocated with the @ref UMP_CONSTRAINT_PHYSICALLY_LINEAR flag.
 *
 * If the operation could not be performed the size remains unchanged.
 * If the operation failed due to an OOM, @a UMP_RESIZE_ERROR_OOM will be returned.
 * If the operation failed due to the memory being pinned due to use by a device, @a UMP_RESIZE_ERROR_PINNED will be returned.
 * If the resize would result in a negative size, the new size is set to zero.
 *
 * Resizing an allocation which is currently mapped on the CPU side is not supported and @a UMP_RESIZE_ERROR_MAPPED  will be returned.
 * If called on a shared allocation this will return @a UMP_RESIZE_ERROR_SHARED and the size will be unchanged.
 *
 * Calling this function on an @a UMP_DD_INVALID_MEMORY_HANDLE results in undefined behavior.
 * Debug builds will assert on this.
 *
 * @param mem The handle to resize
 * @param size_diff Size delta
 * @li If positive, the number of extra bytes to allocate an assign to the end of the allocation
 * @li If negative, the number of bytes to trim away from the end of the allocation
 * @li If zero, do no change but just return current size
 * @param[out] new_size The size of the allocation (in bytes) after the requested operation has been done.
 * @return One of @ref ump_resize_result
 */
UMP_KERNEL_API_EXPORT ump_resize_result ump_dd_resize(ump_dd_handle mem, s64 size_diff, u64 * new_size);


/**
 * Retrieves the secure ID for the specified UMP memory.
 *
 * This identifier is unique across the entire system, and uniquely identifies
 * the specified UMP memory allocation. This identifier can later be used through the
 * @ref ump_dd_from_secure_id or
 * @ref ump_from_secure_id
 * functions in order to access this UMP memory, for instance from another process (if shared of course).
 * Unless the allocation was marked as shared the returned ID will only be resolvable in the same process as did the allocation.
 *
 * Calling on an @a UMP_DD_INVALID_MEMORY_HANDLE will result in undefined behavior.
 * Debug builds will assert on this.
 *
 * @note There is a user space equivalent function called @ref ump_secure_id_get
 *
 * @see ump_dd_from_secure_id
 * @see ump_from_secure_id
 * @see ump_secure_id_get
 *
 * @param mem Handle to UMP memory.
 *
 * @return Returns the secure ID for the specified UMP memory.
 */
UMP_KERNEL_API_EXPORT ump_secure_id ump_dd_secure_id_get(const ump_dd_handle mem);


/**
 * Retrieves a handle to allocated UMP memory.
 *
 * The usage of UMP memory is reference counted, so this will increment the reference
 * count by one for the specified UMP memory.
 * Use @ref ump_dd_release when there is no longer any
 * use for the retrieved handle.
 *
 * If called on an non-shared allocation and this is a different process @a UMP_DD_INVALID_MEMORY_HANDLE will be returned.
 *
 * Calling on an @a UMP_INVALID_SECURE_ID will return @a UMP_DD_INVALID_MEMORY_HANDLE
 *
 * @note There is a user space equivalent function called @ref ump_from_secure_id
 *
 * @see ump_dd_release
 * @see ump_from_secure_id
 *
 * @param secure_id The secure ID of the UMP memory to open, that can be retrieved using the @ref ump_secure_id_get function.
 *
 * @return @a UMP_DD_INVALID_MEMORY_HANDLE indicates failure, otherwise a valid handle is returned.
 */
UMP_KERNEL_API_EXPORT ump_dd_handle ump_dd_from_secure_id(ump_secure_id secure_id);


/**
 * Pin the size of an UMP allocation.
 *
 * A size pinned allocation can not be resized using @ref ump_dd_resize.
 * This must be called before @ref ump_dd_phys_blocks_get is called.
 * A pin-count is kept, so each call to @ref ump_dd_pin_size must be matched with a call to @ref ump_dd_unpin_size.
 *
 * A typical usage example (error checking removed in this example);
 *
 * @code
 * u64 i;
 * u64 count;
 * const ump_dd_physical_block * blocks;
 *
 * ump_dd_pin_size(mem);
 * // All ump_[dd_]_resize calls will now fail with UMP_RESIZE_ERROR_PINNED.
 *
 * ump_dd_phys_blocks_get(mem, &count, &blocks);
 *
 * for (i = 0; i < count; i++)
 * {
 * 	program_device_mmu(i, blocks[i]);
 * }
 *
 * run_device();
 *
 * use_result();
 *
 * for (i = 0; i < count; i++)
 * {
 *  clear_device_mmu_mapping(i);
 * }
 *
 * ump_dd_unpin_size(mem);
 *
 * // Released our pinning, other pinnings might still block resizing
 *
 * @endcode
 *
 * @param mem Handle to the UMP allocation to pin the size of
 *
 */
UMP_KERNEL_API_EXPORT void ump_dd_pin_size(ump_dd_handle mem);

/**
 * Unpin the size of an UMP allocation.
 *
 * See @ref ump_dd_pin_size for a description of pinning and an example.
 *
 * An unmatched call to this function will result in undefined behavior.
 * Debug builds will assert on this.
 *
 * @param mem Handle to the UMP allocation to un-pin the size of
 */
UMP_KERNEL_API_EXPORT void ump_dd_unpin_size(ump_dd_handle mem);

/**
 * Retrieves all physical memory block information for specified UMP memory.
 *
 * This function can be used by other device drivers in order to create MMU tables.
 * This function will return a pointer to an array of @ref ump_dd_physical_block in @a pArray and the number of array elements in @a pCount
 * This function must only be called on pinned memory.
 * As an exception it's safe to call @ref ump_dd_phys_blocks_get in a @ref ump_dd_final_release_callback without pinning memory.
 * To pin memory, see @ref ump_dd_pin_size.
 *
 * Calling on an @a UMP_DD_INVALID_MEMORY_HANDLE results in undefined behavior.
 * Debug builds will assert on this.
 *
 * @param mem Handle to UMP memory.
 * @param[out] pCount Pointer to where to store the number of items in the returned array
 * @param[out] pArray Pointer to where to store a pointer to the physical blocks array
 */
UMP_KERNEL_API_EXPORT void ump_dd_phys_blocks_get(const ump_dd_handle mem, u64 * pCount, const ump_dd_physical_block ** pArray);

/**
 * Retrieves the actual size of the specified UMP memory.
 *
 * The size is reported in bytes, and is typically page aligned.
 *
 * Calling on an @a UMP_DD_INVALID_MEMORY_HANDLE results in undefined behavior.
 * Debug builds will assert on this.
 *
 * @note There is a user space equivalent function called @ref ump_size_get
 *
 * @see ump_size_get
 *
 * @param mem Handle to UMP memory.
 *
 * @return Returns the allocated size of the specified UMP memory, in bytes.
 */
UMP_KERNEL_API_EXPORT u64 ump_dd_size_get(const ump_dd_handle mem);


/**
 * Adds an extra reference to the specified UMP memory allocation.
 *
 * The function @ref ump_dd_release must then be used
 * to release each copy of the UMP memory handle.
 *
 * Calling on an @a UMP_DD_INVALID_MEMORY_HANDLE results in undefined behavior.
 * Debug builds will assert on this.
 *
 * @note You are not required to call @ref ump_dd_retain
 * for UMP handles returned from
 * @ref ump_dd_from_secure_id,
 * because these handles are already reference counted by this function.
 *
 * @note There is a user space equivalent function called @ref ump_retain
 *
 * @see ump_retain
 *
 * @param mem Handle to UMP memory.
 */
UMP_KERNEL_API_EXPORT void ump_dd_retain(ump_dd_handle mem);


/**
 * Releases a reference from the specified UMP memory.
 *
 * This function must be called once for every reference to the UMP memory handle.
 * When the last reference is released, all resources associated with this UMP memory
 * handle are freed.
 *
 * If called on an @a UMP_DD_INVALID_MEMORY_HANDLE the function will early out.
 *
 * @note There is a user space equivalent function called @ref ump_release
 *
 * @see ump_release
 *
 * @param mem Handle to UMP memory.
 */
UMP_KERNEL_API_EXPORT void ump_dd_release(ump_dd_handle mem);

/**
 * Create an ump allocation handle based on externally managed memory.
 * Used to wrap an existing allocation as an UMP memory handle.
 * Once wrapped the memory acts just like a normal allocation coming from @ref ump_dd_allocate.
 * The only exception is that the freed physical memory is not put into the pool of free memory, but instead considered returned to the caller once @a final_release_func returns.
 * The blocks array will be copied, so no need to hold on to it after this function returns.
 * The handle returned will not be resizeable and will return 
 * @param[in] blocks Array of @ref ump_dd_physical_block
 * @param num_blocks Number of elements in the array pointed to by @a blocks
 * @param flags Allocation flags to mark the handle with
 * @param[in] filter_func Pointer to a function which will be called before this allocation is returned to user-space. NULL permitted if no need for a callback.
 * @param[in] final_release_func Pointer to a function which will be called when the last reference is removed, just before the allocation is freed. NULL permitted if no need for a callback.
 * @param[in] callback_data An opaque pointer which will be provided to @a filter_func and @a final_release_func
 * @return Handle to the UMP allocation handle created, or @a UMP_DD_INVALID_MEMORY_HANDLE if no such handle could be created.
 */
UMP_KERNEL_API_EXPORT ump_dd_handle ump_dd_create_from_phys_blocks(const ump_dd_physical_block * blocks, u64 num_blocks, ump_alloc_flags flags, ump_dd_security_filter filter_func, ump_dd_final_release_callback final_release_func, void* callback_data);

#ifdef __cplusplus
}
#endif


/** @} */ /* end group ump_kernel_space_api */

/** @} */ /* end group ump_api */

#endif /* _UMP_KERNEL_INTERFACE_H_ */
