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
 * @file ump.h
 *
 * This file contains the user space part of the UMP API.
 *
 */

#ifndef _UMP_H_
#define _UMP_H_

/**
 * @page page_base_ump Unified Memory Provider API
 *
 * UMP(Universal Memory Provider) is an API to allocate memory with some special unique requirements;
 * @li Known physical addresses
 * @li Non-relocatable/pinned
 * @li Won't be paged out
 * @li Shareable between processes (selectable per allocation for security reasons)
 * @li Shareable with multiple hardware devices
 * @li Physically contiguous (optional)
 * @li Extended (not valid with the physically contiguous requirement for obvious reasons)
 *
 * Allocations from UMP can safely be used with hardware devices and other processes.
 * All uses are reference counted, so memory won't be released until all participating hardware devices and processes have released their use of the allocation.
 * This means that even if a process frees memory too early or crashes any hardware using the memory won't corrupt freed memory.
 *
 * Allocations inside a process is represented using an UMP memory handle.
 *
 * Each allocation is represented by a system-wide unique ID (called a secure ID),
 * which can be obtained from a handle and be shared with other processes or given to a device driver.
 *
 * Based on a secure ID a new handle can be created either in kernel space by a driver
 * or in user space by some other process to use the same allocation.
 *
 * Based on the handle a driver in kernel space can obtain information about the physical memory block(s)
 * an allocation consists of and increment or decrement the reference count.
 *
 * Usage in user-space also adds a reference to the memory, but it's managed by the UMP device driver.
 *
 * The user-space reference count is only local to the process, so a process can't by accident decrement
 * the count one time too many and cause the memory to be freed while it's in use by a hardware device.
 *
 * This is all handled by the UMP kernel code, no user-space code cooperation is needed.
 *
 * By default an allocation is only accessible in the same process or what other security boundary the OS uses.
 * If marked as shared it can be accessed in all processes, the kernel space customer defined security filter permitting of course.
 * See @ref ump_dd_security_filter for more information about this security filter.
 *
 * @sa ump_api
 * @sa example_user_api.c
 * @sa example_kernel_api.c
 *
 * @example example_user_api.c
 * @example example_kernel_api.c
 *
 */

/** @defgroup ump_api Unified Memory Provider APIs
 */

/**
 * @addtogroup ump_api
 * @{
 */

/** @defgroup ump_user_space_api UMP User Space API
 * @{ */


#include <ump/ump_platform.h>
#include <ump/ump_common.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * External representation of a UMP handle in user space.
 */
typedef void * ump_handle;

/**
 * Value to indicate an invalid UMP memory handle.
 */
#define UMP_INVALID_MEMORY_HANDLE ((ump_handle)0)

/**
 * Opens and initializes the UMP library.
 *
 * This function must be called at least once before calling any other UMP API functions.
 * Each successful open call is reference counted and must be matched with a call to @ref ump_close.
 * It is safe to call @a ump_open after a @a ump_close has terminated a previous session.
 *
 * @see ump_close
 *
 * @return UMP_OK indicates success, UMP_ERROR indicates failure.
 */
UMP_API_EXPORT ump_result ump_open(void) CHECK_RESULT;


/**
 * Terminate the UMP library.
 *
 * This must be called once for every successful @ref ump_open. The UMP library is
 * terminated when, and only when, the last open reference to the UMP interface is closed.
 *
 * If this is called while having active allocations the behavior is undefined.
 *
 * @see ump_open
 */
UMP_API_EXPORT void ump_close(void);


/**
 * Allocate a buffer.
 * The life-time of the allocation is controlled by a reference count.
 * The reference count of the returned buffer is set to 1.
 * The memory will be freed once the reference count reaches 0.
 * Use @ref ump_retain and @ref ump_release to control the reference count.
 * The contens of the memory returned will be zero initialized.
 * @param size Number of bytes to allocate. Will be padded up to a multiple of the page size.
 * @param flags Bit-wise OR of zero or more of the allocation flag bits.
 * @return Handle to the new allocation, or @a UMP_INVALID_MEMORY_HANDLE on allocation failure.
 */
UMP_API_EXPORT ump_handle ump_allocate(u64 size, ump_alloc_flags flags) CHECK_RESULT;


/**
 * Change or query buffer size.
 * Allow to obtain, increase or reduce the amount of physical memory allocated.
 * Pages will be added or removed at the end of the allocation, no relocation will happen.
 * When pages are added at the end the new pages will be zero initialized.
 * @a size_diff is used to indicate if the current size is queried, asked to be increased or asked to be reduced.
 * @a size_diff is rounded up to the page size when increasing and rounded down to an even page size when shrinking.
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
 *
 * @param mem The handle to resize
 * @param size_diff Size delta
 * @li If positive, the number of extra bytes to allocate and assign to the end of the allocation
 * @li If negative, the number of bytes to trim away from the end of the allocation
 * @li If zero, do no change but just return current size
 * @param[out] new_size The size of the allocation (in bytes) after the requested operation has been done.
 * @return One of @ref ump_resize_result
 */
UMP_API_EXPORT ump_resize_result ump_resize(ump_handle mem, s64 size_diff, u64 * new_size) CHECK_RESULT;

/**
 * Retrieves the secure ID for the specified UMP memory.
 *
 * This identifier is unique across the entire system, and uniquely identifies
 * the specified UMP memory allocation. This identifier can later be used through the
 * @ref ump_from_secure_id or
 * @ref ump_dd_from_secure_id
 * functions in order to access this UMP memory, for instance from another process.
 * Unless the allocation was marked as shared the returned ID will only be resolvable in the same process as did the allocation.
 *
 * If called on an @a UMP_INVALID_MEMORY_HANDLE it will return @a UMP_INVALID_SECURE_ID.
 *
 * @note There is a kernel space equivalent function called @ref ump_dd_secure_id_get
 *
 * @see ump_from_secure_id
 * @see ump_dd_from_secure_id
 * @see ump_dd_secure_id_get
 *
 * @param mem Handle to UMP memory.
 *
 * @return Returns the secure ID for the specified UMP memory.
 */
UMP_API_EXPORT ump_secure_id ump_secure_id_get(const ump_handle mem) CHECK_RESULT;


/**
 * Creates a handle based on a shared UMP memory allocation.
 *
 * The usage of UMP memory is reference counted, so this will increment the reference
 * count by one for the specified UMP memory.
 *
 * If called on an @a UMP_INVALID_SECURE_ID this will return @a UMP_INVALID_MEMORY_HANDLE.
 * If called on an non-shared allocation and this is a different process @a UMP_INVALID_MEMORY_HANDLE will be returned.
 *
 * Use @ref ump_release when there is no longer any
 * use for the retrieved handle.
 *
 * @note There is a kernel space equivalent function called @ref ump_dd_from_secure_id
 *
 * @see ump_release
 * @see ump_dd_from_secure_id
 *
 * @param secure_id The secure ID of the UMP memory to open, that can be retrieved using the @ref ump_secure_id_get function.
 *
 * @return @a UMP_INVALID_MEMORY_HANDLE indicates failure, otherwise a valid handle is returned.
 */
UMP_API_EXPORT ump_handle ump_from_secure_id(ump_secure_id secure_id) CHECK_RESULT;


/**
 * Retrieves the actual size of the specified UMP memory.
 *
 * The size is reported in bytes, and is typically a multiple of the page size.
 * If called on an @a UMP_INVALID_MEMORY_HANDLE will result in undefined behavior.
 * Debug builds will assert on this.
 *
 * @note There is a kernel space equivalent function called @ref ump_dd_size_get
 *
 * @see ump_dd_size_get
 *
 * @param mem Handle to UMP memory.
 *
 * @return Returns the allocated size of the specified UMP memory, in bytes.
 */
UMP_API_EXPORT u64 ump_size_get(const ump_handle mem) CHECK_RESULT;


/**
 * Synchronous mapping cache sync operation.
 *
 * Performs the requested CPU side cache sync operations before returning.
 * A clean must be done before the memory is guaranteed to be visible in main memory.
 * Any device-specific cache clean/invalidate must be done in combination with this routine, if needed.
 *
 * Example:
 * @code
 * 	ump_cpu_msync_now(handle, UMP_MSYNC_CLEAN, ptr, size);
 * 	device_invalidate(...);
 * 	// ... run device ...
 * 	device_clean(...);
 * 	ump_cpu_msync_now(handle, UMP_MSYNC_CLEAN_AND_INVALIDATE, ptr, size);
 * 	// ... safe to access on the cpu side again ...
 * @endcode
 *
 * Calls to operate on an @a UMP_INVALID_MEMORY_HANDLE will result in undefined behavior.
 * Debug builds will assert on this.
 *
 * If @a address is not inside a mapping previously obtained from the @a ump_handle provided results in undefined behavior.
 * If @a address combined with @a size results on reaching beyond the end of the buffer results in undefined behavior.
 *
 * @param mem Handle to UMP memory
 * @param op Cache operation to perform
 * @param[in] address The CPU address where to start the sync operation, this can be at an offset from the start of the allocation.
 * @param size The number of bytes to be synced.
 *
 */
UMP_API_EXPORT void ump_cpu_msync_now(ump_handle mem, ump_cpu_msync_op op, void * address, size_t size);


/**
 * Retrieves a memory mapped pointer to the specified UMP memory.
 *
 * This function retrieves a memory mapped pointer to the specified UMP memory, that can be used by the CPU.@n
 * Every successful call to @a ump_map must be matched with a call to @ref ump_unmap when the mapping is no longer needed.
 *
 * Multiple mappings within an allocation is permitted, but two mappings can not overlap.
 *
 * An offset and/or size resulting in going beyond the end of the buffer will case  the function to return NULL.
 *
 * Calling on @a UMP_INVALID_MEMORY_HANDLE results in undefined behavior.
 * Debug builds will assert on this.
 *
 * @note Systems without a MMU for the CPU only return the physical address, because no mapping is required.
 *
 * @see ump_unmap
 *
 * @param mem Handle to UMP memory.
 * @param offset An offset at which the mapping begins.
 * @param size The number of bytes to map.
 *
 * @return NULL indicates failure, otherwise a CPU mapped pointer is returned.
 */
UMP_API_EXPORT void * ump_map(ump_handle mem, u64 offset, size_t size) CHECK_RESULT;


/**
 * Releases a previously mapped pointer to the specified UMP memory.
 *
 * Every successful call to @ref ump_map must be matched with a call to @a ump_unmap when the mapping is no longer needed.
 *
 * The following results in undefined behavior:
 * - Called with an address not returned from @ref ump_map
 * - Called with a different @a ump_handle than was used to obtain the pointer
 *
 * @note Systems without a MMU must still implement this function, even though no unmapping should be needed.
 *
 * @param mem Handle to UMP memory.
 * @param[in] address The CPU virtual address returned by @ref ump_map
 * @param size Size matching argument given to ump_map
 */
UMP_API_EXPORT void ump_unmap(ump_handle mem, void* address, size_t size);


/**
 * Adds an extra reference to the specified UMP memory.
 *
 * This function adds an extra reference to the specified UMP memory. This function should
 * be used every time a UMP memory handle is duplicated, that is, assigned to another ump_handle
 * variable. The function @ref ump_release must then be used
 * to release each copy of the UMP memory handle.
 *
 * It's safe to call this on both shared and non-shared handles.
 * Calling on an @a UMP_INVALID_MEMORY_HANDLE results in undefined behavior.
 * Debug builds will assert on this.
 *
 * @note You are not required to call @ref ump_retain
 * for UMP handles returned from
 * @ref ump_from_secure_id,
 * because these handles are already reference counted by this function.
 *
 * @note There is a kernel space equivalent function called @ref ump_dd_retain
 *
 * @see ump_dd_retain
 *
 * @param mem Handle to UMP memory.
 */
UMP_API_EXPORT void ump_retain(ump_handle mem);

/**
 * Releases a reference from the specified UMP memory.
 *
 * This function should be called once for every reference to the UMP memory handle.
 * When the last reference is released, all resources associated with this UMP memory
 * handle are freed.
 *
 * It's safe to call this on both shared and non-shared handles.
 * If called on an @a UMP_INVALID_MEMORY_HANDLE it will return early.
 *
 * @note There is a kernel space equivalent function called @ref ump_dd_release
 *
 * @see ump_release
 *
 * @param mem Handle to UMP memory.
 */
UMP_API_EXPORT void ump_release(ump_handle mem);


#ifdef __cplusplus
}
#endif


/** @} */ /* end group ump_user_space_api */

/** @} */ /* end group ump_api */

#endif /* _UMP_H_ */
