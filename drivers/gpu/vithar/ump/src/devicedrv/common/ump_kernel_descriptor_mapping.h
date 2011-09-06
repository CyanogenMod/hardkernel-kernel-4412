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
 * @file ump_kernel_descriptor_mapping.h
 */

#ifndef _UMP_KERNEL_DESCRIPTOR_MAPPING_H_
#define _UMP_KERNEL_DESCRIPTOR_MAPPING_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * The actual descriptor mapping table, never directly accessed by clients
 */
typedef struct umpp_descriptor_table
{
	/* keep as a unsigned long to rely on the OS's bitops support */
	unsigned long * usage; /**< Pointer to bitpattern indicating if a descriptor is valid/used(1) or not(0) */
	void** mappings; /**< Array of the pointers the descriptors map to */
} umpp_descriptor_table;

/**
 * The descriptor mapping object
 * Provides a separate namespace where we can map an integer to a pointer
 */
typedef struct umpp_descriptor_mapping
{
	osk_rwlock lock; /**< Lock protecting access to the mapping object */
	unsigned int max_nr_mappings_allowed; /**< Max number of mappings to support in this namespace */
	unsigned int current_nr_mappings; /**< Current number of possible mappings */
	umpp_descriptor_table * table; /**< Pointer to the current mapping table */
} umpp_descriptor_mapping;

/**
 * Create a descriptor mapping object.
 * Create a descriptor mapping capable of holding init_entries growable to max_entries.
 * ID 0 is reserved so the number of available entries will be max - 1.
 * @param init_entries Number of entries to preallocate memory for
 * @param max_entries Number of entries to max support
 * @return Pointer to a descriptor mapping object, NULL on failure
 */
umpp_descriptor_mapping * umpp_descriptor_mapping_create(unsigned int init_entries, unsigned int max_entries);

/**
 * Destroy a descriptor mapping object
 * @param[in] map The map to free
 */
void umpp_descriptor_mapping_destroy(umpp_descriptor_mapping * map);

/**
 * Allocate a new mapping entry (descriptor ID)
 * Allocates a new entry in the map.
 * @param[in] map The map to allocate a new entry in
 * @param[in] target The value to map to
 * @return The descriptor allocated, ID 0 on failure.
 */
unsigned int umpp_descriptor_mapping_allocate(umpp_descriptor_mapping * map, void * target);

/**
 * Get the value mapped to by a descriptor ID
 * @param[in] map The map to lookup the descriptor id in
 * @param[in] descriptor The descriptor ID to lookup
 * @param[out] target Pointer to a pointer which will receive the stored value
 *
 * @return MALI_ERROR_NONE on successful lookup, MALI_ERROR_FUNCTION_FAILED on lookup failure.
 */
mali_error umpp_descriptor_mapping_lookup(umpp_descriptor_mapping * map, unsigned int descriptor, void** target);

/**
 * Set the value mapped to by a descriptor ID
 * If growing the descriptor table is needed it could run out of memory.
 * @param[in] map The map to set the descriptor id in
 * @param descriptor The descriptor ID to set
 * @param[in] target Pointer to replace the current value with
 * @return MALI_ERROR_NONE on success, MALI_ERROR_FUNCTION_FAILED on failure.
 */
mali_error umpp_descriptor_mapping_set(umpp_descriptor_mapping * map, unsigned int descriptor, void * target);

/**
 * Free the descriptor ID
 * For the descriptor to be reused it has to be freed
 * @param[in] map The map to free the descriptor from
 * @param descriptor The descriptor ID to free
 */
void umpp_descriptor_mapping_remove(umpp_descriptor_mapping * map, unsigned int descriptor);

#ifdef __cplusplus
}
#endif

#endif /* _UMP_KERNEL_DESCRIPTOR_MAPPING_H_ */
