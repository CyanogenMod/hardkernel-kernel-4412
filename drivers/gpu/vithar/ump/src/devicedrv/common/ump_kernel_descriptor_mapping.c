/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */


#include <osk/mali_osk.h>
#include <common/ump_kernel_descriptor_mapping.h>


#define MALI_PAD_INT(x) (((x) + (OSK_BITS_PER_LONG - 1)) & ~(OSK_BITS_PER_LONG - 1))

/**
 * Allocate a descriptor table capable of holding 'count' mappings
 * @param count Number of mappings in the table
 * @return Pointer to a new table, NULL on error
 */
static umpp_descriptor_table * descriptor_table_alloc(unsigned int count);

/**
 * Free a descriptor table
 * @param table The table to free
 */
static void descriptor_table_free(umpp_descriptor_table * table);

umpp_descriptor_mapping * umpp_descriptor_mapping_create(unsigned int init_entries, unsigned int max_entries)
{
	umpp_descriptor_mapping * map = osk_calloc(sizeof(umpp_descriptor_mapping) );

	init_entries = MALI_PAD_INT(init_entries);
	max_entries = MALI_PAD_INT(max_entries);

	if (NULL != map)
	{
		map->table = descriptor_table_alloc(init_entries);
		if (NULL != map->table)
		{
			if (OSK_ERR_NONE == osk_rwlock_init( &map->lock, 0))
			{
				osk_bitarray_set_bit(0, map->table->usage); /* reserve bit 0 to prevent NULL/zero logic to kick in */
				map->max_nr_mappings_allowed = max_entries;
				map->current_nr_mappings = init_entries;
				return map;
			}
			descriptor_table_free(map->table);
		}
		osk_free(map);
	}
	return NULL;
}

void umpp_descriptor_mapping_destroy(umpp_descriptor_mapping * map)
{
	OSK_ASSERT(NULL != map);
	descriptor_table_free(map->table);
	osk_rwlock_term(&map->lock);
	osk_free(map);
}

unsigned int umpp_descriptor_mapping_allocate(umpp_descriptor_mapping * map, void * target)
{
 	int descriptor = 0;
	OSK_ASSERT(NULL != map);
 	osk_rwlock_write_lock( &map->lock);
 	descriptor = osk_bitarray_find_first_zero_bit(map->table->usage, map->current_nr_mappings);
	if (descriptor == map->current_nr_mappings)
	{
		/* no free descriptor, try to expand the table */
		umpp_descriptor_table * new_table;
		umpp_descriptor_table * old_table = map->table;
		int nr_mappings_new = map->current_nr_mappings + OSK_BITS_PER_LONG;

		if (map->current_nr_mappings >= map->max_nr_mappings_allowed)
		{
			descriptor = 0;
			goto unlock_and_exit;
		}

		new_table = descriptor_table_alloc(nr_mappings_new);
		if (NULL == new_table)
		{
			descriptor = 0;
			goto unlock_and_exit;
		}

 		OSK_MEMCPY(new_table->usage, old_table->usage, (sizeof(unsigned long)*map->current_nr_mappings) / OSK_BITS_PER_LONG);
 		OSK_MEMCPY(new_table->mappings, old_table->mappings, map->current_nr_mappings * sizeof(void*));
		map->table = new_table;
		map->current_nr_mappings = nr_mappings_new;
		descriptor_table_free(old_table);
	}

	/* we have found a valid descriptor, set the value and usage bit */
	osk_bitarray_set_bit(descriptor, map->table->usage);
	map->table->mappings[descriptor] = target;

unlock_and_exit:
	osk_rwlock_write_unlock(&map->lock);
	return descriptor;
}

mali_error umpp_descriptor_mapping_lookup(umpp_descriptor_mapping * map, unsigned int descriptor, void** target)
{
	mali_error result = MALI_ERROR_FUNCTION_FAILED;
 	OSK_ASSERT(map);
	OSK_ASSERT(target);
 	osk_rwlock_read_lock(&map->lock);
 	if ( (descriptor > 0) && (descriptor < map->current_nr_mappings) && osk_bitarray_test_bit(descriptor, map->table->usage) )
	{
		*target = map->table->mappings[descriptor];
		result = MALI_ERROR_NONE;
	}
	/* keep target untouched if the descriptor was not found */
	osk_rwlock_read_unlock(&map->lock);
	return result;
}

mali_error umpp_descriptor_mapping_set(umpp_descriptor_mapping * map, unsigned int descriptor, void * target)
{
	mali_error result = MALI_ERROR_FUNCTION_FAILED;
 	OSK_ASSERT(map);
 	osk_rwlock_read_lock(&map->lock);
 	if ( (descriptor > 0) && (descriptor < map->current_nr_mappings) && osk_bitarray_test_bit(descriptor, map->table->usage) )
	{
		map->table->mappings[descriptor] = target;
		result = MALI_ERROR_NONE;
	}
	osk_rwlock_read_unlock(&map->lock);
	return result;
}

void umpp_descriptor_mapping_remove(umpp_descriptor_mapping * map, unsigned int descriptor)
{
 	OSK_ASSERT(map);
 	osk_rwlock_write_lock(&map->lock);
 	if ( (descriptor > 0) && (descriptor < map->current_nr_mappings) && osk_bitarray_test_bit(descriptor, map->table->usage) )
	{
		map->table->mappings[descriptor] = NULL;
		osk_bitarray_clear_bit(descriptor, map->table->usage);
	}
	osk_rwlock_write_unlock(&map->lock);
}

static umpp_descriptor_table * descriptor_table_alloc(unsigned int count)
{
	umpp_descriptor_table * table;

	table = osk_calloc(sizeof(umpp_descriptor_table) + ((sizeof(unsigned long) * count)/OSK_BITS_PER_LONG) + (sizeof(void*) * count) );

	if (NULL != table)
	{
		table->usage = (unsigned long*)((u8*)table + sizeof(umpp_descriptor_table));
		table->mappings = (void**)((u8*)table + sizeof(umpp_descriptor_table) + ((sizeof(unsigned long) * count)/OSK_BITS_PER_LONG));
	}

	return table;
}

static void descriptor_table_free(umpp_descriptor_table * table)
{
 	OSK_ASSERT(table);
	osk_free(table);
}

