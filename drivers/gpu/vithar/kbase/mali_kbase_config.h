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



#ifndef _KBASE_CONFIG_H_
#define _KBASE_CONFIG_H_

#include <malisw/mali_stdtypes.h>

/* This flag is set for internal builds so we can run tests without credentials.
 * It should be removed from customer builds to actually enable the security checks.
 * See MIDBLD-953.
 */
#define KBASE_HWCNT_DUMP_BYPASS_ROOT 1

/**
 * Relative memory performance indicators. Enum elements should always be defined in slowest to fastest order.
 */
typedef enum kbase_memory_performance
{
	KBASE_MEM_PERF_SLOW,
	KBASE_MEM_PERF_NORMAL,
	KBASE_MEM_PERF_FAST
} kbase_memory_performance;

/**
 * Device wide configuration
 */
enum
{
	/**
	 * Invalid attribute ID (reserve 0).
	 *
	 * Attached value: Ignored
	 * Default value: NA
	 * */
	KBASE_CONFIG_ATTR_INVALID,

	/**
	 * Memory resource object.
	 * Multiple resources can be listed.
	 * The resources will be used in the order listed
	 * in the configuration attribute list if they have no other
	 * preferred order based on the memory resource property list
	 * (see ::kbase_memory_attribute).
	 *
	 * Attached value: Pointer to a kbase_memory_resource object.
	 * Default value: No resources
	 * */

	KBASE_CONFIG_ATTR_MEMORY_RESOURCE,
	/**
	 * Maximum of memory which can be allocated from the OS
	 * to be used by the GPU (shared memory).
	 *
	 * Attached value: number in bytes
	 * Default value: Limited by available memory
	 */
	KBASE_CONFIG_ATTR_MEMORY_OS_SHARED_MAX,

	/**
	 * Relative performance for the GPU to access
	 * OS shared memory.
	 *
	 * Attached value: ::kbase_memory_performance member
	 * Default value: ::KBASE_MEM_PERF_NORMAL
	 */
	KBASE_CONFIG_ATTR_MEMORY_OS_SHARED_PERF_GPU,

	/**
	 * Limit (in bytes) the amount of memory a single process
	 * can allocate across all memory banks (including OS shared memory)
	 * for use by the GPU.
	 *
	 * Attached value: number in bytes
	 * Default value: Limited by available memory
	 */
	KBASE_CONFIG_ATTR_MEMORY_PER_PROCESS_LIMIT,

	/**
	 * UMP device mapping.
	 * Which UMP device this GPU should be mapped to.
	 *
	 * Attached value: UMP_DEVICE_<device>_SHIFT
	 * Default value: UMP_DEVICE_W_SHIFT
	 */
	KBASE_CONFIG_ATTR_UMP_DEVICE,

	/**
	 * Maximun frequency GPU will be clocked at. Given in kHz.
	 * 
	 * Attached value: number in kHz
	 * Default value: NA
	 */
	KBASE_CONFIG_ATTR_GPU_FREQ_KHZ_MAX,

	/**
	 * Minimum frequency GPU will be clocked at. Given in kHz.
	 * 
	 * Attached value: number in kHz
	 * Default value: NA
	 */
	KBASE_CONFIG_ATTR_GPU_FREQ_KHZ_MIN,

	/**
	 * Irq throttle. It is the minimum desired time in between two
	 * consecutive gpu interrupts (given in 'us'). The irq throttle
	 * gpu register will be configured after this, taking into
	 * account the configured max frequency.
	 * 
	 * Attached value: number in micro seconds
	 * Default value: see DEFAULT_IRQ_THROTTLE_TIME_US
	 */
	KBASE_CONFIG_ATTR_GPU_IRQ_THROTTLE_TIME_US,

	/**
	 * End of attribute list indicator.
	 * The configuration loader will stop processing any more elements
	 * when it encounters this attribute.
	 *
	 * Attached value: Ignored
	 * Default value: NA
	 */
	KBASE_CONFIG_ATTR_END = 0x1FFFUL
};

enum
{
	/**
	 * Invalid attribute ID (reserve 0).
	 *
	 * Attached value: Ignored
	 * Default value: NA
	 */
	KBASE_MEM_ATTR_INVALID,

	/**
	 * Relative performance for the CPU to access
	 * the memory resource.
	 *
	 * Attached value: ::kbase_memory_performance member
	 * Default value: ::KBASE_MEM_PERF_NORMAL
	 */
	KBASE_MEM_ATTR_PERF_CPU,

	/**
	 * Relative performance for the GPU to access
	 * the memory resource.
	 *
	 * Attached value: ::kbase_memory_performance member
	 * Default value: ::KBASE_MEM_PERF_NORMAL
	 */
	KBASE_MEM_ATTR_PERF_GPU,

	/**
	 * End of attribute list indicator.
	 * The memory resource loader will stop processing any more
	 * elements when it encounters this attribute.
	 *
	 * Attached value: Ignored
	 * Default value: NA
	 */
	KBASE_MEM_ATTR_END = 0x1FFFUL
};


/*
 * @brief specifies a single attribute
 *
 * Attribute is identified by attr field. Data is either integer or a pointer to attribute-specific structure.
 */
typedef struct kbase_attribute
{
	int id;
	uintptr_t data;
} kbase_attribute;

/*
 * @brief Specifies dedicated memory bank
 *
 * Specifies base, size and attributes of a memory bank
 */
typedef struct kbase_memory_resource
{
	u64 base;
	u64 size;
	struct kbase_attribute * attributes;
	const char * name;
} kbase_memory_resource;

#if !MALI_LICENSE_IS_GPL || (defined(MALI_FAKE_PLATFORM_DEVICE) && MALI_FAKE_PLATFORM_DEVICE)
/*
 * @brief Specifies start and end of I/O memory region.
 */
typedef struct kbase_io_memory_region
{
	u64       start;
	u64       end;
} kbase_io_memory_region;

/*
 * @brief Specifies I/O related resources like IRQs and memory region for I/O operations.
 */
typedef struct kbase_io_resources
{
	u32                      job_irq_number;
	u32                      mmu_irq_number;
	u32                      gpu_irq_number;
	kbase_io_memory_region   io_memory_region;
} kbase_io_resources;

typedef struct kbase_platform_config
{
	const kbase_attribute *attributes;
	const kbase_io_resources *io_resources;
	u32 midgard_type;
} kbase_platform_config;

#endif /* !MALI_LICENSE_IS_GPL || (defined(MALI_FAKE_PLATFORM_DEVICE) && MALI_FAKE_PLATFORM_DEVICE) */
/**
 * @brief Return character string associated with the given midgard type.
 *
 * @param[in]  midgard_type - ID of midgard type
  *
 * @return  Pointer to NULL-terminated character array associated with the given midgard type
 */
const char *kbasep_midgard_type_to_string(u32 midgard_type);

/**
 * @brief Gets the count of attributes in array
 *
 * Function gets the count of attributes in array. Note that end of list indicator is also included.
 *
 * @param[in]  attributes     Array of attributes
  *
 * @return  Number of attributes in the array including end of list indicator.
 */
int kbasep_get_config_attribute_count(const kbase_attribute *attributes);

/**
 * @brief Gets the count of attributes with specified id
 *
 * Function gets the count of attributes with specified id in the given attribute array
 *
 * @param[in]  attributes     Array of attributes
 * @param[in]  attibute_id    Id of attributes to count
  *
 * @return  Number of attributes in the array that have specified id
 */
int kbasep_get_config_attribute_count_by_id(const kbase_attribute *attributes, int attribute_id);

/**
 * @brief Gets the next config attribute with the specified ID from the array of attributes.
 *
 * Function gets the next attribute with specified attribute id within specified array. If no such attribute is found,
 * NULL is returned.
 *
 * @param[in]  attributes     Array of attributes in which lookup is performed
 * @param[in]  attribute_id   ID of attribute
 *
 * @return  Pointer to the first attribute matching id or NULL if none is found.
 */
const kbase_attribute *kbasep_get_next_attribute(const kbase_attribute *attributes, int attribute_id);

/**
 * @brief Gets the value of a single config attribute.
 *
 * Function gets the value of attribute specified as parameter. If no such attribute is found in the array of
 * attributes, default value is used.
 *
 * @param[in]  attributes     Array of attributes in which lookup is performed
 * @param[in]  attribute_id   ID of attribute
 *
 * @return Value of attribute with the given id
 */
uintptr_t kbasep_get_config_value(const kbase_attribute *attributes, int attribute_id);

/**
 * @brief Obtain memory performance values from kbase_memory_resource structure.
 *
 * Function gets cpu and gpu memory performance values from memory resource structure and puts them in the variables
 * provided as parameters. If the performance of memory bank is not in resource attributes, default value is used.
 *
 * @param[in]  resource         Structure containing information about memory bank to use
 * @param[out] cpu_performance  Pointer to variable which will hold CPU performance value
 * @param[out] gpu_performance  Pointer to variable which will hold GPU performance value
 */
void kbasep_get_memory_performance(const kbase_memory_resource *resource,
				kbase_memory_performance *cpu_performance, kbase_memory_performance *gpu_performance);

/**
 * @brief Validates configuration attributes
 *
 * Function checks validity of given configuration attributes. It will fail on any attribute with unknown id, attribute
 * with invalid value or attribute list that is not correctly terminated.
 *
 * @param[in]  attributes  Array of attributes to validate
 *
 * @return   MALI_TRUE if no errors have been found in the config. MALI_FALSE otherwise.
 */
mali_bool kbasep_validate_configuration_attributes(const kbase_attribute *attributes);

#if !MALI_LICENSE_IS_GPL || (defined(MALI_FAKE_PLATFORM_DEVICE) && MALI_FAKE_PLATFORM_DEVICE)
/**
 * @brief Gets the pointer to platform config.
 *
 * @return Pointer to the platform config
 */
kbase_platform_config *kbasep_get_platform_config(void);
#endif /* !MALI_LICENSE_IS_GPL || (defined(MALI_FAKE_PLATFORM_DEVICE) && MALI_FAKE_PLATFORM_DEVICE) */

#endif /* _KBASE_CONFIG_H_ */
