/*
 *
 * (C) COPYRIGHT 2011 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * 
 */



#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_defs.h>
#include <osk/mali_osk.h>
#include <ump/ump_common.h>

/* Specifies how many attributes are permitted in the config (excluding terminating attribute).
 * This is used in validation function so we can detect if configuration is properly terminated. This value can be
 * changed if we need to introduce more attributes or many memory regions need to be defined */
#define ATTRIBUTE_COUNT_MAX 8

/* right now we allow only 2 memory attributes (excluding termination attribute) */
#define MEMORY_ATTRIBUTE_COUNT_MAX 2

/* Limits for gpu frequency configuration parameters. These will use for config validation. */
#define MAX_GPU_ALLOWED_FREQ_KHZ 1000000
#define MIN_GPU_ALLOWED_FREQ_KHZ 1

/* Default irq throttle time. This is the default desired minimum time in between two consecutive
 * interrupts from the gpu. The irq throttle gpu register is set after this value. */
#define DEFAULT_IRQ_THROTTLE_TIME_US 20

#if (!defined(MALI_KBASE_USERSPACE) || !MALI_KBASE_USERSPACE) && (!MALI_LICENSE_IS_GPL || MALI_FAKE_PLATFORM_DEVICE)

extern kbase_platform_config platform_config;
kbase_platform_config *kbasep_get_platform_config(void)
{
	return &platform_config;
}
#endif /* (!defined(MALI_KBASE_USERSPACE) || !MALI_KBASE_USERSPACE) && (!MALI_LICENSE_IS_GPL || MALI_FAKE_PLATFORM_DEVICE) */

#

int kbasep_get_config_attribute_count(const kbase_attribute *attributes)
{
	int count = 1;

	OSK_ASSERT(attributes != NULL);

	while (attributes->id != KBASE_CONFIG_ATTR_END)
	{
		attributes++;
		count++;
	}

	return count;
}

int kbasep_get_config_attribute_count_by_id(const kbase_attribute *attributes, int attribute_id)
{
	int count = 0;
	OSK_ASSERT(attributes != NULL);

	while (attributes->id != KBASE_CONFIG_ATTR_END)
	{
		if (attributes->id == attribute_id)
		{
			count++;
		}
		attributes++;
	}

	return count;
}

static const char* midgard_type_strings[] =
	{
		"mali-t6xm",
		"mali-t6f1",
		"mali-t601",
		"mali-t604",
		"mali-t608"
	};

const char *kbasep_midgard_type_to_string(kbase_midgard_type midgard_type)
{
	OSK_ASSERT(midgard_type >= 0 && midgard_type < KBASE_MALI_COUNT);

	return midgard_type_strings[midgard_type];
}

const kbase_attribute *kbasep_get_next_attribute(const kbase_attribute *attributes, int attribute_id)
{
	OSK_ASSERT(attributes != NULL);

	while (attributes->id != KBASE_CONFIG_ATTR_END)
	{
		if (attributes->id == attribute_id)
		{
			return attributes;
		}
		attributes++;
	}
	return NULL;
}

uintptr_t kbasep_get_config_value(const kbase_attribute *attributes, int attribute_id)
{
	const kbase_attribute *attr;

	OSK_ASSERT(attributes != NULL);

	attr = kbasep_get_next_attribute(attributes, attribute_id);
	if (attr != NULL)
	{
		return attr->data;
	}

	/* default values */
	switch (attribute_id)
	{
		case KBASE_CONFIG_ATTR_MEMORY_PER_PROCESS_LIMIT:
			return (uintptr_t)-1;
		case KBASE_CONFIG_ATTR_UMP_DEVICE:
			return UMP_DEVICE_W_SHIFT;
		case KBASE_CONFIG_ATTR_MEMORY_OS_SHARED_MAX:
			return (uintptr_t)-1;
		case KBASE_CONFIG_ATTR_MEMORY_OS_SHARED_PERF_GPU:
			return KBASE_MEM_PERF_NORMAL;
	    case KBASE_CONFIG_ATTR_GPU_IRQ_THROTTLE_TIME_US:
			return DEFAULT_IRQ_THROTTLE_TIME_US;
		default:
			OSK_PRINT_ERROR(OSK_BASE_CORE, "kbasep_get_config_value. Cannot get value of attribute with id=%i and no default value defined",
					attribute_id);
			return 0;
	}
}


void kbasep_get_memory_performance(const kbase_memory_resource *resource, kbase_memory_performance *cpu_performance,
		kbase_memory_performance *gpu_performance)
{
	kbase_attribute *attributes;

	OSK_ASSERT(resource != NULL);
	OSK_ASSERT(cpu_performance != NULL );
	OSK_ASSERT(gpu_performance != NULL);

	attributes = resource->attributes;
	*cpu_performance = *gpu_performance = KBASE_MEM_PERF_NORMAL; /* default performance */

	if (attributes == NULL)
	{
		return;
	}

	while (attributes->id != KBASE_CONFIG_ATTR_END)
	{
		if (attributes->id == KBASE_MEM_ATTR_PERF_GPU)
		{
			*gpu_performance = (kbase_memory_performance) attributes->data;
		}
		else if (attributes->id == KBASE_MEM_ATTR_PERF_CPU)
		{
			*cpu_performance = (kbase_memory_performance) attributes->data;
		}
		attributes++;
	}
}

static mali_bool kbasep_validate_ump_device(int ump_device)
{
	mali_bool valid;

	switch (ump_device)
	{
		case UMP_DEVICE_W_SHIFT:
		case UMP_DEVICE_X_SHIFT:
		case UMP_DEVICE_Y_SHIFT:
		case UMP_DEVICE_Z_SHIFT:
			valid = MALI_TRUE;
			break;
		default:
			valid = MALI_FALSE;
			break;
	}
	return valid;
}

static mali_bool kbasep_validate_memory_performance(kbase_memory_performance performance)
{
	return performance >= KBASE_MEM_PERF_SLOW && performance <= KBASE_MEM_PERF_FAST;
}

static mali_bool kbasep_validate_memory_resource(const kbase_memory_resource *memory_resource)
{
	OSK_ASSERT(memory_resource != NULL);

	if (memory_resource->name == NULL)
	{
		OSK_PRINT_WARN(OSK_BASE_CORE, "Unnamed memory region found");
		return MALI_FALSE;
	}

	if (memory_resource->base & ((1 << OSK_PAGE_SHIFT) - 1))
	{
		OSK_PRINT_WARN(OSK_BASE_CORE, "Base address of \"%s\" memory region is not page aligned", memory_resource->name);
		return MALI_FALSE;
	}

	if (memory_resource->size & ((1 << OSK_PAGE_SHIFT) - 1))
	{
		OSK_PRINT_WARN(OSK_BASE_CORE, "Size of \"%s\" memory region is not a multiple of page size", memory_resource->name);
		return MALI_FALSE;
	}

	if (memory_resource->attributes != NULL) /* we allow NULL attribute list */
	{
		int i;

		for (i = 0; memory_resource->attributes[i].id != KBASE_MEM_ATTR_END; i++)
		{
			if (i >= MEMORY_ATTRIBUTE_COUNT_MAX)
			{
				OSK_PRINT_WARN(OSK_BASE_CORE, "More than MEMORY_ATTRIBUTE_COUNT_MAX=%i configuration attributes defined. Is memory attribute list properly terminated?",
						MEMORY_ATTRIBUTE_COUNT_MAX);
				return MALI_FALSE;
			}
			switch(memory_resource->attributes[i].id)
			{
				case KBASE_MEM_ATTR_PERF_CPU:
					if (MALI_TRUE != kbasep_validate_memory_performance(
							(kbase_memory_performance)memory_resource->attributes[i].data))
					{
						OSK_PRINT_WARN(OSK_BASE_CORE, "CPU performance of \"%s\" region is invalid: %i",
								memory_resource->name, (kbase_memory_performance)memory_resource->attributes[i].data);
						return MALI_FALSE;
					}
					break;

				case KBASE_MEM_ATTR_PERF_GPU:
					if (MALI_TRUE != kbasep_validate_memory_performance(
											(kbase_memory_performance)memory_resource->attributes[i].data))
					{
						OSK_PRINT_WARN(OSK_BASE_CORE, "GPU performance of \"%s\" region is invalid: %i",
								memory_resource->name, (kbase_memory_performance)memory_resource->attributes[i].data);
							return MALI_FALSE;
					}
					break;
				default:
					OSK_PRINT_WARN(OSK_BASE_CORE, "Invalid memory attribute found in \"%s\" memory region: %i",
							memory_resource->name, memory_resource->attributes[i].id);
					return MALI_FALSE;
			}
		}
	}

	return MALI_TRUE;
}


static mali_bool kbasep_validate_gpu_clock_freq(const kbase_attribute *attributes)
{
	uintptr_t freq_min = kbasep_get_config_value(attributes, KBASE_CONFIG_ATTR_GPU_FREQ_KHZ_MIN);
	uintptr_t freq_max = kbasep_get_config_value(attributes, KBASE_CONFIG_ATTR_GPU_FREQ_KHZ_MAX);

	if ((freq_min > MAX_GPU_ALLOWED_FREQ_KHZ) ||
		(freq_min < MIN_GPU_ALLOWED_FREQ_KHZ) ||
		(freq_max > MAX_GPU_ALLOWED_FREQ_KHZ) ||
		(freq_max < MIN_GPU_ALLOWED_FREQ_KHZ) ||
		(freq_min > freq_max))
	{
		OSK_PRINT_WARN(OSK_BASE_CORE, "Invalid GPU frequencies found in configuration: min=%lkHz, max=%lkHz.", freq_min, freq_max);
		return MALI_FALSE;
	}
	
	return MALI_TRUE;
}


mali_bool kbasep_validate_configuration_attributes(const kbase_attribute *attributes)
{
	int i;

	OSK_ASSERT(attributes);

	for (i = 0; attributes[i].id != KBASE_CONFIG_ATTR_END; i++)
	{
		if (i >= ATTRIBUTE_COUNT_MAX)
		{
			OSK_PRINT_WARN(OSK_BASE_CORE, "More than ATTRIBUTE_COUNT_MAX=%i configuration attributes defined. Is attribute list properly terminated?",
					ATTRIBUTE_COUNT_MAX);
			return MALI_FALSE;
		}

		switch (attributes[i].id)
		{
			case KBASE_CONFIG_ATTR_MEMORY_RESOURCE:
				if (MALI_FALSE == kbasep_validate_memory_resource((kbase_memory_resource *)attributes[i].data))
				{
					OSK_PRINT_WARN(OSK_BASE_CORE, "Invalid memory region found in configuration");
					return MALI_FALSE;
				}
				break;
			case KBASE_CONFIG_ATTR_MEMORY_OS_SHARED_MAX:
				/* any value is allowed */
				break;

			case KBASE_CONFIG_ATTR_MEMORY_OS_SHARED_PERF_GPU:
				if (MALI_FALSE == kbasep_validate_memory_performance((kbase_memory_performance)attributes[i].data))
				{
					OSK_PRINT_WARN(OSK_BASE_CORE, "Shared OS memory GPU performance attribute has invalid value: %i",
							(kbase_memory_performance)attributes[i].data);
					return MALI_FALSE;
				}
				break;

			case KBASE_CONFIG_ATTR_MEMORY_PER_PROCESS_LIMIT:
				/* any value is allowed */
				break;

			case KBASE_CONFIG_ATTR_UMP_DEVICE:
				if (MALI_FALSE == kbasep_validate_ump_device(attributes[i].data))
				{
					OSK_PRINT_WARN(OSK_BASE_CORE, "Unknown UMP device found in configuration: %i",
							(int)attributes[i].data);
					return MALI_FALSE;
				}
				break;

		    case KBASE_CONFIG_ATTR_GPU_FREQ_KHZ_MIN:
		    case KBASE_CONFIG_ATTR_GPU_FREQ_KHZ_MAX:
				if (MALI_FALSE == kbasep_validate_gpu_clock_freq(attributes))
				{
					return MALI_FALSE;
				}
				break;

			default:
				OSK_PRINT_WARN(OSK_BASE_CORE, "Invalid attribute found in configuration: %i", attributes[i].id);
				return MALI_FALSE;
		}
	}

	return MALI_TRUE;
}

