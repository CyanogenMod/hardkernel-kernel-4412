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

/**
 * @brief Contains the module names (modules in the same order as for the osk_module enumeration)
 * @sa oskp_module_to_str
 */
static const char* CONST oskp_str_modules[] =
{
	"UNKNOWN",     /**< Unknown module */
	"OSK",         /**< OSK */
	"UKK",         /**< UKK */
	"BASE_MMU",    /**< Base MMU */
	"BASE_JD",     /**< Base Job Dispatch */
	"BASE_JM",     /**< Base Job Manager */
	"BASE_CORE",   /**< Base Core */
	"BASE_MEM",    /**< Base Memory */
	"BASE_EVENT",  /**< Base Event */
	"BASE_CTX",    /**< Base Context */
	"BASE_PM",     /**< Base Power Management */
	"UMP",         /**< UMP */
};

#define MODULE_STRING_ARRAY_SIZE (sizeof(oskp_str_modules)/sizeof(oskp_str_modules[0]))

static INLINE void oskp_compile_time_assertions(void)
{
	/*
	 * If this assert triggers you have forgotten to update oskp_str_modules
	 * when you added a module to the osk_module enum
	 * */
	CSTD_COMPILE_TIME_ASSERT(OSK_MODULES_ALL == MODULE_STRING_ARRAY_SIZE );
}

const char* oskp_module_to_str(const osk_module module)
{
	if( MODULE_STRING_ARRAY_SIZE <= module)
	{
		return "";
	}
	return oskp_str_modules[module];
}
