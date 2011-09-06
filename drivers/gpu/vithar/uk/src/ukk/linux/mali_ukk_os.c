/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2010-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

#include <linux/module.h>	/* Needed by all modules */
#include <linux/kernel.h>	/* Needed for KERN_INFO */
#include <linux/init.h>		/* Needed for the macros */

#include <osk/mali_osk.h>
#include <uk/mali_ukk.h>
#include <plat/mali_uk_os.h>

mali_error ukk_buffer_open(ukk_buffer * const buffer, void * const user_buffer, size_t size, ukk_buffer_type type)
{
	void * v;

	OSK_ASSERT(NULL != buffer);
	OSK_ASSERT(NULL != user_buffer);
	OSK_ASSERT(size != 0);

	buffer->user_buffer = user_buffer;
	buffer->size = size;
	buffer->type = type;
	buffer->kernel_buffer = NULL;

	v = osk_malloc(size);
	if (NULL == v)
	{
		return MALI_ERROR_OUT_OF_MEMORY;
	}

	if (UKK_BUFFER_OUTPUT != type)
	{
		if (0 != copy_from_user(v, user_buffer, size))
		{
			osk_free(v);
			return MALI_ERROR_FUNCTION_FAILED;
		}
	}

	buffer->kernel_buffer = v;

	return MALI_ERROR_NONE;
}

mali_error ukk_buffer_close(ukk_buffer * const buffer)
{
	mali_error ret = MALI_ERROR_NONE;

	OSK_ASSERT(NULL != buffer);
	OSK_ASSERT(NULL != buffer->user_buffer);
	OSK_ASSERT(buffer->size != 0);

	if (NULL != buffer->kernel_buffer)
	{
		if (UKK_BUFFER_INPUT != buffer->type)
		{
			if (0 != copy_to_user(buffer->user_buffer, buffer->kernel_buffer, buffer->size))
			{
				ret = MALI_ERROR_FUNCTION_FAILED;
			}
		}
		osk_free((void *)buffer->kernel_buffer);
	}

	buffer->kernel_buffer = NULL;
	buffer->user_buffer = NULL;
	buffer->size = 0;

	return ret;
}


mali_error ukk_session_init(ukk_session *ukk_session, ukk_dispatch_function dispatch, u16 version_major, u16 version_minor)
{
	OSK_ASSERT(NULL != ukk_session);
	OSK_ASSERT(NULL != dispatch);

	/* OS independent initialization of UKK context */
	ukk_session->dispatch = dispatch;
	ukk_session->version_major = version_major;
	ukk_session->version_minor = version_minor;

	/* OS specific initialization of UKK context */
	ukk_session->internal_session.dummy = 0;
	return MALI_ERROR_NONE;
}

void ukk_session_term(ukk_session *ukk_session)
{
	OSK_ASSERT(NULL != ukk_session);
}

static int __init ukk_module_init(void)
{
	if (MALI_ERROR_NONE != ukk_start())
	{
		return -EINVAL;
	}
	return 0;
}

static void __exit ukk_module_exit(void)
{
	ukk_stop();
}

EXPORT_SYMBOL(ukk_session_init);
EXPORT_SYMBOL(ukk_session_term);
EXPORT_SYMBOL(ukk_session_get);
EXPORT_SYMBOL(ukk_call_prepare);
EXPORT_SYMBOL(ukk_call_data_set);
EXPORT_SYMBOL(ukk_call_data_get);
EXPORT_SYMBOL(ukk_dispatch);
EXPORT_SYMBOL(ukk_thread_ctx_get);
EXPORT_SYMBOL(ukk_buffer_open);
EXPORT_SYMBOL(ukk_buffer_close);
EXPORT_SYMBOL(ukk_buffer_get);

module_init(ukk_module_init);
module_exit(ukk_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ARM Ltd.");
MODULE_VERSION("0.0");

