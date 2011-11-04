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



#include <linux/module.h>	/* Needed by all modules */
#include <linux/kernel.h>	/* Needed for KERN_INFO */
#include <linux/init.h>		/* Needed for the macros */

#include <osk/mali_osk.h>
#include <uk/mali_ukk.h>

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

#if MALI_LICENSE_IS_GPL || MALI_UNIT_TEST /* See MIDBASE-1204 */
MODULE_LICENSE("GPL");
#else
MODULE_LICENSE("Proprietary");
#endif
MODULE_AUTHOR("ARM Ltd.");
MODULE_VERSION("0.0");

