/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2010-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file mali_kbase_mem_linux.h
 * Base kernel APIs, Linux implementation.
 */

#ifndef _KBASE_LINUX_H_
#define _KBASE_LINUX_H_

/* All things that are needed for the Linux port. */
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/list.h>

typedef struct kbase_os_context
{
	u64			cookies;
	osk_dlist		reg_pending;
	wait_queue_head_t	event_queue;
} kbase_os_context;


#define DEVNAME_SIZE	16

typedef struct kbase_os_device
{
	struct list_head	entry;
	struct device		*dev;
	struct miscdevice	mdev;
	unsigned long		reg_start;
	size_t			reg_size;
	void __iomem		*reg;
	struct resource		*reg_res;
	struct {
		int		irq;
		int		flags;
	} irqs[3];
	char			devname[DEVNAME_SIZE];
} kbase_os_device;

#define KBASE_OS_SUPPORT	1

#endif /* _KBASE_LINUX_H_ */
