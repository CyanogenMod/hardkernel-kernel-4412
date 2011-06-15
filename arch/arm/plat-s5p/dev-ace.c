/*
 * linux/arch/arm/plat-s5p/dev-ace.c
 *
 * Copyright (C) 2011 Samsung Electronics
 *
 * Base S5P Crypto Engine resource and device definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>

#include <mach/map.h>
#include <mach/irqs.h>

static struct resource s5p_ace_resource[] = {
	[0] = {
		.start	= S5P_PA_ACE,
		.end	= S5P_PA_ACE + SZ_32K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_INTFEEDCTRL_SSS,
		.end	= IRQ_INTFEEDCTRL_SSS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device s5p_device_ace = {
	.name		= "s5p-ace",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(s5p_ace_resource),
	.resource	= s5p_ace_resource,
};
EXPORT_SYMBOL(s5p_device_ace);
