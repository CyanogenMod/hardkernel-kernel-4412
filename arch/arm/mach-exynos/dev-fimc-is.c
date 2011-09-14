/* linux/arch/arm/plat-s5p/dev-fimc_is.c
 *
 * Copyright (c) 2011 Samsung Electronics
 *
 * Base FIMC-IS resource and device definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <media/exynos_fimc_is.h>

static struct resource exynos4_fimc_is_resource[] = {
	[0] = {
		.start	= EXYNOS4_PA_FIMC_IS,
		.end	= EXYNOS4_PA_FIMC_IS + SZ_2M + SZ_256K + SZ_128K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_FIMC_IS0,
		.end	= IRQ_FIMC_IS0,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= IRQ_FIMC_IS1,
		.end	= IRQ_FIMC_IS1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device exynos4_device_fimc_is = {
	.name		= "exynos4-fimc-is",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(exynos4_fimc_is_resource),
	.resource	= exynos4_fimc_is_resource,
};

struct exynos4_platform_fimc_is exynos4_fimc_is_default_data __initdata = {
	.fmt		= IS_MIPI_CSI_RAW10,
};

void __init exynos4_fimc_is_set_platdata(struct exynos4_platform_fimc_is *pd)
{
	struct exynos4_platform_fimc_is *npd;

	if (!pd)
		pd = &exynos4_fimc_is_default_data;

	npd = kmemdup(pd, sizeof(struct exynos4_platform_fimc_is), GFP_KERNEL);

	if (!npd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);
	else
		exynos4_device_fimc_is.dev.platform_data = npd;
}

