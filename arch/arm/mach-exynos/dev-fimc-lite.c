/* linux/arch/arm/plat-s5p/dev-fimc-lite.c
 *
 * Copyright (c) 2011 Samsung Electronics
 *
 * Base S5P FIMC-Lite resource and device definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <mach/map.h>
#include <media/exynos_flite.h>

static struct resource exynos_flite0_resource[] = {
	[0] = {
		.start	= EXYNOS_PA_FIMC_LITE0,
		.end	= EXYNOS_PA_FIMC_LITE0 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_FIMC_LITE0,
		.end	= IRQ_FIMC_LITE0,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device exynos_device_flite0 = {
	.name		= "exynos-fimc-lite",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(exynos_flite0_resource),
	.resource	= exynos_flite0_resource,
};

void __init exynos_flite0_set_platdata(struct exynos_platform_flite *pd)
{
	struct exynos_platform_flite *npd;

	npd = kmemdup(pd, sizeof(struct exynos_platform_flite), GFP_KERNEL);

	if (!npd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);
	else
		exynos_device_flite0.dev.platform_data = npd;
}

static struct resource exynos_flite1_resource[] = {
	[0] = {
		.start	= EXYNOS_PA_FIMC_LITE1,
		.end	= EXYNOS_PA_FIMC_LITE1 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_FIMC_LITE1,
		.end	= IRQ_FIMC_LITE1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device exynos_device_flite1 = {
	.name		= "exynos-fimc-lite",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(exynos_flite1_resource),
	.resource	= exynos_flite1_resource,
};

void __init exynos_flite1_set_platdata(struct exynos_platform_flite *pd)
{
	struct exynos_platform_flite *npd;

	npd = kmemdup(pd, sizeof(struct exynos_platform_flite), GFP_KERNEL);

	if (!npd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);
	else
		exynos_device_flite1.dev.platform_data = npd;
}
