/* linux/arch/arm/mach-exynos/dev-c2c.c
 *
 * Copyright (c) 2011 Samsung Electronics
 *
 * Base EXYNOS4 C2C resource and device definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>

#include <mach/map.h>
#include <mach/c2c.h>
#include <mach/regs-pmu.h>
#include <plat/irqs.h>

static struct resource exynos_c2c_resource[] = {
	[0] = {
		.start  = EXYNOS4_PA_C2C,
		.end    = EXYNOS4_PA_C2C + SZ_32K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = EXYNOS4_PA_C2C_CP,
		.end    = EXYNOS4_PA_C2C_CP + SZ_32K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[2] = {
		.start  = IRQ_C2C_SSCM0,
		.end    = IRQ_C2C_SSCM0,
		.flags  = IORESOURCE_IRQ,
	},
	[3] = {
		.start  = IRQ_C2C_SSCM1,
		.end    = IRQ_C2C_SSCM1,
		.flags  = IORESOURCE_IRQ,
	},
};

static u64 exynos4_c2c_dma_mask = DMA_BIT_MASK(32);

struct platform_device exynos_device_c2c = {
	.name		= "samsung-c2c",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(exynos_c2c_resource),
	.resource	= exynos_c2c_resource,
	.dev		= {
		.dma_mask		= &exynos4_c2c_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

void __init exynos4_c2c_set_platdata(struct exynos_c2c_platdata *pd)
{
	struct exynos_c2c_platdata *npd = pd;

	if (!npd->setup_gpio)
		npd->setup_gpio = exynos4_c2c_cfg_gpio;
	if (!npd->set_cprst)
		npd->set_cprst = exynos4_c2c_set_cprst;
	if (!npd->clear_cprst)
		npd->clear_cprst = exynos4_c2c_clear_cprst;

	exynos_device_c2c.dev.platform_data = npd;

	/* Set C2C_CTRL Register */
	writel(0x1, S5P_C2C_CTRL);
}
