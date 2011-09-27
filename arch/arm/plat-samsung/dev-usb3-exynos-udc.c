/* arch/arm/plat-samsung/dev-usb3-dwc-udc.c
 *
 * Copyright (c) 2011 Samsung Electronics Co. Ltd
 * Author: Anton Tikhomirov <av.tikhomirov@samsung.com>
 *
 * Device definition for DWC SuperSpeed USB 3.0 Device Controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <mach/irqs.h>
#include <mach/map.h>

#include <plat/devs.h>

static struct resource exynos_ss_udc_resources[] = {
	[0] = {
		.start	= EXYNOS5_PA_SS_UDC,
		.end	= EXYNOS5_PA_SS_UDC + 0xB00 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_USB3_DRD,
		.end	= IRQ_USB3_DRD,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 exynos_ss_udc_dmamask = DMA_BIT_MASK(32);

struct platform_device exynos_device_ss_udc = {
	.name		= "exynos-ss-udc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(exynos_ss_udc_resources),
	.resource	= exynos_ss_udc_resources,
	.dev		= {
		.dma_mask		= &exynos_ss_udc_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

