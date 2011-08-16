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

/* FIXME */
static struct resource dwc_usb3_udc_resources[] = {
#if 0
	[0] = {
		.start	= S5P_PA_OTG,
		.end	= S5P_PA_OTG + 0x10000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_OTG,
		.end	= IRQ_OTG,
		.flags	= IORESOURCE_IRQ,
	},
#endif
};
/********************/
static u64 dwc_usb3_udc_dmamask = DMA_BIT_MASK(32);

struct platform_device dwc_device_usb3_udc = {
	.name		= "dwc-usb3-udc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(dwc_usb3_udc_resources),
	.resource	= dwc_usb3_udc_resources,
	.dev		= {
		.dma_mask		= &dwc_usb3_udc_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

