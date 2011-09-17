/* linux/arch/arm/mach-exynos/dev-sysmmu.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS - System MMU support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <mach/map.h>
#include <mach/irqs.h>
#include <mach/sysmmu.h>
#include <plat/s5p-clock.h>

#define SYSMMU_RESOURCE(name, irq) [SYSMMU_##name] = {\
		[0] = {\
			.start	= EXYNOS4_PA_SYSMMU_##name,\
			.end	= EXYNOS4_PA_SYSMMU_##name + SZ_4K - 1,\
			.flags	= IORESOURCE_MEM,\
		},\
		[1] = {\
			.start	= IRQ_SYSMMU_##irq##_0,\
			.end	= IRQ_SYSMMU_##irq##_0,\
			.flags	= IORESOURCE_IRQ,\
		},\
	}

static u64 exynos_sysmmu_dma_mask = DMA_BIT_MASK(32);

#define SYSMMU_PLATFORM_DEVICE(ips) [SYSMMU_##ips] = {\
	.name		= "s5p-sysmmu",\
	.id		= SYSMMU_##ips,\
	.num_resources	= ARRAY_SIZE(\
				exynos_sysmmu_resource[SYSMMU_##ips]),\
	.resource	= exynos_sysmmu_resource[SYSMMU_##ips],\
	.dev		= {\
		.dma_mask		= &exynos_sysmmu_dma_mask,\
		.coherent_dma_mask	= DMA_BIT_MASK(32),\
	},\
}

static struct resource exynos_sysmmu_resource[S5P_SYSMMU_TOTAL_IPNUM][2] = {
	SYSMMU_RESOURCE(MDMA, MDMA0),
	SYSMMU_RESOURCE(SSS, SSS),
	SYSMMU_RESOURCE(FIMC0, FIMC0),
	SYSMMU_RESOURCE(FIMC1, FIMC1),
	SYSMMU_RESOURCE(FIMC2, FIMC2),
	SYSMMU_RESOURCE(FIMC3, FIMC3),
	SYSMMU_RESOURCE(JPEG, JPEG),
	SYSMMU_RESOURCE(FIMD0, LCD0_M0),
	SYSMMU_RESOURCE(FIMD1, LCD1_M1),
	SYSMMU_RESOURCE(PCIe, PCIE),
	SYSMMU_RESOURCE(G2D, 2D),
	SYSMMU_RESOURCE(ROTATOR, ROTATOR),
	SYSMMU_RESOURCE(MDMA2, MDMA1),
	SYSMMU_RESOURCE(TV, TV_M0),
	SYSMMU_RESOURCE(MFC_L, MFC_M0),
	SYSMMU_RESOURCE(MFC_R, MFC_M1),
};

struct platform_device exynos_device_sysmmu[S5P_SYSMMU_TOTAL_IPNUM] = {
	SYSMMU_PLATFORM_DEVICE(MDMA),
	SYSMMU_PLATFORM_DEVICE(SSS),
	SYSMMU_PLATFORM_DEVICE(FIMC0),
	SYSMMU_PLATFORM_DEVICE(FIMC1),
	SYSMMU_PLATFORM_DEVICE(FIMC2),
	SYSMMU_PLATFORM_DEVICE(FIMC3),
	SYSMMU_PLATFORM_DEVICE(JPEG),
	SYSMMU_PLATFORM_DEVICE(FIMD0),
	SYSMMU_PLATFORM_DEVICE(FIMD1),
	SYSMMU_PLATFORM_DEVICE(PCIe),
	SYSMMU_PLATFORM_DEVICE(G2D),
	SYSMMU_PLATFORM_DEVICE(ROTATOR),
	SYSMMU_PLATFORM_DEVICE(MDMA2),
	SYSMMU_PLATFORM_DEVICE(TV),
	SYSMMU_PLATFORM_DEVICE(MFC_L),
	SYSMMU_PLATFORM_DEVICE(MFC_R),
};
