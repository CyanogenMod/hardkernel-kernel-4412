/* linux/arch/arm/mach-exynos4/dev-sysmmu.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4 - System MMU support
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

/* These names must be equal to the clock names in mach-exynos4/clock.c */
const char *sysmmu_ips_name[EXYNOS4_SYSMMU_TOTAL_IPNUM] = {
	"SYSMMU_MDMA"	,
	"SYSMMU_SSS"	,
	"SYSMMU_FIMC0"	,
	"SYSMMU_FIMC1"	,
	"SYSMMU_FIMC2"	,
	"SYSMMU_FIMC3"	,
	"SYSMMU_JPEG"	,
	"SYSMMU_FIMD0"	,
	"SYSMMU_FIMD1"	,
	"SYSMMU_PCIe"	,
	"SYSMMU_G2D"	,
	"SYSMMU_ROTATOR",
	"SYSMMU_MDMA2"	,
	"SYSMMU_TV"	,
	"SYSMMU_MFC_L"	,
	"SYSMMU_MFC_R"	,
};

static struct resource exynos4_sysmmu_resource[][2] = {
	[SYSMMU_MDMA] = {
		[0] = {
			.start	= EXYNOS4_PA_SYSMMU_MDMA,
			.end	= EXYNOS4_PA_SYSMMU_MDMA + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= IRQ_SYSMMU_MDMA0_0,
			.end	= IRQ_SYSMMU_MDMA0_0,
			.flags	= IORESOURCE_IRQ,
		},
	},
	[SYSMMU_SSS] = {
		[0] = {
			.start	= EXYNOS4_PA_SYSMMU_SSS,
			.end	= EXYNOS4_PA_SYSMMU_SSS + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= IRQ_SYSMMU_SSS_0,
			.end	= IRQ_SYSMMU_SSS_0,
			.flags	= IORESOURCE_IRQ,
		},
	},
	[SYSMMU_FIMC0] = {
		[0] = {
			.start = EXYNOS4_PA_SYSMMU_FIMC0,
			.end   = EXYNOS4_PA_SYSMMU_FIMC0 + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = IRQ_SYSMMU_FIMC0_0,
			.end   = IRQ_SYSMMU_FIMC0_0,
			.flags = IORESOURCE_IRQ,
		},
	},
	[SYSMMU_FIMC1] = {
		[0] = {
			.start = EXYNOS4_PA_SYSMMU_FIMC1,
			.end   = EXYNOS4_PA_SYSMMU_FIMC1 + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = IRQ_SYSMMU_FIMC1_0,
			.end   = IRQ_SYSMMU_FIMC1_0,
			.flags = IORESOURCE_IRQ,
		},
	},
	[SYSMMU_FIMC2] = {
		[0] = {
			.start = EXYNOS4_PA_SYSMMU_FIMC2,
			.end   = EXYNOS4_PA_SYSMMU_FIMC2 + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = IRQ_SYSMMU_FIMC2_0,
			.end   = IRQ_SYSMMU_FIMC2_0,
			.flags = IORESOURCE_IRQ,
		},
	},
	[SYSMMU_FIMC3] = {
		[0] = {
			.start = EXYNOS4_PA_SYSMMU_FIMC3,
			.end   = EXYNOS4_PA_SYSMMU_FIMC3 + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		},
		[1] = {
			.start = IRQ_SYSMMU_FIMC3_0,
			.end   = IRQ_SYSMMU_FIMC3_0,
			.flags = IORESOURCE_IRQ,
		},
	},
	[SYSMMU_JPEG] = {
		[0] = {
			.start	= EXYNOS4_PA_SYSMMU_JPEG,
			.end	= EXYNOS4_PA_SYSMMU_JPEG + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= IRQ_SYSMMU_JPEG_0,
			.end	= IRQ_SYSMMU_JPEG_0,
			.flags	= IORESOURCE_IRQ,
		},
	},
	[SYSMMU_FIMD0] = {
		[0] = {
			.start	= EXYNOS4_PA_SYSMMU_FIMD0,
			.end	= EXYNOS4_PA_SYSMMU_FIMD0 + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= IRQ_SYSMMU_LCD0_M0_0,
			.end	= IRQ_SYSMMU_LCD0_M0_0,
			.flags	= IORESOURCE_IRQ,
		},
	},
	[SYSMMU_FIMD1] = {
		[0] = {
			.start	= EXYNOS4_PA_SYSMMU_FIMD1,
			.end	= EXYNOS4_PA_SYSMMU_FIMD1 + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= IRQ_SYSMMU_LCD1_M1_0,
			.end	= IRQ_SYSMMU_LCD1_M1_0,
			.flags	= IORESOURCE_IRQ,
		},
	},
	[SYSMMU_PCIe] = {
		[0] = {
			.start	= EXYNOS4_PA_SYSMMU_PCIe,
			.end	= EXYNOS4_PA_SYSMMU_PCIe + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= IRQ_SYSMMU_PCIE_0,
			.end	= IRQ_SYSMMU_PCIE_0,
			.flags	= IORESOURCE_IRQ,
		},
	},
	[SYSMMU_G2D] = {
		[0] = {
			.start	= EXYNOS4_PA_SYSMMU_G2D,
			.end	= EXYNOS4_PA_SYSMMU_G2D + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= IRQ_SYSMMU_2D_0,
			.end	= IRQ_SYSMMU_2D_0,
			.flags	= IORESOURCE_IRQ,
		},
	},
	[SYSMMU_ROTATOR] = {
		[0] = {
			.start	= EXYNOS4_PA_SYSMMU_ROTATOR,
			.end	= EXYNOS4_PA_SYSMMU_ROTATOR + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= IRQ_SYSMMU_ROTATOR_0,
			.end	= IRQ_SYSMMU_ROTATOR_0,
			.flags	= IORESOURCE_IRQ,
		},
	},
	[SYSMMU_MDMA2] = {
		[0] = {
			.start	= EXYNOS4_PA_SYSMMU_MDMA2,
			.end	= EXYNOS4_PA_SYSMMU_MDMA2 + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= IRQ_SYSMMU_MDMA1_0,
			.end	= IRQ_SYSMMU_MDMA1_0,
			.flags	= IORESOURCE_IRQ,
		},
	},
	[SYSMMU_TV] = {
		[0] = {
			.start	= EXYNOS4_PA_SYSMMU_TV,
			.end	= EXYNOS4_PA_SYSMMU_TV + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= IRQ_SYSMMU_TV_M0_0,
			.end	= IRQ_SYSMMU_TV_M0_0,
			.flags	= IORESOURCE_IRQ,
		},
	},
	[SYSMMU_MFC_L] = {
		[0] = {
			.start	= EXYNOS4_PA_SYSMMU_MFC_L,
			.end	= EXYNOS4_PA_SYSMMU_MFC_L + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= IRQ_SYSMMU_MFC_M0_0,
			.end	= IRQ_SYSMMU_MFC_M0_0,
			.flags	= IORESOURCE_IRQ,
		},
	},
	[SYSMMU_MFC_R] = {
		[0] = {
			.start	= EXYNOS4_PA_SYSMMU_MFC_R,
			.end	= EXYNOS4_PA_SYSMMU_MFC_R + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= IRQ_SYSMMU_MFC_M1_0,
			.end	= IRQ_SYSMMU_MFC_M1_0,
			.flags	= IORESOURCE_IRQ,
		},
	},
};

static u64 exynos4_sysmmu_dma_mask = DMA_BIT_MASK(32);

struct platform_device exynos4_device_sysmmu[] = {
	[SYSMMU_MDMA] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_MDMA,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_MDMA]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_MDMA],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_SSS] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_SSS,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_SSS]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_SSS],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_FIMC0] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_FIMC0,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_FIMC0]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_FIMC0],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_FIMC1] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_FIMC1,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_FIMC1]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_FIMC1],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_FIMC2] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_FIMC2,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_FIMC2]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_FIMC2],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_FIMC3] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_FIMC3,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_FIMC3]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_FIMC3],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_JPEG] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_JPEG,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_JPEG]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_JPEG],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_FIMD0] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_FIMD0,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_FIMD0]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_FIMD0],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_FIMD1] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_FIMD1,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_FIMD1]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_FIMD1],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_PCIe] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_PCIe,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_PCIe]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_PCIe],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_G2D] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_G2D,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_G2D]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_G2D],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_ROTATOR] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_ROTATOR,
		.num_resources	= ARRAY_SIZE(
				exynos4_sysmmu_resource[SYSMMU_ROTATOR]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_ROTATOR],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_MDMA2] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_MDMA2,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_MDMA2]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_MDMA2],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_TV] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_TV,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_TV]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_TV],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_MFC_L] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_MFC_L,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_MFC_L]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_MFC_L],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
	[SYSMMU_MFC_R] = {
		.name		= "s5p-sysmmu",
		.id		= SYSMMU_MFC_R,
		.num_resources	= ARRAY_SIZE(
					exynos4_sysmmu_resource[SYSMMU_MFC_R]),
		.resource	= exynos4_sysmmu_resource[SYSMMU_MFC_R],
		.dev		= {
			.dma_mask		= &exynos4_sysmmu_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
	},
};

static struct clk *sysmmu_clk[S5P_SYSMMU_TOTAL_IPNUM];
void sysmmu_clk_init(sysmmu_ips ips, struct device *dev)
{
	sysmmu_clk[ips] = clk_get(dev, "sysmmu");
	if (IS_ERR(sysmmu_clk[ips]))
		sysmmu_clk[ips] = NULL;
}

void sysmmu_clk_enable(sysmmu_ips ips)
{
	if (sysmmu_clk[ips])
		clk_enable(sysmmu_clk[ips]);
}

void sysmmu_clk_disable(sysmmu_ips ips)
{
	if (sysmmu_clk[ips])
		clk_disable(sysmmu_clk[ips]);
}
