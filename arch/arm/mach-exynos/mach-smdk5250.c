/* linux/arch/arm/mach-exynos/mach-smdk5250.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/mmc/host.h>
#include <linux/cma.h>
#include <linux/memblock.h>
#include <linux/fb.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <plat/exynos5.h>
#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/sdhci.h>

#include <plat/fb.h>
#include <plat/fb-s5p.h>
#include <plat/fb-core.h>
#include <plat/regs-fb-v4.h>

#include <mach/map.h>
#include <mach/exynos-ion.h>

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDK5250_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDK5250_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDK5250_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdk5250_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDK5250_UCON_DEFAULT,
		.ulcon		= SMDK5250_ULCON_DEFAULT,
		.ufcon		= SMDK5250_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDK5250_UCON_DEFAULT,
		.ulcon		= SMDK5250_ULCON_DEFAULT,
		.ufcon		= SMDK5250_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDK5250_UCON_DEFAULT,
		.ulcon		= SMDK5250_ULCON_DEFAULT,
		.ufcon		= SMDK5250_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDK5250_UCON_DEFAULT,
		.ulcon		= SMDK5250_ULCON_DEFAULT,
		.ufcon		= SMDK5250_UFCON_DEFAULT,
	},
};

#ifdef CONFIG_FB_S3C
static struct s3c_fb_pd_win smdk5250_fb_win0 = {
	.win_mode = {
		.left_margin	= 80,
		.right_margin	= 48,
		.upper_margin	= 14,
		.lower_margin	= 3,
		.hsync_len	= 32,
		.vsync_len	= 5,
		.xres		= 1360, /* real size : 1366 */
		.yres		= 768,
	},
	.virtual_x		= 1360, /* real size : 1366 */
	.virtual_y		= 768 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk5250_fb_win1 = {
	.win_mode = {
		.left_margin	= 80,
		.right_margin	= 48,
		.upper_margin	= 14,
		.lower_margin	= 3,
		.hsync_len	= 32,
		.vsync_len	= 5,
		.xres		= 1360, /* real size : 1366 */
		.yres		= 768,
	},
	.virtual_x		= 1360, /* real size : 1366 */
	.virtual_y		= 768 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk5250_fb_win2 = {
	.win_mode = {
		.left_margin	= 80,
		.right_margin	= 48,
		.upper_margin	= 14,
		.lower_margin	= 3,
		.hsync_len	= 32,
		.vsync_len	= 5,
		.xres		= 1360, /* real size : 1366 */
		.yres		= 768,
	},
	.virtual_x		= 1360, /* real size : 1366 */
	.virtual_y		= 768 * 2,
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static void exynos_fimd_gpio_setup_24bpp(void)
{
	unsigned int reg = 0;

	/*
	 * Set DISP1BLK_CFG register for Display path selection
	 *
	 * FIMD of DISP1_BLK Bypass selection : DISP1BLK_CFG[15]
	 * ---------------------
	 *  0 | MIE/MDNIE
	 *  1 | FIMD : selected
	 */
	reg = __raw_readl(S3C_VA_SYS + 0x0214);
	reg &= ~(1 << 15);	/* To save other reset values */
	reg |= (1 << 15);
	__raw_writel(reg, S3C_VA_SYS + 0x0214);
}

static struct s3c_fb_platdata smdk5250_lcd1_pdata __initdata = {
	.win[0]		= &smdk5250_fb_win0,
	.win[1]		= &smdk5250_fb_win1,
	.win[2]		= &smdk5250_fb_win2,
	.default_win	= 2,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= 0,
	.setup_gpio	= exynos_fimd_gpio_setup_24bpp,
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC
static struct s3c_sdhci_platdata smdk5250_hsmmc0_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH0_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC1
static struct s3c_sdhci_platdata smdk5250_hsmmc1_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC2
static struct s3c_sdhci_platdata smdk5250_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH2_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC3
static struct s3c_sdhci_platdata smdk5250_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};
#endif

static struct platform_device *smdk5250_devices[] __initdata = {
#ifdef CONFIG_FB_S3C
	&s5p_device_fimd1,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	&s3c_device_hsmmc0,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	&s3c_device_hsmmc1,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	&s3c_device_hsmmc3,
#endif
#ifdef CONFIG_ION_EXYNOS
	&exynos_device_ion,
#endif
};

#if defined(CONFIG_S5P_MEM_CMA)
static void __init exynos_cma_region_reserve(
			struct cma_region *regions_normal,
			struct cma_region *regions_secure)
{
	struct cma_region *reg;
	size_t size_secure = 0, align_secure = 0;
	phys_addr_t paddr = 0;

	for (reg = regions_normal; reg->size != 0; reg++) {
		if ((reg->alignment & (reg->alignment - 1)) || reg->reserved)
			continue;

		if (reg->start) {
			if (!memblock_is_region_reserved(reg->start, reg->size)
			    && memblock_reserve(reg->start, reg->size) >= 0)
				reg->reserved = 1;
		} else {
			paddr = __memblock_alloc_base(reg->size, reg->alignment,
					MEMBLOCK_ALLOC_ACCESSIBLE);
			if (paddr) {
				reg->start = paddr;
				reg->reserved = 1;
			}
		}
	}

	if (regions_secure && regions_secure->size) {
		for (reg = regions_secure; reg->size != 0; reg++)
			size_secure += reg->size;

		reg--;

		align_secure = reg->alignment;
		BUG_ON(align_secure & (align_secure - 1));

		paddr -= size_secure;
		paddr &= ~(align_secure - 1);

		if (!memblock_reserve(paddr, size_secure)) {
			do {
				reg->start = paddr;
				reg->reserved = 1;
				paddr += reg->size;
			} while (reg-- != regions_secure);
		}
	}
}

static void __init exynos_reserve_mem(void)
{
	static struct cma_region regions[] = {
#ifdef CONFIG_ANDROID_PMEM_MEMSIZE_PMEM
		{
			.name = "pmem",
			.size = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_GPU1
		{
			.name = "pmem_gpu1",
			.size = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_GPU1 * SZ_1K,
			.start = 0,
		},
#endif
		{
			.name = "ion",
			.size = 30 * SZ_1M,
			.start = 0
		},
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_GSC0
		{
			.name = "gsc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_GSC0 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_GSC1
		{
			.name = "gsc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_GSC1 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_GSC2
		{
			.name = "gsc2",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_GSC2 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_GSC3
		{
			.name = "gsc3",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_GSC3 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_S5P_MFC
		{
			.name		= "fw",
			.size		= 1 << 20,
			{ .alignment	= 128 << 10 },
			.start		= 0x44000000,
		},
		{
			.name		= "b1",
			.size		= 32 << 20,
			.start		= 0x45000000,
		},
#endif
		{
			.size = 0
		},
	};
	static const char map[] __initconst =
		"android_pmem.0=pmem;android_pmem.1=pmem_gpu1;"
		"s3cfb.0=fimd;"
		"exynos-gsc.0=gsc0;exynos-gsc.1=gsc1;exynos-gsc.2=gsc2;exynos-gsc.3=gsc3;"
		"ion-exynos=ion,gsc0,gsc1,gsc2,gsc3;"
		"s5p-mfc-v6/f=fw;"
		"s5p-mfc-v6/a=b1;";

	cma_set_defaults(regions, map);

	exynos_cma_region_reserve(regions, NULL);
}
#else /* !CONFIG_S5P_MEM_CMA */
static inline void exynos_reserve_mem(void)
{
}
#endif

static void __init smdk5250_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdk5250_uartcfgs, ARRAY_SIZE(smdk5250_uartcfgs));
	exynos_reserve_mem();
}

static void __init smdk5250_machine_init(void)
{
#ifdef CONFIG_S3C_DEV_HSMMC
	s3c_sdhci0_set_platdata(&smdk5250_hsmmc0_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	s3c_sdhci1_set_platdata(&smdk5250_hsmmc1_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s3c_sdhci2_set_platdata(&smdk5250_hsmmc2_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s3c_sdhci3_set_platdata(&smdk5250_hsmmc3_pdata);
#endif
#ifdef CONFIG_ION_EXYNOS
	exynos_ion_set_platdata();
#endif

#ifdef CONFIG_FB_S3C
	dev_set_name(&s5p_device_fimd1.dev, "s3cfb.1");
	clk_add_alias("lcd", "exynos5-fb.1", "lcd", &s5p_device_fimd1.dev);
	clk_add_alias("sclk_fimd", "exynos5-fb.1", "sclk_fimd",
			&s5p_device_fimd1.dev);
	s5p_fb_setname(1, "exynos5-fb");

	s5p_fimd1_set_platdata(&smdk5250_lcd1_pdata);
#endif

	platform_add_devices(smdk5250_devices, ARRAY_SIZE(smdk5250_devices));

#ifdef CONFIG_FB_S3C
	exynos4_fimd_setup_clock(&s5p_device_fimd1.dev, "sclk_fimd", "mout_mpll_user",
				800 * MHZ);
#endif
}

MACHINE_START(SMDK5250, "SMDK5250")
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos5_init_irq,
	.map_io		= smdk5250_map_io,
	.init_machine	= smdk5250_machine_init,
	.timer		= &exynos4_timer,
MACHINE_END
