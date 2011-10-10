/* linux/arch/arm/mach-exynos/mach-smdk5210.c
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
#include <linux/fb.h>
#include <linux/lcd.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/pwm_backlight.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/cma.h>
#include <linux/memblock.h>
#include <linux/mmc/host.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <plat/gpio-cfg.h>
#include <plat/regs-serial.h>
#include <plat/regs-fb-v4.h>
#include <plat/exynos5.h>
#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/fb.h>
#include <plat/fb-s5p.h>
#include <plat/fb-core.h>
#include <plat/regs-fb-v4.h>
#include <plat/backlight.h>
#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
#include <plat/s5p-mfc.h>
#endif
#include <plat/sdhci.h>

#include <mach/map.h>
#include <mach/exynos-ion.h>

#include <video/platform_lcd.h>

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDK5210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDK5210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDK5210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdk5210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDK5210_UCON_DEFAULT,
		.ulcon		= SMDK5210_ULCON_DEFAULT,
		.ufcon		= SMDK5210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDK5210_UCON_DEFAULT,
		.ulcon		= SMDK5210_ULCON_DEFAULT,
		.ufcon		= SMDK5210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDK5210_UCON_DEFAULT,
		.ulcon		= SMDK5210_ULCON_DEFAULT,
		.ufcon		= SMDK5210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDK5210_UCON_DEFAULT,
		.ulcon		= SMDK5210_ULCON_DEFAULT,
		.ufcon		= SMDK5210_UFCON_DEFAULT,
	},
};

#ifdef CONFIG_FB_S3C
#if defined(CONFIG_LCD_LMS501KF03)
static int lcd_power_on(struct lcd_device *ld, int enable)
{
	return 1;
}

static int reset_lcd(struct lcd_device *ld)
{
	int err = 0;

	err = gpio_request_one(EXYNOS5_GPX0(6), GPIOF_OUT_INIT_HIGH, "GPX0");
	if (err) {
		printk(KERN_ERR "failed to request GPX0 for "
				"lcd reset control\n");
		return err;
	}
	gpio_set_value(EXYNOS5_GPX0(6), 0);
	mdelay(1);

	gpio_set_value(EXYNOS5_GPX0(6), 1);

	gpio_free(EXYNOS5_GPX0(6));

	return 1;
}

static struct lcd_platform_data lms501kf03_platform_data = {
	.reset			= reset_lcd,
	.power_on		= lcd_power_on,
	.lcd_enabled		= 0,
	.reset_delay		= 100,	/* 100ms */
};

#define	LCD_BUS_NUM	3
#define	DISPLAY_CS	EXYNOS5_GPA2(5)		/*Chip select */
#define	DISPLAY_CLK	EXYNOS5_GPA2(4)		/* SPI clock */
#define	DISPLAY_SI	EXYNOS5_GPA2(7)		/* SPI MOSI */

static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias		= "lms501kf03",
		.platform_data		= (void *)&lms501kf03_platform_data,
		.max_speed_hz		= 1200000,
		.bus_num		= LCD_BUS_NUM,
		.chip_select		= 0,
		.mode			= SPI_MODE_3,
		.controller_data	= (void *)DISPLAY_CS,
	}
};

static struct spi_gpio_platform_data lms501kf03_spi_gpio_data = {
	.sck	= DISPLAY_CLK,
	.mosi	= DISPLAY_SI,
	.miso	= -1,
	.num_chipselect = 1,
};

static struct platform_device s3c_device_spi_gpio = {
	.name	= "spi_gpio",
	.id	= LCD_BUS_NUM,
	.dev	= {
		.parent		= &s5p_device_fimd1.dev,
		.platform_data	= &lms501kf03_spi_gpio_data,
	},
};

static struct s3c_fb_pd_win smdk5210_fb_win0 = {
	.win_mode = {
		.left_margin	= 8,	/* HBPD */
		.right_margin	= 8,	/* HFPD */
		.upper_margin	= 6,	/* VBPD */
		.lower_margin	= 6,	/* VFPD */
		.hsync_len	= 6,	/* HSPW */
		.vsync_len	= 4,	/* VSPW */
		.xres		= 480,
		.yres		= 800,
	},
	.virtual_x		= 480,
	.virtual_y		= 1600,
	.width			= 48,
	.height			= 80,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk5210_fb_win1 = {
	.win_mode = {
		.left_margin	= 8,	/* HBPD */
		.right_margin	= 8,	/* HFPD */
		.upper_margin	= 6,	/* VBPD */
		.lower_margin	= 6,	/* VFPD */
		.hsync_len	= 6,	/* HSPW */
		.vsync_len	= 4,	/* VSPW */
		.xres		= 480,
		.yres		= 800,
	},
	.virtual_x		= 480,
	.virtual_y		= 1600,
	.width			= 48,
	.height			= 80,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

static struct s3c_fb_pd_win smdk5210_fb_win2 = {
	.win_mode = {
		.left_margin	= 8,	/* HBPD */
		.right_margin	= 8,	/* HFPD */
		.upper_margin	= 6,	/* VBPD */
		.lower_margin	= 6,	/* VFPD */
		.hsync_len	= 6,	/* HSPW */
		.vsync_len	= 4,	/* VSPW */
		.xres		= 480,
		.yres		= 800,
	},
	.virtual_x		= 480,
	.virtual_y		= 1600,
	.width			= 48,
	.height			= 80,
	.max_bpp		= 32,
	.default_bpp		= 24,
};

#elif defined(CONFIG_LCD_WA101S)
static void lcd_wa101s_set_power(struct plat_lcd_data *pd,
				   unsigned int power)
{
	if (power) {
#ifndef CONFIG_BACKLIGHT_PWM
		gpio_request_one(EXYNOS5_GPB2(0), GPIOF_OUT_INIT_HIGH, "GPB2");
		gpio_free(EXYNOS5_GPB2(0));
#endif
		/* fire nRESET on power up */
		gpio_request_one(EXYNOS5_GPX0(6), GPIOF_OUT_INIT_HIGH, "GPX0");
		mdelay(100);

		gpio_set_value(EXYNOS5_GPX0(6), 0);
		mdelay(10);

		gpio_set_value(EXYNOS5_GPX0(6), 1);
		mdelay(10);

		gpio_free(EXYNOS5_GPX0(6));
	} else {
#ifndef CONFIG_BACKLIGHT_PWM
		gpio_request_one(EXYNOS5_GPB2(0), GPIOF_OUT_INIT_LOW, "GPB2");
		gpio_free(EXYNOS5_GPB2(0));
#endif
	}
}

static struct plat_lcd_data smdk5210_lcd_wa101s_data = {
	.set_power		= lcd_wa101s_set_power,
};

static struct platform_device smdk5210_lcd_wa101s = {
	.name			= "platform-lcd",
	.dev.parent		= &s5p_device_fimd1.dev,
	.dev.platform_data      = &smdk5210_lcd_wa101s_data,
};

static struct s3c_fb_pd_win smdk5210_fb_win0 = {
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

static struct s3c_fb_pd_win smdk5210_fb_win1 = {
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

static struct s3c_fb_pd_win smdk5210_fb_win2 = {
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
	.width			= 1360,
	.height			= 768,
	.max_bpp		= 32,
	.default_bpp		= 24,
};
#endif

static void exynos_fimd_gpio_setup_24bpp(void)
{
	unsigned int reg = 0;

#if defined(CONFIG_LCD_WA101S)
	exynos4_fimd_cfg_gpios(EXYNOS5_GPJ0(0), 5, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
	exynos4_fimd_cfg_gpios(EXYNOS5_GPJ1(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
	exynos4_fimd_cfg_gpios(EXYNOS5_GPJ2(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
	exynos4_fimd_cfg_gpios(EXYNOS5_GPJ3(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
	exynos4_fimd_cfg_gpios(EXYNOS5_GPJ4(0), 2, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
#elif defined(CONFIG_LCD_LMS501KF03)
	exynos4_fimd_cfg_gpios(EXYNOS5_GPJ0(0), 5, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
	exynos4_fimd_cfg_gpios(EXYNOS5_GPJ1(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV1);
	exynos4_fimd_cfg_gpios(EXYNOS5_GPJ2(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV1);
	exynos4_fimd_cfg_gpios(EXYNOS5_GPJ3(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV1);
	exynos4_fimd_cfg_gpios(EXYNOS5_GPJ4(0), 2, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV1);
#endif

	/*
	 * Set DISP1BLK_CFG register for Display path selection
	 * DISP1_BLK output source selection : DISP1BLK_CFG[28:27]
	 *---------------------
	 *  10 | From DISP0_BLK
	 *  01 | From DISP1_BLK : selected
	 *
	 * FIMD of DISP1_BLK Bypass selection : DISP1BLK_CFG[15]
	 * ---------------------
	 *  0 | MIE/MDNIE
	 *  1 | FIMD : selected
	 */
	  reg = __raw_readl(S3C_VA_SYS + 0x0214);
	reg &= ~((1 << 28) | (1 << 27) | (1 << 15));	/* To save other reset values */
	reg |=  (0 << 28) | (1 << 27) | (1 << 15);
	__raw_writel(reg, S3C_VA_SYS + 0x0214);
}

static struct s3c_fb_platdata smdk5210_lcd1_pdata __initdata = {
#if defined(CONFIG_LCD_WA101S) || defined(CONFIG_LCD_LMS501KF03)
	.win[0]		= &smdk5210_fb_win0,
	.win[1]		= &smdk5210_fb_win1,
	.win[2]		= &smdk5210_fb_win2,
#endif
	.default_win	= 2,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
#if defined(CONFIG_LCD_LMS501KF03)
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
#elif defined(CONFIG_LCD_WA101S)
	.vidcon1	= VIDCON1_INV_VCLK | VIDCON1_INV_HSYNC |
			  VIDCON1_INV_VSYNC,
#endif
	.setup_gpio	= exynos_fimd_gpio_setup_24bpp,
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC
static struct s3c_sdhci_platdata smdk5210_hsmmc0_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH0_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC1
static struct s3c_sdhci_platdata smdk5210_hsmmc1_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC2
static struct s3c_sdhci_platdata smdk5210_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH2_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC3
static struct s3c_sdhci_platdata smdk5210_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};
#endif

static struct platform_device *smdk5210_devices[] __initdata = {
#ifdef CONFIG_FB_S3C
	&s5p_device_fimd1,
#if defined(CONFIG_LCD_LMS501KF03)
	&s3c_device_spi_gpio,
#elif defined(CONFIG_LCD_WA101S)
	&smdk5210_lcd_wa101s,
#endif
#endif
#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
	&s5p_device_mfc,
#endif
#ifdef CONFIG_ION_EXYNOS
	&exynos_device_ion,
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
};

#ifdef SAMSUNG_DEV_BACKLIGHT
/* LCD Backlight data */
static struct samsung_bl_gpio_info smdk5210_bl_gpio_info = {
	.no = EXYNOS5_GPB2(0),
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data smdk5210_bl_data = {
	.pwm_id = 1,
};
#endif

#if defined(CONFIG_S5P_MEM_CMA)
static void __init exynos5_cma_region_reserve(
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

static void __init exynos5_reserve_mem(void)
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
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD
		{
			.name = "fimd",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0
		{
			.name = "fimc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2
		{
			.name = "fimc2",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3
		{
			.name = "fimc3",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3 * SZ_1K,
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
	static struct cma_region regions_secure[] = {
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1
		{
			.name = "fimc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1 * SZ_1K,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0
		{
			.name = "mfc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0 * SZ_1K,
			{
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
				.alignment = SZ_64M,
#else
				.alignment = 1 << 17,
#endif
			},
		},
#endif
		{
			.size = 0
		},
	};
	static const char map[] __initconst =
		"android_pmem.0=pmem;android_pmem.1=pmem_gpu1;"
		"s3c-fimc.0=fimc0;s3c-fimc.1=fimc1;s3c-fimc.2=fimc2;s3c-fimc.3=fimc3;"
		"s3cfb.0=fimd;"
		"ion-exynos=fimd,fimc0,fimc2,fimc3;"
		"s5p-mfc-v6/f=fw;"
		"s5p-mfc-v6/a=b1;";
	struct cma_region *reg;

	cma_set_defaults(regions, map);

	reg = regions_secure;
	for (; reg->size; ++reg)
		BUG_ON(cma_early_region_register(reg));

	exynos5_cma_region_reserve(regions, regions_secure);
}
#else /* !CONFIG_S5P_MEM_CMA */
static inline void exynos5_reserve_mem() { }
#endif

static void __init smdk5210_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdk5210_uartcfgs, ARRAY_SIZE(smdk5210_uartcfgs));
	exynos5_reserve_mem();
}

static void __init smdk5210_machine_init(void)
{
#ifdef CONFIG_FB_S3C
	dev_set_name(&s5p_device_fimd1.dev, "s3cfb.1");
	clk_add_alias("lcd", "exynos5-fb.1", "lcd", &s5p_device_fimd1.dev);
	clk_add_alias("sclk_fimd", "exynos5-fb.1", "sclk_fimd",
			&s5p_device_fimd1.dev);
	s5p_fb_setname(1, "exynos5-fb");

#if defined(CONFIG_LCD_LMS501KF03)
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif
	s5p_fimd1_set_platdata(&smdk5210_lcd1_pdata);
#endif

#ifdef SAMSUNG_DEV_BACKLIGHT
	samsung_bl_set(&smdk5210_bl_gpio_info, &smdk5210_bl_data);
#endif

#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
	dev_set_name(&s5p_device_mfc.dev, "s3c-mfc");
	clk_add_alias("mfc", "s5p-mfc-v6", "mfc", &s5p_device_mfc.dev);
	s5p_mfc_setname(&s5p_device_mfc, "s5p-mfc-v6");
#endif
#ifdef CONFIG_ION_EXYNOS
	exynos_ion_set_platdata();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	s3c_sdhci0_set_platdata(&smdk5210_hsmmc0_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	s3c_sdhci1_set_platdata(&smdk5210_hsmmc1_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s3c_sdhci2_set_platdata(&smdk5210_hsmmc2_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s3c_sdhci3_set_platdata(&smdk5210_hsmmc3_pdata);
#endif

	platform_add_devices(smdk5210_devices, ARRAY_SIZE(smdk5210_devices));

#ifdef CONFIG_FB_S3C
	exynos4_fimd_setup_clock(&s5p_device_fimd1.dev, "sclk_fimd", "mout_mpll_user",
				800 * MHZ);
#endif
}

MACHINE_START(SMDK5210, "SMDK5210")
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos5_init_irq,
	.map_io		= smdk5210_map_io,
	.init_machine	= smdk5210_machine_init,
	.timer		= &exynos4_timer,
MACHINE_END
