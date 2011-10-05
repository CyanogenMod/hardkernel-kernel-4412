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
#include <linux/pwm_backlight.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

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

#include <mach/map.h>

#include <video/platform_lcd.h>

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDKV310_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDKV310_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDKV310_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdk5210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDKV310_UCON_DEFAULT,
		.ulcon		= SMDKV310_ULCON_DEFAULT,
		.ufcon		= SMDKV310_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDKV310_UCON_DEFAULT,
		.ulcon		= SMDKV310_ULCON_DEFAULT,
		.ufcon		= SMDKV310_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDKV310_UCON_DEFAULT,
		.ulcon		= SMDKV310_ULCON_DEFAULT,
		.ufcon		= SMDKV310_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDKV310_UCON_DEFAULT,
		.ulcon		= SMDKV310_ULCON_DEFAULT,
		.ufcon		= SMDKV310_UFCON_DEFAULT,
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

	err = gpio_request_one(EXYNOS4_GPX1(5), GPIOF_OUT_INIT_HIGH, "GPX1");
	if (err) {
		printk(KERN_ERR "failed to request GPX1 for "
				"lcd reset control\n");
		return err;
	}
	gpio_set_value(EXYNOS4_GPX1(5), 0);
	mdelay(1);

	gpio_set_value(EXYNOS4_GPX1(5), 1);

	gpio_free(EXYNOS4_GPX1(5));

	return 1;
}

static struct lcd_platform_data lms501kf03_platform_data = {
	.reset			= reset_lcd,
	.power_on		= lcd_power_on,
	.lcd_enabled		= 0,
	.reset_delay		= 100,	/* 100ms */
};

#define		LCD_BUS_NUM	3
#define		DISPLAY_CS	EXYNOS4_GPB(5)
#define		DISPLAY_CLK	EXYNOS4_GPB(4)
#define		DISPLAY_SI	EXYNOS4_GPB(7)

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
		.left_margin	= 8,		/* HBPD */
		.right_margin	= 8,		/* HFPD */
		.upper_margin	= 6,	/* VBPD */
		.lower_margin	= 6,		/* VFPD */
		.hsync_len	= 6,		/* HSPW */
		.vsync_len	= 4,		/* VSPW */
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
		.left_margin	= 8,		/* HBPD */
		.right_margin	= 8,		/* HFPD */
		.upper_margin	= 6,	/* VBPD */
		.lower_margin	= 6,		/* VFPD */
		.hsync_len	= 6,		/* HSPW */
		.vsync_len	= 4,		/* VSPW */
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
		.left_margin	= 8,		/* HBPD */
		.right_margin	= 8,		/* HFPD */
		.upper_margin	= 6,	/* VBPD */
		.lower_margin	= 6,		/* VFPD */
		.hsync_len	= 6,		/* HSPW */
		.vsync_len	= 4,		/* VSPW */
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
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(EXYNOS4_GPD0(1), GPIOF_OUT_INIT_HIGH, "GPD0");
		gpio_free(EXYNOS4_GPD0(1));
#endif
	} else {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request_one(EXYNOS4_GPD0(1), GPIOF_OUT_INIT_LOW, "GPD0");
		gpio_free(EXYNOS4_GPD0(1));
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
	.width			= 223,
	.height			= 125,
	.max_bpp		= 32,
	.default_bpp		= 24,
};
#endif
static struct s3c_fb_platdata smdk5210_lcd0_pdata __initdata = {
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
	.setup_gpio	= exynos4_fimd0_gpio_setup_24bpp,
};
#endif
static struct platform_device *smdk5210_devices[] __initdata = {

	/* Samsung Power Domain */
	//&exynos5_device_pd[PD_LCD1],
#ifdef CONFIG_FB_S3C
	&s5p_device_fimd1,
#if  defined(CONFIG_LCD_LMS501KF03)
	&s3c_device_spi_gpio,
#elif defined(CONFIG_LCD_WA101S)
	&smdk5210_lcd_wa101s,
#endif
#endif

};

static void __init smdk5210_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdk5210_uartcfgs, ARRAY_SIZE(smdk5210_uartcfgs));
}

static void __init smdk5210_machine_init(void)
{
	//exynos5_pd_enable(&exynos5_device_pd[PD_LCD0].dev);

#ifdef CONFIG_FB_S3C
	dev_set_name(&s5p_device_fimd1.dev, "s3cfb.1");
	clk_add_alias("lcd", "exynos5-fb.1", "lcd", &s5p_device_fimd1.dev);
	clk_add_alias("sclk_fimd", "exynos5-fb.1", "sclk_fimd", &s5p_device_fimd1.dev);
	s5p_fb_setname(0, "exynos5-fb");
#if  defined(CONFIG_LCD_LMS501KF03)
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif
	s5p_fimd1_set_platdata(&smdk5210_lcd0_pdata);
#ifdef CONFIG_EXYNOS5_DEV_PD
	s5p_device_fimd1.dev.parent = &exynos5_device_pd[PD_LCD0].dev;
#endif
#endif
	platform_add_devices(smdk5210_devices, ARRAY_SIZE(smdk5210_devices));
}

MACHINE_START(SMDK5210, "SMDK5210")
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos5_init_irq,
	.map_io		= smdk5210_map_io,
	.init_machine	= smdk5210_machine_init,
	.timer		= &exynos4_timer,
MACHINE_END
