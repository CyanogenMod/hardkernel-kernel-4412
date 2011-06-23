/* linux/arch/arm/mach-exynos4/mach-smdkc210.c
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/serial_core.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/lcd.h>
#include <linux/pwm_backlight.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/smsc911x.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/max8649.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/wm8994/pdata.h>
#if defined(CONFIG_S5P_MEM_CMA)
#include <linux/cma.h>
#endif
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <video/platform_lcd.h>

#include <plat/regs-serial.h>
#include <plat/regs-srom.h>
#include <plat/exynos4.h>
#include <plat/clock.h>
#include <plat/hwmon.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/fb.h>
#include <plat/fb-s5p.h>
#include <plat/fimc.h>
#include <plat/csis.h>
#include <plat/gpio-cfg.h>
#include <plat/adc.h>
#include <plat/ts.h>
#include <plat/keypad.h>
#include <plat/sdhci.h>
#include <plat/iic.h>
#include <plat/pd.h>
#include <plat/media.h>
#include <plat/s5p-clock.h>
#include <plat/tvout.h>
#include <plat/fimg2d.h>
#include <plat/ehci.h>

#include <mach/map.h>
#include <mach/media.h>
#include <mach/regs-fb.h>
#include <mach/sysmmu.h>

#include <media/s5k4ba_platform.h>
#include <media/s5k4ea_platform.h>
#include <media/m5mo_platform.h>

#if defined(CONFIG_EXYNOS4_SETUP_THERMAL)
#include <plat/s5p-tmu.h>
#endif

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDKC210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDKC210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDKC210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdkc210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDKC210_UCON_DEFAULT,
		.ulcon		= SMDKC210_ULCON_DEFAULT,
		.ufcon		= SMDKC210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDKC210_UCON_DEFAULT,
		.ulcon		= SMDKC210_ULCON_DEFAULT,
		.ufcon		= SMDKC210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDKC210_UCON_DEFAULT,
		.ulcon		= SMDKC210_ULCON_DEFAULT,
		.ufcon		= SMDKC210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDKC210_UCON_DEFAULT,
		.ulcon		= SMDKC210_ULCON_DEFAULT,
		.ufcon		= SMDKC210_UFCON_DEFAULT,
	},
};

#undef WRITEBACK_ENABLED

#ifdef CONFIG_VIDEO_FIMC
/*
 * External camera reset
 * Because the most of cameras take i2c bus signal, so that
 * you have to reset at the boot time for other i2c slave devices.
 * This function also called at fimc_init_camera()
 * Do optimization for cameras on your platform.
*/
#ifdef CONFIG_ITU_A
static int smdkc210_cam0_reset(int dummy)
{
	int err;
	/* Camera A */
	err = gpio_request(EXYNOS4_GPX1(2), "GPX1");
	if (err)
		printk(KERN_ERR "#### failed to request GPX1_2 ####\n");

	s3c_gpio_setpull(EXYNOS4_GPX1(2), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS4_GPX1(2), 0);
	gpio_direction_output(EXYNOS4_GPX1(2), 1);
	gpio_free(EXYNOS4_GPX1(2));

	return 0;
}
#endif
#ifdef CONFIG_ITU_B
static int smdkc210_cam1_reset(int dummy)
{
	int err;

	/* Camera B */
	err = gpio_request(EXYNOS4_GPX1(0), "GPX1");
	if (err)
		printk(KERN_ERR "#### failed to request GPX1_0 ####\n");

	s3c_gpio_setpull(EXYNOS4_GPX1(0), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS4_GPX1(0), 0);
	gpio_direction_output(EXYNOS4_GPX1(0), 1);
	gpio_free(EXYNOS4_GPX1(0));

	return 0;
}
#endif
/* for 12M camera */
#ifdef CE143_MONACO
static int smdkc210_cam0_standby(void)
{
	int err;
	/* Camera A */
	err = gpio_request(EXYNOS4_GPX3(3), "GPX3");
	if (err)
		printk(KERN_ERR "#### failed to request GPX3_3 ####\n");
	s3c_gpio_setpull(EXYNOS4_GPX3(3), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS4_GPX3(3), 0);
	gpio_direction_output(EXYNOS4_GPX3(3), 1);
	gpio_free(EXYNOS4_GPX3(3));

	return 0;
}

static int smdkc210_cam1_standby(void)
{
	int err;

	/* Camera B */
	err = gpio_request(EXYNOS4_GPX1(1), "GPX1");
	if (err)
		printk(KERN_ERR "#### failed to request GPX1_1 ####\n");
	s3c_gpio_setpull(EXYNOS4_GPX1(1), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS4_GPX1(1), 0);
	gpio_direction_output(EXYNOS4_GPX1(1), 1);
	gpio_free(EXYNOS4_GPX1(1));

	return 0;
}
#endif

/* Set for MIPI-CSI Camera module Reset */
#ifdef CONFIG_CSI_C
static int smdkc210_mipi_cam0_reset(int dummy)
{
	int err;

	err = gpio_request(EXYNOS4_GPX1(2), "GPX1");
	if (err)
		printk(KERN_ERR "#### failed to reset(GPX1_2) MIPI CAM\n");

	s3c_gpio_setpull(EXYNOS4_GPX1(2), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS4_GPX1(2), 0);
	gpio_direction_output(EXYNOS4_GPX1(2), 1);
	gpio_free(EXYNOS4_GPX1(2));

	return 0;
}
#endif
#ifdef CONFIG_CSI_D
static int smdkc210_mipi_cam1_reset(int dummy)
{
	int err;

	err = gpio_request(EXYNOS4_GPX1(0), "GPX1");
	if (err)
		printk(KERN_ERR "#### failed to reset(GPX1_0) MIPI CAM\n");

	s3c_gpio_setpull(EXYNOS4_GPX1(0), S3C_GPIO_PULL_NONE);
	gpio_direction_output(EXYNOS4_GPX1(0), 0);
	gpio_direction_output(EXYNOS4_GPX1(0), 1);
	gpio_free(EXYNOS4_GPX1(0));

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_S5K4BA
static struct s5k4ba_platform_data s5k4ba_plat = {
	.default_width = 800,
	.default_height = 600,
	.pixelformat = V4L2_PIX_FMT_YUYV,
	.freq = 24000000,
	.is_mipi = 0,
};

static struct i2c_board_info  s5k4ba_i2c_info = {
	I2C_BOARD_INFO("S5K4BA", 0x2d),
	.platform_data = &s5k4ba_plat,
};

static struct s3c_platform_camera s5k4ba = {
#ifdef CONFIG_ITU_A
	.id		= CAMERA_PAR_A,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 0,
	.cam_power	= smdkc210_cam0_reset,
#endif
#ifdef CONFIG_ITU_B
	.id		= CAMERA_PAR_B,
	.clk_name	= "sclk_cam1",
	.i2c_busnum	= 1,
	.cam_power	= smdkc210_cam1_reset,
#endif
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &s5k4ba_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUYV,
	.srclk_name	= "xusbxti",
	.clk_rate	= 24000000,
	.line_length	= 1920,
	.width		= 1600,
	.height		= 1200,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1600,
		.height	= 1200,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 1,
	.initialized	= 0,
};
#endif

/* 2 MIPI Cameras */
#ifdef CONFIG_VIDEO_S5K4EA
static struct s5k4ea_platform_data s5k4ea_plat = {
	.default_width = 1920,
	.default_height = 1080,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
};

static struct i2c_board_info  s5k4ea_i2c_info = {
	I2C_BOARD_INFO("S5K4EA", 0x2d),
	.platform_data = &s5k4ea_plat,
};

static struct s3c_platform_camera s5k4ea = {
#ifdef CONFIG_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 0,
	.cam_power	= smdkc210_mipi_cam0_reset,
#endif
#ifdef CONFIG_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.i2c_busnum	= 1,
	.cam_power	= smdkc210_mipi_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &s5k4ea_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "mout_mpll",
	.clk_rate	= 48000000,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},

	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 32,

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,

	.initialized	= 0,
};
#endif

#ifdef WRITEBACK_ENABLED
static struct i2c_board_info  writeback_i2c_info = {
	I2C_BOARD_INFO("WriteBack", 0x0),
};

static struct s3c_platform_camera writeback = {
	.id		= CAMERA_WB,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum	= 0,
	.info		= &writeback_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUV444,
	.line_length	= 800,
	.width		= 480,
	.height		= 800,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 480,
		.height	= 800,
	},

	.initialized	= 0,
};
#endif

/* M5MOLS Camera configuration for legacy fimc driver */
#ifdef CONFIG_VIDEO_M5MO
#define CAM_CHECK_ERR_RET(x, msg)	\
	if (unlikely((x) < 0)) { \
		printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x); \
		return x; \
	}
#define CAM_CHECK_ERR(x, msg)	\
		if (unlikely((x) < 0)) { \
			printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x); \
		}

static int m5mo_config_isp_irq(void)
{
	s3c_gpio_cfgpin(EXYNOS4_GPX0(5), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX0(5), S3C_GPIO_PULL_NONE);
	return 0;
}

static struct m5mo_platform_data m5mo_plat = {
	.default_width = 640, /* 1920 */
	.default_height = 480, /* 1080 */
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
	.config_isp_irq = m5mo_config_isp_irq,
	.irq = IRQ_EINT(5),
};

static struct i2c_board_info  m5mo_i2c_info = {
	I2C_BOARD_INFO("M5MO", 0x1F),
	.platform_data = &m5mo_plat,
	.irq = IRQ_EINT(5),
};

static struct s3c_platform_camera m5mo = {
#ifdef CONFIG_CSI_C
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 0,
	.cam_power	= smdkc210_mipi_cam0_reset,
#endif
#ifdef CONFIG_CSI_D
	.id		= CAMERA_CSI_D,
	.clk_name	= "sclk_cam1",
	.i2c_busnum	= 1,
	.cam_power	= smdkc210_mipi_cam1_reset,
#endif
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.info		= &m5mo_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti", /* "mout_mpll" */
	.clk_rate	= 24000000, /* 48000000 */
	.line_length	= 1920,
	.width		= 640,
	.height		= 480,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 640,
		.height	= 480,
	},

	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 32,

	/* Polarity */
	.inv_pclk	= 1,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 0,
	.initialized	= 0,
};
#endif /* #ifdef CONFIG_VIDEO_M5MO */

/* Interface setting */
static struct s3c_platform_fimc fimc_plat = {
#ifdef CONFIG_ITU_A
	.default_cam	= CAMERA_PAR_A,
#endif
#ifdef CONFIG_ITU_B
	.default_cam	= CAMERA_PAR_B,
#endif
#ifdef CONFIG_CSI_C
	.default_cam	= CAMERA_CSI_C,
#endif
#ifdef CONFIG_CSI_D
	.default_cam	= CAMERA_CSI_D,
#endif
#ifdef WRITEBACK_ENABLED
	.default_cam	= CAMERA_WB,
#endif
	.camera		= {
#ifdef CONFIG_VIDEO_S5K4BA
		&s5k4ba,
#endif
#ifdef CONFIG_VIDEO_S5K4EA
		&s5k4ea,
#endif
#ifdef CONFIG_VIDEO_M5MO
		&m5mo,
#endif
#ifdef WRITEBACK_ENABLED
		&writeback,
#endif
	},
	.hw_ver		= 0x51,
};
#endif /* CONFIG_VIDEO_FIMC */

#ifdef CONFIG_S3C_DEV_HSMMC
static struct s3c_sdhci_platdata smdkc210_hsmmc0_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH0_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC1
static struct s3c_sdhci_platdata smdkc210_hsmmc1_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC2
static struct s3c_sdhci_platdata smdkc210_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH2_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC3
static struct s3c_sdhci_platdata smdkc210_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};
#endif

#ifdef CONFIG_VIDEO_FIMG2D
static struct fimg2d_platdata fimg2d_data __initdata = {
	.hw_ver = 30,
	.parent_clkname = "mout_mpll",
	.clkname = "sclk_fimg2d",
	.gate_clkname = "fimg2d",
	.clkrate = 267 * 1000000,	/* 266 Mhz */
};
#endif

#ifdef CONFIG_SAMSUNG_DEV_PWM
#if defined(CONFIG_BACKLIGHT_PWM)
static int smdkc210_backlight_init(struct device *dev)
{
	int ret;

	ret = gpio_request(EXYNOS4_GPD0(1), "Backlight");
	if (ret) {
		printk(KERN_ERR "failed to request GPD for PWM-OUT 1\n");
		return ret;
	}

	/* Configure GPIO pin with S5PC210_GPD_0_1_TOUT_1 */
	s3c_gpio_cfgpin(EXYNOS4_GPD0(1), S3C_GPIO_SFN(2));

	return 0;
}

static void smdkc210_backlight_exit(struct device *dev)
{
	s3c_gpio_cfgpin(EXYNOS4_GPD0(1), S3C_GPIO_OUTPUT);
	gpio_free(EXYNOS4_GPD0(1));
}

static struct platform_pwm_backlight_data smdkc210_backlight_data = {
	.pwm_id		= 1,
	.max_brightness	= 255,
	.dft_brightness	= 255,
	.pwm_period_ns	= 1000,
	.init		= smdkc210_backlight_init,
	.exit		= smdkc210_backlight_exit,
};

static struct platform_device smdkc210_backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.parent		= &s3c_device_timer[1].dev,
		.platform_data	= &smdkc210_backlight_data,
	},
};
#endif
#endif

#ifdef CONFIG_FB_S3C
#if defined(CONFIG_LCD_AMS369FG06)
static int lcd_power_on(struct lcd_device *ld, int enable)
{
	return 1;
}

static int reset_lcd(struct lcd_device *ld)
{
	int err = 0;

	err = gpio_request(EXYNOS4_GPX0(6), "GPX0");
	if (err) {
		printk(KERN_ERR "failed to request GPX0 for "
				"lcd reset control\n");
		return err;
	}

	gpio_direction_output(EXYNOS4_GPX0(6), 1);
	mdelay(100);

	gpio_set_value(EXYNOS4_GPX0(6), 1);
	mdelay(100);

	gpio_free(EXYNOS4_GPX0(6));

	return 1;
}

static struct lcd_platform_data ams369fg06_platform_data = {
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
		.modalias		= "ams369fg06",
		.platform_data		= (void *)&ams369fg06_platform_data,
		.max_speed_hz		= 1200000,
		.bus_num		= LCD_BUS_NUM,
		.chip_select		= 0,
		.mode			= SPI_MODE_3,
		.controller_data	= (void *)DISPLAY_CS,
	}
};

static struct spi_gpio_platform_data ams369fg06_spi_gpio_data = {
	.sck	= DISPLAY_CLK,
	.mosi	= DISPLAY_SI,
	.miso	= -1,
	.num_chipselect = 1,
};

static struct platform_device s3c_device_spi_gpio = {
	.name	= "spi_gpio",
	.id	= LCD_BUS_NUM,
	.dev	= {
		.parent		= &s5p_device_fimd0.dev,
		.platform_data	= &ams369fg06_spi_gpio_data,
	},
};

static struct s3c_fb_pd_win smdkc210_fb_win0 = {
	.win_mode = {
		.left_margin  = 9,
		.right_margin = 9,
		.upper_margin = 5,
		.lower_margin = 5,
		.hsync_len = 2,
		.vsync_len = 2,
		.xres = 480,
		.yres = 800,
	},
	.virtual_x = 480,
	.virtual_y = 1600,
	.width = 48,
	.height = 80,
	.max_bpp = 32,
	.default_bpp = 24,
};

static struct s3c_fb_pd_win smdkc210_fb_win1 = {
	.win_mode = {
		.left_margin  = 9,
		.right_margin = 9,
		.upper_margin = 5,
		.lower_margin = 5,
		.hsync_len = 2,
		.vsync_len = 2,
		.xres = 480,
		.yres = 800,
	},
	.virtual_x = 480,
	.virtual_y = 1600,
	.width = 48,
	.height = 80,
	.max_bpp = 32,
	.default_bpp = 24,
};

#elif defined(CONFIG_LCD_WA101S)
static void lcd_wa101s_set_power(struct plat_lcd_data *pd,
				   unsigned int power)
{
	if (power) {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request(EXYNOS4_GPD0(1), "GPD0");
		gpio_direction_output(EXYNOS4_GPD0(1), 1);
		gpio_free(EXYNOS4_GPD0(1));
#endif
	} else {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request(EXYNOS4_GPD0(1), "GPD0");
		gpio_direction_output(EXYNOS4_GPD0(1), 0);
		gpio_free(EXYNOS4_GPD0(1));
#endif
	}
}

static struct plat_lcd_data smdkc210_lcd_wa101s_data = {
	.set_power      = lcd_wa101s_set_power,
};

static struct platform_device smdkc210_lcd_wa101s = {
	.name                   = "platform-lcd",
	.dev.parent             = &s5p_device_fimd0.dev,
	.dev.platform_data      = &smdkc210_lcd_wa101s_data,
};

static struct s3c_fb_pd_win smdkc210_fb_win0 = {
	.win_mode = {
		.left_margin    = 80,
		.right_margin   = 48,
		.upper_margin   = 14,
		.lower_margin   = 3,
		.hsync_len      = 32,
		.vsync_len      = 5,
		.xres   = 1366,
		.yres   = 768,
	},
	.virtual_x = 1366,
	.virtual_y = 768 * 2,
	.width = 223,
	.height = 125,
	.max_bpp        = 32,
	.default_bpp    = 24,
};

#ifndef CONFIG_LCD_WA101S /* temporarily disables window1 */
static struct s3c_fb_pd_win smdkc210_fb_win1 = {
	.win_mode = {
		.left_margin    = 80,
		.right_margin   = 48,
		.upper_margin   = 14,
		.lower_margin   = 3,
		.hsync_len      = 32,
		.vsync_len      = 5,
		.xres   = 1366,
		.yres   = 768,
	},
	.virtual_x = 1366,
	.virtual_y = 768 * 2,
	.width = 223,
	.height = 125,
	.max_bpp        = 32,
	.default_bpp    = 24,
};
#endif

#elif defined(CONFIG_LCD_LTE480WV)
static void lcd_lte480wv_set_power(struct plat_lcd_data *pd,
				   unsigned int power)
{
	if (power) {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request(EXYNOS4_GPD0(1), "GPD0");
		gpio_direction_output(EXYNOS4_GPD0(1), 1);
		gpio_free(EXYNOS4_GPD0(1));
#endif
		/* fire nRESET on power up */
		gpio_request(EXYNOS4_GPX0(6), "GPX0");

		gpio_direction_output(EXYNOS4_GPX0(6), 1);
		mdelay(100);

		gpio_set_value(EXYNOS4_GPX0(6), 0);
		mdelay(10);

		gpio_set_value(EXYNOS4_GPX0(6), 1);
		mdelay(10);

		gpio_free(EXYNOS4_GPX0(6));
	} else {
#if !defined(CONFIG_BACKLIGHT_PWM)
		gpio_request(EXYNOS4_GPD0(1), "GPD0");
		gpio_direction_output(EXYNOS4_GPD0(1), 0);
		gpio_free(EXYNOS4_GPD0(1));
#endif
	}
}

static struct plat_lcd_data smdkc210_lcd_lte480wv_data = {
	.set_power      = lcd_lte480wv_set_power,
};

static struct platform_device smdkc210_lcd_lte480wv = {
	.name                   = "platform-lcd",
	.dev.parent             = &s5p_device_fimd0.dev,
	.dev.platform_data      = &smdkc210_lcd_lte480wv_data,
};

static struct s3c_fb_pd_win smdkc210_fb_win0 = {
	.win_mode = {
		.left_margin    = 13,
		.right_margin   = 8,
		.upper_margin   = 7,
		.lower_margin   = 5,
		.hsync_len      = 3,
		.vsync_len      = 1,
		.xres   = 800,
		.yres   = 480,
	},
	.virtual_x = 800,
	.virtual_y = 960,
	.width = 104,
	.height = 62,
	.max_bpp        = 32,
	.default_bpp    = 24,
};

static struct s3c_fb_pd_win smdkc210_fb_win1 = {
	.win_mode = {
		.left_margin    = 13,
		.right_margin   = 8,
		.upper_margin   = 7,
		.lower_margin   = 5,
		.hsync_len      = 3,
		.vsync_len      = 1,
		.xres   = 800,
		.yres   = 480,
	},
	.virtual_x = 800,
	.virtual_y = 960,
	.width = 104,
	.height = 62,
	.max_bpp        = 32,
	.default_bpp    = 24,
};
#endif

static struct s3c_fb_platdata smdkc210_lcd0_pdata __initdata = {
#if defined(CONFIG_LCD_AMS369FG06) || defined(CONFIG_LCD_WA101S) || \
	defined(CONFIG_LCD_LTE480WV)
	.win[0]		= &smdkc210_fb_win0,
#ifndef CONFIG_LCD_WA101S /* temporarily disables window1 */
	.win[1]		= &smdkc210_fb_win1,
#endif
#endif
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
#if defined(CONFIG_LCD_AMS369FG06)
	.vidcon1	= VIDCON1_INV_VCLK | VIDCON1_INV_VDEN |
			  VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
#elif defined(CONFIG_LCD_WA101S)
	.vidcon1	= VIDCON1_INV_VCLK | VIDCON1_INV_HSYNC |
			  VIDCON1_INV_VSYNC,
#elif defined(CONFIG_LCD_LTE480WV)
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
#endif
	.setup_gpio	= exynos4_fimd0_gpio_setup_24bpp,
};
#endif

#ifdef CONFIG_FB_S5P
#ifdef CONFIG_FB_S5P_AMS369FG06
static struct s3c_platform_fb ams369fg06_data __initdata = {
	.hw_ver = 0x70,
	.clk_name = "sclk_lcd",
	.nr_wins = 5,
	.default_win = CONFIG_FB_S5P_DEFAULT_WINDOW,
	.swap = FB_SWAP_HWORD | FB_SWAP_WORD,
};

#define		LCD_BUS_NUM	3
#define		DISPLAY_CS	EXYNOS4_GPB(5)
#define		DISPLAY_CLK	EXYNOS4_GPB(4)
#define		DISPLAY_SI	EXYNOS4_GPB(7)

static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias	= "ams369fg06",
		.platform_data	= NULL,
		.max_speed_hz	= 1200000,
		.bus_num	= LCD_BUS_NUM,
		.chip_select	= 0,
		.mode		= SPI_MODE_3,
		.controller_data = (void *)DISPLAY_CS,
	}
};
static struct spi_gpio_platform_data ams369fg06_spi_gpio_data = {
	.sck	= DISPLAY_CLK,
	.mosi	= DISPLAY_SI,
	.miso	= -1,
	.num_chipselect = 1,
};

static struct platform_device s3c_device_spi_gpio = {
	.name	= "spi_gpio",
	.id	= LCD_BUS_NUM,
	.dev	= {
		.parent		= &s3c_device_fb.dev,
		.platform_data	= &ams369fg06_spi_gpio_data,
	},
};
#endif
#endif

static struct resource smdkc210_smsc911x_resources[] = {
	[0] = {
		.start	= EXYNOS4_PA_SROM_BANK(1),
		.end	= EXYNOS4_PA_SROM_BANK(1) + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_EINT(5),
		.end	= IRQ_EINT(5),
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
	},
};

static struct smsc911x_platform_config smsc9215_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_16BIT | SMSC911X_FORCE_INTERNAL_PHY,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
	.mac		= {0x00, 0x80, 0x00, 0x23, 0x45, 0x67},
};

static struct platform_device smdkc210_smsc911x = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smdkc210_smsc911x_resources),
	.resource	= smdkc210_smsc911x_resources,
	.dev		= {
		.platform_data	= &smsc9215_config,
	},
};

static struct regulator_consumer_supply max8952_supply =
	REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply max8649_supply =
	REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply max8649a_supply =
	REGULATOR_SUPPLY("vdd_g3d", NULL);

static struct regulator_init_data max8952_init_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		= 770000,
		.max_uV		= 1400000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.uV		= 1200000,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8952_supply,
};

static struct regulator_init_data max8649_init_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		= 750000,
		.max_uV		= 1380000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.uV		= 1100000,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8649_supply,
};

static struct regulator_init_data max8649a_init_data = {
	.constraints	= {
		.name		= "vdd_g3d range",
		.min_uV		= 750000,
		.max_uV		= 1380000,
		.always_on	= 0,
		.boot_on	= 0,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.uV		= 1200000,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8649a_supply,
};

static struct max8649_platform_data exynos4_max8952_info = {
	.mode		= 3,	/* VID1 = 1, VID0 = 1 */
	.extclk		= 0,
	.ramp_timing	= MAX8649_RAMP_32MV,
	.regulator	= &max8952_init_data,
};

static struct max8649_platform_data exynos4_max8649_info = {
	.mode		= 2,	/* VID1 = 1, VID0 = 0 */
	.extclk		= 0,
	.ramp_timing	= MAX8649_RAMP_32MV,
	.regulator	= &max8649_init_data,
};

static struct max8649_platform_data exynos4_max8649a_info = {
	.mode		= 2,	/* VID1 = 1, VID0 = 0 */
	.extclk		= 0,
	.ramp_timing	= MAX8649_RAMP_32MV,
	.regulator	= &max8649a_init_data,
};

static struct gpio_event_direct_entry smdkc210_keypad_key_map[] = {
	{
		.gpio   = EXYNOS4_GPX0(0),
		.code   = KEY_POWER,
	}
};

static struct gpio_event_input_info smdkc210_keypad_key_info = {
	.info.func              = gpio_event_input_func,
	.info.no_suspend        = true,
	.debounce_time.tv64	= 5 * NSEC_PER_MSEC,
	.type                   = EV_KEY,
	.keymap                 = smdkc210_keypad_key_map,
	.keymap_size            = ARRAY_SIZE(smdkc210_keypad_key_map)
};

static struct gpio_event_info *smdkc210_input_info[] = {
	&smdkc210_keypad_key_info.info,
};

static struct gpio_event_platform_data smdkc210_input_data = {
	.names  = {
		"smdkc210-keypad",
		NULL,
	},
	.info           = smdkc210_input_info,
	.info_count     = ARRAY_SIZE(smdkc210_input_info),
};

static struct platform_device smdkc210_input_device = {
	.name   = GPIO_EVENT_DEV_NAME,
	.id     = 0,
	.dev    = {
		.platform_data = &smdkc210_input_data,
	},
};

static struct regulator_consumer_supply wm8994_fixed_voltage0_supplies[] = {
	REGULATOR_SUPPLY("AVDD2", "1-001a"),
	REGULATOR_SUPPLY("CPVDD", "1-001a"),
};

static struct regulator_consumer_supply wm8994_fixed_voltage1_supplies[] = {
	REGULATOR_SUPPLY("SPKVDD1", "1-001a"),
	REGULATOR_SUPPLY("SPKVDD2", "1-001a"),
};

static struct regulator_consumer_supply wm8994_fixed_voltage2_supplies =
	REGULATOR_SUPPLY("DBVDD", "1-001a");

static struct regulator_init_data wm8994_fixed_voltage0_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wm8994_fixed_voltage0_supplies),
	.consumer_supplies	= wm8994_fixed_voltage0_supplies,
};

static struct regulator_init_data wm8994_fixed_voltage1_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wm8994_fixed_voltage1_supplies),
	.consumer_supplies	= wm8994_fixed_voltage1_supplies,
};

static struct regulator_init_data wm8994_fixed_voltage2_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8994_fixed_voltage2_supplies,
};

static struct fixed_voltage_config wm8994_fixed_voltage0_config = {
	.supply_name	= "VDD_1.8V",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &wm8994_fixed_voltage0_init_data,
};

static struct fixed_voltage_config wm8994_fixed_voltage1_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.init_data	= &wm8994_fixed_voltage1_init_data,
};

static struct fixed_voltage_config wm8994_fixed_voltage2_config = {
	.supply_name	= "VDD_3.3V",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.init_data	= &wm8994_fixed_voltage2_init_data,
};

static struct platform_device wm8994_fixed_voltage0 = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data	= &wm8994_fixed_voltage0_config,
	},
};

static struct platform_device wm8994_fixed_voltage1 = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev		= {
		.platform_data	= &wm8994_fixed_voltage1_config,
	},
};

static struct platform_device wm8994_fixed_voltage2 = {
	.name		= "reg-fixed-voltage",
	.id		= 2,
	.dev		= {
		.platform_data	= &wm8994_fixed_voltage2_config,
	},
};

static struct regulator_consumer_supply wm8994_avdd1_supply =
	REGULATOR_SUPPLY("AVDD1", "1-001a");

static struct regulator_consumer_supply wm8994_dcvdd_supply =
	REGULATOR_SUPPLY("DCVDD", "1-001a");

static struct regulator_init_data wm8994_ldo1_data = {
	.constraints	= {
		.name		= "AVDD1",
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8994_avdd1_supply,
};

static struct regulator_init_data wm8994_ldo2_data = {
	.constraints	= {
		.name		= "DCVDD",
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8994_dcvdd_supply,
};

static struct wm8994_pdata wm8994_platform_data = {
	/* configure gpio1 function: 0x0001(Logic level input/output) */
	.gpio_defaults[0] = 0x0001,
	/* configure gpio3/4/5/7 function for AIF2 voice */
	.gpio_defaults[2] = 0x8100,/* BCLK2 in */
	.gpio_defaults[3] = 0x8100,/* LRCLK2 in */
	.gpio_defaults[4] = 0x8100,/* DACDAT2 in */
	/* configure gpio6 function: 0x0001(Logic level input/output) */
	.gpio_defaults[5] = 0x0001,
	.gpio_defaults[6] = 0x0100,/* ADCDAT2 out */
	.ldo[0] = { 0, NULL, &wm8994_ldo1_data },
	.ldo[1] = { 0, NULL, &wm8994_ldo2_data },
};

static uint32_t smdkc210_keymap[] __initdata = {
	/* KEY(row, col, keycode) */
	KEY(0, 3, KEY_1), KEY(0, 4, KEY_2), KEY(0, 5, KEY_3),
	KEY(0, 6, KEY_4), KEY(0, 7, KEY_5),
	KEY(1, 3, KEY_A), KEY(1, 4, KEY_B), KEY(1, 5, KEY_C),
	KEY(1, 6, KEY_D), KEY(1, 7, KEY_E)
};

static struct matrix_keymap_data smdkc210_keymap_data __initdata = {
	.keymap		= smdkc210_keymap,
	.keymap_size	= ARRAY_SIZE(smdkc210_keymap),
};

static struct samsung_keypad_platdata smdkc210_keypad_data __initdata = {
	.keymap_data	= &smdkc210_keymap_data,
	.rows		= 2,
	.cols		= 8,
};

#ifdef CONFIG_BATTERY_SAMSUNG
static struct platform_device samsung_device_battery = {
	.name	= "samsung-fake-battery",
	.id	= -1,
};
#endif

static struct i2c_board_info i2c_devs0[] __initdata = {
	{
		I2C_BOARD_INFO("max8649", 0x62),
		.platform_data	= &exynos4_max8649a_info,
	}, {
		I2C_BOARD_INFO("max8952", 0x60),
		.platform_data	= &exynos4_max8952_info,
	},
};

static struct i2c_board_info i2c_devs1[] __initdata = {
	{
		I2C_BOARD_INFO("wm8994", 0x1a),
		.platform_data	= &wm8994_platform_data,
	}, {
		I2C_BOARD_INFO("max8649", 0x60),
		.platform_data	= &exynos4_max8649_info,
	},
#ifdef CONFIG_VIDEO_TVOUT
	{
		I2C_BOARD_INFO("s5p_ddc", (0x74 >> 1)),
	},
#endif
};

#ifdef CONFIG_TOUCHSCREEN_S3C2410
static struct s3c2410_ts_mach_info s3c_ts_platform __initdata = {
	.delay			= 10000,
	.presc			= 49,
	.oversampling_shift	= 2,
	.cal_x_max		= 480,
	.cal_y_max		= 800,
	.cal_param		= {
		33, -9156, 34720100, 14819, 57, -4234968, 65536
	},
};
#endif

#ifdef CONFIG_S3C_DEV_HWMON
static struct s3c_hwmon_pdata smdkc210_hwmon_pdata __initdata = {
	/* Reference voltage (1.2V) */
	.in[0] = &(struct s3c_hwmon_chcfg) {
		.name		= "smdk:reference-voltage",
		.mult		= 3300,
		.div		= 4096,
	},
};
#endif

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data pmem_pdata = {
	.name		= "pmem",
	.no_allocator	= 1,
	.cached		= 0,
	.start		= 0,
	.size		= 0
};

static struct android_pmem_platform_data pmem_gpu1_pdata = {
	.name		= "pmem_gpu1",
	.no_allocator	= 1,
	.cached		= 0,
	.start		= 0,
	.size		= 0,
};

static struct platform_device pmem_device = {
	.name	= "android_pmem",
	.id	= 0,
	.dev	= {
		.platform_data = &pmem_pdata
	},
};

static struct platform_device pmem_gpu1_device = {
	.name	= "android_pmem",
	.id	= 1,
	.dev	= {
		.platform_data = &pmem_gpu1_pdata
	},
};

static void __init android_pmem_set_platdata(void)
{
#if defined(CONFIG_S5P_MEM_CMA)
	pmem_pdata.size = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM * SZ_1K;
	pmem_gpu1_pdata.size = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_GPU1 * SZ_1K;
#else
	pmem_pdata.start = (u32)s5p_get_media_memory_bank(S5P_MDEV_PMEM, 0);
	pmem_pdata.size = (u32)s5p_get_media_memsize_bank(S5P_MDEV_PMEM, 0);
	pmem_gpu1_pdata.start = (u32)s5p_get_media_memory_bank(S5P_MDEV_PMEM_GPU1, 0);
	pmem_gpu1_pdata.size = (u32)s5p_get_media_memsize_bank(S5P_MDEV_PMEM_GPU1, 0);
#endif
}
#endif

/* USB EHCI */
#ifdef CONFIG_USB_EHCI_S5P
static struct s5p_ehci_platdata smdkc210_ehci_pdata;

static void __init smdkc210_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &smdkc210_ehci_pdata;

	s5p_ehci_set_platdata(pdata);
}
#endif

static struct platform_device *smdkc210_devices[] __initdata = {
#ifdef CONFIG_ANDROID_PMEM
	&pmem_device,
	&pmem_gpu1_device,
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
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	&exynos4_device_dwmci,
#endif
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_adc,
#ifdef CONFIG_S3C_DEV_HWMON
	&s3c_device_hwmon,
#endif
#ifdef CONFIG_TOUCHSCREEN_S3C2410
#ifdef CONFIG_S3C_DEV_ADC
	&s3c_device_ts,
#elif CONFIG_S3C_DEV_ADC1
	&s3c_device_ts1,
#endif
#endif
	&s3c_device_rtc,
	&s3c_device_wdt,
	&exynos4_device_ac97,
	&exynos4_device_i2s0,
	&exynos4_device_pcm0,
	&samsung_device_keypad,
#ifdef CONFIG_BATTERY_SAMSUNG
	&samsung_device_battery,
#endif
	&exynos4_device_pd[PD_MFC],
	&exynos4_device_pd[PD_G3D],
	&exynos4_device_pd[PD_LCD0],
	&exynos4_device_pd[PD_LCD1],
	&exynos4_device_pd[PD_CAM],
	&exynos4_device_pd[PD_TV],
	&exynos4_device_pd[PD_GPS],
#ifdef CONFIG_EXYNOS4_DEV_SYSMMU
	&exynos4_device_sysmmu[SYSMMU_MDMA],
	&exynos4_device_sysmmu[SYSMMU_SSS],
	&exynos4_device_sysmmu[SYSMMU_FIMC0],
	&exynos4_device_sysmmu[SYSMMU_FIMC1],
	&exynos4_device_sysmmu[SYSMMU_FIMC2],
	&exynos4_device_sysmmu[SYSMMU_FIMC3],
	&exynos4_device_sysmmu[SYSMMU_JPEG],
	&exynos4_device_sysmmu[SYSMMU_FIMD0],
	&exynos4_device_sysmmu[SYSMMU_FIMD1],
	&exynos4_device_sysmmu[SYSMMU_PCIe],
	&exynos4_device_sysmmu[SYSMMU_G2D],
	&exynos4_device_sysmmu[SYSMMU_ROTATOR],
	&exynos4_device_sysmmu[SYSMMU_MDMA2],
	&exynos4_device_sysmmu[SYSMMU_TV],
	&exynos4_device_sysmmu[SYSMMU_MFC_L],
	&exynos4_device_sysmmu[SYSMMU_MFC_R],
#endif
	&wm8994_fixed_voltage0,
	&wm8994_fixed_voltage1,
	&wm8994_fixed_voltage2,
	&samsung_asoc_dma,
/* mainline fimd */
#ifdef CONFIG_FB_S3C
	&s5p_device_fimd0,
#if defined(CONFIG_LCD_AMS369FG06)
	&s3c_device_spi_gpio,
#elif defined(CONFIG_LCD_WA101S)
	&smdkc210_lcd_wa101s,
#elif defined(CONFIG_LCD_LTE480WV)
	&smdkc210_lcd_lte480wv,
#endif
#endif
/* legacy fimd */
#ifdef CONFIG_FB_S5P
	&s3c_device_fb,
#ifdef CONFIG_FB_S5P_AMS369FG06
	&s3c_device_spi_gpio,
#endif
#endif
#ifdef CONFIG_SAMSUNG_DEV_PWM
	&s3c_device_timer[1],
#ifdef CONFIG_BACKLIGHT_PWM
	&smdkc210_backlight_device,
#endif
#endif
#ifdef CONFIG_VIDEO_TVOUT
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,
#endif
	&smdkc210_smsc911x,
	&smdkc210_input_device,
#ifdef CONFIG_VIDEO_FIMC
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
	&s3c_device_fimc3,
#endif
#ifdef CONFIG_VIDEO_MFC5X
	&s5p_device_mfc,
#endif
#ifdef CONFIG_VIDEO_FIMG2D
	&s5p_device_fimg2d,
#endif
#ifdef CONFIG_VIDEO_FIMC_MIPI
	&s3c_device_csis0,
	&s3c_device_csis1,
#endif
#ifdef CONFIG_VIDEO_JPEG
	&s5p_device_jpeg,
#endif
#ifdef CONFIG_USB_EHCI_S5P
	&s5p_device_ehci,
#endif
#ifdef CONFIG_USB_OHCI_S5P
	&s5p_device_ohci,
#endif
#ifdef CONFIG_USB_GADGET
        &s3c_device_usbgadget,
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
        &s3c_device_rndis,
#endif
#ifdef CONFIG_USB_ANDROID
        &s3c_device_android_usb,
        &s3c_device_usb_mass_storage,
#endif
#ifdef CONFIG_EXYNOS4_SETUP_THERMAL
	&s5p_device_tmu,
#endif
#ifdef CONFIG_S5P_DEV_ACE
	&s5p_device_ace,
#endif
};

#if defined(CONFIG_VIDEO_TVOUT)
static struct s5p_platform_hpd hdmi_hpd_data __initdata = {

};
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif

static void __init smdkc210_button_init(void)
{
	s3c_gpio_cfgpin(EXYNOS4_GPX0(0), (0xf << 0));
	s3c_gpio_setpull(EXYNOS4_GPX0(0), S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(EXYNOS4_GPX3(7), (0xf << 28));
	s3c_gpio_setpull(EXYNOS4_GPX3(7), S3C_GPIO_PULL_NONE);
}

static void __init smdkc210_smsc911x_init(void)
{
	u32 cs1;

	/* configure nCS1 width to 16 bits */
	cs1 = __raw_readl(S5P_SROM_BW) &
		~(S5P_SROM_BW__CS_MASK << S5P_SROM_BW__NCS1__SHIFT);
	cs1 |= ((1 << S5P_SROM_BW__DATAWIDTH__SHIFT) |
		(1 << S5P_SROM_BW__WAITENABLE__SHIFT) |
		(1 << S5P_SROM_BW__BYTEENABLE__SHIFT)) <<
		S5P_SROM_BW__NCS1__SHIFT;
	__raw_writel(cs1, S5P_SROM_BW);

	/* set timing for nCS1 suitable for ethernet chip */
	__raw_writel((0x1 << S5P_SROM_BCX__PMC__SHIFT) |
		     (0x9 << S5P_SROM_BCX__TACP__SHIFT) |
		     (0xc << S5P_SROM_BCX__TCAH__SHIFT) |
		     (0x1 << S5P_SROM_BCX__TCOH__SHIFT) |
		     (0x6 << S5P_SROM_BCX__TACC__SHIFT) |
		     (0x1 << S5P_SROM_BCX__TCOS__SHIFT) |
		     (0x1 << S5P_SROM_BCX__TACS__SHIFT), S5P_SROM_BC1);
}

#if defined(CONFIG_S5P_MEM_CMA)
static void __init exynos4_reserve_mem(void)
{
	static struct cma_region regions[] __initdata = {
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
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1
		{
			.name = "fimc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1 * SZ_1K,
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
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1
		{
			.name = "mfc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1 * SZ_1K,
			{
				.alignment = 1 << 17,
			},
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0
		{
			.name = "mfc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0 * SZ_1K,
			{
				.alignment = 1 << 17,
			},
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC
		{
			.name = "mfc",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC * SZ_1K,
			{
				.alignment = 1 << 17,
			},
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG
		{
			.name = "jpeg",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP
		{
			.name = "srp",
			.size = CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMG2D
		{
			.name = "fimg2d",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMG2D * SZ_1K,
			.start = 0
		},
#endif
		{
			.size = 0
		},
	};

	static const char map[] __initconst =
		"android_pmem.0=pmem;android_pmem.1=pmem_gpu1;"
		"s3cfb.0=fimd;exynos4-fb.0=fimd;"
		"s3c-fimc.0=fimc0;s3c-fimc.1=fimc1;s3c-fimc.2=fimc2;"
		"s3c-fimc.3=fimc3;s5p-mfc=mfc,mfc0,mfc1;s5p-rp=srp;"
		"s5p-jpeg=jpeg;"
		"s5p-fimg2d=fimg2d";

	cma_set_defaults(regions, map);
	cma_early_regions_reserve(NULL);
}
#endif

static void __init smdkc210_map_io(void)
{
	clk_xusbxti.rate = 24000000;

	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdkc210_uartcfgs, ARRAY_SIZE(smdkc210_uartcfgs));

#if defined(CONFIG_S5P_MEM_CMA)
	exynos4_reserve_mem();
#else
	s5p_reserve_mem(S5P_RANGE_MFC);
#endif
}

static void __init smdkc210_machine_init(void)
{
#if defined(CONFIG_EXYNOS4_DEV_PD) && !defined(CONFIG_PM_RUNTIME)
	/*
	 * These power domains should be always on
	 * without runtime pm support.
	 */
	exynos4_pd_enable(&exynos4_device_pd[PD_MFC].dev);
	exynos4_pd_enable(&exynos4_device_pd[PD_G3D].dev);
	exynos4_pd_enable(&exynos4_device_pd[PD_LCD0].dev);
	exynos4_pd_enable(&exynos4_device_pd[PD_LCD1].dev);
	exynos4_pd_enable(&exynos4_device_pd[PD_CAM].dev);
	exynos4_pd_enable(&exynos4_device_pd[PD_TV].dev);
	exynos4_pd_enable(&exynos4_device_pd[PD_GPS].dev);
#endif
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

	smdkc210_button_init();
	smdkc210_smsc911x_init();

#ifdef CONFIG_S3C_DEV_HSMMC
	s3c_sdhci0_set_platdata(&smdkc210_hsmmc0_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	s3c_sdhci1_set_platdata(&smdkc210_hsmmc1_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s3c_sdhci2_set_platdata(&smdkc210_hsmmc2_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s3c_sdhci3_set_platdata(&smdkc210_hsmmc3_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HWMON
	s3c_hwmon_set_platdata(&smdkc210_hwmon_pdata);
#endif

#ifdef CONFIG_FB_S3C
#ifdef CONFIG_LCD_AMS369FG06
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif
	s5p_fimd0_set_platdata(&smdkc210_lcd0_pdata);
#ifdef CONFIG_EXYNOS4_DEV_PD
	s5p_device_fimd0.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#endif

#ifdef CONFIG_FB_S5P
#ifdef CONFIG_FB_S5P_AMS369FG06
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
	s3cfb_set_platdata(&ams369fg06_data);
#else
	s3cfb_set_platdata(NULL);
#endif
#ifdef CONFIG_EXYNOS4_DEV_PD
	s3c_device_fb.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#endif

#ifdef CONFIG_EXYNOS4_DEV_PD
#ifdef CONFIG_VIDEO_JPEG
	s5p_device_jpeg.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#endif
#if defined(CONFIG_VIDEO_TVOUT)
	s5p_hdmi_hpd_set_platdata(&hdmi_hpd_data);
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
#ifdef CONFIG_EXYNOS4_DEV_PD
	s5p_device_tvout.dev.parent = &exynos4_device_pd[PD_TV].dev;
#endif
#endif
	samsung_keypad_set_platdata(&smdkc210_keypad_data);

#ifdef CONFIG_TOUCHSCREEN_S3C2410
#ifdef CONFIG_S3C_DEV_ADC
	s3c24xx_ts_set_platdata(&s3c_ts_platform);
#endif
#ifdef CONFIG_S3C_DEV_ADC1
	s3c24xx_ts1_set_platdata(&s3c_ts_platform);
#endif
#endif
#ifdef CONFIG_ANDROID_PMEM
	android_pmem_set_platdata();
#endif
#ifdef CONFIG_VIDEO_FIMC
	s3c_fimc0_set_platdata(&fimc_plat);
	s3c_fimc1_set_platdata(&fimc_plat);
	s3c_fimc2_set_platdata(&fimc_plat);
	s3c_fimc3_set_platdata(&fimc_plat);
#ifdef CONFIG_ITU_A
	smdkc210_cam0_reset(1);
#endif
#ifdef CONFIG_ITU_B
	smdkc210_cam1_reset(1);
#endif
#ifdef CONFIG_VIDEO_FIMC_MIPI
	s3c_csis0_set_platdata(NULL);
	s3c_csis1_set_platdata(NULL);
#ifdef CONFIG_CSI_C
	smdkc210_mipi_cam0_reset(1);
#endif
#ifdef CONFIG_CSI_D
	smdkc210_mipi_cam1_reset(1);
#endif
#endif /* CONFIG_VIDEO_FIMC_MIPI */
#endif /* CONFIG_VIDEO_FIMC */

#ifdef CONFIG_EXYNOS4_DEV_PD
#ifdef CONFIG_VIDEO_FIMC
	s3c_device_fimc0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc2.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc3.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#endif
#ifdef CONFIG_EXYNOS4_SETUP_THERMAL
	s5p_tmu_set_platdata(NULL);
#endif

#ifdef CONFIG_VIDEO_FIMG2D
	s5p_fimg2d_set_platdata(&fimg2d_data);
#ifdef CONFIG_EXYNOS4_DEV_PD
	s5p_device_fimg2d.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#endif
#ifdef CONFIG_USB_EHCI_S5P
	smdkc210_ehci_init();
#endif

	platform_add_devices(smdkc210_devices, ARRAY_SIZE(smdkc210_devices));

#ifdef CONFIG_FB_S3C
	exynos4_fimd0_setup_clock(&s5p_device_fimd0.dev, "mout_mpll",
				134 * MHZ);
#endif
}

MACHINE_START(SMDKC210, "SMDKC210")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= smdkc210_map_io,
	.init_machine	= smdkc210_machine_init,
	.timer		= &exynos4_timer,
MACHINE_END
