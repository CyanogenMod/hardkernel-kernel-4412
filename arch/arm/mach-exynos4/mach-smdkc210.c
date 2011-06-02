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
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/smsc911x.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/wm8994/pdata.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <video/platform_lcd.h>

#include <plat/regs-serial.h>
#include <plat/regs-srom.h>
#include <plat/exynos4.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/fb.h>
#include <plat/fb-s5p.h>
#include <plat/gpio-cfg.h>
#include <plat/adc.h>
#include <plat/ts.h>
#include <plat/keypad.h>
#include <plat/sdhci.h>
#include <plat/iic.h>
#include <plat/pd.h>
#include <plat/media.h>

#include <mach/map.h>
#include <mach/media.h>
#include <mach/regs-fb.h>
#include <mach/sysmmu.h>

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

static struct s3c_sdhci_platdata smdkc210_hsmmc0_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_GPIO,
	.ext_cd_gpio		= EXYNOS4_GPK0(2),
	.ext_cd_gpio_invert	= 1,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH0_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};

static struct s3c_sdhci_platdata smdkc210_hsmmc1_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_GPIO,
	.ext_cd_gpio		= EXYNOS4_GPK0(2),
	.ext_cd_gpio_invert	= 1,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};

static struct s3c_sdhci_platdata smdkc210_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_GPIO,
	.ext_cd_gpio		= EXYNOS4_GPK2(2),
	.ext_cd_gpio_invert	= 1,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH2_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};

static struct s3c_sdhci_platdata smdkc210_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_GPIO,
	.ext_cd_gpio		= EXYNOS4_GPK2(2),
	.ext_cd_gpio_invert	= 1,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};

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
	{
		.dev_name	= "1-001a",
		.supply		= "AVDD2",
	}, {
		.dev_name	= "1-001a",
		.supply		= "CPVDD",
	},
};

static struct regulator_consumer_supply wm8994_fixed_voltage1_supplies[] = {
	{
		.dev_name	= "1-001a",
		.supply		= "SPKVDD1",
	}, {
		.dev_name	= "1-001a",
		.supply		= "SPKVDD2",
	},
};

static struct regulator_consumer_supply wm8994_fixed_voltage2_supplies[] = {
	{
		.dev_name	= "1-001a",
		.supply		= "DBVDD",
	},
};

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
	.num_consumer_supplies	= ARRAY_SIZE(wm8994_fixed_voltage2_supplies),
	.consumer_supplies	= wm8994_fixed_voltage2_supplies,
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

static struct regulator_consumer_supply wm8994_avdd1_supply = {
	.dev_name	= "1-001a",
	.supply		= "AVDD1",
};

static struct regulator_consumer_supply wm8994_dcvdd_supply = {
	.dev_name	= "1-001a",
	.supply		= "DCVDD",
};

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

static struct i2c_board_info i2c_devs1[] __initdata = {
	{
		I2C_BOARD_INFO("wm8994", 0x1a),
		.platform_data	= &wm8994_platform_data,
	},
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

static struct platform_device *smdkc210_devices[] __initdata = {
/* mainline fimd */
#ifdef CONFIG_FB_S3C
	&s5p_device_fimd0,
#ifdef CONFIG_LCD_AMS369FG06
	&s3c_device_spi_gpio,
#endif
#endif
/* legacy fimd */
#ifdef CONFIG_FB_S5P
	&s3c_device_fb,
#ifdef CONFIG_FB_S5P_AMS369FG06
	&s3c_device_spi_gpio,
#endif
#endif
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_hsmmc2,
	&s3c_device_hsmmc3,
	&s3c_device_i2c1,
	&s3c_device_adc,
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
#ifdef CONFIG_FB_S3C
#if defined(CONFIG_LCD_WA101S)
	&smdkc210_lcd_wa101s,
#elif defined(CONFIG_LCD_LTE480WV)
	&smdkc210_lcd_lte480wv,
#endif
#endif
	&smdkc210_smsc911x,
	&smdkc210_input_device,
};

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

static void __init smdkc210_map_io(void)
{
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdkc210_uartcfgs, ARRAY_SIZE(smdkc210_uartcfgs));

	s5p_reserve_mem(S5P_RANGE_MFC);
}

static void __init smdkc210_machine_init(void)
{
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

	smdkc210_button_init();
	smdkc210_smsc911x_init();

	s3c_sdhci0_set_platdata(&smdkc210_hsmmc0_pdata);
	s3c_sdhci1_set_platdata(&smdkc210_hsmmc1_pdata);
	s3c_sdhci2_set_platdata(&smdkc210_hsmmc2_pdata);
	s3c_sdhci3_set_platdata(&smdkc210_hsmmc3_pdata);

#ifdef CONFIG_FB_S3C
#ifdef CONFIG_LCD_AMS369FG06
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif
	s5p_fimd0_set_platdata(&smdkc210_lcd0_pdata);
#endif

#ifdef CONFIG_FB_S5P
#ifdef CONFIG_FB_S5P_AMS369FG06
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
	s3cfb_set_platdata(&ams369fg06_data);
#else
	s3cfb_set_platdata(NULL);
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

	platform_add_devices(smdkc210_devices, ARRAY_SIZE(smdkc210_devices));
}

MACHINE_START(SMDKC210, "SMDKC210")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= smdkc210_map_io,
	.init_machine	= smdkc210_machine_init,
	.timer		= &exynos4_timer,
MACHINE_END
