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
#include <linux/gpio_keys.h>
#include <linux/gpio_event.h>
#include <linux/lcd.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/smsc911x.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max8997.h>
#ifdef CONFIG_SND_SOC_MC1N2
#include <linux/mfd/mc1n2_pdata.h>
#endif
#include <linux/i2c/mxt224_u1.h>
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
#include <plat/mshci.h>
#include <plat/iic.h>
#include <plat/pd.h>
#include <plat/media.h>
#include <plat/s5p-clock.h>
#include <plat/tvout.h>
#include <plat/fimg2d.h>
#include <plat/ehci.h>
#include <plat/usbgadget.h>

#include <mach/map.h>
#include <mach/media.h>
#include <mach/regs-fb.h>
#include <mach/sysmmu.h>
#include <mach/regs-clock.h>

#ifdef CONFIG_VIDEO_M5MO
#include <media/m5mo_platform.h>
#endif
#if defined(CONFIG_EXYNOS4_SETUP_THERMAL)
#include <plat/s5p-tmu.h>
#endif

#include <../../../drivers/video/samsung/s3cfb.h> 
#include "u1.h"

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

#define WRITEBACK_ENABLED

#ifdef CONFIG_VIDEO_FIMC
/*
 * External camera reset
 * Because the most of cameras take i2c bus signal, so that
 * you have to reset at the boot time for other i2c slave devices.
 * This function also called at fimc_init_camera()
 * Do optimization for cameras on your platform.
*/

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

static int m5mo_power_on(void)
{
	struct regulator *regulator;
	int ret = 0;

	printk(KERN_DEBUG "%s: in\n", __func__);

	ret = gpio_request(GPIO_CAM_VGA_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_VGA_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_VGA_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_VGA_nRST)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_SENSOR_CORE, "GPE2");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(CAM_SENSOR_CORE)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_IO_EN, "GPE2");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(CAM_IO_EN)\n");
		return ret;
	}
	ret = gpio_request(GPIO_VT_CAM_15V, "GPE2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_VT_CAM_15V)\n");
		return ret;
	}
	ret = gpio_request(GPIO_ISP_RESET, "GPY3");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(ISP_RESET)\n");
		return ret;
	}
#if defined(CONFIG_MACH_C1_REV02) || defined(CONFIG_MACH_C1Q1_REV02) \
 || defined(CONFIG_MACH_P6_REV02) || defined(CONFIG_MACH_TALBOT_REV02)
	ret = gpio_request(GPIO_8M_AF_EN, "GPK1");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(8M_AF_EN)\n");
		return ret;
	}
#endif

	/* CAM_VT_nSTBY low */
	ret = gpio_direction_output(GPIO_CAM_VGA_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "output VGA_nSTBY");

	/* CAM_VT_nRST	low */
	gpio_direction_output(GPIO_CAM_VGA_nRST, 0);
	CAM_CHECK_ERR_RET(ret, "output VGA_nRST");
	udelay(10);

	/* CAM_ISP_CORE_1.2V */
	regulator = regulator_get(NULL, "cam_isp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "enable cam_isp_core");
	/* No delay */

	/* CAM_SENSOR_CORE_1.2V */
	ret = gpio_direction_output(GPIO_CAM_SENSOR_CORE, 1);
	CAM_CHECK_ERR_RET(ret, "output senser_core");
	udelay(10);

	/* CAM_SENSOR_A2.8V */
	ret = gpio_direction_output(GPIO_CAM_IO_EN, 1);
	CAM_CHECK_ERR_RET(ret, "output IO_EN");
	/* it takes about 100us at least during level transition.*/
	udelay(160); /* 130us -> 160us */

	/* VT_CORE_1.5V */
	ret = gpio_direction_output(GPIO_VT_CAM_15V, 1);
	CAM_CHECK_ERR_RET(ret, "output VT_CAM_1.5V");
	udelay(20);

	/* VT_CAM_1.8V */
	regulator = regulator_get(NULL, "vt_cam_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "enable vt_1.8v");
	udelay(10);

	/* CAM_ISP_1.8V */
	regulator = regulator_get(NULL, "cam_isp");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "enable cam_isp");
	udelay(120); /* at least */

	/* CAM_SENSOR_IO_1.8V */
	regulator = regulator_get(NULL, "cam_sensor_io");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "enable sensor_io");
	udelay(30);

	/* MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	CAM_CHECK_ERR_RET(ret, "cfg mclk");
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_NONE);
	udelay(70);

	/* ISP_RESET */
	ret = gpio_direction_output(GPIO_ISP_RESET, 1);
	CAM_CHECK_ERR_RET(ret, "output reset");
	mdelay(4);

	/* CAM_AF_2.8V */
#if defined(CONFIG_MACH_C1_REV02) || defined(CONFIG_MACH_C1Q1_REV02) \
 || defined(CONFIG_MACH_P6_REV02) || defined(CONFIG_MACH_TALBOT_REV02)
	/* 8M_AF_2.8V_EN */
	ret = gpio_direction_output(GPIO_8M_AF_EN, 1);
	CAM_CHECK_ERR(ret, "output AF");
#else
	regulator = regulator_get(NULL, "cam_af");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "output cam_af");
#endif

	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);
	gpio_free(GPIO_CAM_SENSOR_CORE);
	gpio_free(GPIO_CAM_IO_EN);
	gpio_free(GPIO_VT_CAM_15V);
	gpio_free(GPIO_ISP_RESET);
#if defined(CONFIG_MACH_C1_REV02) || defined(CONFIG_MACH_C1Q1_REV02) \
 || defined(CONFIG_MACH_P6_REV02) || defined(CONFIG_MACH_TALBOT_REV02)
	gpio_free(GPIO_8M_AF_EN);
#endif

	return ret;
}

static int m5mo_power_down(void)
{
	struct regulator *regulator;
	int ret = 0;

	printk(KERN_DEBUG "%s: in\n", __func__);

#if defined(CONFIG_MACH_C1_REV02) || defined(CONFIG_MACH_C1Q1_REV02) \
 || defined(CONFIG_MACH_P6_REV02) || defined(CONFIG_MACH_TALBOT_REV02)
	ret = gpio_request(GPIO_8M_AF_EN, "GPK1");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(8M_AF_EN)\n");
		return ret;
	}
#endif
	ret = gpio_request(GPIO_ISP_RESET, "GPY3");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(ISP_RESET)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_IO_EN, "GPE2");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(GPIO_CAM_IO_EN)\n");
		return ret;
	}
	ret = gpio_request(GPIO_CAM_SENSOR_CORE, "GPE2");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(CAM_SENSOR_COR)\n");
		return ret;
	}
	ret = gpio_request(GPIO_VT_CAM_15V, "GPE2");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(GPIO_VT_CAM_15V)\n");
		return ret;
	}

	/* s3c_i2c0_force_stop(); */

	mdelay(1);

	/* CAM_AF_2.8V */
#if defined(CONFIG_MACH_C1_REV02) || defined(CONFIG_MACH_C1Q1_REV02) \
 || defined(CONFIG_MACH_P6_REV02) || defined(CONFIG_MACH_TALBOT_REV02)
	/* 8M_AF_2.8V_EN */
	ret = gpio_direction_output(GPIO_8M_AF_EN, 0);
	CAM_CHECK_ERR(ret, "output AF");
#else
	regulator = regulator_get(NULL, "cam_af");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "disable cam_af");
#endif

	/* ISP_RESET */
	ret = gpio_direction_output(GPIO_ISP_RESET, 0);
	CAM_CHECK_ERR(ret, "output reset");
	#ifdef CONFIG_TARGET_LOCALE_KOR
	mdelay(3); /* fix without seeing signal form for kor.*/
	#else
	mdelay(2);
	#endif

	/* MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_DOWN);
	CAM_CHECK_ERR(ret, "cfg mclk");
	udelay(20);

	/* CAM_SENSOR_IO_1.8V */
	regulator = regulator_get(NULL, "cam_sensor_io");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "disable, sensor_io");
	udelay(10);

	/* CAM_ISP_1.8V */
	regulator = regulator_get(NULL, "cam_isp");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "disable cam_isp");
	udelay(500); /* 100us -> 500us */

	/* VT_CAM_1.8V */
	regulator = regulator_get(NULL, "vt_cam_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "disable vt_1.8v");
	udelay(250); /* 10us -> 250us */

	/* VT_CORE_1.5V */
	ret = gpio_direction_output(GPIO_VT_CAM_15V, 0);
	CAM_CHECK_ERR(ret, "output VT_CAM_1.5V");
	udelay(300); /*10 -> 300 us */

	/* CAM_SENSOR_A2.8V */
	ret = gpio_direction_output(GPIO_CAM_IO_EN, 0);
	CAM_CHECK_ERR(ret, "output IO_EN");
	udelay(800);

	/* CAM_SENSOR_CORE_1.2V */
	ret = gpio_direction_output(GPIO_CAM_SENSOR_CORE, 0);
	CAM_CHECK_ERR(ret, "output SENSOR_CORE");
	udelay(5);

	/* CAM_ISP_CORE_1.2V */
	regulator = regulator_get(NULL, "cam_isp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "disable isp_core");

#if defined(CONFIG_MACH_C1_REV02) || defined(CONFIG_MACH_C1Q1_REV02) \
 || defined(CONFIG_MACH_P6_REV02) || defined(CONFIG_MACH_TALBOT_REV02)
	gpio_free(GPIO_8M_AF_EN);
#endif
	gpio_free(GPIO_ISP_RESET);
	gpio_free(GPIO_CAM_IO_EN);
	gpio_free(GPIO_CAM_SENSOR_CORE);
	gpio_free(GPIO_VT_CAM_15V);

	return ret;
}

int s3c_csis_power(int enable)
{
	struct regulator *regulator;
	int ret = 0;

	/* mipi_1.1v ,mipi_1.8v are always powered-on.
	* If they are off, we then power them on.
	*/
	if (enable) {
		/* VMIPI_1.1V */
		regulator = regulator_get(NULL, "vmipi_1.1v");
		if (IS_ERR(regulator))
			goto error_out;
		if (!regulator_is_enabled(regulator)) {
			printk(KERN_WARNING "%s: vmipi_1.1v is off. so ON\n",
				__func__);
			ret = regulator_enable(regulator);
			CAM_CHECK_ERR(ret, "enable vmipi_1.1v");
		}
		regulator_put(regulator);

		/* VMIPI_1.8V */
		regulator = regulator_get(NULL, "vmipi_1.8v");
		if (IS_ERR(regulator))
			goto error_out;
		if (!regulator_is_enabled(regulator)) {
			printk(KERN_WARNING "%s: vmipi_1.8v is off. so ON\n",
				__func__);
			ret = regulator_enable(regulator);
			CAM_CHECK_ERR(ret, "enable vmipi_1.8v");
		}
		regulator_put(regulator);
	}

	return 0;

error_out:
	printk(KERN_ERR "%s: ERROR: failed to check mipi-power\n", __func__);
	return 0;
}

static int m5mo_power(int enable)
{
	int ret = 0;

	printk("%s %s\n", __func__, enable ? "on" : "down");
	if (enable) {
		ret = m5mo_power_on();
		if (unlikely(ret))
			goto error_out;
	} else
		ret = m5mo_power_down();

	ret = s3c_csis_power(enable);

error_out:
	return ret;
}

static int m5mo_config_isp_irq(void)
{
	s3c_gpio_cfgpin(GPIO_ISP_INT, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(GPIO_ISP_INT, S3C_GPIO_PULL_NONE);
	return 0;
}

static struct m5mo_platform_data m5mo_plat = {
	.default_width = 640, /* 1920 */
	.default_height = 480, /* 1080 */
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
	.config_isp_irq = m5mo_config_isp_irq,
	.irq = IRQ_EINT(13),
};

static struct i2c_board_info  m5mo_i2c_info = {
	I2C_BOARD_INFO("M5MO", 0x1F),
	.platform_data = &m5mo_plat,
	.irq = IRQ_EINT(13),
};

static struct s3c_platform_camera m5mo = {
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 0,
	.cam_power	= m5mo_power, /*smdkv310_mipi_cam0_reset,*/
	.type		= CAM_TYPE_MIPI,
	.fmt		= ITU_601_YCBCR422_8BIT, /*MIPI_CSI_YCBCR422_8BIT*/
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
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
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 0,
	.initialized	= 0,
};
#endif /* #ifdef CONFIG_VIDEO_M5MO */

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
	.camera		= {
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
static struct s3c_sdhci_platdata exynos4_hsmmc0_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH0_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC2
static struct s3c_sdhci_platdata exynos4_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_GPIO,
	.ext_cd_gpio		= EXYNOS4_GPX3(4),
	.ext_cd_gpio_invert	= 1,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
	.vmmc_name		= "cam_af",
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC3
static struct s3c_sdhci_platdata exynos4_hsmmc3_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
};
#endif

#ifdef CONFIG_S5P_DEV_MSHC
static struct s3c_mshci_platdata exynos4_mshc_pdata __initdata = {
	.cd_type		= S3C_MSHCI_CD_PERMANENT,
#if defined(CONFIG_EXYNOS4_MSHC_CH0_8BIT) && \
	defined(CONFIG_EXYNOS4_MSHC_CH0_DDR)
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA | MMC_CAP_1_8V_DDR,
#elif defined(CONFIG_EXYNOS4_MSHC_CH0_8BIT)
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#elif defined(CONFIG_EXYNOS4_MSHC_CH0_DDR)
	.host_caps		= MMC_CAP_1_8V_DDR,
#endif
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
#ifdef CONFIG_FB_S5P_LD9040
static int lcd_cfg_gpio(void)
{
	int i, f3_end = 4;

	for (i = 0; i < 8; i++) {
	/* set GPF0,1,2[0:7] for RGB Interface and Data lines (32bit) */
		s3c_gpio_cfgpin(EXYNOS4_GPF0(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(EXYNOS4_GPF0(i), S3C_GPIO_PULL_NONE);

	}
	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(EXYNOS4_GPF1(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(EXYNOS4_GPF1(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < 8; i++) {
		s3c_gpio_cfgpin(EXYNOS4_GPF2(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(EXYNOS4_GPF2(i), S3C_GPIO_PULL_NONE);
	}

	for (i = 0; i < f3_end; i++) {
		s3c_gpio_cfgpin(EXYNOS4_GPF3(i), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(EXYNOS4_GPF3(i), S3C_GPIO_PULL_NONE);
	}

	/* drive strength to max */
	writel(0xffffffff, S5P_VA_GPIO + 0x18c);
	writel(0xffffffff, S5P_VA_GPIO + 0x1ac);
	writel(0xffffffff, S5P_VA_GPIO + 0x1cc);
	writel(readl(S5P_VA_GPIO + 0x1ec) | 0xffffff,
	    S5P_VA_GPIO + 0x1ec);
        /* MLCD_RST */
	s3c_gpio_cfgpin(EXYNOS4_GPY4(5), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY4(5), S3C_GPIO_PULL_NONE);

	/* LCD_nCS */
	s3c_gpio_cfgpin(EXYNOS4_GPY4(3), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY4(3), S3C_GPIO_PULL_NONE);
	/* LCD_SCLK */
	s3c_gpio_cfgpin(EXYNOS4_GPY3(1), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY3(1), S3C_GPIO_PULL_NONE);
	/* LCD_SDI */
	s3c_gpio_cfgpin(EXYNOS4_GPY3(3), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY3(3), S3C_GPIO_PULL_NONE);

	return 0;
}
static int lcd_power_on(struct lcd_device *ld, int enable)
{
    	struct regulator *regulator;
	
	if (ld == NULL) {
		printk(KERN_ERR "lcd device object is NULL.\n");
		return 0;
	}

	if (enable) {
			regulator = regulator_get(NULL, "vlcd_3.0v");
		if (IS_ERR(regulator))
			return 0;

		regulator_enable(regulator);
		regulator_put(regulator);
	} else {
		regulator = regulator_get(NULL, "vlcd_3.0v");
	
		if (IS_ERR(regulator))
			return 0;

		if (regulator_is_enabled(regulator))
			regulator_force_disable(regulator);
	
		regulator_put(regulator);
	}
	
	return 1;
}

static int reset_lcd(struct lcd_device *ld)
{
	int reset_gpio = -1;
	int err;

	reset_gpio = EXYNOS4_GPY4(5);

	err = gpio_request(reset_gpio, "MLCD_RST");
	if (err) {
		printk(KERN_ERR "failed to request MLCD_RST for "
			"lcd reset control\n");
	return err;
	}

	gpio_request(reset_gpio, "MLCD_RST");

	mdelay(10);
	gpio_direction_output(reset_gpio, 0);
	mdelay(10);
	gpio_direction_output(reset_gpio, 1);

	gpio_free(reset_gpio);

	return 1;
}

static int lcd_gpio_cfg_earlysuspend(struct lcd_device *ld)
{
	int reset_gpio = -1;
	int err;

	reset_gpio = EXYNOS4_GPY4(5);

	err = gpio_request(reset_gpio, "MLCD_RST");
	if (err) {
		printk(KERN_ERR "failed to request MLCD_RST for "
			"lcd reset control\n");
	return err;
	}

	mdelay(10);
	gpio_direction_output(reset_gpio, 0);

	gpio_free(reset_gpio);

	return 0;
}

static int lcd_gpio_cfg_lateresume(struct lcd_device *ld)
{
	/* MLCD_RST */
	s3c_gpio_cfgpin(EXYNOS4_GPY4(5), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY4(5), S3C_GPIO_PULL_NONE);

	/* LCD_nCS */
	s3c_gpio_cfgpin(EXYNOS4_GPY4(3), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY4(3), S3C_GPIO_PULL_NONE);
	/* LCD_SCLK */
	s3c_gpio_cfgpin(EXYNOS4_GPY3(1), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY3(1), S3C_GPIO_PULL_NONE);
	/* LCD_SDI */
	s3c_gpio_cfgpin(EXYNOS4_GPY3(3), S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(EXYNOS4_GPY3(3), S3C_GPIO_PULL_NONE);

	return 0;
}

static struct s3cfb_lcd ld9040_info = {
	.width = 480,
	.height = 800,
	.p_width = 56,
	.p_height = 93,
	.bpp = 24,

	.freq = 60,
	.timing = {
		.h_fp = 16,
		.h_bp = 14,
		.h_sw = 2,
		.v_fp = 10,
		.v_fpe = 1,
		.v_bp = 4,
		.v_bpe = 1,
		.v_sw = 2,
	},
	.polarity = {
	    .rise_vclk = 1,
	    .inv_hsync = 1,
	    .inv_vsync = 1,
	    .inv_vden = 1,
	},
};

static struct lcd_platform_data ld9040_platform_data = {
	.reset			= reset_lcd,
	.power_on		= lcd_power_on,
	.gpio_cfg_earlysuspend	= lcd_gpio_cfg_earlysuspend,
	.gpio_cfg_lateresume	= lcd_gpio_cfg_lateresume,
	/* it indicates whether lcd panel is enabled from u-boot. */
	.lcd_enabled		= 0,
	.reset_delay		= 20,	/* 20ms */
	.power_on_delay		= 20,	/* 20ms */
	.power_off_delay	= 200,	/* 200ms */
	.sleep_in_delay		= 160,
	.pdata			= &u1_panel_data,
};

#define LCD_BUS_NUM     3
#define DISPLAY_CS      EXYNOS4_GPY4(3)
static struct spi_board_info spi_board_info[] __initdata = {
    {
	.max_speed_hz   = 1200000,
	.bus_num        = LCD_BUS_NUM,
	.chip_select    = 0,
	.mode           = SPI_MODE_3,
	.controller_data = (void *)DISPLAY_CS,
    },
};

#define DISPLAY_CLK     EXYNOS4_GPY3(1)
#define DISPLAY_SI      EXYNOS4_GPY3(3)
static struct spi_gpio_platform_data lcd_spi_gpio_data = {
	.sck                    = DISPLAY_CLK,
	.mosi                   = DISPLAY_SI,
	.miso                   = SPI_GPIO_NO_MISO,
	.num_chipselect         = 1,
};

static struct platform_device ld9040_spi_gpio = {
	.name                   = "spi_gpio",
	.id                     = LCD_BUS_NUM,
	.dev                    = {  
	.parent         	= &s3c_device_fb.dev,
	.platform_data  	= &lcd_spi_gpio_data,
    },   
};
static struct s3c_platform_fb fb_platform_data __initdata = {
	.hw_ver                 = 0x70,
	.clk_name               = "fimd",
	.nr_wins                = 5,
#ifdef CONFIG_FB_S5P_DEFAULT_WINDOW
	.default_win            = CONFIG_FB_S5P_DEFAULT_WINDOW,
#else
	.default_win            = 0,
#endif
	.swap                   = FB_SWAP_HWORD | FB_SWAP_WORD,
	.lcd                    = &ld9040_info,
};

static void __init ld9040_fb_init(void)
{
	strcpy(spi_board_info[0].modalias, "ld9040");
	spi_board_info[0].platform_data =
	(void *)&ld9040_platform_data;

	spi_register_board_info(spi_board_info,
		ARRAY_SIZE(spi_board_info));

	lcd_cfg_gpio();
	s3cfb_set_platdata(&fb_platform_data);
}
#endif




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

#ifdef CONFIG_REGULATOR_MAX8998
static struct regulator_consumer_supply ldo3_supply[] = {
	REGULATOR_SUPPLY("ldo3", NULL),
};

static struct regulator_consumer_supply ldo7_supply[] = {
	REGULATOR_SUPPLY("ldo7", NULL),
};

static struct regulator_consumer_supply ldo8_supply[] = {
	REGULATOR_SUPPLY("ldo8", NULL),
};

static struct regulator_consumer_supply ldo17_supply[] = {
	REGULATOR_SUPPLY("ldo17", NULL),
};

static struct regulator_consumer_supply buck1_supply[] = {
	REGULATOR_SUPPLY("buck1", NULL),
};

static struct regulator_consumer_supply buck2_supply[] = {
	REGULATOR_SUPPLY("buck2", NULL),
};

static struct regulator_init_data ldo3_init_data = {
	.constraints	= {
		.name		= "ldo3 range",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo3_supply[0],
};

static struct regulator_init_data ldo7_init_data = {
	.constraints	= {
		.name		= "ldo7 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.uV		= 1200000,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo7_supply[0],
};

static struct regulator_init_data ldo8_init_data = {
	.constraints	= {
		.name		= "ldo8 range",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo8_supply[0],
};

static struct regulator_init_data ldo17_init_data = {
	.constraints	= {
		.name		= "ldo17 range",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo17_supply[0],
};

static struct regulator_init_data buck1_init_data = {
	.constraints	= {
		.name		= "buck1 range",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck1_supply[0],
};

static struct regulator_init_data buck2_init_data = {
	.constraints	= {
		.name		= "buck2 range",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck2_supply[0],
};

static struct max8998_regulator_data max8998_regulators[] = {
	{ MAX8998_LDO3, &ldo3_init_data, },
	{ MAX8998_LDO7, &ldo7_init_data, },
	{ MAX8998_LDO8, &ldo8_init_data, },
	{ MAX8998_LDO17, &ldo17_init_data, },
	{ MAX8998_BUCK1, &buck1_init_data, },
	{ MAX8998_BUCK2, &buck2_init_data, },
};

static struct max8998_power_data max8998_power = {
	.batt_detect = 1,
};

static struct max8998_platform_data s5pv310_max8998_info = {
	.num_regulators = ARRAY_SIZE(max8998_regulators),
	.regulators	= max8998_regulators,
	.irq_base	= IRQ_BOARD_START,
	.buck1_max_voltage1 = 1100000,
	.buck1_max_voltage2 = 1100000,
	.buck2_max_voltage = 1100000,
	.buck1_set1 = GPIO_BUCK1_EN_A,
	.buck1_set2 = GPIO_BUCK1_EN_B,
	.buck2_set3 = GPIO_BUCK2_EN,
	.power = &max8998_power,
};
#endif /* CONFIG_REGULATOR_MAX8998 */

#ifdef CONFIG_REGULATOR_MAX8997
static struct regulator_consumer_supply ldo3_supply[] = {
	REGULATOR_SUPPLY("vusb_1.1v", "usb_otg"),
	REGULATOR_SUPPLY("vmipi_1.1v", "m5mo"),
	REGULATOR_SUPPLY("vmipi_1.1v", NULL),
};

static struct regulator_consumer_supply ldo4_supply[] = {
	REGULATOR_SUPPLY("vmipi_1.8v", NULL),
};

static struct regulator_consumer_supply ldo5_supply[] = {
	REGULATOR_SUPPLY("vhsic", NULL),
};

static struct regulator_consumer_supply ldo7_supply[] = {
	REGULATOR_SUPPLY("cam_isp", NULL),
};

static struct regulator_consumer_supply ldo8_supply[] = {
	REGULATOR_SUPPLY("vusb_3.3v", NULL),
};

static struct regulator_consumer_supply ldo11_supply[] = {
	REGULATOR_SUPPLY("touch", NULL),
};

static struct regulator_consumer_supply ldo12_supply[] = {
#if defined (CONFIG_MACH_U1)
    	REGULATOR_SUPPLY("vt_cam_1.8v", NULL),
#else
	REGULATOR_SUPPLY("cam_sensor_core", NULL),
#endif
};

static struct regulator_consumer_supply ldo13_supply[] = {
	REGULATOR_SUPPLY("vlcd_3.0v", NULL),
};

static struct regulator_consumer_supply ldo14_supply[] = {
	REGULATOR_SUPPLY("vmotor", NULL),
};

static struct regulator_consumer_supply ldo15_supply[] = {
	REGULATOR_SUPPLY("vled", NULL),
};

static struct regulator_consumer_supply ldo16_supply[] = {
	REGULATOR_SUPPLY("cam_sensor_io", NULL),
};

static struct regulator_consumer_supply ldo17_supply[] = {
	REGULATOR_SUPPLY("cam_af", NULL),
};

static struct regulator_consumer_supply ldo18_supply[] = {
	REGULATOR_SUPPLY("touch_led", NULL),
};

static struct regulator_consumer_supply buck1_supply[] = {
	REGULATOR_SUPPLY("vdd_arm", NULL),
};

static struct regulator_consumer_supply buck2_supply[] = {
	REGULATOR_SUPPLY("vdd_int", NULL),
};

static struct regulator_consumer_supply buck3_supply[] = {
	REGULATOR_SUPPLY("vdd_g3d", NULL),
};

static struct regulator_consumer_supply buck4_supply[] = {
	REGULATOR_SUPPLY("cam_isp_core", NULL),
};

static struct regulator_consumer_supply safeout1_supply[] = {
	REGULATOR_SUPPLY("safeout1", NULL),
};

static struct regulator_consumer_supply safeout2_supply[] = {
	REGULATOR_SUPPLY("safeout2", NULL),
};

static struct regulator_consumer_supply led_flash_supply[] = {
	REGULATOR_SUPPLY("led_flash", NULL),
};

static struct regulator_consumer_supply led_movie_supply[] = {
	REGULATOR_SUPPLY("led_movie", NULL),
};

#define REGULATOR_INIT(_ldo, _name, _min_uV, _max_uV, _always_on, _ops_mask) \
	static struct regulator_init_data _ldo##_init_data = {		\
		.constraints = {					\
			.name	= _name,				\
			.min_uV = _min_uV,				\
			.max_uV = _max_uV,				\
			.always_on	= _always_on,			\
			.boot_on	= _always_on,			\
			.apply_uV	= 1,				\
			.valid_ops_mask = _ops_mask,			\
			.state_mem	= {				\
				.disabled	= 1,			\
			}						\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(_ldo##_supply),	\
		.consumer_supplies = &_ldo##_supply[0],			\
	};

REGULATOR_INIT(ldo3, "VUSB_1.1V", 1100000, 1100000, 0,
		REGULATOR_CHANGE_STATUS);
REGULATOR_INIT(ldo4, "VMIPI_1.8V", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS);
REGULATOR_INIT(ldo5, "VHSIC_1.2V", 1200000, 1200000, 0,
		REGULATOR_CHANGE_STATUS);
REGULATOR_INIT(ldo7, "CAM_ISP_1.8V", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS);
REGULATOR_INIT(ldo8, "VUSB_3.3V", 3300000, 3300000, 0,
		REGULATOR_CHANGE_STATUS);
REGULATOR_INIT(ldo11, "TOUCH_2.8V", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS);
#if defined (CONFIG_MACH_U1_BD)
REGULATOR_INIT(ldo12, "CAM_SENSOR_CORE_1.2V", 1200000, 1200000, 0,
		REGULATOR_CHANGE_STATUS);
#else
REGULATOR_INIT(ldo12, "VT_CAM_1.8V", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS);
#endif
#ifdef CONFIG_FB_S5P_S6E8AA0
REGULATOR_INIT(ldo13, "VCC_3.0V_LCD", 3300000, 3300000, 1,
                REGULATOR_CHANGE_STATUS);
#else
REGULATOR_INIT(ldo13, "VCC_3.0V_LCD", 3000000, 3000000, 1,
		REGULATOR_CHANGE_STATUS);
#endif
REGULATOR_INIT(ldo14, "VCC_3.0V_MOTOR", 3000000, 3000000, 0,
		REGULATOR_CHANGE_STATUS);
REGULATOR_INIT(ldo15, "LED_A_2.8V", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS);
REGULATOR_INIT(ldo16, "CAM_SENSOR_IO_1.8V", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS);
REGULATOR_INIT(ldo17, "CAM_AF_2.8V", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS);
REGULATOR_INIT(ldo18, "TOUCH_LED_3.2V", 3200000, 3200000, 0,
		REGULATOR_CHANGE_STATUS);

static struct regulator_init_data buck1_init_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		= 650000,
		.max_uV		= 2225000,
		.always_on	= 1,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.uV		= 1250000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck1_supply[0],
};

static struct regulator_init_data buck2_init_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		= 650000,
		.max_uV		= 2225000,
		.always_on	= 1,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck2_supply[0],
};

static struct regulator_init_data buck3_init_data = {
	.constraints	= {
		.name		= "G3D_1.1V",
		.min_uV		= 900000,
		.max_uV		= 1200000,
		.always_on	= 0,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck3_supply[0],
};

static struct regulator_init_data buck4_init_data = {
	.constraints	= {
		.name		= "CAM_ISP_CORE_1.2V",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck4_supply[0],
};

static struct regulator_init_data buck5_init_data = {
	.constraints	= {
		.name		= "VMEM_1.2V",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.uV		= 1200000,
			.mode		= REGULATOR_MODE_NORMAL,
			.enabled = 1,
		},
	},
};

static struct regulator_init_data safeout1_init_data = {
        .constraints    = {
                .name           = "safeout1 range",
                .valid_ops_mask = REGULATOR_CHANGE_STATUS,
                .state_mem      = {
                        .disabled = 1,
                },
        },
        .num_consumer_supplies  = ARRAY_SIZE(safeout1_supply),
        .consumer_supplies      = safeout1_supply,
};

static struct regulator_init_data safeout2_init_data = {
        .constraints    = {
                .name           = "safeout2 range",
                .valid_ops_mask = REGULATOR_CHANGE_STATUS,
                .state_mem      = {
                        .enabled = 1,
                },
        },
        .num_consumer_supplies  = ARRAY_SIZE(safeout2_supply),
        .consumer_supplies      = safeout2_supply,
};

static struct regulator_init_data led_flash_init_data = {
	.constraints = {
		.name	= "FLASH_CUR",
		.min_uA = 750000,
		.max_uA = 750000,
		.valid_ops_mask	= REGULATOR_CHANGE_CURRENT |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &led_flash_supply[0],
};

static struct regulator_init_data led_movie_init_data = {
	.constraints = {
		.name	= "MOVIE_CUR",
		.min_uA = 250000,
		.max_uA = 250000,
		.valid_ops_mask	= REGULATOR_CHANGE_CURRENT |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &led_movie_supply[0],
};

static struct max8997_regulator_data max8997_regulators[] = {
	{ MAX8997_LDO3,  &ldo3_init_data, },
	{ MAX8997_LDO4,  &ldo4_init_data, },
	{ MAX8997_LDO5,  &ldo5_init_data, },
	{ MAX8997_LDO7,  &ldo7_init_data, },
	{ MAX8997_LDO8,  &ldo8_init_data, },
	{ MAX8997_LDO11, &ldo11_init_data, },
	{ MAX8997_LDO12, &ldo12_init_data, },
	{ MAX8997_LDO13, &ldo13_init_data, },
	{ MAX8997_LDO14, &ldo14_init_data, },
	{ MAX8997_LDO15, &ldo15_init_data, },
	{ MAX8997_LDO16, &ldo16_init_data, },
	{ MAX8997_LDO17, &ldo17_init_data, },
	{ MAX8997_LDO18, &ldo18_init_data, },
	{ MAX8997_BUCK1, &buck1_init_data, },
	{ MAX8997_BUCK2, &buck2_init_data, },
	{ MAX8997_BUCK3, &buck3_init_data, },
	{ MAX8997_BUCK4, &buck4_init_data, },
	{ MAX8997_BUCK5, &buck5_init_data, },
	{ MAX8997_ESAFEOUT1, &safeout1_init_data, },
	{ MAX8997_ESAFEOUT2, &safeout2_init_data, },
	//{ MAX8997_FLASH_CUR, &led_flash_init_data, },
	//{ MAX8997_MOVIE_CUR, &led_movie_init_data, },
};

static struct max8997_power_data max8997_power = {
	.batt_detect = 1,
};

static struct max8997_platform_data s5pv310_max8997_info = {
	.num_regulators = ARRAY_SIZE(max8997_regulators),
	.regulators	= &max8997_regulators[0],
	.irq_base	= IRQ_BOARD_START,
	.buck1_max_voltage1 = 1200000,
	.buck1_max_voltage2 = 1100000,
	.buck1_max_voltage3 = 1000000,
	.buck1_max_voltage4 = 950000,
	.buck2_max_voltage1 = 1100000,
	.buck2_max_voltage2 = 1000000,
	.buck2_max_voltage3 = 950000,
	.buck_set1 = GPIO_BUCK1_EN_A,
	.buck_set2 = GPIO_BUCK1_EN_B,
	.buck_set3 = GPIO_BUCK2_EN,
	.flash_cntl_val = 0x5F,		/* Flash safety timer duration: 800msec,
					   Maximum timer mode */
	.power = &max8997_power,
};
#endif /* CONFIG_REGULATOR_MAX8997 */


#define SYSTEM_REV_SND 0x09

#ifdef CONFIG_SND_SOC_MC1N2
static DEFINE_SPINLOCK(mic_bias_lock);
static bool mc1n2_mainmic_bias;
static bool mc1n2_submic_bias;

static void set_shared_mic_bias(void)
{
	if (system_rev >= 0x03)
		gpio_set_value(GPIO_MIC_BIAS_EN, mc1n2_mainmic_bias || mc1n2_submic_bias);
	else
		gpio_set_value(GPIO_EAR_MIC_BIAS_EN, mc1n2_mainmic_bias || mc1n2_submic_bias);
}

void sec_set_sub_mic_bias(bool on)
{
#ifdef CONFIG_SND_SOC_USE_EXTERNAL_MIC_BIAS
	if (system_rev < SYSTEM_REV_SND) {
		unsigned long flags;
		spin_lock_irqsave(&mic_bias_lock, flags);
		mc1n2_submic_bias = on;
		set_shared_mic_bias();
		spin_unlock_irqrestore(&mic_bias_lock, flags);
	} else
		gpio_set_value(GPIO_SUB_MIC_BIAS_EN, on);

#endif
}

void sec_set_main_mic_bias(bool on)
{
#ifdef CONFIG_SND_SOC_USE_EXTERNAL_MIC_BIAS
	if (system_rev < SYSTEM_REV_SND) {
		unsigned long flags;
		spin_lock_irqsave(&mic_bias_lock, flags);
		mc1n2_mainmic_bias = on;
		set_shared_mic_bias();
		spin_unlock_irqrestore(&mic_bias_lock, flags);
	} else
		gpio_set_value(GPIO_MIC_BIAS_EN, on);
#endif
}

void sec_set_ldo1_constraints(int disabled)
{
#if 0	/* later */
	/* VDD33_ADC */
	ldo1_init_data.constraints.state_mem.disabled = disabled;
	ldo1_init_data.constraints.state_mem.enabled = !disabled;
#endif
}

static struct mc1n2_platform_data mc1n2_pdata = {
	.set_main_mic_bias = sec_set_main_mic_bias,
	.set_sub_mic_bias = sec_set_sub_mic_bias,
	.set_adc_power_contraints = sec_set_ldo1_constraints,
};

static void u1_sound_init(void)
{
#ifdef CONFIG_SND_SOC_USE_EXTERNAL_MIC_BIAS
	int err;

	err = gpio_request(GPIO_MIC_BIAS_EN, "GPE1");
	if (err) {
		pr_err(KERN_ERR "MIC_BIAS_EN GPIO set error!\n");
		return;
	}
	gpio_direction_output(GPIO_MIC_BIAS_EN, 1);
	gpio_set_value(GPIO_MIC_BIAS_EN, 0);
	gpio_free(GPIO_MIC_BIAS_EN);

	err = gpio_request(GPIO_EAR_MIC_BIAS_EN, "GPE2");
	if (err) {
		pr_err(KERN_ERR "EAR_MIC_BIAS_EN GPIO set error!\n");
		return;
	}
	gpio_direction_output(GPIO_EAR_MIC_BIAS_EN, 1);
	gpio_set_value(GPIO_EAR_MIC_BIAS_EN, 0);
	gpio_free(GPIO_EAR_MIC_BIAS_EN);

	if (system_rev >= SYSTEM_REV_SND) {
		err = gpio_request(GPIO_SUB_MIC_BIAS_EN, "submic_bias");
		if (err) {
			pr_err(KERN_ERR "SUB_MIC_BIAS_EN GPIO set error!\n");
			return;
		}
		gpio_direction_output(GPIO_SUB_MIC_BIAS_EN, 0);
		gpio_free(GPIO_SUB_MIC_BIAS_EN);
	}
#endif /* #ifdef CONFIG_SND_SOC_USE_EXTERNAL_MIC_BIAS */
}
#endif

#ifdef CONFIG_BATTERY_SAMSUNG
static struct platform_device samsung_device_battery = {
	.name	= "samsung-fake-battery",
	.id	= -1,
};
#endif

struct gpio_keys_button u1_buttons[] = {
    { .code=115, .gpio=GPIO_VOL_UP,   .active_low=1, }, /* vol up */
    { .code=114, .gpio=GPIO_VOL_DOWN, .active_low=1, }, /* vol down */
#if defined(GPIO_CAM_HALF)
    { .code=113, .gpio=GPIO_CAM_HALF, .active_low=1, }, /* camera half */
#endif
#if defined(GPIO_CAM_FULL)
    { .code=112, .gpio=GPIO_CAM_FULL, .active_low=1, }, /* camera full */
#endif
    { .code=116, .gpio=GPIO_nPOWER,   .active_low=1, .wakeup = 1}, /* power key */
    { .code=172, .gpio=GPIO_OK_KEY,   .active_low=1, }, /* ok key */
};

struct gpio_keys_platform_data u1_keypad_platform_data = {
    u1_buttons,
    ARRAY_SIZE(u1_buttons),
};

struct platform_device u1_keypad = {
    .name   = "gpio-keys",
    .dev.platform_data = &u1_keypad_platform_data,
};

#ifdef CONFIG_SEC_DEV_JACK
static struct sec_jack_port sec_jack_port[] = {
    {
	{ /* HEADSET detect info */
	    .eint           = IRQ_EINT(26),
	    .gpio           = GPIO_DET_35,
	    .gpio_af        = GPIO_DET_35_AF,
	    .low_active     = 0
	},
	{ /* SEND/END info */
	    .eint           = IRQ_EINT(30),
	    .gpio           = GPIO_EAR_SEND_END,
	    .gpio_af        = GPIO_EAR_SEND_END_AF,
	    .low_active     = 1
	}
    }
};

static struct sec_jack_platform_data sec_jack_data = {
    .port           = sec_jack_port,
    .nheadsets      = ARRAY_SIZE(sec_jack_port),
};

static struct platform_device sec_device_jack = {
    .name           = "sec_jack",
    .id		    = -1,
    .dev            = { .platform_data  = &sec_jack_data },
};
#endif

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT224_U1

static void mxt224_power_on(void)
{
	s3c_gpio_cfgpin(GPIO_TSP_LDO_ON, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_LDO_ON, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_TSP_LDO_ON, 1);
	mdelay(70);
	s3c_gpio_setpull(GPIO_TSP_INT, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_TSP_INT, S3C_GPIO_SFN(0xf));
	mdelay(40);
	/* printk("mxt224_power_on is finished\n"); */
}

static void mxt224_power_off(void)
{
	s3c_gpio_cfgpin(GPIO_TSP_INT, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_TSP_INT, S3C_GPIO_PULL_DOWN);

	s3c_gpio_cfgpin(GPIO_TSP_LDO_ON, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_LDO_ON, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_TSP_LDO_ON, 0);
	/* printk("mxt224_power_off is finished\n"); */
}
#define MXT224_MAX_MT_FINGERS 5

#ifdef CONFIG_TOUCHSCREEN_MXT768E
static u8 t7_config[] = {GEN_POWERCONFIG_T7,
				64, 255, 20};
static u8 t8_config[] = {GEN_ACQUISITIONCONFIG_T8,
				64, 0, 20, 20, 0, 0, 20, 0, 50, 25};
static u8 t9_config[] = {TOUCH_MULTITOUCHSCREEN_T9,
				139, 0, 0, 16, 26, 0, 208, 50, 2, 6, 0, 5, 1,
				0, MXT224_MAX_MT_FINGERS, 10, 10, 5, 255, 3,
				255, 3, 0, 0, 0, 0, 136, 60, 136, 40, 40, 15, 0, 0};

static u8 t15_config[] = {TOUCH_KEYARRAY_T15,
				1, 16, 26, 1, 6, 0, 64, 255, 3, 0, 0};

static u8 t18_config[] = {SPT_COMCONFIG_T18,
				0, 0};

static u8 t40_config[] = {PROCI_GRIPSUPPRESSION_T40,
				0, 0, 0, 0, 0};

static u8 t42_config[] = {PROCI_TOUCHSUPPRESSION_T42,
				0, 0, 0, 0, 0, 0, 0, 0};

static u8 t43_config[] = {SPT_DIGITIZER_T43,
				0, 0, 0, 0};

static u8 t48_config[] = {PROCG_NOISESUPPRESSION_T48,
				1, 0, 65, 0, 12, 24, 36, 48, 8, 16, 11, 40, 0, 0,
				0, 0, 0};


static u8 t46_config[] = {SPT_CTECONFIG_T46,
				0, 0, 8, 32, 0, 0, 0, 0};
static u8 end_config[] = {RESERVED_T255};

static const u8 *mxt224_config[] = {
	t7_config,
	t8_config,
	t9_config,
	t15_config,
	t18_config,
	t40_config,
	t42_config,
	t43_config,
	t46_config,
	t48_config,
	end_config,
};
#else
static u8 t7_config[] = {GEN_POWERCONFIG_T7,
	32, 255, 50};
static u8 t8_config[] = {GEN_ACQUISITIONCONFIG_T8,
	10, 0, 5, 0, 0, 0, 9, 23};
static u8 t9_config[] = {TOUCH_MULTITOUCHSCREEN_T9,
	131, 0, 0, 19, 11, 0, 16, 40, 2, 1, 0, 1, 1,
	14, MXT224_MAX_MT_FINGERS, 5, 10, 10, 31, 3,
	223, 1, 0, 0, 0, 236, 143, 40, 143, 80, 18};
static u8 t18_config[] = {SPT_COMCONFIG_T18,
	0, 1};
static u8 t20_config[] = {PROCI_GRIPFACESUPPRESSION_T20,
	7, 0, 0, 0, 0, 0, 0, 80, 40, 4, 15, 10};
static u8 t22_config[] = {PROCG_NOISESUPPRESSION_T22,
	5, 0, 0, 0, 0, 0, 0, 3, 23, 0, 0, 5, 15, 25,
	35, 45, 3};
static u8 t28_config[] = {SPT_CTECONFIG_T28,
	1, 0, 3, 16, 63, 10};
static u8 end_config[] = {RESERVED_T255};

static const u8 *mxt224_config[] = {
	t7_config,
	t8_config,
	t9_config,
	t18_config,
	t20_config,
	t22_config,
	t28_config,
	end_config,
};
#endif

static struct mxt224_platform_data mxt224_data = {
	.max_finger_touches = MXT224_MAX_MT_FINGERS,
	.gpio_read_done = GPIO_TSP_INT,
	.config = mxt224_config,
#ifdef CONFIG_TOUCHSCREEN_MXT768E
	.min_x = 20,
	.max_x = 1000,
#else
	.min_x = 0,
	.max_x = 470,
#endif
#ifdef CONFIG_TOUCHSCREEN_MXT768E
	.min_y = 20,
	.max_y = 1010,
#else
	.min_y = 20,
	.max_y = 790,
#endif
	.min_z = 0,
	.max_z = 255,
	.min_w = 0,
	.max_w = 30,
	.power_on = mxt224_power_on,
	.power_off = mxt224_power_off,
};

#endif


#ifdef CONFIG_I2C_S3C2410
/* I2C0 */
static struct i2c_board_info i2c_devs0[] __initdata = {
            { I2C_BOARD_INFO("24c128", 0x50), },     /* Samsung S524AD0XD1 */
	    { I2C_BOARD_INFO("24c128", 0x52), },     /* Samsung S524AD0XD1 */
};
#ifdef CONFIG_S3C_DEV_I2C1
/* I2C1 */
static struct i2c_board_info i2c_devs1[] __initdata = {

};
#endif
#ifdef CONFIG_S3C_DEV_I2C2
/* I2C2 */
static struct i2c_board_info i2c_devs2[] __initdata = {
};
#endif
#ifdef CONFIG_S3C_DEV_I2C3
/* I2C3 */
static struct i2c_board_info i2c_devs3[] __initdata = {
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT224_U1
    {
	I2C_BOARD_INFO(MXT224_DEV_NAME, 0x4a),
	.platform_data  = &mxt224_data,
    },
#endif
};
#endif
#ifdef CONFIG_S3C_DEV_I2C4
/* I2C4 */
static struct i2c_board_info i2c_devs4[] __initdata = {
};
#endif

#ifdef CONFIG_S3C_DEV_I2C5
/* I2C5 */
static struct i2c_board_info i2c_devs5[] __initdata = {
#ifdef CONFIG_MFD_MAX8998
    {
	I2C_BOARD_INFO("lp3974", 0x66),
	.platform_data  = &s5pv310_max8998_info,
    },
#endif
#ifdef CONFIG_MFD_MAX8997
    {
	I2C_BOARD_INFO("max8997", (0xcc >> 1)),
	.platform_data  = &s5pv310_max8997_info,
    },
#endif
};
#endif
#ifdef CONFIG_S3C_DEV_I2C6
/* I2C6 */
static struct i2c_board_info i2c_devs6[] __initdata = {
#ifdef CONFIG_SND_SOC_MC1N2
    {
	I2C_BOARD_INFO("mc1n2", 0x3a),          /* MC1N2 */
	.platform_data = &mc1n2_pdata,
    },
#endif
};
#endif
#ifdef CONFIG_S3C_DEV_I2C7
/* I2C7 */
static struct i2c_board_info i2c_devs7[] __initdata = {
};
#endif
#ifdef CONFIG_S3C_DEV_I2C8_EMUL
static struct i2c_gpio_platform_data gpio_i2c_data8 = {
	.sda_pin = GPIO_3_TOUCH_SDA,
	.scl_pin = GPIO_3_TOUCH_SCL,
};

struct platform_device s3c_device_i2c8 = {
	.name = "i2c-gpio",
	.id = 8,
	.dev.platform_data = &gpio_i2c_data8,
};

/* I2C8 */
static struct i2c_board_info i2c_devs8_emul[] = {
	{
		I2C_BOARD_INFO("melfas_touchkey", 0x20),
	},
};
#endif

#ifdef CONFIG_S3C_DEV_I2C9_EMUL
static struct i2c_gpio_platform_data gpio_i2c_data9 __initdata = {
	.sda_pin = GPIO_FUEL_SDA,
	.scl_pin = GPIO_FUEL_SCL,
};

struct platform_device s3c_device_i2c9 = {
	.name = "i2c-gpio",
	.id = 9,
	.dev.platform_data = &gpio_i2c_data9,
};

/* I2C9 */
static struct i2c_board_info i2c_devs9_emul[] __initdata = {
	{
		I2C_BOARD_INFO("max17040", 0x36),
	},
};
#endif
#ifdef CONFIG_S3C_DEV_I2C10_EMUL
static struct i2c_gpio_platform_data gpio_i2c_data10 __initdata = {
	.sda_pin = GPIO_USB_SDA,
	.scl_pin = GPIO_USB_SCL,
};

struct platform_device s3c_device_i2c10 = {
	.name = "i2c-gpio",
	.id = 10,
	.dev.platform_data = &gpio_i2c_data10,
};

/* I2C10 */
static struct fsa9480_platform_data fsa9480_info = {
};

static struct i2c_board_info i2c_devs10_emul[] __initdata = {
    {
	I2C_BOARD_INFO("fsa9480", 0x25),
	.platform_data  = &fsa9480_info,
    },
};
#endif
#endif

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

#ifdef CONFIG_USB_OHCI_S5P
static struct s5p_ohci_platdata smdkc210_ohci_pdata;

static void __init smdkc210_ohci_init(void)
{
	struct s5p_ohci_platdata *pdata = &smdkc210_ohci_pdata;

	s5p_ohci_set_platdata(pdata);
}
#endif

/* USB GADGET */
#ifdef CONFIG_USB_GADGET
static struct s5p_usbgadget_platdata smdkc210_usbgadget_pdata;

static void __init smdkc210_usbgadget_init(void)
{
	struct s5p_usbgadget_platdata *pdata = &smdkc210_usbgadget_pdata;

	s5p_usbgadget_set_platdata(pdata);
}
#endif

static struct platform_device *smdkc210_devices[] __initdata = {
#ifdef CONFIG_ANDROID_PMEM
	&pmem_device,
	&pmem_gpu1_device,
#endif

#ifdef CONFIG_I2C_S3C2410
	&s3c_device_i2c0,
#if defined(CONFIG_S3C_DEV_I2C1)
	&s3c_device_i2c1,
#endif
#if defined(CONFIG_S3C_DEV_I2C2)
	&s3c_device_i2c2,
#endif	
#if defined(CONFIG_S3C_DEV_I2C3)
	&s3c_device_i2c3,
#endif
#if defined(CONFIG_S3C_DEV_I2C4)
	&s3c_device_i2c4,
#endif
#if defined(CONFIG_S3C_DEV_I2C5)
	&s3c_device_i2c5,
#endif
#if defined(CONFIG_S3C_DEV_I2C6)
	&s3c_device_i2c6,
#endif
#if defined(CONFIG_S3C_DEV_I2C7)
	&s3c_device_i2c7,
#endif
#if defined(CONFIG_S3C_DEV_I2C8_EMUL)
	&s3c_device_i2c8,
#endif
#if defined(CONFIG_S3C_DEV_I2C9_EMUL)
	&s3c_device_i2c9,
#endif
#if defined(CONFIG_S3C_DEV_I2C10_EMUL)
	&s3c_device_i2c10,
#endif
#endif

#ifdef CONFIG_S5P_DEV_MSHC
	&s3c_device_mshci,
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


#ifdef CONFIG_TOUCHSCREEN_S3C2410
#ifdef CONFIG_S3C_DEV_ADC
	&s3c_device_ts,
#elif CONFIG_S3C_DEV_ADC1
	&s3c_device_ts1,
#endif
#endif
	&u1_keypad,
	&s3c_device_rtc,
	&s3c_device_wdt,
	&exynos4_device_ac97,
	&exynos4_device_i2s0,
	&exynos4_device_pcm0,
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
#ifdef CONFIG_FB_S5P_LD9040
	&ld9040_spi_gpio,
#endif
#endif
#ifdef CONFIG_VIDEO_TVOUT
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,
#endif
	&smdkc210_smsc911x,
#ifdef CONFIG_VIDEO_FIMC
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
	&s3c_device_fimc3,
#ifdef CONFIG_VIDEO_FIMC_MIPI
	&s3c_device_csis0,
	&s3c_device_csis1,
#endif
#endif
#ifdef CONFIG_VIDEO_MFC5X
	&s5p_device_mfc,
#endif
#ifdef CONFIG_VIDEO_FIMG2D
	&s5p_device_fimg2d,
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
		"s3c-fimc.0=fimc0;s3c-fimc.1=fimc1;s3c-fimc.2=fimc2;s3c-fimc.3=fimc3;"
		"exynos4210-fimc.0=fimc0;exynos4210-fimc.1=fimc1;exynos4210-fimc.2=fimc2;exynos4210-fimc3=fimc3;"
#if !defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
		"s5p-mfc=mfc,mfc0,mfc1;"
#else
		"mfc=mfc,mfc0,mfc1;"
		"s5p-mfc/f=fw;"
		"s5p-mfc/a=b1;"
		"s5p-mfc/b=b2;"
#endif
		"s5p-rp=srp;"
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
static void __init universal_tsp_init(void)
{
    int gpio;

    /* TSP_LDO_ON: XMDMADDR_11 */
    gpio = GPIO_TSP_LDO_ON;
    gpio_request(gpio, "TSP_LDO_ON");
    gpio_direction_output(gpio, 1);
    gpio_export(gpio, 0);

    /* TSP_INT: XMDMADDR_7 */
    gpio = GPIO_TSP_INT;
    gpio_request(gpio, "TSP_INT");
    s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(0xf));
    s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
    i2c_devs3[0].irq = gpio_to_irq(gpio);

    printk("%s touch : %d\n",__func__,i2c_devs3[0].irq);
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
#ifdef CONFIG_I2C_S3C2410
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

#ifdef CONFIG_S3C_DEV_I2C1
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));
#endif
#ifdef CONFIG_S3C_DEV_I2C2
	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, i2c_devs2, ARRAY_SIZE(i2c_devs2));
#endif
#ifdef CONFIG_S3C_DEV_I2C3
	universal_tsp_init();
	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, i2c_devs3, ARRAY_SIZE(i2c_devs3));
#endif
#ifdef CONFIG_S3C_DEV_I2C4
	s3c_i2c4_set_platdata(NULL);
	i2c_register_board_info(4, i2c_devs4, ARRAY_SIZE(i2c_devs4));
#endif
#ifdef CONFIG_S3C_DEV_I2C5
	s3c_i2c5_set_platdata(NULL);
	s3c_gpio_cfgpin(GPIO_PMIC_IRQ, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(GPIO_PMIC_IRQ, S3C_GPIO_PULL_UP);
	i2c_devs5[0].irq = gpio_to_irq(GPIO_PMIC_IRQ);
	i2c_register_board_info(5, i2c_devs5, ARRAY_SIZE(i2c_devs5));
#endif
#ifdef CONFIG_S3C_DEV_I2C6
	s3c_i2c6_set_platdata(NULL);
	i2c_register_board_info(6, i2c_devs6, ARRAY_SIZE(i2c_devs6));
#endif
#ifdef CONFIG_S3C_DEV_I2C7
	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));
#endif
#ifdef CONFIG_S3C_DEV_I2C8_EMUL
	gpio_request(GPIO_3_TOUCH_INT,"touch-led");
        i2c_register_board_info(8, i2c_devs8_emul, ARRAY_SIZE(i2c_devs8_emul));
#endif
#ifdef CONFIG_S3C_DEV_I2C9_EMUL
	i2c_register_board_info(9, i2c_devs9_emul, ARRAY_SIZE(i2c_devs9_emul));
#endif
#ifdef CONFIG_S3C_DEV_I2C10_EMUL
	i2c_register_board_info(10, i2c_devs10_emul, ARRAY_SIZE(i2c_devs10_emul));
#endif
#endif
	smdkc210_smsc911x_init();

	/* 400 kHz for initialization of MMC Card  */
	__raw_writel((__raw_readl(S5P_CLKDIV_FSYS3) & 0xfffffff0)
		     | 0x9, S5P_CLKDIV_FSYS3);
	__raw_writel((__raw_readl(S5P_CLKDIV_FSYS2) & 0xfff0fff0)
		     | 0x80008, S5P_CLKDIV_FSYS2);
	__raw_writel((__raw_readl(S5P_CLKDIV_FSYS1) & 0xfff0fff0)
		     | 0x90009, S5P_CLKDIV_FSYS1);


#ifdef CONFIG_S3C_DEV_HSMMC
	s3c_sdhci0_set_platdata(&exynos4_hsmmc0_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	s3c_sdhci1_set_platdata(&exynos4_hsmmc1_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s3c_sdhci2_set_platdata(&exynos4_hsmmc2_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s3c_sdhci3_set_platdata(&exynos4_hsmmc3_pdata);
#endif

#ifdef CONFIG_S5P_DEV_MSHC
	s3c_mshci_set_platdata(&exynos4_mshc_pdata);
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
	/* fimc */
	s3c_fimc0_set_platdata(&fimc_plat);
	s3c_fimc1_set_platdata(&fimc_plat);
	s3c_fimc2_set_platdata(&fimc_plat);
	s3c_fimc3_set_platdata(&fimc_plat);
#ifdef CONFIG_VIDEO_FIMC_MIPI
	s3c_csis0_set_platdata(NULL);
	s3c_csis1_set_platdata(NULL);
#endif
#endif

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
#ifdef CONFIG_USB_OHCI_S5P
	smdkc210_ohci_init();
#endif
#ifdef CONFIG_USB_GADGET
	smdkc210_usbgadget_init();
#endif

#ifdef CONFIG_FB_S5P_LD9040
	ld9040_fb_init();
#endif

#ifdef CONFIG_SND_SOC_MC1N2
	u1_sound_init();
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
