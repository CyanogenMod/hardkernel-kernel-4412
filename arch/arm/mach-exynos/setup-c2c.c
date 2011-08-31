/* linux/arch/arm/mach-exynos/setup-c2c.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS4212 - Helper functions for setting up C2C device(s) GPIO
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/gpio.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <plat/gpio-cfg.h>

#define ETC8DRV (S5P_VA_GPIO4 + 0xAC)

void exynos4_c2c_set_cprst(void)
{
	/* TODO */
	gpio_set_value(EXYNOS4_GPY4(6), 0);
	gpio_set_value(EXYNOS4_GPL2(5), 0);
	gpio_set_value(EXYNOS4_GPX3(2), 0);
	mdelay(100);
	gpio_set_value(EXYNOS4_GPX3(2), 1);
	mdelay(50);
	/* delay for 50ms */
	gpio_set_value(EXYNOS4_GPY4(6), 1);
	gpio_set_value(EXYNOS4_GPL2(5), 1);
}

void exynos4_c2c_clear_cprst(void)
{
	/* TODO */
}

void exynos4_c2c_cfg_gpio(void)
{
	int i;

	/* Set Rx GPIO */
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPV0(0), 8, S3C_GPIO_SFN(2));
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPV1(0), 8, S3C_GPIO_SFN(2));
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPV2(0), 8, S3C_GPIO_SFN(2));
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPV3(0), 8, S3C_GPIO_SFN(2));
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPV4(0), 2, S3C_GPIO_SFN(2));

	for (i = 0; i < 8; i++) {
		s5p_gpio_set_drvstr(EXYNOS4212_GPV0(i), S5P_GPIO_DRVSTR_LV1);
		s5p_gpio_set_drvstr(EXYNOS4212_GPV1(i), S5P_GPIO_DRVSTR_LV1);
		s5p_gpio_set_drvstr(EXYNOS4212_GPV2(i), S5P_GPIO_DRVSTR_LV3);
		s5p_gpio_set_drvstr(EXYNOS4212_GPV3(i), S5P_GPIO_DRVSTR_LV3);
	}

	//s3c_gpio_cfgrange_nopull(EXYNOS4_GPX3(2), 1, S3C_GPIO_SFN(1));
	s3c_gpio_cfgpin(EXYNOS4_GPX3(2), S3C_GPIO_OUTPUT);
	s3c_gpio_cfgpin(EXYNOS4_GPY4(6), S3C_GPIO_OUTPUT);
	s3c_gpio_cfgpin(EXYNOS4_GPL2(5), S3C_GPIO_OUTPUT);

	writel(0x5, ETC8DRV);
}
