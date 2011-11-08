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
#include <mach/c2c.h>
#include <plat/gpio-cfg.h>

#define ETC8DRV (S5P_VA_GPIO4 + 0xAC)

void exynos4_c2c_set_cprst(void)
{
	/* TODO */
}

void exynos4_c2c_clear_cprst(void)
{
	/* TODO */
}

void exynos4_c2c_cfg_gpio(enum c2c_buswidth rx_width, enum c2c_buswidth tx_width)
{
	int i;
	printk("[C2C] Set GPIO Rx Width %d, Tx Width %d\n", rx_width, tx_width);

	/* Set GPIO for C2C Rx */
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPV0(0), 8, S3C_GPIO_SFN(2));
	for (i = 0; i < 8; i++) {
		s5p_gpio_set_drvstr(EXYNOS4212_GPV0(i), S5P_GPIO_DRVSTR_LV1);
		s5p_gpio_set_pd_cfg(EXYNOS4212_GPV0(i), S5P_GPIO_PD_INPUT);
		s5p_gpio_set_pd_pull(EXYNOS4212_GPV0(i), S5P_GPIO_PD_DOWN_ENABLE);
	}

	if (rx_width == C2C_BUSWIDTH_16) {
		s3c_gpio_cfgrange_nopull(EXYNOS4212_GPV1(0), 8, S3C_GPIO_SFN(2));
		for (i = 0; i < 8; i++) {
			s5p_gpio_set_drvstr(EXYNOS4212_GPV1(i), S5P_GPIO_DRVSTR_LV1);
			s5p_gpio_set_pd_cfg(EXYNOS4212_GPV1(i), S5P_GPIO_PD_INPUT);
			s5p_gpio_set_pd_pull(EXYNOS4212_GPV1(i), S5P_GPIO_PD_DOWN_ENABLE);
		}
	} else if (rx_width == C2C_BUSWIDTH_10) {
		s3c_gpio_cfgrange_nopull(EXYNOS4212_GPV1(0), 2, S3C_GPIO_SFN(2));
		for (i = 0; i < 2; i++) {
			s5p_gpio_set_drvstr(EXYNOS4212_GPV1(i), S5P_GPIO_DRVSTR_LV1);
			s5p_gpio_set_pd_cfg(EXYNOS4212_GPV1(i), S5P_GPIO_PD_INPUT);
			s5p_gpio_set_pd_pull(EXYNOS4212_GPV1(i), S5P_GPIO_PD_DOWN_ENABLE);
		}
	}

	/* Set GPIO for C2C Tx */
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPV2(0), 8, S3C_GPIO_SFN(2));
	for (i = 0; i < 8; i++)
		s5p_gpio_set_drvstr(EXYNOS4212_GPV2(i), S5P_GPIO_DRVSTR_LV3);

	if (tx_width == C2C_BUSWIDTH_16) {
		s3c_gpio_cfgrange_nopull(EXYNOS4212_GPV3(0), 8, S3C_GPIO_SFN(2));
		for (i = 0; i < 8; i++)
			s5p_gpio_set_drvstr(EXYNOS4212_GPV3(i), S5P_GPIO_DRVSTR_LV3);
	} else if (tx_width == C2C_BUSWIDTH_10) {
		s3c_gpio_cfgrange_nopull(EXYNOS4212_GPV3(0), 2, S3C_GPIO_SFN(2));
		for (i = 0; i < 2; i++)
			s5p_gpio_set_drvstr(EXYNOS4212_GPV3(i), S5P_GPIO_DRVSTR_LV3);
	}

	/* Set GPIO for WakeReqOut/In */
	s3c_gpio_cfgrange_nopull(EXYNOS4212_GPV4(0), 2, S3C_GPIO_SFN(2));
	s5p_gpio_set_pd_cfg(EXYNOS4212_GPV4(0), S5P_GPIO_PD_INPUT);
	s5p_gpio_set_pd_pull(EXYNOS4212_GPV4(0), S5P_GPIO_PD_DOWN_ENABLE);

	writel(0x5, ETC8DRV);
}
