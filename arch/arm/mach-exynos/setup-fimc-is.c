/* linux/arch/arm/mach-exynos/setup-fimc-is.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * FIMC-IS gpio and clock configuration
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <plat/clock.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
#include <plat/map-s5p.h>
#include <plat/cpu.h>
#include <mach/map.h>

struct platform_device; /* don't need the contents */

void exynos_fimc_is_cfg_gpio(struct platform_device *pdev)
{
	int ret;
	/* 1. UART setting for FIMC-IS */
	ret = gpio_request(EXYNOS4212_GPM3(4), "GPM3");
	if (ret)
		printk(KERN_ERR "#### failed to request GPM3_4 ####\n");
	s3c_gpio_cfgpin(EXYNOS4212_GPM3(4), (0x3<<16));
	s3c_gpio_setpull(EXYNOS4212_GPM3(4), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4212_GPM3(4));

	ret = gpio_request(EXYNOS4212_GPM3(5), "GPM3");
	if (ret)
		printk(KERN_ERR "#### failed to request GPM3_5 ####\n");
	s3c_gpio_cfgpin(EXYNOS4212_GPM3(5), (0x3<<20));
	s3c_gpio_setpull(EXYNOS4212_GPM3(5), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4212_GPM3(5));

	ret = gpio_request(EXYNOS4212_GPM3(6), "GPM3");
	if (ret)
		printk(KERN_ERR "#### failed to request GPM3_6 ####\n");
	s3c_gpio_cfgpin(EXYNOS4212_GPM3(6), (0x3<<24));
	s3c_gpio_setpull(EXYNOS4212_GPM3(6), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4212_GPM3(6));

	ret = gpio_request(EXYNOS4212_GPM3(7), "GPM3");
	if (ret)
		printk(KERN_ERR "#### failed to request GPM3_7 ####\n");
	s3c_gpio_cfgpin(EXYNOS4212_GPM3(7), (0x3<<28));
	s3c_gpio_setpull(EXYNOS4212_GPM3(7), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4212_GPM3(7));

	/* 2. GPIO setting for FIMC-IS */
	ret = gpio_request(EXYNOS4212_GPM4(0), "GPM4");
	if (ret)
		printk(KERN_ERR "#### failed to request GPM4_0 ####\n");
	s3c_gpio_cfgpin(EXYNOS4212_GPM4(0), (0x2<<0));
	s3c_gpio_setpull(EXYNOS4212_GPM4(0), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4212_GPM4(0));

	ret = gpio_request(EXYNOS4212_GPM4(1), "GPM4");
	if (ret)
		printk(KERN_ERR "#### failed to request GPM4_1 ####\n");
	s3c_gpio_cfgpin(EXYNOS4212_GPM4(1), (0x2<<4));
	s3c_gpio_setpull(EXYNOS4212_GPM4(1), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4212_GPM4(1));

	ret = gpio_request(EXYNOS4212_GPM4(2), "GPM4");
	if (ret)
		printk(KERN_ERR "#### failed to request GPM4_2 ####\n");
	s3c_gpio_cfgpin(EXYNOS4212_GPM4(2), (0x2<<8));
	s3c_gpio_setpull(EXYNOS4212_GPM4(2), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4212_GPM4(2));

	ret = gpio_request(EXYNOS4212_GPM4(3), "GPM4");
	if (ret)
		printk(KERN_ERR "#### failed to request GPM4_3 ####\n");
	s3c_gpio_cfgpin(EXYNOS4212_GPM4(3), (0x2<<12));
	s3c_gpio_setpull(EXYNOS4212_GPM4(3), S3C_GPIO_PULL_NONE);
	gpio_free(EXYNOS4212_GPM4(3));
}

int exynos_fimc_is_cfg_clk(struct platform_device *pdev)
{
	struct clk *aclk_mcuisp_muxed = NULL;
	struct clk *aclk_mcuisp_div0 = NULL;
	struct clk *aclk_mcuisp_div1 = NULL;
	struct clk *aclk_200 = NULL;
	struct clk *aclk_200_div0 = NULL;
	struct clk *aclk_200_div1 = NULL;
	struct clk *sclk_uart_isp = NULL;
	struct clk *sclk_uart_isp_src = NULL;
	struct clk *sclk_pwm_isp = NULL;
	struct clk *sclk_pwm_isp_src = NULL;
	/*
	 * initialize Clocks
	*/
	/* 1. MCUISP */
	aclk_mcuisp_muxed = clk_get(&pdev->dev, "aclk_400_muxed");
	if (IS_ERR(aclk_mcuisp_muxed))
		printk(KERN_ERR "failed to get aclk_mcuisp_muxed\n");
	aclk_mcuisp_div0 = clk_get(&pdev->dev, "sclk_mcuisp_div0");
	if (IS_ERR(aclk_mcuisp_div0))
		printk(KERN_ERR "failed to get aclk_mcuisp_div0\n");
	aclk_mcuisp_div1 = clk_get(&pdev->dev, "sclk_mcuisp_div1");
	if (IS_ERR(aclk_mcuisp_div1))
		printk(KERN_ERR "failed to get aclk_mcuisp_div1\n");
	clk_set_rate(aclk_mcuisp_div0, 400 * 1000000);
	clk_set_rate(aclk_mcuisp_div1, 400 * 1000000);
	clk_put(aclk_mcuisp_muxed);
	clk_put(aclk_mcuisp_div0);
	clk_put(aclk_mcuisp_div1);
	/* 2. ACLK_ISP */
	aclk_200 = clk_get(&pdev->dev, "aclk_200_muxed");
	if (IS_ERR(aclk_200))
		printk(KERN_ERR "failed to get aclk_200\n");
	aclk_200_div0 = clk_get(&pdev->dev, "sclk_aclk_div0");
	if (IS_ERR(aclk_200_div0))
		printk(KERN_ERR "failed to get aclk_200_div0\n");
	aclk_200_div1 = clk_get(&pdev->dev, "sclk_aclk_div1");
	if (IS_ERR(aclk_200_div1))
		printk(KERN_ERR "failed to get aclk_200_div1\n");
	clk_set_rate(aclk_200_div0, 80 * 1000000);
	clk_set_rate(aclk_200_div1, 80 * 1000000);
	clk_put(aclk_200);
	clk_put(aclk_200_div0);
	clk_put(aclk_200_div1);
	/* 3. UART-ISP */
	sclk_uart_isp = clk_get(&pdev->dev, "sclk_uart_isp");
	if (IS_ERR(sclk_uart_isp))
		printk(KERN_ERR "failed to get sclk_uart_isp\n");
	sclk_uart_isp_src = clk_get(&pdev->dev, "mout_mpll_user");
	if (IS_ERR(sclk_uart_isp_src))
		printk(KERN_ERR "failed to get sclk_uart_isp_src\n");
	clk_set_parent(sclk_uart_isp, sclk_uart_isp_src);
	clk_set_rate(sclk_uart_isp, 50 * 1000000);
	clk_put(sclk_uart_isp);
	clk_put(sclk_uart_isp_src);
	/* 4. PWM-ISP */
	sclk_pwm_isp = clk_get(&pdev->dev, "sclk_pwm_isp");
	if (IS_ERR(sclk_pwm_isp))
		printk(KERN_ERR "failed to get sclk_pwm_isp\n");
	sclk_pwm_isp_src = clk_get(&pdev->dev, "mout_mpll_user");
	if (IS_ERR(sclk_pwm_isp_src))
		printk(KERN_ERR "failed to get sclk_pwm_isp_src\n");
	clk_set_parent(sclk_pwm_isp, sclk_pwm_isp_src);
	clk_set_rate(sclk_pwm_isp, 50 * 1000000);
	clk_put(sclk_pwm_isp);
	clk_put(sclk_pwm_isp_src);
	return 0;
}

int exynos_fimc_is_clk_on(struct platform_device *pdev)
{
	struct clk *aclk_mcuisp_muxed = NULL;
	struct clk *aclk_mcuisp_div0 = NULL;
	struct clk *aclk_mcuisp_div1 = NULL;
	struct clk *aclk_200 = NULL;
	struct clk *aclk_200_div0 = NULL;
	struct clk *aclk_200_div1 = NULL;
	struct clk *sclk_uart_isp = NULL;
	struct clk *sclk_pwm_isp = NULL;

	/* 1. MCUISP */
	aclk_mcuisp_muxed = clk_get(&pdev->dev, "aclk_400_muxed");
	if (IS_ERR(aclk_mcuisp_muxed))
		printk(KERN_ERR "failed to get aclk_mcuisp_muxed\n");
	aclk_mcuisp_div0 = clk_get(&pdev->dev, "sclk_mcuisp_div0");
	if (IS_ERR(aclk_mcuisp_div0))
		printk(KERN_ERR "failed to get aclk_mcuisp_div0\n");
	aclk_mcuisp_div1 = clk_get(&pdev->dev, "sclk_mcuisp_div1");
	if (IS_ERR(aclk_mcuisp_div1))
		printk(KERN_ERR "failed to get aclk_mcuisp_div1\n");
	clk_enable(aclk_mcuisp_muxed);
	clk_enable(aclk_mcuisp_div0);
	clk_enable(aclk_mcuisp_div1);
	clk_put(aclk_mcuisp_muxed);
	clk_put(aclk_mcuisp_div0);
	clk_put(aclk_mcuisp_div1);
	/* 2. ACLK_ISP */
	aclk_200 = clk_get(&pdev->dev, "aclk_200_muxed");
	if (IS_ERR(aclk_200))
		printk(KERN_ERR "failed to get aclk_200\n");
	aclk_200_div0 = clk_get(&pdev->dev, "sclk_aclk_div0");
	if (IS_ERR(aclk_200_div0))
		printk(KERN_ERR "failed to get aclk_200_div0\n");
	aclk_200_div1 = clk_get(&pdev->dev, "sclk_aclk_div1");
	if (IS_ERR(aclk_200_div1))
		printk(KERN_ERR "failed to get aclk_200_div1\n");
	clk_enable(aclk_200);
	clk_enable(aclk_200_div0);
	clk_enable(aclk_200_div1);
	clk_put(aclk_200);
	clk_put(aclk_200_div0);
	clk_put(aclk_200_div1);
	/* 3. UART-ISP */
	sclk_uart_isp = clk_get(&pdev->dev, "sclk_uart_isp");
	if (IS_ERR(sclk_uart_isp))
		printk(KERN_ERR "failed to get sclk_uart_isp\n");
	clk_enable(sclk_uart_isp);
	clk_put(sclk_uart_isp);
	/* 4. PWM-ISP */
	sclk_pwm_isp = clk_get(&pdev->dev, "sclk_pwm_isp");
	if (IS_ERR(sclk_pwm_isp))
		printk(KERN_ERR "failed to get sclk_pwm_isp\n");
	clk_enable(sclk_pwm_isp);
	clk_put(sclk_pwm_isp);
	return 0;
}

int exynos_fimc_is_clk_off(struct platform_device *pdev)
{
	struct clk *aclk_mcuisp_muxed = NULL;
	struct clk *aclk_mcuisp_div0 = NULL;
	struct clk *aclk_mcuisp_div1 = NULL;
	struct clk *aclk_200 = NULL;
	struct clk *aclk_200_div0 = NULL;
	struct clk *aclk_200_div1 = NULL;
	struct clk *sclk_uart_isp = NULL;
	struct clk *sclk_pwm_isp = NULL;

	/* 1. MCUISP */
	aclk_mcuisp_muxed = clk_get(&pdev->dev, "aclk_400_muxed");
	if (IS_ERR(aclk_mcuisp_muxed))
		printk(KERN_ERR "failed to get aclk_mcuisp_muxed\n");
	aclk_mcuisp_div0 = clk_get(&pdev->dev, "sclk_mcuisp_div0");
	if (IS_ERR(aclk_mcuisp_div0))
		printk(KERN_ERR "failed to get aclk_mcuisp_div0\n");
	aclk_mcuisp_div1 = clk_get(&pdev->dev, "sclk_mcuisp_div1");
	if (IS_ERR(aclk_mcuisp_div1))
		printk(KERN_ERR "failed to get aclk_mcuisp_div1\n");
	clk_disable(aclk_mcuisp_muxed);
	clk_disable(aclk_mcuisp_div0);
	clk_disable(aclk_mcuisp_div1);
	clk_put(aclk_mcuisp_muxed);
	clk_put(aclk_mcuisp_div0);
	clk_put(aclk_mcuisp_div1);
	/* 2. ACLK_ISP */
	aclk_200 = clk_get(&pdev->dev, "aclk_200_muxed");
	if (IS_ERR(aclk_200))
		printk(KERN_ERR "failed to get aclk_200\n");
	aclk_200_div0 = clk_get(&pdev->dev, "sclk_aclk_div0");
	if (IS_ERR(aclk_200_div0))
		printk(KERN_ERR "failed to get aclk_200_div0\n");
	aclk_200_div1 = clk_get(&pdev->dev, "sclk_aclk_div1");
	if (IS_ERR(aclk_200_div1))
		printk(KERN_ERR "failed to get aclk_200_div1\n");
	clk_disable(aclk_200);
	clk_disable(aclk_200_div0);
	clk_disable(aclk_200_div1);
	clk_put(aclk_200);
	clk_put(aclk_200_div0);
	clk_put(aclk_200_div1);
	/* 3. UART-ISP */
	sclk_uart_isp = clk_get(&pdev->dev, "sclk_uart_isp");
	if (IS_ERR(sclk_uart_isp))
		printk(KERN_ERR "failed to get sclk_uart_isp\n");
	clk_disable(sclk_uart_isp);
	clk_put(sclk_uart_isp);
	/* 4. PWM-ISP */
	sclk_pwm_isp = clk_get(&pdev->dev, "sclk_pwm_isp");
	if (IS_ERR(sclk_pwm_isp))
		printk(KERN_ERR "failed to get sclk_pwm_isp\n");
	clk_disable(sclk_pwm_isp);
	clk_put(sclk_pwm_isp);
	return 0;
}
