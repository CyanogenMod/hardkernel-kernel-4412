/*
 * arch/arm/mach-exynos4/dev-dwmci.c
 *
 * Platform device for Synopsys DesignWare Mobile Storage IP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mmc/dw_mmc.h>
#include <linux/mmc/host.h>
#include <linux/io.h>

#include <mach/map.h>
#include <mach/gpio.h>

#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/gpio-cfg.h>

#define EXYNOS4_SZ_DWMCI	(0x1000)

static int exynos4_mci_get_ocr(u32 slot_id);
static int exynos4_mci_get_bus_wd(u32 slot_id);
static int exynos4_mci_init(u32 slot_id, irq_handler_t handler, void *data);

static struct resource s3c_dwmci_resource[] = {
	[0] = {
		.start = EXYNOS4_PA_DWMCI,
		.end   = EXYNOS4_PA_DWMCI + EXYNOS4_SZ_DWMCI - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_DWMCI,
		.end   = IRQ_DWMCI,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 s3c_device_dwmci_dmamask = 0xffffffffUL;

static struct dw_mci_board exynos4_dwci_pdata __initdata = {
	.num_slots = 1,
	.quirks = DW_MCI_QUIRK_BROKEN_CARD_DETECTION,
	.bus_hz = 80*1000*1000,
	.detect_delay_ms = 200,
	.hclk_name = "dwmci",
	.cclk_name = "sclk_dwmci",
	.init = exynos4_mci_init,
	.get_ocr = exynos4_mci_get_ocr,
	.get_bus_wd = exynos4_mci_get_bus_wd,
	.select_slot = NULL,
};

struct platform_device exynos4_device_dwmci = {
	.name		= "dw_mmc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s3c_dwmci_resource),
	.resource	= s3c_dwmci_resource,
	.dev		= {
		.dma_mask		= &s3c_device_dwmci_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
		.platform_data	= &exynos4_dwci_pdata,
	},
};

static int exynos4_mci_get_ocr(u32 slot_id)
{
	return MMC_VDD_32_33 | MMC_VDD_33_34;
}

static int exynos4_mci_get_bus_wd(u32 slot_id)
{
	return 4;
}

static int exynos4_mci_init(u32 slot_id, irq_handler_t handler, void *data)
{
	unsigned int gpio;
	unsigned int width;

	width = exynos4_mci_get_bus_wd(slot_id);

	for (gpio = EXYNOS4_GPK0(0); gpio < EXYNOS4_GPK0(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	}

	switch (width) {
	case 8:
		for (gpio = EXYNOS4_GPK1(3); gpio <= EXYNOS4_GPK1(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(4));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
	case 4:
		for (gpio = EXYNOS4_GPK0(3); gpio <= EXYNOS4_GPK0(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
		break;
	case 1:
		gpio = EXYNOS4_GPK0(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	default:
		break;
	}

	return 0;
}

