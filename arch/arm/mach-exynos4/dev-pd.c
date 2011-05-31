/* linux/arch/arm/mach-exynos4/dev-pd.c
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4 - Power Domain support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <mach/regs-pmu.h>
#include <mach/regs-clock.h>

#include <plat/pd.h>

static int exynos4_pd_init(struct device *dev)
{
	struct samsung_pd_info *pdata =  dev->platform_data;
	struct exynos4_pd_data *data = (struct exynos4_pd_data *) pdata->data;

	if (data->read_phy_addr) {
		data->read_base = ioremap(data->read_phy_addr, SZ_4K);
		if (!data->read_base)
			return -ENOMEM;
	}

	return 0;
}

int exynos4_pd_enable(struct device *dev)
{
	struct samsung_pd_info *pdata =  dev->platform_data;
	struct exynos4_pd_data *data = (struct exynos4_pd_data *) pdata->data;
	u32 timeout;
	u32 tmp = 0;

	if (data->read_base)
		/*  save IP clock gating register */
		tmp = __raw_readl(data->clk_base);

	/*  enable all the clocks of IPs in the power domain */
	__raw_writel(0xffffffff, data->clk_base);

	__raw_writel(S5P_INT_LOCAL_PWR_EN, pdata->base);

	/* Wait max 1ms */
	timeout = 10;
	while ((__raw_readl(pdata->base + 0x4) & S5P_INT_LOCAL_PWR_EN)
		!= S5P_INT_LOCAL_PWR_EN) {
		if (timeout == 0) {
			printk(KERN_ERR "Power domain %s enable failed.\n",
				dev_name(dev));
			return -ETIMEDOUT;
		}
		timeout--;
		udelay(100);
	}

	if (data->read_base) {
		/* dummy read to check the completion of power-on sequence */
		__raw_readl(data->read_base);

		/* restore IP clock gating register */
		__raw_writel(tmp, data->clk_base);
	}

	return 0;
}

static int exynos4_pd_disable(struct device *dev)
{
	struct samsung_pd_info *pdata =  dev->platform_data;
	u32 timeout;

	__raw_writel(0, pdata->base);

	/* Wait max 1ms */
	timeout = 10;
	while (__raw_readl(pdata->base + 0x4) & S5P_INT_LOCAL_PWR_EN) {
		if (timeout == 0) {
			printk(KERN_ERR "Power domain %s disable failed.\n",
				dev_name(dev));
			return -ETIMEDOUT;
		}
		timeout--;
		udelay(100);
	}

	return 0;
}

struct platform_device exynos4_device_pd[] = {
	{
		.name		= "samsung-pd",
		.id		= 0,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.init		= exynos4_pd_init,
				.enable		= exynos4_pd_enable,
				.disable	= exynos4_pd_disable,
				.base		= S5P_PMU_MFC_CONF,
				.data		= &(struct exynos4_pd_data) {
					.clk_base	= S5P_CLKGATE_IP_MFC,
				},
			},
		},
	}, {
		.name		= "samsung-pd",
		.id		= 1,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.init		= exynos4_pd_init,
				.enable		= exynos4_pd_enable,
				.disable	= exynos4_pd_disable,
				.base		= S5P_PMU_G3D_CONF,
				.data		= &(struct exynos4_pd_data) {
					.clk_base	= S5P_CLKGATE_IP_G3D,
				},
			},
		},
	}, {
		.name		= "samsung-pd",
		.id		= 2,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.init		= exynos4_pd_init,
				.enable		= exynos4_pd_enable,
				.disable	= exynos4_pd_disable,
				.base		= S5P_PMU_LCD0_CONF,
				.data		= &(struct exynos4_pd_data) {
					.clk_base	= S5P_CLKGATE_IP_LCD0,
					.read_phy_addr	= EXYNOS4_PA_FIMD0,
				},
			},
		},
	}, {
		.name		= "samsung-pd",
		.id		= 3,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.init		= exynos4_pd_init,
				.enable		= exynos4_pd_enable,
				.disable	= exynos4_pd_disable,
				.base		= S5P_PMU_LCD1_CONF,
				.data		= &(struct exynos4_pd_data) {
					.clk_base	= S5P_CLKGATE_IP_LCD1,
					.read_phy_addr	= EXYNOS4_PA_FIMD1,
				},
			},
		},
	}, {
		.name		= "samsung-pd",
		.id		= 4,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.init		= exynos4_pd_init,
				.enable		= exynos4_pd_enable,
				.disable	= exynos4_pd_disable,
				.base		= S5P_PMU_TV_CONF,
				.data		= &(struct exynos4_pd_data) {
					.clk_base	= S5P_CLKGATE_IP_TV,
					.read_phy_addr	= EXYNOS4_PA_VP,
				},
			},
		},
	}, {
		.name		= "samsung-pd",
		.id		= 5,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.init		= exynos4_pd_init,
				.enable		= exynos4_pd_enable,
				.disable	= exynos4_pd_disable,
				.base		= S5P_PMU_CAM_CONF,
				.data		= &(struct exynos4_pd_data) {
					.clk_base	= S5P_CLKGATE_IP_CAM,
					.read_phy_addr	= EXYNOS4_PA_FIMC0,
				},
			},
		},
	}, {
		.name		= "samsung-pd",
		.id		= 6,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.init		= exynos4_pd_init,
				.enable		= exynos4_pd_enable,
				.disable	= exynos4_pd_disable,
				.base		= S5P_PMU_GPS_CONF,
				.data		= &(struct exynos4_pd_data) {
					.clk_base	= S5P_CLKGATE_IP_GPS,
				},
			},
		},
	},
};
