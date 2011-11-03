/* linux/arch/arm/mach-exynos/busfreq_opp_4212.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS4 - BUS clock frequency scaling support with OPP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/ktime.h>
#include <linux/tick.h>
#include <linux/kernel_stat.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/opp.h>
#include <mach/busfreq.h>

#include <asm/mach-types.h>

#include <mach/ppmu.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/gpio.h>
#include <mach/regs-mem.h>
#include <mach/cpufreq.h>
#include <mach/dev.h>

#include <plat/map-s5p.h>
#include <plat/gpio-cfg.h>

enum busfreq_level_idx {
	LV_0,
	LV_1,
	LV_2,
	LV_3,
	LV_END
};

static struct busfreq_table exynos4_busfreq_table[] = {
	{LV_0, 400000, 1100000, 0, 0, 0},
	{LV_1, 267000, 1000000, 0, 0, 0},
	{LV_2, 160000, 950000, 0, 0, 0},
	{LV_3, 100000, 950000, 0, 0, 0},
	{0, 0, 0, 0, 0, 0},
};

#define ASV_GROUP	5
static unsigned int exynos4_asv_volt[ASV_GROUP][LV_END] = {
	{1100000, 1000000, 950000, 950000},
	{1100000, 1000000, 950000, 950000},
	{1100000, 1000000, 950000, 950000},
	{1100000, 1000000, 950000, 950000},
	{1100000, 1000000, 950000, 950000},
};

static unsigned int exynos4212_int_volt[LV_END] = {
	1000000, 950000, 950000, 950000
};

static unsigned int clkdiv_dmc0[LV_END][6] = {
	/*
	 * Clock divider value for following
	 * { DIVACP, DIVACP_PCLK, DIVDPHY, DIVDMC, DIVDMCD
	 *              DIVDMCP}
	 */

	/* DMC L0: 400MHz */
	{3, 1, 1, 1, 1, 1},

	/* DMC L1: 266.7MHz */
	{4, 1, 1, 2, 1, 1},

	/* DMC L2: 160MHz */
	{5, 1, 1, 4, 1, 1},

	/* DMC L3: 100MHz */
	{7, 1, 1, 7, 1, 1},
};

static unsigned int clkdiv_dmc1[LV_END][6] = {
	/*
	 * Clock divider value for following
	 * { G2DACP, DIVC2C, DIVC2C_ACLK }
	 */

	/* DMC L0: 400MHz */
	{3, 1, 1},

	/* DMC L1: 266.7MHz */
	{4, 2, 1},

	/* DMC L2: 160MHz */
	{5, 4, 1},

	/* DMC L3: 100MHz */
	{7, 7, 1},
};

static unsigned int clkdiv_top[LV_END][5] = {
	/*
	 * Clock divider value for following
	 * { DIVACLK266_GPS, DIVACLK100, DIVACLK160,
		DIVACLK133, DIVONENAND }
	 */

	/* ACLK160 L0: 160MHz */
	{2, 7, 4, 5, 1},

	/* ACLK160 L1: 133MHz */
	{4, 7, 5, 7, 1},

	/* ACLK160 L1: 133MHz */
	{4, 7, 5, 7, 1},

	/* ACLK160 L2: 100MHz */
	{7, 7, 7, 7, 1},
};

static unsigned int clkdiv_lr_bus[LV_END][2] = {
	/*
	 * Clock divider value for following
	 * { DIVGDL/R, DIVGPL/R }
	 */

	/* ACLK_GDL/R L1: 200MHz */
	{3, 1},

	/* ACLK_GDL/R L2: 160MHz */
	{4, 1},

	/* ACLK_GDL/R L2: 160MHz */
	{4, 1},

	/* ACLK_GDL/R L3: 100MHz */
	{7, 1},
};

static unsigned int clkdiv_sclkip[LV_END][3] = {
	/*
	 * Clock divider value for following
	 * { DIVMFC, DIVJPEG, DIVFIMC0~3}
	 */

	/* SCLK_MFC: 200MHz */
	{3, 3, 4},

	/* SCLK_MFC: 160MHz */
	{4, 4, 5},

	/* SCLK_MFC: 160MHz */
	{4, 4, 5},

	/* SCLK_MFC: 100MHz */
	{7, 7, 7},
};

static void exynos4212_set_bus_volt(void)
{
	unsigned int asv_group;
	unsigned int i;

	asv_group = __raw_readl(S5P_INFORM2) & 0xF;

	printk(KERN_INFO "DVFS : VDD_INT Voltage table set with %d Group\n", asv_group);

	for (i = 0 ; i < LV_END ; i++) {
		switch (asv_group) {
		case 0:
			exynos4_busfreq_table[i].volt =
				exynos4_asv_volt[0][i];
			break;
		case 1:
		case 2:
			exynos4_busfreq_table[i].volt =
				exynos4_asv_volt[1][i];
			break;
		case 3:
		case 4:
			exynos4_busfreq_table[i].volt =
				exynos4_asv_volt[2][i];
			break;
		case 5:
		case 6:
			exynos4_busfreq_table[i].volt =
				exynos4_asv_volt[3][i];
			break;
		case 7:
			exynos4_busfreq_table[i].volt =
				exynos4_asv_volt[4][i];
			break;
		}
	}

	return;
}

unsigned int exynos4212_target(unsigned int div_index)
{
	unsigned int tmp;

	/* Change Divider - DMC0 */
	tmp = exynos4_busfreq_table[div_index].clk_dmc0div;

	__raw_writel(tmp, EXYNOS4_CLKDIV_DMC0);

	do {
		tmp = __raw_readl(EXYNOS4_CLKDIV_STAT_DMC0);
	} while (tmp & 0x11111111);

	/* Change Divider - DMC1 */
	tmp = __raw_readl(EXYNOS4_CLKDIV_DMC1);

	tmp &= ~(EXYNOS4_CLKDIV_DMC1_G2D_ACP_MASK |
		EXYNOS4_CLKDIV_DMC1_C2C_MASK |
		EXYNOS4_CLKDIV_DMC1_C2CACLK_MASK);

	tmp |= ((clkdiv_dmc1[div_index][0] << EXYNOS4_CLKDIV_DMC1_G2D_ACP_SHIFT) |
		(clkdiv_dmc1[div_index][1] << EXYNOS4_CLKDIV_DMC1_C2C_SHIFT) |
		(clkdiv_dmc1[div_index][2] << EXYNOS4_CLKDIV_DMC1_C2CACLK_SHIFT));

	__raw_writel(tmp, EXYNOS4_CLKDIV_DMC1);

	do {
		tmp = __raw_readl(EXYNOS4_CLKDIV_STAT_DMC1);
	} while (tmp & 0x111111);

	/* Change Divider - TOP */
	tmp = __raw_readl(EXYNOS4_CLKDIV_TOP);

	tmp &= ~(EXYNOS4_CLKDIV_TOP_ACLK266_GPS_MASK |
		EXYNOS4_CLKDIV_TOP_ACLK100_MASK |
		EXYNOS4_CLKDIV_TOP_ACLK160_MASK |
		EXYNOS4_CLKDIV_TOP_ACLK133_MASK |
		EXYNOS4_CLKDIV_TOP_ONENAND_MASK);

	tmp |= ((clkdiv_top[div_index][0] << EXYNOS4_CLKDIV_TOP_ACLK266_GPS_SHIFT) |
		(clkdiv_top[div_index][1] << EXYNOS4_CLKDIV_TOP_ACLK100_SHIFT) |
		(clkdiv_top[div_index][2] << EXYNOS4_CLKDIV_TOP_ACLK160_SHIFT) |
		(clkdiv_top[div_index][3] << EXYNOS4_CLKDIV_TOP_ACLK133_SHIFT) |
		(clkdiv_top[div_index][4] << EXYNOS4_CLKDIV_TOP_ONENAND_SHIFT));

	__raw_writel(tmp, EXYNOS4_CLKDIV_TOP);

	do {
		tmp = __raw_readl(EXYNOS4_CLKDIV_STAT_TOP);
	} while (tmp & 0x11111);

	/* Change Divider - LEFTBUS */
	tmp = __raw_readl(EXYNOS4_CLKDIV_LEFTBUS);

	tmp &= ~(EXYNOS4_CLKDIV_BUS_GDLR_MASK | EXYNOS4_CLKDIV_BUS_GPLR_MASK);

	tmp |= ((clkdiv_lr_bus[div_index][0] << EXYNOS4_CLKDIV_BUS_GDLR_SHIFT) |
		(clkdiv_lr_bus[div_index][1] << EXYNOS4_CLKDIV_BUS_GPLR_SHIFT));

	__raw_writel(tmp, EXYNOS4_CLKDIV_LEFTBUS);

	do {
		tmp = __raw_readl(EXYNOS4_CLKDIV_STAT_LEFTBUS);
	} while (tmp & 0x11);

	/* Change Divider - RIGHTBUS */
	tmp = __raw_readl(EXYNOS4_CLKDIV_RIGHTBUS);

	tmp &= ~(EXYNOS4_CLKDIV_BUS_GDLR_MASK | EXYNOS4_CLKDIV_BUS_GPLR_MASK);

	tmp |= ((clkdiv_lr_bus[div_index][0] << EXYNOS4_CLKDIV_BUS_GDLR_SHIFT) |
		(clkdiv_lr_bus[div_index][1] << EXYNOS4_CLKDIV_BUS_GPLR_SHIFT));

	__raw_writel(tmp, EXYNOS4_CLKDIV_RIGHTBUS);

	do {
		tmp = __raw_readl(EXYNOS4_CLKDIV_STAT_RIGHTBUS);
	} while (tmp & 0x11);

	/* Change Divider - MFC */
	tmp = __raw_readl(EXYNOS4_CLKDIV_MFC);

	tmp &= ~(EXYNOS4_CLKDIV_MFC_MASK);

	tmp |= ((clkdiv_sclkip[div_index][0] << EXYNOS4_CLKDIV_MFC_SHIFT));

	__raw_writel(tmp, EXYNOS4_CLKDIV_MFC);

	do {
		tmp = __raw_readl(EXYNOS4_CLKDIV_STAT_MFC);
	} while (tmp & 0x1);

	/* Change Divider - JPEG */
	tmp = __raw_readl(EXYNOS4_CLKDIV_CAM1);

	tmp &= ~(EXYNOS4_CLKDIV_CAM1_JPEG_MASK);

	tmp |= ((clkdiv_sclkip[div_index][1] << EXYNOS4_CLKDIV_CAM1_JPEG_SHIFT));

	__raw_writel(tmp, EXYNOS4_CLKDIV_CAM1);

	do {
		tmp = __raw_readl(EXYNOS4_CLKDIV_STAT_CAM1);
	} while (tmp & 0x1);

	/* Change Divider - FIMC0~3 */
	tmp = __raw_readl(EXYNOS4_CLKDIV_CAM);

	tmp &= ~(EXYNOS4_CLKDIV_CAM_FIMC0_MASK | EXYNOS4_CLKDIV_CAM_FIMC1_MASK |
		EXYNOS4_CLKDIV_CAM_FIMC2_MASK | EXYNOS4_CLKDIV_CAM_FIMC3_MASK);

	tmp |= ((clkdiv_sclkip[div_index][2] << EXYNOS4_CLKDIV_CAM_FIMC0_SHIFT) |
		(clkdiv_sclkip[div_index][2] << EXYNOS4_CLKDIV_CAM_FIMC1_SHIFT) |
		(clkdiv_sclkip[div_index][2] << EXYNOS4_CLKDIV_CAM_FIMC2_SHIFT) |
		(clkdiv_sclkip[div_index][2] << EXYNOS4_CLKDIV_CAM_FIMC3_SHIFT));

	__raw_writel(tmp, EXYNOS4_CLKDIV_CAM);

	do {
		tmp = __raw_readl(EXYNOS4_CLKDIV_STAT_CAM1);
	} while (tmp & 0x1111);

	return div_index;
}

unsigned int exynos4212_get_table_index(struct opp *opp)
{
	unsigned int index;

	for (index = LV_0; index < LV_END; index++)
		if (opp_get_freq(opp) == exynos4_busfreq_table[index].mem_clk)
			break;

	return index;
}


unsigned int exynos4212_get_int_volt(unsigned long index)
{
	return exynos4212_int_volt[index];
}

int exynos4212_init(struct device *dev, struct busfreq_data *data)
{
	unsigned int i;
	unsigned int tmp;
	struct cpufreq_frequency_table *table;
	unsigned long freq;
	unsigned long min_cpufreq = UINT_MAX;
	unsigned long maxfreq = UINT_MAX;
	int ret;

	/* Enable pause function for DREX2 DVFS */
	tmp = __raw_readl(EXYNOS4_DMC_PAUSE_CTRL);
	tmp |= DMC_PAUSE_ENABLE;
	__raw_writel(tmp, EXYNOS4_DMC_PAUSE_CTRL);

	tmp = __raw_readl(EXYNOS4_CLKDIV_DMC0);

	for (i = 0; i <  LV_END; i++) {
		tmp &= ~(EXYNOS4_CLKDIV_DMC0_ACP_MASK |
			EXYNOS4_CLKDIV_DMC0_ACPPCLK_MASK |
			EXYNOS4_CLKDIV_DMC0_DPHY_MASK |
			EXYNOS4_CLKDIV_DMC0_DMC_MASK |
			EXYNOS4_CLKDIV_DMC0_DMCD_MASK |
			EXYNOS4_CLKDIV_DMC0_DMCP_MASK |
			EXYNOS4_CLKDIV_DMC0_COPY2_MASK |
			EXYNOS4_CLKDIV_DMC0_CORETI_MASK);

		tmp |= ((clkdiv_dmc0[i][0] << EXYNOS4_CLKDIV_DMC0_ACP_SHIFT) |
			(clkdiv_dmc0[i][1] << EXYNOS4_CLKDIV_DMC0_ACPPCLK_SHIFT) |
			(clkdiv_dmc0[i][2] << EXYNOS4_CLKDIV_DMC0_DPHY_SHIFT) |
			(clkdiv_dmc0[i][3] << EXYNOS4_CLKDIV_DMC0_DMC_SHIFT) |
			(clkdiv_dmc0[i][4] << EXYNOS4_CLKDIV_DMC0_DMCD_SHIFT) |
			(clkdiv_dmc0[i][5] << EXYNOS4_CLKDIV_DMC0_DMCP_SHIFT));

		exynos4_busfreq_table[i].clk_dmc0div = tmp;
	}

	exynos4212_set_bus_volt();

	for (i = 0; i < LV_END; i++) {
		ret = opp_add(dev, exynos4_busfreq_table[i].mem_clk,
				exynos4_busfreq_table[i].volt);
		if (ret) {
			dev_err(dev, "Fail to add opp entries.\n");
			return ret;
		}
	}

	table = cpufreq_frequency_get_table(0);
	if (IS_ERR(table)) {
		dev_err(dev, "Fail to get cpufreq table.\n");
		data->min_cpufreq = 2000000;
	}

	for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++) {
		freq = table[i].frequency;
		if (freq != CPUFREQ_ENTRY_INVALID && min_cpufreq > freq)
			min_cpufreq = freq;
	}

	data->table = exynos4_busfreq_table;
	data->table_size = LV_END;
	data->min_cpufreq = min_cpufreq;

	/* Find max frequency */
	data->max_opp = opp_find_freq_floor(dev, &maxfreq);

	data->vdd_int = regulator_get(NULL, "vdd_int");
	if (IS_ERR(data->vdd_int)) {
		pr_err("failed to get resource %s\n", "vdd_int");
		return -ENODEV;
	}

	data->vdd_mif = regulator_get(NULL, "vdd_mif");
	if (IS_ERR(data->vdd_mif)) {
		pr_err("failed to get resource %s\n", "vdd_mif");
		regulator_put(data->vdd_int);
		return -ENODEV;
	}

	return 0;
}
