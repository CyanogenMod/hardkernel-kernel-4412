/* linux/arch/arm/mach-exynos/busfreq_opp_5250.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS5 - BUS clock frequency scaling support with OPP
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
#include <mach/dev.h>
#include <mach/asv.h>

#include <plat/map-s5p.h>
#include <plat/gpio-cfg.h>
#include <plat/cpu.h>

enum busfreq_level_idx {
	LV_0,
	LV_1,
	LV_2,
	LV_3,
	LV_4,
	LV_5,
	LV_END
};

static struct busfreq_table exynos5_busfreq_table[] = {
	{LV_0, 800000, 1000000, 0, 0, 0},
	{LV_1, 400000, 1000000, 0, 0, 0},
	{LV_2, 267000, 1000000, 0, 0, 0},
	{LV_3, 200000, 1000000, 0, 0, 0},
	{LV_4, 160000, 1000000, 0, 0, 0},
	{LV_5, 100000, 1000000, 0, 0, 0},
};

#define ASV_GROUP	9
static unsigned int asv_group_index;

static unsigned int exynos5_asv_volt[ASV_GROUP][LV_END] = {
	/* 800      400      267      200      160      100 */
	{1200000, 1200000, 1200000, 1200000, 1200000, 1200000}, /* ASV0 */
	{1200000, 1200000, 1200000, 1200000, 1200000, 1200000}, /* ASV1 */
	{1200000, 1200000, 1200000, 1200000, 1200000, 1200000}, /* ASV2 */
	{1200000, 1200000, 1200000, 1200000, 1200000, 1200000}, /* ASV3 */
	{1200000, 1200000, 1200000, 1200000, 1200000, 1200000}, /* ASV4 */
	{1200000, 1200000, 1200000, 1200000, 1200000, 1200000}, /* ASV5 */
	{1200000, 1200000, 1200000, 1200000, 1200000, 1200000}, /* ASV6 */
	{1200000, 1200000, 1200000, 1200000, 1200000, 1200000}, /* ASV7 */
	{1200000, 1200000, 1200000, 1200000, 1200000, 1200000}, /* ASV8 */
};

static unsigned int exynos5250_int_volt[ASV_GROUP][LV_END] = {
	/* 400      400      267      200      160      100 */
	{1150000, 1150000, 1150000, 1150000, 1150000, 11500000}, /* ASV0 */
	{1150000, 1150000, 1150000, 1150000, 1150000, 11500000}, /* ASV1 */
	{1150000, 1150000, 1150000, 1150000, 1150000, 11500000}, /* ASV2 */
	{1150000, 1150000, 1150000, 1150000, 1150000, 11500000}, /* ASV3 */
	{1150000, 1150000, 1150000, 1150000, 1150000, 11500000}, /* ASV4 */
	{1150000, 1150000, 1150000, 1150000, 1150000, 11500000}, /* ASV5 */
	{1150000, 1150000, 1150000, 1150000, 1150000, 11500000}, /* ASV6 */
	{1150000, 1150000, 1150000, 1150000, 1150000, 11500000}, /* ASV7 */
	{1150000, 1150000, 1150000, 1150000, 1150000, 11500000}, /* ASV8 */
};

/* For CMU_LEX */
static unsigned int clkdiv_lex[LV_END][2] = {
	/*
	 * Clock divider value for following
	 * { DIVATCLK_LEX, DIVPCLK_LEX }
	 */

	/* ATCLK_LEX L0 : 200MHz */
	{0, 1},

	/* ATCLK_LEX L1 : 200MHz */
	{0, 1},

	/* ATCLK_LEX L2 : 166MHz */
	{0, 1},

	/* ATCLK_LEX L3 : 133MHz */
	{0, 1},

	/* ATCLK_LEX L4 : 114MHz */
	{0, 1},

	/* ATCLK_LEX L5 : 100MHz */
	{0, 1},
};

/* For CMU_R0X */
static unsigned int clkdiv_r0x[LV_END][1] = {
	/*
	 * Clock divider value for following
	 * { DIVPCLK_R0X }
	 */

	/* ACLK_PR0X L0 : 133MHz */
	{1},

	/* ACLK_PR0X L1 : 133MHz */
	{1},

	/* ACLK_DR0X L2 : 100MHz */
	{1},

	/* ACLK_PR0X L3 : 80MHz */
	{1},

	/* ACLK_PR0X L4 : 67MHz */
	{1},

	/* ACLK_PR0X L5 : 50MHz */
	{1},
};

/* For CMU_R1X */
static unsigned int clkdiv_r1x[LV_END][1] = {
	/*
	 * Clock divider value for following
	 * { DIVPCLK_R1X }
	 */

	/* ACLK_PR1X L0 : 133MHz */
	{1},

	/* ACLK_PR1X L1 : 133MHz */
	{1},

	/* ACLK_DR1X L2 : 100MHz */
	{1},

	/* ACLK_PR1X L3 : 80MHz */
	{1},

	/* ACLK_PR1X L4 : 67MHz */
	{1},

	/* ACLK_PR1X L5 : 50MHz */
	{1},
};

/* For CMU_TOP */
static unsigned int clkdiv_top[LV_END][10] = {
	/*
	 * Clock divider value for following
	 * { DIVACLK400_ISP, DIVACLK400_IOP, DIVACLK266, DIVACLK_200, DIVACLK_66_PRE,
	 DIVACLK_66, DIVACLK_333, DIVACLK_166, DIVACLK_300_DISP1, DIVACLK300_GSCL }
	 */

	/* ACLK_400_ISP L0 : 400MHz */
	{1, 1, 2, 3, 1, 5, 0, 1, 2, 2},

	/* ACLK_400_ISP L1 : 400MHz */
	{1, 1, 2, 3, 1, 5, 0, 1, 2, 2},

	/* ACLK_400_ISP L2 : 267MHz */
	{2, 3, 3, 4, 1, 5, 1, 2, 3, 3},

	/* ACLK_400_ISP L3 : 200MHz */
	{3, 3, 4, 5, 1, 5, 2, 3, 4, 4},

	/* ACLK_400_ISP L4 : 160MHz */
	{4, 4, 5, 6, 1, 5, 3, 4, 5, 5},

	/* ACLK_400_ISP L5 : 100MHz */
	{7, 7, 7, 7, 1, 5, 7, 7, 7, 7},
};

/* For CMU_CDREX */
static unsigned int __maybe_unused clkdiv_cdrex_for800[LV_END][9] = {
	/*
	 * Clock divider value for following
	 * { DIVMCLK_DPHY, DIVMCLK_CDREX2, DIVACLK_CDREX, DIVMCLK_CDREX,
		DIVPCLK_CDREX, DIVC2C, DIVC2C_ACLK, DIVMCLK_EFPHY, DIVACLK_EFCON }
	 */

	/* ACLK_CDREX L0: 800MHz */
	{0, 0, 1, 0, 5, 1, 1, 4, 1},

	/* ACLK_CDREX L1: 400MHz */
	{0, 1, 1, 1, 5, 2, 1, 5, 1},

	/* ACLK_CDREX L2: 267MHz */
	{0, 2, 1, 2, 5, 3, 1, 6, 1},

	/* ACLK_CDREX L3: 200MHz */
	{0, 3, 1, 3, 5, 4, 1, 7, 1},

	/* ACLK_CDREX L4: 160MHz */
	{0, 4, 1, 4, 5, 5, 1, 8, 1},

	/* ACLK_CDREX L5: 100MHz */
	{0, 7, 1, 5, 7, 7, 1, 15, 1},
};

static unsigned int __maybe_unused clkdiv_cdrex_for667[LV_END][9] = {
	/*
	 * Clock divider value for following
	 * { DIVMCLK_DPHY, DIVMCLK_CDREX2, DIVACLK_CDREX, DIVMCLK_CDREX,
		DIVPCLK_CDREX, DIVC2C, DIVC2C_ACLK, DIVMCLK_EFPHY, DIVACLK_EFCON }
	 */

	/* ACLK_CDREX L0: 800MHz */
	{0, 0, 1, 0, 4, 1, 1, 4, 1},

	/* ACLK_CDREX L1: 400MHz */
	{0, 1, 1, 1, 4, 2, 1, 5, 1},

	/* ACLK_CDREX L2: 267MHz */
	{0, 2, 1, 2, 4, 3, 1, 6, 1},

	/* ACLK_CDREX L3: 200MHz */
	{0, 3, 1, 3, 4, 4, 1, 7, 1},

	/* ACLK_CDREX L4: 160MHz */
	{0, 4, 1, 4, 4, 5, 1, 8, 1},

	/* ACLK_CDREX L5: 100MHz */
	{0, 7, 1, 4, 7, 7, 1, 15, 1},
};

static unsigned int clkdiv_cdrex_for533[LV_END][9] = {
	/*
	 * Clock divider value for following
	 * { DIVMCLK_DPHY, DIVMCLK_CDREX2, DIVACLK_CDREX, DIVMCLK_CDREX,
		DIVPCLK_CDREX, DIVC2C, DIVC2C_ACLK, DIVMCLK_EFPHY, DIVACLK_EFCON }
	 */

	/* ACLK_CDREX L0: 800MHz */
	{0, 0, 1, 0, 3, 1, 1, 4, 1},

	/* ACLK_CDREX L1: 400MHz */
	{0, 1, 1, 1, 3, 2, 1, 5, 1},

	/* ACLK_CDREX L2: 267MHz */
	{0, 2, 1, 2, 3, 3, 1, 6, 1},

	/* ACLK_CDREX L3: 200MHz */
	{0, 3, 1, 3, 3, 4, 1, 7, 1},

	/* ACLK_CDREX L4: 160MHz */
	{0, 4, 1, 4, 3, 5, 1, 8, 1},

	/* ACLK_CDREX L5: 100MHz */
	{0, 7, 1, 3, 7, 7, 1, 15, 1},
};

static unsigned int __maybe_unused clkdiv_cdrex_for400[LV_END][9] = {
	/*
	 * Clock divider value for following
	 * { DIVMCLK_DPHY, DIVMCLK_CDREX2, DIVACLK_CDREX, DIVMCLK_CDREX,
		DIVPCLK_CDREX, DIVC2C, DIVC2C_ACLK, DIVMCLK_EFPHY, DIVACLK_EFCON }
	 */

	/* ACLK_CDREX L0: 800MHz */
	{1, 1, 1, 0, 5, 1, 1, 4, 1},

	/* ACLK_CDREX L1: 400MHz */
	{1, 2, 1, 2, 2, 2, 1, 5, 1},

	/* ACLK_CDREX L2: 267MHz */
	{1, 3, 1, 3, 2, 3, 1, 6, 1},

	/* ACLK_CDREX L3: 200MHz */
	{1, 4, 1, 4, 2, 4, 1, 7, 1},

	/* ACLK_CDREX L4: 160MHz */
	{1, 5, 1, 5, 2, 5, 1, 8, 1},

	/* ACLK_CDREX L5: 100MHz */
	{1, 7, 1, 2, 7, 7, 1, 15, 1},
};

static void exynos5250_set_bus_volt(void)
{
	unsigned int i;

	asv_group_index = 0;

	if (asv_group_index == 0xff)
		asv_group_index = 0;

	printk(KERN_INFO "DVFS : VDD_INT Voltage table set with %d Group\n", asv_group_index);

	for (i = 0 ; i < LV_END ; i++)
		exynos5_busfreq_table[i].volt =
			exynos5_asv_volt[asv_group_index][i];

	return;
}

unsigned int exynos5250_target(unsigned int div_index)
{
	unsigned int tmp;

	/* Temporary value */
	div_index = 0;

	/* Change Divider - CDREX */
	tmp = __raw_readl(EXYNOS5_CLKDIV_CDREX);

	tmp &= ~(EXYNOS5_CLKDIV_CDREX_MCLK_DPHY_MASK |
		EXYNOS5_CLKDIV_CDREX_MCLK_CDREX2_MASK |
		EXYNOS5_CLKDIV_CDREX_ACLK_CDREX_MASK |
		EXYNOS5_CLKDIV_CDREX_MCLK_CDREX_MASK |
		EXYNOS5_CLKDIV_CDREX_PCLK_CDREX_MASK |
		EXYNOS5_CLKDIV_CDREX_ACLK_CLK400_MASK |
		EXYNOS5_CLKDIV_CDREX_ACLK_C2C200_MASK |
		EXYNOS5_CLKDIV_CDREX_ACLK_EFCON_MASK);

	tmp |= ((clkdiv_cdrex_for533[div_index][0] << EXYNOS5_CLKDIV_CDREX_MCLK_DPHY_SHIFT) |
		(clkdiv_cdrex_for533[div_index][1] << EXYNOS5_CLKDIV_CDREX_MCLK_CDREX2_SHIFT) |
		(clkdiv_cdrex_for533[div_index][2] << EXYNOS5_CLKDIV_CDREX_ACLK_CDREX_SHIFT) |
		(clkdiv_cdrex_for533[div_index][3] << EXYNOS5_CLKDIV_CDREX_MCLK_CDREX_SHIFT) |
		(clkdiv_cdrex_for533[div_index][4] << EXYNOS5_CLKDIV_CDREX_PCLK_CDREX_SHIFT) |
		(clkdiv_cdrex_for533[div_index][5] << EXYNOS5_CLKDIV_CDREX_ACLK_CLK400_SHIFT) |
		(clkdiv_cdrex_for533[div_index][6] << EXYNOS5_CLKDIV_CDREX_ACLK_C2C200_SHIFT) |
		(clkdiv_cdrex_for533[div_index][8] << EXYNOS5_CLKDIV_CDREX_ACLK_EFCON_SHIFT));

	__raw_writel(tmp, EXYNOS5_CLKDIV_CDREX);

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_CDREX);
	} while (tmp & 0x11111111);

	tmp = __raw_readl(EXYNOS5_CLKDIV_CDREX2);

	tmp &= ~EXYNOS5_CLKDIV_CDREX2_MCLK_EFPHY_MASK;

	tmp |= clkdiv_cdrex_for533[div_index][7] << EXYNOS5_CLKDIV_CDREX2_MCLK_EFPHY_SHIFT;

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_CDREX2);
	} while (tmp & 0x1);

	/* Change Divider - TOP */
	tmp = __raw_readl(EXYNOS5_CLKDIV_TOP0);

	tmp &= ~(EXYNOS5_CLKDIV_TOP0_ACLK266_MASK |
		EXYNOS5_CLKDIV_TOP0_ACLK200_MASK |
		EXYNOS5_CLKDIV_TOP0_ACLK66_MASK |
		EXYNOS5_CLKDIV_TOP0_ACLK333_MASK |
		EXYNOS5_CLKDIV_TOP0_ACLK166_MASK |
		EXYNOS5_CLKDIV_TOP0_ACLK300_DISP1_MASK);

	tmp |= ((clkdiv_top[div_index][2] << EXYNOS5_CLKDIV_TOP0_ACLK266_SHIFT) |
		(clkdiv_top[div_index][3] << EXYNOS5_CLKDIV_TOP0_ACLK200_SHIFT) |
		(clkdiv_top[div_index][5] << EXYNOS5_CLKDIV_TOP0_ACLK66_SHIFT) |
		(clkdiv_top[div_index][6] << EXYNOS5_CLKDIV_TOP0_ACLK333_SHIFT) |
		(clkdiv_top[div_index][7] << EXYNOS5_CLKDIV_TOP0_ACLK166_SHIFT) |
		(clkdiv_top[div_index][8] << EXYNOS5_CLKDIV_TOP0_ACLK300_DISP1_SHIFT));

	__raw_writel(tmp, EXYNOS5_CLKDIV_TOP0);

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_TOP0);
	} while (tmp & 0x151101);

	tmp = __raw_readl(EXYNOS5_CLKDIV_TOP1);

	tmp &= ~(EXYNOS5_CLKDIV_TOP1_ACLK400_ISP_MASK |
		EXYNOS5_CLKDIV_TOP1_ACLK400_IOP_MASK |
		EXYNOS5_CLKDIV_TOP1_ACLK66_PRE_MASK |
		EXYNOS5_CLKDIV_TOP1_ACLK300_GSCL_MASK);

	tmp |= ((clkdiv_top[div_index][0] << EXYNOS5_CLKDIV_TOP1_ACLK400_ISP_SHIFT) |
		(clkdiv_top[div_index][1] << EXYNOS5_CLKDIV_TOP1_ACLK400_IOP_SHIFT) |
		(clkdiv_top[div_index][2] << EXYNOS5_CLKDIV_TOP1_ACLK66_PRE_SHIFT) |
		(clkdiv_top[div_index][9] << EXYNOS5_CLKDIV_TOP1_ACLK300_GSCL_SHIFT));

	__raw_writel(tmp, EXYNOS5_CLKDIV_TOP1);

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_TOP1);
	} while (tmp & 0x1110000);

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_TOP0);
	} while (tmp & 0x80000);

	/* Change Divider - LEX */
	tmp = __raw_readl(EXYNOS5_CLKDIV_LEX);

	tmp &= ~(EXYNOS5_CLKDIV_LEX_ATCLK_LEX_MASK | EXYNOS5_CLKDIV_LEX_PCLK_LEX_MASK);

	tmp |= ((clkdiv_lex[div_index][0] << EXYNOS5_CLKDIV_LEX_ATCLK_LEX_SHIFT) |
		(clkdiv_lex[div_index][1] << EXYNOS5_CLKDIV_LEX_PCLK_LEX_SHIFT));

	__raw_writel(tmp, EXYNOS5_CLKDIV_LEX);

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_LEX);
	} while (tmp & 0x110);

	/* Change Divider - R0X */
	tmp = __raw_readl(EXYNOS5_CLKDIV_R0X);

	tmp &= ~EXYNOS5_CLKDIV_R0X_PCLK_R0X_MASK;

	tmp |= (clkdiv_r0x[div_index][0] << EXYNOS5_CLKDIV_R0X_PCLK_R0X_SHIFT);

	__raw_writel(tmp, EXYNOS5_CLKDIV_R0X);

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_R0X);
	} while (tmp & 0x10);

	/* Change Divider - R1X */
	tmp = __raw_readl(EXYNOS5_CLKDIV_R1X);

	tmp &= ~EXYNOS5_CLKDIV_R1X_PCLK_R1X_MASK;

	tmp |= (clkdiv_r1x[div_index][0] << EXYNOS5_CLKDIV_R0X_PCLK_R0X_SHIFT);

	__raw_writel(tmp, EXYNOS5_CLKDIV_R1X);

	do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_R1X);
	} while (tmp & 0x10);

	return div_index;
}

unsigned int exynos5250_get_table_index(struct opp *opp)
{
	unsigned int index;

	for (index = LV_0; index < LV_END; index++)
		if (opp_get_freq(opp) == exynos5_busfreq_table[index].mem_clk)
			break;

	return index;
}


unsigned int exynos5250_get_int_volt(unsigned long index)
{
	return exynos5250_int_volt[asv_group_index][index];
}

struct opp *exynos5250_monitor(struct busfreq_data *data)
{
	struct opp *opp = data->curr_opp;
	unsigned long lockfreq;
	unsigned long newfreq = opp_get_freq(data->max_opp);

	ppmu_update(data->dev);

	lockfreq = dev_max_freq(data->dev);

	newfreq = max(lockfreq, newfreq);

	opp = opp_find_freq_ceil(data->dev, &newfreq);

	return opp;
}

int exynos5250_init(struct device *dev, struct busfreq_data *data)
{
	unsigned int i;
	unsigned long maxfreq = ULONG_MAX;
	unsigned long minfreq = 0;
	int ret;

	exynos5250_set_bus_volt();

	for (i = 0; i < LV_END; i++) {
		ret = opp_add(dev, exynos5_busfreq_table[i].mem_clk,
				exynos5_busfreq_table[i].volt);
		if (ret) {
			dev_err(dev, "Fail to add opp entries.\n");
			return ret;
		}
	}

	data->table = exynos5_busfreq_table;
	data->table_size = LV_END;

	/* Find max frequency */
	data->max_opp = opp_find_freq_floor(dev, &maxfreq);
	data->min_opp = opp_find_freq_ceil(dev, &minfreq);

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
