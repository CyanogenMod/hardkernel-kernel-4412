/* linux/arch/arm/mach-exynos/busfreq_opp_4x12.c
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

static struct busfreq_table exynos4_busfreq_table[] = {
	{LV_0, 400200, 1100000, 0, 0, 0}, /* MIF : 400MHz INT : 200MHz */
	{LV_1, 267200, 1000000, 0, 0, 0}, /* MIF : 267MHz INT : 200MHz */
	{LV_2, 267160, 1000000, 0, 0, 0}, /* MIF : 267MHz INT : 160MHz */
	{LV_3, 160160, 950000, 0, 0, 0},  /* MIF : 160MHz INT : 160MHz */
	{LV_4, 133133, 950000, 0, 0, 0},  /* MIF : 133MHz INT : 133MHz */
	{LV_5, 100100, 950000, 0, 0, 0},  /* MIF : 100MHz INT : 100MHz */
};

#define ASV_GROUP	9
static unsigned int asv_group_index;

static unsigned int exynos4_asv_volt[ASV_GROUP][LV_END] = {
	/* 400      267      267      160     133     100 */
	{1050000, 925000,  925000,  900000, 900000, 875000}, /* ASV0 */
	{1050000, 925000,  925000,  900000, 900000, 875000}, /* ASV1 */
	{1025000, 925000,  925000,  900000, 900000, 875000}, /* ASV2 */
	{1025000, 900000,  900000,  875000, 875000, 875000}, /* ASV3 */
	{1025000, 900000,  900000,  875000, 875000, 850000}, /* ASV4 */
	{1025000, 900000,  900000,  875000, 850000, 850000}, /* ASV5 */
	{1025000, 900000,  900000,  850000, 850000, 850000}, /* ASV6 */
	{1025000, 900000,  900000,  850000, 850000, 850000}, /* ASV7 */
	{1025000, 900000,  900000,  850000, 850000, 850000}, /* ASV8 */
};

static unsigned int exynos4x12_int_volt[ASV_GROUP][LV_END] = {
	/* 200      200     160    160      133     100 */
	{1000000, 1000000, 950000, 950000, 925000, 900000}, /* ASV0 */
	{975000,  975000,  925000, 925000, 925000, 900000}, /* ASV1 */
	{950000,  950000,  925000, 925000, 900000, 875000}, /* ASV2 */
	{950000,  950000,  900000, 900000, 900000, 875000}, /* ASV3 */
	{925000,  925000,  875000, 875000, 875000, 875000}, /* ASV4 */
	{900000,  900000,  850000, 850000, 850000, 850000}, /* ASV5 */
	{900000,  900000,  850000, 850000, 850000, 850000}, /* ASV6 */
	{900000,  900000,  850000, 850000, 850000, 850000}, /* ASV7 */
	{900000,  900000,  850000, 850000, 850000, 850000}, /* ASV8 */
};

static unsigned int exynos4x12_timingrow[LV_END] = {
	0x34498691, 0x2336544C, 0x2336544C, 0x152432C7, 0x152432C7, 0x0D233206
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

	/* DMC L2: 266.7MHz */
	{4, 1, 1, 2, 1, 1},

	/* DMC L3: 160MHz */
	{5, 1, 1, 4, 1, 1},

	/* DMC L4: 133MHz */
	{5, 1, 1, 5, 1, 1},

	/* DMC L5: 100MHz */
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

	/* DMC L2: 266.7MHz */
	{4, 2, 1},

	/* DMC L3: 160MHz */
	{5, 4, 1},

	/* DMC L4: 133MHz */
	{5, 5, 1},

	/* DMC L5: 100MHz */
	{7, 7, 1},
};

static unsigned int clkdiv_top[LV_END][5] = {
	/*
	 * Clock divider value for following
	 * { DIVACLK266_GPS, DIVACLK100, DIVACLK160,
		DIVACLK133, DIVONENAND }
	 */

	/* ACLK_GDL/R L0: 200MHz */
	{2, 7, 4, 5, 1},

	/* ACLK_GDL/R L1: 200MHz */
	{2, 7, 4, 5, 1},

	/* ACLK_GDL/R L2: 160MHz */
	{4, 7, 5, 7, 1},

	/* ACLK_GDL/R L3: 160MHz */
	{4, 7, 5, 7, 1},

	/* ACLK_GDL/R L4: 133MHz */
	{4, 7, 5, 7, 1},

	/* ACLK_GDL/R L5: 100MHz */
	{7, 7, 7, 7, 1},
};

static unsigned int clkdiv_lr_bus[LV_END][2] = {
	/*
	 * Clock divider value for following
	 * { DIVGDL/R, DIVGPL/R }
	 */

	/* ACLK_GDL/R L0: 200MHz */
	{3, 1},

	/* ACLK_GDL/R L1: 200MHz */
	{3, 1},

	/* ACLK_GDL/R L2: 160MHz */
	{4, 1},

	/* ACLK_GDL/R L3: 160MHz */
	{4, 1},

	/* ACLK_GDL/R L4: 133MHz */
	{5, 1},

	/* ACLK_GDL/R L5: 100MHz */
	{7, 1},
};

static unsigned int clkdiv_sclkip[LV_END][3] = {
	/*
	 * Clock divider value for following
	 * { DIVMFC, DIVJPEG, DIVFIMC0~3}
	 */

	/* SCLK_MFC: 200MHz */
	{3, 3, 4},

	/* SCLK_MFC: 200MHz */
	{3, 3, 4},

	/* SCLK_MFC: 160MHz */
	{4, 4, 5},

	/* SCLK_MFC: 160MHz */
	{4, 4, 5},

	/* SCLK_MFC: 133MHz */
	{5, 5, 5},

	/* SCLK_MFC: 100MHz */
	{7, 7, 7},
};

static void exynos4x12_set_bus_volt(void)
{
	unsigned int i;

	asv_group_index = exynos_result_of_asv;

	if (asv_group_index == 0xff)
		asv_group_index = 0;

	printk(KERN_INFO "DVFS : VDD_INT Voltage table set with %d Group\n", asv_group_index);

	for (i = 0 ; i < LV_END ; i++)
		exynos4_busfreq_table[i].volt =
			exynos4_asv_volt[asv_group_index][i];

	return;
}

unsigned int exynos4x12_target(unsigned int div_index)
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

unsigned int exynos4x12_get_table_index(struct opp *opp)
{
	unsigned int index;

	for (index = LV_0; index < LV_END; index++)
		if (opp_get_freq(opp) == exynos4_busfreq_table[index].mem_clk)
			break;

	return index;
}

void exynos4x12_prepare(unsigned int index)
{
	unsigned int timing0;

	timing0 = __raw_readl(S5P_VA_DMC0 + TIMINGROW_OFFSET);
	timing0 |= exynos4x12_timingrow[index];
	__raw_writel(timing0, S5P_VA_DMC0 + TIMINGROW_OFFSET);
	__raw_writel(exynos4x12_timingrow[index], S5P_VA_DMC0 + TIMINGROW_OFFSET);
	__raw_writel(timing0, S5P_VA_DMC1 + TIMINGROW_OFFSET);
	__raw_writel(exynos4x12_timingrow[index], S5P_VA_DMC1 + TIMINGROW_OFFSET);
}

void exynos4x12_post(unsigned int index)
{
	unsigned int timing0;

	timing0 = __raw_readl(S5P_VA_DMC0 + TIMINGROW_OFFSET);
	timing0 |= exynos4x12_timingrow[index];
	__raw_writel(timing0, S5P_VA_DMC0 + TIMINGROW_OFFSET);
	__raw_writel(exynos4x12_timingrow[index], S5P_VA_DMC0 + TIMINGROW_OFFSET);
	__raw_writel(timing0, S5P_VA_DMC1 + TIMINGROW_OFFSET);
	__raw_writel(exynos4x12_timingrow[index], S5P_VA_DMC1 + TIMINGROW_OFFSET);
}

unsigned int exynos4x12_get_int_volt(unsigned long index)
{
	return exynos4x12_int_volt[asv_group_index][index];
}

struct opp *exynos4x12_monitor(struct busfreq_data *data)
{
	struct opp *opp = data->curr_opp;
	int i;
	unsigned int cpu_load_average = 0;
	unsigned int dmc0_load_average = 0;
	unsigned int dmc1_load_average = 0;
	unsigned long cpufreq = 0;
	unsigned long lockfreq;
	unsigned long dmc0freq;
	unsigned long dmc1freq;
	unsigned long newfreq;
	unsigned long currfreq = opp_get_freq(data->curr_opp) / 1000;
	unsigned long maxfreq = opp_get_freq(data->max_opp) / 1000;
	unsigned long cpu_load;
	unsigned long dmc0_load;
	unsigned long dmc1_load;
	int cpu_load_slope;

	ppmu_update(data->dev);

	/* Convert from base xxx to base maxfreq */
	cpu_load = ppmu_load[PPMU_CPU];
	dmc0_load = div64_u64(ppmu_load[PPMU_DMC0] * currfreq, maxfreq);
	dmc1_load = div64_u64(ppmu_load[PPMU_DMC1] * currfreq, maxfreq);

	cpu_load_slope = cpu_load -
		data->load_history[PPMU_CPU][data->index ? data->index - 1 : LOAD_HISTORY_SIZE - 1];

	data->load_history[PPMU_CPU][data->index] = cpu_load;
	data->load_history[PPMU_DMC0][data->index] = dmc0_load;
	data->load_history[PPMU_DMC1][data->index++] = dmc1_load;

	if (data->index >= LOAD_HISTORY_SIZE)
		data->index = 0;

	for (i = 0; i < LOAD_HISTORY_SIZE; i++) {
		cpu_load_average += data->load_history[PPMU_CPU][i];
		dmc0_load_average += data->load_history[PPMU_DMC0][i];
		dmc1_load_average += data->load_history[PPMU_DMC1][i];
	}

	/* Calculate average Load */
	cpu_load_average /= LOAD_HISTORY_SIZE;
	dmc0_load_average /= LOAD_HISTORY_SIZE;
	dmc1_load_average /= LOAD_HISTORY_SIZE;

	if (cpu_load >= UP_CPU_THRESHOLD) {
		cpufreq = opp_get_freq(data->max_opp);
		if (cpu_load < MAX_CPU_THRESHOLD) {
			opp = data->curr_opp;
			if (cpu_load_slope > CPU_SLOPE_SIZE) {
				cpufreq--;
				opp = opp_find_freq_floor(data->dev, &cpufreq);
			}
			cpufreq = opp_get_freq(opp);
		}
	}

	if (dmc0_load >= DMC_MAX_THRESHOLD || dmc1_load >= DMC_MAX_THRESHOLD) {
		newfreq = opp_get_freq(data->max_opp);
	} else if (dmc0_load < IDLE_THRESHOLD
			&& dmc1_load < IDLE_THRESHOLD) {
		if (dmc0_load_average < IDLE_THRESHOLD &&  dmc1_load_average < IDLE_THRESHOLD)
			opp = step_down(data, 1);
		else
			opp = data->curr_opp;
		newfreq = opp_get_freq(opp);
	} else {
		if (dmc0_load < dmc0_load_average) {
			dmc0_load = dmc0_load_average;
			if (dmc0_load >= DMC_MAX_THRESHOLD)
				dmc0_load = DMC_MAX_THRESHOLD;
		}
		dmc0freq = div64_u64(maxfreq * dmc0_load * 1000, DMC_MAX_THRESHOLD);

		if (dmc1_load < dmc1_load_average) {
			dmc1_load = dmc1_load_average;
			if (dmc1_load >= DMC_MAX_THRESHOLD)
				dmc1_load = DMC_MAX_THRESHOLD;
		}
		dmc1freq = div64_u64(maxfreq * dmc1_load * 1000, DMC_MAX_THRESHOLD);
		newfreq = max(dmc0freq, dmc1freq);
	}

	lockfreq = dev_max_freq(data->dev);

	newfreq = max3(lockfreq, newfreq, cpufreq);

	opp = opp_find_freq_ceil(data->dev, &newfreq);

	return opp;
}

int exynos4x12_init(struct device *dev, struct busfreq_data *data)
{
	unsigned int i;
	unsigned int tmp;
	unsigned long maxfreq = ULONG_MAX;
	unsigned long minfreq = 0;
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
			EXYNOS4_CLKDIV_DMC0_DMCP_MASK);

		tmp |= ((clkdiv_dmc0[i][0] << EXYNOS4_CLKDIV_DMC0_ACP_SHIFT) |
			(clkdiv_dmc0[i][1] << EXYNOS4_CLKDIV_DMC0_ACPPCLK_SHIFT) |
			(clkdiv_dmc0[i][2] << EXYNOS4_CLKDIV_DMC0_DPHY_SHIFT) |
			(clkdiv_dmc0[i][3] << EXYNOS4_CLKDIV_DMC0_DMC_SHIFT) |
			(clkdiv_dmc0[i][4] << EXYNOS4_CLKDIV_DMC0_DMCD_SHIFT) |
			(clkdiv_dmc0[i][5] << EXYNOS4_CLKDIV_DMC0_DMCP_SHIFT));

		exynos4_busfreq_table[i].clk_dmc0div = tmp;
	}

	exynos4x12_set_bus_volt();

	for (i = 0; i < LV_END; i++) {
		ret = opp_add(dev, exynos4_busfreq_table[i].mem_clk,
				exynos4_busfreq_table[i].volt);
		if (ret) {
			dev_err(dev, "Fail to add opp entries.\n");
			return ret;
		}
	}

	/* Disable MIF 267 INT 200 Level */
	opp_disable(dev, 267200);

	data->table = exynos4_busfreq_table;
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
