/* linux/arch/arm/mach-exynos/cpufreq-4212.c
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4212 - CPU frequency scaling support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/cpufreq.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-pmu.h>
#include <mach/cpufreq.h>

#include <plat/clock.h>
#include <plat/cputype.h>

#define CPUFREQ_LEVEL_END	(L8 + 1)

#undef PRINT_DIV_VAL

/* #define ENABLE_CLKOUT */

static int max_support_idx;
static int min_support_idx = (CPUFREQ_LEVEL_END - 1);
static struct clk *cpu_clk;
static struct clk *moutcore;
static struct clk *mout_mpll;
static struct clk *mout_apll;

struct cpufreq_clkdiv {
	unsigned int	index;
	unsigned int	clkdiv;
	unsigned int	clkdiv1;
};

static unsigned int exynos4212_volt_table[CPUFREQ_LEVEL_END];

static struct cpufreq_frequency_table exynos4212_freq_table[] = {
	{L0, 1500*1000},
	{L1, 1400*1000},
	{L2, 1300*1000},
	{L3, 1200*1000},
	{L4, 1100*1000},
	{L5, 1000*1000},
	{L6, 800*1000},
	{L7, 500*1000},
	{L8, 200*1000},
	{0, CPUFREQ_TABLE_END},
};

static struct cpufreq_clkdiv exynos4212_clkdiv_table[CPUFREQ_LEVEL_END];

static unsigned int clkdiv_cpu0_4212[CPUFREQ_LEVEL_END][7] = {
	/*
	 * Clock divider value for following
	 * { DIVCORE, DIVCOREM0, DIVCOREM1, DIVPERIPH,
	 *		DIVATB, DIVPCLK_DBG, DIVAPLL }
	 */
	/* ARM L0: 1500Mhz */
	{ 0, 6, 7, 7, 6, 1, 2 },

	/* ARM L1: 1400Mhz */
	{ 0, 6, 7, 7, 6, 1, 2 },

	/* ARM L2: 1300Mhz */
	{ 0, 5, 7, 7, 5, 1, 2 },

	/* ARM L3: 1200Mhz */
	{ 0, 5, 7, 7, 5, 1, 2 },

	/* ARM L4: 1100MHz */
	{ 0, 4, 7, 7, 4, 1, 2 },

	/* ARM L5: 1000MHz */
	{ 0, 4, 7, 7, 4, 1, 1 },

	/* ARM L6: 800MHz */
	{ 0, 3, 7, 7, 3, 1, 1 },

	/* ARM L7: 500MHz */
	{ 0, 2, 5, 7, 3, 1, 1 },

	/* ARM L8: 200MHz */
	{ 0, 1, 3, 7, 0, 1, 0 },
};

static unsigned int clkdiv_cpu0_4412[CPUFREQ_LEVEL_END][7] = {
	/*
	 * Clock divider value for following
	 * { DIVCORE, DIVCOREM0, DIVCOREM1, DIVPERIPH,
	 *		DIVATB, DIVPCLK_DBG, DIVAPLL }
	 */
	/* ARM L0: 1500Mhz */
	{ 0, 4, 7, 5, 6, 1, 2 },

	/* ARM L1: 1400Mhz */
	{ 0, 4, 7, 5, 6, 1, 2 },

	/* ARM L2: 1300Mhz */
	{ 0, 4, 7, 5, 5, 1, 2 },

	/* ARM L3: 1200Mhz */
	{ 0, 3, 7, 4, 5, 1, 2 },

	/* ARM L4: 1100MHz */
	{ 0, 3, 6, 4, 4, 1, 2 },

	/* ARM L5: 1000MHz */
	{ 0, 2, 5, 3, 4, 1, 1 },

	/* ARM L6: 800MHz */
	{ 0, 2, 5, 3, 3, 1, 1 },

	/* ARM L7: 500MHz */
	{ 0, 1, 3, 2, 3, 1, 1 },

	/* ARM L8: 200MHz */
	{ 0, 1, 3, 1, 3, 1, 0 },
};

static unsigned int clkdiv_cpu1_4212[CPUFREQ_LEVEL_END][2] = {
	/* Clock divider value for following
	 * { DIVCOPY, DIVHPM }
	 */
	/* ARM L0: 1500MHz */
	{ 6, 0 },

	/* ARM L1: 1400MHz */
	{ 6, 0 },

	/* ARM L2: 1300MHz */
	{ 5, 0 },

	/* ARM L3: 1200MHz */
	{ 5, 0 },

	/* ARM L4: 1100MHz */
	{ 4, 0 },

	/* ARM L5: 1000MHz */
	{ 4, 0 },

	/* ARM L6: 800MHz */
	{ 3, 0 },

	/* ARM L7: 500MHz */
	{ 2, 0 },

	/* ARM L8: 200MHz */
	{ 0, 0 },
};

static unsigned int clkdiv_cpu1_4412[CPUFREQ_LEVEL_END][3] = {
	/* Clock divider value for following
	 * { DIVCOPY, DIVHPM, DIVCORES }
	 */
	/* ARM L0: 1500MHz */
	{ 6, 0, 5 },

	/* ARM L1: 1400MHz */
	{ 6, 0, 5 },

	/* ARM L2: 1300MHz */
	{ 5, 0, 5 },

	/* ARM L3: 1200MHz */
	{ 5, 0, 4 },

	/* ARM L4: 1100MHz */
	{ 4, 0, 4 },

	/* ARM L5: 1000MHz */
	{ 4, 0, 3 },

	/* ARM L6: 800MHz */
	{ 3, 0, 3 },

	/* ARM L7: 500MHz */
	{ 2, 0, 2 },

	/* ARM L8: 200MHz */
	{ 0, 0, 1 },
};

static unsigned int exynos4_apll_pms_table[CPUFREQ_LEVEL_END] = {
	/* APLL FOUT L0: 1500MHz */
	((250<<16)|(4<<8)|(0x0)),

	/* APLL FOUT L1: 1400MHz */
	((175<<16)|(3<<8)|(0x0)),

	/* APLL FOUT L2: 1300MHz */
	((325<<16)|(6<<8)|(0x0)),

	/* APLL FOUT L3: 1200MHz */
	((150<<16)|(3<<8)|(0x0)),

	/* APLL FOUT L4: 1100MHz */
	((275<<16)|(6<<8)|(0x0)),

	/* APLL FOUT L5: 1000MHz */
	((125<<16)|(3<<8)|(0x0)),

	/* APLL FOUT L6: 800MHz */
	((100<<16)|(3<<8)|(0x0)),

	/* APLL FOUT L7: 500MHz */
	((125<<16)|(3<<8)|(0x1)),

	/* APLL FOUT L8: 200MHz */
	((100<<16)|(3<<8)|(0x2)),
};

/*
 * ASV group voltage table
 */

#define NUM_ASV_GROUP	1

static const unsigned int asv_voltage[CPUFREQ_LEVEL_END][NUM_ASV_GROUP] = {
	/*
	 * @1500 :
	 * @1200 :
	 * @1000 :
	 * @800	 :	ASV_VOLTAGE_TABLE
	 * @400  :
	 * @200  :
	 */
	{ 1400000 },
	{ 1350000 },
	{ 1300000 },
	{ 1250000 },
	{ 1200000 },
	{ 1125000 },
	{ 1025000 },
	{ 950000 },
	{ 950000 },
};

static void set_clkdiv(unsigned int div_index)
{
	unsigned int tmp;
	unsigned int stat_cpu1;

	/* Change Divider - CPU0 */

	tmp = exynos4212_clkdiv_table[div_index].clkdiv;

	__raw_writel(tmp, EXYNOS4_CLKDIV_CPU);

	do {
		tmp = __raw_readl(EXYNOS4_CLKDIV_STATCPU);
	} while (tmp & 0x1111111);

#ifdef PRINT_DIV_VAL
	tmp = __raw_readl(EXYNOS4_CLKDIV_CPU);
	pr_info("DIV_CPU0[0x%x]\n", tmp);

#endif

	/* Change Divider - CPU1 */
	tmp = exynos4212_clkdiv_table[div_index].clkdiv1;

	__raw_writel(tmp, EXYNOS4_CLKDIV_CPU1);
	if (cpu_is_exynos4212())
		stat_cpu1 = 0x11;
	else
		stat_cpu1 = 0x111;

	do {
		tmp = __raw_readl(EXYNOS4_CLKDIV_STATCPU1);
	} while (tmp & stat_cpu1);
#ifdef PRINT_DIV_VAL
	tmp = __raw_readl(EXYNOS4_CLKDIV_CPU1);
	pr_info("DIV_CPU1[0x%x]\n", tmp);
#endif
}

static void set_apll(unsigned int new_index,
			     unsigned int old_index)
{
	unsigned int tmp;

	/* 1. MUX_CORE_SEL = MPLL,
	 * ARMCLK uses MPLL for lock time */
	if (clk_set_parent(moutcore, mout_mpll))
		printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
				mout_mpll->name, moutcore->name);

	do {
		tmp = (__raw_readl(EXYNOS4_CLKMUX_STATCPU)
			>> EXYNOS4_CLKSRC_CPU_MUXCORE_SHIFT);
		tmp &= 0x7;
	} while (tmp != 0x2);

	/* 2. Set APLL Lock time */
	__raw_writel(EXYNOS4_APLL_LOCKTIME, EXYNOS4_APLL_LOCK);

	/* 3. Change PLL PMS values */
	tmp = __raw_readl(EXYNOS4_APLL_CON0);
	tmp &= ~((0x3ff << 16) | (0x3f << 8) | (0x7 << 0));
	tmp |= exynos4_apll_pms_table[new_index];
	__raw_writel(tmp, EXYNOS4_APLL_CON0);

	/* 4. wait_lock_time */
	do {
		tmp = __raw_readl(EXYNOS4_APLL_CON0);
	} while (!(tmp & (0x1 << EXYNOS4_APLLCON0_LOCKED_SHIFT)));

	/* 5. MUX_CORE_SEL = APLL */
	if (clk_set_parent(moutcore, mout_apll))
		printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
				mout_apll->name, moutcore->name);

	do {
		tmp = __raw_readl(EXYNOS4_CLKMUX_STATCPU);
		tmp &= EXYNOS4_CLKMUX_STATCPU_MUXCORE_MASK;
	} while (tmp != (0x1 << EXYNOS4_CLKSRC_CPU_MUXCORE_SHIFT));

}

bool exynos4212_pms_change(unsigned int old_index, unsigned int new_index)
{
	unsigned int old_pm = (exynos4_apll_pms_table[old_index] >> 8);
	unsigned int new_pm = (exynos4_apll_pms_table[new_index] >> 8);

	return (old_pm == new_pm) ? 0 : 1;
}

static void exynos4212_set_frequency(unsigned int old_index,
				  unsigned int new_index)
{
	unsigned int tmp;

	if (old_index > new_index) {
		if (!exynos4212_pms_change(old_index, new_index)) {
			/* 1. Change the system clock divider values */
			set_clkdiv(new_index);
			/* 2. Change just s value in apll m,p,s value */
			tmp = __raw_readl(EXYNOS4_APLL_CON0);
			tmp &= ~(0x7 << 0);
			tmp |= (exynos4_apll_pms_table[new_index] & 0x7);
			__raw_writel(tmp, EXYNOS4_APLL_CON0);

		} else {
			/* Clock Configuration Procedure */
			/* 1. Change the system clock divider values */
			set_clkdiv(new_index);
			/* 2. Change the apll m,p,s value */
			set_apll(new_index, old_index);
		}
	} else if (old_index < new_index) {
		if (!exynos4212_pms_change(old_index, new_index)) {
			/* 1. Change just s value in apll m,p,s value */
			tmp = __raw_readl(EXYNOS4_APLL_CON0);
			tmp &= ~(0x7 << 0);
			tmp |= (exynos4_apll_pms_table[new_index] & 0x7);
			__raw_writel(tmp, EXYNOS4_APLL_CON0);
			/* 2. Change the system clock divider values */
			set_clkdiv(new_index);
		} else {
			/* Clock Configuration Procedure */
			/* 1. Change the apll m,p,s value */
			set_apll(new_index, old_index);
			/* 2. Change the system clock divider values */
			set_clkdiv(new_index);
		}
	}
}

static void __init set_volt_table(void)
{
	unsigned int asv_group = 0;
	bool for_1500 = false, for_1000 = false;
	unsigned int i;
#if 0
	tmp = __raw_readl(EXYNOS4_INFORM2);

	asv_group = (tmp & 0xF);

	switch (tmp  & (SUPPORT_FREQ_MASK << SUPPORT_FREQ_SHIFT)) {
	case SUPPORT_1400MHZ:
		for_1400 = true;
		max_support_idx = L0;
		break;
	case SUPPORT_1200MHZ:
		for_1200 = true;
		max_support_idx = L1;
		break;
	case SUPPORT_1000MHZ:
		for_1000 = true;
		max_support_idx = L2;
		break;
	default:
		for_1000 = true;
		max_support_idx = L2;
		break;
	}
#else

#ifdef CONFIG_EXYNOS4212_1500MHZ_SUPPORT
	for_1500 = true;
	max_support_idx = L0;
	asv_group = 0;
#else
	for_1000 = true;
	max_support_idx = L5;
	asv_group = 0;

#endif

#endif
	/*
	 * Should be fixed !!!
	 */
#if 0
	if ((asv_group == 0) || !for_1400)
		exynos4212_freq_table[L0].frequency = CPUFREQ_ENTRY_INVALID;
#else
	if (!for_1500) {
		exynos4212_freq_table[L0].frequency = CPUFREQ_ENTRY_INVALID;
		exynos4212_freq_table[L1].frequency = CPUFREQ_ENTRY_INVALID;
		exynos4212_freq_table[L2].frequency = CPUFREQ_ENTRY_INVALID;
		exynos4212_freq_table[L3].frequency = CPUFREQ_ENTRY_INVALID;
		exynos4212_freq_table[L4].frequency = CPUFREQ_ENTRY_INVALID;
	}
#endif

	printk(KERN_INFO "DVFS : VDD_ARM Voltage table set with %d Group\n", asv_group);

	for (i = 0 ; i < CPUFREQ_LEVEL_END ; i++)
		exynos4212_volt_table[i] = asv_voltage[i][asv_group];
}

int exynos4212_cpufreq_init(struct exynos_dvfs_info *info)
{
	int i;
	unsigned int tmp;
	unsigned long rate;

	set_volt_table();

	cpu_clk = clk_get(NULL, "armclk");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	moutcore = clk_get(NULL, "moutcore");
	if (IS_ERR(moutcore))
		goto err_moutcore;

	mout_mpll = clk_get(NULL, "mout_mpll");
	if (IS_ERR(mout_mpll))
		goto err_mout_mpll;

	rate = clk_get_rate(mout_mpll) / 1000;

	mout_apll = clk_get(NULL, "mout_apll");
	if (IS_ERR(mout_apll))
		goto err_mout_apll;

	for (i = L0; i <  CPUFREQ_LEVEL_END; i++) {

		exynos4212_clkdiv_table[i].index = i;

		tmp = __raw_readl(EXYNOS4_CLKDIV_CPU);

		tmp &= ~(EXYNOS4_CLKDIV_CPU0_CORE_MASK |
			EXYNOS4_CLKDIV_CPU0_COREM0_MASK |
			EXYNOS4_CLKDIV_CPU0_COREM1_MASK |
			EXYNOS4_CLKDIV_CPU0_PERIPH_MASK |
			EXYNOS4_CLKDIV_CPU0_ATB_MASK |
			EXYNOS4_CLKDIV_CPU0_PCLKDBG_MASK |
			EXYNOS4_CLKDIV_CPU0_APLL_MASK);

		if (cpu_is_exynos4212()) {
			tmp |= ((clkdiv_cpu0_4212[i][0] << EXYNOS4_CLKDIV_CPU0_CORE_SHIFT) |
				(clkdiv_cpu0_4212[i][1] << EXYNOS4_CLKDIV_CPU0_COREM0_SHIFT) |
				(clkdiv_cpu0_4212[i][2] << EXYNOS4_CLKDIV_CPU0_COREM1_SHIFT) |
				(clkdiv_cpu0_4212[i][3] << EXYNOS4_CLKDIV_CPU0_PERIPH_SHIFT) |
				(clkdiv_cpu0_4212[i][4] << EXYNOS4_CLKDIV_CPU0_ATB_SHIFT) |
				(clkdiv_cpu0_4212[i][5] << EXYNOS4_CLKDIV_CPU0_PCLKDBG_SHIFT) |
				(clkdiv_cpu0_4212[i][6] << EXYNOS4_CLKDIV_CPU0_APLL_SHIFT));
		} else {
			tmp |= ((clkdiv_cpu0_4412[i][0] << EXYNOS4_CLKDIV_CPU0_CORE_SHIFT) |
				(clkdiv_cpu0_4412[i][1] << EXYNOS4_CLKDIV_CPU0_COREM0_SHIFT) |
				(clkdiv_cpu0_4412[i][2] << EXYNOS4_CLKDIV_CPU0_COREM1_SHIFT) |
				(clkdiv_cpu0_4412[i][3] << EXYNOS4_CLKDIV_CPU0_PERIPH_SHIFT) |
				(clkdiv_cpu0_4412[i][4] << EXYNOS4_CLKDIV_CPU0_ATB_SHIFT) |
				(clkdiv_cpu0_4412[i][5] << EXYNOS4_CLKDIV_CPU0_PCLKDBG_SHIFT) |
				(clkdiv_cpu0_4412[i][6] << EXYNOS4_CLKDIV_CPU0_APLL_SHIFT));

		}

		exynos4212_clkdiv_table[i].clkdiv = tmp;

		tmp = __raw_readl(EXYNOS4_CLKDIV_CPU1);

		if (cpu_is_exynos4212()) {
			tmp &= ~(EXYNOS4_CLKDIV_CPU1_COPY_MASK |
				EXYNOS4_CLKDIV_CPU1_HPM_MASK);
			tmp |= ((clkdiv_cpu1_4212[i][0] << EXYNOS4_CLKDIV_CPU1_COPY_SHIFT) |
				(clkdiv_cpu1_4212[i][1] << EXYNOS4_CLKDIV_CPU1_HPM_SHIFT));
		} else {
			tmp &= ~(EXYNOS4_CLKDIV_CPU1_COPY_MASK |
				EXYNOS4_CLKDIV_CPU1_HPM_MASK |
				EXYNOS4_CLKDIV_CPU1_CORES_MASK);
			tmp |= ((clkdiv_cpu1_4412[i][0] << EXYNOS4_CLKDIV_CPU1_COPY_SHIFT) |
				(clkdiv_cpu1_4412[i][1] << EXYNOS4_CLKDIV_CPU1_HPM_SHIFT) |
				(clkdiv_cpu1_4412[i][2] << EXYNOS4_CLKDIV_CPU1_CORES_SHIFT));
		}
		exynos4212_clkdiv_table[i].clkdiv1 = tmp;
	}

	info->mpll_freq_khz = rate;
	info->pm_lock_idx = L6;
	info->pll_safe_idx = L6;
	info->max_support_idx = max_support_idx;
	info->min_support_idx = min_support_idx;
	info->cpu_clk = cpu_clk;
	info->volt_table = exynos4212_volt_table;
	info->freq_table = exynos4212_freq_table;
	info->set_freq = exynos4212_set_frequency;
	info->need_apll_change = exynos4212_pms_change;

#ifdef ENABLE_CLKOUT
	tmp = __raw_readl(EXYNOS4_CLKOUT_CMU_CPU);
	tmp &= ~0x1f;
	tmp |= 0x304;
	__raw_writel(tmp, EXYNOS4_CLKOUT_CMU_CPU);

	tmp = __raw_readl(EXYNOS4_PMU_DEBUG);
	tmp &= ~0xf00;
	tmp |= 0x400;
	__raw_writel(tmp, EXYNOS4_PMU_DEBUG);

#endif

	return 0;

err_mout_apll:
	if (!IS_ERR(mout_mpll))
		clk_put(mout_mpll);
err_mout_mpll:
	if (!IS_ERR(moutcore))
		clk_put(moutcore);
err_moutcore:
	if (!IS_ERR(cpu_clk))
		clk_put(cpu_clk);

	pr_debug("%s: failed initialization\n", __func__);
	return -EINVAL;
}
EXPORT_SYMBOL(exynos4212_cpufreq_init);
