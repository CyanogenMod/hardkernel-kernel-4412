/* linux/arch/arm/mach-exynos4/cpufreq.c
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4 - CPU frequency scaling support
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
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/cpufreq.h>
#include <linux/suspend.h>
#include <linux/reboot.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-mem.h>
#include <mach/cpufreq.h>

#include <plat/clock.h>
#include <plat/pm.h>

static struct clk *cpu_clk;
static struct clk *moutcore;
static struct clk *mout_mpll;
static struct clk *mout_apll;
static unsigned long mpll_freq_khz;

static struct regulator *arm_regulator;
static struct cpufreq_freqs freqs;

static int exynos4_dvs_locking;
static bool exynos4_cpufreq_init_done;
static DEFINE_MUTEX(set_freq_lock);
static DEFINE_MUTEX(set_cpu_freq_lock);

struct cpufreq_clkdiv {
	unsigned int	index;
	unsigned int	clkdiv;
};

enum cpufreq_level_index {
	L0, L1, L2, L3, L4, L5, CPUFREQ_LEVEL_END,
};

static struct cpufreq_frequency_table exynos4_freq_table[] = {
	{L0, 1400*1000},
	{L1, 1200*1000},
	{L2, 1000*1000},
	{L3, 800*1000},
	{L4, 500*1000},
	{L5, 200*1000},
	{0, CPUFREQ_TABLE_END},
};

/*
 * These index value should be defined by depending on
 * supporting frequency table
 */
static unsigned int pll_safe_idx;
static unsigned int pm_lock_idx;
static unsigned int max_support_level;

static struct cpufreq_clkdiv exynos4_clkdiv_table[] = {
	{L0, 0},
	{L1, 0},
	{L2, 0},
	{L3, 0},
	{L4, 0},
	{L5, 0},
};

#define MASK_ONLY_SET_CPUFREQ		0x40

unsigned int g_cpufreq_limit_id;
unsigned int g_cpufreq_limit_val[DVFS_LOCK_ID_END];
unsigned int g_cpufreq_limit_level;

/* This defines are for cpufreq lock */
#define CPUFREQ_MIN_LEVEL	(CPUFREQ_LEVEL_END - 1)

unsigned int g_cpufreq_lock_id;
unsigned int g_cpufreq_lock_val[DVFS_LOCK_ID_END];
unsigned int g_cpufreq_lock_level = CPUFREQ_MIN_LEVEL;

static unsigned int clkdiv_cpu0[CPUFREQ_LEVEL_END][7] = {
	/*
	 * Clock divider value for following
	 * { DIVCORE, DIVCOREM0, DIVCOREM1, DIVPERIPH,
	 *		DIVATB, DIVPCLK_DBG, DIVAPLL }
	 */
	/* ARM L0: 1400MHz */
	{ 0, 3, 7, 3, 4, 1, 7 },

	/* ARM L1: 1200MHz */
	{ 0, 3, 7, 3, 4, 1, 7 },

	/* ARM L2: 1000MHz */
	{ 0, 3, 7, 3, 4, 1, 7 },

	/* ARM L3: 800MHz */
	{ 0, 3, 7, 3, 3, 1, 7 },

	/* ARM L4: 500MHz */
	{ 0, 3, 7, 3, 3, 1, 7 },

	/* ARM L5: 200MHz */
	{ 0, 1, 3, 1, 3, 1, 0 },
};

static unsigned int clkdiv_cpu1[CPUFREQ_LEVEL_END][2] = {
	/* Clock divider value for following
	 * { DIVCOPY, DIVHPM }
	 */
	/* ARM L0: 1400MHz */
	{ 5, 0 },

	/* ARM L1: 1200MHz */
	{ 5, 0 },

	/* ARM L2: 1000MHz */
	{ 4, 0 },

	/* ARM L3: 800MHz */
	{ 3, 0 },

	/* ARM L4: 500MHz */
	{ 3, 0 },

	/* ARM L5: 200MHz */
	{ 3, 0 },
};

static unsigned int exynos4_apll_pms_table[CPUFREQ_LEVEL_END] = {
	/* APLL FOUT L0: 1400MHz */
	((350<<16)|(6<<8)|(0x1)),

	/* APLL FOUT L1: 1200MHz */
	((150<<16)|(3<<8)|(0x1)),

	/* APLL FOUT L2: 1000MHz */
	((250<<16)|(6<<8)|(0x1)),

	/* APLL FOUT L3: 800MHz */
	((200<<16)|(6<<8)|(0x1)),

	/* APLL FOUT L4: 500MHz */
	((250<<16)|(6<<8)|(0x2)),

	/* APLL FOUT L5: 200MHz */
	((200<<16)|(6<<8)|(0x3)),
};

/*
 * ASV group voltage table
 */

static const unsigned int asv_voltage_A[CPUFREQ_LEVEL_END][8] = {
	/*
	 *	   SS, A1, A2, B1, B2, C1, C2, D
	 * @Dummy:
	 * @1200 :
	 * @1000 :
	 * @800	 :	ASV_VOLTAGE_TABLE
	 * @500  :
	 * @200  :
	 */
	{ 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1350000, 1350000, 1300000, 1275000, 1250000, 1225000, 1200000, 1175000 },
	{ 1300000, 1250000, 1200000, 1175000, 1150000, 1125000, 1100000, 1075000 },
	{ 1200000, 1150000, 1100000, 1075000, 1050000, 1025000, 1000000, 975000 },
	{ 1100000, 1050000, 1000000, 975000, 975000, 950000, 925000, 925000 },
	{ 1050000, 1000000, 975000, 950000, 950000, 925000, 925000, 925000 },

};

static const unsigned int asv_voltage_B[CPUFREQ_LEVEL_END][5] = {
	/*
	 *	   S, A, B, C, D
	 * @1400 :
	 * @1200 :
	 * @1000 :
	 * @800	 :	ASV_VOLTAGE_TABLE
	 * @500	 :
	 * @200	 :
	 */
	{ 1350000, 1350000, 1300000, 1250000, 1225000 },
	{ 1300000, 1250000, 1200000, 1150000, 1125000 },
	{ 1200000, 1150000, 1100000, 1050000, 1025000 },
	{ 1125000, 1075000, 1025000, 975000, 950000 },
	{ 1050000, 1000000, 950000, 950000, 950000 },
	{ 1050000, 1000000, 950000, 950000, 950000 },

};

static unsigned int exynos4_volt_table[CPUFREQ_LEVEL_END];

int exynos4_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, exynos4_freq_table);
}

unsigned int exynos4_getspeed(unsigned int cpu)
{
	return clk_get_rate(cpu_clk) / 1000;
}

void exynos4_set_clkdiv(unsigned int div_index)
{
	unsigned int tmp;
	/* Change Divider - CPU0 */

	tmp = exynos4_clkdiv_table[div_index].clkdiv;

	__raw_writel(tmp, S5P_CLKDIV_CPU);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STATCPU);
	} while (tmp & 0x1111111);

	/* Change Divider - CPU1 */
	tmp = __raw_readl(S5P_CLKDIV_CPU1);

	tmp &= ~((0x7 << 4) | (0x7));

	tmp |= ((clkdiv_cpu1[div_index][0] << 4) |
		(clkdiv_cpu1[div_index][1] << 0));

	__raw_writel(tmp, S5P_CLKDIV_CPU1);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STATCPU1);
	} while (tmp & 0x11);
}

static void exynos4_set_apll(unsigned int new_index,
			     unsigned int old_index)
{
	unsigned int tmp;

	/* 1. MUX_CORE_SEL = MPLL,
	 * ARMCLK uses MPLL for lock time */
	clk_set_parent(moutcore, mout_mpll);

	do {
		tmp = (__raw_readl(S5P_CLKMUX_STATCPU)
			>> S5P_CLKSRC_CPU_MUXCORE_SHIFT);
		tmp &= 0x7;
	} while (tmp != 0x2);

	/* 2. Set APLL Lock time */
	__raw_writel(S5P_APLL_LOCKTIME, S5P_APLL_LOCK);

	/* 3. Change PLL PMS values */
	tmp = __raw_readl(S5P_APLL_CON0);
	tmp &= ~((0x3ff << 16) | (0x3f << 8) | (0x7 << 0));
	tmp |= exynos4_apll_pms_table[new_index];
	__raw_writel(tmp, S5P_APLL_CON0);

	/* 4. wait_lock_time */
	do {
		tmp = __raw_readl(S5P_APLL_CON0);
	} while (!(tmp & (0x1 << S5P_APLLCON0_LOCKED_SHIFT)));

	/* 5. MUX_CORE_SEL = APLL */
	clk_set_parent(moutcore, mout_apll);

	do {
		tmp = __raw_readl(S5P_CLKMUX_STATCPU);
		tmp &= S5P_CLKMUX_STATCPU_MUXCORE_MASK;
	} while (tmp != (0x1 << S5P_CLKSRC_CPU_MUXCORE_SHIFT));

}

static inline int need_pms_change(unsigned int old_index, unsigned int new_index)
{
	unsigned int old_pm = (exynos4_apll_pms_table[old_index] >> 8);
	unsigned int new_pm = (exynos4_apll_pms_table[new_index] >> 8);

	return (old_pm == new_pm) ? 0 : 1;
}

static void exynos4_set_frequency(unsigned int old_index,
				  unsigned int new_index)
{
	unsigned int tmp;

	if (old_index > new_index) {
		if (!need_pms_change(old_index, new_index)) {
			/* 1. Change the system clock divider values */
			exynos4_set_clkdiv(new_index);
			/* 2. Change just s value in apll m,p,s value */
			tmp = __raw_readl(S5P_APLL_CON0);
			tmp &= ~(0x7 << 0);
			tmp |= (exynos4_apll_pms_table[new_index] & 0x7);
			__raw_writel(tmp, S5P_APLL_CON0);

		} else {
			/* Clock Configuration Procedure */
			/* 1. Change the system clock divider values */
			exynos4_set_clkdiv(new_index);
			/* 2. Change the apll m,p,s value */
			exynos4_set_apll(new_index, old_index);
		}
	} else if (old_index < new_index) {
		if (!need_pms_change(old_index, new_index)) {
			/* 1. Change just s value in apll m,p,s value */
			tmp = __raw_readl(S5P_APLL_CON0);
			tmp &= ~(0x7 << 0);
			tmp |= (exynos4_apll_pms_table[new_index] & 0x7);
			__raw_writel(tmp, S5P_APLL_CON0);
			/* 2. Change the system clock divider values */
			exynos4_set_clkdiv(new_index);
		} else {
			/* Clock Configuration Procedure */
			/* 1. Change the apll m,p,s value */
			exynos4_set_apll(new_index, old_index);
			/* 2. Change the system clock divider values */
			exynos4_set_clkdiv(new_index);
		}
	}
}

static int exynos4_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	unsigned int index, old_index;
	unsigned int arm_volt, safe_arm_volt = 0;
	int ret = 0;

	mutex_lock(&set_freq_lock);

	freqs.old = exynos4_getspeed(policy->cpu);

	if (cpufreq_frequency_table_target(policy, exynos4_freq_table,
					   freqs.old, relation, &old_index)) {
		ret = -EINVAL;
		goto out;
	}

	if (cpufreq_frequency_table_target(policy, exynos4_freq_table,
					   target_freq, relation, &index)) {
		ret = -EINVAL;
		goto out;
	}

	/* Need to set performance limitation */
	if (index > g_cpufreq_lock_level)
		index = g_cpufreq_lock_level;

	if (index < g_cpufreq_limit_level)
		index = g_cpufreq_limit_level;

	freqs.new = exynos4_freq_table[index].frequency;
	freqs.cpu = policy->cpu;

	if (freqs.new == freqs.old) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * ARM clock source will be changed APLL to MPLL temporary
	 * To support this level, need to control regulator for
	 * required voltage level
	 */
	if (need_pms_change(old_index, index) &&
	   (exynos4_freq_table[index].frequency < mpll_freq_khz) &&
	   (exynos4_freq_table[old_index].frequency < mpll_freq_khz))
		safe_arm_volt = exynos4_volt_table[pll_safe_idx];

	arm_volt = exynos4_volt_table[index];

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	/* When the new frequency is higher than current frequency */
	if ((freqs.new > freqs.old) && !safe_arm_volt) {
		/* Firstly, voltage up to increase frequency */
		regulator_set_voltage(arm_regulator, arm_volt,
				arm_volt);
	}

	if (safe_arm_volt)
		regulator_set_voltage(arm_regulator, safe_arm_volt,
				      safe_arm_volt);
	if (freqs.new != freqs.old)
		exynos4_set_frequency(old_index, index);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	/* When the new frequency is lower than current frequency */
	if ((freqs.new < freqs.old) ||
	   ((freqs.new > freqs.old) && safe_arm_volt)) {
		/* down the voltage after frequency change */
		regulator_set_voltage(arm_regulator, arm_volt,
				arm_volt);
	}

out:
	mutex_unlock(&set_freq_lock);

	return ret;
}

atomic_t exynos4_cpufreq_lock_count;

int exynos4_cpufreq_lock(unsigned int nId,
			 enum cpufreq_level_request cpufreq_level)
{
	int ret = 0, i, old_idx = 0;
	unsigned int freq_old, freq_new, arm_volt;

	if (!exynos4_cpufreq_init_done)
		return 0;

	if (g_cpufreq_lock_id & (1 << nId)) {
		printk(KERN_ERR "%s:Device [%d] already locked cpufreq\n",
				__func__,  nId);
		return 0;
	}
	mutex_lock(&set_cpu_freq_lock);
	g_cpufreq_lock_id |= (1 << nId);
	g_cpufreq_lock_val[nId] = cpufreq_level;

	/* If the requested cpufreq is higher than current min frequency */
	if (cpufreq_level < g_cpufreq_lock_level)
		g_cpufreq_lock_level = cpufreq_level;

	mutex_unlock(&set_cpu_freq_lock);

	/* If current frequency is lower than requested freq,
	 * it needs to update
	 */
	mutex_lock(&set_freq_lock);
	freq_old = exynos4_getspeed(0);
	freq_new = exynos4_freq_table[cpufreq_level].frequency;
	if (freq_old < freq_new) {
		/* Find out current level index */
		for (i = 0 ; i < CPUFREQ_LEVEL_END ; i++) {
			if (freq_old == exynos4_freq_table[i].frequency) {
				old_idx = exynos4_freq_table[i].index;
				break;
			} else if (i == (CPUFREQ_LEVEL_END - 1)) {
				printk(KERN_ERR "%s: Level not found\n",
					__func__);
				mutex_unlock(&set_freq_lock);
				return -EINVAL;
			} else {
				continue;
			}
		}
		freqs.old = freq_old;
		freqs.new = freq_new;
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

		/* get the voltage value */
		arm_volt = exynos4_volt_table[cpufreq_level];
		regulator_set_voltage(arm_regulator, arm_volt,
				arm_volt);

		exynos4_set_frequency(old_idx, cpufreq_level);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
	mutex_unlock(&set_freq_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos4_cpufreq_lock);

void exynos4_cpufreq_lock_free(unsigned int nId)
{
	unsigned int i;

	if (!exynos4_cpufreq_init_done)
		return;

	mutex_lock(&set_cpu_freq_lock);
	g_cpufreq_lock_id &= ~(1 << nId);
	g_cpufreq_lock_val[nId] = CPUFREQ_MIN_LEVEL;
	g_cpufreq_lock_level = CPUFREQ_MIN_LEVEL;
	if (g_cpufreq_lock_id) {
		for (i = 0; i < DVFS_LOCK_ID_END; i++) {
			if (g_cpufreq_lock_val[i] < g_cpufreq_lock_level)
				g_cpufreq_lock_level = g_cpufreq_lock_val[i];
		}
	}
	mutex_unlock(&set_cpu_freq_lock);
}
EXPORT_SYMBOL_GPL(exynos4_cpufreq_lock_free);

int exynos4_cpufreq_upper_limit(unsigned int nId,
				enum cpufreq_level_request cpufreq_level)
{
	int ret = 0, cpu = 0;
	unsigned int cur_freq;

	if (!exynos4_cpufreq_init_done)
		return 0;

	if (g_cpufreq_limit_id & (1 << nId)) {
		printk(KERN_ERR "[CPUFREQ]This device [%d] already limited cpufreq\n", nId);
		return 0;
	}

	mutex_lock(&set_cpu_freq_lock);
	g_cpufreq_limit_id |= (1 << nId);
	g_cpufreq_limit_val[nId] = cpufreq_level;

	/* If the requested limit level is lower than current value */
	if (cpufreq_level > g_cpufreq_limit_level)
		g_cpufreq_limit_level = cpufreq_level;

	mutex_unlock(&set_cpu_freq_lock);

	/* If cur frequency is higher than limit freq, it needs to update */
	cur_freq = exynos4_getspeed(cpu);
	if (cur_freq > exynos4_freq_table[cpufreq_level].frequency) {
		ret = cpufreq_driver_target(cpufreq_cpu_get(cpu),
				exynos4_freq_table[cpufreq_level].frequency,
				MASK_ONLY_SET_CPUFREQ);
	}

	return ret;
}

void exynos4_cpufreq_upper_limit_free(unsigned int nId)
{
	unsigned int i;

	if (!exynos4_cpufreq_init_done)
		return;

	mutex_lock(&set_cpu_freq_lock);
	g_cpufreq_limit_id &= ~(1 << nId);
	g_cpufreq_limit_val[nId] = max_support_level;
	g_cpufreq_limit_level = max_support_level;

	if (g_cpufreq_limit_id) {
		for (i = 0; i < DVFS_LOCK_ID_END; i++) {
			if (g_cpufreq_limit_val[i] > g_cpufreq_limit_level)
				g_cpufreq_limit_level = g_cpufreq_limit_val[i];
		}
	}
	mutex_unlock(&set_cpu_freq_lock);
}

#ifdef CONFIG_PM
static int exynos4_cpufreq_suspend(struct cpufreq_policy *policy)
{
	return 0;
}

static int exynos4_cpufreq_resume(struct cpufreq_policy *policy)
{
	return 0;
}
#endif

static int exynos4_cpufreq_notifier_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	int ret = 0;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		ret = exynos4_cpufreq_lock(DVFS_LOCK_ID_PM, pm_lock_idx);
		if (ret < 0)
			return NOTIFY_BAD;
		pr_debug("PM_SUSPEND_PREPARE for CPUFREQ\n");
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		pr_debug("PM_POST_SUSPEND for CPUFREQ: %d\n", ret);
		exynos4_cpufreq_lock_free(DVFS_LOCK_ID_PM);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block exynos4_cpufreq_notifier = {
	.notifier_call = exynos4_cpufreq_notifier_event,
};

static int exynos4_cpufreq_cpu_init(struct cpufreq_policy *policy)
{
	policy->cur = policy->min = policy->max = exynos4_getspeed(policy->cpu);

	cpufreq_frequency_table_get_attr(exynos4_freq_table, policy->cpu);

	/* set the transition latency value */
	policy->cpuinfo.transition_latency = 100000;

	/*
	 * EXYNOS4 multi-core processors has 2 cores
	 * that the frequency cannot be set independently.
	 * Each cpu is bound to the same speed.
	 * So the affected cpu is all of the cpus.
	 */
	if (!cpu_online(1)) {
		cpumask_copy(policy->related_cpus, cpu_possible_mask);
		cpumask_copy(policy->cpus, cpu_online_mask);
	} else {
		cpumask_setall(policy->cpus);
	}

	return cpufreq_frequency_table_cpuinfo(policy, exynos4_freq_table);
}

static int exynos4_cpufreq_reboot_notifier_call(struct notifier_block *this,
				   unsigned long code, void *_cmd)
{
	int ret = 0;

	ret = exynos4_cpufreq_lock(DVFS_LOCK_ID_PM, pm_lock_idx);
	if (ret < 0)
		return NOTIFY_BAD;

	printk(KERN_INFO "REBOOT Notifier for CPUFREQ\n");
	return NOTIFY_DONE;
}

static struct notifier_block exynos4_cpufreq_reboot_notifier = {
	.notifier_call = exynos4_cpufreq_reboot_notifier_call,
};

static struct cpufreq_driver exynos4_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= exynos4_verify_speed,
	.target		= exynos4_target,
	.get		= exynos4_getspeed,
	.init		= exynos4_cpufreq_cpu_init,
	.name		= "exynos4_cpufreq",
#ifdef CONFIG_PM
	.suspend	= exynos4_cpufreq_suspend,
	.resume		= exynos4_cpufreq_resume,
#endif
};

static void __init exynos4_set_voltage(void)
{
	unsigned int asv_group = 0;
	bool for_1400 = false, for_1200 = false, for_1000 = false;
	unsigned int tmp;
	unsigned int i;

	tmp = __raw_readl(S5P_INFORM2);

	asv_group = (tmp & 0xF);

	switch (tmp  & (SUPPORT_FREQ_MASK << SUPPORT_FREQ_SHIFT)) {
	case SUPPORT_1400MHZ:
		for_1400 = true;
		max_support_level = L0;
		break;
	case SUPPORT_1200MHZ:
		for_1200 = true;
		max_support_level = L1;
		break;
	case SUPPORT_1000MHZ:
		for_1000 = true;
		max_support_level = L2;
		break;
	default:
		for_1000 = true;
		max_support_level = L2;
		break;
	}

	/*
	 * If ASV group is S, can not support 1.4GHz
	 * Disabling table entry
	 */
	if ((asv_group == 0) || !for_1400)
		exynos4_freq_table[L0].frequency = CPUFREQ_ENTRY_INVALID;

	if (for_1000)
		exynos4_freq_table[L1].frequency = CPUFREQ_ENTRY_INVALID;

	/*
	 * Set lock level for PM and reset notifier
	 * This level should be higher than boot level
	 */
	pm_lock_idx = L2;

	/*
	 * APLL -> MPLL -> APLL change safe level
	 * This level should be higher than 800MHz (MPLL freq)
	 */
	pll_safe_idx = L2;

	printk(KERN_INFO "DVFS : VDD_ARM Voltage table set with %d Group\n", asv_group);

	if (for_1400) {
		for (i = 0 ; i < CPUFREQ_LEVEL_END ; i++) {
				exynos4_volt_table[i] =
					asv_voltage_B[i][asv_group];
		}
	} else {
		for (i = 0 ; i < CPUFREQ_LEVEL_END ; i++) {
				exynos4_volt_table[i] =
					asv_voltage_A[i][asv_group];
		}
	}
}

static int __init exynos4_cpufreq_init(void)
{
	int i;
	unsigned int tmp;

	exynos4_set_voltage();

	cpu_clk = clk_get(NULL, "armclk");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	moutcore = clk_get(NULL, "moutcore");
	if (IS_ERR(moutcore))
		goto err_moutcore;

	mout_mpll = clk_get(NULL, "mout_mpll");
	if (IS_ERR(mout_mpll))
		goto err_mout_mpll;

	mpll_freq_khz = clk_get_rate(mout_mpll) / 1000;

	mout_apll = clk_get(NULL, "mout_apll");
	if (IS_ERR(mout_apll))
		goto err_mout_apll;

	arm_regulator = regulator_get(NULL, "vdd_arm");
	if (IS_ERR(arm_regulator)) {
		printk(KERN_ERR "failed to get resource %s\n", "vdd_arm");
		goto err_vdd_arm;
	}

	exynos4_dvs_locking = 0;

	register_pm_notifier(&exynos4_cpufreq_notifier);
	register_reboot_notifier(&exynos4_cpufreq_reboot_notifier);

	exynos4_cpufreq_init_done = true;

	tmp = __raw_readl(S5P_CLKDIV_CPU);

	for (i = L0; i <  CPUFREQ_LEVEL_END; i++) {
		tmp &= ~(S5P_CLKDIV_CPU0_CORE_MASK |
			S5P_CLKDIV_CPU0_COREM0_MASK |
			S5P_CLKDIV_CPU0_COREM1_MASK |
			S5P_CLKDIV_CPU0_PERIPH_MASK |
			S5P_CLKDIV_CPU0_ATB_MASK |
			S5P_CLKDIV_CPU0_PCLKDBG_MASK |
			S5P_CLKDIV_CPU0_APLL_MASK);

		tmp |= ((clkdiv_cpu0[i][0] << S5P_CLKDIV_CPU0_CORE_SHIFT) |
			(clkdiv_cpu0[i][1] << S5P_CLKDIV_CPU0_COREM0_SHIFT) |
			(clkdiv_cpu0[i][2] << S5P_CLKDIV_CPU0_COREM1_SHIFT) |
			(clkdiv_cpu0[i][3] << S5P_CLKDIV_CPU0_PERIPH_SHIFT) |
			(clkdiv_cpu0[i][4] << S5P_CLKDIV_CPU0_ATB_SHIFT) |
			(clkdiv_cpu0[i][5] << S5P_CLKDIV_CPU0_PCLKDBG_SHIFT) |
			(clkdiv_cpu0[i][6] << S5P_CLKDIV_CPU0_APLL_SHIFT));

		exynos4_clkdiv_table[i].clkdiv = tmp;
	}

	if (cpufreq_register_driver(&exynos4_driver)) {
		pr_err("failed to register cpufreq driver\n");
		goto err_cpufreq;
	}

	return 0;
err_cpufreq:
	unregister_reboot_notifier(&exynos4_cpufreq_reboot_notifier);
	unregister_pm_notifier(&exynos4_cpufreq_notifier);

	if (!IS_ERR(arm_regulator))
		regulator_put(arm_regulator);
err_vdd_arm:
	if (!IS_ERR(mout_apll))
		clk_put(mout_apll);
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
late_initcall(exynos4_cpufreq_init);
