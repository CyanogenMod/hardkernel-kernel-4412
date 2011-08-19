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
#include <plat/cputype.h>

static struct exynos_dvfs_info *exynos_info;

static struct regulator *arm_regulator;
static struct cpufreq_freqs freqs;

static bool exynos4_cpufreq_init_done;
static DEFINE_MUTEX(set_freq_lock);
static DEFINE_MUTEX(set_cpu_freq_lock);

#define MASK_ONLY_SET_CPUFREQ		0x40

unsigned int g_cpufreq_limit_id;
unsigned int g_cpufreq_limit_val[DVFS_LOCK_ID_END];
unsigned int g_cpufreq_limit_level;

unsigned int g_cpufreq_lock_id;
unsigned int g_cpufreq_lock_val[DVFS_LOCK_ID_END];
unsigned int g_cpufreq_lock_level;

int exynos4_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy,
					      exynos_info->freq_table);
}

unsigned int exynos4_getspeed(unsigned int cpu)
{
	return clk_get_rate(exynos_info->cpu_clk) / 1000;
}

static int exynos4_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	unsigned int index, old_index;
	unsigned int arm_volt, safe_arm_volt = 0;
	int ret = 0;
	struct cpufreq_frequency_table *freq_table = exynos_info->freq_table;
	unsigned int *volt_table = exynos_info->volt_table;
	unsigned int mpll_freq_khz = exynos_info->mpll_freq_khz;

	mutex_lock(&set_freq_lock);

	freqs.old = policy->cur;

	if (cpufreq_frequency_table_target(policy, freq_table,
					   freqs.old, relation, &old_index)) {
		ret = -EINVAL;
		goto out;
	}

	if (cpufreq_frequency_table_target(policy, freq_table,
					   target_freq, relation, &index)) {
		ret = -EINVAL;
		goto out;
	}

	/* Need to set performance limitation */
	if (index > g_cpufreq_lock_level)
		index = g_cpufreq_lock_level;

	if (index < g_cpufreq_limit_level)
		index = g_cpufreq_limit_level;

	freqs.new = freq_table[index].frequency;
	freqs.cpu = policy->cpu;

	/*
	 * ARM clock source will be changed APLL to MPLL temporary
	 * To support this level, need to control regulator for
	 * required voltage level
	 */
	if (exynos_info->need_apll_change != NULL) {
		if (exynos_info->need_apll_change(old_index, index) &&
		   (freq_table[index].frequency < mpll_freq_khz) &&
		   (freq_table[old_index].frequency < mpll_freq_khz))
			safe_arm_volt = volt_table[exynos_info->pll_safe_idx];
	}
	arm_volt = volt_table[index];

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
		exynos_info->set_freq(old_index, index);

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
	unsigned int *volt_table = exynos_info->volt_table;
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	struct cpufreq_frequency_table *freq_table = exynos_info->freq_table;

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
	freq_old = policy->cur;
	freq_new = freq_table[cpufreq_level].frequency;
	if (freq_old < freq_new) {
		/* Find out current level index */
		for (i = 0 ; i <= exynos_info->min_support_idx;  i++) {
			if (freq_old == freq_table[i].frequency) {
				old_idx = freq_table[i].index;
				break;
			} else if (i == exynos_info->min_support_idx) {
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
		arm_volt = volt_table[cpufreq_level];
		regulator_set_voltage(arm_regulator, arm_volt,
				arm_volt);

		exynos_info->set_freq(old_idx, cpufreq_level);
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
	g_cpufreq_lock_val[nId] = exynos_info->min_support_idx;
	g_cpufreq_lock_level = exynos_info->min_support_idx;
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
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	struct cpufreq_frequency_table *freq_table = exynos_info->freq_table;

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
	cur_freq = policy->cur;
	if (cur_freq > freq_table[cpufreq_level].frequency) {
		ret = cpufreq_driver_target(cpufreq_cpu_get(cpu),
					    freq_table[cpufreq_level].frequency,
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
	g_cpufreq_limit_val[nId] = exynos_info->max_support_idx;
	g_cpufreq_limit_level = exynos_info->max_support_idx;

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
		ret = exynos4_cpufreq_lock(DVFS_LOCK_ID_PM,
					   exynos_info->pm_lock_idx);
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

	cpufreq_frequency_table_get_attr(exynos_info->freq_table, policy->cpu);

	/* set the transition latency value */
	policy->cpuinfo.transition_latency = 100000;

	/*
	 * EXYNOS4 multi-core processors has 2 cores
	 * that the frequency cannot be set independently.
	 * Each cpu is bound to the same speed.
	 * So the affected cpu is all of the cpus.
	 */
	if (num_online_cpus() == 1) {
		cpumask_copy(policy->related_cpus, cpu_possible_mask);
		cpumask_copy(policy->cpus, cpu_online_mask);
	} else {
		cpumask_setall(policy->cpus);
	}

	return cpufreq_frequency_table_cpuinfo(policy, exynos_info->freq_table);
}

static int exynos4_cpufreq_reboot_notifier_call(struct notifier_block *this,
				   unsigned long code, void *_cmd)
{
	int ret = 0;

	ret = exynos4_cpufreq_lock(DVFS_LOCK_ID_PM, exynos_info->pm_lock_idx);
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

static int __init exynos_cpufreq_init(void)
{
	int ret = -EINVAL;

	exynos_info = kzalloc(sizeof(*exynos_info), GFP_KERNEL);
	if (!exynos_info)
		return -ENOMEM;

	if (cpu_is_exynos4210())
		ret = exynos4210_cpufreq_init(exynos_info);
	else if (cpu_is_exynos4212())
		ret = exynos4212_cpufreq_init(exynos_info);
	else
		printk(KERN_ERR "%s: no cpu type defined\n", __func__);

	if (ret)
		goto err_vdd_arm;

	if (exynos_info->set_freq == NULL) {
		printk(KERN_ERR "%s: No set_freq function (ERR)\n",
				__func__);
		goto err_vdd_arm;
	}

	arm_regulator = regulator_get(NULL, "vdd_arm");
	if (IS_ERR(arm_regulator)) {
		printk(KERN_ERR "failed to get resource %s\n", "vdd_arm");
		goto err_vdd_arm;
	}

	register_pm_notifier(&exynos4_cpufreq_notifier);
	register_reboot_notifier(&exynos4_cpufreq_reboot_notifier);

	exynos4_cpufreq_init_done = true;

	if (cpufreq_register_driver(&exynos4_driver)) {
		pr_err("failed to register cpufreq driver\n");
		goto err_cpufreq;
	}

	g_cpufreq_lock_level = exynos_info->min_support_idx;
	g_cpufreq_limit_level = exynos_info->max_support_idx;

	return 0;
err_cpufreq:
	unregister_reboot_notifier(&exynos4_cpufreq_reboot_notifier);
	unregister_pm_notifier(&exynos4_cpufreq_notifier);

	if (!IS_ERR(arm_regulator))
		regulator_put(arm_regulator);
err_vdd_arm:
	kfree(exynos_info);
	pr_debug("%s: failed initialization\n", __func__);
	return -EINVAL;
}
late_initcall(exynos_cpufreq_init);
