/* linux/arch/arm/mach-exynos4/dvfs-hotplug.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS4 - Integrated DVFS CPU hotplug
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/suspend.h>

static unsigned int total_num_target_freq;
static unsigned int consecutv_highestlevel_cnt;
static unsigned int consecutv_lowestlevel_cnt;
static unsigned int num_hotplug_in;
static unsigned int num_hotplug_out;

static unsigned int freq_max;
static unsigned int freq_min;

static void exynos4_integrated_dvfs_hotplug(unsigned int freq_old,
					unsigned int freq_new)
{
	total_num_target_freq++;
	if (((freq_old >= freq_max) && (freq_new >= freq_max))
				   && (cpu_online(1) == 0)) {
		if (consecutv_highestlevel_cnt >= 10) {
			cpu_up(1);
			num_hotplug_in++;
			consecutv_highestlevel_cnt = 0;
		} else
			consecutv_highestlevel_cnt++;
	} else if ((freq_old <= freq_min && freq_new <= freq_min)
				       && (cpu_online(1) == 1)) {
		if (consecutv_lowestlevel_cnt >= 10) {
			cpu_down(1);
			num_hotplug_out++;
			consecutv_lowestlevel_cnt = 0;
		} else
			consecutv_lowestlevel_cnt++;
	} else {
		consecutv_highestlevel_cnt = 0;
		consecutv_lowestlevel_cnt = 0;
	}
}

static int hotplug_cpufreq_transition(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct cpufreq_freqs *freqs = (struct cpufreq_freqs *)data;

	if (val == CPUFREQ_POSTCHANGE)
		exynos4_integrated_dvfs_hotplug(freqs->old, freqs->new);

	return 0;
}

static struct notifier_block dvfs_hotplug = {
	.notifier_call = hotplug_cpufreq_transition,
};

/*
 * Note : This function should be called after intialization of CPUFreq
 * driver for exynos4. The cpufreq_frequency_table for exynos4 should be
 * established before calling this function.
 */
static int __init exynos4_integrated_dvfs_hotplug_init(void)
{
	int i;
	struct cpufreq_frequency_table *table;

	total_num_target_freq = 0;
	consecutv_highestlevel_cnt = 0;
	consecutv_lowestlevel_cnt = 0;
	num_hotplug_in = 0;
	num_hotplug_out = 0;

	table = cpufreq_frequency_get_table(0);
	if (IS_ERR(table)) {
		printk(KERN_ERR "%s: Check loading cpufreq before\n", __func__);
		return PTR_ERR(table);
	}

	freq_max = table[0].frequency;

	for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++)
		/* Doing nothing currently */

	freq_min = table[i-1].frequency;

	printk(KERN_INFO "%s, max(%d),min(%d)\n", __func__, freq_max, freq_min);

	return cpufreq_register_notifier(&dvfs_hotplug,
					 CPUFREQ_TRANSITION_NOTIFIER);
}

late_initcall(exynos4_integrated_dvfs_hotplug_init);
