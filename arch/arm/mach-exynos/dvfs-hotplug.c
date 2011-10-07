/* linux/arch/arm/mach-exynos/dvfs-hotplug.c
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

#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/suspend.h>
#include <plat/cputype.h>

static unsigned int total_num_target_freq;
static unsigned int consecutv_highestlevel_cnt;
static unsigned int consecutv_lowestlevel_cnt;
static unsigned int num_hotplug_in;
static unsigned int num_hotplug_out;

static unsigned int freq_max;
static unsigned int freq_in_trg;
static unsigned int freq_min = -1UL;

static void exynos4_integrated_dvfs_hotplug(unsigned int freq_old,
					unsigned int freq_new)
{
	total_num_target_freq++;
	freq_in_trg = 800000;

	if ((freq_old >= freq_in_trg) && (freq_new >= freq_in_trg)) {
		if (cpu_is_exynos4412()) {
			if (cpu_online(3) == 0) {
				if (consecutv_highestlevel_cnt >= 2) {
					cpu_up(3);
					num_hotplug_in++;
					consecutv_highestlevel_cnt = 0;
				}
			} else if (cpu_online(2) == 0) {
				if (consecutv_highestlevel_cnt >= 2) {
					cpu_up(2);
					num_hotplug_in++;
					consecutv_highestlevel_cnt = 0;
				}
			} else if (cpu_online(1) == 0) {
				if (consecutv_highestlevel_cnt >= 2) {
					cpu_up(1);
					num_hotplug_in++;
					consecutv_highestlevel_cnt = 0;
				}
			}
			consecutv_highestlevel_cnt++;
		} else {
			if (cpu_online(1) == 0) {
				if (consecutv_highestlevel_cnt >= 2) {
					cpu_up(1);
					num_hotplug_in++;
					consecutv_highestlevel_cnt = 0;
				}
			}
			consecutv_highestlevel_cnt++;
		}
	} else if ((freq_old <= freq_min) && (freq_new <= freq_min)) {
		if (cpu_is_exynos4412()) {
			if (cpu_online(1) == 1) {
				if (consecutv_lowestlevel_cnt >= 2) {
					cpu_down(1);
					num_hotplug_out++;
					consecutv_lowestlevel_cnt = 1;
				} else
					consecutv_lowestlevel_cnt++;
			} else if (cpu_online(2) == 1) {
				if (consecutv_lowestlevel_cnt >= 2) {
					cpu_down(2);
					num_hotplug_out++;
					consecutv_lowestlevel_cnt = 0;
				} else
					consecutv_lowestlevel_cnt++;
			} else if (cpu_online(3) == 1) {
				if (consecutv_lowestlevel_cnt >= 2) {
					cpu_down(3);
					num_hotplug_out++;
					consecutv_lowestlevel_cnt = 0;
				} else
					consecutv_lowestlevel_cnt++;
			}
		} else {
			if (cpu_online(1) == 1) {
				if (consecutv_lowestlevel_cnt >= 2) {
					cpu_down(1);
					num_hotplug_out++;
					consecutv_lowestlevel_cnt = 1;
				} else
					consecutv_lowestlevel_cnt++;
			}
		}
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
	unsigned int freq;

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

	for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++) {
		freq = table[i].frequency;

		if (freq != CPUFREQ_ENTRY_INVALID && freq > freq_max)
			freq_max = freq;
		else if (freq != CPUFREQ_ENTRY_INVALID && freq_min > freq)
			freq_min = freq;
	}

	printk(KERN_INFO "%s, max(%d),min(%d)\n", __func__, freq_max, freq_min);

	return cpufreq_register_notifier(&dvfs_hotplug,
					 CPUFREQ_TRANSITION_NOTIFIER);
}

late_initcall(exynos4_integrated_dvfs_hotplug_init);
