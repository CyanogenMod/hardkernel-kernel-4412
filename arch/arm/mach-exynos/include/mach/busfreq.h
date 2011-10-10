/* linux/arch/arm/mach-exynos/include/mach/busfreq.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4 - BUSFreq support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_BUSFREQ_H
#define __ASM_ARCH_BUSFREQ_H __FILE__

#define MAX_LOAD		100
#define DIVIDING_FACTOR		10000
#define PPC_THRESHOLD		23
#define PPMU_THRESHOLD		5

struct opp;
struct device;

struct busfreq_data {
	bool use;
	unsigned long min_cpufreq;
	struct device *dev;
	struct delayed_work worker;
	struct opp *curr_opp;
	struct opp *max_opp;
	struct regulator *vdd_int;
	struct regulator *vdd_mif;
	unsigned int sampling_rate;

	struct notifier_block exynos4_buspm_notifier;
	struct notifier_block exynos4_reboot_notifier;
	int (*init)	(struct device *dev, struct busfreq_data *data);
	void (*target)	(struct opp *opp);
	unsigned int (*get_int_volt) (unsigned long freq);
};

struct busfreq_table {
	unsigned int idx;
	unsigned int mem_clk;
	unsigned int volt;
	unsigned int clk_topdiv;
	unsigned int clk_dmc0div;
	unsigned int clk_dmc1div;
};

int exynos4210_init(struct device *dev, struct busfreq_data *data);
void exynos4210_target(struct opp *opp);
int exynos4212_init(struct device *dev, struct busfreq_data *data);
void exynos4212_target(struct opp *opp);
unsigned int exynos4212_get_int_volt(unsigned long freq);
#endif /* __ASM_ARCH_BUSFREQ_H */
