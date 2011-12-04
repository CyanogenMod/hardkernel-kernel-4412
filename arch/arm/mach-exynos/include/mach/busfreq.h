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

#include <linux/notifier.h>

#define MAX_LOAD		100
#define DIVIDING_FACTOR		10000
#define PPC_THRESHOLD		23
#define PPMU_THRESHOLD		5
#define IDLE_THRESHOLD		4
#define AVE_CUTLINE		5
#define LV0_CUTLINE		9
#define LV1_CUTLINE		4
#define PRE_LOAD_SIZE		3

struct opp;
struct device;
struct busfreq_table;

struct busfreq_data {
	bool use;
	struct device *dev;
	struct delayed_work worker;
	struct opp *curr_opp;
	struct opp *max_opp;
	struct regulator *vdd_int;
	struct regulator *vdd_mif;
	unsigned int sampling_rate;
	struct kobject *busfreq_kobject;
	int table_size;
	struct busfreq_table *table;
	unsigned long long *time_in_state;
	unsigned long long last_time;
	unsigned int load_average[PRE_LOAD_SIZE];
	int index;

	struct notifier_block exynos4_buspm_notifier;
	struct notifier_block exynos4_reboot_notifier;
	struct notifier_block exynos4_request_notifier;
	struct attribute_group busfreq_attr_group;
	int (*init)	(struct device *dev, struct busfreq_data *data);
	unsigned int (*target)	(unsigned int index);
	unsigned int (*get_int_volt) (unsigned long freq);
	unsigned int (*get_table_index) (struct opp *opp);
};

struct busfreq_table {
	unsigned int idx;
	unsigned int mem_clk;
	unsigned int volt;
	unsigned int clk_topdiv;
	unsigned int clk_dmc0div;
	unsigned int clk_dmc1div;
};

void exynos4_request_apply(unsigned long freq, struct device *dev);

int exynos4210_init(struct device *dev, struct busfreq_data *data);
unsigned int exynos4210_target(unsigned int index);
unsigned int exynos4210_get_table_index(struct opp *opp);
int exynos4x12_init(struct device *dev, struct busfreq_data *data);
unsigned int exynos4x12_target(unsigned int index);
unsigned int exynos4x12_get_int_volt(unsigned long freq);
unsigned int exynos4x12_get_table_index(struct opp *opp);
#endif /* __ASM_ARCH_BUSFREQ_H */
