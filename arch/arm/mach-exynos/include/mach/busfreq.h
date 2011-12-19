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

#include <mach/ppmu.h>

#define MAX_LOAD		100
#define UP_THRESHOLD		30
#define DMC0_MAX_THRESHOLD	32
#define DMC1_MAX_THRESHOLD	30
#define DIVIDING_FACTOR		10000
#define PPC_THRESHOLD		23
#define PPMU_THRESHOLD		5
#define IDLE_THRESHOLD		4
#define AVE_CUTLINE		5
#define LV0_CUTLINE		9
#define LV1_CUTLINE		4
#define LOAD_HISTORY_SIZE	5

struct opp;
struct device;
struct busfreq_table;

struct busfreq_data {
	bool use;
	struct device *dev;
	struct delayed_work worker;
	struct opp *curr_opp;
	struct opp *max_opp;
	struct opp *min_opp;
	struct regulator *vdd_int;
	struct regulator *vdd_mif;
	unsigned int sampling_rate;
	struct kobject *busfreq_kobject;
	int table_size;
	struct busfreq_table *table;
	unsigned long long *time_in_state;
	unsigned long long last_time;
	unsigned int load_history[PPMU_END][LOAD_HISTORY_SIZE];
	int index;

	struct notifier_block exynos_buspm_notifier;
	struct notifier_block exynos_reboot_notifier;
	struct notifier_block exynos_request_notifier;
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

void exynos_request_apply(unsigned long freq, struct device *dev);

#if defined(CONFIG_ARCH_EXYNOS5)
int exynos5250_init(struct device *dev, struct busfreq_data *data);
unsigned int exynos5250_target(unsigned int index);
unsigned int exynos5250_get_int_volt(unsigned long freq);
unsigned int exynos5250_get_table_index(struct opp *opp);
static inline int exynos4x12_init(struct device *dev, struct busfreq_data *data)
{
	return 0;
}

static inline unsigned int exynos4x12_target(unsigned int index)
{
	return 0;
}

static inline unsigned int exynos4x12_get_int_volt(unsigned long freq)
{
	return 0;
}

static inline unsigned int exynos4x12_get_table_index(struct opp *opp)
{
	return 0;
}
#elif defined(CONFIG_ARCH_EXYNOS4)
static inline int exynos5250_init(struct device *dev, struct busfreq_data *data)
{
	return 0;
}

static inline unsigned int exynos5250_target(unsigned int index)
{
	return 0;
}

static inline unsigned int exynos5250_get_int_volt(unsigned long freq)
{
	return 0;
}

static inline unsigned int exynos5250_get_table_index(struct opp *opp)
{
	return 0;
}

int exynos4x12_init(struct device *dev, struct busfreq_data *data);
unsigned int exynos4x12_target(unsigned int index);
unsigned int exynos4x12_get_int_volt(unsigned long freq);
unsigned int exynos4x12_get_table_index(struct opp *opp);
#endif
#endif /* __ASM_ARCH_BUSFREQ_H */
