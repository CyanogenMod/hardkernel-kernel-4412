/* linux/arch/arm/mach-exynos/ppmu.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS4 - CPU PPMU support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/math64.h>

#include <plat/cputype.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/ppmu.h>

static LIST_HEAD(ppmu_list);

unsigned long long ppmu_load[PPMU_END - 1];

void exynos4_ppmu_reset(struct exynos4_ppmu_hw *ppmu)
{
	void __iomem *ppmu_base = ppmu->hw_base;
	int i;

	__raw_writel(0x3 << 1, ppmu_base);
	__raw_writel(0x8000000f, ppmu_base + PPMU_CNTENS);

	if (soc_is_exynos4210())
		for (i = 0; i < NUMBER_OF_COUNTER; i++) {
			__raw_writel(0x0, ppmu_base + DEVT0_ID + (i * DEVT_ID_OFFSET));
			__raw_writel(0x0, ppmu_base + DEVT0_IDMSK + (i * DEVT_ID_OFFSET));
		}
}

void exynos4_ppmu_setevent(struct exynos4_ppmu_hw *ppmu,
				unsigned int evt_num)
{
	void __iomem *ppmu_base = ppmu->hw_base;
	__raw_writel(ppmu->event[evt_num], ppmu_base + PPMU_BEVT0SEL + (evt_num * PPMU_BEVTSEL_OFFSET));
}

void exynos4_ppmu_start(struct exynos4_ppmu_hw *ppmu)
{
	void __iomem *ppmu_base = ppmu->hw_base;
	__raw_writel(0x1, ppmu_base);
}

void exynos4_ppmu_stop(struct exynos4_ppmu_hw *ppmu)
{
	void __iomem *ppmu_base = ppmu->hw_base;
	__raw_writel(0x0, ppmu_base);
}

unsigned long long exynos4_ppmu_update(struct exynos4_ppmu_hw *ppmu)
{
	void __iomem *ppmu_base = ppmu->hw_base;
	unsigned int i;
	unsigned long long total = 0;

	ppmu->ccnt = __raw_readl(ppmu_base + PPMU_CCNT);

	if (ppmu->ccnt == 0)
		ppmu->ccnt = MAX_CCNT;

	for (i = 0; i < NUMBER_OF_COUNTER; i++) {
		if (ppmu->event[i] == 0)
			continue;

		if (i == 3)
			total += (((u64)__raw_readl(ppmu_base + PMCNT_OFFSET(i)) << 32) |
				__raw_readl(ppmu_base + PMCNT_OFFSET(i + 1)));
		else
			total += __raw_readl(ppmu_base + PMCNT_OFFSET(i));
	}

	return div_u64((total * ppmu->weight * 100), ppmu->ccnt);
}

void ppmu_start(struct device *dev)
{
	struct exynos4_ppmu_hw *ppmu;

	list_for_each_entry(ppmu, &ppmu_list, node)
		if (ppmu->dev == dev)
			exynos4_ppmu_start(ppmu);
}

unsigned long long ppmu_update(struct device *dev)
{
	struct exynos4_ppmu_hw *ppmu;
	unsigned long long average = 0;

	list_for_each_entry(ppmu, &ppmu_list, node)
		if (ppmu->dev == dev) {
			exynos4_ppmu_stop(ppmu);
			average = exynos4_ppmu_update(ppmu);
			exynos4_ppmu_reset(ppmu);
			break;
		}

	return average;
}

void ppmu_all_start(struct device *dev)
{
	struct exynos4_ppmu_hw *ppmu;

	list_for_each_entry(ppmu, &ppmu_list, node)
		if (ppmu->dev == dev)
			exynos4_ppmu_start(ppmu);
}

void ppmu_all_update(unsigned int flag)
{
	struct exynos4_ppmu_hw *ppmu;

	list_for_each_entry(ppmu, &ppmu_list, node)
		if (ppmu->flags & flag) {
			exynos4_ppmu_stop(ppmu);
			ppmu_load[ppmu->id] = exynos4_ppmu_update(ppmu);
			exynos4_ppmu_reset(ppmu);
		}
}

void ppmu_reset(void)
{
	struct exynos4_ppmu_hw *ppmu;
	int i;

	list_for_each_entry(ppmu, &ppmu_list, node) {
		exynos4_ppmu_stop(ppmu);
		for (i = 0; i < NUMBER_OF_COUNTER; i++)
			if (ppmu->event[i] != 0)
				exynos4_ppmu_setevent(ppmu, i);
		exynos4_ppmu_reset(ppmu);
	}
}

void ppmu_init(struct exynos4_ppmu_hw *ppmu, struct device *dev)
{
	void __iomem *ppmu_base = ppmu->hw_base;
	int i;

	ppmu->dev = dev;
	list_add(&ppmu->node, &ppmu_list);

	if (soc_is_exynos4210())
		for (i = 0; i < NUMBER_OF_COUNTER; i++) {
			__raw_writel(0x0, ppmu_base + DEVT0_ID + (i * DEVT_ID_OFFSET));
			__raw_writel(0x0, ppmu_base + DEVT0_IDMSK + (i * DEVT_ID_OFFSET));
		}

	for (i = 0; i < NUMBER_OF_COUNTER; i++)
		if (ppmu->event[i] != 0)
			exynos4_ppmu_setevent(ppmu, i);
}

struct exynos4_ppmu_hw exynos_ppmu[] = {
	[PPMU_DMC0] = {
		.id = PPMU_DMC0,
		.hw_base = S5P_VA_PPMU_DMC0,
		.event[0] = RD_DATA_COUNT,
		.event[1] = WR_DATA_COUNT,
		.weight = DEFAULT_WEIGHT,
		.flags = ALL_DOMAIN,
	},
	[PPMU_DMC1] = {
		.id = PPMU_DMC1,
		.hw_base = S5P_VA_PPMU_DMC1,
		.event[0] = RD_DATA_COUNT,
		.event[1] = WR_DATA_COUNT,
		.weight = DEFAULT_WEIGHT,
		.flags = ALL_DOMAIN,
	},
	[PPMU_CPU] = {
		.id = PPMU_CPU,
		.hw_base = S5P_VA_PPMU_CPU,
		.event[0] = RD_DATA_COUNT,
		.event[1] = WR_DATA_COUNT,
		.weight = DEFAULT_WEIGHT,
		.flags = ALL_DOMAIN,
	},
	[PPMU_RIGHT] = {
		.id = PPMU_RIGHT,
		.hw_base = S5P_VA_PPMU_RIGHT,
		.event[0] = RD_DATA_COUNT,
		.event[1] = WR_DATA_COUNT,
		.weight = DEFAULT_WEIGHT,
		.flags = ALL_DOMAIN,
	},
	[PPMU_LEFT] = {
		.id = PPMU_LEFT,
		.hw_base = S5P_VA_PPMU_LEFT,
		.event[0] = RD_DATA_COUNT,
		.event[1] = WR_DATA_COUNT,
		.weight = DEFAULT_WEIGHT,
		.flags = ALL_DOMAIN,
	},
	[PPMU_MFC_L] = {
		.id = PPMU_MFC_L,
		.hw_base = S5P_VA_PPMU_MFC_L,
		.event[0] = RD_DATA_COUNT,
		.event[1] = WR_DATA_COUNT,
		.weight = DEFAULT_WEIGHT,
		.flags = ALL_DOMAIN,
	},
	[PPMU_MFC_R] = {
		.id = PPMU_MFC_R,
		.hw_base = S5P_VA_PPMU_MFC_R,
		.event[0] = RD_DATA_COUNT,
		.event[1] = WR_DATA_COUNT,
		.weight = DEFAULT_WEIGHT,
		.flags = ALL_DOMAIN,
	},
};
