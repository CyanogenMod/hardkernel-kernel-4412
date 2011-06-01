/* linux/arch/arm/mach-exynos4/ppmu.c
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

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/ppmu.h>

void exynos4_ppmu_reset(struct exynos4_ppmu_hw *ppmu)
{
	void __iomem *ppmu_base = ppmu->hw_base;

	__raw_writel(0x3 << 1, ppmu_base);
	__raw_writel(0x8000000f, ppmu_base + 0x0010);
	__raw_writel(0x8000000f, ppmu_base + 0x0030);

	__raw_writel(0x0, ppmu_base + DEVT0_ID);
	__raw_writel(0x0, ppmu_base + DEVT0_IDMSK);

	__raw_writel(0x0, ppmu_base + DEVT1_ID);
	__raw_writel(0x0, ppmu_base + DEVT1_IDMSK);

	ppmu->ccnt = 0;
	ppmu->event = 0;
	ppmu->count[0] = 0;
	ppmu->count[1] = 0;
	ppmu->count[2] = 0;
	ppmu->count[3] = 0;
}

void exynos4_ppmu_setevent(struct exynos4_ppmu_hw *ppmu,
		unsigned int evt, unsigned int evt_num)
{
	void __iomem *ppmu_base = ppmu->hw_base;

	ppmu->event = evt;

	__raw_writel(evt , ppmu_base + DEVT0_SEL + (evt_num * 0x100));
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

void exynos4_ppmu_update(struct exynos4_ppmu_hw *ppmu)
{
	void __iomem *ppmu_base = ppmu->hw_base;
	unsigned int i;

	ppmu->ccnt = __raw_readl(ppmu_base + 0x0100);


	for (i = 0; i < NUMBER_OF_COUNTER; i++) {
		ppmu->count[i] =
			__raw_readl(ppmu_base + (0x110 + (0x10 * i)));
	}

}
