/* linux/arch/arm/mach-exynos/pmu-exynos5.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS5 - CPU PMU(Power Management Unit) support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/kernel.h>

#include <mach/regs-clock.h>
#include <mach/pmu.h>

#include <plat/cputype.h>

static struct exynos4_pmu_conf *exynos5_pmu_config;

static unsigned int entry_cnt;

static struct exynos4_pmu_conf exynos52xx_pmu_config[] = {
	/* { .reg = address, .val = { AFTR, LPA, SLEEP } */
};

void exynos5_sys_powerdown_conf(enum sys_powerdown mode)
{
	unsigned int count = entry_cnt;

	for (; count > 0; count--)
		__raw_writel(exynos5_pmu_config[count - 1].val[mode],
				exynos5_pmu_config[count - 1].reg);
}

static int __init exynos5_pmu_init(void)
{
	exynos5_pmu_config = exynos52xx_pmu_config;
	entry_cnt = ARRAY_SIZE(exynos52xx_pmu_config);
	printk(KERN_INFO "%s: PMU supports 52XX(%d)\n"
				, __func__, entry_cnt);

	return 0;
}
arch_initcall(exynos5_pmu_init);
