/* linux/arch/arm/mach-exynos/pm-exynos5.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS5 - Power Management support
 *
 * Based on arch/arm/mach-s3c2410/pm.c
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/io.h>

#include <asm/cacheflush.h>

#include <plat/cpu.h>
#include <plat/pm.h>

void exynos5_cpu_suspend(void)
{
	/* Nothing here not yet */
}

static void exynos5_pm_prepare(void)
{
	/* Nothing here not yet */
}

static int exynos5_pm_add(struct sys_device *sysdev)
{
	pm_cpu_prep = exynos5_pm_prepare;
	pm_cpu_sleep = exynos5_cpu_suspend;

	return 0;
}

static struct sysdev_driver exynos5_pm_driver = {
	.add		= exynos5_pm_add,
};

static __init int exynos5_pm_drvinit(void)
{
	s3c_pm_init();

	return sysdev_driver_register(&exynos5_sysclass, &exynos5_pm_driver);
}
arch_initcall(exynos5_pm_drvinit);

static int exynos5_pm_suspend(void)
{
	/* Nothing here not yet */

	return 0;
}

static void exynos5_pm_resume(void)
{
	/* Nothing here not yet */
}

static struct syscore_ops exynos5_pm_syscore_ops = {
	.suspend	= exynos5_pm_suspend,
	.resume		= exynos5_pm_resume,
};

static __init int exynos5_pm_syscore_init(void)
{
	register_syscore_ops(&exynos5_pm_syscore_ops);

	return 0;
}
arch_initcall(exynos5_pm_syscore_init);
