/* linux/arch/arm/mach-exynos/clock-exynos5.c
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS5 - Clock support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/syscore_ops.h>

#include <plat/cpu-freq.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/pll.h>
#include <plat/s5p-clock.h>
#include <plat/clock-clksrc.h>
#include <plat/devs.h>
#include <plat/pm.h>
#include <plat/cputype.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-audss.h>
#include <mach/sysmmu.h>
#include <mach/exynos-clock.h>

void __init_or_cpufreq exynos5_setup_clocks(void)
{
}

void __init exynos5_register_clocks(void)
{
}
