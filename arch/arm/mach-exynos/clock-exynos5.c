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

/* Core list of CMU_CPU side */

static struct clksrc_clk exynos5_clk_mout_apll = {
	.clk	= {
		.name		= "mout_apll",
	},
	.sources = &clk_src_apll,
	.reg_src = { .reg = EXYNOS5_CLKSRC_CPU, .shift = 0, .size = 1 },
};

struct clksrc_clk exynos5_clk_sclk_apll = {
	.clk	= {
		.name		= "sclk_apll",
		.parent		= &exynos5_clk_mout_apll.clk,
	},
	.reg_div = { .reg = EXYNOS5_CLKDIV_CPU, .shift = 24, .size = 3 },
};

void __init_or_cpufreq exynos5_setup_clocks(void)
{
}

void __init exynos5_register_clocks(void)
{
}
