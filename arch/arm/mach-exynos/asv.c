/* linux/arch/arm/mach-exynos/asv.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS4 - ASV(Adaptive Supply Voltage) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <asm/mach-types.h>

#include <plat/cputype.h>
#include <plat/clock.h>

#include <mach/map.h>
#include <mach/regs-iem.h>
#include <mach/regs-pmu.h>
#include <mach/regs-clock.h>
#include <mach/cpufreq.h>

#define LOOP_CNT	10

static int __init iem_clock_init(void)
{
	struct clk *clk_hpm;
	struct clk *clk_copy;
	struct clk *clk_parent;

	/* PWI clock setting */
	clk_copy = clk_get(NULL, "sclk_pwi");
	if (IS_ERR(clk_copy)) {
		printk(KERN_ERR"ASV : SCLK_PWI clock get error\n");
		return -EINVAL;
	} else {
		clk_parent = clk_get(NULL, "xusbxti");

		if (IS_ERR(clk_parent)) {
			printk(KERN_ERR"ASV : MOUT_APLL clock get error\n");
			clk_put(clk_copy);
			return -EINVAL;
		}
		if (clk_set_parent(clk_copy, clk_parent))
			printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
					clk_parent->name, clk_copy->name);

		clk_put(clk_parent);
	}
	clk_set_rate(clk_copy, 4800000);

	clk_put(clk_copy);

	/* HPM clock setting */
	clk_copy = clk_get(NULL, "dout_copy");
	if (IS_ERR(clk_copy)) {
		printk(KERN_ERR"ASV : DOUT_COPY clock get error\n");
		return -EINVAL;
	} else {
		clk_parent = clk_get(NULL, "mout_mpll");
		if (IS_ERR(clk_parent)) {
			printk(KERN_ERR"ASV : MOUT_APLL clock get error\n");
			clk_put(clk_copy);
			return -EINVAL;
		}
		if (clk_set_parent(clk_copy, clk_parent))
			printk(KERN_ERR "Unable to set parent %s of clock %s.\n",
					clk_parent->name, clk_copy->name);

		clk_put(clk_parent);
	}

	clk_set_rate(clk_copy, (400 * 1000 * 1000));

	clk_put(clk_copy);

	clk_hpm = clk_get(NULL, "sclk_hpm");
	if (IS_ERR(clk_hpm))
		return -EINVAL;

	clk_set_rate(clk_hpm, (200 * 1000 * 1000));

	clk_put(clk_hpm);

	return 0;
}

void __init iem_clock_set(void)
{
	/* APLL_CON0 level register */
	__raw_writel(0x80FA0601, EXYNOS4_APLL_CON0L8);
	__raw_writel(0x80C80601, EXYNOS4_APLL_CON0L7);
	__raw_writel(0x80C80602, EXYNOS4_APLL_CON0L6);
	__raw_writel(0x80C80604, EXYNOS4_APLL_CON0L5);
	__raw_writel(0x80C80601, EXYNOS4_APLL_CON0L4);
	__raw_writel(0x80C80601, EXYNOS4_APLL_CON0L3);
	__raw_writel(0x80C80601, EXYNOS4_APLL_CON0L2);
	__raw_writel(0x80C80601, EXYNOS4_APLL_CON0L1);

	/* IEM Divider register */
	__raw_writel(0x00500000, EXYNOS4_CLKDIV_IEM_L8);
	__raw_writel(0x00500000, EXYNOS4_CLKDIV_IEM_L7);
	__raw_writel(0x00500000, EXYNOS4_CLKDIV_IEM_L6);
	__raw_writel(0x00500000, EXYNOS4_CLKDIV_IEM_L5);
	__raw_writel(0x00500000, EXYNOS4_CLKDIV_IEM_L4);
	__raw_writel(0x00500000, EXYNOS4_CLKDIV_IEM_L3);
	__raw_writel(0x00500000, EXYNOS4_CLKDIV_IEM_L2);
	__raw_writel(0x00500000, EXYNOS4_CLKDIV_IEM_L1);
}

#define IDS_OFFSET			24
#define IDS_MASK			0xFF
#define PACK_ID				8
#define PACK_MASK			0x3

enum target_asv {
	EXYNOS4210_1200,
	EXYNOS4210_1400,
	EXYNOS4210_SINGLE_1200,
};

struct limit_value {
	unsigned int hpm_limit;
	unsigned int ids_limit;
};

static struct limit_value exynos4210_1200_limit[] = {
	/* HPM , IDS */
	{8 , 4},
	{11 , 8},
	{14 , 12},
	{18 , 17},
	{21 , 27},
	{23 , 45},
	{25 , 55},
};

static struct limit_value exynos4210_1400_limit[] = {
	/* HPM , IDS */
	{13 , 8},
	{17 , 12},
	{22 , 32},
	{26 , 52},
};

static struct limit_value exynos4210_single_1200_limit[] = {
	/* HPM , IDS */
	{8 , 4},
	{14 , 12},
	{21 , 27},
	{25 , 55},
};

static int __init exynos4_find_group(unsigned int hpm_value,
				     unsigned int ids_value,
				     enum target_asv exynos4_target)
{
	unsigned int ret = 0;
	unsigned int i;

	if (exynos4_target == EXYNOS4210_1200) {
		ret = ARRAY_SIZE(exynos4210_1200_limit);

		for (i = 0 ; i < ARRAY_SIZE(exynos4210_1200_limit) ; i++) {
			if (hpm_value <= exynos4210_1200_limit[i].hpm_limit ||
			   ids_value <= exynos4210_1200_limit[i].ids_limit) {
				ret = i;
				break;
			}
		}
	} else if (exynos4_target == EXYNOS4210_1400) {
		ret = ARRAY_SIZE(exynos4210_1400_limit);

		for (i = 0 ; i < ARRAY_SIZE(exynos4210_1400_limit) ; i++) {
			if (hpm_value <= exynos4210_1400_limit[i].hpm_limit ||
			   ids_value <= exynos4210_1200_limit[i].ids_limit) {
				ret = i;
				break;
			}
		}
	} else if (exynos4_target == EXYNOS4210_SINGLE_1200) {
		ret = ARRAY_SIZE(exynos4210_single_1200_limit);

		for (i = 0 ; i < ARRAY_SIZE(exynos4210_single_1200_limit) ; i++) {
			if (hpm_value <= exynos4210_single_1200_limit[i].hpm_limit ||
			   ids_value <= exynos4210_single_1200_limit[i].ids_limit) {
				ret = i;
				break;
			}
		}
	}

	return ret;
}

static void __init exynos4_set_register(unsigned int pkg_id,
				       unsigned int hpm_val,
				       unsigned int ids_val)
{
	unsigned int result_grp;
	char *support_freq;

	/* Single chip is only support 1.2GHz */
	if (!((cpu_idcode >> PACK_ID) & PACK_MASK)) {
		result_grp = exynos4_find_group(hpm_val, ids_val,
						EXYNOS4210_SINGLE_1200);
		result_grp |= SUPPORT_1200MHZ;
		support_freq = "1.2GHz";

		goto set_reg;
	}

	/* Check support freq */
	switch (pkg_id & 0x7) {
	/* Support 1.2GHz */
	case 1:
	case 7:
		result_grp = exynos4_find_group(hpm_val, ids_val,
						EXYNOS4210_1200);
		result_grp |= SUPPORT_1200MHZ;
		support_freq = "1.2GHz";
		break;
	/* Support 1.4GHz */
	case 5:
		result_grp = exynos4_find_group(hpm_val, ids_val,
						EXYNOS4210_1400);
		result_grp |= SUPPORT_1200MHZ;
		support_freq = "1.4GHz";
		break;
	/* Defalut support 1.0GHz */
	default:
		result_grp = exynos4_find_group(hpm_val, ids_val,
						EXYNOS4210_1200);
		result_grp |= SUPPORT_1000MHZ;
		support_freq = "1.0GHz";
		break;
	}

set_reg:
	__raw_writel(result_grp, S5P_INFORM2);

	printk(KERN_INFO "Support %s\n", support_freq);
	printk(KERN_INFO "ASV Group for This Exynos4210 is 0x%x\n", result_grp);
}

static int __init exynos4_asv_init(void)
{
	unsigned int i;
	unsigned long hpm_delay = 0;
	unsigned int tmp;
	unsigned int pkg_id;
	unsigned int ids_arm;
	unsigned int result_group = 0;
	void __iomem *iem_base;

	pkg_id = __raw_readl(S5P_VA_CHIPID + 0x4);

	ids_arm = ((pkg_id >> IDS_OFFSET) & IDS_MASK);

	iem_base = ioremap(EXYNOS4_PA_IEM, (128 * 1024));

	if (iem_base == NULL) {
		printk(KERN_ERR "faile to ioremap\n");
		goto out;
	}

	if (iem_clock_init()) {
		printk(KERN_ERR "ASV driver clock_init fail\n");
		goto out;
	} else {
		/* HPM enable  */
		tmp = __raw_readl(iem_base + EXYNOS4_APC_CONTROL);
		tmp |= APC_HPM_EN;
		__raw_writel(tmp, (iem_base + EXYNOS4_APC_CONTROL));

		iem_clock_set();

		/* IEM enable */
		tmp = __raw_readl(iem_base + EXYNOS4_IECDPCCR);
		tmp |= IEC_EN;
		__raw_writel(tmp, (iem_base + EXYNOS4_IECDPCCR));
	}

	for (i = 0 ; i < LOOP_CNT ; i++) {
		tmp = __raw_readb(iem_base + EXYNOS4_APC_DBG_DLYCODE);
		hpm_delay += tmp;
	}

	hpm_delay /= LOOP_CNT;

	exynos4_set_register(pkg_id, hpm_delay, ids_arm);

	return 0;
out:
	__raw_writel(0, S5P_INFORM2);

	printk(KERN_INFO "ASV Group for This Exynos4210 is %d\n", result_group);

	return -EINVAL;
}
core_initcall(exynos4_asv_init);
