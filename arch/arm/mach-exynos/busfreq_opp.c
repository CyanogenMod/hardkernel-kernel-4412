/* linux/arch/arm/mach-exynos/busfreq.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS4 - BUS clock frequency scaling support with OPP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/ktime.h>
#include <linux/tick.h>
#include <linux/kernel_stat.h>
#include <linux/cpufreq.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/opp.h>

#include <asm/mach-types.h>

#include <mach/ppmu.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/gpio.h>
#include <mach/regs-mem.h>
#include <mach/cpufreq.h>
#include <mach/dev.h>

#include <plat/map-s5p.h>
#include <plat/gpio-cfg.h>
#include <plat/cputype.h>

#define MAX_LOAD		100
#define DIVIDING_FACTOR		10000
#define PPC_THRESHOLD		23
#define PPMU_THRESHOLD		5

struct busfreq_data {
	bool use;
	unsigned long min_cpufreq;
	struct device *dev;
	struct opp *curr_opp;
	struct opp *max_opp;
	struct regulator *vdd_int;
	struct regulator *vdd_mif;

	struct notifier_block exynos4_busfreq_notifier;
	struct notifier_block exynos4_buspm_notifier;
	struct notifier_block exynos4_reboot_notifier;
};

enum busfreq_level_idx {
	LV_0,
	LV_1,
	LV_2,
	LV_END
};

struct busfreq_table {
	unsigned int idx;
	unsigned int mem_clk;
	unsigned int volt;
	unsigned int clk_topdiv;
	unsigned int clk_dmcdiv;
};

static struct busfreq_table exynos4_busfreq_table[] = {
	{LV_0, 400000, 1100000, 0, 0},
	{LV_1, 267000, 1000000, 0, 0},
	{LV_2, 133000, 950000, 0, 0},
	{0, 0, 0, 0, 0},
};

#define ASV_GROUP	5
static unsigned int exynos4_asv_volt[ASV_GROUP][LV_END] = {
	{1150000, 1050000, 1000000},
	{1125000, 1025000, 975000},
	{1100000, 1000000, 950000},
	{1075000, 975000, 950000},
	{1050000, 950000, 950000},
};

static unsigned int clkdiv_dmc0[LV_END][8] = {
	/*
	 * Clock divider value for following
	 * { DIVACP, DIVACP_PCLK, DIVDPHY, DIVDMC, DIVDMCD
	 *              DIVDMCP, DIVCOPY2, DIVCORE_TIMERS }
	 */

	/* DMC L0: 400MHz */
	{ 3, 1, 1, 1, 1, 1, 3, 1 },

	/* DMC L1: 266.7MHz */
	{ 4, 1, 1, 2, 1, 1, 3, 1 },

	/* DMC L2: 133MHz */
	{ 5, 1, 1, 5, 1, 1, 3, 1 },
};

static unsigned int clkdiv_top[LV_END][5] = {
	/*
	 * Clock divider value for following
	 * { DIVACLK200, DIVACLK100, DIVACLK160, DIVACLK133, DIVONENAND }
	 */

	/* ACLK200 L0: 200MHz */
	{ 3, 7, 4, 5, 1 },

	/* ACLK200 L1: 160MHz */
	{ 4, 7, 5, 6, 1 },

	/* ACLK200 L2: 133MHz */
	{ 5, 7, 7, 7, 1 },
};

static unsigned int clkdiv_lr_bus[LV_END][2] = {
	/*
	 * Clock divider value for following
	 * { DIVGDL/R, DIVGPL/R }
	 */

	/* ACLK_GDL/R L1: 200MHz */
	{ 3, 1 },

	/* ACLK_GDL/R L2: 160MHz */
	{ 4, 1 },

	/* ACLK_GDL/R L3: 133MHz */
	{ 5, 1 },
};

static void exynos4_set_busfreq(struct opp *opp)
{
	unsigned int div_index;
	unsigned int tmp;

	for (div_index = LV_0; div_index < LV_END; div_index++)
		if (opp_get_freq(opp) == exynos4_busfreq_table[div_index].mem_clk)
			break;

	if (div_index == LV_END)
		return;

	/* Change Divider - DMC0 */
	tmp = exynos4_busfreq_table[div_index].clk_dmcdiv;

	__raw_writel(tmp, S5P_CLKDIV_DMC0);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STAT_DMC0);
	} while (tmp & 0x11111111);

	/* Change Divider - TOP */
	tmp = exynos4_busfreq_table[div_index].clk_topdiv;

	__raw_writel(tmp, S5P_CLKDIV_TOP);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STAT_TOP);
	} while (tmp & 0x11111);

	/* Change Divider - LEFTBUS */
	tmp = __raw_readl(S5P_CLKDIV_LEFTBUS);

	tmp &= ~(S5P_CLKDIV_BUS_GDLR_MASK | S5P_CLKDIV_BUS_GPLR_MASK);

	tmp |= ((clkdiv_lr_bus[div_index][0] << S5P_CLKDIV_BUS_GDLR_SHIFT) |
		(clkdiv_lr_bus[div_index][1] << S5P_CLKDIV_BUS_GPLR_SHIFT));

	__raw_writel(tmp, S5P_CLKDIV_LEFTBUS);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STAT_LEFTBUS);
	} while (tmp & 0x11);

	/* Change Divider - RIGHTBUS */
	tmp = __raw_readl(S5P_CLKDIV_RIGHTBUS);

	tmp &= ~(S5P_CLKDIV_BUS_GDLR_MASK | S5P_CLKDIV_BUS_GPLR_MASK);

	tmp |= ((clkdiv_lr_bus[div_index][0] << S5P_CLKDIV_BUS_GDLR_SHIFT) |
		(clkdiv_lr_bus[div_index][1] << S5P_CLKDIV_BUS_GPLR_SHIFT));

	__raw_writel(tmp, S5P_CLKDIV_RIGHTBUS);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STAT_RIGHTBUS);
	} while (tmp & 0x11);

}

static struct opp *busfreq_monitor(struct busfreq_data *data)
{
	struct opp *opp;
	unsigned long lockfreq;
	unsigned long newfreq = 0;

	lockfreq = dev_max_freq(data->dev);

	if (lockfreq > newfreq)
		newfreq = lockfreq;

	opp = opp_find_freq_ceil(data->dev, &newfreq);

	newfreq = opp_get_freq(opp);

	return opp;
}

static int exynos4_busfreq_notifier_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct busfreq_data *data = container_of(this, struct busfreq_data,
			exynos4_busfreq_notifier);

	struct cpufreq_freqs *freq = ptr;
	struct opp *opp;
	unsigned int voltage;
	unsigned long currfreq;
	unsigned long newfreq;

	switch (event) {
	case CPUFREQ_PRECHANGE:
		break;
	case CPUFREQ_POSTCHANGE:
		if (freq->old != data->min_cpufreq)
			opp = data->max_opp;
		else
			opp = busfreq_monitor(data);

		newfreq = opp_get_freq(opp);

		currfreq = opp_get_freq(data->curr_opp);

		if (opp == data->curr_opp)
			return NOTIFY_OK;

		voltage = opp_get_voltage(opp);
		if (newfreq > currfreq)
			regulator_set_voltage(data->vdd_int, voltage,
					voltage);

		if (newfreq != currfreq)
			exynos4_set_busfreq(opp);

		if (newfreq < currfreq)
			regulator_set_voltage(data->vdd_int, voltage,
					voltage);
		data->curr_opp = opp;
		break;
	case CPUFREQ_RESUMECHANGE:
		break;
	default:
		/* ignore */
		break;
	}

	return NOTIFY_OK;
}

static int exynos4_buspm_notifier_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct busfreq_data *data = container_of(this, struct busfreq_data,
			exynos4_buspm_notifier);

	unsigned long voltage = opp_get_voltage(data->max_opp);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		data->use = false;
		regulator_set_voltage(data->vdd_int, voltage, voltage);
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		data->use = true;
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static int exynos4_busfreq_reboot_event(struct notifier_block *this,
		unsigned long code, void *unused)
{
	struct busfreq_data *data = container_of(this, struct busfreq_data,
			exynos4_reboot_notifier);

	unsigned long voltage = opp_get_voltage(data->max_opp);

	regulator_set_voltage(data->vdd_int, voltage, voltage);
	data->use = false;

	printk(KERN_INFO "REBOOT Notifier for BUSFREQ\n");
	return NOTIFY_DONE;
}

int exynos4_busfreq_lock(unsigned int nId,
	enum busfreq_level_request busfreq_level)
{
	return 0;
}

void exynos4_busfreq_lock_free(unsigned int nId)
{
}

static void __init exynos4_set_bus_volt(void)
{
	unsigned int asv_group;
	unsigned int i;

	asv_group = __raw_readl(S5P_INFORM2) & 0xF;

	printk(KERN_INFO "DVFS : VDD_INT Voltage table set with %d Group\n", asv_group);

	for (i = 0 ; i < LV_END ; i++) {
		switch (asv_group) {
		case 0:
			exynos4_busfreq_table[i].volt =
				exynos4_asv_volt[0][i];
			break;
		case 1:
		case 2:
			exynos4_busfreq_table[i].volt =
				exynos4_asv_volt[1][i];
			break;
		case 3:
		case 4:
			exynos4_busfreq_table[i].volt =
				exynos4_asv_volt[2][i];
			break;
		case 5:
		case 6:
			exynos4_busfreq_table[i].volt =
				exynos4_asv_volt[3][i];
			break;
		case 7:
			exynos4_busfreq_table[i].volt =
				exynos4_asv_volt[4][i];
			break;
		}
	}

	return;
}

static __devinit int exynos4_busfreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct busfreq_data *data;
	struct cpufreq_frequency_table *table;
	unsigned long freq;
	unsigned long min_cpufreq = UINT_MAX;
	unsigned long minfreq = 0;
	unsigned long maxfreq = UINT_MAX;
	unsigned int i;
	unsigned int tmp;
	unsigned int val;
	int ret;

	if (cpu_is_exynos4212())
		return -ENODEV;

	val = __raw_readl(S5P_VA_DMC0 + 0x4);
	val = (val >> 8) & 0xf;

	/* Check Memory Type Only support -> 0x5: 0xLPDDR2 */
	if (val != 0x05) {
		pr_err("[ %x ] Memory Type Undertermined.\n", val);
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct busfreq_data), GFP_KERNEL);
	if (!data) {
		pr_err("Unable to create busfreq_data struct.\n");
		return -ENOMEM;
	}

	data->exynos4_busfreq_notifier.notifier_call =
		exynos4_busfreq_notifier_event;
	data->exynos4_buspm_notifier.notifier_call =
		exynos4_buspm_notifier_event;
	data->exynos4_reboot_notifier.notifier_call =
		exynos4_busfreq_reboot_event,

	tmp = __raw_readl(S5P_CLKDIV_DMC0);

	for (i = 0; i <  LV_END; i++) {
		tmp &= ~(S5P_CLKDIV_DMC0_ACP_MASK |
			S5P_CLKDIV_DMC0_ACPPCLK_MASK |
			S5P_CLKDIV_DMC0_DPHY_MASK |
			S5P_CLKDIV_DMC0_DMC_MASK |
			S5P_CLKDIV_DMC0_DMCD_MASK |
			S5P_CLKDIV_DMC0_DMCP_MASK |
			S5P_CLKDIV_DMC0_COPY2_MASK |
			S5P_CLKDIV_DMC0_CORETI_MASK);

		tmp |= ((clkdiv_dmc0[i][0] << S5P_CLKDIV_DMC0_ACP_SHIFT) |
			(clkdiv_dmc0[i][1] << S5P_CLKDIV_DMC0_ACPPCLK_SHIFT) |
			(clkdiv_dmc0[i][2] << S5P_CLKDIV_DMC0_DPHY_SHIFT) |
			(clkdiv_dmc0[i][3] << S5P_CLKDIV_DMC0_DMC_SHIFT) |
			(clkdiv_dmc0[i][4] << S5P_CLKDIV_DMC0_DMCD_SHIFT) |
			(clkdiv_dmc0[i][5] << S5P_CLKDIV_DMC0_DMCP_SHIFT) |
			(clkdiv_dmc0[i][6] << S5P_CLKDIV_DMC0_COPY2_SHIFT) |
			(clkdiv_dmc0[i][7] << S5P_CLKDIV_DMC0_CORETI_SHIFT));

		exynos4_busfreq_table[i].clk_dmcdiv = tmp;
	}

	tmp = __raw_readl(S5P_CLKDIV_TOP);

	for (i = 0; i <  LV_END; i++) {
		tmp &= ~(S5P_CLKDIV_TOP_ACLK200_MASK |
			S5P_CLKDIV_TOP_ACLK100_MASK |
			S5P_CLKDIV_TOP_ACLK160_MASK |
			S5P_CLKDIV_TOP_ACLK133_MASK |
			S5P_CLKDIV_TOP_ONENAND_MASK);

		tmp |= ((clkdiv_top[i][0] << S5P_CLKDIV_TOP_ACLK200_SHIFT) |
			(clkdiv_top[i][1] << S5P_CLKDIV_TOP_ACLK100_SHIFT) |
			(clkdiv_top[i][2] << S5P_CLKDIV_TOP_ACLK160_SHIFT) |
			(clkdiv_top[i][3] << S5P_CLKDIV_TOP_ACLK133_SHIFT) |
			(clkdiv_top[i][4] << S5P_CLKDIV_TOP_ONENAND_SHIFT));

		exynos4_busfreq_table[i].clk_topdiv = tmp;
	}

	exynos4_set_bus_volt();

	for (i = 0; i < LV_END; i++) {
		ret = opp_add(dev, exynos4_busfreq_table[i].mem_clk,
				exynos4_busfreq_table[i].volt);
		if (ret) {
			dev_err(dev, "Fail to add opp entries.\n");
			kfree(data);
			return ret;
		}
	}

	table = cpufreq_frequency_get_table(0);
	if (IS_ERR(table)) {
		dev_err(dev, "Fail to get cpufreq table.\n");
		data->min_cpufreq = 2000000;
	}

	for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++) {
		freq = table[i].frequency;
		if (freq != CPUFREQ_ENTRY_INVALID && min_cpufreq > freq)
			min_cpufreq = freq;
	}

	data->min_cpufreq = min_cpufreq;
	data->dev = dev;

	/* Find max frequency */
	data->max_opp = opp_find_freq_floor(dev, &maxfreq);
	/* Find min frequency */
	data->curr_opp = opp_find_freq_ceil(dev, &minfreq);

	data->vdd_int = regulator_get(NULL, "vdd_int");
	if (IS_ERR(data->vdd_int)) {
		pr_err("failed to get resource %s\n", "vdd_int");
		return -ENODEV;
	}

	if (cpu_is_exynos4212()) {
		data->vdd_mif = regulator_get(NULL, "vdd_mif");
		if (IS_ERR(data->vdd_mif))
			pr_err("failed to get resource %s\n", "vdd_mif");
		else
			regulator_set_voltage(data->vdd_mif, 1100000,
					1100000);
	}

	if (cpufreq_register_notifier(&data->exynos4_busfreq_notifier,
				CPUFREQ_TRANSITION_NOTIFIER)) {
		pr_err("Failed to setup cpufreq notifier\n");
		goto err_cpufreq;
	}

	if (register_pm_notifier(&data->exynos4_buspm_notifier)) {
		pr_err("Failed to setup buspm notifier\n");
		goto err_pm;
	}

	data->use = true;

	if (register_reboot_notifier(&data->exynos4_reboot_notifier))
		pr_err("Failed to setup reboot notifier\n");

	platform_set_drvdata(pdev, data);

	return 0;
err_pm:
	cpufreq_unregister_notifier(&data->exynos4_busfreq_notifier,
				CPUFREQ_TRANSITION_NOTIFIER);
err_cpufreq:
	if (!IS_ERR(data->vdd_int))
		regulator_put(data->vdd_int);

	if (!IS_ERR(data->vdd_mif))
		regulator_put(data->vdd_mif);

	kfree(data);
	return -ENODEV;
}

static __devexit int exynos4_busfreq_remove(struct platform_device *pdev)
{
	struct busfreq_data *data = platform_get_drvdata(pdev);

	cpufreq_unregister_notifier(&data->exynos4_busfreq_notifier,
			CPUFREQ_TRANSITION_NOTIFIER);
	unregister_pm_notifier(&data->exynos4_buspm_notifier);
	unregister_reboot_notifier(&data->exynos4_reboot_notifier);
	regulator_put(data->vdd_int);
	regulator_put(data->vdd_mif);
	kfree(data);

	return 0;
}

static int exynos4_busfreq_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops exynos4_busfreq_pm = {
	.resume = exynos4_busfreq_resume,
};

static struct platform_driver exynos4_busfreq_driver = {
	.probe  = exynos4_busfreq_probe,
	.remove = __devexit_p(exynos4_busfreq_remove),
	.driver = {
		.name   = "exynos4-busfreq",
		.owner  = THIS_MODULE,
		.pm     = &exynos4_busfreq_pm,
	},
};

static int __init exynos4_busfreq_init(void)
{
	return platform_driver_register(&exynos4_busfreq_driver);
}
late_initcall(exynos4_busfreq_init);

static void __exit exynos4_busfreq_exit(void)
{
	platform_driver_unregister(&exynos4_busfreq_driver);
}
module_exit(exynos4_busfreq_exit);
