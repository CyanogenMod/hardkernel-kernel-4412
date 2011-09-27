/* linux/arch/arm/mach-exynos/busfreq_opp.c
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
#include <mach/busfreq.h>

#include <plat/map-s5p.h>
#include <plat/gpio-cfg.h>
#include <plat/cputype.h>

static struct opp *busfreq_monitor(struct busfreq_data *data)
{
	struct opp *opp;
	unsigned long lockfreq;
	unsigned long newfreq = 0;

	if (!cpu_is_exynos4210())
		newfreq = 160000;

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
		if (newfreq > currfreq) {
			if (!IS_ERR(data->vdd_mif)) {
				regulator_set_voltage(data->vdd_mif, voltage,
						voltage);
				voltage = data->get_int_volt(newfreq);
			}
			regulator_set_voltage(data->vdd_int, voltage,
					voltage);
		}

		data->target(opp);

		if (newfreq < currfreq) {
			if (!IS_ERR(data->vdd_mif)) {
				regulator_set_voltage(data->vdd_mif, voltage,
						voltage);
				voltage = data->get_int_volt(newfreq);
			}
			regulator_set_voltage(data->vdd_int, voltage,
					voltage);
		}
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
	unsigned long freq = opp_get_freq(data->max_opp);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		data->use = false;
		if (!IS_ERR(data->vdd_mif)) {
			regulator_set_voltage(data->vdd_mif, voltage,
					voltage);
			voltage = data->get_int_volt(freq);
		}
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
	unsigned long freq = opp_get_freq(data->max_opp);

	if (!IS_ERR(data->vdd_mif)) {
		regulator_set_voltage(data->vdd_mif, voltage,
				voltage);
		voltage = data->get_int_volt(freq);
	}
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

static __devinit int exynos4_busfreq_probe(struct platform_device *pdev)
{
	struct busfreq_data *data;
	unsigned int val;

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
		exynos4_busfreq_reboot_event;

	if (cpu_is_exynos4210()) {
		data->init = exynos4210_init;
		data->target = exynos4210_target;
		data->get_int_volt = NULL;
	} else {
		data->init = exynos4212_init;
		data->target = exynos4212_target;
		data->get_int_volt = exynos4212_get_int_volt;
	}

	if (data->init(&pdev->dev, data)) {
		pr_err("Failed to init busfreq.\n");
		goto err_cpufreq;
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
