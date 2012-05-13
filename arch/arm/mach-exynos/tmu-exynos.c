/* linux/arch/arm/mach-exynos/tmu-exynos.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *      http://www.samsung.com
 *
 * EXYNOS - Thermal Management support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/slab.h>

#include <linux/irq.h>

#include <mach/regs-tmu.h>
#include <mach/cpufreq.h>
#include <mach/tmu.h>

#include <plat/cpu.h>

#define MUX_ADDR_VALUE 6

enum tmu_status_t {
	TMU_STATUS_INIT = 0,
	TMU_STATUS_NORMAL,
	TMU_STATUS_THROTTLED,
	TMU_STATUS_WARNING,
	TMU_STATUS_TRIPPED,
};

unsigned int already_limit;
static struct workqueue_struct  *tmu_monitor_wq;

static void tmu_tripped_cb(void)
{
}

static int get_cur_temp(struct tmu_info *info)
{
	unsigned char curr_temp;
	int temperature;

	/* After reading temperature code from register, compensating
	 * its value and calculating celsius temperatue,
	 * get current temperatue.
	 */
	curr_temp = __raw_readl(info->tmu_base + CURRENT_TEMP) & 0xff;

	/* compensate and calculate current temperature */
	temperature = curr_temp - info->te1 + TMU_DC_VALUE;
	if (temperature <= 0) {
		/* temperature code range are between min 25 and 125 */
		pr_err("%s: Current temperature is unreasonable\n", __func__);
	}

	return temperature;
}

#ifdef CONFIG_TMU_DEBUG
static int tmu_test_on;
static struct temperature_params in;
static int tmu_limit_on;
static int freq_limit_1st_throttle;
static int freq_limit_2nd_throttle;

static void print_temperature_params(struct tmu_info *info)
{
	struct tmu_data *data = info->dev->platform_data;

	pr_info("** temperature set value **\n"
		"throttling stop_temp  = %d, start_temp     = %d\n"
		"waring stop_temp      = %d, start_tmep     = %d\n"
		"tripping temp         = %d\n",
		data->ts.stop_throttle,
		data->ts.start_throttle,
		data->ts.stop_warning,
		data->ts.start_warning,
		data->ts.start_tripping);
}

static int __init get_temperature_params(char *str)
{
	unsigned int tmu_temp[6] = { (int)NULL, (int)NULL, (int)NULL,
			(int)NULL, (int)NULL, (int)NULL};

	get_options(str, 6, tmu_temp);
	tmu_test_on = tmu_temp[0];
	pr_info("@@@ tmu_test enable = %d\n", tmu_test_on);

	if (tmu_temp[1] > 0)
		in.stop_throttle = tmu_temp[1];
	if (tmu_temp[2] > 0)
		in.start_throttle = tmu_temp[2];
	if (tmu_temp[3] > 0)
		in.stop_warning = tmu_temp[3];
	if (tmu_temp[4] > 0)
		in.start_warning = tmu_temp[4];
	if (tmu_temp[5] > 0)
		in.start_tripping = tmu_temp[5];

	return 0;
}
early_param("tmu_test", get_temperature_params);

static int __init get_cpufreq_limit_param(char *str)
{
	int tmu_temp[3] = { (int)NULL, (int)NULL, (int)NULL };

	get_options(str, 3, tmu_temp);
	tmu_limit_on = tmu_temp[0];
	pr_info("@@@ tmu_limit_on = %d\n", tmu_limit_on);

	if (tmu_temp[1] > 0)
		freq_limit_1st_throttle = tmu_temp[1];
	if (tmu_temp[2] > 0)
		freq_limit_2nd_throttle = tmu_temp[2];
	pr_info("@@ 1st throttling : cpu_level = %d, 2nd cpu_level = %d\n",
		freq_limit_1st_throttle, freq_limit_2nd_throttle);

	return 0;
}
early_param("cpu_level", get_cpufreq_limit_param);

static void cur_temp_monitor(struct work_struct *work)
{
	char cur_temp;
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct tmu_info *info =
		 container_of(delayed_work, struct tmu_info, monitor);

	cur_temp = get_cur_temp(info);
	pr_info("current temp = %d\n", cur_temp);
	queue_delayed_work_on(0, tmu_monitor_wq, &info->monitor,
			usecs_to_jiffies(200 * 1000));
}
#endif

static void tmu_monitor(struct work_struct *work)
{
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct tmu_info *info =
		container_of(delayed_work, struct tmu_info, polling);
	struct tmu_data *data = info->dev->platform_data;
	char cur_temp;

#ifdef CONFIG_TMU_DEBUG
	cancel_delayed_work(&info->monitor);
#endif
	cur_temp = get_cur_temp(info);
	pr_info("Current: %dc, FLAG=%d\n",
			cur_temp, info->tmu_state);

	switch (info->tmu_state) {
	case TMU_STATUS_NORMAL:
#ifdef CONFIG_TMU_DEBUG
		queue_delayed_work_on(0, tmu_monitor_wq, &info->monitor,
		usecs_to_jiffies(200 * 1000));
#endif
		cancel_delayed_work(&info->polling);
		enable_irq(info->irq);
		break;
	case TMU_STATUS_THROTTLED:
		if (cur_temp >= data->ts.start_warning) {
			info->tmu_state = TMU_STATUS_WARNING;
			exynos_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			already_limit = 0;
		} else if (cur_temp > data->ts.stop_throttle &&
				cur_temp < data->ts.start_warning &&
							!already_limit) {
			exynos_cpufreq_upper_limit(DVFS_LOCK_ID_TMU,
				data->cpulimit.throttle_freq);
			already_limit = 1;
		} else if (cur_temp <= data->ts.stop_throttle) {
			info->tmu_state = TMU_STATUS_NORMAL;
			exynos_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			pr_info("Freq limit is released!!\n");
			already_limit = 0;
		}
		queue_delayed_work_on(0, tmu_monitor_wq,
		&info->polling, usecs_to_jiffies(200 * 1000));
		break;
	case TMU_STATUS_WARNING:
		if (cur_temp >= data->ts.start_tripping) {
			info->tmu_state = TMU_STATUS_TRIPPED;
			already_limit = 0;
		} else if (cur_temp > data->ts.stop_warning && \
				cur_temp < data->ts.start_tripping &&
							!already_limit) {
			exynos_cpufreq_upper_limit(DVFS_LOCK_ID_TMU,
				data->cpulimit.warning_freq);
			already_limit = 1;
		} else if (cur_temp <= data->ts.stop_warning) {
			info->tmu_state = TMU_STATUS_THROTTLED;
			exynos_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
			already_limit = 0;
		}
		queue_delayed_work_on(0, tmu_monitor_wq,
				&info->polling, usecs_to_jiffies(200 * 1000));
		break;
	case TMU_STATUS_TRIPPED:
		tmu_tripped_cb();
		queue_delayed_work_on(0, tmu_monitor_wq,
				&info->polling, usecs_to_jiffies(3000 * 1000));
	default:
	    break;
	}
	return;
}

static void pm_tmu_save(struct tmu_info *info)
{
	info->reg_save[0] = __raw_readl(info->tmu_base + TMU_CON);
	info->reg_save[1] = __raw_readl(info->tmu_base + SAMPLING_INTERNAL);
	info->reg_save[2] = __raw_readl(info->tmu_base + CNT_VALUE0);
	info->reg_save[3] = __raw_readl(info->tmu_base + CNT_VALUE1);
	info->reg_save[4] = __raw_readl(info->tmu_base + INTEN);

	if (soc_is_exynos4210()) {
		info->reg_save[5] = __raw_readl(info->tmu_base + THRESHOLD_TEMP);
		info->reg_save[6] = __raw_readl(info->tmu_base + TRIG_LEV0);
		info->reg_save[7] = __raw_readl(info->tmu_base + TRIG_LEV1);
		info->reg_save[8] = __raw_readl(info->tmu_base + TRIG_LEV2);
		info->reg_save[9] = __raw_readl(info->tmu_base + TRIG_LEV3);
	} else
		info->reg_save[5] = __raw_readl(info->tmu_base + THD_TEMP_RISE);
}

static void pm_tmu_restore(struct tmu_info *info)
{
	__raw_writel(info->reg_save[0], info->tmu_base + TMU_CON);
	__raw_writel(info->reg_save[1], info->tmu_base + SAMPLING_INTERNAL);
	__raw_writel(info->reg_save[2], info->tmu_base + CNT_VALUE0);
	__raw_writel(info->reg_save[3], info->tmu_base + CNT_VALUE1);
	__raw_writel(info->reg_save[4], info->tmu_base + INTEN);

	if (soc_is_exynos4210()) {
		__raw_writel(info->reg_save[5], info->tmu_base + THRESHOLD_TEMP);
		__raw_writel(info->reg_save[6], info->tmu_base + TRIG_LEV0);
		__raw_writel(info->reg_save[7], info->tmu_base + TRIG_LEV1);
		__raw_writel(info->reg_save[8], info->tmu_base + TRIG_LEV2);
		__raw_writel(info->reg_save[9], info->tmu_base + TRIG_LEV3);
	} else
		__raw_writel(info->reg_save[5], info->tmu_base + THD_TEMP_RISE);
}

static int exynos4210_tmu_init(struct tmu_info *info)
{
	struct tmu_data *data = info->dev->platform_data;
	int con;
	unsigned int te_temp;
	unsigned int temp_threshold;
	unsigned int temp_throttle, temp_warning, temp_trip;

	/* get the compensation parameter */
	te_temp = __raw_readl(info->tmu_base + TRIMINFO);
	info->te1 = te_temp & TRIM_INFO_MASK;
	info->te2 = ((te_temp >> 8) & TRIM_INFO_MASK);

	if ((EFUSE_MIN_VALUE > info->te1) || (info->te1 > EFUSE_MAX_VALUE)
		||  (info->te2 != 0))
		info->te1 = data->efuse_value;

	/* Convert celsius temperature value to temperature code value
	* such as threshold_level, 1st throttle, 2nd throttle,
	* tripping temperature.
	*/
	temp_threshold = data->ts.stop_throttle
			+ info->te1 - TMU_DC_VALUE;
	temp_throttle = data->ts.start_throttle
			- data->ts.stop_throttle;
	temp_warning = data->ts.start_warning
			- data->ts.stop_throttle;
	temp_trip = data->ts.start_tripping
			- data->ts.stop_throttle;

	/* Set interrupt trigger level */
	__raw_writel(temp_threshold, info->tmu_base + THRESHOLD_TEMP);
	__raw_writel(temp_throttle, info->tmu_base + TRIG_LEV0);
	__raw_writel(temp_warning, info->tmu_base + TRIG_LEV1);
	__raw_writel(temp_trip, info->tmu_base + TRIG_LEV2);

	/* Clear interrupt ot eliminate dummy interrupt signal */
	__raw_writel(INTCLEARALL, info->tmu_base + INTCLEAR);

	/* Need to initail regsiter setting after getting parameter info */
	/* [28:23] vref [11:8] slope - Tunning parameter */
	/* TMU core enable */
	con = __raw_readl(info->tmu_base + TMU_CON);
	con |= (data->slope | CORE_EN);
	__raw_writel(con, info->tmu_base + TMU_CON);

	__raw_writel(INTEN0 | INTEN1 | INTEN2, info->tmu_base + INTEN);

	return 0;

}
static int exynos_tmu_init(struct tmu_info *info)
{
	struct tmu_data *data = info->dev->platform_data;
	unsigned int te_temp, con;
	unsigned int temp_throttle, temp_warning, temp_trip;
	unsigned int rising_thr = 0, cooling_thr = 0;

	/* must reload for using efuse value at EXYNOS4212 */
	__raw_writel(TRIMINFO_RELOAD, info->tmu_base + TRIMINFO_CON);

	/* get the compensation parameter */
	te_temp = __raw_readl(info->tmu_base + TRIMINFO);
	info->te1 = te_temp & TRIM_INFO_MASK;
	info->te2 = ((te_temp >> 8) & TRIM_INFO_MASK);

	if ((EFUSE_MIN_VALUE > info->te1) || (info->te1 > EFUSE_MAX_VALUE)
		||  (info->te2 != 0))
		info->te1 = data->efuse_value;

	/*Get RISING & FALLING Threshold value */
	temp_throttle = data->ts.start_throttle
			+ info->te1 - TMU_DC_VALUE;
	temp_warning = data->ts.start_warning
			+ info->te1 - TMU_DC_VALUE;
	temp_trip =  data->ts.start_tripping
			+ info->te1 - TMU_DC_VALUE;

	rising_thr = (temp_throttle | (temp_warning<<8) | \
		(temp_trip<<16));

	/* Set interrupt level */
	__raw_writel(rising_thr, info->tmu_base + THD_TEMP_RISE);
	__raw_writel(cooling_thr, info->tmu_base + THD_TEMP_FALL);

	/* Set TMU status */
	info->tmu_state = TMU_STATUS_INIT;

	/* Set frequecny level */
	exynos_cpufreq_get_level(800000, &data->cpulimit.throttle_freq);
	exynos_cpufreq_get_level(200000, &data->cpulimit.warning_freq);

	/* Need to initail regsiter setting after getting parameter info */
	/* [28:23] vref [11:8] slope - Tunning parameter */
	__raw_writel(data->slope, info->tmu_base + TMU_CON);

	__raw_writel((CLEAR_RISE_INT | CLEAR_FALL_INT), info->tmu_base + INTCLEAR);

	/* TMU core enable */
	con = __raw_readl(info->tmu_base + TMU_CON);
	con |= (MUX_ADDR_VALUE<<20 | CORE_EN);

	__raw_writel(con, info->tmu_base + TMU_CON);

	/*LEV0 LEV1 LEV2 interrupt enable */
	__raw_writel(INTEN_RISE0 | INTEN_RISE1 | INTEN_RISE2,	\
		     info->tmu_base + INTEN);

	pr_info("TMU is initialized!!");

	return 0;
}

static int tmu_initialize(struct platform_device *pdev)
{
	struct tmu_info *info = platform_get_drvdata(pdev);
	unsigned int en;
	int ret;

	en = (__raw_readl(info->tmu_base + TMU_STATUS) & 0x1);

	if (!en) {
		dev_err(&pdev->dev, "failed to start tmu drvier\n");
		return -ENOENT;
	}

	if (soc_is_exynos4210())
		ret = exynos4210_tmu_init(info);
	else
		ret = exynos_tmu_init(info);

	return ret;
}

static irqreturn_t tmu_irq(int irq, void *id)
{
	struct tmu_info *info = id;
	unsigned int status;

	disable_irq_nosync(irq);

	status = __raw_readl(info->tmu_base + INTSTAT);

	if (status & INTSTAT_RISE2) {
		pr_info("Tripping interrupt occured!!!!\n");
		info->tmu_state = TMU_STATUS_TRIPPED;
		__raw_writel(INTCLEAR_RISE2, info->tmu_base + INTCLEAR);
		tmu_tripped_cb();
	} else if (status & INTSTAT_RISE1) {
		pr_info("Warning interrupt occured!!!!\n");
		__raw_writel(INTCLEAR_RISE1, info->tmu_base + INTCLEAR);
		info->tmu_state = TMU_STATUS_WARNING;
		queue_delayed_work_on(0, tmu_monitor_wq,
				&info->polling, usecs_to_jiffies(200 * 1000));
	} else if (status & INTSTAT_RISE0) {
		pr_info("Throttling interrupt occured!!!!\n");
		__raw_writel(INTCLEAR_RISE0, info->tmu_base + INTCLEAR);
		info->tmu_state = TMU_STATUS_THROTTLED;
		queue_delayed_work_on(0, tmu_monitor_wq,
				&info->polling, usecs_to_jiffies(200 * 1000));
	} else {
		pr_err("%s: TMU interrupt error\n", __func__);
		return -ENODEV;
	}

	return IRQ_HANDLED;
}

static irqreturn_t exynos4210_tmu_irq(int irq, void *id)
{
	struct tmu_info *info = id;
	unsigned int status;

	disable_irq_nosync(irq);

	status = __raw_readl(info->tmu_base + INTSTAT);

	if (status & INTSTAT2) {
		pr_info("Tripping interrupt occured!!!!\n");
		info->tmu_state = TMU_STATUS_TRIPPED;
		__raw_writel(INTCLEAR2, info->tmu_base + INTCLEAR);
		tmu_tripped_cb();
	} else if (status & INTSTAT1) {
		pr_info("Warning interrupt occured!!!!\n");
		__raw_writel(INTCLEAR1, info->tmu_base + INTCLEAR);
		info->tmu_state = TMU_STATUS_WARNING;
		queue_delayed_work_on(0, tmu_monitor_wq,
				&info->polling, usecs_to_jiffies(500 * 1000));
	} else if (status & INTSTAT0) {
		pr_info("Throttling interrupt occured!!!!\n");
		__raw_writel(INTCLEAR0, info->tmu_base + INTCLEAR);
		info->tmu_state = TMU_STATUS_THROTTLED;
		queue_delayed_work_on(0, tmu_monitor_wq,
				&info->polling, usecs_to_jiffies(500 * 1000));
	} else {
		pr_err("%s: TMU interrupt error\n", __func__);
		return -ENODEV;
	}

	return IRQ_HANDLED;
}

static int __devinit tmu_probe(struct platform_device *pdev)
{
	struct tmu_info *info;
	struct resource *res;
	int	ret = 0;

	pr_debug("%s: probe=%p\n", __func__, pdev);

	info = kzalloc(sizeof(struct tmu_info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "failed to alloc memory!\n");
		ret = -ENOMEM;
		goto err_nores;
	}
	platform_set_drvdata(pdev, info);
	info->dev = &pdev->dev;

	info->irq = platform_get_irq(pdev, 0);
	if (info->irq < 0) {
		dev_err(&pdev->dev, "no irq for thermal\n");
		return -ENOENT;
	}
	if (soc_is_exynos4210())
		ret = request_irq(info->irq, exynos4210_tmu_irq,
				IRQF_DISABLED,  "tmu interrupt", info);
	else
		ret = request_irq(info->irq, tmu_irq,
				IRQF_DISABLED,  "tmu interrupt", info);
	if (ret) {
		dev_err(&pdev->dev, "IRQ%d error %d\n", info->irq, ret);
		goto err_noirq;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource\n");
		return -ENOENT;
	}

	info->ioarea = request_mem_region(res->start,
			res->end-res->start+1, pdev->name);
	if (!(info->ioarea)) {
		dev_err(&pdev->dev, "failed to reserve memory region\n");
		ret = -ENOENT;
		goto err_nores;
	}

	info->tmu_base = ioremap(res->start, (res->end - res->start) + 1);
	if (!(info->tmu_base)) {
		dev_err(&pdev->dev, "failed ioremap()\n");
		ret = -EINVAL;
		goto err_nomap;
	}

	tmu_monitor_wq = create_freezable_workqueue("tmu");
	if (!tmu_monitor_wq) {
		pr_info("Creation of tmu_monitor_wq failed\n");
		return -EFAULT;
	}

#ifdef CONFIG_TMU_DEBUG
	INIT_DELAYED_WORK_DEFERRABLE(&info->monitor, cur_temp_monitor);
	queue_delayed_work_on(0, tmu_monitor_wq, &info->monitor,
			usecs_to_jiffies(200 * 1000));
#endif
	INIT_DELAYED_WORK_DEFERRABLE(&info->polling, tmu_monitor);

	ret = tmu_initialize(pdev);
	if (ret)
		goto err_noinit;

	return ret;

err_noinit:
	free_irq(info->irq, info);
err_noirq:
	iounmap(info->tmu_base);
err_nomap:
	release_resource(info->ioarea);
err_nores:
	return ret;
}

static int __devinit tmu_remove(struct platform_device *pdev)
{
	struct tmu_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, (void *)pdev);
	iounmap(info->tmu_base);

	pr_info("%s is removed\n", dev_name(&pdev->dev));
	return 0;
}

#ifdef CONFIG_PM
static int tmu_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tmu_info *info = platform_get_drvdata(pdev);
	pm_tmu_save(info);

	return 0;
}

static int tmu_resume(struct platform_device *pdev)
{
	struct tmu_info *info = platform_get_drvdata(pdev);
	pm_tmu_restore(info);

	return 0;
}

#else
#define tmu_suspend	NULL
#define tmu_resume	NULL
#endif

static struct platform_driver tmu_driver = {
	.probe		= tmu_probe,
	.remove		= tmu_remove,
	.suspend	= tmu_suspend,
	.resume		= tmu_resume,
	.driver		= {
		.name	=	"tmu",
		.owner	=	THIS_MODULE,
		},
};

static int __init tmu_driver_init(void)
{
	return platform_driver_register(&tmu_driver);
}

late_initcall(tmu_driver_init);
