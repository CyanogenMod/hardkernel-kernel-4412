/* linux/arch/arm/mach-exynos/cpuidle-exynos5.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpuidle.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>

#include <asm/proc-fns.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>

#include <plat/pm.h>

#include <mach/regs-pmu5.h>
#include <mach/pm-core.h>
#include <mach/pmu.h>
#include <mach/regs-clock.h>

#define REG_DIRECTGO_ADDR	(S5P_VA_SYSRAM + 0x24)
#define REG_DIRECTGO_FLAG	(S5P_VA_SYSRAM + 0x20)

extern unsigned long sys_pwr_conf_addr;

static int exynos5_enter_idle(struct cpuidle_device *dev,
			      struct cpuidle_state *state);

static int exynos5_enter_lowpower(struct cpuidle_device *dev,
				  struct cpuidle_state *state);

static struct cpuidle_state exynos5_cpuidle_set[] = {
	[0] = {
		.enter			= exynos5_enter_idle,
		.exit_latency		= 1,
		.target_residency	= 10000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "IDLE",
		.desc			= "ARM clock gating(WFI)",
	},
	[1] = {
		.enter			= exynos5_enter_lowpower,
		.exit_latency		= 300,
		.target_residency	= 10000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "LOW_POWER",
		.desc			= "ARM power down",
	},
};

static DEFINE_PER_CPU(struct cpuidle_device, exynos5_cpuidle_device);

static struct cpuidle_driver exynos5_idle_driver = {
	.name		= "exynos5_idle",
	.owner		= THIS_MODULE,
};

static int exynos5_enter_idle(struct cpuidle_device *dev,
			      struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;

	local_irq_disable();
	do_gettimeofday(&before);

	cpu_do_idle();

	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
		    (after.tv_usec - before.tv_usec);

	return idle_time;
}

void exynos5_flush_cache(void *addr, phys_addr_t phy_ttb_base)
{
	flush_cache_all();
}

static void exynos5_set_wakeupmask(void)
{
	__raw_writel(0x0000ff3e, EXYNOS5_WAKEUP_MASK);
}

static inline void vfp_enable(void *unused)
{
	u32 access = get_copro_access();

	/*
	 * Enable full access to VFP (cp10 and cp11)
	 */
	set_copro_access(access | CPACC_FULL(10) | CPACC_FULL(11));
}

static int exynos5_enter_core0_aftr(struct cpuidle_device *dev,
				    struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;
	unsigned long tmp;

	local_irq_disable();
	do_gettimeofday(&before);

	exynos5_set_wakeupmask();

	__raw_writel(virt_to_phys(exynos5_idle_resume), REG_DIRECTGO_ADDR);
	__raw_writel(0xfcba0d10, REG_DIRECTGO_FLAG);

	/* Set value of power down register for aftr mode */
	exynos5_sys_powerdown_conf(SYS_AFTR);

	exynos4_reset_assert_ctrl(1);

	if (exynos5_enter_lp(0, PLAT_PHYS_OFFSET - PAGE_OFFSET) == 0) {
		/*
		 * Clear Central Sequence Register in exiting early wakeup
		 */
		tmp = __raw_readl(EXYNOS5_CENTRAL_SEQ_CONFIGURATION);
		tmp |= EXYNOS5_CENTRAL_LOWPWR_CFG;
		__raw_writel(tmp, EXYNOS5_CENTRAL_SEQ_CONFIGURATION);

		goto early_wakeup;
	}
	flush_tlb_all();

	cpu_init();

	vfp_enable(NULL);

early_wakeup:
	exynos4_reset_assert_ctrl(0);

	/* Clear wakeup state register */
	__raw_writel(0x0, EXYNOS5_WAKEUP_STAT);

	do_gettimeofday(&after);

	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
		    (after.tv_usec - before.tv_usec);

	return idle_time;
}

static int exynos5_enter_lowpower(struct cpuidle_device *dev,
				  struct cpuidle_state *state)
{
	struct cpuidle_state *new_state = state;
	unsigned int tmp;

	/* This mode only can be entered when only Core0 is online */
	if (num_online_cpus() != 1) {
		BUG_ON(!dev->safe_state);
		new_state = dev->safe_state;
	}
	dev->last_state = new_state;

	if (new_state == &dev->states[0])
		return exynos5_enter_idle(dev, new_state);

	tmp = __raw_readl(EXYNOS5_CENTRAL_SEQ_OPTION);
	tmp = (EXYNOS5_USE_STANDBYWFI_ARM_CORE0 |
		EXYNOS5_USE_STANDBYWFE_ARM_CORE0);
	__raw_writel(tmp, EXYNOS5_CENTRAL_SEQ_OPTION);

	/*In this time, only support aftr mode*/
	return exynos5_enter_core0_aftr(dev, new_state);
}
static int exynos5_cpuidle_notifier_event(struct notifier_block *this,
					  unsigned long event,
					  void *ptr)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		disable_hlt();
		pr_debug("PM_SUSPEND_PREPARE for CPUIDLE\n");
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		enable_hlt();
		pr_debug("PM_POST_SUSPEND for CPUIDLE\n");
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block exynos5_cpuidle_notifier = {
	.notifier_call = exynos5_cpuidle_notifier_event,
};

#ifdef CONFIG_EXYNOS5_ENBLE_CLOCK_DOWN
static void __init exynos5_core_down_clk(void)
{
	unsigned int tmp;

	tmp = __raw_readl(EXYNOS5_PWR_CTRL1);

	tmp &= ~(PWR_CTRL1_CORE2_DOWN_MASK | PWR_CTRL1_CORE1_DOWN_MASK);

	/* set arm clock divider value on idle state */
	tmp |= ((0x7 << PWR_CTRL1_CORE2_DOWN_RATIO) |
		(0x7 << PWR_CTRL1_CORE1_DOWN_RATIO));

	tmp |= (PWR_CTRL1_DIV2_DOWN_EN |
		PWR_CTRL1_DIV1_DOWN_EN |
		PWR_CTRL1_USE_CORE1_WFE |
		PWR_CTRL1_USE_CORE0_WFE |
		PWR_CTRL1_USE_CORE1_WFI |
		PWR_CTRL1_USE_CORE0_WFI);

	__raw_writel(tmp, EXYNOS5_PWR_CTRL1);

	tmp = __raw_readl(EXYNOS5_PWR_CTRL2);

	tmp &= ~(PWR_CTRL2_DUR_STANDBY2_MASK | PWR_CTRL2_DUR_STANDBY1_MASK |
		PWR_CTRL2_CORE2_UP_MASK | PWR_CTRL2_CORE1_UP_MASK);

	/* set duration value on middle wakeup step */
	tmp |=  ((0x1 << PWR_CTRL2_DUR_STANDBY2) |
		 (0x1 << PWR_CTRL2_DUR_STANDBY1));

	/* set arm clock divier value on middle wakeup step */
	tmp |= ((0x1 << PWR_CTRL2_CORE2_UP_RATIO) |
		(0x1 << PWR_CTRL2_CORE1_UP_RATIO));

	/* Set PWR_CTRL2 register to use step up for arm clock */
	tmp |= (PWR_CTRL2_DIV2_UP_EN | PWR_CTRL2_DIV1_UP_EN);

	__raw_writel(tmp, EXYNOS5_PWR_CTRL2);
	printk(KERN_INFO "Exynos5 : ARM Clock down on idle mode is enabled\n");
}
#else
#define exynos5_core_down_clk()	do { } while (0)
#endif

static int __init exynos5_init_cpuidle(void)
{
	int i, max_cpuidle_state, cpu_id;
	struct cpuidle_device *device;

	exynos5_core_down_clk();

	cpuidle_register_driver(&exynos5_idle_driver);

	for_each_cpu(cpu_id, cpu_online_mask) {
		device = &per_cpu(exynos5_cpuidle_device, cpu_id);
		device->cpu = cpu_id;

		if (cpu_id == 0)
			device->state_count = ARRAY_SIZE(exynos5_cpuidle_set);
		else
			device->state_count = 1;	/* Support IDLE only */

		max_cpuidle_state = device->state_count;

		for (i = 0; i < max_cpuidle_state; i++) {
			memcpy(&device->states[i], &exynos5_cpuidle_set[i],
					sizeof(struct cpuidle_state));
		}

		device->safe_state = &device->states[0];

		if (cpuidle_register_device(device)) {
			printk(KERN_ERR "CPUidle register device failed\n,");
			return -EIO;
		}
	}

	register_pm_notifier(&exynos5_cpuidle_notifier);
	sys_pwr_conf_addr = (unsigned long)EXYNOS5_CENTRAL_SEQ_CONFIGURATION;

	return 0;
}
device_initcall(exynos5_init_cpuidle);
