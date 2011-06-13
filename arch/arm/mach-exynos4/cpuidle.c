/* linux/arch/arm/mach-exynos4/cpuidle.c
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

#include <asm/proc-fns.h>

#include <mach/regs-pmu.h>
#include <mach/pmu.h>

#include <plat/exynos4.h>

#define REG_DIRECTGO_ADDR	(exynos4_subrev() == 0 ?\
				(S5P_VA_SYSRAM + 0x24) : S5P_INFORM7)
#define REG_DIRECTGO_FLAG	(exynos4_subrev() == 0 ?\
				(S5P_VA_SYSRAM + 0x20) : S5P_INFORM6)

/*
 * This function is called by exynos_enter_lp() in idle.S.
 * The contents of the SVC mode stack area should be updated
 * to physical memory until entering AFTR mode. The memory
 * contents are used by cpu_resume().
 */
void exynos4_lpidle_cache_clean(void *stack_addr)
{
	void *saveblk;
	unsigned long saveblk_size;

	saveblk = stack_addr - 36;
	/*
	 * Refer to v7 cpu_suspend function.
	 * From saveblk to stack_addr + (4 * 4) + (4 * 8)
	 * 4byte * (v:p offset, virt sp, ret fn, phy resume fn)
	 * 4byte * (Saving Coporcessor and MMU control register: 8)
	 */
	saveblk_size = 84;
	outer_cache.clean_range(virt_to_phys(saveblk), saveblk_size);

	/* To clean sleep_save_sp area */
	outer_cache.clean_range(virt_to_phys(cpu_resume), 64);
}

void exynos4_set_core0_pwroff(void)
{
	unsigned long tmp;
	/*
	 * Setting Central Sequence Register for power down mode
	 */
	tmp = __raw_readl(S5P_CENTRAL_SEQ_CONFIGURATION);
	tmp &= ~(S5P_CENTRAL_LOWPWR_CFG);
	__raw_writel(tmp, S5P_CENTRAL_SEQ_CONFIGURATION);

	cpu_do_idle();
}

static void exynos4_set_wakeupmask(void)
{
	__raw_writel(0x0000ff3e, S5P_WAKEUP_MASK);
}

static int exynos4_enter_core0_aftr(struct cpuidle_device *dev,
				    struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;
	unsigned long tmp;

	local_irq_disable();
	do_gettimeofday(&before);

	exynos4_set_wakeupmask();

	__raw_writel(virt_to_phys(exynos4_idle_resume), REG_DIRECTGO_ADDR);
	__raw_writel(0xfcba0d10, REG_DIRECTGO_FLAG);

	/* Set value of power down register for aftr mode */
	exynos4_sys_powerdown_conf(SYS_AFTR);

	if (exynos4_enter_lp(0, PLAT_PHYS_OFFSET - PAGE_OFFSET) == 0) {

		/*
		 * Clear Central Sequence Register in exiting early wakeup
		 */
		tmp = __raw_readl(S5P_CENTRAL_SEQ_CONFIGURATION);
		tmp |= (S5P_CENTRAL_LOWPWR_CFG);
		__raw_writel(tmp, S5P_CENTRAL_SEQ_CONFIGURATION);

		goto early_wakeup;
	}

	cpu_init();

early_wakeup:

	/* Clear wakeup state register */
	__raw_writel(0x0, S5P_WAKEUP_STAT);

	do_gettimeofday(&after);

	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
		    (after.tv_usec - before.tv_usec);

	return idle_time;
}

static int exynos4_enter_idle(struct cpuidle_device *dev,
			      struct cpuidle_state *state);

static int exynos4_enter_lowpower(struct cpuidle_device *dev,
				  struct cpuidle_state *state);

static struct cpuidle_state exynos4_cpuidle_set[] = {
	[0] = {
		.enter			= exynos4_enter_idle,
		.exit_latency		= 1,
		.target_residency	= 100000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "IDLE",
		.desc			= "ARM clock gating(WFI)",
	},
	[1] = {
		.enter			= exynos4_enter_lowpower,
		.exit_latency		= 300,
		.target_residency	= 100000,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "LOW_POWER",
		.desc			= "ARM power down",
	},
};

static DEFINE_PER_CPU(struct cpuidle_device, exynos4_cpuidle_device);

static struct cpuidle_driver exynos4_idle_driver = {
	.name		= "exynos4_idle",
	.owner		= THIS_MODULE,
};

static int exynos4_enter_idle(struct cpuidle_device *dev,
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

static int exynos4_enter_lowpower(struct cpuidle_device *dev,
				  struct cpuidle_state *state)
{
	struct cpuidle_state *new_state = state;

	/* This mode only can be entered when Core1 is offline */
	if (cpu_online(1)) {
		BUG_ON(!dev->safe_state);
		new_state = dev->safe_state;
	}
	dev->last_state = new_state;

	if (new_state == &dev->states[0])
		return exynos4_enter_idle(dev, new_state);
	else
		return exynos4_enter_core0_aftr(dev, new_state);

	return exynos4_enter_idle(dev, new_state);
}

static int __init exynos4_init_cpuidle(void)
{
	int i, max_cpuidle_state, cpu_id;
	struct cpuidle_device *device;

	cpuidle_register_driver(&exynos4_idle_driver);

	for_each_cpu(cpu_id, cpu_online_mask) {
		device = &per_cpu(exynos4_cpuidle_device, cpu_id);
		device->cpu = cpu_id;

		if (cpu_id == 0)
			device->state_count = (sizeof(exynos4_cpuidle_set) /
					       sizeof(struct cpuidle_state));
		else
			device->state_count = 1;	/* Support IDLE only */

		max_cpuidle_state = device->state_count;

		for (i = 0; i < max_cpuidle_state; i++) {
			memcpy(&device->states[i], &exynos4_cpuidle_set[i],
					sizeof(struct cpuidle_state));
		}

		device->safe_state = &device->states[0];

		if (cpuidle_register_device(device)) {
			printk(KERN_ERR "CPUidle register device failed\n,");
			return -EIO;
		}
	}
	return 0;
}
device_initcall(exynos4_init_cpuidle);
