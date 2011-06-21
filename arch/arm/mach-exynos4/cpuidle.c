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
#include <linux/suspend.h>
#include <linux/platform_device.h>

#include <asm/proc-fns.h>

#include <mach/regs-clock.h>
#include <mach/regs-pmu.h>
#include <mach/pmu.h>

#include <plat/exynos4.h>
#include <plat/pm.h>
#include <plat/devs.h>

#define REG_DIRECTGO_ADDR	(exynos4_subrev() == 0 ?\
				(S5P_VA_SYSRAM + 0x24) : S5P_INFORM7)
#define REG_DIRECTGO_FLAG	(exynos4_subrev() == 0 ?\
				(S5P_VA_SYSRAM + 0x20) : S5P_INFORM6)

struct check_device_op {
	void __iomem	*base;
	struct platform_device	*pdev;
	unsigned int ch;
};

static struct check_device_op chk_dev_op[] = {
#ifdef CONFIG_I2C_S3C2410
	{.base = 0, .pdev = &s3c_device_i2c0, .ch = 0},
#if defined(CONFIG_S3C_DEV_I2C1)
	{.base = 0, .pdev = &s3c_device_i2c1, .ch = 1},
#endif
#if defined(CONFIG_S3C_DEV_I2C2)
	{.base = 0, .pdev = &s3c_device_i2c2, .ch = 2},
#endif
#if defined(CONFIG_S3C_DEV_I2C3)
	{.base = 0, .pdev = &s3c_device_i2c3, .ch = 3},
#endif
#if defined(CONFIG_S3C_DEV_I2C4)
	{.base = 0, .pdev = &s3c_device_i2c4, .ch = 4},
#endif
#if defined(CONFIG_S3C_DEV_I2C5)
	{.base = 0, .pdev = &s3c_device_i2c5, .ch = 5},
#endif
#if defined(CONFIG_S3C_DEV_I2C6)
	{.base = 0, .pdev = &s3c_device_i2c6, .ch = 6},
#endif
#if defined(CONFIG_S3C_DEV_I2C7)
	{.base = 0, .pdev = &s3c_device_i2c7, .ch = 7},
#endif
#endif
	{.base = 0, .pdev = NULL, .ch = 0xffffffff},
};

static int check_i2c_op(void)
{
	unsigned int val, i, chan;
	void __iomem *base_addr;

	for (i = 0; i < ARRAY_SIZE(chk_dev_op); i++) {
		if (chk_dev_op[i].pdev == NULL)
			break;
		chan = chk_dev_op[i].ch;

		if (chan != 0xffffffff) {
			base_addr = chk_dev_op[i].base;

			val = __raw_readl(base_addr + 0x04);

			if (val & (1<<5)) {
				printk(KERN_INFO "I2C[%d] is working\n", chan);
				return 1;
			}
		} else
			break;
	}
	return 0;
}

static struct sleep_save exynos4_lpa_save[] = {
	/* CMU side */
	SAVE_ITEM(S5P_CLKSRC_MASK_TOP),
	SAVE_ITEM(S5P_CLKSRC_MASK_CAM),
	SAVE_ITEM(S5P_CLKSRC_MASK_TV),
	SAVE_ITEM(S5P_CLKSRC_MASK_LCD0),
	SAVE_ITEM(S5P_CLKSRC_MASK_LCD1),
	SAVE_ITEM(S5P_CLKSRC_MASK_MAUDIO),
	SAVE_ITEM(S5P_CLKSRC_MASK_FSYS),
	SAVE_ITEM(S5P_CLKSRC_MASK_PERIL0),
	SAVE_ITEM(S5P_CLKSRC_MASK_PERIL1),
	SAVE_ITEM(S5P_CLKSRC_MASK_DMC),
};

static struct sleep_save exynos4_set_clksrc[] = {
	{ .reg = S5P_CLKSRC_MASK_TOP			, .val = 0x00000001, },
	{ .reg = S5P_CLKSRC_MASK_CAM			, .val = 0x11111111, },
	{ .reg = S5P_CLKSRC_MASK_TV			, .val = 0x00000111, },
	{ .reg = S5P_CLKSRC_MASK_LCD0			, .val = 0x00001111, },
	{ .reg = S5P_CLKSRC_MASK_LCD1			, .val = 0x00001111, },
	{ .reg = S5P_CLKSRC_MASK_MAUDIO			, .val = 0x00000001, },
	{ .reg = S5P_CLKSRC_MASK_FSYS			, .val = 0x01011111, },
	{ .reg = S5P_CLKSRC_MASK_PERIL0			, .val = 0x01111111, },
	{ .reg = S5P_CLKSRC_MASK_PERIL1			, .val = 0x01110111, },
	{ .reg = S5P_CLKSRC_MASK_DMC			, .val = 0x00010000, },
};

static int exynos4_check_enter(void)
{
	unsigned int ret;
	unsigned int check_val;

	ret = 0;

	/* Check UART for console is empty */
	check_val = __raw_readl(S5P_VA_UART(CONFIG_S3C_LOWLEVEL_UART_PORT) +
				0x18);

	ret = ((check_val >> 16) & 0xff);

	return ret;
}

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

static int exynos4_enter_core0_lpa(struct cpuidle_device *dev,
				   struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;
	unsigned long tmp;

	s3c_pm_do_save(exynos4_lpa_save, ARRAY_SIZE(exynos4_lpa_save));

	/*
	 * Before enter central sequence mode, clock src register have to set
	 */
	s3c_pm_do_restore_core(exynos4_set_clksrc,
			       ARRAY_SIZE(exynos4_set_clksrc));

	local_irq_disable();

	do_gettimeofday(&before);

	/*
	 * Unmasking all wakeup source.
	 */
	__raw_writel(0x0, S5P_WAKEUP_MASK);

	/* ensure at least INFORM0 has the resume address */
	__raw_writel(virt_to_phys(s3c_cpu_resume), S5P_INFORM0);

	__raw_writel(virt_to_phys(s3c_cpu_resume), REG_DIRECTGO_ADDR);
	__raw_writel(0xfcba0d10, REG_DIRECTGO_FLAG);

	__raw_writel(S5P_CHECK_LPA, S5P_INFORM1);
	exynos4_sys_powerdown_conf(SYS_LPA);

	do {
		/* Waiting for flushing UART fifo */
	} while (exynos4_check_enter());

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

	s3c_pm_do_restore_core(exynos4_lpa_save,
			       ARRAY_SIZE(exynos4_lpa_save));

	/* For release retention */
	__raw_writel((1 << 28), S5P_PAD_RET_GPIO_OPTION);
	__raw_writel((1 << 28), S5P_PAD_RET_UART_OPTION);
	__raw_writel((1 << 28), S5P_PAD_RET_MMCA_OPTION);
	__raw_writel((1 << 28), S5P_PAD_RET_MMCB_OPTION);
	__raw_writel((1 << 28), S5P_PAD_RET_EBIA_OPTION);
	__raw_writel((1 << 28), S5P_PAD_RET_EBIB_OPTION);

early_wakeup:

	/* Clear wakeup state register */
	__raw_writel(0x0, S5P_WAKEUP_STAT);

	__raw_writel(0x0, S5P_WAKEUP_MASK);

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

static unsigned int cpu_core;
static unsigned int old_div;
static DEFINE_SPINLOCK(idle_lock);

static int exynos4_enter_idle(struct cpuidle_device *dev,
			      struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;
	int cpu;
	unsigned int tmp;

	local_irq_disable();
	do_gettimeofday(&before);

	cpu = get_cpu();

	spin_lock(&idle_lock);
	cpu_core |= (1 << cpu);

	if ((cpu_core == 0x3) || (cpu_online(1) == 0)) {
		old_div = __raw_readl(S5P_CLKDIV_CPU);
		tmp = old_div;
		tmp |= ((0x7 << 28) | (0x7 << 0));
		__raw_writel(tmp, S5P_CLKDIV_CPU);

		do {
			tmp = __raw_readl(S5P_CLKDIV_STATCPU);
		} while (tmp & 0x10000001);

	}

	spin_unlock(&idle_lock);

	cpu_do_idle();

	spin_lock(&idle_lock);

	if ((cpu_core == 0x3) || (cpu_online(1) == 0)) {
		__raw_writel(old_div, S5P_CLKDIV_CPU);

		do {
			tmp = __raw_readl(S5P_CLKDIV_STATCPU);
		} while (tmp & 0x10000001);

	}

	cpu_core &= ~(1 << cpu);
	spin_unlock(&idle_lock);

	put_cpu();

	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
		    (after.tv_usec - before.tv_usec);

	return idle_time;
}

static int exynos4_lcd_check(unsigned int lcd_number)
{
	unsigned int tmp;
	unsigned int ret;

	if (lcd_number == 0) {
		tmp = __raw_readl(S5P_CLKSRC_MASK_LCD0);
		tmp &= 0x1;
		if (tmp)
			ret = 1;
		else
			ret = 0;
	} else {
		tmp = __raw_readl(S5P_CLKSRC_MASK_LCD1);
		tmp &= 0x1;
		if (tmp)
			ret = 1;
		else
			ret = 0;
	}

	return ret;
}

static int exynos4_check_entermode(void)
{
	unsigned int ret;

	if (exynos4_lcd_check(0) || check_i2c_op())
		ret = S5P_CHECK_DIDLE;
	else
		ret = S5P_CHECK_LPA;

	return ret;
}

static int exynos4_enter_lowpower(struct cpuidle_device *dev,
				  struct cpuidle_state *state)
{
	struct cpuidle_state *new_state = state;
	unsigned int enter_mode;

	/* This mode only can be entered when Core1 is offline */
	if (cpu_online(1)) {
		BUG_ON(!dev->safe_state);
		new_state = dev->safe_state;
	}
	dev->last_state = new_state;

	if (new_state == &dev->states[0]) {
		return exynos4_enter_idle(dev, new_state);
	} else {
		enter_mode = exynos4_check_entermode();
		if (enter_mode == S5P_CHECK_DIDLE)
			return exynos4_enter_core0_aftr(dev, new_state);
		else
			return exynos4_enter_core0_lpa(dev, new_state);
	}
	return exynos4_enter_idle(dev, new_state);
}

static int exynos4_cpuidle_notifier_event(struct notifier_block *this,
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

static struct notifier_block exynos4_cpuidle_notifier = {
	.notifier_call = exynos4_cpuidle_notifier_event,
};

static int __init exynos4_init_cpuidle(void)
{
	int i, max_cpuidle_state, cpu_id;
	struct cpuidle_device *device;
	struct platform_device *pdev;
	struct resource *res;

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

	for (i = 0; i < ARRAY_SIZE(chk_dev_op); i++) {

		pdev = chk_dev_op[i].pdev;

		if (pdev == NULL)
			break;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			printk(KERN_ERR "failed to get iomem region\n");
			return -EINVAL;
		}
		chk_dev_op[i].base = ioremap(res->start, 4096);

		if (!chk_dev_op[i].base) {
			printk(KERN_ERR "failed to map io region\n");
			return -EINVAL;
		}
	}

	register_pm_notifier(&exynos4_cpuidle_notifier);
	return 0;
}
device_initcall(exynos4_init_cpuidle);
