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

#include <mach/regs-irq.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/regs-pmu5.h>
#include <mach/pm-core.h>
#include <mach/pmu.h>

#include <mach/map-exynos5.h>

void __iomem *exynos5_jpeg_base;

static struct sleep_save exynos5_core_save[] = {
	/* GIC side */
	SAVE_ITEM(S5P_VA_GIC_CPU + 0x000),
	SAVE_ITEM(S5P_VA_GIC_CPU + 0x004),
	SAVE_ITEM(S5P_VA_GIC_CPU + 0x008),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x000),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x004),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x100),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x104),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x108),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x300),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x304),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x308),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x400),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x404),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x408),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x40C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x410),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x414),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x418),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x41C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x420),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x424),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x428),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x42C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x430),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x434),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x438),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x43C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x440),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x444),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x448),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x44C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x450),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x454),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x458),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x45C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x460),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x464),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x468),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x46C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x470),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x474),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x478),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x47C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x480),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x484),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x488),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x48C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x490),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x494),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x498),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x49C),

	SAVE_ITEM(S5P_VA_GIC_DIST + 0x800),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x804),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x808),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x80C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x810),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x814),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x818),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x81C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x820),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x824),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x828),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x82C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x830),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x834),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x838),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x83C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x840),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x844),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x848),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x84C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x850),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x854),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x858),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x85C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x860),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x864),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x868),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x86C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x870),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x874),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x878),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x87C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x880),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x884),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x888),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x88C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x890),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x894),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x898),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0x89C),

	SAVE_ITEM(S5P_VA_GIC_DIST + 0xC00),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0xC04),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0xC08),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0xC0C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0xC10),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0xC14),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0xC18),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0xC1C),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0xC20),
	SAVE_ITEM(S5P_VA_GIC_DIST + 0xC24),

	SAVE_ITEM(S5P_VA_COMBINER_BASE + 0x000),
	SAVE_ITEM(S5P_VA_COMBINER_BASE + 0x010),
	SAVE_ITEM(S5P_VA_COMBINER_BASE + 0x020),
	SAVE_ITEM(S5P_VA_COMBINER_BASE + 0x030),
};

/* This function copy from linux/arch/arm/kernel/smp_scu.c */
void exynos5_scu_enable(void __iomem *scu_base)
{
	u32 scu_ctrl;

	scu_ctrl = __raw_readl(scu_base);
	/* already enabled? */
	if (scu_ctrl & 1)
		return;

	scu_ctrl |= 1;
	__raw_writel(scu_ctrl, scu_base);

	/*
	 * Ensure that the data accessed by CPU0 before the SCU was
	 * initialised is visible to the other CPUs.
	 */
	flush_cache_all();
}

void exynos5_cpu_suspend(void)
{
	unsigned int tmp;

	/*
	 * There is some issue on JPEG Device.
	 * Before enter sleep mode, Jpeg be reset.
	 */
	tmp = __raw_readl(EXYNOS5_CLKGATE_IP_GEN);
	tmp |= (1 << 2);
	__raw_writel(tmp, EXYNOS5_CLKGATE_IP_GEN);

	__raw_writel(0x10000000, exynos5_jpeg_base);
	__raw_writel(0x30000000, exynos5_jpeg_base);

	tmp = __raw_readl(EXYNOS5_CLKGATE_IP_GEN);
	tmp &= ~(1 << 2);
	__raw_writel(tmp, EXYNOS5_CLKGATE_IP_GEN);

	/*
	 * GPS LPI mask.
	 */
	__raw_writel(0x10000, EXYNOS5_GPS_LPI);

	/* issue the standby signal into the pm unit. */
	cpu_do_idle();
}

void __iomem *list_both_cnt_feed[] = {
	EXYNOS5_ARM_CORE0_OPTION,
	EXYNOS5_ARM_CORE1_OPTION,
	EXYNOS5_ARM_COMMON_OPTION,
	EXYNOS5_GSCL_OPTION,
	EXYNOS5_ISP_OPTION,
	EXYNOS5_MFC_OPTION,
	EXYNOS5_G3D_OPTION,
	EXYNOS5_DISP1_OPTION,
	EXYNOS5_MAU_OPTION,
	EXYNOS5_GPS_OPTION,
	EXYNOS5_TOP_PWR_OPTION,
	EXYNOS5_TOP_PWR_SYSMEM_OPTION,
};

void __iomem *list_diable_wfi_wfe[] = {
	EXYNOS5_ARM_CORE1_OPTION,
	EXYNOS5_FSYS_ARM_OPTION,
	EXYNOS5_ISP_ARM_OPTION,
};

static void exynos5_init_pmu(void)
{
	unsigned int i;
	unsigned int tmp;

	for (i = 0 ; i < ARRAY_SIZE(list_both_cnt_feed) ; i++) {
		tmp = __raw_readl(list_both_cnt_feed[i]);
		tmp |= (EXYNOS5_USE_SC_FEEDBACK |
			EXYNOS5_USE_SC_COUNTER);
		__raw_writel(tmp, list_both_cnt_feed[i]);
	}

	/*
	 * SKIP_DEACTIVATE_ACEACP_IN_PWDN_BITFIELD Enable
	 * MANUAL_L2RSTDISABLE_CONTROL_BITFIELD Enable
	 */
	tmp = __raw_readl(EXYNOS5_ARM_COMMON_OPTION);
	tmp |= (EXYNOS5_MANUAL_L2RSTDISABLE_CONTROL |
		EXYNOS5_SKIP_DEACTIVATE_ACEACP_IN_PWDN);
	__raw_writel(tmp, EXYNOS5_ARM_COMMON_OPTION);

	for (i = 0 ; i < ARRAY_SIZE(list_diable_wfi_wfe) ; i++) {
		tmp = __raw_readl(list_diable_wfi_wfe[i]);
		tmp &= ~(EXYNOS5_OPTION_USE_STANDBYWFE |
			 EXYNOS5_OPTION_USE_STANDBYWFI);
		__raw_writel(tmp, list_diable_wfi_wfe[i]);
	}
}

static void exynos5_pm_prepare(void)
{
	exynos5_init_pmu();

	s3c_pm_do_save(exynos5_core_save, ARRAY_SIZE(exynos5_core_save));

	/* Set value of power down register for sleep mode */
	exynos5_sys_powerdown_conf(SYS_SLEEP);

	__raw_writel(S5P_CHECK_SLEEP, EXYNOS5_INFORM1);

	/* ensure at least INFORM0 has the resume address */
	__raw_writel(virt_to_phys(s3c_cpu_resume), EXYNOS5_INFORM0);
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
	unsigned long tmp;

	tmp = __raw_readl(EXYNOS5_CENTRAL_SEQ_CONFIGURATION);
	tmp &= ~(EXYNOS5_CENTRAL_LOWPWR_CFG);
	__raw_writel(tmp, EXYNOS5_CENTRAL_SEQ_CONFIGURATION);

	exynos4_reset_assert_ctrl(0);

	tmp = __raw_readl(EXYNOS5_CENTRAL_SEQ_OPTION);

	tmp = (EXYNOS5_USE_STANDBYWFI_ARM_CORE0 |
		EXYNOS5_USE_STANDBYWFE_ARM_CORE0);

	__raw_writel(tmp, EXYNOS5_CENTRAL_SEQ_OPTION);

	return 0;
}

static void exynos5_pm_resume(void)
{
	unsigned long tmp;

	/* If PMU failed while entering sleep mode, WFI will be
	 * ignored by PMU and then exiting cpu_do_idle().
	 * EXYNOS5_CENTRAL_SEQ_CONFIGURATION bit will not be set
	 * automatically in this situation.
	 */
	tmp = __raw_readl(EXYNOS5_CENTRAL_SEQ_CONFIGURATION);

	if (!(tmp & EXYNOS5_CENTRAL_LOWPWR_CFG)) {
		tmp |= EXYNOS5_CENTRAL_LOWPWR_CFG;
		__raw_writel(tmp, EXYNOS5_CENTRAL_SEQ_CONFIGURATION);
		/* No need to perform below restore code */
		goto early_wakeup;
	}

	exynos4_reset_assert_ctrl(1);

	s3c_pm_do_restore_core(exynos5_core_save, ARRAY_SIZE(exynos5_core_save));

	exynos5_scu_enable(S5P_VA_SCU);

early_wakeup:
	__raw_writel(0x0, EXYNOS5_INFORM1);
}

static struct syscore_ops exynos5_pm_syscore_ops = {
	.suspend	= exynos5_pm_suspend,
	.resume		= exynos5_pm_resume,
};

static __init int exynos5_pm_syscore_init(void)
{
	exynos5_jpeg_base = ioremap(EXYNOS4_PA_JPEG, SZ_16K);

	if (exynos5_jpeg_base)
		pr_err("EXYNOS5 PMU :exynos5_jpeg_base get fail\n");

	register_syscore_ops(&exynos5_pm_syscore_ops);

	return 0;
}
arch_initcall(exynos5_pm_syscore_init);
