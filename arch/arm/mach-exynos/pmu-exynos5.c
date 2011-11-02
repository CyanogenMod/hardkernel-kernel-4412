/* linux/arch/arm/mach-exynos/pmu-exynos5.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS5 - CPU PMU(Power Management Unit) support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/kernel.h>

#include <mach/regs-clock.h>
#include <mach/pmu.h>
#include <mach/regs-pmu5.h>

#include <plat/cputype.h>

static struct exynos4_pmu_conf *exynos5_pmu_config;

static unsigned int entry_cnt;

static struct exynos4_pmu_conf exynos52xx_pmu_config[] = {
	/* { .reg = address, .val = { AFTR, LPA, SLEEP } */
	{ EXYNOS5_ARM_CORE0_SYS_PWR_REG,		{ 0x0, 0x0, 0x2} },
	{ EXYNOS5_ARM_CORE1_SYS_PWR_REG,		{ 0x0, 0x0, 0x2} },
	{ EXYNOS5_FSYS_ARM_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_ISP_ARM_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_ARM_COMMON_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_ARM_L2_SYS_PWR_REG,			{ 0x0, 0x0, 0x3} },
	{ EXYNOS5_CMU_ACLKSTOP_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_SCLKSTOP_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_CMU_ACLKSTOP_SYSMEM_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_SCLKSTOP_SYSMEM_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_DRAM_FREQ_DOWN_SYS_PWR_REG,		{ 0x1, 0x1, 0x1} },
	{ EXYNOS5_DDRPHY_DLLOFF_SYS_PWR_REG,		{ 0x1, 0x1, 0x1} },
	{ EXYNOS5_DDRPHY_DLLLOCK_SYS_PWR_REG,		{ 0x1, 0x1, 0x1} },
	{ EXYNOS5_APLL_SYSCLK_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_MPLL_SYSCLK_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_VPLL_SYSCLK_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_EPLL_SYSCLK_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_BPLL_SYSCLK_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CPLL_SYSCLK_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_MPLLUSER_SYSCLK_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_BPLLUSER_SYSCLK_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_TOP_BUS_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_TOP_RETENTION_SYS_PWR_REG,		{ 0x0, 0x0, 0x1} },
	{ EXYNOS5_TOP_PWR_SYS_PWR_REG,			{ 0x0, 0x0, 0x3} },
	{ EXYNOS5_TOP_BUS_SYSMEM_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_TOP_RETENTION_SYSMEM_SYS_PWR_REG,	{ 0x0, 0x0, 0x1} },
	{ EXYNOS5_TOP_PWR_SYSMEM_SYS_PWR_REG,		{ 0x0, 0x0, 0x3} },
	{ EXYNOS5_LOGIC_RESET_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_OSCCLK_GATE_SYS_PWR_REG,		{ 0x0, 0x0, 0x1} },
	{ EXYNOS5_LOGIC_RESET_SYSMEM_SYS_PWR_REG,	{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_OSCCLK_GATE_SYSMEM_SYS_PWR_REG,	{ 0x0, 0x0, 0x1} },
	{ EXYNOS5_USBOTG_MEM_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_G2D_MEM_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_USBDRD_MEM_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_SDMMC_MEM_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CSSYS_MEM_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_SECSS_MEM_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_ROTATOR_MEM_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_INTRAM_MEM_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_INTROM_MEM_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_JPEG_MEM_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_HSI_MEM_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_MCUIOP_MEM_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_SATA_MEM_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_DRAM_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_MAU_SYS_PWR_REG,	{ 0x0, 0x1, 0x0} },
	{ EXYNOS5_PAD_RETENTION_GPIO_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_UART_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_MMCA_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_MMCB_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_EBIA_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_EBIB_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_SPI_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_PAD_RETENTION_GPIO_SYSMEM_SYS_PWR_REG,{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_PAD_ISOLATION_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_PAD_ISOLATION_SYSMEM_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_PAD_ALV_SEL_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_XUSBXTI_SYS_PWR_REG,			{ 0x0, 0x1, 0x0} },
	{ EXYNOS5_XXTI_SYS_PWR_REG,			{ 0x0, 0x1, 0x0} },
	{ EXYNOS5_EXT_REGULATOR_SYS_PWR_REG,		{ 0x1, 0x1, 0x0} },
	{ EXYNOS5_GPIO_MODE_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_GPIO_MODE_SYSMEM_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_GPIO_MODE_MAU_SYS_PWR_REG,		{ 0x0, 0x1, 0x0} },
	{ EXYNOS5_TOP_ASB_RESET_SYS_PWR_REG,		{ 0x1, 0x1, 0x1} },
	{ EXYNOS5_TOP_ASB_ISOLATION_SYS_PWR_REG,	{ 0x0, 0x0, 0x1} },
	{ EXYNOS5_GSCL_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_ISP_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_MFC_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_G3D_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_DISP1_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_MAU_SYS_PWR_REG,			{ 0x0, 0x7, 0x0} },
	{ EXYNOS5_GPS_SYS_PWR_REG,			{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_GSCL_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_ISP_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_MFC_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_G3D_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_DISP1_SYS_PWR_REG,	{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_MAU_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_CLKSTOP_GPS_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_GSCL_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_ISP_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_MFC_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_G3D_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_DISP1_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_MAU_SYS_PWR_REG,		{ 0x0, 0x1, 0x0} },
	{ EXYNOS5_CMU_SYSCLK_GPS_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	/*
	 * There are no register definition on Manual for following
	 * I will check and fix it
	 *
	 * { EXYNOS5_CMU_SCLKSTOP_GSCL_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	 * { EXYNOS5_CMU_SCLKSTOP_ISP_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	 * { EXYNOS5_CMU_SCLKSTOP_MFC_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	 * { EXYNOS5_CMU_SCLKSTOP_G3D_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	 * { EXYNOS5_CMU_SCLKSTOP_DISP1_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	 * { EXYNOS5_CMU_SCLKSTOP_MAU_SYS_PWR_REG,		{ 0x0, 0x1, 0x0} },
	 * { EXYNOS5_CMU_SCLKSTOP_GPS_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	 */
	{ EXYNOS5_CMU_RESET_GSCL_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_ISP_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_MFC_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_G3D_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_DISP1_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_MAU_SYS_PWR_REG,		{ 0x1, 0x0, 0x0} },
	{ EXYNOS5_CMU_RESET_GPS_SYS_PWR_REG,		{ 0x0, 0x0, 0x0} },
};

void exynos5_sys_powerdown_conf(enum sys_powerdown mode)
{
	unsigned int count = entry_cnt;

	for (; count > 0; count--)
		__raw_writel(exynos5_pmu_config[count - 1].val[mode],
				exynos5_pmu_config[count - 1].reg);
}

static int __init exynos5_pmu_init(void)
{
	exynos5_pmu_config = exynos52xx_pmu_config;
	entry_cnt = ARRAY_SIZE(exynos52xx_pmu_config);
	printk(KERN_INFO "%s: PMU supports 52XX(%d)\n"
				, __func__, entry_cnt);

	return 0;
}
arch_initcall(exynos5_pmu_init);
