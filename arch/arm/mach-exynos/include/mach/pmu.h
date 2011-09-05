/* linux/arch/arm/mach-exynos4/include/mach/pmu.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS4210 - PMU support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_PMU_H
#define __ASM_ARCH_PMU_H __FILE__

enum sys_powerdown {
	SYS_AFTR,
	SYS_LPA,
	SYS_SLEEP,
	NUM_SYS_POWERDOWN,
};

struct exynos4_pmu_conf {
	void __iomem *reg;
	unsigned long val[NUM_SYS_POWERDOWN];
};

enum c2c_pwr_mode {
	MIN_LATENCY,
	SHORT_LATENCY,
	MAX_LATENCY,
};

struct exynos4_c2c_pmu_conf {
	void __iomem *reg;
	unsigned long val;
};

extern void exynos4_sys_powerdown_conf(enum sys_powerdown mode);
extern int exynos4_enter_lp(unsigned long *saveblk, long);
extern void exynos4_idle_resume(void);
extern void exynos4_c2c_request_pwr_mode(enum c2c_pwr_mode mode);
#endif /* __ASM_ARCH_PMU_H */
