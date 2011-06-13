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

extern void exynos4_sys_powerdown_conf(enum sys_powerdown mode);
extern int exynos4_enter_lp(unsigned long *saveblk, long);
extern void exynos4_idle_resume(void);

#endif /* __ASM_ARCH_PMU_H */
