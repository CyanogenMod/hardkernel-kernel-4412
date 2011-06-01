/* linux/arch/arm/mach-exynos4/include/mach/cpufreq.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4 - CPUFreq support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

/* CPU frequency level index for using cpufreq lock API
 * This should be same with cpufreq_frequency_table
*/
enum cpufreq_level_request {
	CPU_L0,		/* 1200MHz */
	CPU_L1,		/* 1000MHz */
	CPU_L2,		/* 800MHz */
	CPU_L3,		/* 500MHz */
	CPU_L4,		/* 200MHz */
	CPU_LEVEL_END,
};

enum busfreq_level_request {
	BUS_L0,		/* MEM 400MHz BUS 200MHz */
	BUS_L1,		/* MEM 267MHz BUS 160MHz */
	BUS_L2,		/* MEM 133MHz BUS 133MHz */
	BUS_LEVEL_END,
};

enum cpufreq_lock_ID {
	DVFS_LOCK_ID_G2D,	/* G2D */
	DVFS_LOCK_ID_TV,	/* TV */
	DVFS_LOCK_ID_MFC,	/* MFC */
	DVFS_LOCK_ID_USB,	/* USB */
	DVFS_LOCK_ID_CAM,	/* CAM */
	DVFS_LOCK_ID_PM,	/* PM */
	DVFS_LOCK_ID_USER,	/* USER */
	DVFS_LOCK_ID_END,
};

int exynos4_cpufreq_lock(unsigned int nId,
			enum cpufreq_level_request cpufreq_level);
void exynos4_cpufreq_lock_free(unsigned int nId);

int exynos4_busfreq_lock(unsigned int nId,
			enum busfreq_level_request busfreq_level);
void exynos4_busfreq_lock_free(unsigned int nId);
