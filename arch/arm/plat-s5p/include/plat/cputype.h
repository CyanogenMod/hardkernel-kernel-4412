/* linux/arch/arm/plat-s5p/include/plat/cputype.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5P - SOC type definition
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _ASM_PLAT_CPUTYPE_H
#define _ASM_PLAT_CPUTYPE_H

extern unsigned long cpu_idcode;

#define	S5P_CPU_ID_6440			0x56440100
#define	S5P_CPU_ID_6450			0x36450000
#define	S5P_CPU_ID_C100			0x43100000
#define	S5P_CPU_ID_V210			0x43110000
#define	EXYNOS_CPU_ID_4210		0x43210000
#define	EXYNOS_CPU_ID_4212		0x43220000

#ifdef CONFIG_MACH_FPGA5210
#define	EXYNOS_CPU_ID_5210		0x00000000
#else
#define	EXYNOS_CPU_ID_5210		0x53210000
#endif

#define S5P_CPU_MASK			0xfffff000
#define EXYNOS_CPU_MASK			0xfffe0000

#define IS_EXYNOS_CPU(type, id, mask)			\
static inline int is_exynos ##type(void)		\
{							\
	return ((cpu_idcode & mask) == (id & mask));	\
}

IS_EXYNOS_CPU(4210, EXYNOS_CPU_ID_4210, EXYNOS_CPU_MASK)
IS_EXYNOS_CPU(4212, EXYNOS_CPU_ID_4212, EXYNOS_CPU_MASK)

#ifdef CONFIG_CPU_EXYNOS4210
#define cpu_is_exynos4210() is_exynos4210()
#else
#define cpu_is_exynos4210() 0
#endif

#ifdef CONFIG_CPU_EXYNOS4212
#define cpu_is_exynos4212() is_exynos4212()
#else
#define cpu_is_exynos4212() 0
#endif

#endif /* __ASM_PLAT_CPUTYPE_H */
