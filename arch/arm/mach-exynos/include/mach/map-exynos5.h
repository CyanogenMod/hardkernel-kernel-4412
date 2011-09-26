/* linux/arch/arm/mach-exynos4/include/mach/map-exynos5.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS5 - Memory map definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_MAP_EXYNOS5_H
#define __ASM_ARCH_MAP_EXYNOS5_H __FILE__

#define EXYNOS5_PA_CHIPID		0x10000000

#define EXYNOS5_PA_CMU			0x10010000

#define EXYNOS5_PA_PMU			0x10040000

#define EXYNOS5_PA_SYSTIMER		0x101C0000
#define EXYNOS5_PA_WATCHDOG		0x101D0000

#define EXYNOS5_PA_COMBINER		0x10440000

#define EXYNOS5_PA_GIC_CPU		0x10480000
#define EXYNOS5_PA_GIC_DIST		0x10490000

#define EXYNOS5_PA_SYSCON		0x10050100

#define EXYNOS5_PA_HSOTG		0x12480000
#define EXYNOS5_PA_HSPHY		0x125B0000

#define EXYNOS5_PA_SROMC		0x12570000

#define EXYNOS5_PA_UART			0x12C00000

#define EXYNOS5_PA_TIMER		0x12DD0000

#define EXYNOS5_PA_FIMD0		0x13800000
#define EXYNOS5_PA_FIMD1		0x14400000

#define EXYNOS5_PA_SDRAM		0x40000000

#define EXYNOS5_PA_IIC			0xE1800000

/* Compatibiltiy Defines */

#define S3C_PA_IIC			EXYNOS5_PA_IIC
#define S3C_PA_WDT			EXYNOS5_PA_WATCHDOG
#define S5P_PA_CHIPID			EXYNOS5_PA_CHIPID
#define S5P_PA_SYSCON			EXYNOS5_PA_SYSCON
#define S5P_PA_SROMC			EXYNOS5_PA_SROMC
#define S5P_PA_TIMER			EXYNOS5_PA_TIMER
#define S5P_PA_HSOTG			EXYNOS5_PA_HSOTG
#define S5P_PA_HSPHY			EXYNOS5_PA_HSPHY
#define S5P_PA_FIMD0			EXYNOS5_PA_FIMD0
#define S5P_PA_FIMD1			EXYNOS5_PA_FIMD1
#define S5P_PA_SDRAM			EXYNOS5_PA_SDRAM

/* UART */

#define S3C_VA_UARTx(x)			(S3C_VA_UART + ((x) * S3C_UART_OFFSET))

#define S3C_PA_UART			EXYNOS5_PA_UART

#define S5P_PA_UART(x)			(S3C_PA_UART + ((x) * S3C_UART_OFFSET))
#define S5P_PA_UART0			S5P_PA_UART(0)
#define S5P_PA_UART1			S5P_PA_UART(1)
#define S5P_PA_UART2			S5P_PA_UART(2)
#define S5P_PA_UART3			S5P_PA_UART(3)
#define S5P_PA_UART4			S5P_PA_UART(4)

#define S5P_SZ_UART			SZ_256

#endif /* __ASM_ARCH_MAP_EXYNOS5_H */
