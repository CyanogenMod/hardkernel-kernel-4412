/* linux/arch/arm/mach-exynos/include/mach/smc.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS - SMC Call
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_SMC_H
#define __ASM_ARCH_SMC_H __FILE__

#define SMC_CMD_INIT		(-1)
#define SMC_CMD_INFO		(-2)
#define SMC_CMD_SLEEP		(-3)
#define SMC_CMD_CPU1BOOT	(-4)
#define SMC_CMD_L2X0UP		(-5)
#define SMC_CMD_CPU0AFTR	(-6)
#define SMC_CMD_L2X0DEBUG	(-7)
#define SMC_CMD_L2X0CTRL	(-8)
#define SMC_CMD_C15RESUME	(-9)

#ifndef __ASSEMBLY__
static inline u32 exynos_smc(u32 cmd, u32 arg1, u32 arg2, u32 arg3)
{
	register u32 reg0 __asm__("r0") = cmd;
	register u32 reg1 __asm__("r1") = arg1;
	register u32 reg2 __asm__("r2") = arg2;
	register u32 reg3 __asm__("r3") = arg3;

	__asm__ volatile (
		"smc	0\n"
		: "+r"(reg0), "+r"(reg1), "+r"(reg2), "+r"(reg3)
	);

	return reg0;
}
#endif

#endif /* __ASM_ARCH_SMC_H */
