/* linux/arch/arm/mach-exynos/idle-exynos5.S
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS5 AFTR/LPA idle support
 * Based on S3C2410 sleep code by:
 *	Ben Dooks, (c) 2004 Simtec Electronics
 *
 * Based on PXA/SA1100 sleep code by:
 *	Nicolas Pitre, (c) 2002 Monta Vista Software Inc
 *	Cliff Brake, (c) 2001
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/memory.h>
#include <plat/map-base.h>
#include <plat/map-s5p.h>
#include <mach/smc.h>

	.text

	/*
	 * exynos5_enter_lp
	 *
	 * entry:
	 *	r1 = v:p offset
	 */

ENTRY(exynos5_enter_lp)
	stmfd	sp!, { r3 - r12, lr }

	adr	r0, sleep_save_misc

	mrc	p15, 1, r2, c9, c0, 2	@ read l2 control register
	str	r2, [r0], #4
	mrc	p15, 1, r2, c15, c0, 3	@ read l2 prefetch register
	str	r2, [r0], #4

	ldr	r3, =resume_with_mmu
	bl	cpu_suspend

	mov	r0, sp

	mrc	p15, 0, r1, c2, c0, 0	@ TTB 0
	mov	r2, r1, lsr #14		@ get TTB0 base
	mov	r1, r2, lsl #14
	bl	exynos5_flush_cache

	adr	r0, sys_pwr_conf_addr
	ldr	r1, [r0]
	ldr	r2, [r1]
	bic	r2, r2, #(1<<16)
	str	r2, [r1]

	dsb
	wfi

	/* Restore original sp */
	mov	r0, sp
	add	r0, r0, #4
	ldr	sp, [r0]

	mov	r0, #0
	b	early_wakeup

resume_with_mmu:
	adr	r0, sleep_save_misc

	ldr	r1, [r0], #4
	mcr	p15, 1, r1, c9, c0, 2	@ write l2 control register
	ldr	r1, [r0], #4
	mcr	p15, 1, r1, c15, c0, 3	@ write l2 prefetch register

	mov	r0, #1
early_wakeup:

	ldmfd	sp!, { r3 - r12, pc }

	.ltorg

	/*
	 * sleep magic, to allow the bootloader to check for an valid
	 * image to resume to. Must be the first word before the
	 * s3c_cpu_resume entry.
	 */

	.word	0x2bedf00d

sleep_save_misc:
	.long	0
	.long	0

	.global sys_pwr_conf_addr
sys_pwr_conf_addr:
	.long	0

	/*
	 * exynos5_idle_resume
	 *
	 * resume code entry for IROM to call
	 *
	 * we must put this code here in the data segment as we have no
	 * other way of restoring the stack pointer after sleep, and we
	 * must not write to the code segment (code is read-only)
	 */

ENTRY(exynos5_idle_resume)
	/*
	 * To use JTEG after wakeup from power mode
	 * Set DBGEN, NIDEN, SPIDEN, SPNIDEN on TZPC1
	 */
	ldr	r0, =0x10110810
	mov	r1, #0xf
	str	r1, [r0]
	dsb
	isb

	b	cpu_resume
