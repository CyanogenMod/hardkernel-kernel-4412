/* linux/arch/arm/mach-exynos4/idle.S
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4210 AFTR/LPA idle support
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
	.text

	/*
	 * exynos4_enter_lp
	 *
	 * entry:
	 *	r1 = v:p offset
	 */

ENTRY(exynos4_enter_lp)
	stmfd	sp!, { r3 - r12, lr }
	ldr	r3, =resume_with_mmu
	bl	cpu_suspend

	mov	r0, sp
	bl	exynos4_lpidle_cache_clean
	bl	exynos4_set_core0_pwroff

	/* Restore original sp */
	mov	r0, sp
	add	r0, r0, #4
	ldr	sp, [r0]

	mov	r0, #0
	b	early_wakeup

resume_with_mmu:

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

	/*
	 * exynos4_idle_resume
	 *
	 * resume code entry for IROM to call
	 *
	 * we must put this code here in the data segment as we have no
	 * other way of restoring the stack pointer after sleep, and we
	 * must not write to the code segment (code is read-only)
	 */

ENTRY(exynos4_idle_resume)
	/* SCU enable */
	ldr	r1, =0x10500000
	ldr	r0, [r1]
	orr	r0, r0, #1
	orr	r0, r0, #(1 << 5)
	str	r0, [r1]

	/* Outer cache TAG,DATA latency */
	ldr	r1, =0x10502000
	mov	r0, #0x110
	str	r0, [r1, #0x108]
	str	r0, [r1, #0x10C]

	/* L2 cache prefetch control register */
	ldr	r0, =0x30000007
	str	r0, [r1, #0xF60]

	/* Power control register setting*/
	mov	r0, #0x3
	str	r0, [r1, #0xF80]

	ldr	r0, [r1, #0x104]
	ldr	r2, =0x7c470001
	ldr	r3, =0xc200ffff

	and	r0, r0, r3
	orr	r0, r0, r2
	str	r0, [r1, #0x104]

	/* Enable L2 cache */
	mov	r0, #1
	str	r0, [r1, #0x100]

	b	cpu_resume
