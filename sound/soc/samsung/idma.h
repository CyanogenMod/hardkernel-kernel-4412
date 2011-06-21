/*
 * idma.h  --  I2S0's Internal Dma driver
 *
 * Copyright (c) 2010 Samsung Electronics Co. Ltd
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __S3C_IDMA_H_
#define __S3C_IDMA_H_

#define I2SAHB		        0x20
#define I2SSTR0			0x24
#define I2SSIZE			0x28
#define I2STRNCNT		0x2c
#define I2SLVL0ADDR		0x30
#define I2SLVL1ADDR		0x34
#define I2SLVL2ADDR		0x38
#define I2SLVL3ADDR		0x3c
#define I2SSTR1			0x40

#define AHB_INTENLVL0		(1 << 24)
#define AHB_LVL0INT		(1 << 20)
#define AHB_CLRLVL0INT		(1 << 16)
#define AHB_DMARLD		(1 << 5)
#define AHB_INTMASK		(1 << 3)
#define AHB_DMAEN		(1 << 0)
#define AHB_LVLINTMASK		(0xf << 20)

#define I2SSIZE_TRNMSK		(0xffff)
#define I2SSIZE_SHIFT		(16)

#ifdef CONFIG_ARCH_EXYNOS4
#define LP_TXBUFF_ADDR		(0x02030000)
#else
#define LP_TXBUFF_ADDR		(0xC0000000)
#endif

/* idma_state */
#define LPAM_DMA_STOP		0
#define LPAM_DMA_START		1

extern struct snd_soc_platform_driver asoc_idma_platform;
extern void idma_init(void *regs);
#endif /* __S3C_IDMA_H_ */
