/* linux/arch/arm/mach-exynos/include/mach/ppmu.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS4 - PPMU support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_PPMU_H
#define __ASM_ARCH_PPMU_H __FILE__

#define NUMBER_OF_COUNTER	4

#define PPMU_CNTENS	0x10
#define PPMU_CNTENC	0x20
#define PPMU_INTENS	0x30
#define PPMU_INTENC	0x40
#define PPMU_FLAG	0x50

#define PPMU_CCNT		0x100
#define PPMU_PMCNT0		0x110
#define PPMU_PMCNT_OFFSET	0x10

#define PPMU_BEVT0SEL		0x1000
#define PPMU_BEVTSEL_OFFSET	0x100
#define PPMU_CNT_RESET		0x1800

#define DEVT0_SEL	0x1000
#define DEVT0_ID	0x1010
#define DEVT0_IDMSK	0x1014
#define DEVT_ID_OFFSET	0x100

#define DEFAULT_WEIGHT	1

#define MAX_CCNT	100

/* For flags */
#define VIDEO_DOMAIN	0x00000001
#define AUDIO_DOMAIN	0x00000002
#define ALL_DOMAIN	0xffffffff

/* For event */
#define RD_BUSY_CYCLE		0x00000000
#define WR_BUSY_CYCLE		0x00000001
#define RW_BUSY_CYCLS		0x00000002
#define RD_REQUEST_COUNT	0x00000003
#define WR_REQUEST_COUNT	0x00000004
#define RD_DATA_COUNT		0x00000005
#define WR_DATA_COUNT		0x00000006

#define PMCNT_OFFSET(i)		(PPMU_PMCNT0 + (PPMU_PMCNT_OFFSET * i))

enum ppmu_counter {
	PPMU_PMNCNT0,
	PPMU_PMCCNT1,
	PPMU_PMNCNT2,
	PPMU_PMNCNT3,
	PPMU_PMNCNT_MAX,
};

enum ppmu_ch {
	DMC0,
	DMC1,
};

enum exynos4_ppmu {
	PPMU_DMC0,
	PPMU_DMC1,
	PPMU_CPU,
	PPMU_ACP,
	PPMU_RIGHT,
	PPMU_LEFT,
	PPMU_CAMIF,
	PPMU_LCD0,
	PPMU_ISPX,
	PPMU_ISPMX,
	PPMU_FSYS,
	PPMU_IMAGE,
	PPMU_TV,
	PPMU_3D,
	PPMU_MFC_L,
	PPMU_MFC_R,
	PPMU_END,
};

extern unsigned long long ppmu_load[PPMU_MFC_R];

struct exynos4_ppmu_hw {
	struct list_head node;
	void __iomem *hw_base;
	unsigned int ccnt;
	unsigned int event[NUMBER_OF_COUNTER];
	unsigned int weight;
	int usage;
	int id;
	unsigned int flags;
	struct device *dev;
	unsigned int count[NUMBER_OF_COUNTER];
};

void exynos4_ppc_reset(struct exynos4_ppmu_hw *ppmu);
void exynos4_ppc_start(struct exynos4_ppmu_hw *ppmu);
void exynos4_ppc_stop(struct exynos4_ppmu_hw *ppmu);
void exynos4_ppc_setevent(struct exynos4_ppmu_hw *ppmu,
				  unsigned int evt_num);
unsigned long long exynos4_ppc_update(struct exynos4_ppmu_hw *ppmu);

void exynos4_ppmu_reset(struct exynos4_ppmu_hw *ppmu);
void exynos4_ppmu_start(struct exynos4_ppmu_hw *ppmu);
void exynos4_ppmu_stop(struct exynos4_ppmu_hw *ppmu);
void exynos4_ppmu_setevent(struct exynos4_ppmu_hw *ppmu,
				   unsigned int evt_num);
unsigned long long exynos4_ppmu_update(struct exynos4_ppmu_hw *ppmu);

unsigned long long ppmu_update(struct device *dev);
void ppmu_start(struct device *dev);
void ppmu_init(struct exynos4_ppmu_hw *ppmu, struct device *dev);

void ppmu_all_start(struct device *dev);
void ppmu_all_update(unsigned int flag);
extern struct exynos4_ppmu_hw exynos_ppmu[];
#endif /* __ASM_ARCH_PPMU_H */

