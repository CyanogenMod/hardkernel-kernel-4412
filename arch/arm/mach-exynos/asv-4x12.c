/* linux/arch/arm/mach-exynos/asv-4x12.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS4X12 - ASV(Adaptive Supply Voltage) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/asv.h>
#include <mach/map.h>
#include <mach/regs-pmu.h>

#define SS_LID		0X197EA7
#define VA_LID		(S5P_VA_CHIPID + 0x14)

unsigned int rev_lid;

static int exynos4x12_get_ids(struct samsung_asv *asv_info)
{
	unsigned int lid;

	lid = (rev_lid >> 11) & 0x1FFFFF;

	if (lid == SS_LID)
		asv_info->ids_result = 0;
	else
		asv_info->ids_result = 1;

	return 0;
}

static int exynos4x12_get_hpm(struct samsung_asv *asv_info)
{
	unsigned int wno;

	wno = (rev_lid >> 6) & 0x1F;

	if ((wno == 6) || (wno == 7))
		asv_info->hpm_result = 1;
	else
		asv_info->hpm_result = 0;

	return 0;
}

static int exynos4x12_asv_store_result(struct samsung_asv *asv_info)
{
	if ((asv_info->hpm_result) || (asv_info->ids_result))
		__raw_writel(0x1, S5P_INFORM2);
	else
		__raw_writel(0x0, S5P_INFORM2);

	return 0;
}

int exynos4x12_asv_init(struct samsung_asv *asv_info)
{
	unsigned int lid_reg;
	unsigned int tmp;
	unsigned int i;

	rev_lid = 0;

	pr_info("EXYNOS4X12: Adaptive Support Voltage init\n");

	lid_reg = __raw_readl(VA_LID);

	for (i = 0; i < 32; i++) {
		tmp = (lid_reg >> i) & 0x1;
		rev_lid += tmp << (31 - i);
	}

	asv_info->get_ids = exynos4x12_get_ids;
	asv_info->get_hpm = exynos4x12_get_hpm;
	asv_info->store_result = exynos4x12_asv_store_result;

	return 0;
}
