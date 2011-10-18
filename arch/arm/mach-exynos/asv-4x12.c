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

#define VA_LID		(S5P_VA_CHIPID + 0x14)
#define NR_SS_CASE	3

unsigned int rev_lid;

unsigned int not_ss_lid[NR_SS_CASE][2] = {
	/* lid, wno */
	{0X00197EA7, 0x6},
	{0X00197EA7, 0x7},
	{0x00197EF9, 0xFFFFFFFF},
};

static int exynos4x12_get_hpm(struct samsung_asv *asv_info)
{
	unsigned int wno;

	wno = (rev_lid >> 6) & 0x1F;

	asv_info->hpm_result = wno;

	return 0;
}

static int exynos4x12_get_ids(struct samsung_asv *asv_info)
{
	unsigned int lid;

	lid = (rev_lid >> 11) & 0x1FFFFF;

	asv_info->ids_result = lid;

	return 0;
}

static int exynos4x12_asv_store_result(struct samsung_asv *asv_info)
{
	unsigned int i;

	for (i = 0; i < NR_SS_CASE; i++) {
		if (asv_info->ids_result == not_ss_lid[i][0]) {
			if (asv_info->hpm_result == not_ss_lid[i][1])
				__raw_writel(0x1, S5P_INFORM2);
			else
				__raw_writel(0x0, S5P_INFORM2);

			return 0;
		}
	}

	__raw_writel(0x1, S5P_INFORM2);

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
