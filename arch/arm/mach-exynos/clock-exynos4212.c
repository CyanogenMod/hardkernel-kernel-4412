/*
 * linux/arch/arm/mach-exynos/clock-exynos4212.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4212 - Clock support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/syscore_ops.h>

#include <plat/cpu-freq.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/pll.h>
#include <plat/s5p-clock.h>
#include <plat/clock-clksrc.h>
#include <plat/exynos4.h>
#include <plat/pm.h>

#include <mach/hardware.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/exynos-clock.h>

#ifdef CONFIG_PM
static struct sleep_save exynos4212_clock_save[] = {
	/* CMU side */
	SAVE_ITEM(S5P_CLKSRC_CAM1),
	SAVE_ITEM(S5P_CLKSRC_ISP),
	SAVE_ITEM(S5P_CLKDIV_CAM1),
	SAVE_ITEM(S5P_CLKDIV_ISP),
	SAVE_ITEM(S5P_CLKDIV_IMAGE),
	SAVE_ITEM(S5P_CLKSRC_MASK_ISP),
	SAVE_ITEM(S5P_CLKGATE_IP_ISP),
	SAVE_ITEM(S5P_CLKGATE_IP_DMC1),
	SAVE_ITEM(S5P_CLKGATE_IP_IMAGE_4212),
	SAVE_ITEM(S5P_CLKGATE_IP_PERIR_4212),
};

static struct sleep_save exynos4212_epll_save[] = {
	SAVE_ITEM(S5P_EPLL_CON2),
};

static struct sleep_save exynos4212_vpll_save[] = {
	SAVE_ITEM(S5P_VPLL_CON2),
};
#endif

static int __maybe_unused exynos4_clk_bus_peril_ctrl(struct clk *clk, int enable)
{
	return s5p_gatectrl(S5P_CLKGATE_BUS_PERIL, clk, enable);
}

static int __maybe_unused exynos4_clk_bus_perir_ctrl(struct clk *clk, int enable)
{
	return s5p_gatectrl(S5P_CLKGATE_BUS_PERIR, clk, enable);
}

static struct clk *clk_src_mpll_user_list[] = {
	[0] = &clk_fin_mpll,
	[1] = &clk_mout_mpll.clk,
};

static struct clksrc_sources clk_src_mpll_user = {
	.sources	= clk_src_mpll_user_list,
	.nr_sources	= ARRAY_SIZE(clk_src_mpll_user_list),
};

static struct clksrc_clk clk_mout_mpll_user = {
	.clk	= {
		.name		= "mout_mpll_user",
		.id		= -1,
	},
	.sources = &clk_src_mpll_user,
	.reg_src = { .reg = S5P_CLKSRC_CPU, .shift = 24, .size = 1 },
};

static struct clk *clkset_aclk_lrbus_user_list[] = {
	[0] = &clk_fin_mpll,
	[1] = &clk_mout_mpll.clk,
};

static struct clksrc_sources clkset_aclk_lrbus_user = {
	.sources	= clkset_aclk_lrbus_user_list,
	.nr_sources	= ARRAY_SIZE(clkset_aclk_lrbus_user_list),
};

static struct clksrc_clk clk_aclk_gdl_user = {
	.clk	= {
		.name		= "aclk_gdl_user",
		.id		= -1,
	},
	.sources = &clkset_aclk_lrbus_user,
	.reg_src = { .reg = S5P_CLKSRC_LEFTBUS, .shift = 4, .size = 1 },
};

static struct clksrc_clk clk_aclk_gdr_user = {
	.clk	= {
		.name		= "aclk_gdr_user",
		.id		= -1,
	},
	.sources = &clkset_aclk_lrbus_user,
	.reg_src = { .reg = S5P_CLKSRC_RIGHTBUS, .shift = 4, .size = 1 },
};

static struct clksrc_clk clk_mout_aclk_400 = {
	.clk	= {
		.name		= "mout_aclk_400",
		.id		= -1,
	},
	.sources = &clkset_aclk,
	.reg_src = { .reg = S5P_CLKSRC_TOP1, .shift = 8, .size = 1 },
	.reg_div = { .reg = S5P_CLKDIV_TOP, .shift = 24, .size = 3 },
};

static struct clksrc_clk clk_mout_aclk_266 = {
	.clk	= {
		.name		= "mout_aclk_266",
		.id		= -1,
	},
	.sources = &clkset_aclk,
	.reg_src = { .reg = S5P_CLKSRC_TOP1, .shift = 4, .size = 1 },
	.reg_div = { .reg = S5P_CLKDIV_TOP, .shift = 20, .size = 3 },
};

static struct clksrc_clk clk_mout_aclk_200 = {
	.clk	= {
		.name		= "mout_aclk_200",
		.id		= -1,
	},
	.sources = &clkset_aclk,
	.reg_src = { .reg = S5P_CLKSRC_TOP0, .shift = 12, .size = 1 },
	.reg_div = { .reg = S5P_CLKDIV_TOP, .shift = 0, .size = 3 },
};

static struct clk *clk_aclk_400_list[] = {
	[0] = &clk_fin_mpll,
	[1] = &clk_mout_aclk_400.clk,
};

static struct clksrc_sources clkset_aclk_400 = {
	.sources	= clk_aclk_400_list,
	.nr_sources	= ARRAY_SIZE(clk_aclk_400_list),
};

struct clksrc_clk clk_aclk_400 = {
	.clk	= {
		.name		= "aclk_400",
		.id		= -1,
	},
	.sources = &clkset_aclk_400,
	.reg_src = { .reg = S5P_CLKSRC_TOP1, .shift = 24, .size = 1 },
};

static struct clk *clk_aclk_266_list[] = {
	[0] = &clk_fin_mpll,
	[1] = &clk_mout_aclk_266.clk,
};

static struct clksrc_sources clkset_aclk_266 = {
	.sources	= clk_aclk_266_list,
	.nr_sources	= ARRAY_SIZE(clk_aclk_266_list),
};

struct clksrc_clk clk_aclk_266 = {
	.clk	= {
		.name		= "aclk_266",
		.id		= -1,
	},
	.sources = &clkset_aclk_266,
	.reg_src = { .reg = S5P_CLKSRC_TOP1, .shift = 16, .size = 1 },
};

static struct clk *clk_aclk_200_list[] = {
	[0] = &clk_fin_mpll,
	[1] = &clk_mout_aclk_200.clk,
};

static struct clksrc_sources clkset_aclk_200 = {
	.sources	= clk_aclk_200_list,
	.nr_sources	= ARRAY_SIZE(clk_aclk_200_list),
};

static struct clksrc_clk *sysclks[] = {
	&clk_mout_mpll_user,
	&clk_aclk_gdl_user,
	&clk_aclk_gdr_user,
	&clk_mout_aclk_400,
	&clk_mout_aclk_266,
	&clk_mout_aclk_200,
	&clk_aclk_400,
	&clk_aclk_266,
};

static struct clk init_clocks_off[] = {
	{
		.name		= "mipihsi",
		.id		= -1,
		.parent		= &clk_aclk_133.clk,
		.enable		= exynos4_clk_ip_fsys_ctrl,
		.ctrlbit	= (1 << 10),
	},
};

static struct clksrc_clk clksrcs[] = {
	{
		.clk	= {
			.name		= "sclk_mipihsi",
			.id		= -1,
			.enable		= exynos4_clksrc_mask_fsys_ctrl,
			.ctrlbit	= (1 << 24),
		},
		.sources = &clkset_mout_corebus,
		.reg_src = { .reg = S5P_CLKSRC_FSYS, .shift = 24, .size = 1 },
		.reg_div = { .reg = S5P_CLKDIV_FSYS0, .shift = 20, .size = 4 },
	},
};

static struct clk clk_isp[] = {
	{
		.name	= "aclk_400_muxed",
		.id	= -1,
		.parent = &clk_aclk_400.clk,
	}, {
		.name	= "aclk_200_muxed",
		.id	= -1,
		.parent	= &clk_aclk_200.clk,
	},
};

static struct clksrc_clk clk_isp_srcs_div0 = {
	.clk		= {
		.name		= "sclk_mcuisp_div0",
		.id		= -1,
		.parent = &clk_aclk_400.clk,
	},
	.reg_div = { .reg = S5P_CLKDIV_ISP1, .shift = 4, .size = 3 },
};

static struct clksrc_clk clk_isp_srcs[] = {
	{
		.clk		= {
			.name		= "sclk_mcuisp_div1",
			.id		= -1,
			.parent = &clk_isp_srcs_div0.clk,
		},
		.reg_div = { .reg = S5P_CLKDIV_ISP1, .shift = 0, .size = 3 },
	}, {
		.clk		= {
			.name		= "sclk_aclk_div0",
			.id		= -1,
			.parent = &clk_aclk_200.clk,
		},
		.reg_div = { .reg = S5P_CLKDIV_ISP0, .shift = 0, .size = 3 },
	}, {
		.clk		= {
			.name		= "sclk_aclk_div1",
			.id		= -1,
			.parent = &clk_aclk_200.clk,
		},
		.reg_div = { .reg = S5P_CLKDIV_ISP0, .shift = 4, .size = 3 },
	}, {
		.clk		= {
			.name		= "sclk_uart_isp",
			.id		= -1,
			.enable		= exynos4_clksrc_gate_isp_ctrl,
			.ctrlbit	= (1 << 3),
		},
		.sources = &clkset_group,
		.reg_src = { .reg = S5P_CLKSRC_ISP, .shift = 12, .size = 4 },
		.reg_div = { .reg = S5P_CLKDIV_ISP, .shift = 28, .size = 4 },
	},
};

#ifdef CONFIG_PM
static int exynos4212_clock_suspend(void)
{
	s3c_pm_do_save(exynos4212_clock_save, ARRAY_SIZE(exynos4212_clock_save));
	s3c_pm_do_save(exynos4212_vpll_save, ARRAY_SIZE(exynos4212_vpll_save));
	s3c_pm_do_save(exynos4212_epll_save, ARRAY_SIZE(exynos4212_epll_save));

	return 0;
}

static void exynos4212_clock_resume(void)
{
	s3c_pm_do_restore_core(exynos4212_clock_save, ARRAY_SIZE(exynos4212_clock_save));
	s3c_pm_do_restore_core(exynos4212_vpll_save, ARRAY_SIZE(exynos4212_vpll_save));
	s3c_pm_do_restore_core(exynos4212_epll_save, ARRAY_SIZE(exynos4212_epll_save));
}
#else
#define exynos4212_clock_suspend NULL
#define exynos4212_clock_resume NULL
#endif

struct syscore_ops exynos4212_clock_syscore_ops = {
	.suspend        = exynos4212_clock_suspend,
	.resume         = exynos4212_clock_resume,
};

void __init exynos4212_register_clocks(void)
{
	int ptr;

	/* usbphy1 is removed in exynos 4212 */
	clkset_group_list[4] = NULL;

	/* mout_mpll_user is used instead of mout_mpll in exynos 4212 */
	clkset_group_list[6] = &clk_mout_mpll_user.clk;
	clkset_aclk_top_list[0] = &clk_mout_mpll_user.clk;

	clk_mout_mpll.reg_src.reg = S5P_CLKSRC_DMC;
	clk_mout_mpll.reg_src.shift = 12;
	clk_mout_mpll.reg_src.size = 1;

	clk_aclk_200.sources = &clkset_aclk_200;
	clk_aclk_200.reg_src.reg = S5P_CLKSRC_TOP1;
	clk_aclk_200.reg_src.shift = 20;
	clk_aclk_200.reg_src.size = 1;

	for (ptr = 0; ptr < ARRAY_SIZE(sysclks); ptr++)
		s3c_register_clksrc(sysclks[ptr], 1);

	s3c_register_clksrc(clksrcs, ARRAY_SIZE(clksrcs));

	s3c_register_clocks(init_clocks_off, ARRAY_SIZE(init_clocks_off));
	s3c_disable_clocks(init_clocks_off, ARRAY_SIZE(init_clocks_off));

	s3c_register_clksrc(&clk_isp_srcs_div0, 1);
	s3c_register_clksrc(clk_isp_srcs, ARRAY_SIZE(clk_isp_srcs));
	s3c_register_clocks(clk_isp, ARRAY_SIZE(clk_isp));

	register_syscore_ops(&exynos4212_clock_syscore_ops);
}
