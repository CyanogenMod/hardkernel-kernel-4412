/* sound/soc/samsung/audss.c
 *
 * ALSA SoC Audio Layer - Samsung Audio Subsystem driver
 *
 * Copyright (c) 2010 Samsung Electronics Co. Ltd.
 *	Lakkyung Jung <lakkyung.jung@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <plat/audio.h>
#include <mach/map.h>
#include <mach/regs-audss.h>

#include "audss.h"

#define USE_RCLKSRC_BUS

static struct audss_runtime_data {
	struct clk *i2s_rclk;
	struct clk *srp_clk;

	char	*i2s_rclksrc;
	u32	clk_src_rate;
	u32	suspend_audss_clksrc;
	u32	suspend_audss_clkdiv;
	u32	suspend_audss_clkgate;

	bool	clk_enabled;
	bool	reg_saved;
} audss;

static char *i2s_rclksrc =
#ifdef USE_RCLKSRC_BUS
	"busclk";
#else
	"i2sclk";
#endif

/* Lock for cross i/f checks */
static DEFINE_SPINLOCK(lock);

int audss_set_clk_div(u32 mode)
{
	u32 i2s_rclk_rate = 0;
	u32 srp_clk_rate = 0;
	u32 i2s_shift = 0;
	u32 bus_shift = 0;
	u32 srp_shift = 0;
	u32 ret = -1;

	if (!audss.clk_src_rate) {
		pr_err("%s: Can't get current clk_rate %d\n",
			__func__, audss.clk_src_rate);
		return ret;
	}

	pr_debug("%s: Current src clock %d\n", __func__, audss.clk_src_rate);

	switch (mode) {
	case AUDSS_ACTIVE:
		switch (audss.clk_src_rate) {
		case 192000000:
		case 180000000:
			i2s_shift = 2;
			bus_shift = 1;
			srp_shift = 1;
			break;
		case 73728000:
		case 67737600:
			i2s_shift = 1;
			bus_shift = 1;
			srp_shift = 1;
			break;
		case 49152000:
		case 45158000:
			i2s_shift = 0;
			bus_shift = 0;
			srp_shift = 0;
			break;
		}
		break;
	case AUDSS_INACTIVE:
		switch (audss.clk_src_rate) {
		case 192000000:
		case 180000000:
			i2s_shift = 4;
			bus_shift = 4;
			srp_shift = 4;
			break;
		case 73728000:
		case 67737600:
			i2s_shift = 2;
			bus_shift = 2;
			srp_shift = 2;
			break;
		case 49152000:
		case 45158000:
			i2s_shift = 1;
			bus_shift = 1;
			srp_shift = 1;
			break;
		}
		break;
	}

	pr_debug("%s: i2s shift [%d], bus shift[%d], srp shift[%d]\n",
		__func__, i2s_shift, bus_shift, srp_shift);

	if (!strcmp(audss.i2s_rclksrc, "busclk"))
		i2s_rclk_rate = audss.clk_src_rate >> bus_shift;
	else
		i2s_rclk_rate = audss.clk_src_rate >> i2s_shift;

	if (i2s_rclk_rate != clk_get_rate(audss.i2s_rclk)) {
		clk_set_rate(audss.i2s_rclk, i2s_rclk_rate);
		pr_debug("%s: I2S_CLK[%ld]\n",
			__func__, clk_get_rate(audss.i2s_rclk));
	}

	if (audss.srp_clk) {
		srp_clk_rate = audss.clk_src_rate >> srp_shift;
		if (srp_clk_rate != clk_get_rate(audss.srp_clk)) {
			clk_set_rate(audss.srp_clk, srp_clk_rate);
			pr_debug("%s: SRP_CLK[%ld]\n",
				__func__, clk_get_rate(audss.srp_clk));
		}

	}

	pr_debug("%s: AUDSS DIV REG[0x%x]\n",
		__func__, readl(S5P_CLKDIV_AUDSS));

	return 0;
}

void audss_reg_save_restore(int cmd)
{
	if (!cmd) {
		if (audss.reg_saved)
			goto exit_func;

		audss.suspend_audss_clksrc = readl(S5P_CLKSRC_AUDSS);
		audss.suspend_audss_clkdiv = readl(S5P_CLKDIV_AUDSS);
		audss.suspend_audss_clkgate = readl(S5P_CLKGATE_AUDSS);
		audss.reg_saved = true;
	} else {
		if (!audss.reg_saved)
			goto exit_func;

		writel(audss.suspend_audss_clksrc, S5P_CLKSRC_AUDSS);
		writel(audss.suspend_audss_clkdiv, S5P_CLKDIV_AUDSS);
		writel(audss.suspend_audss_clkgate, S5P_CLKGATE_AUDSS);
		audss.reg_saved = false;
	}

exit_func:
	return;
}

void audss_clk_enable(bool enable)
{
	unsigned long flags;

	pr_debug("%s: state %d\n", __func__, enable);
	spin_lock_irqsave(&lock, flags);

	if (enable) {
		if (audss.clk_enabled) {
			pr_debug("%s: Already enabled audss clk %d\n",
					__func__, audss.clk_enabled);
			goto exit_func;
		}

		clk_enable(audss.i2s_rclk);
		if (audss.srp_clk)
			clk_enable(audss.srp_clk);

		audss_reg_save_restore(AUDSS_REG_RESTORE);
		audss_set_clk_div(AUDSS_ACTIVE);
		audss.clk_enabled = true;


		pr_debug("%s: CLKSRC[0x%x], CLKGATE[0x%x]\n", __func__,
				readl(S5P_CLKSRC_AUDSS),
				readl(S5P_CLKGATE_AUDSS));

	} else {
		if (!audss.clk_enabled) {
			pr_debug("%s: Already disabled audss clk %d\n",
					__func__, audss.clk_enabled);
			goto exit_func;
		}

		audss_set_clk_div(AUDSS_INACTIVE);
		audss_reg_save_restore(AUDSS_REG_SAVE);

		clk_disable(audss.i2s_rclk);
		if (audss.srp_clk)
			clk_disable(audss.srp_clk);

		audss.clk_enabled = false;

		pr_debug("%s: CLKSRC[0x%x], CLKGATE[0x%x]\n", __func__,
				readl(S5P_CLKSRC_AUDSS),
				readl(S5P_CLKGATE_AUDSS));
	}

exit_func:
	spin_unlock_irqrestore(&lock, flags);

	return;
}

int audss_init(void)
{
	struct clk *mout_audss;
	int ret = 0;

	audss.i2s_rclksrc = i2s_rclksrc;
	audss.i2s_rclk = NULL;
	audss.srp_clk = NULL;

#ifdef CONFIG_SND_SAMSUNG_RP
	audss.srp_clk = clk_get(NULL, "srp_clk");
	if (IS_ERR(audss.srp_clk)) {
		pr_err("%s:failed to get srp_clk\n", __func__);
		ret = PTR_ERR(audss.srp_clk);
		return ret;
	}
#endif
	mout_audss = clk_get(NULL, "mout_audss");
	if (IS_ERR(mout_audss)) {
		pr_err("%s: failed to get mout audss\n", __func__);
		ret = PTR_ERR(mout_audss);
		goto err1;
	}

	audss.i2s_rclk = clk_get(NULL, audss.i2s_rclksrc);
	if (IS_ERR(audss.i2s_rclk)) {
		pr_err("%s: failed to get i2s root clk\n", __func__);
		ret = PTR_ERR(audss.i2s_rclk);
		goto err2;
	}
	pr_info("%s: i2s rclk source is %s\n", __func__, audss.i2s_rclksrc);

	if (!strcmp(audss.i2s_rclksrc, "busclk"))
		clk_set_parent(audss.i2s_rclk, mout_audss);

	if (audss.srp_clk)
		clk_set_parent(audss.srp_clk, mout_audss);

	audss.clk_src_rate = clk_get_rate(mout_audss);
	ret = audss_set_clk_div(AUDSS_ACTIVE);
	if (ret < 0) {
		pr_err("%s:failed to set clock rate\n", __func__);
		goto err3;
	}

	audss.reg_saved = false;
	audss_clk_enable(true);

	return ret;

err3:
	clk_put(audss.i2s_rclk);
err2:
	clk_put(mout_audss);
err1:
	if (audss.srp_clk)
		clk_put(audss.srp_clk);

	return ret;
}

static __devexit int audss_deinit(void)
{
	clk_put(audss.i2s_rclk);
	audss.i2s_rclk = NULL;

	if (audss.srp_clk) {
		clk_put(audss.srp_clk);
		audss.srp_clk = NULL;
	}

	return 0;
}

static char banner[] __initdata = KERN_INFO "Samsung Audio Subsystem Driver, (c) 2011 Samsung Electronics";

static int __init samsung_audss_init(void)
{
	int ret = 0;

	pr_info("%s\n", banner);

	ret = audss_init();
	if (ret < 0)
		pr_err("%s:failed to init audss clock\n", __func__);

	return ret;

}
module_init(samsung_audss_init);

static void __exit samsung_audss_exit(void)
{
	audss_deinit();
}
module_exit(samsung_audss_exit);

/* Module information */
MODULE_AUTHOR("Lakkyung Jung, <lakkyung.jung@samsung.com>");
MODULE_DESCRIPTION("Samsung Audio subsystem Interface");
MODULE_ALIAS("platform:samsung-audss");
MODULE_LICENSE("GPL");
