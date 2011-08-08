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

static struct audss_runtime_data {
	struct clk *i2s_clk;
	struct clk *bus_clk;
	struct clk *srp_clk;

	u32	clk_src_rate;
	u32	suspend_audss_clksrc;
	u32	suspend_audss_clkdiv;
	u32	suspend_audss_clkgate;

	bool	clk_enabled;
	bool	reg_saved;
} audss;

/* Lock for cross i/f checks */
static DEFINE_SPINLOCK(lock);

int audss_set_clk_div(u32 mode)
{
	u32 i2s_clk_rate = 0;
	u32 bus_clk_rate = 0;
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
			bus_shift = 2;
			srp_shift = 2;
			break;
		case 73728000:
		case 67737600:
			bus_shift = 1;
			srp_shift = 1;
			break;
		case 49152000:
		case 45158000:
			bus_shift = 0;
			srp_shift = 0;
			break;
		}
		break;
	case AUDSS_INACTIVE:
		switch (audss.clk_src_rate) {
		case 192000000:
		case 180000000:
			bus_shift = 4;
			srp_shift = 4;
			break;
		case 73728000:
		case 67737600:
			bus_shift = 2;
			srp_shift = 2;
			break;
		case 49152000:
		case 45158000:
			bus_shift = 1;
			srp_shift = 1;
			break;
		}
		break;
	}

	pr_debug("%s: bus shift[%d], srp shift[%d]\n",
		__func__, bus_shift, srp_shift);

	i2s_clk_rate = audss.clk_src_rate >> i2s_shift;
	bus_clk_rate = audss.clk_src_rate >> bus_shift;

	if (i2s_clk_rate != clk_get_rate(audss.i2s_clk)) {
		clk_set_rate(audss.i2s_clk, i2s_clk_rate);
		pr_debug("%s: I2S_CLK[%ld]\n",
			__func__, clk_get_rate(audss.i2s_clk));
	}

	if (bus_clk_rate != clk_get_rate(audss.bus_clk)) {
		clk_set_rate(audss.bus_clk, bus_clk_rate);
		pr_debug("%s: BUS_CLK[%ld]\n",
			__func__, clk_get_rate(audss.bus_clk));
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
			spin_unlock_irqrestore(&lock, flags);
			return;
		}

		clk_enable(audss.bus_clk);
		clk_enable(audss.i2s_clk);
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
			spin_unlock_irqrestore(&lock, flags);
			return;
		}

		audss_set_clk_div(AUDSS_INACTIVE);
		audss_reg_save_restore(AUDSS_REG_SAVE);

		clk_disable(audss.i2s_clk);
		clk_disable(audss.bus_clk);
		if (audss.srp_clk)
			clk_disable(audss.srp_clk);

		audss.clk_enabled = false;

		pr_debug("%s: CLKSRC[0x%x], CLKGATE[0x%x]\n", __func__,
				readl(S5P_CLKSRC_AUDSS),
				readl(S5P_CLKGATE_AUDSS));
	}

	spin_unlock_irqrestore(&lock, flags);

	return;
}

#ifdef CONFIG_PM
static int
samsung_audss_suspend(struct platform_device *pdev, pm_message_t state)
{
	pr_debug("%s: Entered function\n", __func__);
	return 0;
}

static int samsung_audss_resume(struct platform_device *pdev)
{
	pr_debug("%s: Entered function\n", __func__);
	return 0;
}
#else
#define samsung_audss_suspend NULL
#define samsung_audss_resume  NULL
#endif

static __devinit int samsung_audss_probe(struct platform_device *pdev)
{
	struct clk *mout_audss;
	int ret = 0;

	audss.bus_clk = NULL;
	audss.i2s_clk = NULL;
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

	audss.bus_clk = clk_get(NULL, "busclk");
	if (IS_ERR(audss.bus_clk)) {
		pr_err("%s: failed to get i2s bus_clk\n", __func__);
		ret = PTR_ERR(audss.bus_clk);
		goto err2;
	}

	audss.i2s_clk = clk_get(NULL, "i2sclk");
	if (IS_ERR(audss.bus_clk)) {
		pr_err("%s:failed to get i2s clk\n", __func__);
		ret = PTR_ERR(audss.i2s_clk);
		goto err3;
	}

	clk_set_parent(audss.bus_clk, mout_audss);
	if (audss.srp_clk)
		clk_set_parent(audss.srp_clk, mout_audss);

	audss.clk_src_rate = clk_get_rate(mout_audss);
	ret = audss_set_clk_div(AUDSS_ACTIVE);
	if (ret < 0) {
		pr_err("%s:failed to set clock rate\n", __func__);
		goto err4;
	}

	audss.reg_saved = false;
	audss_clk_enable(true);

	return ret;

err4:
	clk_put(audss.i2s_clk);
err3:
	clk_put(audss.bus_clk);
err2:
	clk_put(mout_audss);
err1:
	if (audss.srp_clk)
		clk_put(audss.srp_clk);

	return ret;
}

static __devexit int samsung_audss_remove(struct platform_device *pdev)
{
	clk_put(audss.bus_clk);
	clk_put(audss.i2s_clk);

	audss.i2s_clk = NULL;
	audss.bus_clk = NULL;

	if (audss.srp_clk) {
		clk_put(audss.srp_clk);
		audss.srp_clk = NULL;
	}
	return 0;
}

static struct platform_driver samsung_audss_driver = {
	.probe  = samsung_audss_probe,
	.remove = samsung_audss_remove,
	.resume = samsung_audss_resume,
	.suspend = samsung_audss_suspend,
	.driver = {
		.name = "samsung-audss",
		.owner = THIS_MODULE,
	},
};

static char banner[] __initdata = KERN_INFO "Samsung Audio Subsystem Driver, (c) 2011 Samsung Electronics";

static int __init samsung_audss_init(void)
{
	pr_info("%s\n", banner);

	return platform_driver_register(&samsung_audss_driver);
}
module_init(samsung_audss_init);

static void __exit samsung_audss_exit(void)
{
	platform_driver_unregister(&samsung_audss_driver);
}
module_exit(samsung_audss_exit);

/* Module information */
MODULE_AUTHOR("Lakkyung Jung, <lakkyung.jung@samsung.com>");
MODULE_DESCRIPTION("Samsung Audio subsystem Interface");
MODULE_ALIAS("platform:samsung-audss");
MODULE_LICENSE("GPL");
