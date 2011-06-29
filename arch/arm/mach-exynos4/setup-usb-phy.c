/*
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/regs-pmu.h>
#include <mach/regs-usb-phy.h>
#include <plat/cpu.h>
#include <plat/usb-phy.h>

#define ETC6PUD		(S5P_VA_GPIO2 + 0x228)

static int exynos4_usb_phy0_init(struct platform_device *pdev)
{
	struct clk *otg_clk;
	struct clk *xusbxti_clk;
	u32 phyclk;
	u32 rstcon;
	int err;

	otg_clk = clk_get(&pdev->dev, "usbotg");
	if (IS_ERR(otg_clk)) {
		printk(KERN_ERR"Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	err = clk_enable(otg_clk);
	if (err) {
		clk_put(otg_clk);
		return err;
	}

	writel(readl(S5P_USBOTG_PHY_CONTROL) | S5P_USBOTG_PHY_ENABLE,
			S5P_USBOTG_PHY_CONTROL);

	/* set to normal of PHY0 */
	writel((readl(EXYNOS4_PHYPWR) & ~PHY0_NORMAL_MASK),
			EXYNOS4_PHYPWR);

	/* set clock frequency for PLL */
	phyclk = readl(EXYNOS4_PHYCLK) &
		~(PHY0_ID_PULLUP | CLKSEL_MASK);

	xusbxti_clk = clk_get(&pdev->dev, "xusbxti");
	if (xusbxti_clk && !IS_ERR(xusbxti_clk)) {
		switch (clk_get_rate(xusbxti_clk)) {
		case 12 * MHZ:
			phyclk |= CLKSEL_12M;
			break;
		case 24 * MHZ:
			phyclk |= CLKSEL_24M;
			break;
		default:
		case 48 * MHZ:
			/* default reference clock */
			break;
		}
		clk_put(xusbxti_clk);
	}

	phyclk |= PHY0_COMMON_ON_N;
	writel(phyclk, EXYNOS4_PHYCLK);

	/* reset all ports of both PHY and Link */
	rstcon = readl(EXYNOS4_RSTCON) | PHY0_SWRST_MASK;
	writel(rstcon, EXYNOS4_RSTCON);
	udelay(10);

	rstcon &= ~PHY0_SWRST_MASK;
	writel(rstcon, EXYNOS4_RSTCON);

	clk_put(otg_clk);
	return 0;
}

static int exynos4_usb_phy0_exit(struct platform_device *pdev)
{
	struct clk *otg_clk;

	otg_clk = clk_get(&pdev->dev, "usbotg");
	if (IS_ERR(otg_clk)) {
		dev_err(&pdev->dev, "Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	/* unset to normal of PHY0 */
	writel((readl(EXYNOS4_PHYPWR) | PHY0_NORMAL_MASK),
			EXYNOS4_PHYPWR);

	writel(readl(S5P_USBOTG_PHY_CONTROL) & ~S5P_USBOTG_PHY_ENABLE,
			S5P_USBOTG_PHY_CONTROL);

	clk_disable(otg_clk);
	clk_put(otg_clk);

	return 0;
}

int is_usb_host_phy_suspend(void)
{
	struct clk *otg_clk;
	unsigned long flags;
	int err;

	otg_clk = clk_get(NULL, "usbotg");
	if (IS_ERR(otg_clk)) {
		printk(KERN_ERR"Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	err = clk_enable(otg_clk);
	if (err) {
		clk_put(otg_clk);
		return err;
	}
	/*If USB Host PHY1 is suspended,  */
	if (readl(EXYNOS4_PHYPWR) & PHY1_STD_FORCE_SUSPEND){
		local_irq_save(flags);
		if (readl(S5P_USBHOST_PHY_CONTROL) & S5P_USBHOST_PHY_ENABLE) {
			writel(readl(EXYNOS4_PHYPWR)
				| PHY1_STD_ANALOG_POWERDOWN, EXYNOS4_PHYPWR);
			writel(readl(S5P_USBHOST_PHY_CONTROL)
				&~(S5P_USBHOST_PHY_ENABLE), S5P_USBHOST_PHY_CONTROL);
		}

		local_irq_restore(flags);
		clk_disable(otg_clk);
		clk_put(otg_clk);
		return 1;
	}

	clk_disable(otg_clk);
	clk_put(otg_clk);

	return 0;
}
EXPORT_SYMBOL(is_usb_host_phy_suspend);

static int exynos4_usb_phy1_suspend(struct platform_device *pdev)
{
	struct clk *otg_clk;
	u32 phypwr;
	int err;

	otg_clk = clk_get(&pdev->dev, "usbotg");
	if (IS_ERR(otg_clk)) {
		dev_err(&pdev->dev, "Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	err = clk_enable(otg_clk);
	if (err) {
		clk_put(otg_clk);
		return err;
	}

	/* set to suspend HSIC 0 and 1 and standard of PHY1 */
	phypwr = readl(EXYNOS4_PHYPWR);
	phypwr |= (PHY1_STD_FORCE_SUSPEND | PHY1_HSIC0_FORCE_SUSPEND
			| PHY1_HSIC1_FORCE_SUSPEND);
	writel(phypwr, EXYNOS4_PHYPWR);

	clk_disable(otg_clk);
	clk_put(otg_clk);

	return 0;
}

static int exynos4_usb_phy1_resume(struct platform_device *pdev)
{
	struct clk *otg_clk;
	u32 rstcon;
	u32 phypwr;
	int err;

	otg_clk = clk_get(&pdev->dev, "usbotg");
	if (IS_ERR(otg_clk)) {
		dev_err(&pdev->dev, "Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	err = clk_enable(otg_clk);
	if (err) {
		clk_put(otg_clk);
		return err;
	}

	if (readl(S5P_USBHOST_PHY_CONTROL) & S5P_USBHOST_PHY_ENABLE) {
		/* set to suspend HSIC 0 and 1 and standard of PHY1 */
		phypwr = readl(EXYNOS4_PHYPWR);
		phypwr &= ~(PHY1_STD_FORCE_SUSPEND | PHY1_HSIC0_FORCE_SUSPEND
				| PHY1_HSIC1_FORCE_SUSPEND);
		writel(phypwr, EXYNOS4_PHYPWR);
	} else {
		writel(readl(S5P_USBHOST_PHY_CONTROL) | S5P_USBHOST_PHY_ENABLE,
				S5P_USBHOST_PHY_CONTROL);

		/* set to normal HSIC 0 and 1 of PHY1 */
		writel((readl(EXYNOS4_PHYPWR) & ~PHY1_HSIC_NORMAL_MASK),
				EXYNOS4_PHYPWR);

		/* set to normal standard USB of PHY1 */
		writel((readl(EXYNOS4_PHYPWR) & ~PHY1_STD_NORMAL_MASK), EXYNOS4_PHYPWR);

		/* reset all ports of both PHY and Link */
		rstcon = readl(EXYNOS4_RSTCON) | HOST_LINK_PORT_SWRST_MASK |
			PHY1_SWRST_MASK;
		writel(rstcon, EXYNOS4_RSTCON);
		udelay(10);

		rstcon &= ~(HOST_LINK_PORT_SWRST_MASK | PHY1_SWRST_MASK);
		writel(rstcon, EXYNOS4_RSTCON);
		udelay(80);
	}

	clk_disable(otg_clk);
	clk_put(otg_clk);

	return 0;
}

static int exynos4_usb_phy1_init(struct platform_device *pdev)
{
	struct clk *otg_clk;
	struct clk *xusbxti_clk;
	u32 phyclk;
	u32 rstcon;
	int err;

	otg_clk = clk_get(&pdev->dev, "usbotg");
	if (IS_ERR(otg_clk)) {
		dev_err(&pdev->dev, "Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	err = clk_enable(otg_clk);
	if (err) {
		clk_put(otg_clk);
		return err;
	}

	if (readl(S5P_USBHOST_PHY_CONTROL) & S5P_USBHOST_PHY_ENABLE) {
		dev_err(&pdev->dev, "Already power on PHY\n");
		clk_disable(otg_clk);
		clk_put(otg_clk);
		return -EINVAL;
	}

	writel(readl(S5P_USBHOST_PHY_CONTROL) | S5P_USBHOST_PHY_ENABLE,
			S5P_USBHOST_PHY_CONTROL);

#ifdef CONFIG_CPU_EXYNOS4210
	writel((__raw_readl(ETC6PUD) & ~(0x3 << 14)) | (0x3 << 14),
		ETC6PUD);
#else
	writel((__raw_readl(ETC6PUD) & ~(0x3 << 14)) | (0x1 << 14),
		ETC6PUD);
#endif
	/* set clock frequency for PLL */
	phyclk = readl(EXYNOS4_PHYCLK) & ~CLKSEL_MASK;

	xusbxti_clk = clk_get(&pdev->dev, "xusbxti");
	if (xusbxti_clk && !IS_ERR(xusbxti_clk)) {
		switch (clk_get_rate(xusbxti_clk)) {
		case 12 * MHZ:
			phyclk |= CLKSEL_12M;
			break;
		case 24 * MHZ:
			phyclk |= CLKSEL_24M;
			break;
		default:
		case 48 * MHZ:
			/* default reference clock */
			break;
		}
		clk_put(xusbxti_clk);
	}

	phyclk |= PHY1_COMMON_ON_N;
	writel(phyclk, EXYNOS4_PHYCLK);

	/* floating prevention logic: disable */
	writel((readl(EXYNOS4_PHY1CON) | FPENABLEN), EXYNOS4_PHY1CON);

	/* set to normal HSIC 0 and 1 of PHY1 */
	writel((readl(EXYNOS4_PHYPWR) & ~PHY1_HSIC_NORMAL_MASK),
			EXYNOS4_PHYPWR);

	/* set to normal standard USB of PHY1 */
	writel((readl(EXYNOS4_PHYPWR) & ~PHY1_STD_NORMAL_MASK), EXYNOS4_PHYPWR);

	/* reset all ports of both PHY and Link */
	rstcon = readl(EXYNOS4_RSTCON) | HOST_LINK_PORT_SWRST_MASK |
		PHY1_SWRST_MASK;
	writel(rstcon, EXYNOS4_RSTCON);
	udelay(10);

	rstcon &= ~(HOST_LINK_PORT_SWRST_MASK | PHY1_SWRST_MASK);
	writel(rstcon, EXYNOS4_RSTCON);
	udelay(80);

	clk_disable(otg_clk);
	clk_put(otg_clk);

	return 0;
}

static int exynos4_usb_phy1_exit(struct platform_device *pdev)
{
	struct clk *otg_clk;
	int err;

	otg_clk = clk_get(&pdev->dev, "usbotg");
	if (IS_ERR(otg_clk)) {
		dev_err(&pdev->dev, "Failed to get otg clock\n");
		return PTR_ERR(otg_clk);
	}

	err = clk_enable(otg_clk);
	if (err) {
		clk_put(otg_clk);
		return err;
	}

	writel((readl(EXYNOS4_PHYPWR) | PHY1_STD_ANALOG_POWERDOWN),
			EXYNOS4_PHYPWR);

	writel(readl(S5P_USBHOST_PHY_CONTROL) & ~S5P_USBHOST_PHY_ENABLE,
			S5P_USBHOST_PHY_CONTROL);

	clk_disable(otg_clk);
	clk_put(otg_clk);

	return 0;
}

int s5p_usb_phy_suspend(struct platform_device *pdev, int type)
{
	if (type == S5P_USB_PHY_HOST)
		return exynos4_usb_phy1_suspend(pdev);

	return -EINVAL;
}

int s5p_usb_phy_resume(struct platform_device *pdev, int type)
{
	if (type == S5P_USB_PHY_HOST)
		return exynos4_usb_phy1_resume(pdev);

	return -EINVAL;
}

int s5p_usb_phy_init(struct platform_device *pdev, int type)
{
	if (type == S5P_USB_PHY_HOST)
		return exynos4_usb_phy1_init(pdev);
	else if (type == S5P_USB_PHY_DEVICE)
		return exynos4_usb_phy0_init(pdev);

	return -EINVAL;
}

int s5p_usb_phy_exit(struct platform_device *pdev, int type)
{
	if (type == S5P_USB_PHY_HOST)
		return exynos4_usb_phy1_exit(pdev);
	else if (type == S5P_USB_PHY_DEVICE)
		return exynos4_usb_phy0_exit(pdev);

	return -EINVAL;
}
