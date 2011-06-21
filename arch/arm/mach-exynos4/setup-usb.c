/* linux/arch/arm/mach-exynos4/setup-usb.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 *
 * Base USB PHY configuration
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/usb/ch9.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/regs-clock.h>

#include <plat/regs-otg.h>
#define ETC6PUD		(S5P_VA_GPIO2 + 0x228)

enum usb_clk_type {
	USBOTG_CLK, USBHOST_CLK
};

static void usb_clk_get(enum usb_clk_type clk_type)
{
	struct clk *usb_clk;

	if (clk_type == USBOTG_CLK) {
		usb_clk = clk_get(NULL, "usbotg");

		if (IS_ERR(usb_clk)) {
			printk(KERN_ERR"cannot get usbotg clock\n");
			return;
		}
	} else if (clk_type == USBHOST_CLK) {
		usb_clk = clk_get(NULL, "usbhost");

		if (IS_ERR(usb_clk)) {
			printk(KERN_ERR"cannot get usbhost clock\n");
			return;
		}
	} else {
		printk(KERN_ERR"Undefine usb_clock\n");
		return ;
	}

	clk_enable(usb_clk);
	clk_put(usb_clk);
}

static void usb_clk_put(enum usb_clk_type clk_type)
{
	struct clk *usb_clk;

	if (clk_type == USBOTG_CLK) {
		usb_clk = clk_get(NULL, "usbotg");

		if (IS_ERR(usb_clk)) {
			printk(KERN_ERR"cannot get usbotg clock\n");
			return;
		}
	} else if (clk_type == USBHOST_CLK) {
		usb_clk = clk_get(NULL, "usbhost");

		if (IS_ERR(usb_clk)) {
			printk(KERN_ERR"cannot get usbhost clock\n");
			return;
		}
	} else {
		printk(KERN_ERR"Undefine usb_clock\n");
		return ;
	}

	clk_disable(usb_clk);
	clk_put(usb_clk);
}

/* Initializes OTG Phy. */
void otg_phy_init(void)
{
	usb_clk_get(USBOTG_CLK);

	__raw_writel(__raw_readl(S5P_USBOTG_PHY_CONTROL)
		|(0x1<<0), S5P_USBOTG_PHY_CONTROL);
	__raw_writel((__raw_readl(S3C_USBOTG_PHYPWR)
		&~(0x7<<3)&~(0x1<<0)), S3C_USBOTG_PHYPWR);
	__raw_writel((__raw_readl(S3C_USBOTG_PHYCLK)
		&~(0x5<<2))|(0x3<<0), S3C_USBOTG_PHYCLK); /* PLL 24Mhz */
	__raw_writel((__raw_readl(S3C_USBOTG_RSTCON)
		&~(0x3<<1))|(0x1<<0), S3C_USBOTG_RSTCON);
	udelay(10);
	__raw_writel(__raw_readl(S3C_USBOTG_RSTCON)
		&~(0x7<<0), S3C_USBOTG_RSTCON);
	udelay(10);
}
EXPORT_SYMBOL(otg_phy_init);

/* OTG PHY Power Off */
void otg_phy_off(void)
{
	__raw_writel(__raw_readl(S3C_USBOTG_PHYPWR)
		|(0x3<<3), S3C_USBOTG_PHYPWR);
	__raw_writel(__raw_readl(S5P_USBOTG_PHY_CONTROL)
		&~(1<<0), S5P_USBOTG_PHY_CONTROL);

	usb_clk_put(USBOTG_CLK);
}
EXPORT_SYMBOL(otg_phy_off);

void usb_host_phy_init(void __iomem *base_address)
{
	/*  Must be enable usbhost & usbotg clk  */
	usb_clk_get(USBHOST_CLK);
	usb_clk_get(USBOTG_CLK);

	if (__raw_readl(S5P_USBHOST_PHY_CONTROL) & (0x1<<0)) {
		printk(KERN_ERR"[usb_host_phy_init]Already power on PHY\n");
		usb_clk_get(USBOTG_CLK);
		return;
	}

#ifdef CONFIG_CPU_EXYNOS4210
	__raw_writel((__raw_readl(ETC6PUD) & ~(0x3 << 14)) | (0x3 << 14),
		ETC6PUD);
#else
	__raw_writel((__raw_readl(ETC6PUD) & ~(0x3 << 14)) | (0x1 << 14),
		ETC6PUD);
#endif

	__raw_writel(__raw_readl(S5P_USBHOST_PHY_CONTROL)
		|(0x1<<0), S5P_USBHOST_PHY_CONTROL);

	/* floating prevention logic : disable */
	__raw_writel((__raw_readl(S3C_USBOTG_PHY1CON) | (0x1<<0)),
		S3C_USBOTG_PHY1CON);

	/* set hsic phy0,1 to normal */
	__raw_writel((__raw_readl(S3C_USBOTG_PHYPWR) & ~(0xf<<9)),
		S3C_USBOTG_PHYPWR);

	/* phy-power on */
	__raw_writel((__raw_readl(S3C_USBOTG_PHYPWR) & ~(0x7<<6)),
		S3C_USBOTG_PHYPWR);

	/* set clock source for PLL (24MHz) */
	__raw_writel((__raw_readl(S3C_USBOTG_PHYCLK) | (0x1<<7) | (0x3<<0)),
		S3C_USBOTG_PHYCLK);

	/* reset all ports of both PHY and Link */
	__raw_writel((__raw_readl(S3C_USBOTG_RSTCON) | (0x1<<6) | (0x7<<3)),
		S3C_USBOTG_RSTCON);
	udelay(10);
	__raw_writel((__raw_readl(S3C_USBOTG_RSTCON) & ~(0x1<<6) & ~(0x7<<3)),
		S3C_USBOTG_RSTCON);
	udelay(80);

	usb_clk_put(USBOTG_CLK);
	__raw_writel(0x03C00000, base_address + 0x90);
}
EXPORT_SYMBOL(usb_host_phy_init);

void usb_host_phy_off(void)
{
	usb_clk_get(USBOTG_CLK);

	__raw_writel(__raw_readl(S3C_USBOTG_PHYPWR)
		|(0x1<<7)|(0x1<<6), S3C_USBOTG_PHYPWR);
	__raw_writel(__raw_readl(S5P_USBHOST_PHY_CONTROL)
		&~(1<<0), S5P_USBHOST_PHY_CONTROL);

	usb_clk_put(USBOTG_CLK);
	usb_clk_put(USBHOST_CLK);
}
EXPORT_SYMBOL(usb_host_phy_off);

/* For L2 suspend */
void usb_host_phy_suspend(void)
{
	unsigned int reg = __raw_readl(S3C_USBOTG_PHYPWR);

	usb_clk_get(USBOTG_CLK);
	/* if OHCI isn't used, 7 bit clear */
	__raw_writel((__raw_readl(S3C_USBOTG_PHYCLK) & ~(0x1<<7)),
		S3C_USBOTG_PHYCLK);

	reg |= (0x1<<6);  /* phy port0 suspend */
	reg |= (0x1<<9);  /* phy hsic port0 suspend */
	reg |= (0x1<<11); /* phy hsic port1 suspend */
	__raw_writel(reg, S3C_USBOTG_PHYPWR);

	usb_clk_put(USBOTG_CLK);
}
EXPORT_SYMBOL(usb_host_phy_suspend);

void usb_host_phy_resume(void)
{
	usb_clk_get(USBOTG_CLK);

	__raw_writel(__raw_readl(S3C_USBOTG_PHYPWR)
		& ~((0x1<<6)|(0x1<<9)|(0x1<<11)), S3C_USBOTG_PHYPWR);
	/* if OHCI 48mhz, 7 bit set */
	__raw_writel((__raw_readl(S3C_USBOTG_PHYCLK) | (0x1<<7)),
		S3C_USBOTG_PHYCLK);

	usb_clk_put(USBOTG_CLK);
}
EXPORT_SYMBOL(usb_host_phy_resume);

/* For LPM(L1) suspend */
void usb_host_phy_sleep(void)
{
	unsigned int reg = __raw_readl(S3C_USBOTG_PHYPWR);

	usb_clk_get(USBOTG_CLK);

	/* if OHCI isn't used, 7 bit clear */
	__raw_writel((__raw_readl(S3C_USBOTG_PHYCLK) & ~(0x1<<7)),
		S3C_USBOTG_PHYCLK);

	reg |= (0x1<<8);  /* phy port0 sleep */
	reg |= (0x1<<10); /* phy hsic port0 sleep */
	reg |= (0x1<<12); /* phy hsic port1 sleep */
	__raw_writel(reg, S3C_USBOTG_PHYPWR);

	usb_clk_put(USBOTG_CLK);
}
EXPORT_SYMBOL(usb_host_phy_sleep);

void usb_host_phy_wakeup(void)
{
	usb_clk_get(USBOTG_CLK);

	__raw_writel(__raw_readl(S3C_USBOTG_PHYPWR)
		& ~((0x1<<8)|(0x1<<10)|(0x1<<12)), S3C_USBOTG_PHYPWR);
	/* if OHCI 48mhz, 7 bit set */
	__raw_writel((__raw_readl(S3C_USBOTG_PHYCLK) | (0x1<<7)),
		S3C_USBOTG_PHYCLK);

	usb_clk_put(USBOTG_CLK);
}
EXPORT_SYMBOL(usb_host_phy_wakeup);
