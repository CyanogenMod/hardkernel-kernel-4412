/* linux/arch/arm/mach-exynos/dev-dwc_ahsata.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * http://www.samsung.com
 * Author: Srikanth TS <ts.srikanth@samsung.com> for Samsung
 *
 * EXYNOS5 - AHCI SATA3.0 support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/ahci_platform.h>

#include <plat/cpu.h>

#include <mach/irqs.h>
#include <mach/map.h>
#include <mach/regs-pmu.h>
#include <mach/regs-pmu5.h>
#include <mach/map-exynos5.h>

#define	SATA_TIME_LIMIT			10000
#define	PMU_BASE_ADDRS			0x10040000
#define	PMU_SATA_PHY_CTRL		0x724
#define	SATA_PHY_I2C_SLAVE_ADDRS	0x70

#define	SATA_SFR_ID			0x0
#define	SFR_IDb_ID_NUM			(1<<16)
#define	SFR_IDb_REV_CODE		(1<<0)

#define	SATA_RESET			0x4
#define	RESETb_CMN_RST_N		(1<<1)
#define	LINK_RESET			0xF0000

#define	SATA_MODE0			0x10
#define	MODE0b_P0_PHY_SPDMODE(x)	(x<<0)

#define	SATA_CTRL0			0x14
#define	CTRL0b_P0_PHY_CALIBRATED_SEL	(1<<9)
#define	CTRL0b_P0_PHY_CALIBRATED	(1<<8)

#define	SATA_STAT0			0x18

#define	SATA_TBD			0x9C
#define	SATA_PHSATA_CTRLM		0xE0
#define	PHCTRLMb_REF_RATE		(1<<1)
#define	PHCTRLMb_HIGH_SPEED		(1<<0)

#define	SATA_PHSATA_CTRL0		0xE4

#define	SATA_PHSATA_STATM		0xF0
#define	PHSTATMb_PLL_LOCKED		(1<<0)

#define	SATA_PHSATA_STAT0		0xF4

/********************** I2C**************/
#define	SATA_I2C_PHY_ADDR		0x70
#define	SATA_I2C_CON			0x00
#define	SATA_I2C_STAT			0x04
#define	SATA_I2C_ADDR			0x08
#define	SATA_I2C_DS			0x0C
#define	SATA_I2C_LC			0x10

/* I2CCON reg */
#define	CONb_ACKEN			(1<<7)
#define	CONb_CLK512			(1<<6)
#define	CONb_CLK16			(~CONb_CLK512)
#define	CONb_INTEN			(1<<5)
#define	CONb_INTPND			(1<<4)
#define	CONb_TXCLK_PS			(0xF)

/* I2CSTAT reg */
#define	STAb_MSTR			(0x2<<6)
#define	STAb_MSTT			(0x3<<6)
#define	STAb_BSYST			(1<<5)
#define	STAb_RTEN			(1<<4)
#define	STAb_LAST			(1<<0)

#define	LC_FLTR_EN			(1<<2)

#define	SATA_PHY_CON_RESET		0xF003F

#define	HOST_PORTS_IMPL			0xC
#define	SCLK_SATA_FREQ			(66 * MHZ)

enum {
	GEN1 = 0,
	GEN2 = 1,
	GEN3 = 2,
};

static void __iomem *phy_i2c_base, *phy_ctrl;
u32 time_limit_cnt;

bool sata_is_reg(void __iomem *base, u32 reg, u32 checkbit, u32 Status)
{
	if ((__raw_readl(base + reg) & checkbit) == Status)
		return true;
	else
		return false;
}

bool wait_for_reg_status(void __iomem *base,  u32 reg,  u32 checkbit,
		u32 Status)
{
	time_limit_cnt = 0;
	while (!sata_is_reg(base, reg,  checkbit, Status)) {
		if (time_limit_cnt == SATA_TIME_LIMIT) {
			printk(KERN_ERR " Register Status wait FAIL\n");
			return false;
		}
		udelay(1000);
		time_limit_cnt++;
	}
	return true;
}


void sata_set_gen(u8 gen)
{
	__raw_writel(gen, phy_ctrl + SATA_MODE0);
}

/* Address :I2C Address */
void sata_i2c_write_addrs(u8 data)
{
	__raw_writeb((data & 0xFE), phy_i2c_base + SATA_I2C_DS);
}

void sata_i2c_write_data(u8 data)
{
	__raw_writeb((data), phy_i2c_base + SATA_I2C_DS);
}

u8 sata_i2c_read_data(void)
{
	return __raw_readb(phy_i2c_base + SATA_I2C_DS);
}

void sata_i2c_start(void)
{
	u32 val;
	val = __raw_readl(phy_i2c_base + SATA_I2C_STAT);
	val  |= STAb_BSYST;
	__raw_writel(val, phy_i2c_base + SATA_I2C_STAT);
}

void sata_i2c_stop(void)
{
	u32 val;
	val = __raw_readl(phy_i2c_base + SATA_I2C_STAT);
	val  &= ~STAb_BSYST;
	__raw_writel(val, phy_i2c_base + SATA_I2C_STAT);
}

bool sata_i2c_get_int_status(void)
{
	if ((__raw_readl(phy_i2c_base + SATA_I2C_CON)) & CONb_INTPND)
		return true;
	else
		return false;
}

bool sata_i2c_is_tx_ack(void)
{
	if ((__raw_readl(phy_i2c_base +  SATA_I2C_STAT)) & STAb_LAST)
		return false;
	else
		return true;
}

bool sata_i2c_is_bus_ready(void)
{
	if ((__raw_readl(phy_i2c_base +  SATA_I2C_STAT)) & STAb_BSYST)
		return false;
	else
		return true;
}

bool sata_i2c_wait_for_busready(u32 time_out)
{
	while (--time_out) {
		if (sata_i2c_is_bus_ready())
			return true;
		udelay(100);
	}
	printk(KERN_ERR "SATA I2C wait fail for bus ready...\n");
	return false;
}

bool sata_i2c_wait_for_tx_ack(u32 time_out)
{
	while (--time_out) {
		if (sata_i2c_get_int_status()) {
			if (sata_i2c_is_tx_ack())
				return true;
		}
		udelay(100);
	}
	return false;
}

bool sata_i2c_wait_for_tx_noack(u32 time_out)
{
	while (--time_out) {
		if (sata_i2c_get_int_status())
			return true;

		udelay(100);
	}
	printk(KERN_ERR "SATA I2C wait fail for transfer no ack...\n");
	return false;
}

void sata_i2c_clear_int_status(void)
{
	u32 val;
	val = __raw_readl(phy_i2c_base + SATA_I2C_CON);
	val  &= ~CONb_INTPND;
	__raw_writel(val, phy_i2c_base + SATA_I2C_CON);
}


void sata_i2c_set_ack_gen(bool enable)
{
	u32 val;
	if (enable) {
		val = (__raw_readl(phy_i2c_base + SATA_I2C_CON))  | CONb_ACKEN;
		__raw_writel(val, phy_i2c_base + SATA_I2C_CON);
	} else {
		val = __raw_readl(phy_i2c_base + SATA_I2C_CON);
		val  &= ~CONb_ACKEN;
		__raw_writel(val, phy_i2c_base + SATA_I2C_CON);
	}

}

void sata_i2c_set_master_tx(void)
{
	u32 val;
	/* Disable I2C */
	val = __raw_readl(phy_i2c_base + SATA_I2C_STAT);
	val  &= ~STAb_RTEN;
	__raw_writel(val, phy_i2c_base + SATA_I2C_STAT);
	/* Clear Mode */
	val = __raw_readl(phy_i2c_base + SATA_I2C_STAT);
	val  &= ~STAb_MSTT;
	__raw_writel(val, phy_i2c_base + SATA_I2C_STAT);

	sata_i2c_clear_int_status();
	/* interrupt disable */
	val = __raw_readl(phy_i2c_base + SATA_I2C_CON);
	val  &= ~CONb_INTEN;
	__raw_writel(val, phy_i2c_base + SATA_I2C_CON);

	/* Master, Send mode */
	val = __raw_readl(phy_i2c_base + SATA_I2C_STAT);
	val  |=	STAb_MSTT;
	__raw_writel(val, phy_i2c_base + SATA_I2C_STAT);

	/* interrupt enable */
	val = __raw_readl(phy_i2c_base + SATA_I2C_CON);
	val  |=	CONb_INTEN;
	__raw_writel(val, phy_i2c_base + SATA_I2C_CON);

	/* Enable I2C */
	val = __raw_readl(phy_i2c_base + SATA_I2C_STAT);
	val  |= STAb_RTEN;
	__raw_writel(val, phy_i2c_base + SATA_I2C_STAT);
}
void sata_i2c_set_master_rx(void)
{
	u32 val;
	/* Disable I2C */
	val = __raw_readl(phy_i2c_base + SATA_I2C_STAT);
	val  &= ~STAb_RTEN;
	__raw_writel(val, phy_i2c_base + SATA_I2C_STAT);

	/* Clear Mode */
	val = __raw_readl(phy_i2c_base + SATA_I2C_STAT);
	val  &= ~STAb_MSTT;
	__raw_writel(val, phy_i2c_base + SATA_I2C_STAT);

	sata_i2c_clear_int_status();
	/* interrupt disable */
	val = __raw_readl(phy_i2c_base + SATA_I2C_CON);
	val  &= ~CONb_INTEN;
	__raw_writel(val, phy_i2c_base + SATA_I2C_CON);

	/* Master, Received mode */
	val = __raw_readl(phy_i2c_base + SATA_I2C_STAT);
	val  |= STAb_MSTR;
	__raw_writel(val, phy_i2c_base + SATA_I2C_STAT);

	/* interrupt enable */
	val = __raw_readl(phy_i2c_base + SATA_I2C_CON);
	val   |= CONb_INTEN;
	__raw_writel(val, phy_i2c_base + SATA_I2C_CON);
	/* Enable I2C */
	val = __raw_readl(phy_i2c_base + SATA_I2C_STAT);
	val   |= STAb_RTEN;
	__raw_writel(val, phy_i2c_base + SATA_I2C_STAT);
}

void sata_i2c_init(void)
{
	u32 val;

	val = __raw_readl(phy_i2c_base + SATA_I2C_CON);
	val  &= CONb_CLK16;
	__raw_writel(val, phy_i2c_base + SATA_I2C_CON);

	val = __raw_readl(phy_i2c_base + SATA_I2C_CON);
	val  &= ~(CONb_TXCLK_PS);
	__raw_writel(val, phy_i2c_base + SATA_I2C_CON);

	val = __raw_readl(phy_i2c_base + SATA_I2C_CON);
	val  |= (2 & CONb_TXCLK_PS);
	__raw_writel(val, phy_i2c_base + SATA_I2C_CON);

	val = __raw_readl(phy_i2c_base + SATA_I2C_LC);
	val  &= ~(LC_FLTR_EN);
	__raw_writel(val, phy_i2c_base + SATA_I2C_LC);

	sata_i2c_set_ack_gen(false);
}
bool sata_i2c_send(u8 slave_addrs, u8 addrs, u8 ucData)
{
	s32 ret = 0;
	if (!sata_i2c_wait_for_busready(SATA_TIME_LIMIT))
		return false;

	sata_i2c_init();
	sata_i2c_set_master_tx();

	__raw_writel(SATA_PHY_CON_RESET, phy_ctrl + SATA_RESET);
	sata_i2c_write_addrs(slave_addrs);
	sata_i2c_start();
	if (!sata_i2c_wait_for_tx_ack(SATA_TIME_LIMIT)) {
		ret = false;
		goto STOP;
	}
	sata_i2c_write_data(addrs);
	sata_i2c_clear_int_status();
	if (!sata_i2c_wait_for_tx_ack(SATA_TIME_LIMIT)) {
		ret = false;
		goto STOP;
	}
	sata_i2c_write_data(ucData);
	sata_i2c_clear_int_status();
	if (!sata_i2c_wait_for_tx_ack(SATA_TIME_LIMIT)) {
		ret = false;
		goto STOP;
	}
	ret = true;

STOP:
	sata_i2c_stop();
	sata_i2c_clear_int_status();
	sata_i2c_wait_for_busready(SATA_TIME_LIMIT);

	return ret;
}

bool sata_i2c_recv(u8 slave_addrs, u8 addrs, u8 *data)
{
	s32 ret = 0;
	if (!sata_i2c_wait_for_busready(SATA_TIME_LIMIT))
		return false;

	sata_i2c_init();
	sata_i2c_set_master_tx();

	sata_i2c_write_addrs(slave_addrs);
	sata_i2c_start();
	if (!sata_i2c_wait_for_tx_ack(SATA_TIME_LIMIT)) {
		ret = false;
		goto STOP;
	}
	sata_i2c_write_data(addrs);
	sata_i2c_clear_int_status();
	if (!sata_i2c_wait_for_tx_ack(SATA_TIME_LIMIT)) {
		ret = false;
		goto STOP;
	}
	sata_i2c_set_master_rx();
	sata_i2c_write_addrs(slave_addrs);
	sata_i2c_start();
	if (!sata_i2c_wait_for_tx_ack(SATA_TIME_LIMIT)) {
		ret = false;
		goto STOP;
	}
	sata_i2c_set_ack_gen(false);
	sata_i2c_clear_int_status();
	if (!sata_i2c_wait_for_tx_noack(SATA_TIME_LIMIT)) {
		ret = false;
		goto STOP;
	}

	*data = sata_i2c_read_data();
	ret = true;

STOP:
	sata_i2c_stop();
	sata_i2c_clear_int_status();
	sata_i2c_wait_for_busready(SATA_TIME_LIMIT);

	return ret;
}

static int ahci_phy_init(void __iomem *mmio)
{
	u8 uCount, i = 0;
	/* 0x3A for 40bit I/F */
	u8 reg_addrs[] = {0x22, 0x21, 0x3A};
	/* 0x0B for 40bit I/F */
	u8 default_setting_value[] = {0x30, 0x4f, 0x0B};
	u8 ucReadData[20];

	memset(ucReadData, 0, sizeof(ucReadData)/sizeof(u8));

	uCount = sizeof(reg_addrs)/sizeof(u8);
	while (i < uCount) {
		if (!sata_i2c_send(SATA_PHY_I2C_SLAVE_ADDRS, reg_addrs[i],
					default_setting_value[i]))
			return false;

		if (!sata_i2c_recv(SATA_PHY_I2C_SLAVE_ADDRS, reg_addrs[i],
					&ucReadData[i]))
			return false;

		if (ucReadData[i] != default_setting_value[i])
			return false;
		i++;
	}
	return 0;
}

static int exynos5_ahci_init(struct device *dev, void __iomem *mmio)
{
	struct clk *clk_sata, *clk_sataphy, *clk_sata_i2c, *clk_sclk_sata;
	int val, ret;

	phy_i2c_base = ioremap(EXYNOS5_PA_SATA_PHY_I2C, SZ_4K);
	if (!phy_i2c_base) {
		dev_err(dev, "failed to allocate memory for SATA PHY\n");
		return -ENOMEM;
	}

	phy_ctrl = ioremap(EXYNOS5_PA_SATA_PHY_CTRL, SZ_64K);
	if (!phy_ctrl) {
		dev_err(dev, "failed to allocate memory for SATA PHY CTRL\n");
		ret = -ENOMEM;
		goto err1;
	}

	__raw_writel(S5P_PMU_SATA_PHY_CONTROL_EN, EXYNOS5_SATA_PHY_CONTROL);

	val = 0;
	__raw_writel(val, phy_ctrl + SATA_RESET);
	val = __raw_readl(phy_ctrl + SATA_RESET);
	val  |= 0x3D;
	__raw_writel(val, phy_ctrl + SATA_RESET);

	clk_sata = clk_get(dev, "sata");
	if (IS_ERR(clk_sata)) {
		dev_err(dev, "failed to get sata clock\n");
		ret = PTR_ERR(clk_sata);
		clk_sata = NULL;
		goto err2;

	}
	clk_enable(clk_sata);

	clk_sataphy = clk_get(dev, "sata_phy");
	if (IS_ERR(clk_sataphy)) {
		dev_err(dev, "failed to get sataphy clock\n");
		ret = PTR_ERR(clk_sataphy);
		clk_sataphy = NULL;
		goto err3;
	}
	clk_enable(clk_sataphy);

	clk_sata_i2c = clk_get(dev, "sata_phy_i2c");
	if (IS_ERR(clk_sata_i2c)) {
		dev_err(dev, "failed to get sclk_sata\n");
		ret = PTR_ERR(clk_sata_i2c);
		clk_sata_i2c = NULL;
		goto err4;
	}
	clk_enable(clk_sata_i2c);

	clk_sclk_sata = clk_get(dev, "sclk_sata");
	clk_enable(clk_sclk_sata);
	if (IS_ERR(clk_sclk_sata)) {
		dev_err(dev, "failed to get sclk_sata\n");
		ret = PTR_ERR(clk_sclk_sata);
		clk_sclk_sata = NULL;
		goto err5;
	}
	clk_set_rate(clk_sclk_sata, SCLK_SATA_FREQ);

	val = __raw_readl(phy_ctrl + SATA_RESET);
	val  |= LINK_RESET;
	__raw_writel(val, phy_ctrl + SATA_RESET);

	val = __raw_readl(phy_ctrl + SATA_RESET);
	val  |= RESETb_CMN_RST_N;
	__raw_writel(val, phy_ctrl + SATA_RESET);

	val = __raw_readl(phy_ctrl + SATA_PHSATA_CTRLM);
	val  &= ~PHCTRLMb_REF_RATE;
	__raw_writel(val, phy_ctrl + SATA_PHSATA_CTRLM);

	/* High speed enable for Gen3 */
	val = __raw_readl(phy_ctrl + SATA_PHSATA_CTRLM);
	val   |= PHCTRLMb_HIGH_SPEED;
	__raw_writel(val, phy_ctrl + SATA_PHSATA_CTRLM);

	/* Port0 is available */
	__raw_writel(0x1, mmio + HOST_PORTS_IMPL);

	ret = ahci_phy_init(mmio);

	val = __raw_readl(phy_ctrl + SATA_CTRL0);
	val |= CTRL0b_P0_PHY_CALIBRATED_SEL|CTRL0b_P0_PHY_CALIBRATED;
	__raw_writel(val, phy_ctrl + SATA_CTRL0);
	sata_set_gen(GEN3);

	/* release cmu reset */
	val = __raw_readl(phy_ctrl + SATA_RESET);
	val  &= ~RESETb_CMN_RST_N;
	__raw_writel(val, phy_ctrl + SATA_RESET);

	val = __raw_readl(phy_ctrl + SATA_RESET);
	val  |= RESETb_CMN_RST_N;
	__raw_writel(val, phy_ctrl + SATA_RESET);

	if (wait_for_reg_status(phy_ctrl, SATA_PHSATA_STATM,
				PHSTATMb_PLL_LOCKED, 1)) {
		return ret;
	}
	dev_err(dev, " ahci_phy_init FAIL\n");

err5:
	clk_disable(clk_sata_i2c);
	clk_put(clk_sata_i2c);
err4:
	clk_disable(clk_sataphy);
	clk_put(clk_sataphy);
err3:
	clk_disable(clk_sata);
	clk_put(clk_sata);
err2:
	iounmap(phy_ctrl);
err1:
	iounmap(phy_i2c_base);

	return false;
}

static struct ahci_platform_data exynos5_ahci_pdata = {
	.init = exynos5_ahci_init,
};

static struct resource exynos5_ahci_resource[] = {
	[0] = {
		.start	= EXYNOS5_PA_SATA_BASE,
		.end	= EXYNOS5_PA_SATA_BASE + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SATA,
		.end	= IRQ_SATA,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 exynos5_ahci_dmamask = DMA_BIT_MASK(32);

struct platform_device exynos5_device_ahci = {
	.name		= "ahci",
	.id		= -1,
	.resource	= exynos5_ahci_resource,
	.num_resources	= ARRAY_SIZE(exynos5_ahci_resource),
	.dev		= {
		.platform_data		= &exynos5_ahci_pdata,
		.dma_mask		= &exynos5_ahci_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};
