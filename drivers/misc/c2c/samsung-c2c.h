/*
 * Samsung C2C driver
 *
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Author: Kisang Lee <kisang80.lee@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef SAMSUNG_C2C_H
#define SAMSUNG_C2C_H

/* This timer will be only used for debugging
#defien ENABLE_C2CSTATE_TIMER
*/
#define C2C_DEV_NAME "c2c_dev"

enum c2c_set_clear {
	C2C_CLEAR = 0,
	C2C_SET = 1,
};

struct c2c_state_control {
	void __iomem *ap_sscm_addr;
	void __iomem *cp_sscm_addr;

	struct device* c2c_dev;

	u32 rx_width;
	u32 tx_width;

	u32 max_clk;
	u32 default_clk;

	enum c2c_opp_mode opp_mode;
	/* Below variables are needed in reset for retention */
	u32 rtt_enableset;
};

static struct c2c_state_control c2c_con;

static inline void c2c_writel(u32 val, int reg)
{
        writel(val, c2c_con.ap_sscm_addr + reg);
}

static inline void c2c_writew(u16 val, int reg)
{
        writew(val, c2c_con.ap_sscm_addr + reg);
}

static inline void c2c_writeb(u8 val, int reg)
{
        writeb(val, c2c_con.ap_sscm_addr + reg);
}

static inline u32 c2c_readl(int reg)
{
        return readl(c2c_con.ap_sscm_addr + reg);
}

static inline u16 c2c_readw(int reg)
{
        return readw(c2c_con.ap_sscm_addr + reg);
}

static inline u8 c2c_readb(int reg)
{
        return readb(c2c_con.ap_sscm_addr + reg);
}

static inline void c2c_writel_cp(u32 val, int reg)
{
        writel(val, c2c_con.cp_sscm_addr + reg);
}

static inline void c2c_writew_cp(u16 val, int reg)
{
        writew(val, c2c_con.cp_sscm_addr + reg);
}

static inline void c2c_writeb_cp(u8 val, int reg)
{
        writeb(val, c2c_con.cp_sscm_addr + reg);
}

static inline u32 c2c_readl_cp(int reg)
{
        return readl(c2c_con.cp_sscm_addr + reg);
}

static inline u16 c2c_readw_cp(int reg)
{
        return readw(c2c_con.cp_sscm_addr + reg);
}

static inline u8 c2c_readb_cp(int reg)
{
        return readb(c2c_con.cp_sscm_addr + reg);
}

static inline enum c2c_set_clear c2c_get_clock_gating(void)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);
	if (sysreg & (1 << C2C_SYSREG_CG))
		return C2C_SET;
	else
		return C2C_CLEAR;
}

static inline void c2c_set_clock_gating(enum c2c_set_clear val)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	if (val == C2C_SET)
		sysreg |= (1 << C2C_SYSREG_CG);
	else
		sysreg &= ~(1 << C2C_SYSREG_CG);

	writel(sysreg, SYSREG_CTRL_C2C);
}

static inline enum c2c_set_clear c2c_get_memdone(void)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);
	if (sysreg & (1 << C2C_SYSREG_MD))
		return C2C_SET;
	else
		return C2C_CLEAR;
}

static inline void c2c_set_memdone(enum c2c_set_clear val)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	if (val == C2C_SET)
		sysreg |= (1 << C2C_SYSREG_MD);
	else
		sysreg &= ~(1 << C2C_SYSREG_MD);

	writel(sysreg, SYSREG_CTRL_C2C);
}

static inline enum c2c_set_clear c2c_get_master_on(void)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);
	if (sysreg & (1 << C2C_SYSREG_MO))
		return C2C_SET;
	else
		return C2C_CLEAR;
}

static inline void c2c_set_master_on(enum c2c_set_clear val)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	if (val == C2C_SET)
		sysreg |= (1 << C2C_SYSREG_MO);
	else
		sysreg &= ~(1 << C2C_SYSREG_MO);

	writel(sysreg, SYSREG_CTRL_C2C);
}

static inline u32 c2c_get_func_clk(void)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	sysreg &= (0x3ff << C2C_SYSREG_FCLK);

	return (sysreg >> C2C_SYSREG_FCLK);
}

static inline void c2c_set_func_clk(u32 val)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	sysreg &= ~(0x3ff << C2C_SYSREG_FCLK);
	sysreg |= (val << C2C_SYSREG_FCLK);

	writel(sysreg, SYSREG_CTRL_C2C);
}

static inline u32 c2c_get_tx_buswidth(void)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	sysreg &= (0x3 << C2C_SYSREG_TXW);

	return (sysreg >> C2C_SYSREG_TXW);
}

static inline void c2c_set_tx_buswidth(u32 val)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	sysreg &= ~(0x3 << C2C_SYSREG_TXW);
	sysreg |= (val << C2C_SYSREG_TXW);

	writel(sysreg, SYSREG_CTRL_C2C);
}

static inline u32 c2c_get_rx_buswidth(void)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	sysreg &= (0x3 << C2C_SYSREG_RXW);

	return (sysreg >> C2C_SYSREG_RXW);
}

static inline void c2c_set_rx_buswidth(u32 val)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	sysreg &= ~(0x3 << C2C_SYSREG_RXW);
	sysreg |= (val << C2C_SYSREG_RXW);

	writel(sysreg, SYSREG_CTRL_C2C);
}

static inline enum c2c_set_clear c2c_get_reset(void)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);
	if (sysreg & (1 << C2C_SYSREG_RST))
		return C2C_SET;
	else
		return C2C_CLEAR;
}

static inline void c2c_set_reset(enum c2c_set_clear val)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	if (val == C2C_SET)
		sysreg |= (1 << C2C_SYSREG_RST);
	else
		sysreg &= ~(1 << C2C_SYSREG_RST);

	writel(sysreg, SYSREG_CTRL_C2C);
}

static inline void reset_c2c_retreg(void)
{
	writel((0x1 << C2C_SYSREG_RTRST), SYSREG_CTRL_C2C);
}

static inline u32 c2c_get_base_addr(void)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	sysreg &= (0x3ff << C2C_SYSREG_BASE_ADDR);

	return (sysreg >> C2C_SYSREG_BASE_ADDR);
}

static inline void c2c_set_base_addr(u32 val)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	sysreg &= ~(0x3ff << C2C_SYSREG_BASE_ADDR);
	sysreg |= (val << C2C_SYSREG_BASE_ADDR);

	writel(sysreg, SYSREG_CTRL_C2C);
}

static inline u32 c2c_get_shdmem_size(void)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	sysreg &= (0x7 << C2C_SYSREG_DRAM_SIZE);

	return (sysreg >> C2C_SYSREG_DRAM_SIZE);
}

static inline void c2c_set_shdmem_size(u32 val)
{
	u32 sysreg = readl(SYSREG_CTRL_C2C);

	sysreg &= ~(0x7 << C2C_SYSREG_DRAM_SIZE);
	sysreg |= (val << C2C_SYSREG_DRAM_SIZE);

	writel(sysreg, SYSREG_CTRL_C2C);
}

#ifdef CONFIG_C2C_DEBUG
#define c2c_dbg printk
#else
#define c2c_dbg(x...)
#endif

#endif