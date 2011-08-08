/*
 * EXYNOS4 MIPI-DSIM driver.
 *
 * Author: InKi Dae <inki.dae@samsung.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <plat/clock.h>
#include <plat/regs-dsim.h>
#include <mach/map.h>
#include <mach/regs-pmu.h>

#define S5P_MIPI_M_RESETN 4

static void s5p_dsim_enable_d_phy(unsigned char enable)
{
	unsigned int reg;

	reg = (readl(S5P_MIPI_DPHY_CONTROL(0))) & ~(1 << 0);
	reg |= (enable << 0);
	writel(reg, S5P_MIPI_DPHY_CONTROL(0));
}

static void s5p_dsim_enable_dsi_master(unsigned char enable)
{
	unsigned int reg;

	reg = (readl(S5P_MIPI_DPHY_CONTROL(0))) & ~(1 << 2);
	reg |= (enable << 2);
	writel(reg, S5P_MIPI_DPHY_CONTROL(0));
}

void s5p_dsim_enable_clk(void *d_clk, unsigned char enable)
{
	struct clk *dsim_clk = (struct clk *) d_clk;

	if (enable)
		clk_enable(dsim_clk);
	else
		clk_disable(dsim_clk);
}

void s5p_dsim_part_reset(void)
{
	writel(S5P_MIPI_M_RESETN, S5P_MIPI_DPHY_CONTROL(0));
}

void s5p_dsim_init_d_phy(unsigned int dsim_base)
{
	/* enable D-PHY */
	s5p_dsim_enable_d_phy(1);

	/* enable DSI master block */
	s5p_dsim_enable_dsi_master(1);
}
