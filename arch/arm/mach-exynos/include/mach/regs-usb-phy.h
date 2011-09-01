/*
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Author: Yulgon Kim <yulgon.kim@samsung.com>
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __ASM_ARCH_REGS_USB_PHY_H
#define __ASM_ARCH_REGS_USB_PHY_H

#define EXYNOS4_HSOTG_PHYREG(x)			((x) + S3C_VA_USB_HSPHY)

#include "regs-usb-phy-4210.h"
#include "regs-usb-phy-4212.h"

#define EXYNOS4_PHYPWR				EXYNOS4_HSOTG_PHYREG(0x00)

#define PHY1_STD_SLEEP				(1 << 8)
#define PHY1_STD_ANALOG_POWERDOWN		(1 << 7)
#define PHY1_STD_FORCE_SUSPEND			(1 << 6)
#define PHY1_STD_NORMAL_MASK			(0x7 << 6)

#define PHY0_SLEEP				(1 << 5)
#define PHY0_OTG_DISABLE			(1 << 4)
#define PHY0_ANALOG_POWERDOWN			(1 << 3)
#define PHY0_FORCE_SUSPEND			(1 << 0)
#define PHY0_NORMAL_MASK			(0x29 << 0)

#define EXYNOS4_PHYCLK				EXYNOS4_HSOTG_PHYREG(0x04)
#define PHY1_COMMON_ON_N			(1 << 7)
#define PHY0_COMMON_ON_N			(1 << 4)

#define CLKSEL_SHIFT				(0)

#define EXYNOS4_RSTCON				EXYNOS4_HSOTG_PHYREG(0x08)

#define PHY0_SWRST_MASK				(0x7 << 0)
#define PHY0_PHYLINK_SWRST			(1 << 2)
#define PHY0_HLINK_SWRST			(1 << 1)
#define PHY0_SWRST				(1 << 0)

#define EXYNOS4_PHY1CON				EXYNOS4_HSOTG_PHYREG(0x34)
#define FPENABLEN				(1 << 0)

#endif /* __ASM_ARCH_REGS_USB_PHY_H */
