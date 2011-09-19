/* linux/arch/arm/mach-exynos/include/mach/dev-sysmmu.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS - System MMU support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ARM_MACH_EXYNOS_SYSMMU_H_
#define _ARM_MACH_EXYNOS_SYSMMU_H_

#ifdef CONFIG_S5P_SYSTEM_MMU
#include <linux/device.h>

#define SYSMMU_PLATDEV(ipname) exynos_device_sysmmu_##ipname

#ifdef CONFIG_EXYNOS4_DEV_PD
#define ASSIGN_SYSMMU_POWERDOMAIN(ipname, powerdomain) \
		SYSMMU_PLATDEV(mfc_l).dev.parent = powerdomain
#else
#define ASSIGN_SYSTEM_POWERDOMAIN(ipname, powerdomain) do { } while (0)
#endif

#define SYSMMU_DEVNAME_BASE "s5p-sysmmu"
#define SYSMMU_CLOCK_NAME(ipname, id) SYSMMU_DEVNAME_BASE "." #id

extern struct platform_device SYSMMU_PLATDEV(sss);
extern struct platform_device SYSMMU_PLATDEV(fimc0);
extern struct platform_device SYSMMU_PLATDEV(fimc1);
extern struct platform_device SYSMMU_PLATDEV(fimc2);
extern struct platform_device SYSMMU_PLATDEV(fimc3);
extern struct platform_device SYSMMU_PLATDEV(jpeg);
extern struct platform_device SYSMMU_PLATDEV(fimd0);
extern struct platform_device SYSMMU_PLATDEV(fimd1);
extern struct platform_device SYSMMU_PLATDEV(pcie);
extern struct platform_device SYSMMU_PLATDEV(g2d);
extern struct platform_device SYSMMU_PLATDEV(rot);
extern struct platform_device SYSMMU_PLATDEV(mdma);
extern struct platform_device SYSMMU_PLATDEV(tv);
extern struct platform_device SYSMMU_PLATDEV(mfc_l);
extern struct platform_device SYSMMU_PLATDEV(mfc_r);
extern struct platform_device SYSMMU_PLATDEV(g2d_acp);

static inline void sysmmu_set_owner(struct device *sysmmu, struct device *owner)
{
	sysmmu->platform_data = owner;
}

#else /* !CONFIG_S5P_SYSTEM_MMU */
#define sysmmu_set_owner(sysmmu, owner) do { } while (0)
#define ASSIGN_SYSMMU_POWERDOMAIN(ipname, powerdomain) do { } while (0)
#endif

#endif /* _ARM_MACH_EXYNOS_SYSMMU_H_ */
