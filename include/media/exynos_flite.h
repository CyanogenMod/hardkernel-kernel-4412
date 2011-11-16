/*
 * Samsung S5P SoC camera interface driver header
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef EXYNOS_FLITE_H_
#define EXYNOS_FLITE_H_

#ifdef CONFIG_ARCH_EXYNOS4
#include <plat/fimc.h>
#else
enum flite_cam_type {
	CAM_TYPE_ITU,
	CAM_TYPE_MIPI,
};

struct s3c_platform_camera {
	enum flite_cam_type type;
	bool use_isp;
	int inv_pclk;
	int inv_vsync;
	int inv_href;
	int inv_hsync;
};
#endif

/**
 * struct exynos_platform_flite - camera host interface platform data
 *
 * @cam: properties of camera sensor required for host interface setup
 */
struct exynos_platform_flite {
	struct s3c_platform_camera *cam;
};

extern void exynos_flite0_set_platdata(struct exynos_platform_flite *pd);
extern void exynos_flite1_set_platdata(struct exynos_platform_flite *pd);
#endif /* EXYNOS_FLITE_H_*/
