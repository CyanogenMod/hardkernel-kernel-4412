/* linux/arch/arm/plat-s5p/include/plat/fimc_is.h
 *
 * Copyright (C) 2011 Samsung Electronics, Co. Ltd
 *
 * Exynos 4 series FIMC-IS slave device support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef EXYNOS_FIMC_IS_H_
#define EXYNOS_FIMC_IS_H_ __FILE__

#include <plat/fimc.h>
#include <linux/videodev2.h>

#define FIMC_IS_MAX_CAMIF_CLIENTS	2

enum fimc_is_cam_format {
	IS_MIPI_CSI_YCBCR422_8BIT	= 0x1e,
	IS_MIPI_CSI_RAW8		= 0x2a,
	IS_MIPI_CSI_RAW10		= 0x2b,
	IS_MIPI_CSI_RAW12		= 0x2c,
	IS_MIPI_USER_DEF_PACKET_1       = 0x30,
};

/**
 * struct exynos4_fimc_is_sensor_info  - image sensor information required for host
 *			      interace configuration.
 *
 * @board_info: pointer to I2C subdevice's board info
 * @clk_frequency: frequency of the clock the host interface provides to sensor
 * @bus_type: determines bus type, MIPI, ITU-R BT.601 etc.
 * @csi_data_align: MIPI-CSI interface data alignment in bits
 * @i2c_bus_num: i2c control bus id the sensor is attached to
 * @mux_id: FIMC camera interface multiplexer index (separate for MIPI and ITU)
 * @flags: flags defining bus signals polarity inversion (High by default)
*/
struct exynos4_fimc_is_sensor_info {

};

/**
 * struct exynos4_platform_fimc_is - camera host interface platform data
 *
 * @isp_info: properties of camera sensor required for host interface setup
*/
struct exynos4_platform_fimc_is {
	struct exynos4_fimc_is_sensor_info
		*sensor_info[FIMC_IS_MAX_CAMIF_CLIENTS];
	enum fimc_is_cam_format	fmt;	/* input format */
	int mipi_lanes;     /* MIPI data lanes */
	int mipi_settle;    /* MIPI settle */
	int mipi_align;     /* MIPI data align: 24/32 */
};

extern struct exynos4_platform_fimc_is exynos4_fimc_is_default_data;
extern void exynos4_fimc_is_set_platdata(struct exynos4_platform_fimc_is *pd);

#endif /* EXYNOS_FIMC_IS_H_ */
