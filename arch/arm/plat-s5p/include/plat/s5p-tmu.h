/* linux/arch/arm/plat-s5p/include/plat/s5p-tmu.h
 *
 * Copyright 2011 Samsung Electronics Co., Ltd.
 *      http://www.samsung.com/
 *
 * Header file for s5p tmu support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _S5P_THERMAL_H
#define _S5P_THERMAL_H

struct tmu_data {
	char	te1;			/* e-fused temperature for 25 */
	char	te2;			/* e-fused temperature for 85 */
	int		cooling;
	int		mode;			/* compensation mode */
	int		tmu_flag;
};

struct s5p_tmu {
	int				id;
	void __iomem	*tmu_base;
	char			temperature;
	struct device	*dev;
	struct tmu_data		data;
};

void s5p_tmu_set_platdata(struct tmu_data *pd);
struct s5p_tmu *s5p_tmu_get_platdata(void);
int s5p_tmu_get_irqno(int num);
extern struct platform_device s5p_device_tmu;
#endif /* _S5P_THERMAL_H */
