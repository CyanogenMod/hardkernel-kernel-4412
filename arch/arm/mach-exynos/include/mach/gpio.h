/* linux/arch/arm/mach-exynos/include/mach/gpio.h
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS - gpio map definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H __FILE__

#include "gpio-exynos4.h"
#include "gpio-exynos5.h"

#if defined(CONFIG_ARCH_EXYNOS4)
#define S3C_GPIO_END		EXYNOS4_GPIO_END
#define ARCH_NR_GPIOS		(EXYNOS4XXX_GPIO_END +	\
				CONFIG_SAMSUNG_GPIO_EXTRA)
#define EXYNOS_GPZ(_nr)		EXYNOS4_GPZ(_nr)
#define EXYNOS_GPC0(_nr)	EXYNOS4_GPC0(_nr)
#define EXYNOS_GPC1(_nr)	EXYNOS4_GPC1(_nr)
#elif defined(CONFIG_ARCH_EXYNOS5)
#define S3C_GPIO_END		EXYNOS5_GPIO_END
#define ARCH_NR_GPIOS		(EXYNOS5_GPIO_END +	\
				CONFIG_SAMSUNG_GPIO_EXTRA)
#define EXYNOS_GPZ(_nr)		EXYNOS5_GPZ(_nr)
#define EXYNOS_GPC0(_nr)	EXYNOS5_GPC0(_nr)
#define EXYNOS_GPC1(_nr)	EXYNOS5_GPC1(_nr)
#else
#error "ARCH_EXYNOS* is not defined"
#endif

#include <asm-generic/gpio.h>

#endif /* __ASM_ARCH_GPIO_H */
