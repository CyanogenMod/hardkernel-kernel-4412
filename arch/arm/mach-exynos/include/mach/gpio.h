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

#if defined(CONFIG_ARCH_EXYNOS4)
#include "gpio-exynos4.h"
#elif defined(CONFIG_ARCH_EXYNOS5)
#include "gpio-exynos5.h"
#else
#error "ARCH_EXYNOS* is not defined"
#endif

#endif /* __ASM_ARCH_GPIO_H */
