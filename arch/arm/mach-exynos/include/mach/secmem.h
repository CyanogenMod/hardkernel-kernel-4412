/* linux/arch/arm/mach-exynos/include/mach/secmem.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS - Secure memory support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_SECMEM_H
#define __ASM_ARCH_SECMEM_H __FILE__

#include <linux/miscdevice.h>

struct secchunk_info {
	int		index;
	phys_addr_t	base;
	size_t		size;
};

extern struct miscdevice secmem;

#define SECMEM_IOC_CHUNKINFO	_IOWR('S', 1, struct secchunk_info)
#define SECMEM_IOC_DRM_ONOFF	_IOWR('S', 2, int)

#endif /* __ASM_ARCH_SECMEM_H */
