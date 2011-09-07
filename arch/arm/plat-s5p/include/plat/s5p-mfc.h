/* linux/arch/arm/plat-s5p/include/plat/s5p-mfc.h
 *
 * Copyright 2011 Samsung Electronics Co., Ltd.
 *      http://www.samsung.com/
 *
 * Header file for s5p mfc support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _S5P_MFC_H
#define _S5P_MFC_H

static inline void s5p_mfc_setname(char *name)
{
	s5p_device_mfc.name = name;
}
#endif /* _S5P_MFC_H */
