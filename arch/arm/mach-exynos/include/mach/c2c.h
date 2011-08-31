/* linux/arch/arm/mach-exynos/include/mach/c2c.h
 *
 * Copyright 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Platform header file for Samsung C2C Interface driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#ifndef __ASM_PLAT_C2C_H
#define __ASM_PLAT_C2C_H __FILE__

#include <linux/clk.h>

#define C2C_SHAREDMEM_BASE 0x60000000

enum c2c_buswidth {
	C2C_BUSWIDTH_8 = 0,
	C2C_BUSWIDTH_10 = 1,
	C2C_BUSWIDTH_16 = 2,
};

enum c2c_shrdmem_size {
	C2C_MEMSIZE_4 = 0,
	C2C_MEMSIZE_8 = 1,
	C2C_MEMSIZE_16 = 2,
	C2C_MEMSIZE_32 = 3,
	C2C_MEMSIZE_64 = 4,
	C2C_MEMSIZE_128 = 5,
	C2C_MEMSIZE_256 = 6,
	C2C_MEMSIZE_512 = 7,
};

struct exynos_c2c_platdata {
	void (*setup_gpio)(void);
	void (*set_cprst)(void);
	void (*clear_cprst)(void);
	u32 (*get_c2c_state)(void);

	u32 shdmem_addr;
	enum c2c_shrdmem_size shdmem_size;

	void __iomem *ap_sscm_addr;
	void __iomem  *cp_sscm_addr;

	enum c2c_buswidth rx_width;
	enum c2c_buswidth tx_width;
	u32 max_clk; /* Maximum clk */
	u32 default_clk; /* Default(init) clk */

	char *c2c_clk;
};

void exynos4_c2c_set_platdata(struct exynos_c2c_platdata *pd);
extern struct exynos_c2c_platdata smdk4212_c2c_pdata;
extern void exynos4_c2c_cfg_gpio(void);
extern void exynos4_c2c_set_cprst(void);
extern void exynos4_c2c_clear_cprst(void);
extern u32 exynos4_get_c2c_state(void);
#endif /*__ASM_PLAT_C2C_H */
