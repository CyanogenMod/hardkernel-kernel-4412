/* linux/arch/arm/mach-exynos/cpu-exynos5.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/sched.h>
#include <linux/sysdev.h>

#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/proc-fns.h>

#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/fb-core.h>
#include <plat/exynos5.h>
#include <plat/sdhci.h>
#include <plat/pm.h>
#include <plat/iic-core.h>
#include <plat/tv-core.h>

#include <mach/regs-irq.h>

unsigned int gic_bank_offset __read_mostly;

extern int combiner_init(unsigned int combiner_nr, void __iomem *base,
			 unsigned int irq_start);
extern void combiner_cascade_irq(unsigned int combiner_nr, unsigned int irq);

/* Initial IO mappings */
static struct map_desc exynos5_iodesc[] __initdata = {
	{
		.virtual	= (unsigned long)S5P_VA_SYSTIMER,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_SYSTIMER),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual        = (unsigned long)S5P_VA_SYSRAM,
		.pfn            = __phys_to_pfn(EXYNOS5_PA_SYSRAM),
		.length         = SZ_4K,
		.type           = MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_CMU,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_CMU),
		.length		= 144 * SZ_1K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_PMU,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_PMU),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_COMBINER_BASE,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_COMBINER),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S3C_VA_UART,
		.pfn		= __phys_to_pfn(S3C_PA_UART),
		.length		= SZ_512K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_GIC_CPU,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_GIC_CPU),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_GIC_DIST,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_GIC_DIST),
		.length		= SZ_64K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_GPIO1,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_GPIO1),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_GPIO2,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_GPIO2),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_GPIO3,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_GPIO3),
		.length		= SZ_256,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_GPIO4,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_GPIO4),
		.length		= SZ_256,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (unsigned long)S5P_VA_AUDSS,
		.pfn		= __phys_to_pfn(EXYNOS5_PA_AUDSS),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
};

static void exynos5_idle(void)
{
	if (!need_resched())
		cpu_do_idle();

	local_irq_enable();
}

/*
 * exynos5_map_io
 *
 * register the standard cpu IO areas
 */
void __init exynos5_map_io(void)
{
	iotable_init(exynos5_iodesc, ARRAY_SIZE(exynos5_iodesc));

#ifdef CONFIG_S3C_DEV_HSMMC
	exynos5_default_sdhci0();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	exynos5_default_sdhci1();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	exynos5_default_sdhci2();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	exynos5_default_sdhci3();
#endif

	s5p_fb_setname(1, "exynos5-fb");        /* FIMD1 */

	s5p_hdmi_setname("exynos5-hdmi");

	/* The I2C bus controllers are directly compatible with s3c2440 */
	s3c_i2c0_setname("s3c2440-i2c");
	s3c_i2c1_setname("s3c2440-i2c");
	s3c_i2c2_setname("s3c2440-i2c");
}

void __init exynos5_init_clocks(int xtal)
{
	printk(KERN_DEBUG "%s: initializing clocks\n", __func__);

	s3c24xx_register_baseclocks(xtal);

	s5p_register_clocks(xtal);
	exynos5_register_clocks();
	exynos5_setup_clocks();
}

void __init exynos5_init_irq(void)
{
	int irq;

	gic_init(0, IRQ_PPI(0), S5P_VA_GIC_DIST, S5P_VA_GIC_CPU);

	for (irq = 0; irq < MAX_COMBINER_NR; irq++) {
		combiner_init(irq, (void __iomem *)S5P_VA_COMBINER(irq),
				COMBINER_IRQ(irq, 0));
		combiner_cascade_irq(irq, IRQ_SPI(irq));
	}

	/* The parameters of s5p_init_irq() are for VIC init.
	 * Theses parameters should be NULL and 0 because EXYNOS5
	 * uses GIC instead of VIC.
	 */
	s5p_init_irq(NULL, 0);
}

struct sysdev_class exynos5_sysclass = {
	.name	= "exynos5-core",
};

static struct sys_device exynos5_sysdev = {
	.cls	= &exynos5_sysclass,
};

static int __init exynos5_core_init(void)
{
	return sysdev_class_register(&exynos5_sysclass);
}

core_initcall(exynos5_core_init);

int __init exynos5_init(void)
{
	printk(KERN_INFO "EXYNOS5: Initializing architecture\n");

	/* set idle function */
	pm_idle = exynos5_idle;

	return sysdev_register(&exynos5_sysdev);
}
