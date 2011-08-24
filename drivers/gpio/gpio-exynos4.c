/* linux/arch/arm/mach-exynos4/gpiolib.c
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4 - GPIOlib support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <mach/map.h>

#include <plat/gpio-core.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-cfg-helpers.h>
#include <plat/cputype.h>

int s3c_gpio_setpull_exynos4(struct s3c_gpio_chip *chip,
				unsigned int off, s3c_gpio_pull_t pull)
{
	if (pull == S3C_GPIO_PULL_UP)
		pull = 3;

	return s3c_gpio_setpull_updown(chip, off, pull);
}

s3c_gpio_pull_t s3c_gpio_getpull_exynos4(struct s3c_gpio_chip *chip,
						unsigned int off)
{
	s3c_gpio_pull_t pull;

	pull = s3c_gpio_getpull_updown(chip, off);
	if (pull == 3)
		pull = S3C_GPIO_PULL_UP;

	return pull;
}

static struct s3c_gpio_cfg gpio_cfg = {
	.set_config	= s3c_gpio_setcfg_s3c64xx_4bit,
	.set_pull	= s3c_gpio_setpull_exynos4,
	.get_pull	= s3c_gpio_getpull_exynos4,
};

static struct s3c_gpio_cfg gpio_cfg_noint = {
	.set_config	= s3c_gpio_setcfg_s3c64xx_4bit,
	.set_pull	= s3c_gpio_setpull_exynos4,
	.get_pull	= s3c_gpio_getpull_exynos4,
};

/*
 * Following are the gpio banks in v310.
 *
 * The 'config' member when left to NULL, is initialized to the default
 * structure gpio_cfg in the init function below.
 *
 * The 'base' member is also initialized in the init function below.
 * Note: The initialization of 'base' member of s3c_gpio_chip structure
 * uses the above macro and depends on the banks being listed in order here.
 */
static struct s3c_gpio_chip exynos4_gpio_common_4bit[] = {
	{
		.base	= S5P_VA_GPIO1,
		.chip	= {
			.base	= EXYNOS4_GPA0(0),
			.ngpio	= EXYNOS4_GPIO_A0_NR,
			.label	= "GPA0",
		},
	}, {
		.base	= (S5P_VA_GPIO1 + 0x20),
		.chip	= {
			.base	= EXYNOS4_GPA1(0),
			.ngpio	= EXYNOS4_GPIO_A1_NR,
			.label	= "GPA1",
		},
	}, {
		.base	= (S5P_VA_GPIO1 + 0x40),
		.chip	= {
			.base	= EXYNOS4_GPB(0),
			.ngpio	= EXYNOS4_GPIO_B_NR,
			.label	= "GPB",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0x60),
		.chip	= {
			.base	= EXYNOS4_GPC0(0),
			.ngpio	= EXYNOS4_GPIO_C0_NR,
			.label	= "GPC0",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0x80),
		.chip	= {
			.base	= EXYNOS4_GPC1(0),
			.ngpio	= EXYNOS4_GPIO_C1_NR,
			.label	= "GPC1",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0xA0),
		.chip	= {
			.base	= EXYNOS4_GPD0(0),
			.ngpio	= EXYNOS4_GPIO_D0_NR,
			.label	= "GPD0",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0xC0),
		.chip	= {
			.base	= EXYNOS4_GPD1(0),
			.ngpio	= EXYNOS4_GPIO_D1_NR,
			.label	= "GPD1",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0x180),
		.chip	= {
			.base	= EXYNOS4_GPF0(0),
			.ngpio	= EXYNOS4_GPIO_F0_NR,
			.label	= "GPF0",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0x1A0),
		.chip	= {
			.base	= EXYNOS4_GPF1(0),
			.ngpio	= EXYNOS4_GPIO_F1_NR,
			.label	= "GPF1",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0x1C0),
		.chip	= {
			.base	= EXYNOS4_GPF2(0),
			.ngpio	= EXYNOS4_GPIO_F2_NR,
			.label	= "GPF2",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0x1E0),
		.chip	= {
			.base	= EXYNOS4_GPF3(0),
			.ngpio	= EXYNOS4_GPIO_F3_NR,
			.label	= "GPF3",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x40),
		.chip	= {
			.base	= EXYNOS4_GPK0(0),
			.ngpio	= EXYNOS4_GPIO_K0_NR,
			.label	= "GPK0",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x60),
		.chip	= {
			.base	= EXYNOS4_GPK1(0),
			.ngpio	= EXYNOS4_GPIO_K1_NR,
			.label	= "GPK1",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x80),
		.chip	= {
			.base	= EXYNOS4_GPK2(0),
			.ngpio	= EXYNOS4_GPIO_K2_NR,
			.label	= "GPK2",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0xA0),
		.chip	= {
			.base	= EXYNOS4_GPK3(0),
			.ngpio	= EXYNOS4_GPIO_K3_NR,
			.label	= "GPK3",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0xC0),
		.chip	= {
			.base	= EXYNOS4_GPL0(0),
			.ngpio	= EXYNOS4_GPIO_L0_NR,
			.label	= "GPL0",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0xE0),
		.chip	= {
			.base	= EXYNOS4_GPL1(0),
			.ngpio	= EXYNOS4_GPIO_L1_NR,
			.label	= "GPL1",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x100),
		.chip	= {
			.base	= EXYNOS4_GPL2(0),
			.ngpio	= EXYNOS4_GPIO_L2_NR,
			.label	= "GPL2",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x120),
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= EXYNOS4_GPY0(0),
			.ngpio	= EXYNOS4_GPIO_Y0_NR,
			.label	= "GPY0",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x140),
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= EXYNOS4_GPY1(0),
			.ngpio	= EXYNOS4_GPIO_Y1_NR,
			.label	= "GPY1",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x160),
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= EXYNOS4_GPY2(0),
			.ngpio	= EXYNOS4_GPIO_Y2_NR,
			.label	= "GPY2",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x180),
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= EXYNOS4_GPY3(0),
			.ngpio	= EXYNOS4_GPIO_Y3_NR,
			.label	= "GPY3",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x1A0),
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= EXYNOS4_GPY4(0),
			.ngpio	= EXYNOS4_GPIO_Y4_NR,
			.label	= "GPY4",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x1C0),
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= EXYNOS4_GPY5(0),
			.ngpio	= EXYNOS4_GPIO_Y5_NR,
			.label	= "GPY5",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x1E0),
		.config	= &gpio_cfg_noint,
		.chip	= {
			.base	= EXYNOS4_GPY6(0),
			.ngpio	= EXYNOS4_GPIO_Y6_NR,
			.label	= "GPY6",
		},
	}, {
		.base	= (S5P_VA_GPIO2 + 0xC00),
		.config	= &gpio_cfg_noint,
		.irq_base = IRQ_EINT(0),
		.chip	= {
			.base	= EXYNOS4_GPX0(0),
			.ngpio	= EXYNOS4_GPIO_X0_NR,
			.label	= "GPX0",
			.to_irq	= samsung_gpiolib_to_irq,
		},
	}, {
		.base	= (S5P_VA_GPIO2 + 0xC20),
		.config	= &gpio_cfg_noint,
		.irq_base = IRQ_EINT(8),
		.chip	= {
			.base	= EXYNOS4_GPX1(0),
			.ngpio	= EXYNOS4_GPIO_X1_NR,
			.label	= "GPX1",
			.to_irq	= samsung_gpiolib_to_irq,
		},
	}, {
		.base	= (S5P_VA_GPIO2 + 0xC40),
		.config	= &gpio_cfg_noint,
		.irq_base = IRQ_EINT(16),
		.chip	= {
			.base	= EXYNOS4_GPX2(0),
			.ngpio	= EXYNOS4_GPIO_X2_NR,
			.label	= "GPX2",
			.to_irq	= samsung_gpiolib_to_irq,
		},
	}, {
		.base	= (S5P_VA_GPIO2 + 0xC60),
		.config	= &gpio_cfg_noint,
		.irq_base = IRQ_EINT(24),
		.chip	= {
			.base	= EXYNOS4_GPX3(0),
			.ngpio	= EXYNOS4_GPIO_X3_NR,
			.label	= "GPX3",
			.to_irq	= samsung_gpiolib_to_irq,
		},
	}, {
		.base   = S5P_VA_GPIO3,
		.chip	= {
			.base	= EXYNOS4_GPZ(0),
			.ngpio	= EXYNOS4_GPIO_Z_NR,
			.label	= "GPZ",
		},
	},
};

static struct s3c_gpio_chip exynos4210_gpio_4bit[] = {
	{
		.base   = (S5P_VA_GPIO1 + 0xE0),
		.chip	= {
			.base	= EXYNOS4210_GPE0(0),
			.ngpio	= EXYNOS4210_GPIO_E0_NR,
			.label	= "GPE0",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0x100),
		.chip	= {
			.base	= EXYNOS4210_GPE1(0),
			.ngpio	= EXYNOS4210_GPIO_E1_NR,
			.label	= "GPE1",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0x120),
		.chip	= {
			.base	= EXYNOS4210_GPE2(0),
			.ngpio	= EXYNOS4210_GPIO_E2_NR,
			.label	= "GPE2",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0x140),
		.chip	= {
			.base	= EXYNOS4210_GPE3(0),
			.ngpio	= EXYNOS4210_GPIO_E3_NR,
			.label	= "GPE3",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0x160),
		.chip	= {
			.base	= EXYNOS4210_GPE4(0),
			.ngpio	= EXYNOS4210_GPIO_E4_NR,
			.label	= "GPE4",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x0),
		.chip	= {
			.base	= EXYNOS4210_GPJ0(0),
			.ngpio	= EXYNOS4210_GPIO_J0_NR,
			.label	= "GPJ0",
		},
	}, {
		.base   = (S5P_VA_GPIO2 + 0x20),
		.chip	= {
			.base	= EXYNOS4210_GPJ1(0),
			.ngpio	= EXYNOS4210_GPIO_J1_NR,
			.label	= "GPJ1",
		},
	},
};

static struct s3c_gpio_chip exynos4212_gpio_4bit[] = {
	{
		.base   = (S5P_VA_GPIO1 + 0x240),
		.chip	= {
			.base	= EXYNOS4212_GPJ0(0),
			.ngpio	= EXYNOS4212_GPIO_J0_NR,
			.label	= "GPJ0",
		},
	}, {
		.base   = (S5P_VA_GPIO1 + 0x260),
		.chip	= {
			.base	= EXYNOS4212_GPJ1(0),
			.ngpio	= EXYNOS4212_GPIO_J1_NR,
			.label	= "GPJ1",
		}
	}, {
		.base	= (S5P_VA_GPIO2 + 0x260),
		.chip	= {
			.base	= EXYNOS4212_GPM0(0),
			.ngpio	= EXYNOS4212_GPIO_M0_NR,
			.label	= "GPM0",
		},
	}, {
		.base	= (S5P_VA_GPIO2 + 0x280),
		.chip	= {
			.base	= EXYNOS4212_GPM1(0),
			.ngpio	= EXYNOS4212_GPIO_M1_NR,
			.label	= "GPM1",
		},
	}, {
		.base	= (S5P_VA_GPIO2 + 0x2A0),
		.chip	= {
			.base	= EXYNOS4212_GPM2(0),
			.ngpio	= EXYNOS4212_GPIO_M2_NR,
			.label	= "GPM2",
		},
	}, {
		.base	= (S5P_VA_GPIO2 + 0x2C0),
		.chip	= {
			.base	= EXYNOS4212_GPM3(0),
			.ngpio	= EXYNOS4212_GPIO_M3_NR,
			.label	= "GPM3",
		},
	}, {
		.base	= (S5P_VA_GPIO2 + 0x2E0),
		.chip	= {
			.base	= EXYNOS4212_GPM4(0),
			.ngpio	= EXYNOS4212_GPIO_M4_NR,
			.label	= "GPM4",
		},
	}, {
		.base   = (S5P_VA_GPIO4 + 0x0),
		.chip	= {
			.base	= EXYNOS4212_GPV0(0),
			.ngpio	= EXYNOS4212_GPIO_V0_NR,
			.label	= "GPV0",
		},
	}, {
		.base   = (S5P_VA_GPIO4 + 0x20),
		.chip	= {
			.base	= EXYNOS4212_GPV1(0),
			.ngpio	= EXYNOS4212_GPIO_V1_NR,
			.label	= "GPV1",
		},
	}, {
		.base   = (S5P_VA_GPIO4 + 0x60),
		.chip	= {
			.base	= EXYNOS4212_GPV2(0),
			.ngpio	= EXYNOS4212_GPIO_V2_NR,
			.label	= "GPV2",
		},
	}, {
		.base   = (S5P_VA_GPIO4 + 0x80),
		.chip	= {
			.base	= EXYNOS4212_GPV3(0),
			.ngpio	= EXYNOS4212_GPIO_V3_NR,
			.label	= "GPV3",
		},
	}, {
		.base   = (S5P_VA_GPIO4 + 0xC0),
		.chip	= {
			.base	= EXYNOS4212_GPV4(0),
			.ngpio	= EXYNOS4212_GPIO_V4_NR,
			.label	= "GPV4",
		},
	},
};

static __init int exynos4_gpiolib_init(void)
{
	struct s3c_gpio_chip *chip;
	int i;
	int group = 0;
	int nr_chips;

	/* GPIO common part  */

	chip = exynos4_gpio_common_4bit;
	nr_chips = ARRAY_SIZE(exynos4_gpio_common_4bit);

	for (i = 0; i < nr_chips; i++, chip++) {
		if (chip->config == NULL) {
			chip->config = &gpio_cfg;
			/* Assign the GPIO interrupt group */
			chip->group = group++;
		}
		if (chip->base == NULL)
			pr_err("No allocation of base address for [common gpio]");
	}

	samsung_gpiolib_add_4bit_chips(exynos4_gpio_common_4bit, nr_chips);

	/* Only 4210 GPIO  part */
	if (cpu_is_exynos4210()) {
		chip = exynos4210_gpio_4bit;
		nr_chips = ARRAY_SIZE(exynos4210_gpio_4bit);

		for (i = 0; i < nr_chips; i++, chip++) {
			if (chip->config == NULL) {
				chip->config = &gpio_cfg;
				/* Assign the GPIO interrupt group */
				chip->group = group++;
			}
			if (chip->base == NULL)
				pr_err("No allocation of base address [4210 gpio]");
		}

		samsung_gpiolib_add_4bit_chips(exynos4210_gpio_4bit, nr_chips);
	} else if (cpu_is_exynos4212()) {
	/* Only 4212 GPIO part */
		chip = exynos4212_gpio_4bit;
		nr_chips = ARRAY_SIZE(exynos4212_gpio_4bit);

		for (i = 0; i < nr_chips; i++, chip++) {
			if (chip->config == NULL) {
				chip->config = &gpio_cfg;
				/* Assign the GPIO interrupt group */
				chip->group = group++;
			}
			if (chip->base == NULL)
				pr_err("No allocation of base address [4212 gpio]");
		}

		samsung_gpiolib_add_4bit_chips(exynos4212_gpio_4bit, nr_chips);
	}

	s5p_register_gpioint_bank(IRQ_GPIO_XA, 0, IRQ_GPIO1_NR_GROUPS);
	s5p_register_gpioint_bank(IRQ_GPIO_XB, IRQ_GPIO1_NR_GROUPS, IRQ_GPIO2_NR_GROUPS);

	return 0;
}
core_initcall(exynos4_gpiolib_init);
