/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2010-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file mali_kbase_platform.c
 * Platform init.
 */

#include <osk/mali_osk.h>
#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_pm.h>
#include <kbase/src/common/mali_kbase_uku.h>
#include <kbase/src/common/mali_kbase_mem.h>
#include <kbase/src/common/mali_midg_regmap.h>
#include <kbase/src/linux/mali_kbase_mem_linux.h>
#include <uk/mali_ukk.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/miscdevice.h>
#include <linux/list.h>
#include <linux/semaphore.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include <mach/map.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <mach/regs-clock.h>
#include <asm/delay.h>
#include <kbase/src/platform/mali_kbase_runtime_pm.h>

static struct clk *mpll = NULL;
static struct clk *sclk_g3d = NULL;
#define VITHAR_DEFAULT_CLOCK 267000000
static int vithar_clock = VITHAR_DEFAULT_CLOCK;
#define EXYNOS5_PMU_G3D_CONF	    S5P_PMUREG(0x4060)
#define EXYNOS5_PMU_G3D_STATUS	    S5P_PMUREG(0x4064)

int kbase_platform_clock_on(struct device *dev)
{
    (void) clk_enable(sclk_g3d);

    clk_set_rate(sclk_g3d, vithar_clock);
    if(IS_ERR(sclk_g3d)) {
	dev_err(dev, "failed to clk_set_rate [sclk_g3d] = %d\n", vithar_clock);
	goto out;
    }

    return 0;
out:
	return -EPERM;
}

int kbase_platform_clock_off(struct device *dev)
{
    clk_disable(sclk_g3d);

    return 0;
}

static inline int kbase_platform_is_power_on(void)
{
    return ((__raw_readl(EXYNOS5_PMU_G3D_STATUS) & 0x7) == 0x7) ? 1 : 0;
}

int kbase_platform_power_on(struct device *dev)
{
#ifdef CONFIG_CPU_EXYNOS5210
    return 0;
#else
    int timeout;

    if(kbase_platform_is_power_on()) 
	return 0;

    /* Turn on G3D  */
    __raw_writel(0x7, EXYNOS5_PMU_G3D_CONF);
    
    /* Wait for G3D power stability for 1ms */
    timeout = 10;

    while((__raw_readl(EXYNOS5_PMU_G3D_STATUS) & 0x7) != 0x7) {
	if(timeout == 0) {
	    /* need to call panic  */
	    dev_err(dev, "failed to turn on g3d via g3d_configuration\n");
	    goto out;
	}
	timeout--;
	udelay(100);
    }

    return 0;
out:
    return -ETIMEDOUT;
#endif
}

int kbase_platform_power_off(struct device *dev)
{
#ifdef CONFIG_CPU_EXYNOS5210
    return 0;
#else
    int timeout;

    /* Turn off G3D  */
    __raw_writel(0x0, EXYNOS5_PMU_G3D_CONF);
    
    /* Wait for G3D power stability for 1ms */
    timeout = 10;

    while(__raw_readl(EXYNOS5_PMU_G3D_STATUS) & 0x7) {
	if(timeout == 0) {
	    /* need to call panic  */
	    dev_err(dev, "failed to turn off g3d via g3d_configuration\n");
	    goto out;
	}
	timeout--;
	udelay(100);
    }

    return 0;
out:
    return -ETIMEDOUT;
#endif
}

static ssize_t show_clock(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev;
	ssize_t ret = 0;
	unsigned int clkrate;

	kbdev = dev_get_drvdata(dev);

	if (!kbdev)
		return -ENODEV;

	if(!sclk_g3d)
		return -ENODEV;

	clkrate = clk_get_rate(sclk_g3d);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "Current sclk_g3d[G3D_BLK] = %dMhz", clkrate/1000000);

	/* To be revised  */
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "\nPossible settings : 400, 266, 200, 100, 50Mhz");

	if (ret < PAGE_SIZE - 1)
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "\n");
	else
	{
		buf[PAGE_SIZE-2] = '\n';
		buf[PAGE_SIZE-1] = '\0';
		ret = PAGE_SIZE-1;
	}

	return ret;
}

static ssize_t set_clock(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct kbase_device *kbdev;
	unsigned int tmp = 0;
	unsigned int cmd = 0;
	kbdev = dev_get_drvdata(dev);

	if (!kbdev)
		return -ENODEV;

	if(!sclk_g3d)
		return -ENODEV;

	if (sysfs_streq("400", buf)) {
	    cmd = 1;
	    clk_set_rate(sclk_g3d, 400000000);
	} else if (sysfs_streq("266", buf)) {
	    cmd = 1;
	    clk_set_rate(sclk_g3d, 267000000);
	} else if (sysfs_streq("200", buf)) {
	    cmd = 1;
	    clk_set_rate(sclk_g3d, VITHAR_DEFAULT_CLOCK);
	} else if (sysfs_streq("100", buf)) {
	    cmd = 1;
	    clk_set_rate(sclk_g3d, 100000000);
	} else if (sysfs_streq("50", buf)) {
	    cmd = 1;
	    clk_set_rate(sclk_g3d, 50000000);
	} else if (sysfs_streq("on", buf)) {
	    cmd = 2;
	    kbase_platform_power_on(dev);
	    clk_enable(sclk_g3d);
	    kbase_pm_send_event(kbdev, KBASE_PM_EVENT_RESUME);
	    kbase_pm_wait_for_power_up(kbdev);
	} else if (sysfs_streq("off", buf)) {
	    cmd = 2;
	    kbase_pm_send_event(kbdev, KBASE_PM_EVENT_SUSPEND);
	    kbase_pm_wait_for_power_down(kbdev);
	    clk_disable(sclk_g3d);
	    kbase_platform_power_off(dev);
	} else {
	    dev_err(dev, "set_clock: invalid value\n");
	    return -ENOENT;
	}

	if(cmd == 1) {
	    /* Waiting for clock is stable */
	    do {
		tmp = __raw_readl(EXYNOS5_CLKDIV_STAT_TOP0);
	    } while (tmp & 0x1000000);
	}
	else if(cmd == 2) {
	    /* Do we need to check */
	}

	return count;
}

static ssize_t show_fbdev(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev;
	ssize_t ret = 0;
	int i;

	kbdev = dev_get_drvdata(dev);

	if (!kbdev)
		return -ENODEV;

	for(i = 0 ; i < num_registered_fb ; i++) {
	    ret += snprintf(buf+ret, PAGE_SIZE-ret, "fb[%d] xres=%d, yres=%d, addr=0x%lx\n", i, registered_fb[i]->var.xres, registered_fb[i]->var.yres, registered_fb[i]->fix.smem_start);
	}

	if (ret < PAGE_SIZE - 1)
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "\n");
	else
	{
		buf[PAGE_SIZE-2] = '\n';
		buf[PAGE_SIZE-1] = '\0';
		ret = PAGE_SIZE-1;
	}

	return ret;
} 

/** The sysfs file @c clock, fbdev.
 *
 * This is used for obtaining information about the vithar operating clock & framebuffer address,
 */
DEVICE_ATTR(clock, S_IRUGO|S_IWUSR, show_clock, set_clock);
DEVICE_ATTR(fbdev, S_IRUGO, show_fbdev, NULL);

int kbase_platform_create_sysfs_file(struct device *dev)
{
	if (device_create_file(dev, &dev_attr_clock))
	{
		dev_err(dev, "Couldn't create sysfs file [clock]\n");
		goto out;
	}

	if (device_create_file(dev, &dev_attr_fbdev))
	{
		dev_err(dev, "Couldn't create sysfs file [fbdev]\n");
		goto out;
	}

	return 0;
out:
	return -ENOENT;
}

void kbase_platform_remove_sysfs_file(struct device *dev)
{
	device_remove_file(dev, &dev_attr_clock);
	device_remove_file(dev, &dev_attr_fbdev);
}

int kbase_platform_init(struct device *dev)
{
	mpll = clk_get(dev, "mout_mpll_user");
	if(IS_ERR(mpll)) {
	    goto out;
	}

	sclk_g3d = clk_get(dev, "sclk_g3d");
	if(IS_ERR(sclk_g3d)) {
	    goto out;
	}

	clk_set_parent(sclk_g3d, mpll);
	if(IS_ERR(sclk_g3d)) {
	    goto out;
	}

	if(kbase_platform_power_on(dev))
	    goto out;

	if(kbase_platform_clock_on(dev))
	    goto out;

#ifdef CONFIG_VITHAR_RT_PM
	kbase_device_runtime_init(dev);
#endif

	return 0;
out:
	return -ENOENT;
}
