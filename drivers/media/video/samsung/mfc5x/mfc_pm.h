/*
 * linux/drivers/media/video/samsung/mfc5x/mfc_pm.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Power management module for Samsung MFC (Multi Function Codec - FIMV) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MFC_PM_H
#define __MFC_PM_H __FILE__

#define CPU_LOCK_FREQ	500000

int mfc_init_pm(struct mfc_dev *mfcdev);
void mfc_final_pm(struct mfc_dev *mfcdev);

int mfc_clock_on(void);
void mfc_clock_off(void);
int mfc_power_on(void);
int mfc_power_off(void);
#ifdef CONFIG_CPU_EXYNOS4210
bool mfc_power_chk(void);
#endif
void mfc_pd_enable(void);

#ifdef CONFIG_CPU_FREQ
int mfc_cpufreq_lock(unsigned int freq);
void mfc_cpufreq_lock_free(void);
#endif

#endif /* __MFC_PM_H */
