/*
 * Copyright (C) 2010 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/**
 * @file mali_platform_dvfs.c
 * Platform specific Mali driver dvfs functions
 */

#include "mali_kernel_common.h"
#include "mali_osk.h"
#include "mali_platform.h"

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>

#include <asm/io.h>

#include "mali_device_pause_resume.h"
#include <linux/workqueue.h>

#define MALI_DVFS_STEPS 3
#define MAX_MALI_DVFS_STEPS 4
#define MALI_DVFS_WATING 10 // msec

#ifdef CONFIG_CPU_FREQ
#include <mach/asv.h>
#define EXYNOS4_ASV_ENABLED
#endif

static int bMaliDvfsRun=0;

typedef struct mali_dvfs_tableTag{
	unsigned int clock;
	unsigned int freq;
	unsigned int vol;
}mali_dvfs_table;

typedef struct mali_dvfs_statusTag{
	unsigned int currentStep;
	mali_dvfs_table * pCurrentDvfs;

}mali_dvfs_currentstatus;

typedef struct mali_dvfs_thresholdTag{
	unsigned int downthreshold;
	unsigned int upthreshold;
}mali_dvfs_threshold_table;

typedef struct mali_dvfs_staycount{
	unsigned int staycount;
}mali_dvfs_staycount_table;

mali_dvfs_staycount_table mali_dvfs_staycount[MALI_DVFS_STEPS]={
	/*step 0*/{1},
	/*step 1*/{1},
	/*step 2*/{1} };

/* dvfs information */
// L0 = 440Mhz, 1.00V
// L1 = 350Mhz, 0.95V
// L2 = 266Mhz, 0.90V
// L3 = 160Mhz, 0.90V

int step0_clk = 266;
int step0_vol = 900000;
int step1_clk = 350;
int step1_vol = 950000;
int step2_clk = 440;
int step2_vol = 1000000;
int step0_up = 70;
int step1_up = 70;
int step1_down = 30;
int step2_down = 70;

mali_dvfs_table mali_dvfs_all[MAX_MALI_DVFS_STEPS]={
	{440   ,1000000   , 1000000},
	{350   ,1000000   ,  950000},
	{266   ,1000000   ,  900000},
	{160   ,1000000   ,  900000} };

mali_dvfs_table mali_dvfs[MALI_DVFS_STEPS]={
	{266   ,1000000   , 900000},
	{350   ,1000000   , 950000},
	{440   ,1000000   ,1000000} };

mali_dvfs_threshold_table mali_dvfs_threshold[MALI_DVFS_STEPS]={
	{0   , 70},
	{50  , 85},
	{75  ,100} };

#ifdef EXYNOS4_ASV_ENABLED
#define ASV_9_LEVEL     9

static unsigned int asv_3d_volt_9_table[ASV_9_LEVEL][MALI_DVFS_STEPS] = {
	/* L2(266MHz)), L1(350MHz), L0(440Mhz) */
	{ 950000,  975000,  1050000},	/* SS */
	{ 925000,  975000,  1025000},	/* A1 */
	{ 925000,  950000,  1025000},	/* A2 */
	{ 900000,  950000,  1025000},	/* B1 */
	{ 900000,  950000,  1000000},	/* B2 */
	{ 900000,  950000,  1000000},	/* C1 */
	{ 900000,  925000,  1000000},	/* C2 */
	{ 900000,  925000,  1000000},	/* D1 */
	{ 900000,  925000,  1000000},	/* D2 */
};
#endif

/*dvfs status*/
mali_dvfs_currentstatus maliDvfsStatus;
int mali_dvfs_control=0;

static u32 mali_dvfs_utilization = 400;

static void mali_dvfs_work_handler(struct work_struct *w);

static struct workqueue_struct *mali_dvfs_wq = 0;
extern mali_io_address clk_register_map;

static DECLARE_WORK(mali_dvfs_work, mali_dvfs_work_handler);

static unsigned int get_mali_dvfs_status(void)
{
	return maliDvfsStatus.currentStep;
}

static mali_bool set_mali_dvfs_status(u32 step,mali_bool boostup)
{
	u32 validatedStep=step;

#ifdef CONFIG_REGULATOR
	if (mali_regulator_get_usecount() == 0) {
		MALI_DEBUG_PRINT(1, ("regulator use_count is 0 \n"));
		return MALI_FALSE;
	}
#endif

	if (boostup) {
#ifdef CONFIG_REGULATOR
		/*change the voltage*/
		mali_regulator_set_voltage(mali_dvfs[step].vol, mali_dvfs[step].vol);
#endif
		/*change the clock*/
		mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);
	} else {
		/*change the clock*/
		mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);
#ifdef CONFIG_REGULATOR
		/*change the voltage*/
		mali_regulator_set_voltage(mali_dvfs[step].vol, mali_dvfs[step].vol);
#endif
	}

	maliDvfsStatus.currentStep = validatedStep;
	/*for future use*/
	maliDvfsStatus.pCurrentDvfs = &mali_dvfs[validatedStep];

	return MALI_TRUE;
}

static void mali_platform_wating(u32 msec)
{
	/*sample wating
	change this in the future with proper check routine.
	*/
	unsigned int read_val;
	while(1) {
		read_val = _mali_osk_mem_ioread32(clk_register_map, 0x00);
		if ((read_val & 0x8000)==0x0000) break;
			_mali_osk_time_ubusydelay(100); // 1000 -> 100 : 20101218
		}
		/* _mali_osk_time_ubusydelay(msec*1000);*/
}

static mali_bool change_mali_dvfs_status(u32 step, mali_bool boostup )
{

	MALI_DEBUG_PRINT(1, ("> change_mali_dvfs_status: %d, %d \n",step, boostup));

	if (!set_mali_dvfs_status(step, boostup)) {
		MALI_DEBUG_PRINT(1, ("error on set_mali_dvfs_status: %d, %d \n",step, boostup));
		return MALI_FALSE;
	}

	/*wait until clock and voltage is stablized*/
	mali_platform_wating(MALI_DVFS_WATING); /*msec*/

	return MALI_TRUE;
}

#ifdef EXYNOS4_ASV_ENABLED
extern unsigned int exynos_result_of_asv;

static mali_bool mali_dvfs_table_update(void)
{
	unsigned int i;

	for (i = 0; i < MALI_DVFS_STEPS; i++) {
		MALI_PRINT((":::exynos_result_of_asv : %d\n", exynos_result_of_asv));
		mali_dvfs[i].vol = asv_3d_volt_9_table[exynos_result_of_asv][i];
		MALI_PRINT(("mali_dvfs[%d].vol = %d\n", i, mali_dvfs[i].vol));
	}

	return MALI_TRUE;
}
#endif

static unsigned int decideNextStatus(unsigned int utilization)
{
	unsigned int level = 0; // 0:stay, 1:up
	static int mali_dvfs_clk = 0;

	if (!mali_dvfs_control) {
		switch (maliDvfsStatus.currentStep) {
		case 0:
			if (utilization > (int)(400 * mali_dvfs_threshold[maliDvfsStatus.currentStep].upthreshold / 100))
				level = 1;
			else
				level = maliDvfsStatus.currentStep;
			break;
		case 1:
			if (utilization > (int)(400 * mali_dvfs_threshold[maliDvfsStatus.currentStep].upthreshold / 100))
				level = 2;
			else if (utilization < (int)(400 * mali_dvfs_threshold[maliDvfsStatus.currentStep].downthreshold / 100))
				level = 0;
			else
				level = maliDvfsStatus.currentStep;
			break;
		case 2:
			if (utilization < (int)(400 * mali_dvfs_threshold[maliDvfsStatus.currentStep].downthreshold / 100))
				level = 1;
			else
				level = maliDvfsStatus.currentStep;
			break;
		}
	} else if (mali_dvfs_control == 999) {
		step0_clk = mali_dvfs_all[2].clock;
		step1_clk = mali_dvfs_all[1].clock;
		step2_clk = mali_dvfs_all[0].clock;
#ifdef EXYNOS4_ASV_ENABLED
		mali_dvfs_table_update();
#endif
		mali_dvfs[0].clock = step0_clk;
		mali_dvfs[1].clock = step1_clk;
		mali_dvfs[2].clock = step2_clk;

		mali_dvfs_control = 0;
		level = !(maliDvfsStatus.currentStep);
	} else if (mali_dvfs_control != mali_dvfs_clk) {
		if (mali_dvfs_control < mali_dvfs_all[2].clock && mali_dvfs_control > 0) {
			step0_clk = mali_dvfs_all[3].clock;
			step1_clk = mali_dvfs_all[3].clock;
			step2_clk = mali_dvfs_all[3].clock;
		} else if (mali_dvfs_control < mali_dvfs_all[1].clock && mali_dvfs_control >= mali_dvfs_all[2].clock) {
			step0_clk = mali_dvfs_all[2].clock;
			step1_clk = mali_dvfs_all[2].clock;
			step2_clk = mali_dvfs_all[2].clock;
		} else if (mali_dvfs_control < mali_dvfs_all[0].clock && mali_dvfs_control >= mali_dvfs_all[1].clock) {
			step0_clk = mali_dvfs_all[1].clock;
			step1_clk = mali_dvfs_all[1].clock;
			step2_clk = mali_dvfs_all[1].clock;
		} else {
			step0_clk = mali_dvfs_all[0].clock;
			step1_clk = mali_dvfs_all[0].clock;
			step2_clk = mali_dvfs_all[0].clock;
		}

		change_dvfs_tableset(step0_clk, 0);
		change_dvfs_tableset(step1_clk, 1);
		change_dvfs_tableset(step2_clk, 2);
		level = maliDvfsStatus.currentStep;
	}

	mali_dvfs_clk = mali_dvfs_control;
	return level;
}

static mali_bool mali_dvfs_status(u32 utilization)
{
	unsigned int nextStatus = 0;
	unsigned int curStatus = 0;
	mali_bool boostup = MALI_FALSE;
	static int stay_count = 0;
#ifdef EXYNOS4_ASV_ENABLED
	static mali_bool asv_applied = MALI_FALSE;
#endif

	MALI_DEBUG_PRINT(1, ("> mali_dvfs_status: %d \n",utilization));
#ifdef EXYNOS4_ASV_ENABLED
	if (asv_applied == MALI_FALSE) {
		mali_dvfs_table_update();
		change_mali_dvfs_status(0, 0);
		asv_applied = MALI_TRUE;

		return MALI_TRUE;
	}
#endif

	/*decide next step*/
	curStatus = get_mali_dvfs_status();
	nextStatus = decideNextStatus(utilization);

	MALI_DEBUG_PRINT(1, ("= curStatus %d, nextStatus %d, maliDvfsStatus.currentStep %d \n", curStatus, nextStatus, maliDvfsStatus.currentStep));

	/*if next status is same with current status, don't change anything*/
	if ((curStatus != nextStatus && stay_count == 0)) {
		/*check if boost up or not*/
		if (nextStatus > maliDvfsStatus.currentStep) boostup = 1;

		/*change mali dvfs status*/
		if (!change_mali_dvfs_status(nextStatus,boostup)) {
			MALI_DEBUG_PRINT(1, ("error on change_mali_dvfs_status \n"));
			return MALI_FALSE;
		}
		stay_count = mali_dvfs_staycount[maliDvfsStatus.currentStep].staycount;
	} else {
		if (stay_count > 0)
			stay_count--;
	}

	return MALI_TRUE;
}



int mali_dvfs_is_running(void)
{
	return bMaliDvfsRun;
}



void mali_dvfs_late_resume(void)
{
	// set the init clock as low when resume
	set_mali_dvfs_status(0,0);
}


static void mali_dvfs_work_handler(struct work_struct *w)
{
	int change_clk = 0;
	int change_step = 0;
	bMaliDvfsRun=1;

	/* dvfs table change when step0_clk or step1_clk changed only */
	if (step0_clk != mali_dvfs[0].clock) {
		MALI_PRINT(("::: step0_clk change to %d Mhz\n", step0_clk));
		change_clk = step0_clk;
		change_step = 0;
		step0_clk = change_dvfs_tableset(change_clk, change_step);
	}
	if (step1_clk != mali_dvfs[1].clock) {
		MALI_PRINT(("::: step1_clk change to %d Mhz\n", step1_clk));
		change_clk = step1_clk;
		change_step = 1;
		step1_clk = change_dvfs_tableset(change_clk, change_step);
	}
	if (step2_clk != mali_dvfs[2].clock) {
		MALI_PRINT(("::: step2_clk change to %d Mhz\n", step2_clk));
		change_clk = step2_clk;
		change_step = 2;
		step2_clk = change_dvfs_tableset(change_clk, change_step);
	}
	if (step0_up != mali_dvfs_threshold[0].upthreshold) {
		MALI_PRINT(("::: step0_up change to %d %\n", step0_up));
		mali_dvfs_threshold[0].upthreshold = step0_up;
	}
	if (step1_down != mali_dvfs_threshold[1].downthreshold) {
		MALI_PRINT((":::step1_down change to %d %\n", step1_down));
		mali_dvfs_threshold[1].downthreshold = step1_down;
	}
	if (step1_up != mali_dvfs_threshold[1].upthreshold) {
		MALI_PRINT((":::step1_up change to %d %\n", step1_up));
		mali_dvfs_threshold[1].upthreshold = step1_up;
	}
	if (step2_down != mali_dvfs_threshold[2].downthreshold) {
		MALI_PRINT((":::step2_down change to %d %\n", step2_down));
		mali_dvfs_threshold[2].downthreshold = step2_down;
	}

#ifdef DEBUG
	mali_dvfs[0].vol = step0_vol;
	mali_dvfs[1].vol = step1_vol;
	mali_dvfs[2].vol = step2_vol;
#endif
	MALI_DEBUG_PRINT(3, ("=== mali_dvfs_work_handler\n"));

	if (!mali_dvfs_status(mali_dvfs_utilization))
		MALI_DEBUG_PRINT(1,( "error on mali dvfs status in mali_dvfs_work_handler"));

	bMaliDvfsRun=0;
}

mali_bool init_mali_dvfs_status(int step)
{
	/*default status
	add here with the right function to get initilization value.
	*/
	if (!mali_dvfs_wq)
		mali_dvfs_wq = create_singlethread_workqueue("mali_dvfs");

	/*add a error handling here*/
	maliDvfsStatus.currentStep = step;
	return MALI_TRUE;
}

void deinit_mali_dvfs_status(void)
{
	if (mali_dvfs_wq)
		destroy_workqueue(mali_dvfs_wq);
	mali_dvfs_wq = NULL;
}

mali_bool mali_dvfs_handler(u32 utilization)
{
	mali_dvfs_utilization = utilization;
	queue_work_on(0, mali_dvfs_wq,&mali_dvfs_work);

	/*add error handle here*/
	return MALI_TRUE;
}

int change_dvfs_tableset(int change_clk, int change_step)
{
	if (change_clk < mali_dvfs_all[2].clock) {
		mali_dvfs[change_step].clock = mali_dvfs_all[3].clock;
		mali_dvfs[change_step].vol = mali_dvfs_all[3].vol;
	} else if (change_clk < mali_dvfs_all[1].clock && change_clk >= mali_dvfs_all[2].clock) {
		mali_dvfs[change_step].clock = mali_dvfs_all[2].clock;
		mali_dvfs[change_step].vol = mali_dvfs_all[2].vol;
	} else if (change_clk < mali_dvfs_all[0].clock && change_clk >= mali_dvfs_all[1].clock) {
		mali_dvfs[change_step].clock = mali_dvfs_all[1].clock;
		mali_dvfs[change_step].vol = mali_dvfs_all[1].vol;
	} else {
		mali_dvfs[change_step].clock = mali_dvfs_all[0].clock;
		mali_dvfs[change_step].vol = mali_dvfs_all[0].vol;
	}

	MALI_PRINT((":::mali dvfs step %d clock and voltage = %d Mhz, %d V\n",change_step, mali_dvfs[change_step].clock, mali_dvfs[change_step].vol));

	if (maliDvfsStatus.currentStep == change_step) {
#ifdef CONFIG_REGULATOR
		/*change the voltage*/
		mali_regulator_set_voltage(mali_dvfs[change_step].vol, mali_dvfs[change_step].vol);
#endif
		/*change the clock*/
		mali_clk_set_rate(mali_dvfs[change_step].clock, mali_dvfs[change_step].freq);
	}

	return mali_dvfs[change_step].clock;
}

void mali_default_step_set(int step, mali_bool boostup)
{
	mali_clk_set_rate(mali_dvfs[step].clock, mali_dvfs[step].freq);

	if (maliDvfsStatus.currentStep == 1)
		set_mali_dvfs_status(step, boostup);
}
