# Copyright:
# ----------------------------------------------------------------------------
# This confidential and proprietary software may be used only as authorized 
# by a licensing agreement from ARM Limited.
#      (C) COPYRIGHT 2010-2011 ARM Limited, ALL RIGHTS RESERVED
# The entire notice above must be reproduced on all authorized copies and 
# copies may only be made to the extent permitted by a licensing agreement
# from ARM Limited.
# ----------------------------------------------------------------------------
#

#ifneq ($(KERNELRELEASE),)
ifneq ($(VITHAR_ROOT),)
SRC=\
	common/mali_kbase_device.c		\
	common/mali_kbase_mem.c			\
	common/mali_kbase_mmu.c			\
	common/mali_kbase_jd.c			\
	common/mali_kbase_jm.c			\
	common/mali_kbase_js.c			\
	common/mali_kbase_pm.c			\
	common/mali_kbase_event.c		\
	common/mali_kbase_context.c		\
	common/mali_kbase_pm.c			\
	common/mali_kbase_pm_driver.c		\
	common/mali_kbase_pm_always_on.c	\
	common/mali_kbase_pm_demand.c		\
	linux/mali_kbase_mem_linux.c		\
	linux/mali_kbase_core.c	

obj-$(CONFIG_VITHAR) += $(KBASE_PATH)/platform/mali_kbase_platform.o
obj-$(CONFIG_VITHAR_RT_PM) += $(KBASE_PATH)/platform/mali_kbase_runtime_pm.o
obj-$(CONFIG_VITHAR_RT_DVFS) += $(KBASE_PATH)/platform/mali_kbase_dvfs.o

SRC+=common/mali_kbase_js_policy_$(MALI_KBASE_SCHEDULING_POLICY).c

ifeq ($(MALI_BASE_TRACK_MEMLEAK), 1)
SRC += common/mali_kbase_mem_track.c
endif

RELATIVE_ROOT=../..
ROOT=$(KBUILD_EXTMOD)/$(RELATIVE_ROOT)
include $(KBUILD_EXTMOD)/Makefile.kbase
OSK_PATH = $(ROOT)/osk
include $(OSK_PATH)/src/linux/Makefile.osk

EXTRA_CFLAGS += -DMALI_DEBUG=$(MALI_DEBUG) -DMALI_HW_TYPE=$(MALI_HW_TYPE) -DMALI_KBASE_USE_UMP=$(MALI_KBASE_USE_UMP) -DMALI_ANDROID=$(MALI_ANDROID) -DMALI_BASE_TRACK_MEMLEAK=$(MALI_BASE_TRACK_MEMLEAK) -I$(ROOT)/kbase/midg_gpus/$(MALI_HW_VERSION)


#MODULE:=mali_kbase.ko
#obj-m := $(MODULE:.ko=.o)
#$(MODULE:.ko=-y) := $(SRC:.c=.o)
#$(MODULE:.ko=-objs) := $(SRC:.c=.o) $(RELATIVE_ROOT)/osk/src/linux/lib.a

else

KDIR ?= /lib/modules/$(shell uname -r)/build

# the mali_kbase module depends on the ukk module
# we get the symbols from the ukk module using KBUILD_EXTRA_SYMBOLS to prevent warnings about unknown functions
all:
	$(MAKE) -C $(KDIR) M=$(PWD) KBUILD_EXTRA_SYMBOLS="$(PWD)/../../uk/src/ukk/linux/Module.symvers $(PWD)/../../ump/src/devicedrv/Module.symvers" modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

endif
