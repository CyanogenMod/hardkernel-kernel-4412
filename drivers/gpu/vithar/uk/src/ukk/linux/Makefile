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

OS=linux
RELATIVE_ROOT=../../../..
ROOT = $(KBUILD_EXTMOD)/$(RELATIVE_ROOT)
OSK_PATH = $(ROOT)/osk
include $(OSK_PATH)/src/$(OS)/Makefile.osk

SRC=\
	../mali_ukk.c \
	mali_ukk_os.c 

#MODULE := ukk.ko

#obj-m := $(MODULE:.ko=.o)

#$(MODULE:.ko=-y) := $(SRC:.c=.o)

#$(MODULE:.ko=-objs) := $(SRC:.c=.o) $(RELATIVE_ROOT)/osk/src/$(OS)/lib.a

EXTRA_CFLAGS += -I$(ROOT)/uk -I$(ROOT)/uk/platform_$(PLATFORM) -DMALI_DEBUG=$(MALI_DEBUG) -DMALI_HW_TYPE=$(MALI_HW_TYPE)  -DMALI_BASE_TRACK_MEMLEAK=$(MALI_BASE_TRACK_MEMLEAK) 

else

KDIR ?= /lib/modules/$(shell uname -r)/build

all: 
	$(MAKE) -C $(KDIR) M=$(PWD) modules


endif
