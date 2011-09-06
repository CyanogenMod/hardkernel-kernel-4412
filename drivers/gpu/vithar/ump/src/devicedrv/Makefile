#
# This confidential and proprietary software may be used only as
# authorised by a licensing agreement from ARM Limited
# (C) COPYRIGHT 2008-2011 ARM Limited
# ALL RIGHTS RESERVED
# The entire notice above must be reproduced on all authorised
# copies and copies may only be made to the extent permitted
# by a licensing agreement from ARM Limited.
#

ROOT = $(PWD)/../../..

UMP_FILE_PREFIX =
UKK_PATH = $(ROOT)/uk/src/ukk
OSK_PATH = $(ROOT)/osk

ifneq ($(KBUILD_EXTMOD),)
OS=linux
include $(KBUILD_EXTMOD)/Makefile.common
else
include ./Makefile.common
endif

# default to building for the host
ARCH ?= $(shell uname -m)

# linux build system integration

#ifneq ($(KERNELRELEASE),)
ifneq ($(VITHAR_ROOT),)
# Inside the kernel build system

RELATIVE_ROOT=../../..
ROOT = $(KBUILD_EXTMOD)/$(RELATIVE_ROOT)
include $(OSK_PATH)/src/$(OS)/Makefile.osk

EXTRA_CFLAGS += -I$(KBUILD_EXTMOD) -I$(KBUILD_EXTMOD)/common -I$(KBUILD_EXTMOD)/linux -I$(KBUILD_EXTMOD)/$(UKK_PATH)/common -I$(KBUILD_EXTMOD)/$(UKK_PATH)/linux -I$(KBUILD_EXTMOD)/../../.. -I$(KBUILD_EXTMOD)/../../../uk/platform_$(PLATFORM) -DMALI_DEBUG=$(MALI_DEBUG) -DMALI_HW_TYPE=$(MALI_HW_TYPE)  -DMALI_BASE_TRACK_MEMLEAK=$(MALI_BASE_TRACK_MEMLEAK)  

# For customer releases the Linux Device Drivers will be provided as ARM proprietary and GPL releases:
# The ARM proprietary product will only include the license/proprietary directory
# The GPL product will only include the license/gpl directory

ifeq ($(wildcard $(KBUILD_EXTMOD)/linux/license/gpl/*),)
EXTRA_CFLAGS += -I$(KBUILD_EXTMOD)/linux/license/proprietary
else
EXTRA_CFLAGS += -I$(KBUILD_EXTMOD)/linux/license/gpl
endif

SRC +=	$(UMP_FILE_PREFIX)linux/ump_kernel_linux.c $(UMP_FILE_PREFIX)linux/ump_kernel_linux_mem.c

# Selecting files to compile by parsing the config file

#MODULE:=ump.ko

#obj-m := $(MODULE:.ko=.o)
#$(MODULE:.ko=-y) := $(SRC:.c=.o)

#$(MODULE:.ko=-objs) := $(SRC:.c=.o) $(RELATIVE_ROOT)/osk/src/linux/lib.a

else
# Outside the kernel build system
#
#

# Get any user defined KDIR-<names> or maybe even a hardcoded KDIR
-include KDIR_CONFIGURATION

# Define host system directory
KDIR-$(shell uname -m):=/lib/modules/$(shell uname -r)/build

ifeq ($(ARCH), arm)
	# when compiling for ARM we're cross compiling
	export CROSS_COMPILE ?= arm-none-linux-gnueabi-
	CONFIG ?= arm
else
	# Compiling for the host
	CONFIG ?= $(shell uname -m)
endif

# default cpu to select
CPU ?= $(shell uname -m)

# look up KDIR based om CPU selection
KDIR ?= $(KDIR-$(CPU))

ifeq ($(KDIR),)
$(error No KDIR found for platform $(CPU))
endif

# Validate selected config
ifneq ($(shell [ -d arch-$(CONFIG) ] && [ -f arch-$(CONFIG)/config.h ] && echo "OK"), OK)
$(warning Current directory is $(shell pwd))
$(error No configuration found for config $(CONFIG). Check that arch-$(CONFIG)/config.h exists)
else
# Link arch to the selected arch-config directory
$(shell [ -L arch ] && rm arch)
$(shell ln -sf arch-$(CONFIG) arch)
$(shell touch arch/config.h)
endif

# the ump module depends on the ukk module
# we get the symbols from the ukk module using KBUILD_EXTRA_SYMBOLS to prevent warnings about unknown functions
all:
	$(MAKE) ARCH=$(ARCH) -C $(KDIR) M=$(CURDIR) KBUILD_EXTRA_SYMBOLS=$(UKK_PATH)/linux/Module.symvers modules

kernelrelease:
	$(MAKE) -C $(KDIR) kernelrelease

clean:
	$(MAKE) ARCH=$(ARCH) -C $(KDIR) M=$(CURDIR) clean

endif
