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
	mali_osk_debug.c \
	../common/mali_osk_cutils_cstr.c \
	../common/mali_osk_bitops_cmn.c \
	../common/mali_osk_debug_cmn.c 

ifeq ($(MALI_BASE_TRACK_MEMLEAK), 1)
SRC += ../common/mali_osk_failure.c ../common/mali_osk_mem_track.c
endif
#lib-y := $(SRC:.c=.o)

ROOT=$(KBUILD_EXTMOD)/../../..
include $(KBUILD_EXTMOD)/Makefile.osk

EXTRA_CFLAGS += -DMALI_DEBUG=$(MALI_DEBUG) -DMALI_BASE_TRACK_MEMLEAK=$(MALI_BASE_TRACK_MEMLEAK)

else

KDIR ?= /lib/modules/$(shell uname -r)/build

all: 
	$(MAKE) -C $(KDIR) M=$(PWD) 

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

endif
