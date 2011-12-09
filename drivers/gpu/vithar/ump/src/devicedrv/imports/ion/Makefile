#
# This confidential and proprietary software may be used only as
# authorised by a licensing agreement from ARM Limited
# (C) COPYRIGHT 2011 ARM Limited
# ALL RIGHTS RESERVED
# The entire notice above must be reproduced on all authorised
# copies and copies may only be made to the extent permitted
# by a licensing agreement from ARM Limited.
#

# default to building for the host
ARCH ?= $(shell uname -m)

# linux build system integration

#ifneq ($(KERNELRELEASE),)
ifneq ($(VITHAR_ROOT),)
# Inside the kernel build system

EXTRA_CFLAGS += -I$(KBUILD_EXTMOD) -I$(KBUILD_EXTMOD)/../../../../..
KBUILD_EXTRA_SYMBOLS += "$(KBUILD_EXTMOD)/../../Module.symvers"

SRC += imports/ion/ump_kernel_import_ion.c

#MODULE:=ump_ion_import.ko

#obj-m := $(MODULE:.ko=.o)
#$(MODULE:.ko=-y) := $(SRC:.c=.o)
#$(MODULE:.ko=-objs) := $(SRC:.c=.o)

else
# Outside the kernel build system
#
#

ifeq ($(KDIR),)
$(error Must specify KDIR to point to the kernel to target))
endif

all:
	$(MAKE) ARCH=$(ARCH) -C $(KDIR) M=$(CURDIR)

clean:
	$(MAKE) ARCH=$(ARCH) -C $(KDIR) M=$(CURDIR) clean

endif

