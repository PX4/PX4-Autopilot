# Bare-metal Cortex-M4 rpmsg remote for the i.MX RT1176 on FMU-v6X-RT.
# Invoked by CMakeLists.txt in this directory during the PX4 build; the ELF
# lands in the PX4 ROMFS as /etc/extras/cm4_rpmsg.elf.
#
# Standalone:
#   make -C boards/px4/fmu-v6xrt/cm4 BUILDDIR=/tmp/cm4 APP=basic

CROSS    ?= arm-none-eabi-
# CC has a make default (cc); only honour it when set on the command line
ifeq ($(origin CC),default)
CC       := $(CROSS)gcc
endif
STRIP    ?= $(CROSS)strip
SIZE     ?= $(CROSS)size
BUILDDIR ?= build

SRCDIR  := $(dir $(realpath $(lastword $(MAKEFILE_LIST))))
TARGET  := $(BUILDDIR)/cm4_rpmsg.elf

# One application (apps/<APP>/) on top of the shared runtime; the runtime
# never references the application by name. CONFIG_BOARD_CM4_APP selects it.
APP     ?= basic
APPDIR  := $(SRCDIR)apps/$(APP)
ifeq ($(wildcard $(APPDIR)/main.c),)
$(error no CM4 application at $(APPDIR))
endif

SRCS    := $(addprefix $(SRCDIR),startup.c libc.c mu.c rpmsg_remote.c) $(wildcard $(APPDIR)/*.c)
HDRS    := $(addprefix $(SRCDIR),libc.h mu.h rpmsg_remote.h status.h fault.h memmap.h) $(wildcard $(APPDIR)/*.h)
DEFINES := -DCM4_APP_NAME='"$(APP)"'

# PX4 tree and build directory (generated headers), for applications that need them.
PX4_SRC   ?= $(SRCDIR)../../../..
PX4_BUILD ?= $(PX4_SRC)/build/px4_fmu-v6xrt_rpmsg

# Optional per-application fragment: APP_SRCS, APP_INCLUDES, APP_DEFINES.
-include $(APPDIR)/app.mk
SRCS    += $(APP_SRCS)
DEFINES += $(APP_DEFINES)

# "!fault <kind>" on the hello endpoint (nsh: remote_core fault); 0 for a production image
FAULT_INJECT ?= 1
ifeq ($(FAULT_INJECT),1)
SRCS    += $(SRCDIR)fault.c
DEFINES += -DCM4_FAULT_INJECT
endif

INCLUDES := -I$(SRCDIR) -I$(APPDIR) $(APP_INCLUDES)
LDSCRIPT := $(BUILDDIR)/link.ld

CFLAGS  := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c11 -O2 -g \
           -Wall -Wextra -Wno-override-init -Werror \
           -ffreestanding -fno-builtin -fno-common -fno-math-errno \
           -ffunction-sections -fdata-sections
# -n: no 64K segment page alignment, keeps the ELF at a few KB
LDFLAGS := -nostdlib -nostartfiles -T $(LDSCRIPT) \
           -Wl,--gc-sections -Wl,-n -Wl,-Map=$(TARGET:.elf=.map)

all: $(TARGET)

$(LDSCRIPT): $(SRCDIR)link.ld.in $(SRCDIR)memmap.h
	@mkdir -p $(BUILDDIR)
	$(CC) -E -P -x c -I$(SRCDIR) -o $@ $<

$(TARGET): $(SRCS) $(HDRS) $(LDSCRIPT) $(SRCDIR)Makefile
	$(CC) $(CFLAGS) $(DEFINES) $(INCLUDES) $(LDFLAGS) -o $@ $(SRCS) -lgcc
	$(STRIP) --strip-debug $@
	$(SIZE) $@

clean:
	rm -f $(TARGET) $(TARGET:.elf=.map) $(LDSCRIPT)

.PHONY: all clean
