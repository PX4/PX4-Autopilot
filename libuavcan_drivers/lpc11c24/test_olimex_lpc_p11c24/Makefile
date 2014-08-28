#
# Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
#

CPPSRC := $(wildcard src/*.cpp)                  \
          $(wildcard src/sys/*.cpp)

CSRC   := $(wildcard lpc_chip_11cxx_lib/src/*.c) \
          $(wildcard src/sys/*.c)

DEF = -DFW_VERSION_MAJOR=1 -DFW_VERSION_MINOR=0

INC = -Isrc/sys                         \
      -isystem lpc_chip_11cxx_lib/inc

#
# UAVCAN library
#

DEF += -DUAVCAN_TINY=1

include ../../../libuavcan/include.mk
CPPSRC += $(LIBUAVCAN_SRC)
INC += -I$(LIBUAVCAN_INC)

include ../driver/include.mk
CPPSRC += $(LIBUAVCAN_LPC11C24_SRC)
INC += -I$(LIBUAVCAN_LPC11C24_INC)

$(info $(shell $(LIBUAVCAN_DSDLC) $(UAVCAN_DSDL_DIR)))
INC += -Idsdlc_generated

#
# Git commit hash
#

GIT_HASH := $(shell git rev-parse --short HEAD)
ifeq ($(words $(GIT_HASH)),1)
    DEF += -DGIT_HASH=0x$(GIT_HASH)
endif

#
# Build configuration
#

BUILDDIR = build
OBJDIR = $(BUILDDIR)/obj
DEPDIR = $(BUILDDIR)/dep

DEF += -DNDEBUG -DCHIP_LPC11CXX -DCORE_M0 -DTHUMB_NO_INTERWORKING -U__STRICT_ANSI__

FLAGS = -mthumb -mcpu=cortex-m0 -mno-thumb-interwork -flto -Os -g3 -Wall -Wextra -Werror -ffunction-sections \
        -fdata-sections -fno-common -fno-exceptions -fno-unwind-tables -fno-stack-protector -fomit-frame-pointer \
        -ftracer -ftree-loop-distribute-patterns -frename-registers -freorder-blocks -fconserve-stack \
        -Wfloat-equal -Wconversion -Wsign-conversion -Wmissing-declarations

C_CPP_FLAGS = $(FLAGS) -MD -MP -MF $(DEPDIR)/$(@F).d

CFLAGS = $(C_CPP_FLAGS) -std=c99

CPPFLAGS = $(C_CPP_FLAGS) -pedantic -std=c++11 -fno-rtti -fno-threadsafe-statics

LDFLAGS = $(FLAGS) -nodefaultlibs -lc -lgcc -nostartfiles -Tlpc11c24.ld -Xlinker --gc-sections \
          -Wl,-Map,$(BUILDDIR)/output.map

# Link with nano newlib. Other toolchains may not support this option, so it can be safely removed.
LDFLAGS += --specs=nano.specs

COBJ   = $(addprefix $(OBJDIR)/, $(notdir $(CSRC:.c=.o)))
CPPOBJ = $(addprefix $(OBJDIR)/, $(notdir $(CPPSRC:.cpp=.o)))
OBJ = $(COBJ) $(CPPOBJ)

VPATH = $(sort $(dir $(CSRC)) $(dir $(CPPSRC)))

ELF = $(BUILDDIR)/firmware.elf
BIN = $(BUILDDIR)/firmware.bin

#
# Rules
#

TOOLCHAIN ?= arm-none-eabi-
CC   = $(TOOLCHAIN)gcc
CPPC = $(TOOLCHAIN)g++
AS   = $(TOOLCHAIN)gcc
LD   = $(TOOLCHAIN)g++
CP   = $(TOOLCHAIN)objcopy
SIZE = $(TOOLCHAIN)size

all: $(OBJ) $(ELF) $(BIN) size

$(OBJ): | $(BUILDDIR)

$(BUILDDIR):
	@mkdir -p $(BUILDDIR)
	@mkdir -p $(DEPDIR)
	@mkdir -p $(OBJDIR)

$(BIN): $(ELF)
	@echo
	$(CP) -O binary $(ELF) $@

$(ELF): $(OBJ)
	@echo
	$(LD) $(OBJ) $(LDFLAGS) -o $@

$(COBJ): $(OBJDIR)/%.o: %.c
	@echo
	$(CC) -c $(DEF) $(INC) $(CFLAGS) $< -o $@

$(CPPOBJ): $(OBJDIR)/%.o: %.cpp
	@echo
	$(CPPC) -c $(DEF) $(INC) $(CPPFLAGS) $< -o $@

clean:
	rm -rf $(BUILDDIR)

size:
	@if [ -f $(ELF) ]; then echo; $(SIZE) $(ELF); echo; fi;

.PHONY: all clean size $(BUILDDIR)

# Include the dependency files, should be the last of the makefile
-include $(shell mkdir $(DEPDIR) 2>/dev/null) $(wildcard $(DEPDIR)/*)
