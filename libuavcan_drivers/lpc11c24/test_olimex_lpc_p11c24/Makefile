#
# Pavel Kirienko, 2014 <pavel.kirienko@gmail.com>
#

CPPSRC := $(wildcard src/*.cpp)          \
          $(wildcard src/sys/*.cpp)

CSRC   := $(wildcard lpc_chip_11cxx_lib/src/*.c)

ASMSRC := $(wildcard src/sys/*.S)

DEF =

INC = -Isrc/sys                         \
      -Ilpc_chip_11cxx_lib/inc

#
# Build configuration
#

BUILDDIR = build
OBJDIR = $(BUILDDIR)/obj
DEPDIR = $(BUILDDIR)/dep

DEF += -DNDEBUG -DCHIP_LPC11CXX -DCORE_M0 -DTHUMB_NO_INTERWORKING

FLAGS = -mthumb -mcpu=cortex-m0 -mno-thumb-interwork

ASMFLAGS = $(FLAGS)

C_CPP_FLAGS = $(FLAGS) -g3 -Os -Wall -Wextra -Werror -ffunction-sections -fdata-sections \
              -fno-common -fno-exceptions -fno-unwind-tables -fno-stack-protector -fomit-frame-pointer

# Dependencies
C_CPP_FLAGS += -MD -MP -MF $(DEPDIR)/$(@F).d

CFLAGS = $(C_CPP_FLAGS) -std=c99

CPPFLAGS = $(C_CPP_FLAGS) -pedantic -std=c++11 -fno-rtti -fno-threadsafe-statics

LDFLAGS = $(FLAGS) -nodefaultlibs -lc -lgcc -nostartfiles -Tlpc11c24.ld -Xlinker --gc-sections

COBJ   = $(addprefix $(OBJDIR)/, $(notdir $(CSRC:.c=.o)))
CPPOBJ = $(addprefix $(OBJDIR)/, $(notdir $(CPPSRC:.cpp=.o)))
ASMOBJ = $(addprefix $(OBJDIR)/, $(notdir $(ASMSRC:.S=.o)))
OBJ  = $(COBJ) $(ASMOBJ) $(CPPOBJ)

VPATH = $(sort $(dir $(CSRC)) $(dir $(CPPSRC)) $(dir $(ASMSRC)))

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

$(ASMOBJ): $(OBJDIR)/%.o: %.S
	@echo
	$(AS) -c $(DEF) $(INC) $(ASMFLAGS) $< -o $@

clean:
	rm -rf $(BUILDDIR)

size:
	@if [ -f $(ELF) ]; then echo; $(SIZE) $(OBJ) -t; echo; fi;

.PHONY: all clean size $(BUILDDIR)

# Include the dependency files, should be the last of the makefile
-include $(shell mkdir $(DEPDIR) 2>/dev/null) $(wildcard $(DEPDIR)/*)
