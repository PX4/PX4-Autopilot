#
# Definitions for a generic GNU ARM-EABI toolchain
#

$(info %% Toolchain: gnu-arm-eabi)

CROSSDEV		 = arm-none-eabi-

CC			 = $(CROSSDEV)gcc
CXX			 = $(CROSSDEV)g++
CPP			 = $(CROSSDEV)gcc -E
LD			 = $(CROSSDEV)ld
AR			 = $(CROSSDEV)ar rcs
NM			 = $(CROSSDEV)nm
OBJCOPY			 = $(CROSSDEV)objcopy
OBJDUMP			 = $(CROSSDEV)objdump

# XXX this is pulled pretty directly from the fmu Make.defs - needs cleanup

MAXOPTIMIZATION		 = -O3

# base CPU flags
ARCHCPUFLAGS_CORTEXM4F	 = -mcpu=cortex-m4 \
			   -mthumb \
			   -march=armv7e-m \
			   -mfpu=fpv4-sp-d16 \
			   -mfloat-abi=hard

ARCHCPUFLAGS_CORTEXM4	 = -mcpu=cortex-m4 \
			   -mthumb \
			   -march=armv7e-m \
			   -mfloat-abi=soft

ARCHCPUFLAGS_CORTEXM3	 = -mcpu=cortex-m3 \
			   -mthumb \
			   -march=armv7-m \
			   -mfloat-abi=soft

ARCHCPUFLAGS		 = $(ARCHCPUFLAGS_$(CONFIG_ARCH))
ifeq ($(ARCHCPUFLAGS),)
$(error Must set CONFIG_ARCH to one of CORTEXM4F, CORTEXM4 or CORTEXM3)
endif

# optimisation flags
ARCHOPTIMIZATION	 = $(MAXOPTIMIZATION) \
			   -g \
			   -fno-strict-aliasing \
			   -fno-strength-reduce \
			   -fomit-frame-pointer \
   			   -funsafe-math-optimizations \
   			   -fno-builtin-printf \
   			   -ffunction-sections \
   			   -fdata-sections

# enable precise stack overflow tracking
# note - requires corresponding support in NuttX
INSTRUMENTATIONDEFINES	 = -finstrument-functions \
			   -ffixed-r10

ARCHCFLAGS		 = -std=gnu99
ARCHCXXFLAGS		 = -fno-exceptions -fno-rtti -std=gnu++0x
ARCHWARNINGS		 = -Wall \
			   -Wextra \
			   -Wdouble-promotion \
			   -Wshadow \
			   -Wfloat-equal \
			   -Wframe-larger-than=1024 \
			   -Wpointer-arith \
			   -Wlogical-op \
			   -Wmissing-declarations \
			   -Wpacked \
			   -Wno-unused-parameter
#   -Wcast-qual  - generates spurious noreturn attribute warnings, try again later
#   -Wconversion - would be nice, but too many "risky-but-safe" conversions in the code
#   -Wcast-align - would help catch bad casts in some cases, but generates too many false positives

ARCHCWARNINGS		 = $(ARCHWARNINGS) \
			   -Wbad-function-cast \
			   -Wstrict-prototypes \
			   -Wold-style-declaration \
			   -Wmissing-parameter-type \
			   -Wmissing-prototypes \
			   -Wnested-externs \
			   -Wunsuffixed-float-constants
ARCHWARNINGSXX		 = $(ARCHWARNINGS)

# pull in *just* libm from the toolchain ... this is grody
LIBM			:= $(shell $(CC) $(ARCHCPUFLAGS) -print-file-name=libm.a)
EXTRA_LIBS		+= $(LIBM)

CFLAGS			 = $(ARCHCFLAGS) \
			   $(ARCHCWARNINGS) \
			   $(ARCHOPTIMIZATION) \
			   $(ARCHCPUFLAGS) \
			   $(ARCHINCLUDES) \
			   $(INSTRUMENTATIONDEFINES) \
			   $(ARCHDEFINES) \
			   $(EXTRADEFINES) \
			   -fno-common \
			   $(addprefix -I,$(INCLUDE_DIRS))

CXXFLAGS		 = $(ARCHCXXFLAGS) \
			   $(ARCHWARNINGSXX) \
			   $(ARCHOPTIMIZATION) \
			   $(ARCHCPUFLAGS) \
			   $(ARCHXXINCLUDES) \
			   $(INSTRUMENTATIONDEFINES) \
			   $(ARCHDEFINES) \
			   $(EXTRADEFINES) \
			   $(addprefix -I,$(INCLUDE_DIRS))

CPPFLAGS		 = $(ARCHINCLUDES) \
			   $(INSTRUMENTATIONDEFINES) \
			   $(ARCHDEFINES) \
			   $(EXTRADEFINES) \
			   $(addprefix -I,$(INCLUDE_DIRS))

AFLAGS			 = $(CFLAGS) -D__ASSEMBLY__

LDFLAGS			+= --warn-common \
			   --gc-sections \
			   -T $(LDSCRIPT) \
			   $(addprefix -L,$(LIB_DIRS))

LIBGCC			:= $(shell $(CC) $(ARCHCPUFLAGS) -print-libgcc-file-name)

# files that the final link depends on
# XXX add libraries that we know about here...
LINK_DEPS		+= $(LDSCRIPT)

# files to include to get automated dependencies
DEP_INCLUDES		 = $(subst .o,.d,$(OBJS))

ifeq ($(V),)
Q			 = @
else
Q			 =
endif

# compile C source $1 to object $2
# as a side-effect, generate a dependency file
define COMPILE
	@echo "CC: $1"
	$(Q) $(CC) -MD -c $(CFLAGS) $(abspath $1) -o $2
endef

# compile C++ source $1 to $2
# as a side-effect, generate a dependency file
define COMPILEXX
	@echo "CXX: $1"
	$(Q) $(CXX) -MD -c $(CXXFLAGS) $(abspath $1) -o $2
endef

# assemble $1 into $2
define ASSEMBLE
	@echo "AS: $1"
	$(Q) $(CC) -c $(AFLAGS) $(abspath $1) -o $2
endef

# produce partially-linked $1 from files in $2
define PRELINK
	@echo "PRELINK: $1"
	$(Q) $(LD) -Ur -o $1 $2 && $(OBJCOPY) --localize-hidden $1
endef

# update the archive $1 with the files in $2
define ARCHIVE
	@echo "AR: $2"
	$(Q) $(AR) $1 $2
endef

# Link the objects in $2 into the binary $1
define LINK
	@echo "LINK: $1"
	$(Q) $(LD) $(LDFLAGS) -o $1 --start-group $(LIBS) $(EXTRA_LIBS) $(LIBGCC) --end-group
endef

# convert $1 from a linked object to a raw binary
define SYM_TO_BIN
	@echo "BIN: $2"
	$(Q) $(OBJCOPY) -O binary $1 $2
endef
