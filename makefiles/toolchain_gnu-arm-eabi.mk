#
# Definitions for a generic GNU ARM-EABI toolchain
#

#$(info TOOLCHAIN  gnu-arm-eabi)

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
			   $(addprefix -T,$(LDSCRIPT)) \
			   $(addprefix -L,$(LIB_DIRS))

LIBGCC			:= $(shell $(CC) $(ARCHCPUFLAGS) -print-libgcc-file-name)

# files that the final link depends on
# XXX add libraries that we know about here...
LINK_DEPS		+= $(LDSCRIPT)

# files to include to get automated dependencies
DEP_INCLUDES		 = $(subst .o,.d,$(OBJS))

# compile C source $1 to object $2
# as a side-effect, generate a dependency file
define COMPILE
	@$(ECHO) "CC <- $1"
	@$(ECHO) "CC   -> $2"
	@$(MKDIR) -p $(dir $2)
	$(Q) $(CC) -MD -c $(CFLAGS) $(abspath $1) -o $2
endef

# compile C++ source $1 to $2
# as a side-effect, generate a dependency file
define COMPILEXX
	@$(ECHO) "CXX: $1"
	@$(MKDIR) -p $(dir $2)
	$(Q) $(CXX) -MD -c $(CXXFLAGS) $(abspath $1) -o $2
endef

# assemble $1 into $2
define ASSEMBLE
	@$(ECHO) "AS: $1"
	@$(MKDIR) -p $(dir $2)
	$(Q) $(CC) -c $(AFLAGS) $(abspath $1) -o $2
endef

# produce partially-linked $1 from files in $2
define PRELINK
	@$(ECHO) "PRELINK: $1"
	@$(MKDIR) -p $(dir $1)
	$(Q) $(LD) -Ur -o $1 $2 && $(OBJCOPY) --localize-hidden $1
endef

# update the archive $1 with the files in $2
define ARCHIVE
	@$(ECHO) "AR: $2"
	@$(MKDIR) -p $(dir $1)
	$(Q) $(AR) $1 $2
endef

# link the objects in $2 into the binary $1
define LINK
	@$(ECHO) "LINK: $1"
	@$(MKDIR) -p $(dir $1)
	$(Q) $(LD) $(LDFLAGS) -o $1 --start-group $2 $(LIBS) $(EXTRA_LIBS) $(LIBGCC) --end-group
endef

# convert $1 from a linked object to a raw binary in $2
define SYM_TO_BIN
	@$(ECHO) "BIN: $2"
	@$(MKDIR) -p $(dir $2)
	$(Q) $(OBJCOPY) -O binary $1 $2
endef

# Take the raw binary $1 and make it into an object file $2. 
# The symbol $3 points to the beginning of the file, and $3_len
# gives its length.
#
# - compile an empty file to generate a suitable object file
# - relink the object and insert the binary file
# - edit symbol names to suit
#
define BIN_SYM_PREFIX
	_binary_$(subst /,_,$(subst .,_,$1))
endef
define BIN_TO_OBJ
	@$(ECHO) "WRAP: $2"
	@$(MKDIR) -p $(dir $2)
	$(Q) $(ECHO) > $2.c
	$(call COMPILE,$2.c,$2.c.o)
	$(LD) -r -o $2 $2.c.o -b binary $1
	$(OBJCOPY) $2 \
		--redefine-sym $(call BIN_SYM_PREFIX,$1)_start=$3 \
		--redefine-sym $(call BIN_SYM_PREFIX,$1)_size=$3_len \
		--strip-symbol $(call BIN_SYM_PREFIX,$1)_end
endef

#	$(Q) $(OBJCOPY) --add-section .rodata.$3=$1 $2
