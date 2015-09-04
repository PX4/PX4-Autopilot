#
#   Copyright (C) 2015 Mark Charlebois. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

#
# Definitions for a generic GNU ARM-EABI toolchain
#

#$(info TOOLCHAIN  gnu-arm-eabi)

# Toolchain commands. Normally only used inside this file.
#
HEXAGON_TOOLS_ROOT	 ?= /opt/6.4.03
#HEXAGON_TOOLS_ROOT	 = /opt/6.4.05
HEXAGON_SDK_ROOT	 ?= /opt/Hexagon_SDK/2.0
V_ARCH			 = v5
CROSSDEV		 = hexagon-
HEXAGON_BIN		 = $(addsuffix /gnu/bin,$(HEXAGON_TOOLS_ROOT))
HEXAGON_CLANG_BIN	 = $(addsuffix /qc/bin,$(HEXAGON_TOOLS_ROOT))
HEXAGON_LIB_DIR		 = $(HEXAGON_TOOLS_ROOT)/gnu/hexagon/lib
HEXAGON_ISS_DIR		 = $(HEXAGON_TOOLS_ROOT)/qc/lib/iss
TOOLSLIB		 = $(HEXAGON_TOOLS_ROOT)/dinkumware/lib/$(V_ARCH)/G0
QCTOOLSLIB		 = $(HEXAGON_TOOLS_ROOT)/qc/lib/$(V_ARCH)/G0
QURTLIB			 = $(HEXAGON_SDK_ROOT)/lib/common/qurt/ADSP$(V_ARCH)MP/lib
DSPAL_INCS		 ?= $(PX4_BASE)/src/lib/dspal


CC			 = $(HEXAGON_CLANG_BIN)/$(CROSSDEV)clang
CXX			 = $(HEXAGON_CLANG_BIN)/$(CROSSDEV)clang++
CPP			 = $(HEXAGON_CLANG_BIN)/$(CROSSDEV)clang -E
LD			 = $(HEXAGON_BIN)/$(CROSSDEV)ld
AR			 = $(HEXAGON_BIN)/$(CROSSDEV)ar rcs
NM			 = $(HEXAGON_BIN)/$(CROSSDEV)nm
OBJCOPY			 = $(HEXAGON_BIN)/$(CROSSDEV)objcopy
OBJDUMP			 = $(HEXAGON_BIN)/$(CROSSDEV)objdump
HEXAGON_GCC		 = $(HEXAGON_BIN)/$(CROSSDEV)gcc

QURTLIBS		 = \
			   $(QCTOOLSLIB)/libdl.a \
			   $(TOOLSLIB)/init.o \
			   $(TOOLSLIB)/libc.a \
			   $(TOOLSLIB)/libqcc.a \
			   $(TOOLSLIB)/libstdc++.a \
			   $(QURTLIB)/crt0.o \
			   $(QURTLIB)/libqurt.a \
			   $(QURTLIB)/libqurtkernel.a \
			   $(QURTLIB)/libqurtcfs.a \
			   $(QURTLIB)/libqube_compat.a \
			   $(QURTLIB)/libtimer.a \
			   $(QURTLIB)/libposix.a \
			   $(QURTLIB)/../examples/cust_config.o \
			   $(QCTOOLSLIB)/libhexagon.a \
			   $(TOOLSLIB)/fini.o 

DYNAMIC_LIBS            = \
			   -Wl,$(TOOLSLIB)/pic/libstdc++.a


# Check if the right version of the toolchain is available
#
CROSSDEV_VER_SUPPORTED	 = 6.4.03 6.4.05
CROSSDEV_VER_FOUND	 = $(shell $(CC) --version | sed -n 's/^.*version \([\. 0-9]*\),.*$$/\1/p')

ifeq (,$(findstring $(CROSSDEV_VER_FOUND), $(CROSSDEV_VER_SUPPORTED)))
$(error Unsupported version of $(CC), found: $(CROSSDEV_VER_FOUND) instead of one in: $(CROSSDEV_VER_SUPPORTED))
endif


# XXX this is pulled pretty directly from the fmu Make.defs - needs cleanup

MAXOPTIMIZATION		 := -O0

# Base CPU flags for each of the supported architectures.
#
ARCHCPUFLAGS	 	 = -m$(V_ARCH) -G0


# Set the board flags
#
ifeq ($(CONFIG_BOARD),)
$(error Board config does not define CONFIG_BOARD)
endif
ARCHDEFINES		+= -DCONFIG_ARCH_BOARD_$(CONFIG_BOARD) \
			    -D__PX4_QURT -D__PX4_POSIX \
			    -D_PID_T -D_UID_T -D_TIMER_T\
			    -Dnoreturn_function= \
			    -D__EXPORT= \
			    -Drestrict= \
			    -D_DEBUG \
			    -I$(DSPAL_INCS)/include \
			    -I$(DSPAL_INCS)/sys \
			    -I$(HEXAGON_TOOLS_ROOT)/gnu/hexagon/include \
			    -I$(PX4_BASE)/src/lib/eigen \
			    -I$(PX4_BASE)/src/platforms/qurt/include \
			    -I$(PX4_BASE)/src/platforms/posix/include \
			    -I$(PX4_BASE)/mavlink/include/mavlink \
			    -I$(PX4_BASE)/../inc \
			    -I$(QURTLIB)/..//include \
			    -I$(HEXAGON_SDK_ROOT)/inc \
			    -I$(HEXAGON_SDK_ROOT)/inc/stddef \
			    -Wno-error=shadow



# optimisation flags
#
ARCHOPTIMIZATION	 = \
                           -O0 \
			   -g \
			   -fno-strict-aliasing \
			   -fdata-sections \
                           -fpic  \
                           -fno-zero-initialized-in-bss

#-fomit-frame-pointer \
#-funsafe-math-optimizations \
#-ffunction-sections
#$(MAXOPTIMIZATION)

# enable precise stack overflow tracking
# note - requires corresponding support in NuttX
INSTRUMENTATIONDEFINES	 = $(ARCHINSTRUMENTATIONDEFINES_$(CONFIG_ARCH))

# Language-specific flags
#
ARCHCFLAGS		 = -std=gnu99 -D__CUSTOM_FILE_IO__
ARCHCXXFLAGS		 = -fno-exceptions -fno-rtti -std=gnu++0x -fno-threadsafe-statics -D__CUSTOM_FILE_IO__

# Generic warnings
#
ARCHWARNINGS		 = -Wall \
			   -Wextra \
			   -Werror \
			   -Wno-unused-parameter \
			   -Wno-unused-function \
			   -Wno-unused-variable \
			   -Wno-gnu-array-member-paren-init \
			   -Wno-cast-align \
			   -Wno-missing-braces \
			   -Wno-strict-aliasing
#   -Werror=float-conversion - works, just needs to be phased in with some effort and needs GCC 4.9+
#   -Wcast-qual  - generates spurious noreturn attribute warnings, try again later
#   -Wconversion - would be nice, but too many "risky-but-safe" conversions in the code
#   -Wcast-align - would help catch bad casts in some cases, but generates too many false positives

# C-specific warnings
#
ARCHCWARNINGS		 = $(ARCHWARNINGS) \
			   -Wstrict-prototypes \
			   -Wnested-externs

# C++-specific warnings
#
ARCHWARNINGSXX		 = $(ARCHWARNINGS) \
			   -Wno-missing-field-initializers

# pull in *just* libm from the toolchain ... this is grody
LIBM			:= $(shell $(CC) $(ARCHCPUFLAGS) -print-file-name=libm.a)
EXTRA_LIBS		+= $(LIBM)

# Flags we pass to the C compiler
#
CFLAGS			 = $(ARCHCFLAGS) \
			   $(ARCHCWARNINGS) \
			   $(ARCHOPTIMIZATION) \
			   $(ARCHCPUFLAGS) \
			   $(ARCHINCLUDES) \
			   $(INSTRUMENTATIONDEFINES) \
			   $(ARCHDEFINES) \
			   $(EXTRADEFINES) \
			   $(EXTRACFLAGS) \
			   $(addprefix -I,$(INCLUDE_DIRS))

			   #-fno-common
# Flags we pass to the C++ compiler
#
CXXFLAGS		 = $(ARCHCXXFLAGS) \
			   $(ARCHWARNINGSXX) \
			   $(ARCHOPTIMIZATION) \
			   $(ARCHCPUFLAGS) \
			   $(ARCHXXINCLUDES) \
			   $(INSTRUMENTATIONDEFINES) \
			   $(ARCHDEFINES) \
			   -DCONFIG_WCHAR_BUILTIN \
			   $(EXTRADEFINES) \
			   $(EXTRACXXFLAGS) \
			   $(addprefix -I,$(INCLUDE_DIRS))

# Flags we pass to the assembler
#
AFLAGS			 = $(CFLAGS) -D__ASSEMBLY__ \
			   $(EXTRADEFINES) \
			   $(EXTRAAFLAGS)

LDSCRIPT		 = $(PX4_BASE)/makefiles/posix/ld.script
# Flags we pass to the linker
#
LDFLAGS			+=  -g -mv5 -mG0lib -G0 -fpic -shared \
			   -Wl,-Bsymbolic \
			   -Wl,--wrap=malloc \
			   -Wl,--wrap=calloc \
			   -Wl,--wrap=free \
			   -Wl,--wrap=realloc \
			   -Wl,--wrap=memalign \
			   -Wl,--wrap=__stack_chk_fail  \
                           -lc \
			   $(EXTRALDFLAGS) \
			   $(addprefix -L,$(LIB_DIRS))

# Compiler support library
#
LIBGCC			:= $(shell $(CC) $(ARCHCPUFLAGS) -print-libgcc-file-name)

# Files that the final link depends on
#
LINK_DEPS		+= $(LDSCRIPT)

# Files to include to get automated dependencies
#
DEP_INCLUDES		 = $(subst .o,.d,$(OBJS))

# Compile C source $1 to object $2
# as a side-effect, generate a dependency file
#
define COMPILENOSHARED
	@$(ECHO) "CC:      $1"
	@$(MKDIR) -p $(dir $2)
	@echo $(Q) $(CCACHE) $(CC) -MD -c $(CFLAGS) $(abspath $1) -o $2
	$(Q) $(CCACHE) $(CC) -MD -c $(CFLAGS) $(abspath $1) -o $2
endef

# Compile C source $1 to object $2 for use in shared library
# as a side-effect, generate a dependency file
#
define COMPILE
	@$(ECHO) "CC:      $1"
	@$(MKDIR) -p $(dir $2)
	@echo $(Q) $(CCACHE) $(CC) -MD -c $(CFLAGS) $(abspath $1) -o $2
	#$(Q) $(CCACHE) $(CC) -MD -c $(CFLAGS) -D__V_DYNAMIC__ -fPIC $(abspath $1) -o $2
	#$(CCACHE) $(CC) -MD -c $(CFLAGS) -D__V_DYNAMIC__  -D__FILENAME__=\"$(notdir $1)\" -fPIC $(abspath $1) -o $2
	$(CCACHE) $(CC) -c $(CFLAGS) -D__V_DYNAMIC__  -D__FILENAME__=\"$(notdir $1)\" $(abspath $1) -o $2
endef

# Compile C++ source $1 to $2 for use in shared library
# as a side-effect, generate a dependency file
#
define COMPILEXX
	@$(ECHO) "CXX:     $1"
	@$(MKDIR) -p $(dir $2)
	@echo $(Q) $(CCACHE) $(CXX) -MD -c $(CXXFLAGS) $(abspath $1) -o $2
	#$(Q) $(CCACHE) $(CXX) -MD -c $(CXXFLAGS) -D__V_DYNAMIC__ -fPIC $(abspath $1) -o $2
	#$(CCACHE) $(CXX) -MD -c $(CXXFLAGS) -D__V_DYNAMIC__ -D__FILENAME__=\"$(notdir $1)\" -fPIC $(abspath $1) -o $2
	$(CCACHE) $(CXX) -c $(CXXFLAGS) -D__V_DYNAMIC__ -D__FILENAME__=\"$(notdir $1)\" $(abspath $1) -o $2
endef

# Assemble $1 into $2
#
define ASSEMBLE
	@$(ECHO) "AS:      $1"
	@$(MKDIR) -p $(dir $2)
	$(Q) $(CC) -c $(AFLAGS) $(abspath $1) -o $2
endef

# Produce partially-linked $1 from files in $2
#
#$(Q) $(LD) -Ur -o $1 $2 # -Ur not supported in ld.gold
define PRELINK
	@$(ECHO) "PRELINK: $1"
	@$(MKDIR) -p $(dir $1)
	@echo $(Q) $(LD) -Ur -o $1 $2
	$(Q) $(LD) -Ur -o $1 $2

endef
# Produce partially-linked $1 from files in $2
#
#$(Q) $(LD) -Ur -o $1 $2 # -Ur not supported in ld.gold
define PRELINKF
	@$(ECHO) "PRELINKF: $1"
	@$(MKDIR) -p $(dir $1)
	@echo $(Q) $(LD) -Ur -T$(LDSCRIPT) -o $1 $2
	$(Q) $(LD) -Ur -T$(LDSCRIPT) -o $1 $2 

endef
#	$(Q) $(LD) -Ur -o $1 $2 && $(OBJCOPY) --localize-hidden $1

# Update the archive $1 with the files in $2
#
define ARCHIVE
	@$(ECHO) "AR:      $2"
	@$(MKDIR) -p $(dir $1)
	$(Q) $(AR) $1 $2
endef

# Link the objects in $2 into the shared library $1
#
define LINK_A
	@$(ECHO) "LINK_A:    $1"
	@$(MKDIR) -p $(dir $1)
	echo "$(Q) $(AR) $1 $2"
	$(Q) $(AR) $1 $2
endef

# Link the objects in $2 into the shared library $1
#
define LINK_SO
	@$(ECHO) "LINK_SO:    $1"
	@$(MKDIR) -p $(dir $1)
	$(HEXAGON_GCC) $(LDFLAGS) -o $1 -Wl,--whole-archive $2 -Wl,--no-whole-archive $(LIBS) $(DYNAMIC_LIBS)
endef

# Link the objects in $2 into the application $1
#
define LINK
	@$(ECHO) "LINK:    $1"
	@$(MKDIR) -p $(dir $1)
	$(LD) --section-start .start=0x1d000000 -o $1 --start-group $2 $(EXTRA_LIBS) $(QURTLIBS) --end-group --dynamic-linker= -E --force-dynamic
endef

