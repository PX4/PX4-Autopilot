#
#   Copyright (C) 2012-2014 PX4 Development Team. All rights reuint32_tserved.
#
#   2005 Modified for clang and GCC on POSIX:
#        Author: Mark Charlebois <charlebm@gmail.com>
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
# Definitions for a native GCC toolchain
#

#$(info TOOLCHAIN  native)

# Toolchain commands. Normally only used inside this file.
#

# Set to 1 for GCC-4.8.2 and to 0 for Clang-3.5 (Ubuntu 14.04)
USE_GCC?=0

ifneq ($(USE_GCC),1)

HAVE_CLANG35:=$(shell clang-3.5 -dumpversion 2>/dev/null)

# Clang will report 4.2.1 as GCC version
HAVE_CLANG:=$(shell clang -dumpversion)

#If using ubuntu 14.04 and packaged clang 3.5
ifeq ($(HAVE_CLANG35),4.2.1)
USE_GCC=0
CLANGVER=-3.5
else

#If using ubuntu 12.04 and downloaded clang 3.4.2
ifeq ($(HAVE_CLANG),4.2.1)
USE_GCC=0
CLANGVER=
endif
endif

# If no version of clang was found
ifeq ($(HAVE_CLANG35),)
ifeq ($(HAVE_CLANG),)
$(error Clang not found. Try make USE_GCC=1)
endif
endif
endif # USE_GCC is not 1

ifeq ($(USE_GCC),1)
# GCC Options:
CC			 = gcc
CXX			 = g++
CPP			 = gcc -E

# GCC Version
DEV_VER_SUPPORTED	 = 4.8.1 4.8.2 4.9.1

else
# Clang options
CC			 = clang$(CLANGVER)
CXX			 = clang++$(CLANGVER)
CPP			 = clang$(CLANGVER) -E

# Clang GCC reported version
DEV_VER_SUPPORTED	 = 4.2.1
endif

#LD			 = ld.gold
LD			 = ld.bfd
AR			 = ar rcs
NM			 = nm
OBJCOPY			 = objcopy
OBJDUMP			 = objdump

# Check if the right version of the toolchain is available
#
DEV_VER_FOUND	 = $(shell $(CC) -dumpversion)

ifeq (,$(findstring $(DEV_VER_FOUND), $(DEV_VER_SUPPORTED)))
$(error Unsupported version of $(CC), found: $(DEV_VER_FOUND) instead of one in: $(DEV_VER_SUPPORTED))
endif


# XXX this is pulled pretty directly from the fmu Make.defs - needs cleanup

MAXOPTIMIZATION		 ?= -O3

# Enabling stack checks if OS was build with them
#

# Set the board flags
#
ifeq ($(CONFIG_BOARD),)
$(error Board config does not define CONFIG_BOARD)
endif
ARCHDEFINES		+= -DCONFIG_ARCH_BOARD_$(CONFIG_BOARD) \
			-D__PX4_LINUX -D__PX4_POSIX \
			-Dnoreturn_function= \
			-I$(PX4_BASE)/src/modules/systemlib \
			-I$(PX4_BASE)/src/lib/eigen \
			-I$(PX4_BASE)/src/platforms/posix/include \
			-I$(PX4_BASE)/mavlink/include/mavlink \
			-Wno-error=shadow

# optimisation flags
#
ARCHOPTIMIZATION	 = $(MAXOPTIMIZATION) \
			   -g3 \
			   -fno-strict-aliasing \
			   -fomit-frame-pointer \
			   -funsafe-math-optimizations \
			   -fno-builtin-printf \
			   -ffunction-sections \
			   -fdata-sections

# Language-specific flags
#
ARCHCFLAGS		 = -std=gnu99 -g
ARCHCXXFLAGS		 = -fno-exceptions -fno-rtti -std=c++0x -fno-threadsafe-statics -D__CUSTOM_FILE_IO__ -g

# Generic warnings
#
# Disabled
#			   -Wshadow 			- Breaks for the libeigen package headers
#			   -Wframe-larger-than=1024  	- Only needed for embedded

ARCHWARNINGS		 = -Wall \
			   -Wextra \
			   -Werror \
			   -Wfloat-equal \
			   -Wpointer-arith \
			   -Wmissing-declarations \
			   -Wpacked \
			   -Wno-unused-parameter \
			   -Wno-packed \
			   -Werror=format-security \
			   -Werror=array-bounds \
			   -Wfatal-errors \
			   -Werror=unused-variable \
			   -Werror=reorder \
			   -Werror=uninitialized \
			   -Werror=init-self

# Add compiler specific options
ifeq ($(USE_GCC),1)
ARCHDEFINES		+= -Wno-error=logical-op
ARCHWARNINGS		+= -Wdouble-promotion \
			   -Wlogical-op \
			   -Wformat=1 \
			   -Werror=unused-but-set-variable \
			   -Werror=double-promotion
ARCHOPTIMIZATION	+= -fno-strength-reduce
else
ARCHWARNINGS		+= -Wno-gnu-array-member-paren-init 
endif

#   -Werror=float-conversion - works, just needs to be phased in with some effort and needs GCC 4.9+
#   -Wcast-qual  - generates spurious noreturn attribute warnings, try again later
#   -Wconversion - would be nice, but too many "risky-but-safe" conversions in the code
#   -Wcast-align - would help catch bad casts in some cases, but generates too many false positives

# C-specific warnings
#
ARCHCWARNINGS		 = $(ARCHWARNINGS) \
			   -Wbad-function-cast \
			   -Wstrict-prototypes \
			   -Wmissing-prototypes \
			   -Wnested-externs

# Add compiler specific options
ifeq ($(USE_GCC),1)
ARCHCWARNINGS		+= -Wold-style-declaration \
			   -Wmissing-parameter-type \
			   -Wno-error=unused-local-typedefs \
			   -Wno-error=enum-compare \
			   -Wno-error=float-equal
endif

# C++-specific warnings
#
ARCHWARNINGSXX		 = $(ARCHWARNINGS) \
			   -Wno-missing-field-initializers

# pull in *just* libm from the toolchain ... this is grody
LIBM			:= $(shell $(CC) $(ARCHCPUFLAGS) -print-file-name=libm.a)
#EXTRA_LIBS		+= $(LIBM)
EXTRA_LIBS		+= -pthread -lm -lrt

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
			   -fno-common \
			   $(addprefix -I,$(INCLUDE_DIRS))

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
			   -Wno-effc++ \
			   $(addprefix -I,$(INCLUDE_DIRS))

ifeq ($(USE_GCC),0)
CXXFLAGS		+= -Wno-deprecated-register \
			   -Wno-tautological-constant-out-of-range-compare \
			   -Wno-unused-private-field \
			   -Wno-unused-const-variable
endif

# Flags we pass to the assembler
#
AFLAGS			 = $(CFLAGS) -D__ASSEMBLY__ \
			   $(EXTRADEFINES) \
			   $(EXTRAAFLAGS)

LDSCRIPT		 = $(PX4_BASE)/posix-configs/posixtest/scripts/ld.script
# Flags we pass to the linker
#
LDFLAGS			+= $(EXTRALDFLAGS) \
			   $(addprefix -L,$(LIB_DIRS))

# Compiler support library
#
LIBGCC			:= $(shell $(CC) $(ARCHCPUFLAGS) -print-libgcc-file-name)

# Files that the final link depends on
#
#LINK_DEPS		+= $(LDSCRIPT)
LINK_DEPS		+=

# Files to include to get automated dependencies
#
DEP_INCLUDES		 = $(subst .o,.d,$(OBJS))

# Compile C source $1 to object $2
# as a side-effect, generate a dependency file
#
define COMPILE
	@$(ECHO) "CC:      $1"
	@$(MKDIR) -p $(dir $2)
	$(Q) $(CCACHE) $(CC) -MD -c $(CFLAGS) $(abspath $1) -o $2
endef

# Compile C++ source $1 to $2
# as a side-effect, generate a dependency file
#
define COMPILEXX
	@$(ECHO) "CXX:     $1"
	@$(MKDIR) -p $(dir $2)
	@echo $(Q) $(CCACHE) $(CXX) -MD -c $(CXXFLAGS) $(abspath $1) -o $2
	$(Q) $(CCACHE) $(CXX) -MD -c $(CXXFLAGS) $(abspath $1) -o $2
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
	$(Q) $(LD) -Ur -o $1 $2

endef
# Produce partially-linked $1 from files in $2
#
#$(Q) $(LD) -Ur -o $1 $2 # -Ur not supported in ld.gold
define PRELINKF
	@$(ECHO) "PRELINK: $1"
	@$(MKDIR) -p $(dir $1)
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
	echo "$(Q) $(CXX) $(LDFLAGS) -shared -Wl,-soname,`basename $1`.1 -o $1 $2 $(LIBS) $(EXTRA_LIBS)"
	$(Q) $(CXX) $(LDFLAGS) -shared -Wl,-soname,`basename $1`.1 -o $1 $2 $(LIBS) -pthread -lc
endef

# Link the objects in $2 into the application $1
#
define LINK
	@$(ECHO) "LINK:    $1"
	@$(MKDIR) -p $(dir $1)
	$(Q) $(CXX) $(CXXFLAGS) $(LDFLAGS) -o $1 $2 $(LIBS) $(EXTRA_LIBS) $(LIBGCC)

endef

