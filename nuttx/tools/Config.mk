############################################################################
# Config.mk
# Global build rules and macros.
#
#   Copyright (C) 2011 Gregory Nutt. All rights reserved.
#   Author: Richard Cochran
#           Gregory Nutt <gnutt@nuttx.org>
#
# This file (along with $(TOPDIR)/.config) must be included by every
# configuration-specific Make.defs file.
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
# 3. Neither the name NuttX nor the names of its contributors may be
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
############################################################################

# These are configuration variables that are quoted by configuration tool
# but which must be unquoated when used in the build system.

CONFIG_ARCH       := $(patsubst "%",%,$(strip $(CONFIG_ARCH)))
CONFIG_ARCH_CHIP  := $(patsubst "%",%,$(strip $(CONFIG_ARCH_CHIP)))
CONFIG_ARCH_BOARD := $(patsubst "%",%,$(strip $(CONFIG_ARCH_BOARD)))

# Some defaults just to prohibit some bad behavior if for some reason they
# are not defined

OBJEXT ?= .o
LIBEXT ?= .a

# DELIM - Path segment delimiter character
#
# Depends on this settings defined in board-specific defconfig file installed
# at $(TOPDIR)/.config:
#
#   CONFIG_WINDOWS_NATIVE - Defined for a Windows native build

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  DELIM = $(strip \)
else
  DELIM = $(strip /)
endif

# INCDIR - Convert a list of directory paths to a list of compiler include
#   directirves
# Example: CFFLAGS += ${shell $(INCDIR) [options] "compiler" "dir1" "dir2" "dir2" ...}
#
# Note that the compiler string and each directory path string must quoted if
# they contain spaces or any other characters that might get mangled by the
# shell
#
# Depends on this setting passed as a make commaond line definition from the
# toplevel Makefile:
#
#   TOPDIR - The path to the the top level NuttX directory in the form
#     appropriate for the current build environment
#
# Depends on this settings defined in board-specific defconfig file installed
# at $(TOPDIR)/.config:
#
#   CONFIG_WINDOWS_NATIVE - Defined for a Windows native build

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  INCDIR = "$(TOPDIR)\tools\incdir.bat"
else
  INCDIR = "$(TOPDIR)/tools/incdir.sh"
endif

# PREPROCESS - Default macro to run the C pre-processor
# Example: $(call PREPROCESS, in-file, out-file)
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   CPP - The command to invoke the C pre-processor
#   CPPFLAGS - Options to pass to the C pre-processor

define PREPROCESS
	@echo "CPP: $1->$2"
	$(Q) $(CPP) $(CPPFLAGS) $1 -o $2
endef

# COMPILE - Default macro to compile one C file
# Example: $(call COMPILE, in-file, out-file)
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   CC - The command to invoke the C compiler
#   CFLAGS - Options to pass to the C compiler

define COMPILE
	@echo "CC: $1"
	$(Q) $(CC) -c $(CFLAGS) $1 -o $2
endef

# COMPILEXX - Default macro to compile one C++ file
# Example: $(call COMPILEXX, in-file, out-file)
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   CXX - The command to invoke the C++ compiler
#   CXXFLAGS - Options to pass to the C++ compiler

define COMPILEXX
	@echo "CXX: $1"
	$(Q) $(CXX) -c $(CXXFLAGS) $1 -o $2
endef

# ASSEMBLE - Default macro to assemble one assembly language file
# Example: $(call ASSEMBLE, in-file, out-file)
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   CC - By default, the C compiler is used to compile assembly lagnuage
#        files
#   AFLAGS - Options to pass to the C+compiler

define ASSEMBLE
	@echo "AS: $1"
	$(Q) $(CC) -c $(AFLAGS) $1 -o $2
endef

# ARCHIVE - Add a list of files to an archive
# Example: $(call ARCHIVE, archive-file, "file1 file2 file3 ...")
#
# Note: The fileN strings may not contain spaces or  characters that may be
# interpreted strangely by the shell
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   AR - The command to invoke the archiver (includes any options)
#
# Depends on this settings defined in board-specific defconfig file installed
# at $(TOPDIR)/.config:
#
#   CONFIG_WINDOWS_NATIVE - Defined for a Windows native build

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define ARCHIVE
	@echo "AR: $2"
	$(AR) $1
	$(Q) $(AR) $1 $(2)
endef
else
define ARCHIVE
	@echo "AR: $2"
	$(Q) $(AR) $1 $(2) || { echo "$(AR) $1 FAILED!" ; exit 1 ; }
endef
endif

# DELFILE - Delete one file

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define DELFILE
	$(Q) if exist $1 (del /f /q $1)
endef
else
define CLEAN
	$(Q) rm -f $1
endef
endif

# CLEAN - Default clean target

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define CLEAN
	$(Q) if exist *$(OBJEXT) (del /f /q *$(OBJEXT))
	$(Q) if exist *$(LIBEXT) (del /f /q *$(LIBEXT))
	$(Q) if exist *~ (del /f /q *~)
	$(Q) if exist (del /f /q  .*.swp)
endef
else
define CLEAN
	$(Q) rm -f *$(OBJEXT) *$(LIBEXT) *~ .*.swp
endef
endif
 