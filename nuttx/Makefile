############################################################################
# Makefile
#
#   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

TOPDIR		:= ${shell pwd | sed -e 's/ /\\ /g'}
-include ${TOPDIR}/.config
-include ${TOPDIR}/Make.defs

# Default tools

ifeq ($(DIRLINK),)
DIRLINK		= $(TOPDIR)/tools/link.sh
DIRUNLINK	= $(TOPDIR)/tools/unlink.sh
endif

# This define is passed as EXTRADEFINES for kernel-mode builds.  It is also passed
# during PASS1 (but not PASS2) context and depend targets.

KDEFINE		= ${shell $(TOPDIR)/tools/define.sh $(CC) __KERNEL__}

# Process architecture and board-specific directories

ARCH_DIR	= arch/$(CONFIG_ARCH)
ARCH_SRC	= $(ARCH_DIR)/src
ARCH_INC	= $(ARCH_DIR)/include
BOARD_DIR	= configs/$(CONFIG_ARCH_BOARD)

# Add-on directories.  These may or may not be in place in the
# NuttX source tree (they must be specifically installed)
#
# CONFIG_APPS_DIR can be over-ridden from the command line or in the .config file.
# The default value of CONFIG_APPS_DIR is ../apps.  Ultimately, the application
# will be built if APPDIR is defined.  APPDIR will be defined if a directory containing
# a Makefile is found at the path provided by CONFIG_APPS_DIR

ifeq ($(CONFIG_APPS_DIR),)
CONFIG_APPS_DIR	= ../apps
endif
APPDIR		:= ${shell if [ -r $(CONFIG_APPS_DIR)/Makefile ]; then echo "$(CONFIG_APPS_DIR)"; fi}

# All add-on directories.
#
# NUTTX_ADDONS is the list of directories built into the NuttX kernel.
# USER_ADDONS is the list of directories that will be built into the user application

NUTTX_ADDONS	:= $(NX_DIR)
USER_ADDONS		:=

ifeq ($(CONFIG_NUTTX_KERNEL),y)
USER_ADDONS	+= $(APPDIR)
else
NUTTX_ADDONS	+= $(APPDIR)
endif

# Lists of build directories.
#
# FSDIRS depend on file descriptor support; NONFSDIRS do not (except for parts
#   of FSDIRS).  We will exclude FSDIRS from the build if file descriptor
#   support is disabled
# CONTEXTDIRS include directories that have special, one-time pre-build
#   requirements.  Normally this includes things like auto-generation of
#   configuration specific files or creation of configurable symbolic links
# USERDIRS - When NuttX is build is a monolithic kernel, this provides the
#   list of directories that must be built
# OTHERDIRS - These are directories that are not built but probably should
#   be cleaned to prevent garbarge from collecting in them when changing
#   configurations.

NONFSDIRS	= sched $(ARCH_SRC) $(NUTTX_ADDONS)
FSDIRS		= fs drivers binfmt
CONTEXTDIRS	= $(APPDIR)
USERDIRS	=

ifeq ($(CONFIG_NUTTX_KERNEL),y)

NONFSDIRS	+= syscall
CONTEXTDIRS	+= syscall
USERDIRS	+= syscall lib mm $(USER_ADDONS)
ifeq ($(CONFIG_HAVE_CXX),y)
USERDIRS	+= libxx
endif

else

NONFSDIRS	+= lib mm
OTHERDIRS	+= syscall $(USER_ADDONS)
ifeq ($(CONFIG_HAVE_CXX),y)
NONFSDIRS	+= libxx
else
OTHERDIRS	+= libxx
endif

endif

ifeq ($(CONFIG_NX),y)
NONFSDIRS	+= graphics
CONTEXTDIRS	+= graphics
else
OTHERDIRS	+= graphics
endif

# CLEANDIRS are the directories that will clean in.  These are
#   all directories that we know about.
# KERNDEPDIRS are the directories in which we will build target dependencies.
#   If NuttX and applications are built separately (CONFIG_NUTTX_KERNEL),
#   then this holds only the directories containing kernel files.
# USERDEPDIRS. If NuttX and applications are built separately (CONFIG_NUTTX_KERNEL),
#   then this holds only the directories containing user files.

CLEANDIRS	= $(NONFSDIRS) $(FSDIRS) $(USERDIRS) $(OTHERDIRS)
KERNDEPDIRS	= $(NONFSDIRS)
USERDEPDIRS	= $(USERDIRS)

# Add file system directories to KERNDEPDIRS (they are already in CLEANDIRS)

ifeq ($(CONFIG_NFILE_DESCRIPTORS),0)
ifeq ($(CONFIG_NET),y)
ifneq ($(CONFIG_NSOCKET_DESCRIPTORS),0)
KERNDEPDIRS	+= fs
endif
KERNDEPDIRS	+= drivers
endif
else
KERNDEPDIRS	+= $(FSDIRS)
endif

# Add networking directories to KERNDEPDIRS and CLEANDIRS

ifeq ($(CONFIG_NET),y)
KERNDEPDIRS	+= net
endif
CLEANDIRS	+= net

#
# Extra objects used in the final link.
#
# Pass 1 1ncremental (relative) link objects should be put into the
# processor-specific source directory (where other link objects will
# be created).  If the pass1 obect is an archive, it could go anywhere.

ifeq ($(CONFIG_BUILD_2PASS),y)
EXTRA_OBJS	+= $(CONFIG_PASS1_OBJECT)
endif

# NUTTXLIBS is the list of NuttX libraries that is passed to the
#   processor-specific Makefile to build the final NuttX target.
#   Libraries in FSDIRS are excluded if file descriptor support
#   is disabled.
# USERLIBS is the list of libraries used to build the final user-space
#   application

NUTTXLIBS	= sched/libsched$(LIBEXT) $(ARCH_SRC)/libarch$(LIBEXT)
USERLIBS	=

# Add libraries for syscall support.  The C library will be needed by
# both the kernel- and user-space builds.  For now, the memory manager (mm)
# is placed in user space (only).

ifeq ($(CONFIG_NUTTX_KERNEL),y)
NUTTXLIBS	+= syscall/libstubs$(LIBEXT) lib/libklib$(LIBEXT)
USERLIBS	+= syscall/libproxies$(LIBEXT) lib/libulib$(LIBEXT) mm/libmm$(LIBEXT)
else
NUTTXLIBS	+= mm/libmm$(LIBEXT) lib/liblib$(LIBEXT)
endif

# Add libraries for C++ support.  CXX, CXXFLAGS, and COMPILEXX must
# be defined in Make.defs for this to work!

ifeq ($(CONFIG_HAVE_CXX),y)
ifeq ($(CONFIG_NUTTX_KERNEL),y)
USERLIBS	+= libxx/liblibxx$(LIBEXT)
else
NUTTXLIBS	+= libxx/liblibxx$(LIBEXT)
endif
endif

# Add library for application support.

ifneq ($(APPDIR),)
ifeq ($(CONFIG_NUTTX_KERNEL),y)
USERLIBS	+= $(APPDIR)/libapps$(LIBEXT)
else
NUTTXLIBS	+= $(APPDIR)/libapps$(LIBEXT)
endif
endif

# Add libraries for network support

ifeq ($(CONFIG_NET),y)
NUTTXLIBS	+= net/libnet$(LIBEXT)
endif

# Add libraries for file system support

ifeq ($(CONFIG_NFILE_DESCRIPTORS),0)
ifneq ($(CONFIG_NSOCKET_DESCRIPTORS),0)
NUTTXLIBS	+= fs/libfs$(LIBEXT)
endif
ifeq ($(CONFIG_NET),y)
NUTTXLIBS	+= drivers/libdrivers$(LIBEXT)
endif
else
NUTTXLIBS	+= fs/libfs$(LIBEXT) drivers/libdrivers$(LIBEXT) binfmt/libbinfmt$(LIBEXT)
endif

# Add libraries for the NX graphics sub-system

ifneq ($(NX_DIR),)
NUTTXLIBS	+= $(NX_DIR)/libnx$(LIBEXT)
endif

ifeq ($(CONFIG_NX),y)
NUTTXLIBS        += graphics/libgraphics$(LIBEXT)
endif

# This is the name of the final target (relative to the top level directorty)

BIN		= nuttx$(EXEEXT)

all: $(BIN)
.PHONY: context clean_context check_context export subdir_clean clean subdir_distclean distclean apps_clean apps_distclean

# Targets used to copy include/nuttx/math.h.  If CONFIG_ARCH_MATH_H is
# defined, then there is an architecture specific math.h header file
# that will be included indirectly from include/math.h.  But first, we
# have to copy math.h from include/nuttx/. to include/.

ifeq ($(CONFIG_ARCH_MATH_H),y)
include/math.h: include/nuttx/math.h
	@cp -f include/nuttx/math.h include/math.h
else
include/math.h:
endif

# Targets used to build include/nuttx/version.h.  Creation of version.h is
# part of the overall NuttX configuration sequency.  Notice that the
# tools/mkversion tool is cuilt and used to create include/nuttx/version.h

tools/mkversion:
	@$(MAKE) -C tools -f Makefile.host TOPDIR="$(TOPDIR)"  mkversion

$(TOPDIR)/.version:
	@if [ ! -f .version ]; then \
		echo "No .version file found, creating one"; \
		tools/version.sh -v 0.0 -b 0 .version; \
		chmod 755 .version; \
	fi

include/nuttx/version.h: $(TOPDIR)/.version tools/mkversion
	@tools/mkversion $(TOPDIR) > include/nuttx/version.h

# Targets used to build include/nuttx/config.h.  Creation of config.h is
# part of the overall NuttX configuration sequency.  Notice that the
# tools/mkconfig tool is cuilt and used to create include/nuttx/config.h

tools/mkconfig:
	@$(MAKE) -C tools -f Makefile.host TOPDIR="$(TOPDIR)"  mkconfig

include/nuttx/config.h: $(TOPDIR)/.config tools/mkconfig
	@tools/mkconfig $(TOPDIR) > include/nuttx/config.h

# dirlinks, and helpers
#
# Directories links.  Most of establishing the NuttX configuration involves
# setting up symbolic links with 'generic' directory names to specific,
# configured directories.
#
# Link the apps/include directory to include/apps

include/apps: Make.defs
ifneq ($(APPDIR),)
	@if [ -d $(TOPDIR)/$(APPDIR)/include ]; then \
		$(DIRLINK) $(TOPDIR)/$(APPDIR)/include include/apps; \
	fi
endif

# Link the arch/<arch-name>/include directory to include/arch

include/arch: Make.defs
	@$(DIRLINK) $(TOPDIR)/$(ARCH_DIR)/include include/arch

# Link the configs/<board-name>/include directory to include/arch/board

include/arch/board: include/arch Make.defs include/arch
	@$(DIRLINK) $(TOPDIR)/$(BOARD_DIR)/include include/arch/board

# Link the configs/<board-name>/src dir to arch/<arch-name>/src/board

$(ARCH_SRC)/board: Make.defs
	@$(DIRLINK) $(TOPDIR)/$(BOARD_DIR)/src $(ARCH_SRC)/board

# Link arch/<arch-name>/include/<chip-name> to arch/<arch-name>/include/chip

$(ARCH_SRC)/chip: Make.defs
ifneq ($(CONFIG_ARCH_CHIP),)
	@$(DIRLINK) $(TOPDIR)/$(ARCH_SRC)/$(CONFIG_ARCH_CHIP) $(ARCH_SRC)/chip
endif

# Link arch/<arch-name>/src/<chip-name> to arch/<arch-name>/src/chip

include/arch/chip: include/arch Make.defs
ifneq ($(CONFIG_ARCH_CHIP),)
	@$(DIRLINK) $(TOPDIR)/$(ARCH_INC)/$(CONFIG_ARCH_CHIP) include/arch/chip
endif

dirlinks: include/arch include/arch/board include/arch/chip $(ARCH_SRC)/board $(ARCH_SRC)/chip include/apps

# context
#
# The context target is invoked on each target build to assure that NuttX is
# properly configured.  The basic configuration steps include creation of the
# the config.h and version.h header files in the include/nuttx directory and
# the establishment of symbolic links to configured directories.

context: check_context include/nuttx/config.h include/nuttx/version.h include/math.h dirlinks
	@for dir in $(CONTEXTDIRS) ; do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" context; \
	done

# clean_context
#
# This is part of the distclean target.  It removes all of the header files
# and symbolic links created by the context target.

clean_context:
	@rm -f include/nuttx/config.h
	@rm -f include/nuttx/version.h
	@rm -f include/math.h
	@$(DIRUNLINK) include/arch/board
	@$(DIRUNLINK) include/arch/chip
	@$(DIRUNLINK) include/arch
	@$(DIRUNLINK) $(ARCH_SRC)/board
	@$(DIRUNLINK) $(ARCH_SRC)/chip
	@$(DIRUNLINK) include/apps

# check_context
#
# This target checks if NuttX has been configured.  NuttX is configured using
# the script tools/configure.sh.  That script will install certain files in
# the top-level NuttX build directory.  This target verifies that those
# configuration files have been installed and that NuttX is ready to be built.

check_context:
	@if [ ! -e ${TOPDIR}/.config -o ! -e ${TOPDIR}/Make.defs ]; then \
		echo "" ; echo "Nuttx has not been configured:" ; \
		echo "  cd tools; ./configure.sh <target>" ; echo "" ; \
		exit 1 ; \
	fi

# Archive targets.  The target build sequency will first create a series of
# libraries, one per configured source file directory.  The final NuttX
# execution will then be built from those libraries.  The following targets
# built those libraries.
#
# Possible kernel-mode builds

lib/libklib$(LIBEXT): context
	@$(MAKE) -C lib TOPDIR="$(TOPDIR)" libklib$(LIBEXT) EXTRADEFINES=$(KDEFINE)

sched/libsched$(LIBEXT): context
	@$(MAKE) -C sched TOPDIR="$(TOPDIR)" libsched$(LIBEXT) EXTRADEFINES=$(KDEFINE)

$(ARCH_SRC)/libarch$(LIBEXT): context
	@$(MAKE) -C $(ARCH_SRC) TOPDIR="$(TOPDIR)" libarch$(LIBEXT) EXTRADEFINES=$(KDEFINE)

net/libnet$(LIBEXT): context
	@$(MAKE) -C net TOPDIR="$(TOPDIR)" libnet$(LIBEXT) EXTRADEFINES=$(KDEFINE)

fs/libfs$(LIBEXT): context
	@$(MAKE) -C fs TOPDIR="$(TOPDIR)" libfs$(LIBEXT) EXTRADEFINES=$(KDEFINE)

drivers/libdrivers$(LIBEXT): context
	@$(MAKE) -C drivers TOPDIR="$(TOPDIR)" libdrivers$(LIBEXT) EXTRADEFINES=$(KDEFINE)

binfmt/libbinfmt$(LIBEXT): context
	@$(MAKE) -C binfmt TOPDIR="$(TOPDIR)" libbinfmt$(LIBEXT) EXTRADEFINES=$(KDEFINE)

graphics/libgraphics$(LIBEXT): context
	@$(MAKE) -C graphics TOPDIR="$(TOPDIR)" libgraphics$(LIBEXT) EXTRADEFINES=$(KDEFINE)

syscall/libstubs$(LIBEXT): context
	@$(MAKE) -C syscall TOPDIR="$(TOPDIR)" libstubs$(LIBEXT) EXTRADEFINES=$(KDEFINE)

# Possible user-mode builds

lib/libulib$(LIBEXT): context
	@$(MAKE) -C lib TOPDIR="$(TOPDIR)" libulib$(LIBEXT)

libxx/liblibxx$(LIBEXT): context
	@$(MAKE) -C libxx TOPDIR="$(TOPDIR)" liblibxx$(LIBEXT)

mm/libmm$(LIBEXT): context
	@$(MAKE) -C mm TOPDIR="$(TOPDIR)" libmm$(LIBEXT) EXTRADEFINES=$(KDEFINE)

$(APPDIR)/libapps$(LIBEXT): context
	@$(MAKE) -C $(APPDIR) TOPDIR="$(TOPDIR)" libapps$(LIBEXT)

syscall/libproxies$(LIBEXT): context
	@$(MAKE) -C syscall TOPDIR="$(TOPDIR)" libproxies$(LIBEXT)

# Possible non-kernel builds

lib/liblib$(LIBEXT): context
	@$(MAKE) -C lib TOPDIR="$(TOPDIR)" liblib$(LIBEXT)

# pass1 and pass2
#
# If the 2 pass build option is selected, then this pass1 target is
# configured to built before the pass2 target.  This pass1 target may, as an
# example, build an extra link object (CONFIG_PASS1_OBJECT) which may be an
# incremental (relative) link object, but could be a static library (archive);
# some modification to this Makefile would be required if CONFIG_PASS1_OBJECT
# is an archive.  Exactly what is performed during pass1 or what it generates
# is unknown to this makefule unless CONFIG_PASS1_OBJECT is defined.

pass1deps: context pass1dep $(USERLIBS)

pass1: pass1deps
ifeq ($(CONFIG_BUILD_2PASS),y)
	@if [ -z "$(CONFIG_PASS1_BUILDIR)" ]; then \
		echo "ERROR: CONFIG_PASS1_BUILDIR not defined"; \
		exit 1; \
	fi
	@if [ ! -d "$(CONFIG_PASS1_BUILDIR)" ]; then \
		echo "ERROR: CONFIG_PASS1_BUILDIR does not exist"; \
		exit 1; \
	fi
	@if [ ! -f "$(CONFIG_PASS1_BUILDIR)/Makefile" ]; then \
		echo "ERROR: No Makefile in CONFIG_PASS1_BUILDIR"; \
		exit 1; \
	fi
	@$(MAKE) -C $(CONFIG_PASS1_BUILDIR) TOPDIR="$(TOPDIR)" LINKLIBS="$(NUTTXLIBS)" USERLIBS="$(USERLIBS)" "$(CONFIG_PASS1_TARGET)"
endif

pass2deps: context pass2dep $(NUTTXLIBS)

pass2: pass2deps
	@$(MAKE) -C $(ARCH_SRC) TOPDIR="$(TOPDIR)" EXTRA_OBJS="$(EXTRA_OBJS)" LINKLIBS="$(NUTTXLIBS)" EXTRADEFINES=$(KDEFINE) $(BIN)
	@if [ -w /tftpboot ] ; then \
		cp -f $(BIN) /tftpboot/$(BIN).${CONFIG_ARCH}; \
	fi
ifeq ($(CONFIG_RRLOAD_BINARY),y)
	@echo "MK: $(BIN).rr"
	@$(TOPDIR)/tools/mkimage.sh --Prefix $(CROSSDEV) $(BIN) $(BIN).rr
	@if [ -w /tftpboot ] ; then \
		cp -f $(BIN).rr /tftpboot/$\(BIN).rr.$(CONFIG_ARCH); \
	fi
endif
ifeq ($(CONFIG_INTELHEX_BINARY),y)
	@echo "CP: $(BIN).hex"
	@$(OBJCOPY) $(OBJCOPYARGS) -O ihex $(BIN) $(BIN).hex
endif
ifeq ($(CONFIG_MOTOROLA_SREC),y)
	@echo "CP: $(BIN).srec"
	@$(OBJCOPY) $(OBJCOPYARGS) -O srec $(BIN) $(BIN).srec
endif
ifeq ($(CONFIG_RAW_BINARY),y)
	@echo "CP: $(BIN).bin"
	@$(OBJCOPY) $(OBJCOPYARGS) -O binary $(BIN) $(BIN).bin
endif

# $(BIN)
#
# Create the final NuttX executable in a two pass build process.  In the
# normal case, all pass1 and pass2 dependencies are created then pass1
# and pass2 targets are built.  However, in some cases, you may need to build
# pass1 depenencies and pass1 first, then build pass2 dependencies and pass2.
# in that case, execute 'make pass1 pass2' from the command line.

$(BIN): pass1deps pass2deps pass1 pass2

# download
#
# This is a helper target that will rebuild NuttX and download it to the target
# system in one step.  The operation of this target depends completely upon
# implementation of the DOWNLOAD command in the user Make.defs file.  It will
# generate an error an error if the DOWNLOAD command is not defined.

download: $(BIN)
	$(call DOWNLOAD, $<)

# pass1dep: Create pass1 build dependencies
# pass2dep: Create pass2 build dependencies

pass1dep: context
	@for dir in $(USERDEPDIRS) ; do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" depend ; \
	done

pass2dep: context
	@for dir in $(KERNDEPDIRS) ; do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" EXTRADEFINES=$(KDEFINE) depend; \
	done

# export
#
# The export target will package the NuttX libraries and header files into
# an exportable package.  Caveats: (1) These needs some extension for the KERNEL
# build; it needs to receive USERLIBS and create a libuser.a). (2) The logic
# in tools/mkexport.sh only supports GCC and, for example, explicitly assumes
# that the archiver is 'ar'

export: pass2deps
	@tools/mkexport.sh -w$(WINTOOL) -t "$(TOPDIR)" -l "$(NUTTXLIBS)"

# General housekeeping targets:  dependencies, cleaning, etc.
#
# depend:    Create both PASS1 and PASS2 dependencies
# clean:     Removes derived object files, archives, executables, and
#            temporary files, but retains the configuration and context
#            files and directories.
# distclean: Does 'clean' then also removes all configuration and context
#            files.  This essentially restores the directory structure
#            to its original, unconfigured stated.

depend: pass1dep pass2dep

subdir_clean:
	@for dir in $(CLEANDIRS) ; do \
		if [ -e $$dir/Makefile ]; then \
			$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" clean ; \
		fi \
	done
	@$(MAKE) -C tools -f Makefile.host TOPDIR="$(TOPDIR)" clean
	@$(MAKE) -C mm -f Makefile.test TOPDIR="$(TOPDIR)" clean
ifeq ($(CONFIG_BUILD_2PASS),y)
	@$(MAKE) -C $(CONFIG_PASS1_BUILDIR) TOPDIR="$(TOPDIR)" clean
endif

clean: subdir_clean
	@rm -f $(BIN) nuttx.* mm_test *.map _SAVED_APPS_config *~
	@rm -f nuttx-export*

subdir_distclean:
	@for dir in $(CLEANDIRS) ; do \
		if [ -e $$dir/Makefile ]; then \
			$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" distclean ; \
		fi \
	done

distclean: clean subdir_distclean clean_context
ifeq ($(CONFIG_BUILD_2PASS),y)
	@$(MAKE) -C $(CONFIG_PASS1_BUILDIR) TOPDIR="$(TOPDIR)" distclean
endif
	@rm -f Make.defs setenv.sh .config

# Application housekeeping targets.  The APPDIR variable refers to the user
# application directory.  A sample apps/ directory is included with NuttX,
# however, this is not treated as part of NuttX and may be replaced with a
# different application directory.  For the most part, the application
# directory is treated like any other build directory in this script.  However,
# as a convenience, the following targets are included to support housekeeping
# functions in the user application directory from the NuttX build directory.
#
# apps_clean:     Perform the clean operation only in the user application
#                 directory
# apps_distclean: Perform the distclean operation only in the user application
#                 directory.  Note that the apps/.config file is preserved
#                 so that this is not a "full" distclean but more of a
#                 configuration "reset."

apps_clean:
ifneq ($(APPDIR),)
	@$(MAKE) -C "$(TOPDIR)/$(APPDIR)" TOPDIR="$(TOPDIR)" clean
endif

apps_distclean:
ifneq ($(APPDIR),)
	@cp "$(TOPDIR)/$(APPDIR)/.config" _SAVED_APPS_config || \
		{ echo "Copy of $(APPDIR)/.config failed" ; exit 1 ; }
	@$(MAKE) -C "$(TOPDIR)/$(APPDIR)" TOPDIR="$(TOPDIR)" distclean
	@mv _SAVED_APPS_config "$(TOPDIR)/$(APPDIR)/.config" || \
		{ echo "Copy of _SAVED_APPS_config failed" ; exit 1 ; }
endif

ARCH ?=sim
menuconfig:
	SRCARCH=${ARCH} APPSDIR=${CONFIG_APPS_DIR} mconf Kconfig
	
