#
# Generic Makefile for PX4 firmware.
#
# Currently this assumes that we're just compiling SRCS
# and then linking the whole thing together.
#
# Requires:
#
# PLATFORM
#	Must be set to a platform name known to the PX4 distribution.
#
# Optional:
#
# PX4_BASE:
#	Points to a PX4 distribution. Normally determined based on the
#	path to this file.
#
# CONFIG:
#	Used to set the output filename; defaults to 'firmware'.
#
# WORK_DIR:
#	Sets the directory in which the firmware will be built. Defaults
#	to the directory 'build' under the directory containing the
#	parent Makefile.
#

################################################################################
# Paths and configuration
################################################################################

#
# Work out where this file is, so we can find other makefiles in the
# same directory.
#
# If PX4_BASE wasn't set previously, work out what it should be
# and set it here now.
#
export PX4_MK_INCLUDE	?= $(dir $(lastword $(MAKEFILE_LIST)))
ifeq ($(PX4_BASE),)
export PX4_BASE		:= $(abspath $(PX4_MK_INCLUDE)/..)
endif
$(info %% PX4_BASE $(PX4_BASE))

#
# If WORK_DIR is not set, create a 'build' directory next to the
# parent Makefile.
#
PARENT_MAKEFILE		:= $(lastword $(filter-out $(lastword $(MAKEFILE_LIST)),$(MAKEFILE_LIST)))
ifeq ($(WORK_DIR),)
export WORK_DIR		:= $(dir $(PARENT_MAKEFILE))/build
endif
$(info %% WORK_DIR $(WORK_DIR))

#
# Paths
#
export NUTTX_SRC	 = $(PX4_BASE)/nuttx
export NUTTX_APPS	 = $(PX4_BASE)/apps
export MAVLINK_SRC	 = $(PX4_BASE)/mavlink
export ROMFS_SRC	 = $(PX4_BASE)/ROMFS
export IMAGE_DIR	 = $(PX4_BASE)/Images
export BUILD_DIR	 = $(PX4_BASE)/Build
export ARCHIVE_DIR	 = $(PX4_BASE)/Archives

#
# Extra tools.
#
# XXX should be in a common toolchain config somewhere.
#
MKFW			 = $(PX4_BASE)/Tools/px_mkfw.py
COPY			 = cp
REMOVE			 = rm -f
RMDIR			 = rm -rf

#
# Sanity-check the PLATFORM variable and then get the platform config.
#
# The platform config in turn will fetch the toolchain configuration.
#
ifeq ($(PLATFORM),)
$(error The PLATFORM variable must be set before including firmware.mk)
endif
include $(PX4_MK_INCLUDE)/$(PLATFORM).mk

#
# Makefile debugging.
#
Q			:= $(if $(V),,@)

#
# Host-specific paths, hacks and fixups
#
SYSTYPE			:= $(shell uname -s)

ifeq ($(SYSTYPE),Darwin)
# Eclipse may not have the toolchain on its path.
export PATH		:= $(PATH):/usr/local/bin
endif

################################################################################
# NuttX libraries and paths
################################################################################

#
# Check that the NuttX archive for the selected platform is available.
#
NUTTX_ARCHIVE		:= $(wildcard $(ARCHIVE_DIR)/$(PLATFORM).export)
ifeq ($(NUTTX_ARCHIVE),)
$(error The NuttX export archive for $(PLATFORM) is missing from $(ARCHIVE_DIR) - try 'make archives' in $(PX4_BASE))
endif

#
# The NuttX config header should always be present in the NuttX archive, and
# if it changes, everything should be rebuilt. So, use it as the trigger to
# unpack the NuttX archive.
#
NUTTX_EXPORT_DIR	 = $(WORK_DIR)/nuttx-export
NUTTX_CONFIG_HEADER	 = $(NUTTX_EXPORT_DIR)/include/nuttx/config.h

#
# Use the linker script from the NuttX export
#
LDSCRIPT		 = $(NUTTX_EXPORT_DIR)/build/ld.script

#
# Add directories from the NuttX export to the relevant search paths
#
INCLUDE_DIRS		+= $(NUTTX_EXPORT_DIR)/include
LIB_DIRS		+= $(NUTTX_EXPORT_DIR)/libs
LIBS			+= -lapps -lnuttx
LINK_DEPS		+= $(wildcard $(addsuffix /*.a,$(LIB_DIRS)))

################################################################################
# Build rules
################################################################################

#
# Things that, if they change, might affect everything
#
GLOBAL_DEPS		+= $(MAKEFILE_LIST) $(NUTTX_CONFIG_HEADER)

#
# What we're going to build.
#
PRODUCT_BUNDLE		 = $(WORK_DIR)/firmware.px4
PRODUCT_BIN		 = $(WORK_DIR)/firmware.bin
PRODUCT_SYM		 = $(WORK_DIR)/firmware.sym

.PHONY:			all
all:			$(PRODUCT_BUNDLE)

#
# Object files we will generate from sources
#
OBJS			 = $(foreach src,$(SRCS),$(WORK_DIR)/$(src).o)

#
# Rules
#

$(filter %.c.o,$(OBJS)): $(WORK_DIR)/%.c.o: %.c $(GLOBAL_DEPS)
	@mkdir -p $(dir $@)
	$(call COMPILE,$<,$@)

$(filter %.cpp.o,$(OBJS)): $(WORK_DIR)/%.cpp.o: %.cpp $(GLOBAL_DEPS)
	@mkdir -p $(dir $@)
	$(call COMPILEXX,$<,$@)

$(filter %.S.o,$(OBJS)): $(WORK_DIR)/%.S.o: %.S $(GLOBAL_DEPS)
	@mkdir -p $(dir $@)
	$(call ASSEMBLE,$<,$@)

$(NUTTX_CONFIG_HEADER):	$(NUTTX_ARCHIVE)
	@echo %% Unpacking $(NUTTX_ARCHIVE)
	$(Q) unzip -q -o -d $(WORK_DIR) $(NUTTX_ARCHIVE)
	$(Q) touch $@

$(PRODUCT_BUNDLE):	$(PRODUCT_BIN)
	@echo %% Generating $@
	$(Q) $(MKFW) --prototype $(IMAGE_DIR)/$(PLATFORM).prototype \
		--git_identity $(PX4_BASE) \
		--image $< > $@

$(PRODUCT_BIN):		$(PRODUCT_SYM)
	$(call SYM_TO_BIN,$<,$@)

$(PRODUCT_SYM):		$(OBJS) $(GLOBAL_DEPS) $(LINK_DEPS)
	$(call LINK,$@,$(OBJS))

upload:	$(PRODUCT_BUNDLE) $(PRODUCT_BIN)
	$(Q) make -f $(PX4_MK_INCLUDE)/upload.mk \
		METHOD=serial \
		PRODUCT=$(PRODUCT) \
		BUNDLE=$(PRODUCT_BUNDLE) \
		BIN=$(PRODUCT_BIN)

clean:
	@echo %% cleaning
	$(Q) $(REMOVE) $(PRODUCT_BUNDLE) $(PRODUCT_BIN) $(PRODUCT_SYM)
	$(Q) $(REMOVE) $(OBJS) $(DEP_INCLUDES)
	$(Q) $(RMDIR) $(NUTTX_EXPORT_DIR)

#
# DEP_INCLUDES is defined by the toolchain include in terms of $(OBJS)
#
-include $(DEP_INCLUDES)
