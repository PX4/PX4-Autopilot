#
#   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
# Generic Makefile for PX4 firmware images.
#
# Requires:
#
# BOARD
#	Must be set to a board name known to the PX4 distribution (as
#	we need a corresponding NuttX export archive to link with).
#
# Optional:
#
# MODULES
#	Contains a list of module paths or path fragments used
#	to find modules. The names listed here are searched in
#	the following directories:
#		<absolute path>
#		$(MODULE_SEARCH_DIRS)
#		WORK_DIR
#		MODULE_SRC
#		PX4_MODULE_SRC
#
#	Application directories are expected to contain a module.mk
#	file which provides build configuration for the module. See
#	makefiles/module.mk for more details.
#
# BUILTIN_COMMANDS
#	Contains a list of built-in commands not explicitly provided
#	by modules / libraries. Each entry in this list is formatted
#	as <command>.<priority>.<stacksize>.<entrypoint>
#
# PX4_BASE:
#	Points to a PX4 distribution. Normally determined based on the
#	path to this file.
#
# CONFIG:
#	Used when searching for the configuration file, and available
#	to module Makefiles to select optional features.
#	If not set, CONFIG_FILE must be set and CONFIG will be derived
#	automatically from it.
#
# CONFIG_FILE:
#	If set, overrides the configuration file search logic. Sets
#	CONFIG to the name of the configuration file, strips any
#	leading config_ prefix and any suffix. e.g. config_board_foo.mk
#	results in CONFIG being set to 'board_foo'.
#
# WORK_DIR:
#	Sets the directory in which the firmware will be built. Defaults
#	to the directory 'build' under the directory containing the
#	parent Makefile.
#
# ROMFS_ROOT:
#	If set to the path to a directory, a ROMFS image will be generated
#	containing the files under the directory and linked into the final
#	image.
#
# MODULE_SEARCH_DIRS:
#	Extra directories to search first for MODULES before looking in the
#	usual places.
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
MK_DIR			?= $(dir $(lastword $(MAKEFILE_LIST)))
ifeq ($(PX4_BASE),)
export PX4_BASE		:= $(abspath $(MK_DIR)/..)
endif
$(info %  PX4_BASE            = $(PX4_BASE))
ifneq ($(words $(PX4_BASE)),1)
$(error Cannot build when the PX4_BASE path contains one or more space characters.)
endif

$(info %  GIT_DESC            = $(GIT_DESC))

#
# Set a default target so that included makefiles or errors here don't
# cause confusion.
#
# XXX We could do something cute here with $(DEFAULT_GOAL) if it's not one
#     of the maintenance targets and set CONFIG based on it.
#
all:		firmware

#
# Get path and tool config
#
include $(MK_DIR)/setup.mk

#
# Locate the configuration file
#
ifneq ($(CONFIG_FILE),)
CONFIG			:= $(subst config_,,$(basename $(notdir $(CONFIG_FILE))))
else
CONFIG_FILE		:= $(wildcard $(PX4_MK_DIR)/config_$(CONFIG).mk)
endif
ifeq ($(CONFIG),)
$(error Missing configuration name or file (specify with CONFIG=<config>))
endif
export CONFIG
include $(CONFIG_FILE)
$(info %  CONFIG              = $(CONFIG))

#
# Sanity-check the BOARD variable and then get the board config.
# If BOARD was not set by the configuration, extract it automatically.
#
# The board config in turn will fetch the toolchain configuration.
#
ifeq ($(BOARD),)
BOARD			:= $(firstword $(subst _, ,$(CONFIG)))
endif
BOARD_FILE		:= $(wildcard $(PX4_MK_DIR)/board_$(BOARD).mk)
ifeq ($(BOARD_FILE),)
$(error Config $(CONFIG) references board $(BOARD), but no board definition file found)
endif
export BOARD
export BOARD_FILE
include $(BOARD_FILE)
$(info %  BOARD               = $(BOARD))

#
# If WORK_DIR is not set, create a 'build' directory next to the
# parent Makefile.
#
PARENT_MAKEFILE		:= $(lastword $(filter-out $(lastword $(MAKEFILE_LIST)),$(MAKEFILE_LIST)))
ifeq ($(WORK_DIR),)
export WORK_DIR		:= $(dir $(PARENT_MAKEFILE))build/
endif
$(info %  WORK_DIR            = $(WORK_DIR))

#
# Things that, if they change, might affect everything
#
GLOBAL_DEPS		+= $(MAKEFILE_LIST)

#
# Extra things we should clean
#
EXTRA_CLEANS		 =


#
# Append the per-board driver directory to the header search path.
#
INCLUDE_DIRS		+= $(PX4_MODULE_SRC)drivers/boards/$(BOARD)

################################################################################
# NuttX libraries and paths
################################################################################

include $(PX4_MK_DIR)/nuttx.mk

################################################################################
# Modules
################################################################################

# where to look for modules
MODULE_SEARCH_DIRS	+= $(WORK_DIR) $(MODULE_SRC) $(PX4_MODULE_SRC)

# sort and unique the modules list
MODULES			:= $(sort $(MODULES))

# locate the first instance of a module by full path or by looking on the
# module search path
define MODULE_SEARCH
	$(firstword $(abspath $(wildcard $(1)/module.mk)) \
		$(abspath $(foreach search_dir,$(MODULE_SEARCH_DIRS),$(wildcard $(search_dir)/$(1)/module.mk))) \
		MISSING_$1)
endef

# make a list of module makefiles and check that we found them all
MODULE_MKFILES		:= $(foreach module,$(MODULES),$(call MODULE_SEARCH,$(module)))
MISSING_MODULES		:= $(subst MISSING_,,$(filter MISSING_%,$(MODULE_MKFILES)))
ifneq ($(MISSING_MODULES),)
$(error Can't find module(s): $(MISSING_MODULES))
endif

# Make a list of the object files we expect to build from modules
# Note that this path will typically contain a double-slash at the WORK_DIR boundary; this must be
# preserved as it is used below to get the absolute path for the module.mk file correct.
#
MODULE_OBJS		:= $(foreach path,$(dir $(MODULE_MKFILES)),$(WORK_DIR)$(path)module.pre.o)

# rules to build module objects
.PHONY: $(MODULE_OBJS)
$(MODULE_OBJS):		relpath = $(patsubst $(WORK_DIR)%,%,$@)
$(MODULE_OBJS):		mkfile = $(patsubst %module.pre.o,%module.mk,$(relpath))
$(MODULE_OBJS):		workdir = $(@D)
$(MODULE_OBJS):		$(GLOBAL_DEPS) $(NUTTX_CONFIG_HEADER)
	$(Q) $(MKDIR) -p $(workdir)
	$(Q) $(MAKE) -r -f $(PX4_MK_DIR)module.mk \
		-C $(workdir) \
		MODULE_WORK_DIR=$(workdir) \
		MODULE_OBJ=$@ \
		MODULE_MK=$(mkfile) \
		MODULE_NAME=$(lastword $(subst /, ,$(workdir))) \
		module

# make a list of phony clean targets for modules
MODULE_CLEANS		:= $(foreach path,$(dir $(MODULE_MKFILES)),$(WORK_DIR)$(path)/clean)

# rules to clean modules
.PHONY: $(MODULE_CLEANS)
$(MODULE_CLEANS):	relpath = $(patsubst $(WORK_DIR)%,%,$@)
$(MODULE_CLEANS):	mkfile = $(patsubst %clean,%module.mk,$(relpath))
$(MODULE_CLEANS):
	@$(ECHO) %% cleaning using $(mkfile)
	$(Q) $(MAKE) -r -f $(PX4_MK_DIR)module.mk \
	MODULE_WORK_DIR=$(dir $@) \
	MODULE_MK=$(mkfile) \
	clean

################################################################################
# Libraries
################################################################################

# where to look for libraries
LIBRARY_SEARCH_DIRS	+= $(WORK_DIR) $(MODULE_SRC) $(PX4_MODULE_SRC)

# sort and unique the library list
LIBRARIES		:= $(sort $(LIBRARIES))

# locate the first instance of a library by full path or by looking on the
# library search path
define LIBRARY_SEARCH
	$(firstword $(abspath $(wildcard $(1)/library.mk)) \
		$(abspath $(foreach search_dir,$(LIBRARY_SEARCH_DIRS),$(wildcard $(search_dir)/$(1)/library.mk))) \
		MISSING_$1)
endef

# make a list of library makefiles and check that we found them all
LIBRARY_MKFILES		:= $(foreach library,$(LIBRARIES),$(call LIBRARY_SEARCH,$(library)))
MISSING_LIBRARIES	:= $(subst MISSING_,,$(filter MISSING_%,$(LIBRARY_MKFILES)))
ifneq ($(MISSING_LIBRARIES),)
$(error Can't find library(s): $(MISSING_LIBRARIES))
endif

# Make a list of the archive files we expect to build from libraries
# Note that this path will typically contain a double-slash at the WORK_DIR boundary; this must be
# preserved as it is used below to get the absolute path for the library.mk file correct.
#
LIBRARY_LIBS		:= $(foreach path,$(dir $(LIBRARY_MKFILES)),$(WORK_DIR)$(path)library.a)

# rules to build module objects
.PHONY: $(LIBRARY_LIBS)
$(LIBRARY_LIBS):	relpath = $(patsubst $(WORK_DIR)%,%,$@)
$(LIBRARY_LIBS):	mkfile = $(patsubst %library.a,%library.mk,$(relpath))
$(LIBRARY_LIBS):	workdir = $(@D)
$(LIBRARY_LIBS):	$(GLOBAL_DEPS) $(NUTTX_CONFIG_HEADER)
	$(Q) $(MKDIR) -p $(workdir)
	$(Q) $(MAKE) -r -f $(PX4_MK_DIR)library.mk \
		-C $(workdir) \
		LIBRARY_WORK_DIR=$(workdir) \
		LIBRARY_LIB=$@ \
		LIBRARY_MK=$(mkfile) \
		LIBRARY_NAME=$(lastword $(subst /, ,$(workdir))) \
		library

# make a list of phony clean targets for modules
LIBRARY_CLEANS		:= $(foreach path,$(dir $(LIBRARY_MKFILES)),$(WORK_DIR)$(path)/clean)

# rules to clean modules
.PHONY: $(LIBRARY_CLEANS)
$(LIBRARY_CLEANS):	relpath = $(patsubst $(WORK_DIR)%,%,$@)
$(LIBRARY_CLEANS):	mkfile = $(patsubst %clean,%library.mk,$(relpath))
$(LIBRARY_CLEANS):
	@$(ECHO) %% cleaning using $(mkfile)
	$(Q) $(MAKE) -r -f $(PX4_MK_DIR)library.mk \
	LIBRARY_WORK_DIR=$(dir $@) \
	LIBRARY_MK=$(mkfile) \
	clean

################################################################################
# ROMFS generation
################################################################################

ifneq ($(ROMFS_ROOT),)
ifeq ($(wildcard $(ROMFS_ROOT)),)
$(error ROMFS_ROOT specifies a directory that does not exist)
endif

#
# Note that there is no support for more than one root directory or constructing
# a root from several templates. That would be a nice feature.
#

# Add dependencies on anything in the ROMFS root directory
ROMFS_FILES		+= $(wildcard \
			     $(ROMFS_ROOT)/* \
			     $(ROMFS_ROOT)/*/* \
			     $(ROMFS_ROOT)/*/*/* \
			     $(ROMFS_ROOT)/*/*/*/* \
			     $(ROMFS_ROOT)/*/*/*/*/* \
			     $(ROMFS_ROOT)/*/*/*/*/*/*)
ifeq ($(ROMFS_FILES),)
$(error ROMFS_ROOT $(ROMFS_ROOT) specifies a directory containing no files)
endif
ROMFS_DEPS		+= $(ROMFS_FILES)

# Extra files that may be copied into the ROMFS /extras directory
# ROMFS_EXTRA_FILES are required, ROMFS_OPTIONAL_FILES are optional
ROMFS_EXTRA_FILES	+= $(wildcard $(ROMFS_OPTIONAL_FILES))
ROMFS_DEPS		+= $(ROMFS_EXTRA_FILES)

ROMFS_IMG		 = romfs.img
ROMFS_SCRATCH		 = romfs_scratch
ROMFS_CSRC		 = $(ROMFS_IMG:.img=.c)
ROMFS_OBJ		 = $(ROMFS_CSRC:.c=.o)
LIBS			+= $(ROMFS_OBJ)
LINK_DEPS		+= $(ROMFS_OBJ)

# Remove all comments from startup and mixer files
ROMFS_PRUNER	 = $(PX4_BASE)/Tools/px_romfs_pruner.py

# Turn the ROMFS image into an object file
$(ROMFS_OBJ): $(ROMFS_IMG) $(GLOBAL_DEPS)
	$(call BIN_TO_OBJ,$<,$@,romfs_img)

# Generate the ROMFS image from the root
$(ROMFS_IMG): $(ROMFS_SCRATCH) $(ROMFS_DEPS) $(GLOBAL_DEPS)
	@$(ECHO) "ROMFS:   $@"
	$(Q) $(GENROMFS) -f $@ -d $(ROMFS_SCRATCH) -V "NSHInitVol"

# Construct the ROMFS scratch root from the canonical root
$(ROMFS_SCRATCH): $(ROMFS_DEPS) $(GLOBAL_DEPS)
	$(Q) $(MKDIR) -p $(ROMFS_SCRATCH)
	$(Q) $(COPYDIR) $(ROMFS_ROOT)/* $(ROMFS_SCRATCH)
# delete all files in ROMFS_SCRATCH which start with a . or end with a ~
	$(Q) $(RM) $(ROMFS_SCRATCH)/*/.[!.]* $(ROMFS_SCRATCH)/*/*~
ifneq ($(ROMFS_EXTRA_FILES),)
	$(Q) $(MKDIR) -p $(ROMFS_SCRATCH)/extras
	$(Q) $(COPY) $(ROMFS_EXTRA_FILES) $(ROMFS_SCRATCH)/extras
endif
	$(Q) $(PYTHON) -u $(ROMFS_PRUNER) --folder $(ROMFS_SCRATCH)

EXTRA_CLEANS		+= $(ROMGS_OBJ) $(ROMFS_IMG)

endif

################################################################################
# Builtin command list generation
################################################################################

#
# Builtin commands can be generated by the configuration, in which case they
# must refer to commands that already exist, or indirectly generated by modules
# when they are built.
#
# The configuration supplies builtin command information in the BUILTIN_COMMANDS
# variable. Applications make empty files in $(WORK_DIR)/builtin_commands whose
# filename contains the same information.
#
# In each case, the command information consists of four fields separated with a
# period. These fields are the command's name, its thread priority, its stack size
# and the name of the function to call when starting the thread.
#
BUILTIN_CSRC		 = $(WORK_DIR)builtin_commands.c

# command definitions from modules (may be empty at Makefile parsing time...)
MODULE_COMMANDS		 = $(subst COMMAND.,,$(notdir $(wildcard $(WORK_DIR)builtin_commands/COMMAND.*)))

# We must have at least one pre-defined builtin command in order to generate
# any of this.
#
ifneq ($(BUILTIN_COMMANDS),)

# (BUILTIN_PROTO,<cmdspec>,<outputfile>)
define BUILTIN_PROTO
	$(ECHO) 'extern int $(word 4,$1)(int argc, char *argv[]);' >> $2;
endef

# (BUILTIN_DEF,<cmdspec>,<outputfile>)
define BUILTIN_DEF
	$(ECHO) '    {"$(word 1,$1)", $(word 2,$1), $(word 3,$1), $(word 4,$1)},' >> $2;
endef

# Don't generate until modules have updated their command files
$(BUILTIN_CSRC):	$(GLOBAL_DEPS) $(MODULE_OBJS) $(MODULE_MKFILES) $(BUILTIN_COMMAND_FILES)
	@$(ECHO) "CMDS:    $@"
	$(Q) $(ECHO) '/* builtin command list - automatically generated, do not edit */' > $@
	$(Q) $(ECHO) '#include <nuttx/config.h>' >> $@
	$(Q) $(ECHO) '#include <nuttx/binfmt/builtin.h>' >> $@
	$(Q) $(foreach spec,$(BUILTIN_COMMANDS),$(call BUILTIN_PROTO,$(subst ., ,$(spec)),$@))
	$(Q) $(foreach spec,$(MODULE_COMMANDS),$(call BUILTIN_PROTO,$(subst ., ,$(spec)),$@))
	$(Q) $(ECHO) 'const struct builtin_s g_builtins[] = {' >> $@
	$(Q) $(foreach spec,$(BUILTIN_COMMANDS),$(call BUILTIN_DEF,$(subst ., ,$(spec)),$@))
	$(Q) $(foreach spec,$(MODULE_COMMANDS),$(call BUILTIN_DEF,$(subst ., ,$(spec)),$@))
	$(Q) $(ECHO) '    {NULL, 0, 0, NULL}' >> $@
	$(Q) $(ECHO) '};' >> $@
	$(Q) $(ECHO) 'const int g_builtin_count = $(words $(BUILTIN_COMMANDS) $(MODULE_COMMANDS));' >> $@

SRCS			+= $(BUILTIN_CSRC)

EXTRA_CLEANS		+= $(BUILTIN_CSRC)

endif

################################################################################
# Default SRCS generation
################################################################################

#
# If there are no SRCS, the build will fail; in that case, generate an empty
# source file.
#
ifeq ($(SRCS),)
EMPTY_SRC		 = $(WORK_DIR)empty.c
$(EMPTY_SRC):
	$(Q) $(ECHO) '/* this is an empty file */' > $@

SRCS			+= $(EMPTY_SRC)
endif

################################################################################
# Build rules
################################################################################

#
# What we're going to build.
#
PRODUCT_BUNDLE		 = $(WORK_DIR)firmware.px4
PRODUCT_BIN		 = $(WORK_DIR)firmware.bin
PRODUCT_ELF		 = $(WORK_DIR)firmware.elf
PRODUCT_PARAMXML = $(WORK_DIR)/parameters.xml
PRODUCT_AIRFRAMEXML = $(WORK_DIR)/airframes.xml

.PHONY:			firmware
firmware:		$(PRODUCT_BUNDLE)

#
# Object files we will generate from sources
#
OBJS			:= $(foreach src,$(SRCS),$(WORK_DIR)$(src).o)

#
# SRCS -> OBJS rules
#

$(OBJS):		$(GLOBAL_DEPS)

$(filter %.c.o,$(OBJS)): $(WORK_DIR)%.c.o: %.c $(GLOBAL_DEPS)
	$(call COMPILE,$<,$@)

$(filter %.cpp.o,$(OBJS)): $(WORK_DIR)%.cpp.o: %.cpp $(GLOBAL_DEPS)
	$(call COMPILEXX,$<,$@)

$(filter %.S.o,$(OBJS)): $(WORK_DIR)%.S.o: %.S $(GLOBAL_DEPS)
	$(call ASSEMBLE,$<,$@)

#
# Built product rules
#

$(PRODUCT_BUNDLE):	$(PRODUCT_BIN)
	@$(ECHO) %% Generating $@
ifdef GEN_PARAM_XML
	$(Q) $(PYTHON) $(PX4_BASE)/Tools/px_process_params.py --src-path $(PX4_BASE)/src --board CONFIG_ARCH_BOARD_$(CONFIG_BOARD) --xml
	$(Q) $(COPY) $(PX4_BASE)/Tools/airframes.xml $(WORK_DIR)/airframes.xml
	$(Q) $(MKFW) --prototype $(IMAGE_DIR)/$(BOARD).prototype \
		--git_identity $(PX4_BASE) \
		--parameter_xml $(PRODUCT_PARAMXML) \
		--airframe_xml $(PRODUCT_AIRFRAMEXML) \
		--image $< > $@
else
	$(Q) $(MKFW) --prototype $(IMAGE_DIR)/$(BOARD).prototype \
		--git_identity $(PX4_BASE) \
		--image $< > $@
endif

$(PRODUCT_BIN):		$(PRODUCT_ELF)
	$(call SYM_TO_BIN,$<,$@)

$(PRODUCT_ELF):		$(OBJS) $(MODULE_OBJS) $(LIBRARY_LIBS) $(GLOBAL_DEPS) $(LINK_DEPS) $(MODULE_MKFILES)
	$(call LINK,$@,$(OBJS) $(MODULE_OBJS) $(LIBRARY_LIBS))

#
# Utility rules
#

.PHONY: upload
upload:	$(PRODUCT_BUNDLE) $(PRODUCT_BIN)
	$(Q) $(MAKE) -f $(PX4_MK_DIR)/upload.mk \
		METHOD=serial \
		CONFIG=$(CONFIG) \
		BOARD=$(BOARD) \
		BUNDLE=$(PRODUCT_BUNDLE) \
		BIN=$(PRODUCT_BIN)

.PHONY: clean
clean:			$(MODULE_CLEANS)
	@$(ECHO) %% cleaning
	$(Q) $(REMOVE) $(PRODUCT_BUNDLE) $(PRODUCT_BIN) $(PRODUCT_ELF)
	$(Q) $(REMOVE) $(OBJS) $(DEP_INCLUDES) $(EXTRA_CLEANS)
	$(Q) $(RMDIR) $(NUTTX_EXPORT_DIR)


#
# DEP_INCLUDES is defined by the toolchain include in terms of $(OBJS)
#
-include $(DEP_INCLUDES)
