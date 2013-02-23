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
# APPS
#	Contains a list of application paths or path fragments used
#	to find applications. The names listed here are searched in
#	the following directories:
#		<absolute path>
#		$(APP_SEARCH_DIRS)
#		WORK_DIR
#		APP_SRC
#		PX4_APP_SRC
#
#	Application directories are expected to contain an 'app.mk'
#	file which provides build configuration for the app. See
#	application.mk for more details.
#
# LIBS
#	Contains a list of library paths or path fragments used
#	to find libraries. The names listed here are searched in the
#	following directories:
#		<absolute path>
#		$(LIB_SEARCH_DIRS)
#		WORK_DIR
#		LIB_SRC
#		PX4_LIB_SRC
#
#	Library directories are expected to contain a 'lib.mk'
#	file which provides build configuration for the library. See
#	library.mk for more details.
#
# BUILTIN_COMMANDS
#	Contains a list of built-in commands not explicitly provided
#	by applications / libraries. Each entry in this list is formatted
#	as <command>.<priority>.<stacksize>.<entrypoint>
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
# ROMFS_ROOT:
#	If set to the path to a directory, a ROMFS image will be generated
#	containing the files under the directory and linked into the final
#	image.
#
# APP_SEARCH_DIRS:
#	Extra directories to search first for APPS before looking in the
#	usual places.
#
# LIB_SEARCH_DIRS:
#	Extra directories to search first for LIBS before looking in the
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
MK_DIR	?= $(dir $(lastword $(MAKEFILE_LIST)))
ifeq ($(PX4_BASE),)
export PX4_BASE		:= $(abspath $(MK_DIR)/..)
$(info %% set PX4_BASE to $(PX4_BASE))
endif

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
ifeq ($(CONFIG),)
$(error Missing configuration name or file (specify with CONFIG=<config>))
endif
CONFIG_FILE		:= $(firstword $(wildcard $(CONFIG)) $(wildcard $(PX4_MK_DIR)/config_$(CONFIG).mk))
ifeq ($(CONFIG_FILE),)
$(error Can't find a config file called $(CONFIG) or $(PX4_MK_DIR)/config_$(CONFIG).mk)
endif
export CONFIG
include $(CONFIG_FILE)
$(info %% CONFIG     $(CONFIG))

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
include $(BOARD_FILE)
$(info %% BOARD      $(BOARD))

#
# If WORK_DIR is not set, create a 'build' directory next to the
# parent Makefile.
#
PARENT_MAKEFILE		:= $(lastword $(filter-out $(lastword $(MAKEFILE_LIST)),$(MAKEFILE_LIST)))
ifeq ($(WORK_DIR),)
export WORK_DIR		:= $(dir $(PARENT_MAKEFILE))/build
endif
$(info %% WORK_DIR   $(WORK_DIR))

#
# Things that, if they change, might affect everything
#
GLOBAL_DEPS		+= $(MAKEFILE_LIST)

################################################################################
# Applications
################################################################################

# where to look for applications
APP_SEARCH_DIRS		 += $(WORK_DIR) $(APP_SRC) $(PX4_APP_SRC)

# sort and unique the apps list
APPS			:= $(sort $(APPS))

# locate the first instance of an app by full path or by looking on the
# application search path
define APP_SEARCH
	$(firstword $(wildcard $(1)/app.mk) \
		$(foreach search_dir,$(APP_SEARCH_DIRS),$(wildcard $(search_dir)/$(1)/app.mk)) \
		MISSING_$1)
endef

APP_MKFILES		:= $(foreach app,$(APPS),$(call APP_SEARCH,$(app)))
MISSING_APPS		:= $(subst MISSING_,,$(filter MISSING_%,$(APP_MKFILES)))
ifneq ($(MISSING_APPS),)
$(error Can't find application(s): $(MISSING_APPS))
endif

APP_OBJS		:= $(foreach path,$(dir $(APP_MKFILES)),$(WORK_DIR)/$(path)/app.pre.o)

$(APP_OBJS):		relpath = $(patsubst $(WORK_DIR)/%,%,$@)
$(APP_OBJS):		mkfile = $(patsubst %/app.pre.o,%/app.mk,$(relpath))
$(APP_OBJS):		$(GLOBAL_DEPS) $(NUTTX_CONFIG_HEADER)
	@echo %%
	@echo %% Building app in $(relpath) using $(mkfile)
	@echo %%
	$(Q) make -f $(PX4_MK_DIR)/application.mk \
		APP_WORK_DIR=$(dir $@) \
		APP_OBJ=$@ \
		APP_MK=$(mkfile) \
		app

APP_CLEANS		:= $(foreach path,$(dir $(APP_MKFILES)),$(WORK_DIR)/$(path)/clean)

.PHONY: $(APP_CLEANS)
$(APP_CLEANS):		relpath = $(patsubst $(WORK_DIR)/%,%,$@)
$(APP_CLEANS):		mkfile = $(patsubst %/clean,%/app.mk,$(relpath))
$(APP_CLEANS):
	@echo %% cleaning using $(mkfile)
	$(Q) make -f $(PX4_MK_DIR)/application.mk \
	APP_WORK_DIR=$(dir $@) \
	APP_MK=$(mkfile) \
	clean

################################################################################
# NuttX libraries and paths
################################################################################

include $(PX4_MK_DIR)/nuttx.mk

################################################################################
# ROMFS generation
################################################################################

#
# Note that we can't just put romfs.c in SRCS, as it's depended on by the
# NuttX export library. Instead, we have to treat it like a library.
#
ifneq ($(ROMFS_ROOT),)
ROMFS_DEPS		+= $(wildcard \
			     (ROMFS_ROOT)/* \
			     (ROMFS_ROOT)/*/* \
			     (ROMFS_ROOT)/*/*/* \
			     (ROMFS_ROOT)/*/*/*/* \
			     (ROMFS_ROOT)/*/*/*/*/* \
			     (ROMFS_ROOT)/*/*/*/*/*/*)
ROMFS_IMG		 = $(WORK_DIR)/romfs.img
ROMFS_CSRC		 = $(ROMFS_IMG:.img=.c)
ROMFS_OBJ		 = $(ROMFS_CSRC:.c=.o)
LIBS			+= $(ROMFS_OBJ)
LINK_DEPS		+= $(ROMFS_OBJ)

$(ROMFS_OBJ): $(ROMFS_CSRC)
	$(Q) $(call COMPILE,$<,$@)

$(ROMFS_CSRC): $(ROMFS_IMG)
	@echo %% generating $@
	$(Q) (cd $(dir $<) && xxd -i $(notdir $<)) > $@

$(ROMFS_IMG): $(ROMFS_DEPS) $(GLOBAL_DEPS)
	@echo %% generating $@
	$(Q) $(GENROMFS) -f $@ -d $(ROMFS_ROOT) -V "NSHInitVol"

endif

################################################################################
# Builtin command list generation
################################################################################

#
# Note that we can't just put builtin_commands.c in SRCS, as it's depended on by the
# NuttX export library. Instead, we have to treat it like a library.
#
# Builtin commands can be generated by the configuration, in which case they
# must refer to commands that already exist, or indirectly generated by applications
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
#
BUILTIN_CSRC		 = $(WORK_DIR)/builtin_commands.c

# add command definitions from apps
BUILTIN_COMMANDS	+= $(subst COMMAND.,,$(notdir $(wildcard $(WORK_DIR)/builtin_commands/COMMAND.*)))

# (BUILTIN_PROTO,<cmdspec>,<outputfile>)
define BUILTIN_PROTO
	echo 'extern int $(word 4,$1)(int argc, char *argv[]);' >> $2;
endef

# (BUILTIN_DEF,<cmdspec>,<outputfile>)
define BUILTIN_DEF
	echo '    {"$(word 1,$1)", $(word 2,$1), $(word 3,$1), $(word 4,$1)},' >> $2;
endef

$(BUILTIN_CSRC):	$(GLOBAL_DEPS)
	@echo %% generating $@
	$(Q) echo '/* builtin command list - automatically generated, do not edit */' > $@
	$(Q) echo '#include <nuttx/config.h>' >> $@
	$(Q) echo '#include <nuttx/binfmt/builtin.h>' >> $@
	$(Q) $(foreach spec,$(BUILTIN_COMMANDS),$(call BUILTIN_PROTO,$(subst ., ,$(spec)),$@))
	$(Q) echo 'const struct builtin_s g_builtins[] = {' >> $@
	$(Q) $(foreach spec,$(BUILTIN_COMMANDS),$(call BUILTIN_DEF,$(subst ., ,$(spec)),$@))
	$(Q) echo '    {NULL, 0, 0, NULL}' >> $@
	$(Q) echo '};' >> $@
	$(Q) echo 'const int g_builtin_count = $(words $(BUILTIN_COMMANDS));' >> $@

BUILTIN_OBJ		 = $(BUILTIN_CSRC:.c=.o)
LIBS			+= $(BUILTIN_OBJ)
LINK_DEPS		+= $(BUILTIN_OBJ)

$(BUILTIN_OBJ): $(BUILTIN_CSRC)
	$(Q) $(call COMPILE,$<,$@)

################################################################################
# Default SRCS generation
################################################################################

#
# If there are no SRCS, the build will fail; in that case, generate an empty
# source file.
#
ifeq ($(SRCS),)
EMPTY_SRC		 = $(WORK_DIR)/empty.c
$(EMPTY_SRC):
	$(Q) echo '/* this is an empty file */' > $@

SRCS			+= $(EMPTY_SRC)
endif

################################################################################
# Build rules
################################################################################

#
# What we're going to build.
#
PRODUCT_BUNDLE		 = $(WORK_DIR)/firmware.px4
PRODUCT_BIN		 = $(WORK_DIR)/firmware.bin
PRODUCT_SYM		 = $(WORK_DIR)/firmware.sym

.PHONY:			firmware
firmware:		$(PRODUCT_BUNDLE)

#
# Object files we will generate from sources
#
OBJS			:= $(foreach src,$(SRCS),$(WORK_DIR)/$(src).o)

#
# SRCS -> OBJS rules
#

$(OBJS):		$(GLOBAL_DEPS)

$(filter %.c.o,$(OBJS)): $(WORK_DIR)/%.c.o: %.c
	$(call COMPILE,$<,$@)

$(filter %.cpp.o,$(OBJS)): $(WORK_DIR)/%.cpp.o: %.cpp $(GLOBAL_DEPS)
	$(call COMPILEXX,$<,$@)

$(filter %.S.o,$(OBJS)): $(WORK_DIR)/%.S.o: %.S $(GLOBAL_DEPS)
	$(call ASSEMBLE,$<,$@)

#
# Built product rules
#

$(PRODUCT_BUNDLE):	$(PRODUCT_BIN)
	@echo %% Generating $@
	$(Q) $(MKFW) --prototype $(IMAGE_DIR)/$(BOARD).prototype \
		--git_identity $(PX4_BASE) \
		--image $< > $@

$(PRODUCT_BIN):		$(PRODUCT_SYM)
	$(call SYM_TO_BIN,$<,$@)

$(PRODUCT_SYM):		$(OBJS) $(APP_OBJS) $(GLOBAL_DEPS) $(LINK_DEPS) $(APP_MKFILES)
	$(call LINK,$@,$(OBJS) $(APP_OBJS))

#
# Utility rules
#

.PHONY: upload
upload:	$(PRODUCT_BUNDLE) $(PRODUCT_BIN)
	$(Q) make -f $(PX4_MK_DIR)/upload.mk \
		METHOD=serial \
		PRODUCT=$(PRODUCT) \
		BUNDLE=$(PRODUCT_BUNDLE) \
		BIN=$(PRODUCT_BIN)

.PHONY: clean
clean:			$(APP_CLEANS)
	@echo %% cleaning
	$(Q) $(REMOVE) $(PRODUCT_BUNDLE) $(PRODUCT_BIN) $(PRODUCT_SYM)
	$(Q) $(REMOVE) $(OBJS) $(DEP_INCLUDES)
	$(Q) $(RMDIR) $(NUTTX_EXPORT_DIR)

#
# DEP_INCLUDES is defined by the toolchain include in terms of $(OBJS)
#
-include $(DEP_INCLUDES)
