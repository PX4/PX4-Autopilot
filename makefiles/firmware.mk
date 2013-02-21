#
# Generic Makefile for PX4 firmware.
#
# Currently this assumes that we're just compiling SRCS
# and then linking the whole thing together.
#
# Requires:
#
# BOARD
#	Must be set to a board name known to the PX4 distribution (as
#	we need a corresponding NuttX export archive to link with).
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
# ROMFS_ROOT:
#	If set to the path to a directory, a ROMFS image will be generated
#	containing the files under the directory and linked into the final
#	image.
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
export MK_DIR	?= $(dir $(lastword $(MAKEFILE_LIST)))
ifeq ($(PX4_BASE),)
export PX4_BASE		:= $(abspath $(MK_DIR)/..)
$(info %% set PX4_BASE to $(PX4_BASE))
endif

#
# Get path and tool config
#
include $(MK_DIR)/setup.mk

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
# Sanity-check the BOARD variable and then get the board config.
# If BOARD is not set, but CONFIG is, use that.
#
# The board config in turn will fetch the toolchain configuration.
#
ifeq ($(BOARD),)
ifeq ($(CONFIG),)
$(error At least one of the BOARD or CONFIG variables must be set before including firmware.mk)
endif
BOARD		:= $(firstword $(subst _, ,$(CONFIG)))
endif
include $(PX4_MK_DIR)/board_$(BOARD).mk

#
# Things that, if they change, might affect everything
#
GLOBAL_DEPS		+= $(MAKEFILE_LIST)

################################################################################
# NuttX libraries and paths
################################################################################

#
# Check that the NuttX archive for the selected board is available.
#
NUTTX_ARCHIVE		:= $(wildcard $(ARCHIVE_DIR)/$(BOARD).export)
ifeq ($(NUTTX_ARCHIVE),)
$(error The NuttX export archive for $(BOARD) is missing from $(ARCHIVE_DIR) - try 'make archives' in $(PX4_BASE))
endif

#
# The NuttX config header should always be present in the NuttX archive, and
# if it changes, everything should be rebuilt. So, use it as the trigger to
# unpack the NuttX archive.
#
NUTTX_EXPORT_DIR	 = $(WORK_DIR)/nuttx-export
NUTTX_CONFIG_HEADER	 = $(NUTTX_EXPORT_DIR)/include/nuttx/config.h
GLOBAL_DEPS		+= $(NUTTX_CONFIG_HEADER)

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

$(NUTTX_CONFIG_HEADER):	$(NUTTX_ARCHIVE)
	@echo %% Unpacking $(NUTTX_ARCHIVE)
	$(Q) unzip -q -o -d $(WORK_DIR) $(NUTTX_ARCHIVE)
	$(Q) touch $@

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

$(ROMFS_IMG): $(ROMFS_DEPS)
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
# XXX need to fix stack size numbers here so that apps can set them.
#
BUILTIN_CSRC		 = $(WORK_DIR)/builtin_commands.c

$(BUILTIN_CSRC):	$(MAKEFILE_LIST)
	@echo %% generating $@
	$(Q) echo '/* builtin command list - automatically generated, do not edit */' > $@
	$(Q) echo '#include <nuttx/config.h>' >> $@
	$(Q) echo '#include <nuttx/binfmt/builtin.h>' >> $@
	$(Q) $(foreach app,$(APPS),echo 'extern int $(app)_main(int argc, char *argv[]);' >> $@;)
	$(Q) echo 'const struct builtin_s g_builtins[] = {' >> $@
	$(Q) $(foreach app,$(APPS),echo '    {"$(app)", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, $(app)_main},' >> $@;)
	$(Q) echo '};' >> $@
	$(Q) echo 'const int g_builtin_count = sizeof(g_builtins) / sizeof(g_builtins[0]);' >> $@

BUILTIN_OBJ		 = $(BUILTIN_CSRC:.c=.o)
LIBS			+= $(BUILTIN_OBJ)
LINK_DEPS		+= $(BUILTIN_OBJ)

$(BUILTIN_OBJ): $(BUILTIN_CSRC) $(GLOBAL_DEPS)
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

.PHONY:			all
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
	@mkdir -p $(dir $@)
	$(call COMPILE,$<,$@)

$(filter %.cpp.o,$(OBJS)): $(WORK_DIR)/%.cpp.o: %.cpp $(GLOBAL_DEPS)
	@mkdir -p $(dir $@)
	$(call COMPILEXX,$<,$@)

$(filter %.S.o,$(OBJS)): $(WORK_DIR)/%.S.o: %.S $(GLOBAL_DEPS)
	@mkdir -p $(dir $@)
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

$(PRODUCT_SYM):		$(OBJS) $(GLOBAL_DEPS) $(LINK_DEPS)
	$(call LINK,$@,$(OBJS))

#
# Utility rules
#

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
