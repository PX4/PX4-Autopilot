#
# Framework makefile for PX4 modules
#
# This makefile is invoked by firmware.mk to build each of the modules
# that will subsequently be linked into the firmware image.
#
# Applications are built as prelinked objects with a limited set of exported
# symbols, as the global namespace is shared between all modules. Normally an 
# module will just export one or more <command>_main functions.
#

#
# Variables that can be set by the module's module.mk:
#
#
# SRCS			(required)
#	Lists the .c, cpp and .S files that should be compiled/assembled to
#	produce the module.
#
# MODULE_NAME		(optional)
# MODULE_ENTRYPOINT	(optional if MODULE_NAME is set)
# MODULE_STACKSIZE	(optional if MODULE_NAME is set)
# MODULE_PRIORITY	(optional if MODULE_NAME is set)
#	Defines a single builtin command exported by the module.
#	MODULE_NAME must be unique for any configuration, but need not be the 
#	same as the module directory name.
#	
#	If MODULE_ENTRYPOINT is set, it names the function (which must be exported)
#	that will be the entrypoint for the builtin command. It defaults to
#	$(MODULE_NAME)_main.
#
#	If MODULE_STACKSIZE is set, it is the size in bytes of the stack to be
#	allocated for the builtin command. If it is not set, it defaults
#	to CONFIG_PTHREAD_STACK_DEFAULT.
#
#	If MODULE_PRIORITY is set, it is the thread priority for the builtin
#	command. If it is not set, it defaults to SCHED_PRIORITY_DEFAULT.
#
# MODULE_COMMANDS		(optional)
#	Defines builtin commands exported by the module. Each word in
#	the list should be formatted as: 
#		<command>.<priority>.<stacksize>.<entrypoint>
#

#
# Variables visible to the module's module.mk:
#
# CONFIG
# BOARD
# MODULE_WORK_DIR
# Anything set in setup.mk, board_$(BOARD).mk and the toolchain file.
# Anything exported from config_$(CONFIG).mk
#

################################################################################
# No user-serviceable parts below.
################################################################################

ifeq ($(MODULE_MK),)
$(error No module makefile specified)
endif
$(info MODULE_MK           $(MODULE_MK))

#
# Get path and tool config
#
include $(PX4_BASE)/makefiles/setup.mk

#
# Get the board/toolchain config
#
include $(PX4_MK_DIR)/board_$(BOARD).mk

#
# Get the module's config
#
include $(MODULE_MK)
MODULE_SRC		:= $(dir $(MODULE_MK))
$(info MODULE_SRC          $(MODULE_SRC))
$(info MODULE_WORK_DIR     $(MODULE_WORK_DIR))

#
# Things that, if they change, might affect everything
#
GLOBAL_DEPS		+= $(MAKEFILE_LIST)

################################################################################
# Builtin command definitions
################################################################################

ifneq ($(MODULE_NAME),)
MODULE_ENTRYPOINT	?= $(MODULE_NAME)_main
MODULE_STACKSIZE	?= CONFIG_PTHREAD_STACK_DEFAULT
MODULE_PRIORITY		?= SCHED_PRIORITY_DEFAULT
MODULE_COMMANDS		+= $(MODULE_NAME).$(MODULE_PRIORITY).$(MODULE_STACKSIZE).$(MODULE_ENTRYPOINT)
endif

ifneq ($(MODULE_COMMANDS),)
MODULE_COMMAND_FILES	:= $(addprefix $(WORK_DIR)/builtin_commands/COMMAND.,$(MODULE_COMMANDS))

.PHONY: $(MODULE_COMMAND_FILES)
$(MODULE_COMMAND_FILES): $(GLOBAL_DEPS)
	@$(ECHO) COMMAND    $(word 2,$(subst ., ,$(notdir $(@))))
	@$(MKDIR) -p $(dir $@)
	$(Q) $(TOUCH) $@
endif

################################################################################
# Build rules
################################################################################

#
# What we're going to build
#
module:			$(MODULE_OBJ) $(MODULE_COMMAND_FILES)

#
# Locate sources (allows relative source paths in module.mk)
#
define SRC_SEARCH
	$(abspath $(firstword $(wildcard $(MODULE_SRC)/$1) MISSING_$1))
endef

ABS_SRCS		:= $(foreach src,$(SRCS),$(call SRC_SEARCH,$(src)))
MISSING_SRCS		:= $(subst MISSING_,,$(filter MISSING_%,$(ABS_SRCS)))
ifneq ($(MISSING_SRCS),)
$(error $(MODULE_MK): missing in SRCS: $(MISSING_SRCS))
endif
ifeq ($(ABS_SRCS),)
$(error $(MODULE_MK): nothing to compile in SRCS)
endif

#
# Object files we will generate from sources
#
OBJS			:= $(foreach src,$(ABS_SRCS),$(MODULE_WORK_DIR)$(src).o)

#
# SRCS -> OBJS rules
#

$(OBJS):		$(GLOBAL_DEPS)

$(filter %.c.o,$(OBJS)): $(MODULE_WORK_DIR)%.c.o: %.c $(GLOBAL_DEPS)
	$(call COMPILE,$<,$@)

$(filter %.cpp.o,$(OBJS)): $(MODULE_WORK_DIR)%.cpp.o: %.cpp $(GLOBAL_DEPS)
	$(call COMPILEXX,$<,$@)

$(filter %.S.o,$(OBJS)): $(MODULE_WORK_DIR)%.S.o: %.S $(GLOBAL_DEPS)
	$(call ASSEMBLE,$<,$@)

#
# Built product rules
#

$(MODULE_OBJ):		$(OBJS) $(GLOBAL_DEPS)
	$(call PRELINK,$@,$(OBJS))

#
# Utility rules
#

clean:
	$(Q) $(REMOVE) $(MODULE_PRELINK) $(OBJS)
