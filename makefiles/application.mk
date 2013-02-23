#
# Framework makefile for PX4 applications
#
# This makefile is invoked by firmware.mk to build each of the applications
# that will subsequently be linked into the firmware image.
#
# Applications are built as prelinked objects with a limited set of exported
# symbols, as the global namespace is shared between all apps. Normally an 
# application will just export one or more <command>_main functions.
#

#
# Variables that can be set by the application's app.mk:
#
#
# SRCS			(required)
#	Lists the .c, cpp and .S files that should be compiled/assembled to
#	produce the application.
#
# APP_NAME		(optional)
# APP_ENTRYPOINT	(optional if APP_NAME is set)
# APP_STACKSIZE		(optional if APP_NAME is set)
# APP_PRIORITY		(optional if APP_NAME is set)
#	Defines a single builtin command exported by the application.
#	APP_NAME must be unique for any configuration, but need not be the 
#	same as the app directory name.
#	
#	If APP_ENTRYPOINT is set, it names the function (which must be exported)
#	that will be the entrypoint for the builtin command. It defaults to
#	$(APP_NAME)_main.
#
#	If APP_STACKSIZE is set, it is the size in bytes of the stack to be
#	allocated for the builtin command. If it is not set, it defaults
#	to CONFIG_PTHREAD_STACK_DEFAULT.
#
#	If APP_PRIORITY is set, it is the thread priority for the builtin
#	command. If it is not set, it defaults to SCHED_PRIORITY_DEFAULT.
#
# APP_COMMANDS		(optional)
#	Defines builtin commands exported by the application. Each word in
#	the list should be formatted as: 
#		<command>.<priority>.<stacksize>.<entrypoint>
#

#
# Variables visible to the application's app.mk:
#
# CONFIG
# BOARD
# APP_WORK_DIR
# Anything set in setup.mk, board_$(BOARD).mk and the toolchain file.
# Anything exported from config_$(CONFIG).mk
#

################################################################################
# No user-serviceable parts below.
################################################################################

ifeq ($(APP_MK),)
$(error No application makefile specified)
endif

#
# Get path and tool config
#
include $(PX4_BASE)/makefiles/setup.mk

#
# Get the board/toolchain config
#
include $(PX4_MK_DIR)/board_$(BOARD).mk

#
# Get the application's config
#
include $(APP_MK)
APP_SRC_DIR		:= $(dir $(APP_MK))

#
# Things that, if they change, might affect everything
#
GLOBAL_DEPS		+= $(MAKEFILE_LIST)

################################################################################
# Builtin command definitions
################################################################################

ifneq ($(APP_NAME),)
APP_ENTRYPOINT		?= $(APP_NAME)_main
APP_STACKSIZE		?= CONFIG_PTHREAD_STACK_DEFAULT
APP_PRIORITY		?= SCHED_PRIORITY_DEFAULT
APP_COMMANDS		+= $(APP_NAME).$(APP_PRIORITY).$(APP_STACKSIZE).$(APP_ENTRYPOINT)
endif

ifneq ($(APP_COMMANDS),)
APP_COMMAND_FILES	:= $(addprefix $(WORK_DIR)/builtin_commands/COMMAND.,$(APP_COMMANDS))

.PHONY: $(APP_COMMAND_FILES)
$(APP_COMMAND_FILES): $(GLOBAL_DEPS)
	@echo %% registering: $(word 2,$(subst ., ,$(notdir $(@))))
	@mkdir -p $@
	$(Q) touch $@
endif

################################################################################
# Build rules
################################################################################

#
# What we're going to build
#
app:			$(APP_OBJ) $(APP_COMMAND_FILES)

#
# Locate sources (allows relative source paths in app.mk)
#
define SRC_SEARCH
	$(firstword $(wildcard $(APP_SRC_DIR)/$1) MISSING_$1)
endef

ABS_SRCS		:= $(foreach src,$(SRCS),$(call SRC_SEARCH,$(src)))
MISSING_SRCS		:= $(subst MISSING_,,$(filter MISSING_%,$(ABS_SRCS)))
ifneq ($(MISSING_SRCS),)
$(error $(APP_MK): missing in SRCS: $(MISSING_SRCS))
endif
ifeq ($(ABS_SRCS),)
$(error $(APP_MK): nothing to compile in SRCS)
endif

#
# Object files we will generate from sources
#
OBJS			:= $(foreach src,$(ABS_SRCS),$(APP_WORK_DIR)/$(src).o)

#
# SRCS -> OBJS rules
#

$(OBJS):		$(GLOBAL_DEPS)

$(filter %.c.o,$(OBJS)): $(APP_WORK_DIR)/%.c.o: %.c
	$(call COMPILE,$<,$@)

$(filter %.cpp.o,$(OBJS)): $(APP_WORK_DIR)/%.cpp.o: %.cpp $(GLOBAL_DEPS)
	$(call COMPILEXX,$<,$@)

$(filter %.S.o,$(OBJS)): $(APP_WORK_DIR)/%.S.o: %.S $(GLOBAL_DEPS)
	$(call ASSEMBLE,$<,$@)

#
# Built product rules
#

$(APP_OBJ):		$(OBJS) $(GLOBAL_DEPS)
	$(call PRELINK,$@,$(OBJS))

#
# Utility rules
#

clean:
	$(Q) $(REMOVE) $(APP_PRELINK) $(OBJS)
