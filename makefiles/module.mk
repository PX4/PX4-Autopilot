#
#   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
# Framework makefile for PX4 modules
#
# This makefile is invoked by firmware.mk to build each of the modules
# that will subsequently be linked into the firmware image.
#
# Modules are built as prelinked objects with a limited set of exported
# symbols, as the global namespace is shared between all modules. Normally an 
# module will just export one or more <command>_main functions.
#
# IMPORTANT NOTE:
#
# This makefile assumes it is being invoked in the module's output directory.
#

#
# Variables that can be set by the module's module.mk:
#
#
# SRCS			(required)
#
#	Lists the .c, cpp and .S files that should be compiled/assembled to
#	produce the module.
#
# MODULE_COMMAND	(optional)
# MODULE_ENTRYPOINT	(optional if MODULE_COMMAND is set)
# MODULE_STACKSIZE	(optional if MODULE_COMMAND is set)
# MODULE_PRIORITY	(optional if MODULE_COMMAND is set)
#
#	Defines a single builtin command exported by the module.
#	MODULE_COMMAND must be unique for any configuration, but need not be the 
#	same as the module directory name.
#	
#	If MODULE_ENTRYPOINT is set, it names the function (which must be exported)
#	that will be the entrypoint for the builtin command. It defaults to
#	$(MODULE_COMMAND)_main.
#
#	If MODULE_STACKSIZE is set, it is the size in bytes of the stack to be
#	allocated for the builtin command. If it is not set, it defaults
#	to CONFIG_PTHREAD_STACK_DEFAULT.
#
#	If MODULE_PRIORITY is set, it is the thread priority for the builtin
#	command. If it is not set, it defaults to SCHED_PRIORITY_DEFAULT.
#
# MODULE_COMMANDS	(optional if MODULE_COMMAND is not set)
#
#	Defines builtin commands exported by the module. Each word in
#	the list should be formatted as: 
#		<command>.<priority>.<stacksize>.<entrypoint>
#
# INCLUDE_DIRS		(optional, must be appended)
#
#	The list of directories searched for include files. If non-standard
#	includes (e.g. those from another module) are required, paths to search
#	can be added here.
#
# DEFAULT_VISIBILITY	(optional)
#
#	If not set, global symbols defined in a module will not be visible 
#	outside the module. Symbols that should be globally visible must be
#	marked __EXPORT.
#	If set, global symbols defined in a module will be globally visible.
#

#
# Variables visible to the module's module.mk:
#
# CONFIG
# BOARD
# BOARD_FILE
# MODULE_WORK_DIR
# MODULE_OBJ
# MODULE_MK
# Anything set in setup.mk, board_$(BOARD).mk and the toolchain file.
# Anything exported from config_$(CONFIG).mk
#

################################################################################
# No user-serviceable parts below.
################################################################################

ifeq ($(MODULE_MK),)
$(error No module makefile specified)
endif
$(info %% MODULE_MK           = $(MODULE_MK))

#
# Get the board/toolchain config
#
include $(BOARD_FILE)

#
# Get the module's config
#
include $(MODULE_MK)
MODULE_SRC		:= $(dir $(MODULE_MK))
$(info %  MODULE_NAME         = $(MODULE_NAME))
$(info %  MODULE_SRC          = $(MODULE_SRC))
$(info %  MODULE_OBJ          = $(MODULE_OBJ))
$(info %  MODULE_WORK_DIR     = $(MODULE_WORK_DIR))

#
# Things that, if they change, might affect everything
#
GLOBAL_DEPS		+= $(MAKEFILE_LIST)

################################################################################
# Builtin command definitions
################################################################################

ifneq ($(MODULE_COMMAND),)
MODULE_ENTRYPOINT	?= $(MODULE_COMMAND)_main
MODULE_STACKSIZE	?= CONFIG_PTHREAD_STACK_DEFAULT
MODULE_PRIORITY		?= SCHED_PRIORITY_DEFAULT
MODULE_COMMANDS		+= $(MODULE_COMMAND).$(MODULE_PRIORITY).$(MODULE_STACKSIZE).$(MODULE_ENTRYPOINT)
CXXFLAGS		+= -DPX4_MAIN=$(MODULE_COMMAND)_app_main
endif

ifneq ($(MODULE_COMMANDS),)
MODULE_COMMAND_FILES	:= $(addprefix $(WORK_DIR)/builtin_commands/COMMAND.,$(MODULE_COMMANDS))

# Create the command files
# Ensure that there is only one entry for each command
#
.PHONY: $(MODULE_COMMAND_FILES)
$(MODULE_COMMAND_FILES): command = $(word 2,$(subst ., ,$(notdir $(@))))
$(MODULE_COMMAND_FILES): exclude = $(dir $@)COMMAND.$(command).*
$(MODULE_COMMAND_FILES): $(GLOBAL_DEPS)
	@$(REMOVE) -f $(exclude)
	@$(MKDIR) -p $(dir $@)
	@$(ECHO) "CMD:     $(command)"
	$(Q) $(TOUCH) $@
endif

################################################################################
# Adjust compilation flags to implement EXPORT
################################################################################

ifeq ($(DEFAULT_VISIBILITY),)
DEFAULT_VISIBILITY = hidden
else
DEFAULT_VISIBILITY = default
endif

CFLAGS		+= -fvisibility=$(DEFAULT_VISIBILITY) -include $(PX4_INCLUDE_DIR)visibility.h
CXXFLAGS	+= -fvisibility=$(DEFAULT_VISIBILITY) -include $(PX4_INCLUDE_DIR)visibility.h

################################################################################
# Build rules
################################################################################

#
# What we're going to build
#
module:			$(MODULE_OBJ) $(MODULE_COMMAND_FILES)

#
# Object files we will generate from sources
#
OBJS			 = $(addsuffix .o,$(SRCS))

#
# Dependency files that will be auto-generated
#
DEPS			 = $(addsuffix .d,$(SRCS))

#
# SRCS -> OBJS rules
#

$(OBJS):		$(GLOBAL_DEPS)

vpath %.c $(MODULE_SRC)
$(filter %.c.o,$(OBJS)): %.c.o: %.c $(GLOBAL_DEPS)
	$(call COMPILE,$<,$@)

vpath %.cpp $(MODULE_SRC)
$(filter %.cpp.o,$(OBJS)): %.cpp.o: %.cpp $(GLOBAL_DEPS)
	$(call COMPILEXX,$<,$@)

vpath %.S $(MODULE_SRC)
$(filter %.S.o,$(OBJS)): %.S.o: %.S $(GLOBAL_DEPS)
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

-include $(DEPS)
