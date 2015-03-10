#
#   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
# Framework makefile for PX4 libraries
#
# This makefile is invoked by firmware.mk to build each of the linraries
# that will subsequently be linked into the firmware image.
#
# Applications are built as standard ar archives. Unlike modules,
# all public symbols in library objects are visible across the entire
# firmware stack.
#
# In general, modules should be preferred to libraries when possible.
# Libraries may also be pre-built.
#
# IMPORTANT NOTE:
#
# This makefile assumes it is being invoked in the library's output directory.
#

#
# Variables that can be set by the library's library.mk:
#
#
# SRCS			(optional)
#
#	Lists the .c, cpp and .S files that should be compiled/assembled to
#	produce the library.
#
# PREBUILT_LIB		(optional)
#
#	Names the prebuilt library in the source directory that should be
#	linked into the firmware.
#
# INCLUDE_DIRS		(optional, must be appended, ignored if SRCS not set)
#
#	The list of directories searched for include files. If non-standard
#	includes (e.g. those from another module) are required, paths to search
#	can be added here.
#
#

#
# Variables visible to the library's library.mk:
#
# CONFIG
# BOARD
# LIBRARY_WORK_DIR
# LIBRARY_LIB
# LIBRARY_MK
# Anything set in setup.mk, board_$(BOARD).mk and the toolchain file.
# Anything exported from config_$(CONFIG).mk
#

################################################################################
# No user-serviceable parts below.
################################################################################

ifeq ($(LIBRARY_MK),)
$(error No library makefile specified)
endif
$(info %% LIBRARY_MK          = $(LIBRARY_MK))

#
# Get the board/toolchain config
#
include $(PX4_MK_DIR)/$(PX4_TARGET_OS)/board_$(BOARD).mk

#
# Get the library's config
#
include $(LIBRARY_MK)
LIBRARY_SRC		:= $(dir $(LIBRARY_MK))
$(info %  LIBRARY_NAME        = $(LIBRARY_NAME))
$(info %  LIBRARY_SRC         = $(LIBRARY_SRC))
$(info %  LIBRARY_LIB         = $(LIBRARY_LIB))
$(info %  LIBRARY_WORK_DIR    = $(LIBRARY_WORK_DIR))

#
# Things that, if they change, might affect everything
#
GLOBAL_DEPS		+= $(MAKEFILE_LIST)

################################################################################
# Build rules
################################################################################

#
# What we're going to build
#
library:			$(LIBRARY_LIB)

ifneq ($(PREBUILT_LIB),)

VPATH			 = $(LIBRARY_SRC)
$(LIBRARY_LIB):		$(PREBUILT_LIB) $(GLOBAL_DEPS)
	@$(ECHO) "PREBUILT: $(PREBUILT_LIB)"
	$(Q) $(COPY) $< $@

else

##
## Object files we will generate from sources
##

OBJS			 = $(addsuffix .o,$(SRCS))

#
# SRCS -> OBJS rules
#

$(OBJS):		$(GLOBAL_DEPS)

vpath %.c $(LIBRARY_SRC)
$(filter %.c.o,$(OBJS)): %.c.o: %.c $(GLOBAL_DEPS)
	$(call COMPILE,$<,$@)

vpath %.cpp $(LIBRARY_SRC)
$(filter %.cpp.o,$(OBJS)): %.cpp.o: %.cpp $(GLOBAL_DEPS)
	$(call COMPILEXX,$<,$@)

vpath %.S $(LIBRARY_SRC)
$(filter %.S.o,$(OBJS)): %.S.o: %.S $(GLOBAL_DEPS)
	$(call ASSEMBLE,$<,$@)

#
# Built product rules
#

$(LIBRARY_LIB):		$(OBJS) $(GLOBAL_DEPS)
	$(call ARCHIVE,$@,$(OBJS))

endif

#
# Utility rules
#

clean:
	$(Q) $(REMOVE) $(LIBRARY_LIB) $(OBJS)
