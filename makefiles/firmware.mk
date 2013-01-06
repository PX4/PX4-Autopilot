#
# Generic Makefile for PX4 firmware.
#
# Currently this assumes that we're just compiling SRCS
# and then linking the whole thing together.
#

#
# Work out where this file is, so we can find other makefiles in the
# same directory.
#
export PX4_MK_INCLUDE	?= $(dir $(lastword $(MAKEFILE_LIST)))

#
# Use the linker script from the NuttX export
#
LDSCRIPT		 = $(WORK_DIR)/nuttx-export/build/ld.script

#
# Add directories from the NuttX export to the relevant search paths
#
INCLUDE_DIRS		+= $(WORK_DIR)/nuttx-export/include
LIB_DIRS		+= $(WORK_DIR)/nuttx-export/libs
LIBS			+= -lapps -lnuttx

#
# Things that, if they change, might affect everything
#
GLOBAL_DEPS		+= $(MAKEFILE_LIST)

#
# Include the platform configuration
#
include $(PX4_MK_INCLUDE)/$(PLATFORM).mk

#
# What we're going to build
#
PRODUCT_BIN		 = $(WORK_DIR)/firmware.bin
PRODUCT_SYM		 = $(WORK_DIR)/firmware.sym
PRODUCTS		 = $(PRODUCT_BIN) $(PRODUCT_SYM)

.PHONY:			all
all:			$(PRODUCTS)

#
# Rules for building objects
#
OBJS			 = $(foreach src,$(SRCS),$(WORK_DIR)/$(src).o)

$(filter %.c.o,$(OBJS)): $(WORK_DIR)/%.c.o: %.c
	@echo compile $<
	@mkdir -p $(dir $@)
	$(call COMPILE,$<,$@)

$(filter %.cpp.o,$(OBJS)): $(WORK_DIR)/%.cpp.o: %.cpp
	@mkdir -p $(dir $@)
	$(call COMPILEXX,$<,$@)

$(filter %.S.o,$(OBJS)): $(WORK_DIR)/%.S.o: %.S
	@mkdir -p $(dir $@)
	$(call ASSEMBLE,$<,$@)

-include $(DEP_INCLUDES)

$(PRODUCT_BIN):		$(PRODUCT_SYM)
	$(call SYM_TO_BIN,$<,$@)

$(PRODUCT_SYM):		$(OBJS) $(GLOBAL_DEPS) $(LINK_DEPS)
	$(call LINK,$@,$(OBJS))

