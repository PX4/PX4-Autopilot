#
# Rules and definitions related to handling the NuttX export archives when
# building firmware.
#

#
# Check that the NuttX archive for the selected board is available.
#
NUTTX_ARCHIVE		:= $(wildcard $(ARCHIVE_DIR)$(BOARD).export)
ifeq ($(NUTTX_ARCHIVE),)
$(error The NuttX export archive for $(BOARD) is missing from $(ARCHIVE_DIR) - try 'make archives' in $(PX4_BASE))
endif

#
# The NuttX config header should always be present in the NuttX archive, and
# if it changes, everything should be rebuilt. So, use it as the trigger to
# unpack the NuttX archive.
#
NUTTX_EXPORT_DIR	 = $(WORK_DIR)nuttx-export/
NUTTX_CONFIG_HEADER	 = $(NUTTX_EXPORT_DIR)include/nuttx/config.h
$(info NUTTX_EXPORT_DIR    $(NUTTX_EXPORT_DIR))
$(info NUTTX_CONFIG_HEADER $(NUTTX_CONFIG_HEADER))


GLOBAL_DEPS		+= $(NUTTX_CONFIG_HEADER)

#
# Use the linker script from the NuttX export
#
LDSCRIPT		 = $(NUTTX_EXPORT_DIR)build/ld.script

#
# Add directories from the NuttX export to the relevant search paths
#
INCLUDE_DIRS		+= $(NUTTX_EXPORT_DIR)include
LIB_DIRS		+= $(NUTTX_EXPORT_DIR)libs
LIBS			+= -lapps -lnuttx
LINK_DEPS		+= $(NUTTX_EXPORT_DIR)libs/libapps.a \
			   $(NUTTX_EXPORT_DIR)libs/libnuttx.a

$(NUTTX_CONFIG_HEADER):	$(NUTTX_ARCHIVE)
	@echo %% Unpacking $(NUTTX_ARCHIVE)
	$(Q) unzip -q -o -d $(WORK_DIR) $(NUTTX_ARCHIVE)
	$(Q) touch $@
