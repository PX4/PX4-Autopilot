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
# Rules and definitions related to handling the NuttX export archives when
# building firmware.
#

MODULES += platforms/nuttx/px4_layer platforms/common

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
$(info %  NUTTX_EXPORT_DIR    = $(NUTTX_EXPORT_DIR))
$(info %  NUTTX_CONFIG_HEADER = $(NUTTX_CONFIG_HEADER))


GLOBAL_DEPS		+= $(NUTTX_CONFIG_HEADER)

#
# Use the linker script from the NuttX export
#
LDSCRIPT		+= $(NUTTX_EXPORT_DIR)build/ld.script

#
# Add directories from the NuttX export to the relevant search paths
#
INCLUDE_DIRS		+= $(NUTTX_EXPORT_DIR)include \
			   $(NUTTX_EXPORT_DIR)include/cxx \
			   $(NUTTX_EXPORT_DIR)arch/chip \
			   $(NUTTX_EXPORT_DIR)arch/common

LIB_DIRS		+= $(NUTTX_EXPORT_DIR)libs
LIBS			+= -lapps -lnuttx
NUTTX_LIBS		 = $(NUTTX_EXPORT_DIR)libs/libapps.a \
			   $(NUTTX_EXPORT_DIR)libs/libnuttx.a
LINK_DEPS		+= $(NUTTX_LIBS)

$(NUTTX_CONFIG_HEADER):	$(NUTTX_ARCHIVE)
	@$(ECHO) %% Unpacking $(NUTTX_ARCHIVE)
	$(Q) $(UNZIP_CMD) -q -o -d $(WORK_DIR) $(NUTTX_ARCHIVE)
	$(Q) $(TOUCH) $@

 $(LDSCRIPT): $(NUTTX_CONFIG_HEADER)
 $(NUTTX_LIBS): $(NUTTX_CONFIG_HEADER)
