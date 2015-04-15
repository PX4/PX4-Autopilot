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
# Path and tool setup
#

#
# Some useful paths.
#
# Note that in general we always keep directory paths with the separator
# at the end, and join paths without explicit separators. This reduces
# the number of duplicate slashes we have lying around in paths,
# and is consistent with joining the results of $(dir) and $(notdir).
#
export PX4_INCLUDE_DIR	 = $(abspath $(PX4_BASE)/src/include)/
export PX4_MODULE_SRC	 = $(abspath $(PX4_BASE)/src)/
export PX4_LIB_DIR	 = $(abspath $(PX4_BASE)/src/lib)/
export PX4_PLATFORMS_DIR = $(abspath $(PX4_BASE)/src/platforms)/
export PX4_MK_DIR	 = $(abspath $(PX4_BASE)/makefiles)/
export NUTTX_SRC	 = $(abspath $(PX4_BASE)/NuttX/nuttx)/
export MAVLINK_SRC	 = $(abspath $(PX4_BASE)/mavlink/include/mavlink/v1.0)/
export NUTTX_APP_SRC	 = $(abspath $(PX4_BASE)/NuttX/apps)/
export MAVLINK_SRC	 = $(abspath $(PX4_BASE)/mavlink)/
export UAVCAN_DIR	 = $(abspath $(PX4_BASE)/uavcan)/
export ROMFS_SRC	 = $(abspath $(PX4_BASE)/ROMFS)/
export IMAGE_DIR	 = $(abspath $(PX4_BASE)/Images)/
export BUILD_DIR	 = $(abspath $(PX4_BASE)/Build)/
export ARCHIVE_DIR	 = $(abspath $(PX4_BASE)/Archives)/

#
# Default include paths
#
export INCLUDE_DIRS	:= $(PX4_MODULE_SRC) \
			   $(PX4_MODULE_SRC)/modules/ \
			   $(PX4_INCLUDE_DIR) \
			   $(PX4_LIB_DIR) \
			   $(PX4_PLATFORMS_DIR)

#
# Tools
#
export MKFW		 = $(PX4_BASE)/Tools/px_mkfw.py
export UPLOADER		 = $(PX4_BASE)/Tools/px_uploader.py
export COPY		 = cp
export COPYDIR		 = cp -Rf
export REMOVE		 = rm -f
export RMDIR		 = rm -rf
export GENROMFS		 = genromfs
export TOUCH		 = touch
export MKDIR		 = mkdir
export FIND		 = find
export ECHO		 = echo
export UNZIP_CMD	 = unzip
export PYTHON		 = python
export OPENOCD		 = openocd
export GREP		 = grep

#
# Host-specific paths, hacks and fixups
#
export SYSTYPE		:= $(shell uname -s)

ifeq ($(SYSTYPE),Darwin)
# Eclipse may not have the toolchain on its path.
export PATH		:= $(PATH):/usr/local/bin
endif

#
# Makefile debugging.
#
export Q		:= $(if $(V),,@)
