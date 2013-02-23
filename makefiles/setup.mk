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
export PX4_MODULE_SRC	 = $(abspath $(PX4_BASE)/src/modules)/
export PX4_MK_DIR	 = $(abspath $(PX4_BASE)/makefiles)/
export NUTTX_SRC	 = $(abspath $(PX4_BASE)/nuttx)/
export NUTTX_APP_SRC	 = $(abspath $(PX4_BASE)/apps)/
export MAVLINK_SRC	 = $(abspath $(PX4_BASE)/mavlink)/
export ROMFS_SRC	 = $(abspath $(PX4_BASE)/ROMFS)/
export IMAGE_DIR	 = $(abspath $(PX4_BASE)/Images)/
export BUILD_DIR	 = $(abspath $(PX4_BASE)/Build)/
export ARCHIVE_DIR	 = $(abspath $(PX4_BASE)/Archives)/

#
# Tools
#
MKFW			 = $(PX4_BASE)/Tools/px_mkfw.py
UPLOADER		 = $(PX4_BASE)/Tools/px_uploader.py
COPY			 = cp
REMOVE			 = rm -f
RMDIR			 = rm -rf
GENROMFS		 = genromfs

#
# Host-specific paths, hacks and fixups
#
SYSTYPE			:= $(shell uname -s)

ifeq ($(SYSTYPE),Darwin)
# Eclipse may not have the toolchain on its path.
export PATH		:= $(PATH):/usr/local/bin
endif

#
# Makefile debugging.
#
export Q		:= $(if $(V),,@)

