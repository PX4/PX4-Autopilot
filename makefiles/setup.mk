#
# Path and tool setup
#

#
# Some useful paths.
#
export PX4_APP_SRC	 = $(PX4_BASE)/src/apps
export PX4_LIB_SRC	 = $(PX4_BASE)/src/libs
export NUTTX_SRC	 = $(PX4_BASE)/nuttx
export NUTTX_APP_SRC	 = $(PX4_BASE)/apps
export MAVLINK_SRC	 = $(PX4_BASE)/mavlink
export ROMFS_SRC	 = $(PX4_BASE)/ROMFS
export IMAGE_DIR	 = $(PX4_BASE)/Images
export BUILD_DIR	 = $(PX4_BASE)/Build
export ARCHIVE_DIR	 = $(PX4_BASE)/Archives

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
Q			:= $(if $(V),,@)

