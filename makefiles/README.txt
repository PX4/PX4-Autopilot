PX4 Build System
================

The files in this directory implement the PX4 runtime firmware build system
and configuration for the standard PX4 boards and software, in conjunction
with Makefile in the parent directory.

../Makefile

	Top-level makefile for the PX4 build system. 
	This makefile supports building NuttX archives for the NuttX based
	configurations, as well as supervising the building of all
	of the defined PX4 firmware configurations.

	Try 'make help' in the parent directory for documentation.

firmware.mk

	Manages the build for one specific firmware configuration. 
	See the comments at the top of this file for detailed documentation.

	Builds modules, builtin command lists and the ROMFS (if configured).

	This is the makefile directly used by external build systems; it can
	be configured to compile modules both inside and outside the PX4
	source tree. When used in this mode, at least BOARD, MODULES and 
	CONFIG_FILE must be set.

firmware_nuttx.mk

	Called by firmware.mk to build NuttX based firmware.

firmware_posix.mk

	Called by firmware.mk to build POSIX (non-ROS) based firmware.

module.mk

	Called by firmware.mk to build individual modules.
	See the comments at the top of this file for detailed documentation.

	Not normally used other than by firmware.mk.

nuttx.mk

	Called by ../Makefile to build or download the NuttX archives if 
	PX4_TARGET_OS is set to "nuttx".

posix.mk

	Called by ../Makefile to set POSIX specific parameters if 
	PX4_TARGET_OS is set to "posix".

upload.mk

	Called by ../Makefile to upload files to a target board. Can be used
	by external build systems as well. (NuttX targets only)

setup.mk

	Provides common path and tool definitions. Implements host 
	system-specific compatibility hacks. Sets PX4_TARGET_OS.

board_<boardname>.mk

	Board-specific configuration for <boardname>. Typically sets 
	CONFIG_ARCH and then includes the toolchain definition for the board.

config_<boardname>_<configname>.mk

	Parameters for a specific configuration on a specific board.
	The board name is derived from the filename.  Sets MODULES to select
	source modules to be included in the configuration, may also set
	ROMFS_ROOT to build a ROMFS and BUILTIN_COMMANDS to include non-module
	commands (e.g. from NuttX)

toolchain_<toolchainname>.mk

	Provides macros used to compile and link source files.
	Accepts EXTRADEFINES to add additional pre-processor symbol definitions,
	EXTRACFLAGS, EXTRACXXFLAGS, EXTRAAFLAGS and EXTRALDFLAGS to pass
	additional flags to the C compiler, C++ compiler, assembler and linker
	respectively.

	Defines the COMPILE, COMPILEXX, ASSEMBLE, PRELINK, ARCHIVE and LINK
	macros that are used elsewhere in the build system.
