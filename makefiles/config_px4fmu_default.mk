#
# Makefile for the px4fmu_default configuration
#

#
# Use the configuration's ROMFS.
#
ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/$(CONFIG)

#
# Add commands from the NuttX export archive.
#
# Each entry here is <command>.<priority>.<stacksize>.<entrypoint>
#
BUILTIN_COMMANDS = perf.SCHED_PRIORITY_DEFAULT.CONFIG_PTHREAD_STACK_DEFAULT.perf_main

#
# Build the test module
#
MODULES		 = test
