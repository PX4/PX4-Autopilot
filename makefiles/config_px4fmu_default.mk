#
# Makefile for the px4fmu_default configuration
#

CONFIG		 = px4fmu_default
SRCS		 = $(PX4_BASE)/platforms/empty.c
ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/$(CONFIG)

# Commands from the NuttX export archive
#
# Each entry here is <command>.<priority>.<stacksize>.<entrypoint>
BUILTIN_COMMANDS = perf.SCHED_PRIORITY_DEFAULT.CONFIG_PTHREAD_STACK_DEFAULT.perf_main

include $(PX4_MK_DIR)/firmware.mk
