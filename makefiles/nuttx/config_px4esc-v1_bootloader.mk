#
# Makefile for the px4esc-v1_bootloader configuration
#

#
# Board support modules
#
MODULES		+= drivers/boards/px4esc-v1/bootloader

INCLUDE_DIRS += $(PX4_MODULE_SRC)$(MODULES) $(PX4_BOOTLOADER_BASE)include $(PX4_MODULE_SRC)/modules/systemlib

START_UP_FILES = stm32_vectors.c

