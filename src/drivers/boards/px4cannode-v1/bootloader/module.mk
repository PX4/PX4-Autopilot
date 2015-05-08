#
# Board-specific Bootloader code for the PX4CANNODE
#

ABS_BOOTLOADER_SRC := $(PX4_BOOTLOADER_BASE)src/

BOOTLOADER_SRC =  boot.c \
					$(ABS_BOOTLOADER_SRC)uavcan/main.c	\
					$(ABS_BOOTLOADER_SRC)common/boot_app_shared.c \
					$(ABS_BOOTLOADER_SRC)sched/timer.c \
					$(ABS_BOOTLOADER_SRC)fs/flash.c	\
					$(ABS_BOOTLOADER_SRC)util/crc.c \
					$(ABS_BOOTLOADER_SRC)arch/stm32/drivers/can/driver.c \
					$(ABS_BOOTLOADER_SRC)protocol/uavcan.c

# Use the relitive path to keep the genrated files in the BUILD_DIR

SRCS = $(subst  $(PX4_MODULE_SRC),../../../../,$(BOOTLOADER_SRC))


MAXOPTIMIZATION	 = -Os
