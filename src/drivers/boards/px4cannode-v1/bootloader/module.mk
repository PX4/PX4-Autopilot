#
# Board-specific Bootloader code for the PX4CANNODE
#

BOOTLOADER_SRC = $(PX4_BOOTLOADER_BASE)src/

SRCS   =  boot.c \
					$(BOOTLOADER_SRC)uavcan/main.c	\
					$(BOOTLOADER_SRC)common/boot_app_shared.c \
					$(BOOTLOADER_SRC)sched/timer.c \
					$(BOOTLOADER_SRC)fs/flash.c	\
					$(BOOTLOADER_SRC)util/crc.c \
					$(BOOTLOADER_SRC)arch/stm32/drivers/can/driver.c \
					$(BOOTLOADER_SRC)protocol/uavcan.c

MAXOPTIMIZATION	 = -Os
