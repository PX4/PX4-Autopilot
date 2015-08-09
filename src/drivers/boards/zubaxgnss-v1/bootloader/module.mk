#
# Board-specific Bootloader code for the ZUBAXGNSS
#

ABS_BOOTLOADER_SRC := $(PX4_BOOTLOADER_BASE)src/

BOOTLOADER_SRC =  boot.c \
					led.c \
					$(ABS_BOOTLOADER_SRC)uavcan/main.c	\
					$(ABS_BOOTLOADER_SRC)common/boot_app_shared.c \
					$(ABS_BOOTLOADER_SRC)sched/timer.c \
					$(ABS_BOOTLOADER_SRC)fs/flash.c	\
					$(ABS_BOOTLOADER_SRC)util/crc.c \
					$(ABS_BOOTLOADER_SRC)util/random.c \
					$(ABS_BOOTLOADER_SRC)arch/stm32/drivers/can/driver.c \
					$(ABS_BOOTLOADER_SRC)protocol/uavcan.c

# Use the relitive path to keep the genrated files in the BUILD_DIR

SRCS = $(subst  $(PX4_MODULE_SRC),../../../../,$(BOOTLOADER_SRC))

override EXTRADEFINES += -DHW_UAVCAN_NAME=$(UAVCANBLID_NAME) -DHW_VERSION_MAJOR=$(UAVCANBLID_HW_VERSION_MAJOR) -DHW_VERSION_MINOR=$(UAVCANBLID_HW_VERSION_MINOR)

MAXOPTIMIZATION	 = -Os
