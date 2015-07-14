#
# Board-specific startup code for the PX4ESC
#

ABS_BOOTLOADER_SRC := $(PX4_BOOTLOADER_BASE)src/

SRCS   = \
		   px4esc_init.c \
		   px4esc_led.c 

ifeq ($(PX4_TARGET_OS),nuttx)
SRCS +=		  \
		   ../../../drivers/device/cdev.cpp \
		   ../../../drivers/device/device_nuttx.cpp
else
SRCS +=		  \
		   ../../../drivers/device/device_posix.cpp \
		   ../../../drivers/device/vdev.cpp
endif

BOOTLOADER_SRC =  \
			$(ABS_BOOTLOADER_SRC)common/boot_app_shared.c \
 			$(ABS_BOOTLOADER_SRC)util/crc.c
		   
# Use the relitive path to keep the genrated files in the BUILD_DIR

SRCS += $(subst  $(PX4_MODULE_SRC),../../../,$(BOOTLOADER_SRC))
		   

MAXOPTIMIZATION	 = -Os
