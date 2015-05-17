#
# Board-specific startup code for the PX4CANNODE
#

ABS_BOOTLOADER_SRC := $(PX4_BOOTLOADER_BASE)src/

SRCS   = \
		   px4cannode_can.c \
		   px4cannode_buttons.c \
		   px4cannode_init.c \
		   px4cannode_led.c \
		   px4cannode_spi.c \
		   ../../../drivers/device/cdev.cpp \
		   ../../../drivers/device/device.cpp \
		   ../../../modules/systemlib/up_cxxinitialize.c 

BOOTLOADER_SRC =  \
			$(ABS_BOOTLOADER_SRC)common/boot_app_shared.c \
 			$(ABS_BOOTLOADER_SRC)util/crc.c
		   
# Use the relitive path to keep the genrated files in the BUILD_DIR

SRCS += $(subst  $(PX4_MODULE_SRC),../../../,$(BOOTLOADER_SRC))
		   

MAXOPTIMIZATION	 = -Os
