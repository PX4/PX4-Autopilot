#
# Board-specific startup code for the PX4CANNODE
#

SRCS   = \
		   px4cannode_can.c \
		   px4cannode_buttons.c \
		   px4cannode_init.c \
		   px4cannode_led.c \
		   px4cannode_spi.c \
		   ../../../drivers/device/cdev.cpp \
		   ../../../drivers/device/device.cpp \
		   ../../../modules/systemlib/up_cxxinitialize.c

MAXOPTIMIZATION	 = -Os
