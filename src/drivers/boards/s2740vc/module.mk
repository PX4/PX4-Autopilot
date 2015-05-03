#
# Board-specific startup code for the S2740VC
#

SRCS   = \
		   s2740vc_can.c \
		   s2740vc_buttons.c \
		   s2740vc_init.c \
		   s2740vc_led.c \
		   s2740vc_spi.c \
		   ../../../drivers/device/cdev.cpp \
		   ../../../drivers/device/device.cpp \
		   ../../../modules/systemlib/up_cxxinitialize.c

MAXOPTIMIZATION	 = -Os
