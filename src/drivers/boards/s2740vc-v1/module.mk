#
# Board-specific startup code for the S2740VC
#

SRCS   = \
		   s2740vc_can.c \
		   s2740vc_init.c \
		   ../../../drivers/device/cdev.cpp \
		   ../../../drivers/device/device.cpp

MAXOPTIMIZATION	 = -Os
