#
# Board-specific startup code for the S2740VC
#

SRCS   = \
		   s2740vc_can.c \
		   s2740vc_init.c

ifeq ($(PX4_TARGET_OS),nuttx)
SRCS +=		  \
		   ../../../drivers/device/cdev.cpp \
		   ../../../drivers/device/device_nuttx.cpp
else
SRCS +=		  \
		   ../../../drivers/device/device_posix.cpp \
		   ../../../drivers/device/vdev.cpp
endif

MAXOPTIMIZATION	 = -Os
