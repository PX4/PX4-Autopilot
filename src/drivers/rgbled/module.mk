#
# TCA62724FMG driver for RGB LED
#

MODULE_COMMAND	 = rgbled

ifeq ($(PX4_TARGET_OS),nuttx)
SRCS		 = rgbled.cpp
else
SRCS		 = rgbled_linux.cpp
endif

MAXOPTIMIZATION	 = -Os
