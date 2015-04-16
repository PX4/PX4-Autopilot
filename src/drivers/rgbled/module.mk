#
# TCA62724FMG driver for RGB LED
#

MODULE_COMMAND	 = rgbled

ifdef ($(PX4_TARGET_OS),nuttx)
SRCS		 = rgbled.cpp
else
SRCS		 = rgbled_linux.cpp
endif

MAXOPTIMIZATION	 = -Os
