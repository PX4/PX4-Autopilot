#
# Interface driver for the PX4FMU board
#

MODULE_COMMAND	 = fmu
SRCS		 = fmu.cpp \
		   px4fmu_params.c

MODULE_STACKSIZE = 1200

EXTRACXXFLAGS	= -Weffc++

MAXOPTIMIZATION	 = -Os
