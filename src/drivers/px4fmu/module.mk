#
# Interface driver for the PX4FMU board
#

MODULE_COMMAND	 = fmu
SRCS		 = fmu.cpp \
		  ../../lib/rc/sbus.c \
		  ../../lib/rc/dsm.c \
		  ../../lib/rc/st24.c \
		  ../../lib/rc/sumd.c \
		  ../../lib/rc/srxl.c \
		   px4fmu_params.c

MODULE_STACKSIZE = 1200

EXTRACXXFLAGS	= -Weffc++

MAXOPTIMIZATION	 = -Os
