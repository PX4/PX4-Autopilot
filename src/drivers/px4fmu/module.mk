#
# Interface driver for the PX4FMU board
#

MODULE_COMMAND	 = fmu
SRCS		 = fmu.cpp \
			../../modules/systemlib/pwm_limit/pwm_limit.c
