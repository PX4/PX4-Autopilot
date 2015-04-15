#
# Attitude estimator (Nonlinear SO(3) complementary Filter)
#

MODULE_COMMAND	 = attitude_estimator_so3

SRCS		 = attitude_estimator_so3_main.cpp \
		   attitude_estimator_so3_params.c

MODULE_STACKSIZE = 1200

EXTRACXXFLAGS = -Wno-float-equal
