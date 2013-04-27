
MODULE_NAME	 = attitude_estimator_ekf
SRCS		 = attitude_estimator_ekf_main.cpp \
			   attitude_estimator_ekf_params.c \
			   codegen/attitudeKalmanfilter_initialize.c \
			   codegen/attitudeKalmanfilter_terminate.c \
			   codegen/attitudeKalmanfilter.c \
			   codegen/cross.c \
			   codegen/eye.c \
			   codegen/mrdivide.c \
			   codegen/norm.c \
			   codegen/rdivide.c \
			   codegen/rt_nonfinite.c \
			   codegen/rtGetInf.c \
			   codegen/rtGetNaN.c

