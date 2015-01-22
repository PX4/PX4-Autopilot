#
# Board-specific startup code for the PX4IOv2
#

SRCS		= px4iov2_init.c \
		  px4iov2_pwm_servo.c

ENABLE_CXXINITIALIZE=$(call check_nuttx_config ,"CONFIG_HAVE_CXX 1", $(NUTTX_CONFIG_H))
ENABLE_CXXINITIALIZE+=$(call check_nuttx_config ,"CONFIG_HAVE_CXXINITIALIZE 1", $(NUTTX_CONFIG_H))
ifeq ("$(ENABLE_CXXINITIALIZE)",$(nuttx_config_2true))
SRCS		 +=	   ../../../modules/systemlib/up_cxxinitialize.c
endif

MAXOPTIMIZATION	 = -Os
