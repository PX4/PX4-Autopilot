#
# Board-specific startup code for the PX4IO
#

SRCS	= px4io_init.c \
	  px4io_pwm_servo.c

ENABLE_CXXINITIALIZE=$(call check_nuttx_config ,"CONFIG_HAVE_CXX 1", $(NUTTX_CONFIG_H))
ENABLE_CXXINITIALIZE+=$(call check_nuttx_config ,"CONFIG_HAVE_CXXINITIALIZE 1", $(NUTTX_CONFIG_H))
ifeq ("$(ENABLE_CXXINITIALIZE)",$(nuttx_config_2true))
SRCS		 +=	   ../../../modules/systemlib/up_cxxinitialize.c
endif

MAXOPTIMIZATION	 = -Os
