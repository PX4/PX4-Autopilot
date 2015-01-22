#
# Board-specific startup code for the PX4FMUv2
#

SRCS		 = px4fmu_can.c \
		   px4fmu2_init.c \
		   px4fmu_pwm_servo.c \
		   px4fmu_spi.c \
		   px4fmu_usb.c \
		   px4fmu2_led.c


ENABLE_CXXINITIALIZE=$(call check_nuttx_config ,"CONFIG_HAVE_CXX 1", $(NUTTX_CONFIG_H))
ENABLE_CXXINITIALIZE+=$(call check_nuttx_config ,"CONFIG_HAVE_CXXINITIALIZE 1", $(NUTTX_CONFIG_H))
ifeq ("$(ENABLE_CXXINITIALIZE)",$(nuttx_config_2true))
SRCS		 +=	   ../../../modules/systemlib/up_cxxinitialize.c
endif

MAXOPTIMIZATION	 = -Os
