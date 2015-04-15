#
# Board-specific startup code for the PX4FMUv2
#

SRCS		 = px4fmu_can.c \
		   px4fmu2_init.c \
		   px4fmu_pwm_servo.c \
		   px4fmu_spi.c \
		   px4fmu_usb.c \
		   px4fmu2_led.c

MAXOPTIMIZATION	 = -Os
