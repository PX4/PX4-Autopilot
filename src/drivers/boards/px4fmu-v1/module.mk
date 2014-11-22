#
# Board-specific startup code for the PX4FMU
#

SRCS		 = px4fmu_can.c \
		   px4fmu_init.c \
		   px4fmu_pwm_servo.c \
		   px4fmu_spi.c \
		   px4fmu_usb.c \
		   px4fmu_led.c

MAXOPTIMIZATION	 = -Os
