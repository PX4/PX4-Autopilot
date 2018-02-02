#
# Board-specific startup code for the VRBRAIN
#

SRCS		 = board_can.c \
		   board_init.c \
		   board_pwm_input.c \
		   board_pwm_servo.c \
		   board_spi.c \
		   board_usb.c \
		   board_led.c \
		   board_buzzer.c

MAXOPTIMIZATION	 = -Os
