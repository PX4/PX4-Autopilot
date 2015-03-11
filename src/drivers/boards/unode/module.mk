#
# Board-specific startup code for the uNode
#

SRCS		 = unode_can.c \
		   unode_init.c \
		   unode_pwm_servo.c \
		   unode_spi.c \
		   unode_usb.c \
		   unode_led.c

MAXOPTIMIZATION	 = -Os
