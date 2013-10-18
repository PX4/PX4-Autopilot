#
# Assorted tests and the like
#

MODULE_COMMAND		 = tests
MODULE_STACKSIZE	 = 12000
MAXOPTIMIZATION		 = -Os

SRCS			 = test_adc.c \
			   test_bson.c \
			   test_float.c \
			   test_gpio.c \
			   test_hott_telemetry.c \
			   test_hrt.c \
			   test_int.c \
			   test_jig_voltages.c \
			   test_led.c \
			   test_sensors.c \
			   test_servo.c \
			   test_sleep.c \
			   test_time.c \
			   test_uart_baudchange.c \
			   test_uart_console.c \
			   test_uart_loopback.c \
			   test_uart_send.c \
			   test_mixer.cpp \
			   tests_file.c \
			   tests_main.c \
			   tests_param.c
