#
# Assorted tests and the like
#

MODULE_COMMAND		 = tests
MODULE_STACKSIZE	 = 60000
MAXOPTIMIZATION		 = -O0

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
			   test_uart_baudchange.c \
			   test_uart_console.c \
			   test_uart_loopback.c \
			   test_uart_send.c \
			   test_mixer.cpp \
			   test_mathlib.cpp \
			   test_file.c \
			   test_file2.c \
			   tests_main.c \
			   test_param.c \
			   test_ppm_loopback.c \
			   test_rc.c \
			   test_conv.cpp \
			   test_mount.c \
			   test_eigen.cpp

ifeq ($(PX4_TARGET_OS), nuttx)
SRCS			+= test_time.c

EXTRACXXFLAGS = -Wframe-larger-than=6000
else
EXTRACXXFLAGS =
endif

EXTRACXXFLAGS += -Wno-float-equal

# Flag is only valid for GCC, not clang
ifneq ($(USE_GCC), 0)
EXTRACXXFLAGS += -Wno-double-promotion -Wno-error=logical-op
endif

