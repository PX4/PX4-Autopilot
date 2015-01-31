
MODULE_COMMAND		= mavstation

SRCS		= adc.c \
		  i2c_slave.c \
		  slave_registers.c \
		  gpio.c \
		  mavstation.c \
		  appdebug.c \
		  ../systemlib/perf_counter.c

MODULE_STACKSIZE = 512

MAXOPTIMIZATION = -Os
