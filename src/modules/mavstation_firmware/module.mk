
MODULE_COMMAND		= mavstation

SRCS		= gpio.c \
		  mavstation.c \
		  appdebug.c 

MODULE_STACKSIZE = 512

MAXOPTIMIZATION = -Os
