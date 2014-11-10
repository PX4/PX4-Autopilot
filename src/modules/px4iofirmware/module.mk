

SRCS		= adc.c \
		  controls.c \
		  dsm.c \
		  px4io.c \
		  registers.c \
		  safety.c \
		  sbus.c \
		  ../systemlib/up_cxxinitialize.c \
		  ../systemlib/perf_counter.c \
		  mixer.cpp \
		  ../systemlib/mixer/mixer.cpp \
		  ../systemlib/mixer/mixer_group.cpp \
		  ../systemlib/mixer/mixer_multirotor.cpp \
		  ../systemlib/mixer/mixer_simple.cpp \
		  ../systemlib/pwm_limit/pwm_limit.c \
		  ../../lib/rc/st24.c

ifeq ($(BOARD),px4io-v1)
SRCS		+= i2c.c
endif
ifeq ($(BOARD),px4io-v2)
SRCS		+= serial.c \
		   ../systemlib/hx_stream.c
endif
