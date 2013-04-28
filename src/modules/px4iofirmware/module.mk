

SRCS		= adc.c \
		  controls.c \
		  dsm.c \
		  px4io.c \
		  registers.c \
		  safety.c \
		  sbus.c \
		  ../systemlib/up_cxxinitialize.c \
		  ../systemlib/hx_stream.c \
		  ../systemlib/perf_counter.c \
		  mixer.cpp \
		  ../systemlib/mixer/mixer.cpp \
		  ../systemlib/mixer/mixer_group.cpp \
		  ../systemlib/mixer/mixer_multirotor.cpp \
		  ../systemlib/mixer/mixer_simple.cpp \

ifneq ($(CONFIG_ARCH_BOARD_PX4IO),)
SRCS		+= i2c.c
EXTRADEFINES	+= -DINTERFACE_I2C
endif
ifneq ($(CONFIG_ARCH_BOARD_PX4IOV2),)
#SRCS		+= serial.c
EXTRADEFINES	+= -DINTERFACE_SERIAL
endif
