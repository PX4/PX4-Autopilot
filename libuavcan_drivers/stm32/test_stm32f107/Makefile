#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

PROJECT = uavcan_test_stm32f107

#
# Test application
#

MAIN ?= main.cpp

CPPSRC = src/$(MAIN)              \
         src/dummy.cpp            \
         src/board/board.cpp

#
# UAVCAN library
#

export LIBUAVCAN_REPO_ROOT := $(abspath ../../..)

UDEFS = -DUAVCAN_STM32_CHIBIOS=1         \
        -DUAVCAN_STM32_TIMER_NUMBER=6    \
        -DUAVCAN_TINY=1                  \
        -DUAVCAN_STM32_NUM_IFACES=2      \
        -DUAVCAN_MEM_POOL_BLOCK_SIZE=48

include $(LIBUAVCAN_REPO_ROOT)/libuavcan/include.mk
CPPSRC += $(LIBUAVCAN_SRC)
UINCDIR += $(LIBUAVCAN_INC)

include $(LIBUAVCAN_REPO_ROOT)/libuavcan_drivers/stm32/driver/include.mk
CPPSRC += $(LIBUAVCAN_STM32_SRC)
UINCDIR += $(LIBUAVCAN_STM32_INC)

$(info $(shell $(LIBUAVCAN_DSDLC) $(UAVCAN_DSDL_DIR)))
UINCDIR += dsdlc_generated

#
# Git commit hash
#

GIT_HASH := $(shell git rev-parse --short HEAD)
UDEFS += -DGIT_HASH=0x$(GIT_HASH)

#
# Platform
#

UINCDIR += src/sys

SERIAL_CLI_PORT_NUMBER = 2

CPPWARN := -Wundef -Wno-error=undef

RELEASE_OPT = -Os -fomit-frame-pointer
DEBUG_OPT = -Os -g3
#USE_OPT = -flto

include zubax_chibios/rules_stm32f105_107.mk
