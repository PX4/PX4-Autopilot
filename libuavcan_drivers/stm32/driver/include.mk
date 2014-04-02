#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

LIBUAVCAN_STM32_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

LIBUAVCAN_STM32_SRC := $(shell find $(LIBUAVCAN_STM32_DIR)/src/ -type f -name '*.cpp')

LIBUAVCAN_STM32_INC := $(LIBUAVCAN_STM32_DIR)/include/
