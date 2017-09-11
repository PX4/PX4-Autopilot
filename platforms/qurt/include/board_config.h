#pragma once

/*
 * I2C busses
 */
#define PX4_I2C_BUS_ESC		1
#define PX4_I2C_BUS_EXPANSION	3
#define PX4_I2C_BUS_LED		3

#define BOARD_OVERRIDE_UUID "SYSMEMID0000   " // must be of length 16
#define PX4_SOC_ARCH_ID 0x1000

#define BOARD_HAS_NO_RESET
#define BOARD_HAS_NO_BOOTLOADER

#include "../common/board_common.h"
