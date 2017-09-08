#pragma once

/*
 * I2C busses
 */
#define PX4_I2C_BUS_ESC		1
#define PX4_I2C_BUS_EXPANSION	3
#define PX4_I2C_BUS_LED		3

#define BOARD_OVERRIDE_UUID "QURT        " // must be of length 12 (PX4_CPU_UUID_BYTE_LENGTH)
#define BOARD_OVERRIDE_PX4_GUID "00000000" BOARD_OVERRIDE_UUID // must be of length 18 (PX4_GUID_BYTE_LENGTH)

#define BOARD_HAS_NO_RESET
#define BOARD_HAS_NO_BOOTLOADER

#include "../common/board_common.h"
