#pragma once

/*
 * I2C busses
 */
#define PX4_I2C_BUS_ESC		1
#define PX4_I2C_BUS_EXPANSION	3
#define PX4_I2C_BUS_LED		3

#define BOARD_OVERRIDE_UUID "SYSTEMID0000    " // must be of length 16
#define PX4_SOC_ARCH_ID     PX4_SOC_ARCH_ID_QURT

#define BOARD_HAS_NO_RESET
#define BOARD_HAS_NO_BOOTLOADER

#include <px4_platform_common/board_common.h>
