/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <px4_arch/i2c_hw_description.h>

constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	initI2CBusExternal(1),  // Use I2C1 (PB8/PB9)
};
