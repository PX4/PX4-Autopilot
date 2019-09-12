/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rm3100_main.h
 */

#pragma once

#include "rm3100.h"

namespace rm3100
{
/**
 * @struct List of supported bus configurations
 */
struct rm3100_bus_option {
	RM3100_BUS bus_id;
	const char *devpath;
	RM3100_constructor interface_constructor;
	uint8_t busnum;
	RM3100 *dev;
} bus_options[] = {
#ifdef PX4_I2C_BUS_EXPANSION
	{ RM3100_BUS_I2C_EXTERNAL, "/dev/rm3100_ext", &RM3100_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#endif /* PX4_I2C_BUS_EXPANSION */
#ifdef PX4_I2C_BUS_EXPANSION1
	{ RM3100_BUS_I2C_EXTERNAL, "/dev/rm3100_ext1", &RM3100_I2C_interface, PX4_I2C_BUS_EXPANSION1, NULL },
#endif /* PX4_I2C_BUS_EXPANSION1 */
#ifdef PX4_I2C_BUS_EXPANSION2
	{ RM3100_BUS_I2C_EXTERNAL, "/dev/rm3100_ext2", &RM3100_I2C_interface, PX4_I2C_BUS_EXPANSION2, NULL },
#endif /* PX4_I2C_BUS_EXPANSION2 */
#ifdef PX4_I2C_BUS_ONBOARD
	{ RM3100_BUS_I2C_INTERNAL, "/dev/rm3100_int", &RM3100_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif /* PX4_I2C_BUS_ONBOARD */
#ifdef PX4_SPIDEV_RM
	{ RM3100_BUS_SPI_INTERNAL, "/dev/rm3100_spi_int", &RM3100_SPI_interface, PX4_SPI_BUS_SENSORS, NULL },
#endif /* PX4_SPIDEV_RM */
#ifdef PX4_SPIDEV_RM_EXT
	{ RM3100_BUS_SPI_EXTERNAL, "/dev/rm3100_spi_ext", &RM3100_SPI_interface, PX4_SPI_BUS_EXT, NULL },
#endif /* PX4_SPIDEV_RM_EXT */
};

/**
 * @brief Finds a bus structure for a bus_id
 */
rm3100_bus_option &find_bus(RM3100_BUS bus_id);

/**
 * @brief Prints info about the driver.
 */
int info(RM3100_BUS bus_id);

/**
 * @brief Initializes the driver -- sets defaults and starts a cycle
 */
bool init(RM3100_BUS bus_id);

/**
 * @brief Resets the driver.
 */
bool reset(RM3100_BUS bus_id);

/**
 * @brief Starts the driver for a specific bus option
 */
bool start_bus(struct rm3100_bus_option &bus, Rotation rotation);

/**
 * @brief Starts the driver. This function call only returns once the driver
 *        is either successfully up and running or failed to start.
 */
int start(RM3100_BUS bus_id, Rotation rotation);

/**
 * @brief Stop the driver.
 */
int stop();

/**
 * @brief Perform some basic functional tests on the driver;
 * 	  make sure we can collect data from the sensor in polled
 * 	  and automatic modes.
 */
bool test(RM3100_BUS bus_id);

/**
 * @brief Prints info about the driver argument usage.
 */
void usage();

} // namespace RM3100
