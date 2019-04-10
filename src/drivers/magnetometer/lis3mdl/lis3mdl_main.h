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
 * @file lis3mdl_main.h
 */

#pragma once

#include "lis3mdl.h"

namespace lis3mdl
{
/**
 * @struct List of supported bus configurations
 */
struct lis3mdl_bus_option {
	LIS3MDL_BUS bus_id;
	const char *devpath;
	LIS3MDL_constructor interface_constructor;
	uint8_t busnum;
	LIS3MDL *dev;
} bus_options[] = {
#ifdef PX4_I2C_BUS_EXPANSION
	{ LIS3MDL_BUS_I2C_EXTERNAL, "/dev/lis3mdl_ext", &LIS3MDL_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#endif /* PX4_I2C_BUS_EXPANSION */
#ifdef PX4_I2C_BUS_EXPANSION1
	{ LIS3MDL_BUS_I2C_EXTERNAL, "/dev/lis3mdl_ext1", &LIS3MDL_I2C_interface, PX4_I2C_BUS_EXPANSION1, NULL },
#endif /* PX4_I2C_BUS_EXPANSION1 */
#ifdef PX4_I2C_BUS_EXPANSION2
	{ LIS3MDL_BUS_I2C_EXTERNAL, "/dev/lis3mdl_ext2", &LIS3MDL_I2C_interface, PX4_I2C_BUS_EXPANSION2, NULL },
#endif /* PX4_I2C_BUS_EXPANSION2 */
#ifdef PX4_I2C_BUS_ONBOARD
	{ LIS3MDL_BUS_I2C_INTERNAL, "/dev/lis3mdl_int", &LIS3MDL_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif /* PX4_I2C_BUS_ONBOARD */
#ifdef PX4_SPIDEV_LIS
	{ LIS3MDL_BUS_SPI, "/dev/lis3mdl_spi", &LIS3MDL_SPI_interface, PX4_SPI_BUS_SENSORS, NULL },
#endif /* PX4_SPIDEV_LIS */
};

/**
 * @brief Calibrate and self test. Self test feature cannot be used to calculate scale.
 *
 * SELF TEST OPERATION
 * Note: To check the LIS3MDL for proper operation, a self test feature is incorporated :
 *       sensor offset straps are excited to create a nominal field strength
 *       (bias field) to be measured. To implement self test, the least significant bits
 *       (MS1 and MS0) of configuration register A are changed from 00 to 01 (positive bias).
 *       A few measurements are taken and stored with and without the additional magnetic
 *       field. According to ST datasheet, those values must stay between thresholds in order
 *       to pass the self test.
 */
int calibrate(struct lis3mdl_bus_option &bus);

/**
 * @brief Prints info about the driver.
 */
int info(struct lis3mdl_bus_option &bus);

/**
 * @brief Initializes the driver -- sets defaults and starts a cycle
 */
int init(struct lis3mdl_bus_option &bus);

/**
 * @brief Resets the driver.
 */
int reset(struct lis3mdl_bus_option &bus);

/**
 * @brief Starts the driver for a specific bus option
 */
int start_bus(struct lis3mdl_bus_option &bus, Rotation rotation);

/**
 * @brief Starts the driver. This function call only returns once the driver
 *        is either successfully up and running or failed to start.
 */
int start(struct lis3mdl_bus_option &bus, Rotation rotation);

/**
 * @brief Stop the driver.
 */
int stop(struct lis3mdl_bus_option &bus);

/**
 * @brief Perform some basic functional tests on the driver;
 * 	  make sure we can collect data from the sensor in polled
 * 	  and automatic modes.
 */
int test(struct lis3mdl_bus_option &bus);

/**
 * @brief Prints info about the driver argument usage.
 */
void usage();

} // namespace lis3mdl
