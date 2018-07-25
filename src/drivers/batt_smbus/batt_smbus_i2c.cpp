/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file batt_smbus_i2c.cpp
 *
 * I2C interface for batt_smbus
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 */

#include "batt_smbus.h"

#include <cstring>

#include <px4_config.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_device.h>

#include "board_config.h"

device::Device *BATT_SMBUS_I2C_interface(int bus);

class BATT_SMBUS_I2C : public device::I2C
{
public:
	BATT_SMBUS_I2C(int bus);
	virtual ~BATT_SMBUS_I2C() = default;

	/**
	 * @brief Sends a block read command.
	 * @param cmd_code The command code.
	 * @param data Pointer to the data being returned.
	 * @param count The number of bytes being read
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	virtual int read(unsigned cmd_code, void *data, unsigned count);

	/**
	 * @brief Sends a block write command.
	 * @param cmd_code The command code.
	 * @param data Pointer to the data to be written.
	 * @param count The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	virtual int write(unsigned cmd_code, void *data, unsigned count);

	/**
	 * @brief Sends a block read command.
	 * @param cmd_code The command code.
	 * @param data Pointer to the data being returned.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int read(unsigned cmd_code, void *data);


	/**
	 * @brief Sends a block write command.
	 * @param cmd_code The command code.
	 * @param data Pointer to the data to be written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int write(unsigned cmd_code, void *data);

protected:

	/**
	 * @brief Calculates the PEC from the data.
	 * @param buffer The buffer that stores the data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	uint8_t get_pec(uint8_t *buffer, uint8_t length);

};

device::Device *
BATT_SMBUS_I2C_interface(int bus)
{
	return new BATT_SMBUS_I2C(bus);
}

BATT_SMBUS_I2C::BATT_SMBUS_I2C(int bus) :
	I2C("BATT_SMBUS_I2C", nullptr, bus, BATT_SMBUS_ADDR, 100000)
{
}


int
BATT_SMBUS_I2C::read(unsigned cmd_code, void *data, unsigned length)
{
	uint8_t buf = (uint8_t) cmd_code;
	return transfer(&buf, 1, (uint8_t *)data, length);
}

int
BATT_SMBUS_I2C::write(unsigned cmd_code, void *data, unsigned length)
{
	return transfer((uint8_t *)data, length, nullptr, 0);
}