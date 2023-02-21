/****************************************************************************
 *
 *   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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
 * @file I2C.cpp
 *
 * Base class for devices attached via the I2C bus.
 *
 * @todo Bus frequency changes; currently we do nothing with the value
 *       that is supplied.  Should we just depend on the bus knowing?
 */

#include "I2C.hpp"

#if defined(CONFIG_I2C)

#include <px4_platform_common/time.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>

namespace device
{

I2C::_config_i2c_bus_func_t  I2C::_config_i2c_bus  = NULL;
I2C::_set_i2c_address_func_t I2C::_set_i2c_address = NULL;
I2C::_i2c_transfer_func_t    I2C::_i2c_transfer    = NULL;

pthread_mutex_t I2C::_mutex = PTHREAD_MUTEX_INITIALIZER;

I2C::I2C(uint8_t device_type, const char *name, const int bus, const uint16_t address, const uint32_t frequency) :
	CDev(name, nullptr),
	_frequency(frequency / 1000)
{
	_device_id.devid = 0;

	// fill in _device_id fields for a I2C device
	_device_id.devid_s.devtype = device_type;
	_device_id.devid_s.bus_type = DeviceBusType_I2C;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = address;

	PX4_INFO("*** I2C Device ID 0x%x %d", _device_id.devid, _device_id.devid);
}

I2C::I2C(const I2CSPIDriverConfig &config)
	: I2C(config.devid_driver_index, config.module_name, config.bus, config.i2c_address, config.bus_frequency)
{
}

I2C::~I2C()
{
}

int
I2C::init()
{
	int ret = PX4_ERROR;

	if (_config_i2c_bus == NULL) {
		PX4_ERR("NULL i2c init function");
		goto out;
	}

	pthread_mutex_lock(&_mutex);
	// Open the actual I2C device
	_i2c_fd = _config_i2c_bus(get_device_bus(), get_device_address(), _frequency);
	pthread_mutex_unlock(&_mutex);

	if (_i2c_fd == PX4_ERROR) {
		PX4_ERR("i2c init failed");
		goto out;
	}

	// call the probe function to check whether the device is present
	ret = probe();

	if (ret != OK) {
		PX4_ERR("i2c probe failed");
		goto out;
	}

	// do base class init, which will create device node, etc
	ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("i2c cdev init failed");
		goto out;
	}

	// tell the world where we are
	// PX4_INFO("on I2C bus %d at 0x%02x", get_device_bus(), get_device_address());

out:

	return ret;
}

void
I2C::set_device_address(int address)
{
	if ((_i2c_fd != PX4_ERROR) && (_set_i2c_address != NULL)) {
		PX4_INFO("Set i2c address 0x%x, fd %d", address, _i2c_fd);

		pthread_mutex_lock(&_mutex);
		_set_i2c_address(_i2c_fd, address);
		pthread_mutex_unlock(&_mutex);

		Device::set_device_address(address);
	}
}


int
I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
{
	int ret = PX4_ERROR;
	unsigned retry_count = 1;

	if ((_i2c_fd != PX4_ERROR) && (_i2c_transfer != NULL)) {
		do {
			// PX4_INFO("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);

			pthread_mutex_lock(&_mutex);
			ret = _i2c_transfer(_i2c_fd, send, send_len, recv, recv_len);
			pthread_mutex_unlock(&_mutex);

			if (ret != PX4_ERROR) { break; }

			px4_usleep(1000);

		} while (retry_count++ < _retries);
	}

	return ret;
}

} // namespace device

#endif
