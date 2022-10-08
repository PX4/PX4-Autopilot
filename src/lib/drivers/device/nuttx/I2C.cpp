/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/i2c_spi_buses.h>
#include <nuttx/i2c/i2c_master.h>

namespace device
{

I2C::I2C(uint8_t device_type, const char *name, const int bus, const uint16_t address, const uint32_t frequency) :
	CDev(name, nullptr),
	_frequency(frequency)
{
	// fill in _device_id fields for a I2C device
	_device_id.devid_s.devtype = device_type;
	_device_id.devid_s.bus_type = DeviceBusType_I2C;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = address;
}

I2C::I2C(const I2CSPIDriverConfig &config)
	: I2C(config.devid_driver_index, config.module_name, config.bus, config.i2c_address, config.bus_frequency)
{
}

I2C::~I2C()
{
	if (_dev) {
		px4_i2cbus_uninitialize(_dev);
		_dev = nullptr;
	}
}

int
I2C::init()
{
	// attach to the i2c bus
	_dev = px4_i2cbus_initialize(get_device_bus());

	if (_dev == nullptr) {
		DEVICE_DEBUG("failed to init I2C");
		return -ENOENT;
	}

	const uint32_t max_speed_hz = px4_i2c_bus_max_speed(get_device_bus());

	if (_frequency > max_speed_hz) {
		DEVICE_DEBUG("frequency %" PRIu32 " Hz exceeds bus %d maximum, limited to max %" PRIu32 " Hz", _frequency,
			     get_device_bus(), max_speed_hz);
		_frequency = max_speed_hz;
	}

	// call the probe function to check whether the device is present
	int probe_ret = probe();

	if (probe_ret != OK) {
		DEVICE_DEBUG("probe failed");
		px4_i2cbus_uninitialize(_dev);
		_dev = nullptr;
		return probe_ret;
	}

	// do base class init, which will create device node, etc
	int cdev_init_ret = CDev::init();

	if (cdev_init_ret != OK) {
		DEVICE_DEBUG("cdev init failed");
		px4_i2cbus_uninitialize(_dev);
		_dev = nullptr;
		return cdev_init_ret;
	}

	// tell the world where we are
	DEVICE_DEBUG("on I2C bus %d at 0x%02x (bus: %" PRIu32 " KHz)",
		     get_device_bus(), get_device_address(), _frequency / 1000);

	return PX4_OK;
}

int
I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
{
	int ret = PX4_ERROR;
	unsigned retry_count = 0;

	if (_dev == nullptr) {
		PX4_ERR("I2C device not opened");
		return PX4_ERROR;
	}

	do {
		DEVICE_DEBUG("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);

		i2c_msg_s msgv[2] {};
		unsigned msgs = 0;

		if (send_len > 0) {
			msgv[msgs].frequency = _frequency;
			msgv[msgs].addr = get_device_address();
			msgv[msgs].flags = 0;
			msgv[msgs].buffer = const_cast<uint8_t *>(send);
			msgv[msgs].length = send_len;
			msgs++;
		}

		if (recv_len > 0) {
			msgv[msgs].frequency = _frequency;
			msgv[msgs].addr = get_device_address();
			msgv[msgs].flags = I2C_M_READ;
			msgv[msgs].buffer = recv;
			msgv[msgs].length = recv_len;
			msgs++;
		}

		if (msgs == 0) {
			return -EINVAL;
		}

		int ret_transfer = I2C_TRANSFER(_dev, &msgv[0], msgs);

		if (ret_transfer != 0) {
			DEVICE_DEBUG("I2C transfer failed, result %d", ret_transfer);
			ret = PX4_ERROR;

		} else {
			// success
			ret = PX4_OK;
			break;
		}

		// if we have already retried once, and we aren't going to give up, then reset the bus
		if ((_retries > 0) && (retry_count < _retries)) {
#if defined(CONFIG_I2C_RESET)
			DEVICE_DEBUG("I2C bus: %d, Addr: %X, I2C_RESET %d/%d",
				     get_device_bus(), get_device_address(), retry_count + 1, _retries);
			I2C_RESET(_dev);
#endif // CONFIG_I2C_RESET
		}

	} while (retry_count++ < _retries);

	return ret;
}

} // namespace device

#endif // CONFIG_I2C
