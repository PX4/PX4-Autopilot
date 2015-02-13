/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file i2c.cpp
 *
 * Base class for devices attached via the I2C bus.
 *
 * @todo Bus frequency changes; currently we do nothing with the value
 *       that is supplied.  Should we just depend on the bus knowing?
 */

#include "i2c.h"

namespace device
{

unsigned int I2C::_bus_clocks[3] = { 100000, 100000, 100000 };

I2C::I2C(const char *name,
	 const char *devname,
	 int bus,
	 uint16_t address,
	 uint32_t frequency,
	 int irq) :
	// base class
	CDev(name, devname, irq),
	// public
	// protected
	_retries(0),
	// private
	_bus(bus),
	_address(address),
	_frequency(frequency),
	_dev(nullptr)
{
	// fill in _device_id fields for a I2C device
	_device_id.devid_s.bus_type = DeviceBusType_I2C;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = address;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;     
}

I2C::~I2C()
{
	if (_dev) {
		up_i2cuninitialize(_dev);
		_dev = nullptr;
	}
}

int
I2C::set_bus_clock(unsigned bus, unsigned clock_hz)
{
	int index = bus - 1;

	if (index < 0 || index >= static_cast<int>(sizeof(_bus_clocks) / sizeof(_bus_clocks[0]))) {
		return -EINVAL;
	}

	if (_bus_clocks[index] > 0) {
		// debug("overriding clock of %u with %u Hz\n", _bus_clocks[index], clock_hz);
	}
	_bus_clocks[index] = clock_hz;

	return OK;
}

int
I2C::init()
{
	int ret = OK;
	unsigned bus_index;

	// attach to the i2c bus
	_dev = up_i2cinitialize(_bus);

	if (_dev == nullptr) {
		debug("failed to init I2C");
		ret = -ENOENT;
		goto out;
	}

	// the above call fails for a non-existing bus index,
	// so the index math here is safe.
	bus_index = _bus - 1;

	// abort if the max frequency we allow (the frequency we ask)
	// is smaller than the bus frequency
	if (_bus_clocks[bus_index] > _frequency) {
		(void)up_i2cuninitialize(_dev);
		_dev = nullptr;
		log("FAIL: too slow for bus #%u: %u KHz, device max: %u KHz)",
			_bus, _bus_clocks[bus_index] / 1000, _frequency / 1000);
		ret = -EINVAL;
		goto out;
	}

	// set the bus frequency on the first access if it has
	// not been set yet
	if (_bus_clocks[bus_index] == 0) {
		_bus_clocks[bus_index] = _frequency;
	}

	// set frequency for this instance once to the bus speed
	// the bus speed is the maximum supported by all devices on the bus,
	// as we have to prioritize performance over compatibility.
	// If a new device requires a lower clock speed, this has to be
	// manually set via "fmu i2c <bus> <clock>" before starting any
	// drivers.
	// This is necessary as automatically lowering the bus speed
	// for maximum compatibility could induce timing issues on
	// critical sensors the adopter might be unaware of.
	I2C_SETFREQUENCY(_dev, _bus_clocks[bus_index]);

	// call the probe function to check whether the device is present
	ret = probe();

	if (ret != OK) {
		debug("probe failed");
		goto out;
	}

	// do base class init, which will create device node, etc
	ret = CDev::init();

	if (ret != OK) {
		debug("cdev init failed");
		goto out;
	}

	// tell the world where we are
	log("on I2C bus %d at 0x%02x (bus: %u KHz, max: %u KHz)",
		_bus, _address, _bus_clocks[bus_index] / 1000, _frequency / 1000);

out:
	if ((ret != OK) && (_dev != nullptr)) {
		up_i2cuninitialize(_dev);
		_dev = nullptr;
	}
	return ret;
}

int
I2C::probe()
{
	// Assume the device is too stupid to be discoverable.
	return OK;
}

int
I2C::transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	struct i2c_msg_s msgv[2];
	unsigned msgs;
	int ret;
	unsigned retry_count = 0;

	do {
		//	debug("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);

		msgs = 0;

		if (send_len > 0) {
			msgv[msgs].addr = _address;
			msgv[msgs].flags = 0;
			msgv[msgs].buffer = const_cast<uint8_t *>(send);
			msgv[msgs].length = send_len;
			msgs++;
		}

		if (recv_len > 0) {
			msgv[msgs].addr = _address;
			msgv[msgs].flags = I2C_M_READ;
			msgv[msgs].buffer = recv;
			msgv[msgs].length = recv_len;
			msgs++;
		}

		if (msgs == 0)
			return -EINVAL;

		ret = I2C_TRANSFER(_dev, &msgv[0], msgs);

		/* success */
		if (ret == OK)
			break;

		/* if we have already retried once, or we are going to give up, then reset the bus */
		if ((retry_count >= 1) || (retry_count >= _retries))
			up_i2creset(_dev);

	} while (retry_count++ < _retries);

	return ret;

}

int
I2C::transfer(i2c_msg_s *msgv, unsigned msgs)
{
	int ret;
	unsigned retry_count = 0;

	/* force the device address into the message vector */
	for (unsigned i = 0; i < msgs; i++)
		msgv[i].addr = _address;


	do {
		ret = I2C_TRANSFER(_dev, msgv, msgs);

		/* success */
		if (ret == OK)
			break;

		/* if we have already retried once, or we are going to give up, then reset the bus */
		if ((retry_count >= 1) || (retry_count >= _retries))
			up_i2creset(_dev);

	} while (retry_count++ < _retries);

	return ret;
}

} // namespace device
