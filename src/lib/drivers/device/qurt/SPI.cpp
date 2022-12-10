/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file SPI.cpp
 *
 * Base class for devices connected via SPI.
 *
 */

#include "SPI.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>

static int (*register_interrupt_callback_func)(int (*)(int, void *, void *), void *arg) = NULL;

int px4_arch_gpiosetevent(spi_drdy_gpio_t pin, bool r, bool f, bool e, int (*func)(int, void *, void *), void *arg)
{

	if ((register_interrupt_callback_func != NULL) && (func != NULL) && (arg != NULL)) {
		PX4_INFO("Register interrupt %p %p %p", register_interrupt_callback_func, func, arg);
		return register_interrupt_callback_func(func, arg);
	}

	return -1;
}

void register_interrupt_callback_initalizer(int (*func)(int (*)(int, void *, void *), void *arg))
{
	register_interrupt_callback_func = func;
}

namespace device
{

SPI::_config_spi_bus_func_t  SPI::_config_spi_bus  = NULL;
SPI::_spi_transfer_func_t    SPI::_spi_transfer    = NULL;

pthread_mutex_t SPI::_mutex = PTHREAD_MUTEX_INITIALIZER;

SPI::SPI(uint8_t device_type, const char *name, int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency) :
	CDev(name, nullptr)
	// CDev(name, nullptr),
	// _device(device),
	// _mode(mode),
	// _frequency(frequency)
{
	_device_id.devid = 0;

	_device_id.devid_s.devtype = device_type;
	// fill in _device_id fields for a SPI device
	_device_id.devid_s.bus_type = DeviceBusType_SPI;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = (uint8_t)device;

	PX4_INFO("*** SPI Device ID 0x%x %d", _device_id.devid, _device_id.devid);
}

SPI::SPI(const I2CSPIDriverConfig &config)
	: SPI(config.devid_driver_index, config.module_name, config.bus, config.spi_devid, config.spi_mode,
	      config.bus_frequency)
{
}

SPI::~SPI()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

int
SPI::init()
{
	int ret = PX4_ERROR;

	if (_config_spi_bus == NULL) {
		PX4_ERR("NULL spi init function");
		return ret;
	}

	pthread_mutex_lock(&_mutex);
	_fd = _config_spi_bus();
	pthread_mutex_unlock(&_mutex);

	if (_fd == PX4_ERROR) {
		PX4_ERR("spi init failed");
		return ret;
	}

	/* call the probe function to check whether the device is present */
	ret = probe();

	if (ret != OK) {
		PX4_INFO("SPI probe failed");
		return ret;
	}

	/* do base class init, which will create the device node, etc. */
	ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("cdev init failed");
		return ret;
	}

	/* tell the world where we are */
	PX4_INFO("on SPI bus %d", get_device_bus());

	return PX4_OK;
}

int
SPI::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	int ret = PX4_ERROR;
	unsigned retry_count = 1;

	if ((_fd != PX4_ERROR) && (_spi_transfer != NULL)) {
		do {
			// PX4_DEBUG("SPI transfer out %p in %p len %u", send, recv, len);

			if (_spi_transfer != NULL) {
				pthread_mutex_lock(&_mutex);
				ret = _spi_transfer(_fd, send, recv, len);
				pthread_mutex_unlock(&_mutex);

			} else {
				PX4_ERR("SPI transfer function is NULL");
			}

			if (ret != PX4_ERROR) { break; }

		} while (retry_count++ < _retries);
	}

	return ret;
}

int
SPI::transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	// Not supported on SLPI
	return PX4_ERROR;
}

} // namespace device
