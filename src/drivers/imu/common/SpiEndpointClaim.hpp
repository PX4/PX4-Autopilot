/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file
 * @brief Startup-only physical SPI endpoint ownership for cooperating family instances.
 */

#pragma once

#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/spi.h>

#include <pthread.h>

namespace imu
{

/**
 * @brief Reserve a board SPI bus/CS independently of model, device type or instance name.
 * @note Acquire before probe, reset GPIO access or IRQ setup. Initializing, running,
 * recovering and faulted instances retain ownership until explicitly released or destroyed.
 * The mutex protects admission/removal only: no bus transfer, wait or sampling operation holds it.
 * This supplements the native lifecycle; it does not replace SPI bus locking or protect against
 * drivers that do not use this claim. One claim object's lifecycle must be externally serialized.
 */
class SpiEndpointClaim
{
public:
	/** Resolve the current board registration without touching hardware. Unknown registrations fail closed. */
	explicit SpiEndpointClaim(const I2CSPIDriverConfig &config) :
		_bus(config.bus),
		_cs_gpio(chipSelect(config))
	{}

	~SpiEndpointClaim() { release(); }

	SpiEndpointClaim(const SpiEndpointClaim &) = delete;
	SpiEndpointClaim &operator=(const SpiEndpointClaim &) = delete;
	SpiEndpointClaim(SpiEndpointClaim &&) = delete;
	SpiEndpointClaim &operator=(SpiEndpointClaim &&) = delete;

	/** @return True when no other participating instance owns this physical endpoint. */
	[[nodiscard]] bool acquire()
	{
		if (_cs_gpio == 0 || pthread_mutex_lock(&_mutex) != 0) {
			return false;
		}

		bool available = true;

		for (const SpiEndpointClaim *owner = _head; owner; owner = owner->_next) {
			if (owner != this && owner->_bus == _bus && owner->_cs_gpio == _cs_gpio) {
				available = false;
				break;
			}
		}

		if (available && !_claimed) {
			_next    = _head;
			_head    = this;
			_claimed = true;
		}

		pthread_mutex_unlock(&_mutex);

		return available;
	}

	/** Release after failed initialization or after acquisition and IRQ cleanup have completed. */
	void release()
	{
		if (!_claimed) {
			return;
		}

		pthread_mutex_lock(&_mutex);

		for (SpiEndpointClaim **owner = &_head; *owner; owner = &(*owner)->_next) {
			if (*owner == this) {
				*owner = _next;
				break;
			}
		}

		_next    = nullptr;
		_claimed = false;
		pthread_mutex_unlock(&_mutex);
	}

private:
	static uint32_t chipSelect(const I2CSPIDriverConfig &config)
	{
		if (config.bus <= 0 || config.bus_device_index < 0 || config.bus_device_index >= SPI_BUS_MAX_DEVICES) {
			return 0;
		}

#if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1

		if (!px4_spi_buses) {
			return 0;
		}

#endif // BOARD_NUM_SPI_CFG_HW_VERSIONS > 1

		for (unsigned i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
			const px4_spi_bus_t &bus = px4_spi_buses[i];

			if (bus.bus == config.bus) {
				const px4_spi_bus_device_t &device = bus.devices[config.bus_device_index];

				// Alias registrations can have different device IDs but the same physical CS.
				return device.devid == config.spi_devid ? device.cs_gpio : 0;
			}
		}

		return 0;
	}

	inline static pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;
	inline static SpiEndpointClaim *_head { nullptr };

	const int      _bus;
	const uint32_t _cs_gpio;
	SpiEndpointClaim *_next { nullptr };
	bool _claimed { false };
};

} // namespace imu
