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
 * @file spi.cpp
 *
 * Base class for devices connected via SPI.
 *
 * @todo Work out if caching the mode/frequency would save any time.
 *
 */

#include "SPI.hpp"

#include <px4_config.h>
#include <nuttx/arch.h>

#ifndef CONFIG_SPI_EXCHANGE
# error This driver requires CONFIG_SPI_EXCHANGE
#endif

namespace device
{

// static member bitmask
px4::atomic<uint32_t>  SPI::_is_locked{0};
// uint32_t SPI::_is_locked = 0;

SPI::SPI(const char *name,
	 const char *devname,
	 int bus,
	 uint32_t device,
	 enum spi_mode_e mode,
	 uint32_t frequency) :
	CDev(name, devname),
	_locking_mode(LOCK_PREEMPTION),
	_device(device),
	_mode(mode),
	_frequency(frequency),
	_dev(nullptr),
	_isr_deferred(perf_alloc(PC_COUNT, "spi_isr_deferred"))
{
	_device_id.devid_s.bus_type = DeviceBusType_SPI;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = (uint8_t)device;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

SPI::~SPI()
{
	perf_free(_isr_deferred);
}

int
SPI::init()
{
	/* attach to the spi bus */
	if (_dev == nullptr) {
		int bus = get_device_bus();

		if (!board_has_bus(BOARD_SPI_BUS, bus)) {
			return -ENOENT;
		}

		_dev = px4_spibus_initialize(bus);
	}

	if (_dev == nullptr) {
		DEVICE_DEBUG("failed to init SPI");
		return PX4_ERROR;
	}

	/* deselect device to ensure high to low transition of pin select */
	SPI_SELECT(_dev, _device, false);

	/* call the probe function to check whether the device is present */
	if (probe() != PX4_OK) {
		DEVICE_DEBUG("probe failed");
		return PX4_ERROR;
	}

	/* do base class init, which will create the device node, etc. */
	if (CDev::init() != PX4_OK) {
		DEVICE_DEBUG("cdev init failed");
		return PX4_ERROR;
	}

	/* tell the workd where we are */
	DEVICE_LOG("on SPI bus %d at %d (%u KHz)", get_device_bus(), PX4_SPI_DEV_ID(_device), _frequency / 1000);

	return PX4_OK;
}

void SPI::lock(struct spi_dev_s *dev)
{
	// _is_locked |= (1 << _device_id.devid_s.bus);
	_is_locked.fetch_or(1 << _device_id.devid_s.bus);
	SPI_LOCK(dev, true);
}

void SPI::unlock(struct spi_dev_s *dev)
{
	SPI_LOCK(dev, false);
	// _is_locked &= ~(1 << _device_id.devid_s.bus);
	_is_locked.fetch_and(~(1 << _device_id.devid_s.bus));
}

int
SPI::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	// LOCK_NONE if we are in interrupt context because we cannot wait on a semaphore.
	LockMode mode = up_interrupt_context() ? LOCK_NONE : _locking_mode;

	switch (mode) {
	case LOCK_PREEMPTION: {
			irqstate_t state = px4_enter_critical_section();
			_transfer(send, recv, len);
			px4_leave_critical_section(state);
			break;
		}

	case LOCK_THREADS: {
			lock(_dev);
			_transfer(send, recv, len);
			unlock(_dev);
			break;
		}

	case LOCK_NONE: {
			// if (_is_locked & (1 << _device_id.devid_s.bus)) {
			if (_is_locked.load() & (1 << _device_id.devid_s.bus)) {
				// Someone is using the bus
				perf_count(_isr_deferred);
				return PX4_ERROR;
			}

			_transfer(send, recv, len);
			break;
		}

	default:
		break;
	}

	return PX4_OK;
}

void
SPI::_transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	SPI_SETFREQUENCY(_dev, _frequency);
	SPI_SETMODE(_dev, _mode);
	SPI_SETBITS(_dev, 8);
	SPI_SELECT(_dev, _device, true);

	/* do the transfer */
	SPI_EXCHANGE(_dev, send, recv, len);

	/* and clean up */
	SPI_SELECT(_dev, _device, false);
}

int
SPI::transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	LockMode mode = up_interrupt_context() ? LOCK_NONE : _locking_mode;

	switch (mode) {
	case LOCK_PREEMPTION: {
			irqstate_t state = px4_enter_critical_section();
			_transferhword(send, recv, len);
			px4_leave_critical_section(state);
			break;
		}

	case LOCK_THREADS: {
			lock(_dev);
			_transferhword(send, recv, len);
			unlock(_dev);
			break;
		}

	case LOCK_NONE: {
			if (_is_locked.load() & (1 << _device_id.devid_s.bus)) {
				// Someone is using the bus
				perf_count(_isr_deferred);
				return PX4_ERROR;
			}

			_transferhword(send, recv, len);
			break;
		}

	default:
		break;
	}

	return PX4_OK;
}

void
SPI::_transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	SPI_SETFREQUENCY(_dev, _frequency);
	SPI_SETMODE(_dev, _mode);
	SPI_SETBITS(_dev, 16);							/* 16 bit transfer */
	SPI_SELECT(_dev, _device, true);

	/* do the transfer */
	SPI_EXCHANGE(_dev, send, recv, len);

	/* and clean up */
	SPI_SELECT(_dev, _device, false);
}

} // namespace device
