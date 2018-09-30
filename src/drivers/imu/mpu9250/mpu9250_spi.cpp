/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file mpu9250_spi.cpp
 *
 * Driver for the Invensense MPU9250 connected via SPI.
 *
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author David sidrane
 */


#include <px4_config.h>
#include <lib/drivers/device/spi.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_device.h>

#define DIR_READ			0x80
#define DIR_WRITE			0x00

#define WHOAMI_ADDR			0x75
#define WHOAMI_9250			0x71
#define WHOAMI_6500			0x70

device::Device *MPU9250_SPI_interface(int bus, uint32_t cs);

class MPU9250_SPI : public device::SPI
{
public:
	MPU9250_SPI(int bus, uint32_t device);
	~MPU9250_SPI() = default;

	int	read(unsigned reg, void *data, unsigned count) override;
	int	write(unsigned reg, void *data, unsigned count) override;

protected:

	int probe() override;
};

device::Device *MPU9250_SPI_interface(int bus, uint32_t cs)
{
	device::Device *interface = nullptr;

	if (cs != 0) {
		interface = new MPU9250_SPI(bus, cs);
	}

	return interface;
}

MPU9250_SPI::MPU9250_SPI(int bus, uint32_t device) :
	SPI("MPU9250", nullptr, bus, device, SPIDEV_MODE3, 20e6)
{
	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_MPU9250;
	// _device_id.devid_s.bus_type = (device::Device::DeviceBusType)this->get_device_bus_type();
	// _device_id.devid_s.bus = this->get_device_bus();
	// _device_id.devid_s.address = this->get_device_address();

	set_lockmode(LOCK_THREADS);
}

int MPU9250_SPI::write(unsigned reg, void *data, unsigned count)
{
	uint32_t clock_speed = 1e6;

	set_frequency(clock_speed);

	uint8_t temp = ((uint8_t *)data)[0];

	((uint8_t *)data)[0] = reg | DIR_WRITE;
	((uint8_t *)data)[1] = temp;

	return transfer((uint8_t *)data, (uint8_t *)data, count);
}

int MPU9250_SPI::read(unsigned reg, void *data, unsigned count)
{
	// If we are doing dma read we do this
	uint32_t clock_speed = 20e6;

	set_frequency(clock_speed);

	uint8_t reg_u8 = reg | DIR_READ;

	// We will use the data buffer for both the send and recv values as NuttX DMA requires this.
	((uint8_t *)data)[0] = reg_u8;

	int ret = transfer((uint8_t *)data, (uint8_t *)data, count);

	return ret;
}

int MPU9250_SPI::probe()
{
	uint8_t whoami[2] = {};

	int ret = read(WHOAMI_ADDR, whoami, 2);

	if (ret != OK) {
		return -EIO;
	}

	switch (whoami[1]) {
	case WHOAMI_9250:

	case WHOAMI_6500:
		ret = 0;
		break;

	default:
		PX4_WARN("probe failed! %u", whoami);
		ret = -EIO;
	}

	return ret;
	//return PX4_OK;
}