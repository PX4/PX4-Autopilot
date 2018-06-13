/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file amov_imu_spi.cpp
 *
 * Driver for the AMOV IMU connected via SPI.
 *
 * @author Jin Wu (www.jinwu.science)
 * @author Shuangqi Mo
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/spi.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_device.h>

#include "amov_imu.h"
#include <board_config.h>

#define DIR_READ			0x80
#define DIR_WRITE			0x00


#define AMOV_IMU_LOW_SPI_BUS_SPEED	(2 * 1000 * 1000)
#define AMOV_IMU_HIGH_SPI_BUS_SPEED	(1000 * 1000)


device::Device *AMOV_IMU_SPI_interface(int bus, int device_type, bool external_bus);


class AMOV_IMU_SPI : public device::SPI
{
public:
    AMOV_IMU_SPI(int bus, uint32_t device, int device_type);
	virtual ~AMOV_IMU_SPI();

	virtual int	init();
	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);

	virtual int	ioctl(unsigned operation, unsigned &arg);
protected:
	virtual int probe();

private:

    int _device_type;
	/* Helper to set the desired speed and isolate the register on return */

	void set_bus_frequency(unsigned &reg_speed_reg_out);
};

device::Device *
AMOV_IMU_SPI_interface(int bus, int device_type, bool external_bus)
{
    int cs = SPIDEV_NONE(0);
	device::Device *interface = nullptr;

    cs = PX4_SPIDEV_EXT_AMOV_IMU;

    if (cs != SPIDEV_NONE(0)) {

        interface = new AMOV_IMU_SPI(bus, (uint32_t) cs, device_type);
	}

	return interface;
}

AMOV_IMU_SPI::AMOV_IMU_SPI(int bus, uint32_t device, int device_type) :
	SPI("AMOV_IMU", nullptr, bus, device, SPIDEV_MODE3, AMOV_IMU_LOW_SPI_BUS_SPEED),
    _device_type(device_type)
{
	_device_id.devid_s.devtype =  DRV_ACC_DEVTYPE_AMOV_IMU;
}

AMOV_IMU_SPI::~AMOV_IMU_SPI()
{
}

int
AMOV_IMU_SPI::init()
{
	int ret;

	ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}

	return OK;
}

int
AMOV_IMU_SPI::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {

	case ACCELIOCGEXTERNAL:
        return 0;

	case DEVIOCGDEVICEID:
		return CDev::ioctl(nullptr, operation, arg);

	case AMOV_IMU_IOCGIS_I2C:
		return 0;

	default: {
			ret = -EINVAL;
		}
	}

	return ret;
}

void
AMOV_IMU_SPI::set_bus_frequency(unsigned &reg_speed)
{
	/* Set the desired speed */

	set_frequency(AMOV_IMU_IS_HIGH_SPEED(reg_speed) ? AMOV_IMU_HIGH_SPI_BUS_SPEED : AMOV_IMU_LOW_SPI_BUS_SPEED);

	/* Isoolate the register on return */

	reg_speed = AMOV_IMU_REG(reg_speed);
}


int
AMOV_IMU_SPI::write(unsigned reg_speed, void *data, unsigned count)
{
	uint8_t cmd[AMOV_IMU_MAX_WRITE_BUFFER_SIZE];

	if (sizeof(cmd) < (count + 1)) {
		return -EIO;
	}

	/* Set the desired speed and isolate the register */

	set_bus_frequency(reg_speed);

	cmd[0] = reg_speed | DIR_WRITE;
	cmd[1] = *(uint8_t *)data;

	return transfer(&cmd[0], &cmd[0], count + 1);
}

int
AMOV_IMU_SPI::read(unsigned reg_speed, void *data, unsigned count)
{
    uint8_t cmd[3] = {0, 0, 0};

    uint8_t *pbuff  =  count < sizeof(AMOVReport) ? cmd : (uint8_t *) data ;


    if (count < sizeof(AMOVReport))  {

        /* add command */

        count++;
    }

    set_bus_frequency(reg_speed);

    /* Set command */

    pbuff[0] = reg_speed | DIR_READ ;

    /* Transfer the command and get the data */

    int ret = transfer(pbuff, pbuff, count);

    if (ret == OK && pbuff == &cmd[0]) {

        /* Adjust the count back */

        count--;

        /* Return the data */

        memcpy(data, &cmd[1], count);

    }

    return ret == OK ? count : ret;
}

int
AMOV_IMU_SPI::probe()
{
        return OK;
        /*
    uint8_t whoami = 0;
    uint8_t expected = AMOV_IMU_WHOAMI;

    return (read(AMOV_IMU_REG_WHOAMI, &whoami, 1) > 0 && (whoami == expected)) ? 0 : -EIO;
        */
}

