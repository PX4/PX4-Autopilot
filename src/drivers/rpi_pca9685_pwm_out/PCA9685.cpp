/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Name        : PCA9685.cpp
 * Original Author      : Georgi Todorov
 * Edited by	: Tord Wessman
 * Version     :
 * Created on  : Dec 9, 2012
 *
 * Copyright © 2012 Georgi Todorov  <terahz@geodar.com>
 */

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <syslog.h>		/* Syslog functionallity */
#include <inttypes.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
//

#include "PCA9685.h"

//! Constructor takes bus and address arguments
/*!
 \param bus the bus to use in /dev/i2c-%d.
 \param address the device address on bus
 */

void PCA9685::init(int bus, int address)
{
	_i2cbus = bus;
	_i2caddr = address;
	snprintf(busfile, sizeof(busfile), "/dev/i2c-%d", bus);
	reset();
	//usleep(10*1000);
}

PCA9685::PCA9685() :
	_i2caddr(0x40),
	_i2cbus(1),
	busfile {},
	dataBuffer {}
{

}

PCA9685::~PCA9685()
{
	reset();
}
//! Sets PCA9685 mode to 00
void PCA9685::reset()
{
	int fd = openfd();

	if (fd != -1) {
		write_byte(fd, MODE1, 0x00); //Normal mode
		write_byte(fd, MODE2, 0x04); //Normal mode
		close(fd);
	}
}
//! Set the frequency of PWM
/*!
 \param freq desired frequency. 40Hz to 1000Hz using internal 25MHz oscillator.
 */
void PCA9685::setPWMFreq(int freq)
{
	int fd = openfd();

	if (fd != -1) {
		uint8_t prescale = (CLOCK_FREQ / 4096 / freq)  - 1;
		//printf ("Setting prescale value to: %d\n", prescale);
		//printf ("Using Frequency: %d\n", freq);

		uint8_t oldmode = read_byte(fd, MODE1);
		uint8_t newmode = (oldmode & 0x7F) | 0x10;    //sleep
		write_byte(fd, MODE1, newmode);        // go to sleep
		write_byte(fd, PRE_SCALE, prescale);
		write_byte(fd, MODE1, oldmode);
		usleep(10 * 1000);
		write_byte(fd, MODE1, oldmode | 0x80);

		close(fd);
	}
}

//! PWM a single channel
/*!
 \param led channel to set PWM value for
 \param value 0-4095 value for PWM
 */
void PCA9685::setPWM(uint8_t led, int value)
{
	setPWM(led, 0, value);
}
//! PWM a single channel with custom on time
/*!
 \param led channel to set PWM value for
 \param on_value 0-4095 value to turn on the pulse
 \param off_value 0-4095 value to turn off the pulse
 */
void PCA9685::setPWM(uint8_t led, int on_value, int off_value)
{
	int fd = openfd();

	if (fd != -1) {

		write_byte(fd, LED0_ON_L + LED_MULTIPLYER * led, on_value & 0xFF);

		write_byte(fd, LED0_ON_H + LED_MULTIPLYER * led, on_value >> 8);

		write_byte(fd, LED0_OFF_L + LED_MULTIPLYER * led, off_value & 0xFF);

		write_byte(fd, LED0_OFF_H + LED_MULTIPLYER * led, off_value >> 8);

		close(fd);
	}

}

//! Read a single byte from PCA9685
/*!
 \param fd file descriptor for I/O
 \param address register address to read from
 */
uint8_t PCA9685::read_byte(int fd, uint8_t address)
{

	return 0;

	uint8_t buff[BUFFER_SIZE];
	buff[0] = address;

	if (write(fd, buff, BUFFER_SIZE) != BUFFER_SIZE) {
		printf("I2C slave 0x%x failed to go to register 0x%x [read_byte():write %d]", _i2caddr, address, errno);
		return (-1);

	} else {
		if (read(fd, dataBuffer, BUFFER_SIZE) != BUFFER_SIZE) {
			printf("Could not read from I2C slave 0x%x, register 0x%x [read_byte():read %d]", _i2caddr, address, errno);
			return (-1);
		}
	}


}
//! Write a single byte from PCA9685
/*!
 \param fd file descriptor for I/O
 \param address register address to write to
 \param data 8 bit data to write
 */
void PCA9685::write_byte(int fd, uint8_t address, uint8_t data)
{
	uint8_t buff[2];
	buff[0] = address;
	buff[1] = data;

	if (write(fd, buff, sizeof(buff)) != 2) {
		printf("Failed to write to I2C Slave 0x%x @ register 0x%x [write_byte():write %d]", _i2caddr, address, errno);
		usleep(5000);

	} else {
		//printf("Wrote to I2C Slave 0x%x @ register 0x%x [0x%x]\n", _i2caddr, address, data);
	}
}
//! Open device file for PCA9685 I2C bus
/*!
 \return fd returns the file descriptor number or -1 on error
 */
int PCA9685::openfd()
{
	int fd;

	if ((fd = open(busfile, O_RDWR)) < 0) {
		printf("Couldn't open I2C Bus %d [openfd():open %d]", _i2cbus, errno);
		return -1;
	}

	if (ioctl(fd, I2C_SLAVE, _i2caddr) < 0) {
		printf("I2C slave %d failed [openfd():ioctl %d]", _i2caddr, errno);
		return -1;
	}

	return fd;
}
