/****************************************************************************
 *
 *   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
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
 *
 * Original Author      : Georgi Todorov
 * Edited by	: Tord Wessman
 * Created on  : Nov 22, 2013
 * Rewrite	   : Fan.zhang  421395590@qq.com
 * Updated on  : Mar 2, 2017
 ****************************************************************************/

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

void PCA9685::init(int bus, int address) {
	_i2cbus = bus;
	_i2caddr = address;
	snprintf(busfile, sizeof(busfile), "/dev/i2c-%d", bus);
	reset();
	//usleep(10*1000);
}


PCA9685::PCA9685() :
_i2caddr(PCA9685_DEFAULT_I2C_ADDR),
_i2cbus(PCA9685_DEFAULT_I2C_BUS),
dataBuffer {}
{

}

PCA9685::PCA9685(int bus,int address) :
busfile {},
dataBuffer {}
{
	_i2cbus = bus;
	_i2caddr = address;
	snprintf(busfile, sizeof(busfile), "/dev/i2c-%d", bus);
	reset();
}

PCA9685::~PCA9685() {
	reset();
}
//! Sets PCA9685 mode to 00
void PCA9685::reset() {
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
void PCA9685::setPWMFreq(int freq) {
	int fd = openfd();
	if (fd != -1) {
		uint8_t prescale = (CLOCK_FREQ / MAX_PWM_RES / freq)  - 1;
		//printf ("Setting prescale value to: %d\n", prescale);
		//printf ("Using Frequency: %d\n", freq);

 		uint8_t oldmode = read_byte(fd, MODE1);
    	uint8_t newmode = (oldmode & 0x7F) | 0x10;    //sleep
    write_byte(fd, MODE1, newmode);        // go to sleep
    write_byte(fd, PRE_SCALE, prescale);
    write_byte(fd, MODE1, oldmode);
    usleep(10*1000);
    write_byte(fd, MODE1, oldmode | 0x80);

		close(fd);
	}
}

//! PWM a single channel
/*!
 \param led channel to set PWM value for
 \param value 0-4095 value for PWM
 */
void PCA9685::setPWM(uint8_t led, int value) {
	setPWM(led, 0, value);
}
//! PWM a single channel with custom on time
/*!
 \param led channel to set PWM value for
 \param on_value 0-4095 value to turn on the pulse
 \param off_value 0-4095 value to turn off the pulse
 */
void PCA9685::setPWM(uint8_t led, int on_value, int off_value) {
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
uint8_t PCA9685::read_byte(int fd, uint8_t address) {

	return 0;

	uint8_t buff[BUFFER_SIZE];
	buff[0] = address;
	if (write(fd, buff, BUFFER_SIZE) != BUFFER_SIZE) {
		printf("I2C slave 0x%x failed to go to register 0x%x [read_byte():write %d]", _i2caddr, address, errno);
		return (-1);
	} else {
		if (read(fd, dataBuffer, BUFFER_SIZE) != BUFFER_SIZE) {
			printf ("Could not read from I2C slave 0x%x, register 0x%x [read_byte():read %d]", _i2caddr, address, errno);
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
void PCA9685::write_byte(int fd, uint8_t address, uint8_t data) {
	uint8_t buff[2];
	buff[0] = address;
	buff[1] = data;
	if (write(fd, buff, sizeof(buff)) != 2) {
		printf("Failed to write to I2C Slave 0x%x @ register 0x%x [write_byte():write %d]", _i2caddr, address, errno);
		usleep(5000);
	}else{
		//printf("Wrote to I2C Slave 0x%x @ register 0x%x [0x%x]\n", _i2caddr, address, data);
	}
}
//! Open device file for PCA9685 I2C bus
/*!
 \return fd returns the file descriptor number or -1 on error
 */
int PCA9685::openfd() {
	int fd;
	if ((fd = open(busfile, O_RDWR)) < 0) {
		printf ("Couldn't open I2C Bus %d [openfd():open %d]", _i2cbus, errno);
		return -1;
	}
	if (ioctl(fd, I2C_SLAVE, _i2caddr) < 0) {
		printf ("I2C slave %d failed [openfd():ioctl %d]", _i2caddr, errno);
		return -1;
	}

	return fd;
}
