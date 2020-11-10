/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
#include <inttypes.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "PCA9685.h"

#include <px4_platform_common/log.h>

using namespace linux_pwm_out;

int PCA9685::init(int bus, int address)
{
	_fd = open_fd(bus, address);
	reset();

	usleep(1000 * 100);
	/* 12BIT 精度输出下，好赢电调可以到200HZ刷新 */
	/* 200HZ for 12bit Resolution, supported by most of the esc's */
	setPWMFreq(200);
	usleep(1000 * 1000);
	return 0;
}

int PCA9685::send_output_pwm(const uint16_t *pwm, int num_outputs)
{
	if (num_outputs > 16) {
		num_outputs = 16;
	}

	for (int i = 0; i < num_outputs; ++i) {
		setPWM(i, pwm[i]);
	}

	return 0;
}

PCA9685::PCA9685()
{
	init(PCA9685_DEFAULT_I2C_BUS, PCA9685_DEFAULT_I2C_ADDR);
}

PCA9685::PCA9685(int bus, int address)
{
	init(bus, address);
}

PCA9685::~PCA9685()
{
	reset();

	if (_fd >= 0) {
		close(_fd);
	}
}

void PCA9685::reset()
{
	if (_fd != -1) {
		write_byte(_fd, MODE1, 0x00); //Normal mode
		write_byte(_fd, MODE2, 0x04); //Normal mode
	}
}

void PCA9685::setPWMFreq(int freq)
{
	if (_fd != -1) {
		uint8_t prescale = (CLOCK_FREQ / MAX_PWM_RES / freq)  - 1;
		//printf ("Setting prescale value to: %d\n", prescale);
		//printf ("Using Frequency: %d\n", freq);

		uint8_t oldmode = read_byte(_fd, MODE1);
		uint8_t newmode = (oldmode & 0x7F) | 0x10;    //sleep
		write_byte(_fd, MODE1, newmode);        // go to sleep
		write_byte(_fd, PRE_SCALE, prescale);
		write_byte(_fd, MODE1, oldmode);
		usleep(10 * 1000);
		write_byte(_fd, MODE1, oldmode | 0x80);
	}
}


void PCA9685::setPWM(uint8_t led, int value)
{
	setPWM(led, 0, value);
}

void PCA9685::setPWM(uint8_t led, int on_value, int off_value)
{
	if (_fd != -1) {
		write_byte(_fd, LED0_ON_L + LED_MULTIPLYER * led, on_value & 0xFF);

		write_byte(_fd, LED0_ON_H + LED_MULTIPLYER * led, on_value >> 8);

		write_byte(_fd, LED0_OFF_L + LED_MULTIPLYER * led, off_value & 0xFF);

		write_byte(_fd, LED0_OFF_H + LED_MULTIPLYER * led, off_value >> 8);
	}

}

uint8_t PCA9685::read_byte(int fd, uint8_t address)
{

	uint8_t buf[1];
	buf[0] = address;

	if (write(fd, buf, 1) != 1) {
		return -1;
	}

	if (read(fd, buf, 1) != 1) {
		return -1;
	}

	return buf[0];
}

void PCA9685::write_byte(int fd, uint8_t address, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = address;
	buf[1] = data;

	if (write(fd, buf, sizeof(buf)) != sizeof(buf)) {
		PX4_ERR("Write failed (%i)", errno);
	}
}

int PCA9685::open_fd(int bus, int address)
{
	int fd;
	char bus_file[64];
	snprintf(bus_file, sizeof(bus_file), "/dev/i2c-%d", bus);

	if ((fd = open(bus_file, O_RDWR)) < 0) {
		PX4_ERR("Couldn't open I2C Bus %d [open_fd():open %d]", bus, errno);
		return -1;
	}

	if (ioctl(fd, I2C_SLAVE, address) < 0) {
		PX4_ERR("I2C slave %d failed [open_fd():ioctl %d]", address, errno);
		return -1;
	}

	return fd;
}
