/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file VescSerialDevice.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 */

#include "VescSerialDevice.hpp"
#include <stdint.h>
#include <termios.h>
#include <lib/drivers/device/Device.hpp>

using namespace time_literals;

VescDevice::VescDevice(const char *port) :
	CDev("/dev/vesc"),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_vesc_driver(*this)
{
	// Store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// Enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	_mixing_output.setAllDisarmedValues(DISARM_VALUE);
	_mixing_output.setAllMinValues(MIN_THROTTLE);
	_mixing_output.setAllMaxValues(MAX_THROTTLE);
}

VescDevice::~VescDevice()
{
	perf_free(_cycle_perf);
}

int VescDevice::init()
{
	// Do regular cdev init
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	// Set Baudrate and schedule for the first time
	ret = setBaudrate(115200);

	if (ret != OK) {
		return ret;
	}

	ScheduleNow();
	return OK;
}

void VescDevice::printStatus()
{
	printf("Using port '%s'\n", _port);
	printf("VESC version: %d.%d\n", _vesc_driver.getVersionMajor(), _vesc_driver.getVersionMinor());
	_mixing_output.printStatus();
	perf_print_counter(_cycle_perf);
}

bool VescDevice::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			       unsigned num_control_groups_updated)
{
	// for (unsigned i = 0; i < num_outputs; i++) {
	// 	if (!stop_motors && outputs[i] != DISARM_VALUE) {
	// 		_vesc_driver.commandDutyCycle(static_cast<float>(outputs[i]) / 1e3f);
	// 	}
	// }

	if (!stop_motors) {
		_vesc_driver.commandDutyCycle(static_cast<float>(outputs[0]) / 1e3f);
		_vesc_driver.commandDutyCycle(static_cast<float>(outputs[1]) / 1e3f, 11);
		_vesc_driver.commandDutyCycle(static_cast<float>(outputs[2]) / 1e3f, 7);
		_vesc_driver.commandDutyCycle(static_cast<float>(outputs[3]) / 1e3f, 107);

	} else {
		_vesc_driver.commandDutyCycle(0.f);
		_vesc_driver.commandDutyCycle(0.f, 11);
		_vesc_driver.commandDutyCycle(0.f, 7);
		_vesc_driver.commandDutyCycle(0.f, 107);
	}

	return true;
}

void VescDevice::Run()
{
	perf_begin(_cycle_perf);

	if (_serial_fd < 0) {
		// Reopen fd from the work queue context
		_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);
		_vesc_driver.requestFirmwareVersion();
	}

	_mixing_output.update();

	// Check the number of bytes available in the buffer
	int bytes_available{0};
	int ret = ::ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);

	while (ret >= 0 && bytes_available > 0) {
		printf("read: ");
		uint8_t readbuf[READ_BUFFER_SIZE] {};

		// Read from the uart buffer
		ret = ::read(_serial_fd, readbuf, READ_BUFFER_SIZE);

		for (int i = 0; i < ret; i++) {
			printf("%02X ", readbuf[i]);
		}

		printf("\n");

		for (int i = 0; i < ret; i++) {
			_vesc_driver.parseInputByte(readbuf[i]);
		}

		bytes_available -= ret;
	}

	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

int VescDevice::setBaudrate(const unsigned baudrate)
{
	// Open fd in main task context
	_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);

	// Translate baud rate to define
	int speed;

	switch (baudrate) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

#ifndef B460800
#define B460800 460800
#endif

	case 460800: speed = B460800; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baudrate);
		return -EINVAL;
	}

	// Fill the struct for the new configuration
	struct termios uart_config;
	tcgetattr(_serial_fd, &uart_config);

	// Configure the terminal (see https://en.wikibooks.org/wiki/Serial_Programming/termios)

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// No parity, one stop bit, disable flow control
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	// Apply configuration and baud rate
	int termios_state;

	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("ERR: %d (cfsetispeed)", termios_state);
		return PX4_ERROR;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("ERR: %d (cfsetospeed)", termios_state);
		return PX4_ERROR;
	}

	if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("ERR: %d (tcsetattr)", termios_state);
		return PX4_ERROR;
	}

	// Close fd in main task context
	::close(_serial_fd);
	_serial_fd = -1;

	return PX4_OK;
}

size_t VescDevice::writeCallback(const uint8_t *buffer, const uint16_t length)
{
	printf("write: ");

	for (int i = 0; i < length; i++) {
		printf("%02X ", buffer[i]);
	}

	printf("\n");
	int ret = ::write(_serial_fd, buffer, length);
	printf("write result: %d %d\n", ret, _serial_fd);
	return ret;
}

int VescDevice::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("ioctl cmd: %d, arg: %ld", cmd, arg);

	lock();

	switch (cmd) {
	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);
			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}
