/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "telemetry.h"

#include <px4_log.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

using namespace time_literals;

#define DSHOT_TELEMETRY_UART_BAUDRATE 115200

DShotTelemetry::~DShotTelemetry()
{
	deinit();
}

int DShotTelemetry::init(const char *uart_device)
{
	deinit();
	_uart_fd = ::open(uart_device, O_RDONLY | O_NOCTTY);

	if (_uart_fd < 0) {
		PX4_ERR("failed to open serial port: %s err: %d", uart_device, errno);
		return -errno;
	}

	_current_motor_index_request = -1;
	return setBaudrate(DSHOT_TELEMETRY_UART_BAUDRATE);
}

void DShotTelemetry::deinit()
{
	if (_uart_fd >= 0) {
		close(_uart_fd);
		_uart_fd = -1;
	}
}

int DShotTelemetry::update()
{
	if (_uart_fd < 0) {
		return -1;
	}

	if (_current_motor_index_request == -1) {
		// nothing in progress, start a request
		_current_motor_index_request = 0;
		_current_request_start = 0;
		_frame_position = 0;
		return -1;
	}

	// read from the uart. This must be non-blocking, so check first if there is data available
	int bytes_available = 0;
	int ret = ioctl(_uart_fd, FIONREAD, (unsigned long)&bytes_available);

	if (ret != 0 || bytes_available <= 0) {
		// no data available. Check for a timeout
		const hrt_abstime now = hrt_absolute_time();

		if (_current_request_start > 0 && now - _current_request_start > 30_ms) {
			PX4_DEBUG("ESC telemetry timeout for motor %i (frame pos=%i)", _current_motor_index_request, _frame_position);
			requestNextMotor();
		}

		return -1;
	}

	const int buf_length = ESC_FRAME_SIZE;
	uint8_t buf[buf_length];

	int num_read = read(_uart_fd, buf, buf_length);
	ret = -1;

	for (int i = 0; i < num_read; ++i) {
		if (decodeByte(buf[i])) {
			ret = _current_motor_index_request;
			requestNextMotor();
		}
	}

	return ret;
}

bool DShotTelemetry::decodeByte(uint8_t byte)
{
	_frame_buffer[_frame_position++] = byte;

	if (_frame_position == ESC_FRAME_SIZE) {
		PX4_DEBUG("got ESC frame for motor %i", _current_motor_index_request);
		uint8_t checksum = crc8(_frame_buffer, ESC_FRAME_SIZE - 1);
		uint8_t checksum_data = _frame_buffer[ESC_FRAME_SIZE - 1];

		if (checksum == checksum_data) {
			_latest_data.time = hrt_absolute_time();
			_latest_data.temperature = _frame_buffer[0];
			_latest_data.voltage = (_frame_buffer[1] << 8) | _frame_buffer[2];
			_latest_data.current = (_frame_buffer[3] << 8) | _frame_buffer[4];
			_latest_data.consumption = (_frame_buffer[5]) << 8 | _frame_buffer[6];
			_latest_data.erpm = (_frame_buffer[7] << 8) | _frame_buffer[8];
			PX4_DEBUG("Motor %i: temp=%i, V=%i, cur=%i, consumpt=%i, rpm=%i", _current_motor_index_request,
				  _latest_data.temperature, _latest_data.voltage, _latest_data.current, _latest_data.consumption,
				  _latest_data.erpm);
		}

		return true;
	}

	return false;
}

uint8_t DShotTelemetry::updateCrc8(uint8_t crc, uint8_t crc_seed)
{
	uint8_t crc_u = crc ^ crc_seed;

	for (int i = 0; i < 8; ++i) {
		crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
	}

	return crc_u;
}


uint8_t DShotTelemetry::crc8(const uint8_t *buf, uint8_t len)
{
	uint8_t crc = 0;

	for (int i = 0; i < len; ++i) {
		crc = updateCrc8(buf[i], crc);
	}

	return crc;
}


void DShotTelemetry::requestNextMotor()
{
	_current_motor_index_request = (_current_motor_index_request + 1) % _num_motors;
	_current_request_start = 0;
	_frame_position = 0;
}

int DShotTelemetry::getRequestMotorIndex()
{
	if (_current_request_start != 0) {
		// already in progress, do not send another request
		return -1;
	}

	_current_request_start = hrt_absolute_time();
	return _current_motor_index_request;
}

int DShotTelemetry::setBaudrate(unsigned baud)
{
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	default:
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_uart_fd, &uart_config);

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

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		return -errno;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		return -errno;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		return -errno;
	}

	return 0;
}

