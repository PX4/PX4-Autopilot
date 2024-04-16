/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "string.h"
#include "voxl_esc_serial.hpp"

VoxlEscSerial::VoxlEscSerial()
{
}

VoxlEscSerial::~VoxlEscSerial()
{
	if (_uart.isOpen()) {
		_uart.close();
	}
}

int VoxlEscSerial::uart_open(const char *dev, uint32_t speed)
{
	if (_uart.isOpen()) {
		PX4_ERR("Port in use: %s", dev);
		return -1;
	}

	// Configure UART port
	if (! _uart.setPort(dev)) {
		PX4_ERR("Error configuring serial device on port %s", dev);
		return -1;
	}

	if (! _uart.setBaudrate(speed)) {
		PX4_ERR("Error setting baudrate to %d on %s", (int) speed, dev);
		return -1;
	}

	if (! _uart.setNonBlocking()) {
		PX4_ERR("Error setting non-blocking mode on %s", dev);
		return -1;
	}

	// Open the UART. If this is successful then the UART is ready to use.
	if (! _uart.open()) {
		PX4_ERR("Error opening serial device  %s", dev);
		return -1;
	}

	return 0;
}

int VoxlEscSerial::uart_set_baud(uint32_t speed)
{
	if (! _uart.setBaudrate(speed)) {
		PX4_ERR("Error setting baudrate to %d on %s", (int) speed, _uart.getPort());
		return -1;
	}

	return 0;
}

int VoxlEscSerial::uart_close()
{
	_uart.close();

	return 0;
}

int VoxlEscSerial::uart_write(FAR void *buf, size_t len)
{
	return _uart.write(buf, len);
}

int VoxlEscSerial::uart_read(FAR void *buf, size_t len)
{
	return _uart.read((uint8_t *) buf, len);
}
