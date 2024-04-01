/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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

#include <SerialImpl.hpp>
#include <string.h> // strncpy
#include <px4_log.h>
#include <drivers/device/qurt/uart.h>
#include <drivers/drv_hrt.h>

namespace device
{

SerialImpl::SerialImpl(const char *port, uint32_t baudrate, ByteSize bytesize, Parity parity, StopBits stopbits,
		       FlowControl flowcontrol) :
	_baudrate(baudrate),
	_bytesize(bytesize),
	_parity(parity),
	_stopbits(stopbits),
	_flowcontrol(flowcontrol)
{
	if (validatePort(port)) {
		setPort(port);

	} else {
		_port[0] = 0;
	}
}

SerialImpl::~SerialImpl()
{
	if (isOpen()) {
		close();
	}
}

bool SerialImpl::validateBaudrate(uint32_t baudrate)
{
	if ((baudrate != 9600)   &&
	    (baudrate != 38400)  &&
	    (baudrate != 57600)  &&
	    (baudrate != 115200) &&
	    (baudrate != 230400) &&
	    (baudrate != 250000) &&
	    (baudrate != 420000) &&
	    (baudrate != 460800) &&
	    (baudrate != 921600) &&
	    (baudrate != 1000000) &&
	    (baudrate != 1843200) &&
	    (baudrate != 2000000)) {
		return false;
	}

	return true;
}

bool SerialImpl::open()
{
	// There's no harm in calling open multiple times on the same port.
	// In fact, that's the only way to change the baudrate

	_open = false;
	_serial_fd = -1;

	if (! validateBaudrate(_baudrate)) {
		PX4_ERR("Invalid baudrate: %u", _baudrate);
		return false;
	}

	if (_bytesize != ByteSize::EightBits) {
		PX4_ERR("Qurt platform only supports ByteSize::EightBits");
		return false;
	}

	if (_parity != Parity::None) {
		PX4_ERR("Qurt platform only supports Parity::None");
		return false;
	}

	if (_stopbits != StopBits::One) {
		PX4_ERR("Qurt platform only supports StopBits::One");
		return false;
	}

	if (_flowcontrol != FlowControl::Disabled) {
		PX4_ERR("Qurt platform only supports FlowControl::Disabled");
		return false;
	}

	if (!validatePort(_port)) {
		PX4_ERR("Invalid port %s", _port);
		return false;
	}

	// qurt_uart_open will check validity of port and baudrate
	int serial_fd = qurt_uart_open(_port, _baudrate);

	if (serial_fd < 0) {
		PX4_ERR("failed to open %s, fd returned: %d", _port, serial_fd);
		return false;

	} else {
		PX4_INFO("Successfully opened UART %s with baudrate %u", _port, _baudrate);
	}

	_serial_fd = serial_fd;
	_open = true;

	return _open;
}

bool SerialImpl::isOpen() const
{
	return _open;
}

bool SerialImpl::close()
{
	// No close defined for qurt uart yet
	// if (_serial_fd >= 0) {
	// 	qurt_uart_close(_serial_fd);
	// }

	_serial_fd = -1;
	_open = false;

	return true;
}

ssize_t SerialImpl::read(uint8_t *buffer, size_t buffer_size)
{
	if (!_open) {
		PX4_ERR("Cannot read from serial device until it has been opened");
		return -1;
	}

	int ret_read = qurt_uart_read(_serial_fd, (char *) buffer, buffer_size, 500);

	if (ret_read < 0) {
		PX4_DEBUG("%s read error %d", _port, ret_read);

	}

	return ret_read;
}

ssize_t SerialImpl::readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count, uint32_t timeout_us)
{
	if (!_open) {
		PX4_ERR("Cannot readAtLeast from serial device until it has been opened");
		return -1;
	}

	if (buffer_size < character_count) {
		PX4_ERR("%s: Buffer not big enough to hold desired amount of read data", __FUNCTION__);
		return -1;
	}

	const hrt_abstime start_time_us = hrt_absolute_time();
	int total_bytes_read = 0;

	while (total_bytes_read < (int) character_count) {

		if (timeout_us > 0) {
			const uint64_t elapsed_us = hrt_elapsed_time(&start_time_us);

			if (elapsed_us >= timeout_us) {
				// If there was a partial read but not enough to satisfy the minimum then they will be lost
				// but this really should never happen when everything is working normally.
				// PX4_WARN("%s timeout %d bytes read (%llu us elapsed)", __FUNCTION__, total_bytes_read, elapsed_us);
				// Or, instead of returning an error, should we return the number of bytes read (assuming it is greater than zero)?
				return total_bytes_read;
			}
		}

		int current_bytes_read = read(&buffer[total_bytes_read], buffer_size - total_bytes_read);

		if (current_bytes_read < 0) {
			// Again, if there was a partial read but not enough to satisfy the minimum then they will be lost
			// but this really should never happen when everything is working normally.
			PX4_ERR("%s failed to read uart", __FUNCTION__);
			// Or, instead of returning an error, should we return the number of bytes read (assuming it is greater than zero)?
			return -1;
		}

		// Current bytes read could be zero
		total_bytes_read += current_bytes_read;

		// If we have at least reached our desired minimum number of characters
		// then we can return now
		if (total_bytes_read >= (int) character_count) {
			return total_bytes_read;
		}

		// Wait a set amount of time before trying again or the remaining time
		// until the timeout if we are getting close
		const uint64_t elapsed_us = hrt_elapsed_time(&start_time_us);
		int64_t time_until_timeout = timeout_us - elapsed_us;
		uint64_t time_to_sleep = 5000;

		if ((time_until_timeout >= 0) &&
		    (time_until_timeout < (int64_t) time_to_sleep)) {
			time_to_sleep = time_until_timeout;
		}

		px4_usleep(time_to_sleep);
	}

	return -1;
}

ssize_t SerialImpl::write(const void *buffer, size_t buffer_size)
{
	if (!_open) {
		PX4_ERR("Cannot write to serial device until it has been opened");
		return -1;
	}

	int ret_write = qurt_uart_write(_serial_fd, (const char *) buffer, buffer_size);

	if (ret_write < 0) {
		PX4_ERR("%s write error %d", _port, ret_write);

	}

	return ret_write;
}

void SerialImpl::flush()
{
	// TODO: Flush not implemented yet on Qurt
}

const char *SerialImpl::getPort() const
{
	return _port;
}

bool SerialImpl::validatePort(const char *port)
{
	return (qurt_uart_get_port(port) >= 0);
}

bool SerialImpl::setPort(const char *port)
{
	if (_open) {
		PX4_ERR("Cannot set port after port has already been opened");
		return false;
	}

	if (validatePort(port)) {
		strncpy(_port, port, sizeof(_port) - 1);
		_port[sizeof(_port) - 1] = '\0';
		return true;
	}

	return false;
}

uint32_t SerialImpl::getBaudrate() const
{
	return _baudrate;
}

bool SerialImpl::setBaudrate(uint32_t baudrate)
{
	if (! validateBaudrate(baudrate)) {
		PX4_ERR("Invalid baudrate: %u", baudrate);
		return false;
	}

	// check if already configured
	if (baudrate == _baudrate) {
		return true;
	}

	_baudrate = baudrate;

	// process baud rate change now if port is already open
	if (_open) {
		return open();
	}

	return true;
}

ByteSize SerialImpl::getBytesize() const
{
	return _bytesize;
}

bool SerialImpl::setBytesize(ByteSize bytesize)
{
	return bytesize == ByteSize::EightBits;
}

Parity SerialImpl::getParity() const
{
	return _parity;
}

bool SerialImpl::setParity(Parity parity)
{
	return parity == Parity::None;
}

StopBits SerialImpl::getStopbits() const
{
	return _stopbits;
}

bool SerialImpl::setStopbits(StopBits stopbits)
{
	return stopbits == StopBits::One;
}

FlowControl SerialImpl::getFlowcontrol() const
{
	return _flowcontrol;
}

bool SerialImpl::setFlowcontrol(FlowControl flowcontrol)
{
	return flowcontrol == FlowControl::Disabled;
}

bool SerialImpl::getSingleWireMode() const
{
	return false;
}

bool SerialImpl::setSingleWireMode()
{
	// Qurt platform does not support single wire mode
	return false;
}

bool SerialImpl::getSwapRxTxMode() const
{
	return false;
}

bool SerialImpl::setSwapRxTxMode()
{
	// Qurt platform does not support swap rx tx mode
	return false;
}

bool SerialImpl::setInvertedMode(bool enable)
{
	// Qurt platform does not support inverted mode
	return false == enable;
}
bool SerialImpl::getInvertedMode() const
{
	return false;
}

} // namespace device
