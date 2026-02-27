/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "JRE30.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <lib/drivers/device/Device.hpp>

JRE30::JRE30(const char *serial_port, uint8_t device_orientation):
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port)),
	_px4_rangefinder(0, device_orientation)
{
	_serial_port = strdup(serial_port);
	_file_descriptor = -1;
	memset(_buffer, 0, sizeof(_buffer));
	_buffer_len = 0;
	_radio_strength = 0;
	_status.value = 0;
	_crc16_calc = 0;
	_measurement_time = 0;

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_serial_port[strlen(_serial_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_JRE30);
	_px4_rangefinder.set_max_distance(JRE30_MAX_DISTANCE);
	_px4_rangefinder.set_min_distance(JRE30_MIN_DISTANCE);
	_px4_rangefinder.set_fov(0.3490658503988659);  // degrees 20.0 to radians
}

JRE30::~JRE30()
{
	stop();

	free((char *)_serial_port);
	perf_free(_comms_error);
	perf_free(_sample_perf);
}

uint16_t
JRE30::crc16_signature(const uint8_t *data, uint32_t length, uint16_t crc, uint16_t final_xor)
{
	for (uint32_t i = 0; i < length; ++i) {
		crc ^= static_cast<uint16_t>(data[i]);

		for (uint8_t j = 0; j < 8; ++j) {
			crc = (crc & 0x0001) ? (0x8408 ^ (crc >> 1)) : (crc >> 1);
		}
	}

	return crc ^ final_xor;
}

int
JRE30::collect()
{
	perf_begin(_sample_perf);

	size_t buffer_size = sizeof(_buffer);
	const int message_size = sizeof(reading_msg);

	int bytes_read = ::read(_file_descriptor, _buffer + _buffer_len, buffer_size - _buffer_len);

	if (bytes_read < 1) {
		// Trigger a new measurement.
		return measure();
	}

	_buffer_len += bytes_read;

	if ((_buffer_len >= 1 && _buffer[0] != 'R') || (_buffer_len >= 2 && _buffer[1] != 'A')) {
		_buffer_len = 0;
		return measure();
	}

	if (_buffer_len < message_size) {
		// Return on next scheduled cycle to collect remaining data.
		return PX4_OK;
	}

	reading_msg *msg {nullptr};
	msg = (reading_msg *)_buffer;

	_crc16_calc = crc16_signature((uint8_t *)_buffer, size_t(buffer_size - 2), 0xffffu, 0xffffu);

	if (_crc16_calc != msg->crc) {
		PX4_ERR("crc error");
		perf_count(_comms_error);
		perf_end(_sample_perf);
		return measure();
	}

	_status.value = msg->status_high << 8 | msg->status_low;

	if (_status.bits.distance_disable) {
		PX4_ERR("distance not enabled");
		perf_count(_comms_error);
		perf_end(_sample_perf);
		return measure();
	}

	_radio_strength = msg->radio_strength_high << 8 | msg->radio_strength_low;

	uint16_t distance_cm = (msg->distance_high << 8 | msg->distance_low);
	float distance_m = static_cast<float>(distance_cm) / 100.0f;

	int8_t signal_quality = (_radio_strength / 300) >= 100 ? 100 : (_radio_strength / 300);

	_px4_rangefinder.update(_measurement_time, distance_m, signal_quality);

	perf_end(_sample_perf);

	// Trigger the next measurement.
	return measure();
}

int
JRE30::init()
{
	if (open_serial_port() != PX4_OK) {
		return PX4_ERROR;
	}

	hrt_abstime time_now = hrt_absolute_time();

	const hrt_abstime timeout_usec = time_now + 500000_us; // 0.5sec

	while (time_now < timeout_usec) {
		if (measure() == PX4_OK) {
			px4_usleep(JRE30_MEASURE_INTERVAL);

			if (collect() == PX4_OK) {
				// The file descriptor can only be accessed by the process that opened it,
				// so closing here allows the port to be opened from scheduled work queue.
				stop();
				return PX4_OK;
			}
		}

		px4_usleep(1000);
		time_now = hrt_absolute_time();
	}

	PX4_ERR("No readings from JRE30");
	return PX4_ERROR;
}

int
JRE30::measure()
{
	// Flush the receive buffer.
	tcflush(_file_descriptor, TCIFLUSH);

	_measurement_time = hrt_absolute_time();
	_buffer_len = 0;
	return PX4_OK;
}

int
JRE30::open_serial_port(const speed_t speed)
{
	// File descriptor already initialized?
	if (_file_descriptor > 0) {
		// PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_serial_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	if (tcgetattr(_file_descriptor, &uart_config)) {
		PX4_ERR("Unable to get termios from %s.", _serial_port);
		::close(_file_descriptor);
		_file_descriptor = -1;
		return PX4_ERROR;
	}

	// Clear: data bit size, two stop bits, parity, hardware flow control.
	uart_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);

	// Set: 8 data bits, enable receiver, ignore modem status lines.
	uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

	// Clear: echo, echo new line, canonical input and extended input.
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// Set the input baud rate in the uart_conmsg->fig struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Flush the hardware buffers.
	tcflush(_file_descriptor, TCIOFLUSH);

	PX4_DEBUG("opened UART port %s", _serial_port);

	return PX4_OK;
}

void
JRE30::print_info()
{
	perf_print_counter(_comms_error);
	perf_print_counter(_sample_perf);

	PX4_INFO_RAW("jre30: msg %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		     _buffer[0],  _buffer[1],  _buffer[2],  _buffer[3],
		     _buffer[4],  _buffer[5],  _buffer[6],  _buffer[7],
		     _buffer[8],  _buffer[9],  _buffer[10], _buffer[11],
		     _buffer[12], _buffer[13], _buffer[14], _buffer[15]);
	PX4_INFO_RAW("jre30: crc_calc %04x crc_msg %04x\n", _crc16_calc, ((reading_msg *)_buffer)->crc);
	PX4_INFO_RAW("jre30: status %04x strength %04x\n", _status.value, _radio_strength);
}

void
JRE30::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	collect();
}

void
JRE30::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(JRE30_MEASURE_INTERVAL, JRE30_MEASURE_INTERVAL);
}

void
JRE30::stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);
	_file_descriptor = -1;

	// Clear the work queue schedule.
	ScheduleClear();
}
