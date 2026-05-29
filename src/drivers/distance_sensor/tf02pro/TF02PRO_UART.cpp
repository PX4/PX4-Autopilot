/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file TF02PRO_UART.cpp
 *
 * UART driver for the Benewake TF02 Pro distance sensor.
 */

#include "TF02PRO_UART.hpp"

#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>

TF02PRO_UART::TF02PRO_UART(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';

	// Build a device ID using the same devtype as the I2C driver so EKF sees one sensor type
	device::Device::DeviceId device_id;
	device_id.devid_s.devtype  = DRV_DIST_DEVTYPE_TF02PRO;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	uint8_t bus_num = (uint8_t)atoi(&_port[strlen(_port) - 1]);

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);

	// Physical parameters set in constructor (NOT in init) so that print_info() and
	// print_status() work correctly at any point after construction.
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_TF02PRO);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
	_px4_rangefinder.set_min_distance(TF02PRO_UART_MIN_DISTANCE);
	_px4_rangefinder.set_max_distance(TF02PRO_UART_MAX_DISTANCE);
	_px4_rangefinder.set_fov(math::radians(3.0f));
}

TF02PRO_UART::~TF02PRO_UART()
{
	stop();

	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}

	perf_free(_comms_errors);
	perf_free(_sample_perf);
}

int TF02PRO_UART::init()
{
	// Open tty to configure termios, then close. NuttX persists termios config on
	// the device node; Run() re-opens the fd on first cycle without reconfiguring.
	int fd = ::open(_port, O_RDWR | O_NOCTTY);

	if (fd < 0) {
		PX4_ERR("open %s failed: %d", _port, errno);
		return PX4_ERROR;
	}

	termios uart_config{};
	tcgetattr(fd, &uart_config);
	uart_config.c_oflag &= ~ONLCR;

	if (cfsetispeed(&uart_config, B115200) < 0 ||
	    cfsetospeed(&uart_config, B115200) < 0) {
		PX4_ERR("baud set failed");
		::close(fd);
		return PX4_ERROR;
	}

	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;
	uart_config.c_cflag &= ~PARENB;
	uart_config.c_cflag &= ~CSTOPB;
	uart_config.c_cflag &= ~CRTSCTS;
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	uart_config.c_oflag &= ~OPOST;
	uart_config.c_cc[VMIN]  = 1;
	uart_config.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr failed");
		::close(fd);
		return PX4_ERROR;
	}

	// 1. Send command to set frame rate to 250Hz (0x5A 0x06 0x03 0xFA 0x00 0x5D)
	const uint8_t cmd_250hz[] = {0x5A, 0x06, 0x03, 0xFA, 0x00, 0x5D};
	::write(fd, cmd_250hz, sizeof(cmd_250hz));
	usleep(10000); // 10ms delay

	// 2. Send save settings command (0x5A 0x04 0x11 0x6F) so it persists
	const uint8_t cmd_save[] = {0x5A, 0x04, 0x11, 0x6F};
	::write(fd, cmd_save, sizeof(cmd_save));
	usleep(10000);

	::close(fd);   // closed here, re-opened lazily in Run()

	start();
	return PX4_OK;
}

int TF02PRO_UART::collect()
{
	perf_begin(_sample_perf);

	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		perf_end(_sample_perf);
		return 0;
	}

	uint8_t readbuf[9] {};
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);
	int     ret          = 0;
	float   distance_m   = -1.0f;
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	do {
		ret = ::read(_fd, readbuf, sizeof(readbuf));

		if (ret < 0) {
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			if (read_elapsed > (kCONVERSIONINTERVAL * 2)) {
				tcflush(_fd, TCIFLUSH);
				return ret;
			}

			return -EAGAIN;
		}

		_last_read = hrt_absolute_time();

		// Parse all read bytes; keep only the last valid frame
		for (int i = 0; i < ret; i++) {
			float candidate = -1.0f;

			if (tf02pro_parse(readbuf[i], _uart_buf, &_uart_buf_idx,
					  &_parse_state, &candidate) == 0) {
				distance_m = candidate;   // last valid frame wins
			}
		}

		bytes_available -= ret;

	} while (bytes_available > 0);

	if (distance_m >= 0.0f) {
		_px4_rangefinder.update(timestamp_sample, distance_m);
	}

	perf_end(_sample_perf);
	return (distance_m >= 0.0f) ? PX4_OK : -EAGAIN;
}

void TF02PRO_UART::Run()
{
	if (_fd < 0) {
		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			return; // try again on next scheduled cycle
		}
	}

	collect();
}

void TF02PRO_UART::start()
{
	ScheduleOnInterval(kCONVERSIONINTERVAL);
}

void TF02PRO_UART::stop()
{
	ScheduleClear();
}

void TF02PRO_UART::print_info()
{
	printf("TF02PRO UART on port: %s\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
