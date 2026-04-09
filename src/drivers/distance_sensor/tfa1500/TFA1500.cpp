/****************************************************************************
 *
 *   Copyright (c) 2017-2021 PX4 Development Team. All rights reserved.
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

#include "TFA1500.hpp"

#include <lib/parameters/param.h>
#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>
TFA1500::TFA1500(const char *port, uint8_t rotation) : ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
													   _px4_rangefinder(0, rotation)
{
	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_TFA1500;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10)
	{
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
}

TFA1500::~TFA1500()
{
	// make sure we are truly inactive
	stop();
	if (_fd >= 0)
	{
		::close(_fd);
		_fd = -1;
		PX4_INFO("close fd %d", _fd);
	}
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TFA1500::init()
{
	int32_t hw_model = 1;
	param_get(param_find("SENS_TFA1500_HW"), &hw_model);

	switch (hw_model)
	{
	case 1: // TFA1500 (13000m, 100 Hz)
		_px4_rangefinder.set_min_distance(0.5f);
		_px4_rangefinder.set_max_distance(13000.0f);
		_px4_rangefinder.set_fov(math::radians(2.3f));
		break;

	default:
		PX4_ERR("invalid HW model %" PRId32 ".", hw_model);
		return -1;
	}

	// status
	int ret = 0;

	do
	{ // create a scope to handle exit conditions using break

		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0)
		{
			PX4_ERR("Error opening fd");
			return -1;
		}

		// baudrate 115200, 8 bits, no parity, 1 stop bit
		unsigned speed = B500000;
		termios uart_config{};
		int termios_state{};

		tcgetattr(_fd, &uart_config);

		// clear ONLCR flag (which appends a CR for every LF)
		uart_config.c_oflag &= ~ONLCR;

		// set baud rate
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0)
		{
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0)
		{
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0)
		{
			PX4_ERR("baud %d ATTR", termios_state);
			ret = -1;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD); // ignore modem controls
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;		 // 8-bit characters
		uart_config.c_cflag &= ~PARENB;	 // no parity bit
		uart_config.c_cflag &= ~CSTOPB;	 // only need 1 stop bit
		uart_config.c_cflag &= ~CRTSCTS; // no hardware flowcontrol

		// setup for non-canonical mode
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		// fetch bytes as they become available
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd < 0)
		{
			PX4_ERR("FAIL: laser fd");
			ret = -1;
			break;
		}
	} while (0);
	::close(_fd);
	_fd = -1;
	if (ret == PX4_OK)
	{
		start();
	}

	return ret;
}

int TFA1500::collect()
{
	perf_begin(_sample_perf);
	int ret = 0;
	float distance_m = -1.0f;
	static bool distance_flag = false;
	if(!distance_flag)
	{
		for (int i = 0; i < 8; i++)
		{
			ret = ::write(_fd, &TFA1500_CMD_START[i], 1);
			if (ret < 0)
			{
				PX4_ERR("Send start command failed: %zd, len=%zu, errno=%d", ret, 1, errno);
				perf_count(_comms_errors);
				return -1;
			}
		}
		PX4_WARN("collect start");
		PX4_WARN("collect start return %d", ret);

	}



	// clear buffer if last read was too long ago
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	// the buffer for read chars is buflen minus null termination
	char readbuf[sizeof(_linebuf)]{};
	unsigned readlen = sizeof(readbuf) - 1;




	// Check the number of bytes available in the buffer
	int bytes_available = 0;

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	PX4_WARN("collect read");
	// do
	// {
		// read from the sensor (uart buffer)
		ret = ::read(_fd, &readbuf[0], readlen);

		if (ret < 0)
		{
			PX4_ERR("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			distance_flag = false;
			// only throw an error if we time out
			if (read_elapsed > (kCONVERSIONINTERVAL * 2))
			{
				/* flush anything in RX buffer */
				tcflush(_fd, TCIFLUSH);
				return ret;
			}
			else
			{
				return -EAGAIN;
			}
		}

		_last_read = hrt_absolute_time();

		// parse buffer
		for (int i = 0; i < ret; i++)
		{
			tfa1500_parse(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &distance_m);
			PX4_WARN("collect parse: %02x", (uint8_t)readbuf[i]);
		}

		// bytes left to parse
		bytes_available -= ret;

	//} while (bytes_available > 0);
	PX4_WARN("collect distance: %f", (double)distance_m);
	// no valid measurement after parsing buffer
	if (distance_m < 0.0f)
	{
		perf_end(_sample_perf);
		distance_flag = false;
		return -EAGAIN;
	}
	distance_flag = true;
	// publish most recent valid measurement from buffer
	_px4_rangefinder.update(timestamp_sample, distance_m);

	perf_end(_sample_perf);
	// send stop command
	PX4_WARN("collect stop");
        // ret = ::write(_fd, &TFA1500_CMD_STOP[0], sizeof(TFA1500_CMD_STOP));
	// if (ret != sizeof(TFA1500_CMD_STOP))
	// {
	// 	PX4_ERR("Send stop command failed: %zd, len=%zu, errno=%d", ret, sizeof(TFA1500_CMD_STOP), errno);
	// 	perf_count(_comms_errors);
	// 	return -1;
	// }
	return PX4_OK;
}

void TFA1500::start()
{
	// schedule a cycle to start things (the sensor sends at 100Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(7_ms);
}

void TFA1500::stop()
{
	ScheduleClear();
}

void TFA1500::Run()
{
	// fds initialized?
	// perform collection
	if (_fd < 0)
	{
		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
		PX4_WARN("run open fd: %d", _fd);
	}
	PX4_WARN("run collect");
	if (collect() == -EAGAIN)
	{
		// reschedule to grab the missing bits, time to transmit 9 bytes @ 115200 bps
		ScheduleClear();
		ScheduleOnInterval(7_ms, 87 * 9);
		return;
	}
}

void TFA1500::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
