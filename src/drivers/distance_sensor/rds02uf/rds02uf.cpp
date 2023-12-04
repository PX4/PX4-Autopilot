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

#include "rds02uf.hpp"

#include <lib/drivers/device/Device.hpp>
#include <parameters/param.h>
#include <fcntl.h>

#include <string.h>
#include <stdlib.h>

#define RDS02UF_HEAD_LEN 		2
#define RDS02UF_PRE_DATA_LEN		6
#define RDS02UF_DATA_LEN 		10
#define RDS02UF_DIST_MAX_M		20.0f
#define RDS02UF_DIST_MIN_M		1.5f
#define RDS02UF_DIST_HFOV		17.0f
#define RDS02UF_DIST_SFOV		3.0f
#define RDS02_TARGET_FC_INDEX_L		8
#define RDS02_TARGET_FC_INDEX_H		9
#define RDS02UF_IGNORE_ID_BYTE		0x0F0F
#define RDS02UF_UAV_PRODUCTS_ID		0x03FF
#define RDS02_TARGET_INFO_FC		0x070C
#define RDS02UF_TIMEOUT_MS		200
#define RDS02UF_IGNORE_CRC		0xFF
#define RDS02UF_NO_ERR			0x00

Rds02uf::Rds02uf(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_RDS02UF;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
}

Rds02uf::~Rds02uf()
{
	// make sure we are truly inactive
	stop();
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
Rds02uf::init()
{
	param_t _param_rds02uf_rot = param_find("SENS_RDS02_ROT");
	int32_t rotation;

	if (param_get(_param_rds02uf_rot, &rotation) == PX4_OK) {
		_px4_rangefinder.set_orientation(rotation);
	}

	_px4_rangefinder.set_min_distance(RDS02UF_DIST_MIN_M);
	_px4_rangefinder.set_max_distance(RDS02UF_DIST_MAX_M);
	_px4_rangefinder.set_hfov(math::radians(17.0f));
	_px4_rangefinder.set_vfov(math::radians(3.0f));

	// status
	int ret = 0;

	do { // create a scope to handle exit conditions using break

		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("Error opening fd");
			return -1;
		}

		// baudrate 115200, 8 bits, no parity, 1 stop bit
		unsigned speed = B115200;
		termios uart_config{};
		int termios_state{};

		tcgetattr(_fd, &uart_config);

		// clear ONLCR flag (which appends a CR for every LF)
		uart_config.c_oflag &= ~ONLCR;

		// set baud rate
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = -1;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;			// 8-bit characters
		uart_config.c_cflag &= ~PARENB;			// no parity bit
		uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
		uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

		// setup for non-canonical mode
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		// fetch bytes as they become available
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd < 0) {
			PX4_ERR("FAIL: laser fd");
			ret = -1;
			break;
		}
	} while (0);

	// close the fd
	::close(_fd);
	_fd = -1;

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

int
Rds02uf::collect()
{
	perf_begin(_sample_perf);
	// clear buffer if last read was too long ago
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);
	// the buffer for read chars is buflen minus null termination
	char readbuf[sizeof(parser_buffer)] {};
	unsigned readlen = sizeof(readbuf) - 1;
	int ret = 0;
	float distance_m = -1.0f;
	// Check the number of bytes available in the buffer
	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		perf_end(_sample_perf);
		return 0;
	}

	// parse entire buffer
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	do {
		// read from the sensor (uart buffer)
		ret = ::read(_fd, &readbuf[0], readlen);

		if (ret < 0) {
			PX4_ERR("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			// only throw an error if we time out
			if (read_elapsed > (kCONVERSIONINTERVAL * 2)) {
				/* flush anything in RX buffer */
				tcflush(_fd, TCIFLUSH);
				return ret;

			} else {
				return -EAGAIN;
			}
		}

		_last_read = hrt_absolute_time();

		// parse buffer
		for (int i = 0; i < ret; i++) {
			rds02uf_parse(readbuf[i], &distance_m);
		}

		// bytes left to parse
		bytes_available -= ret;

	} while (bytes_available > 0);

	// no valid measurement after parsing buffer
	if (distance_m < 0.0f) {
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	// publish most recent valid measurement from buffer
	_px4_rangefinder.update(timestamp_sample, distance_m);

	perf_end(_sample_perf);

	return PX4_OK;
}



int
Rds02uf::rds02uf_parse(char c, float *dist)
{
	int ret = -1;
	char data = c;

	switch (_parse_state) {
	case RDS02UF_PARSE_STATE::STATE0_SYNC_1:
		if (data == RDS02_HEAD1) {
			parser_buffer[parserbuf_index++] = data;
			_parse_state = RDS02UF_PARSE_STATE::STATE1_SYNC_2;
		}

		break;

	case RDS02UF_PARSE_STATE::STATE1_SYNC_2:
		if (data == RDS02_HEAD2) {
			parser_buffer[parserbuf_index++] = data;
			_parse_state = RDS02UF_PARSE_STATE::STATE2_ADDRESS;

		} else {
			parserbuf_index = 0;
			_parse_state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
		}

		break;

	case RDS02UF_PARSE_STATE::STATE2_ADDRESS: // address
		parser_buffer[parserbuf_index++] = data;
		_parse_state = RDS02UF_PARSE_STATE::STATE3_ERROR_CODE;
		break;

	case RDS02UF_PARSE_STATE::STATE3_ERROR_CODE: // error_code
		parser_buffer[parserbuf_index++] = data;
		_parse_state = RDS02UF_PARSE_STATE::STATE4_FC_CODE_L;
		break;

	case RDS02UF_PARSE_STATE::STATE4_FC_CODE_L: // fc_code low
		parser_buffer[parserbuf_index++] = data;
		_parse_state = RDS02UF_PARSE_STATE::STATE5_FC_CODE_H;
		break;

	case RDS02UF_PARSE_STATE::STATE5_FC_CODE_H: // fc_code high
		parser_buffer[parserbuf_index++] = data;
		_parse_state = RDS02UF_PARSE_STATE::STATE6_LENGTH_L;
		break;

	case RDS02UF_PARSE_STATE::STATE6_LENGTH_L: // lengh_low
		parser_buffer[parserbuf_index++] = data;
		_parse_state = RDS02UF_PARSE_STATE::STATE7_LENGTH_H;
		break;

	case RDS02UF_PARSE_STATE::STATE7_LENGTH_H: { // lengh_high
			uint8_t read_len = data << 8 | parser_buffer[parserbuf_index - 1];

			if (read_len == RDS02UF_DATA_LEN) {	// rds02uf data length is 10
				parser_buffer[parserbuf_index++] = data;
				_parse_state = RDS02UF_PARSE_STATE::STATE8_REAL_DATA;

			} else {
				parserbuf_index = 0;
				_parse_state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
			}

			break;
		}

	case RDS02UF_PARSE_STATE::STATE8_REAL_DATA: // real_data
		parser_buffer[parserbuf_index++] = data;

		if ((parserbuf_index) == (RDS02UF_HEAD_LEN + RDS02UF_PRE_DATA_LEN + RDS02UF_DATA_LEN)) {
			_parse_state = RDS02UF_PARSE_STATE::STATE9_CRC;
		}

		break;

	case RDS02UF_PARSE_STATE::STATE9_CRC: { // crc
#ifdef RDS02UF_USE_CRC
			uint8_t crc_data = crc8(&parser_buffer[2], RDS02UF_PRE_DATA_LEN + RDS02UF_DATA_LEN);
			parser_buffer[parserbuf_index++] = data;

			if (crc_data == data || data == RDS02UF_IGNORE_CRC) {
				_parse_state = RDS02UF_PARSE_STATE::STATE10_END_1;

			} else {
				parserbuf_index = 0;
				_parse_state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
			}

#else
			parser_buffer[parserbuf_index++] = data;
			_parse_state = RDS02UF_PARSE_STATE::STATE10_END_1;
#endif
			break;
		}

	case RDS02UF_PARSE_STATE::STATE10_END_1:
		if (data == RDS02_END) {
			parser_buffer[parserbuf_index++] = data;
			_parse_state = RDS02UF_PARSE_STATE::STATE11_END_2;

		} else {
			parserbuf_index = 0;
			_parse_state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;
		}

		break;

	case RDS02UF_PARSE_STATE::STATE11_END_2: {
			uint16_t fc_code = (parser_buffer[STATE5_FC_CODE_H] << 8 | parser_buffer[STATE4_FC_CODE_L]);
			uint8_t err_code = parser_buffer[STATE3_ERROR_CODE];

			if (data == RDS02_END) {
				if (fc_code == RDS02UF_UAV_PRODUCTS_ID && err_code == RDS02UF_NO_ERR) {	// get targer information
					uint16_t read_info_fc = (parser_buffer[RDS02_TARGET_FC_INDEX_H] << 8 | parser_buffer[RDS02_TARGET_FC_INDEX_L]);

					// read_info_fc = 0x70C + ID * 0x10, ID: 0~0xF
					if ((read_info_fc & RDS02UF_IGNORE_ID_BYTE) == RDS02_TARGET_INFO_FC) {
						*dist = (parser_buffer[RDS02_DATA_Y_INDEX + 1] * 256 + parser_buffer[RDS02_DATA_Y_INDEX]) / 100.0f;
						ret = true;
					}
				}
			}

			parserbuf_index = 0;
			_parse_state = RDS02UF_PARSE_STATE::STATE0_SYNC_1;

			break;
		}
	}

#ifdef RDS02UF_DEBUG
	static u_int16_t cnt;
	cnt += 1;
	static RDS02UF_PARSE_STATE last_state = RDS02UF_PARSE_STATE::STATE12_END_2;

	if (_parse_state != last_state || cnt > 500) {
		printf("state: %s,read: %02x\n", parser_state[_parse_state], data);
		last_state = _parse_state;
		cnt = 0;
	}

#endif

	return ret;
}

#ifdef RDS02UF_USE_CRC
uint8_t
Rds02uf::crc8(char *pbuf, int32_t len)
{
	char *data = pbuf;
	uint8_t crc = 0;

	while (len--) {
		crc = crc8_table[crc^ * (data++)];
	}

	return crc;
}
#endif

void
Rds02uf::start()
{
	// schedule a cycle to start things (the sensor sends at 20Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(47_ms);
}

void
Rds02uf::stop()
{
	ScheduleClear();
}

void
Rds02uf::Run()
{
	// fds initialized?
	if (_fd < 0) {
		// open fd
		_fd = ::open(_port, O_RDWR | O_NOCTTY);
	}

	// perform collection
	if (collect() == -EAGAIN) {
		// reschedule to grab the missing bits, time to transmit 21 bytes @ 115200 bps
		ScheduleClear();
		ScheduleOnInterval(57_ms, 87 * 21);
		return;
	}
}

void
Rds02uf::print_info()
{
	printf("Using port '%s'\n", _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
