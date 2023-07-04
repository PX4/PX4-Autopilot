/**************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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

#include "lightware_sf45_serial.hpp"
#include "sf45_commands.h"

#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>
#include <lib/crc/crc.h>

#include <float.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

/* Configuration Constants */
#define SF45_MAX_PAYLOAD 256
#define SF45_SCALE_FACTOR 0.01f

SF45LaserSerial::SF45LaserSerial(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_num_retries = 2;
	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_LIGHTWARE_LASER);

	// populate obstacle map members
	_obstacle_map_msg.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_map_msg.increment = 5;
	_obstacle_map_msg.angle_offset = -2.5;
	_obstacle_map_msg.min_distance = UINT16_MAX;
	_obstacle_map_msg.max_distance = 5000;

}

SF45LaserSerial::~SF45LaserSerial()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int SF45LaserSerial::init()
{

	param_get(param_find("SF45_UPDATE_CFG"), &_update_rate);
	param_get(param_find("SF45_ORIENT_CFG"), &_orient_cfg);
	param_get(param_find("SF45_YAW_CFG"), &_yaw_cfg);

	/* SF45/B (50M) */
	_px4_rangefinder.set_min_distance(0.2f);
	_px4_rangefinder.set_max_distance(50.0f);
	_interval = 10000;

	start();

	return PX4_OK;
}

int SF45LaserSerial::measure()
{

	int rate = (int)_update_rate;
	_data_output = 0x101; // raw distance + yaw readings
	_stream_data = 5; // enable constant streaming

	// send some packets so the sensor starts scanning
	switch (_sensor_state) {

	// sensor should now respond
	case 0:
		while (_num_retries--) {
			sf45_send(SF_PRODUCT_NAME, false, &_product_name[0], 0);
			_sensor_state = 0;
		}

		_sensor_state = 1;
		break;

	case 1:
		// Update rate default to 50 readings/s
		sf45_send(SF_UPDATE_RATE, true, &rate, sizeof(uint8_t));
		_sensor_state = 2;
		break;

	case 2:
		sf45_send(SF_DISTANCE_OUTPUT, true, &_data_output, sizeof(_data_output));
		_sensor_state = 3;
		break;

	case 3:
		sf45_send(SF_STREAM, true, &_stream_data, sizeof(_stream_data));
		_sensor_state = 4;
		break;

	default:
		break;
	}

	return PX4_OK;
}

int SF45LaserSerial::collect()
{
	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);
	int ret;
	/* the buffer for read chars is buflen minus null termination */
	param_get(param_find("SF45_CP_LIMIT"), &_collision_constraint);
	uint8_t readbuf[SF45_MAX_PAYLOAD];

	float distance_m = -1.0f;

	/* read from the sensor (uart buffer) */
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (_sensor_state == 1) {

		ret = ::read(_fd, &readbuf[0], 22);
		sf45_request_handle(ret, readbuf);
		ScheduleDelayed(_interval * 2);

	} else if (_sensor_state == 2) {

		ret = ::read(_fd, &readbuf[0], 7);

		if (readbuf[3] == SF_UPDATE_RATE) {
			sf45_request_handle(ret, readbuf);
			ScheduleDelayed(_interval * 5);
		}

	} else if (_sensor_state == 3) {

		ret = ::read(_fd, &readbuf[0], 8);

		if (readbuf[3] == SF_DISTANCE_OUTPUT) {
			sf45_request_handle(ret, readbuf);
			ScheduleDelayed(_interval * 5);
		}

	} else {

		ret = ::read(_fd, &readbuf[0], 10);
		uint8_t flags_payload = (readbuf[1] >> 6) | (readbuf[2] << 2);

		if (readbuf[3] == SF_DISTANCE_DATA_CM && flags_payload == 5) {

			for (uint8_t i = 0; i < ret; ++i) {
				sf45_request_handle(ret, readbuf);
			}

			if (_init_complete) {
				sf45_process_replies(&distance_m);
			} // end if

		} else {

			ret = ::read(_fd, &readbuf[0], 10);

		}
	}

	if (ret < 0) {
		PX4_DEBUG("read err: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		/* only throw an error if we time out */
		if (read_elapsed > (_interval * 2)) {
			PX4_DEBUG("Timing out...");
			return ret;

		} else {

			return -EAGAIN;
		}

	} else if (ret == 0) {
		return -EAGAIN;
	}

	_last_read = hrt_absolute_time();

	if (!_crc_valid) {
		return -EAGAIN;
	}

	PX4_DEBUG("val (float): %8.4f, raw: %s, valid: %s", (double)distance_m, _linebuf, ((_crc_valid) ? "OK" : "NO"));
	_px4_rangefinder.update(timestamp_sample, distance_m);

	perf_end(_sample_perf);

	return PX4_OK;
}

void SF45LaserSerial::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void SF45LaserSerial::stop()
{
	ScheduleClear();
}

void SF45LaserSerial::Run()
{
	/* fds initialized? */
	if (_fd < 0) {
		/* open fd: non-blocking read mode*/

		_fd = ::open(_port, O_RDWR | O_NOCTTY);

		if (_fd < 0) {
			PX4_ERR("open failed (%i)", errno);
			return;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_fd, &uart_config);

		uart_config.c_cflag = (uart_config.c_cflag & ~CSIZE) | CS8;

		uart_config.c_cflag |= (CLOCAL | CREAD);

		// no parity, 1 stop bit, flow control disabled
		uart_config.c_cflag &= ~(PARENB | PARODD);

		uart_config.c_cflag |= 0;

		uart_config.c_cflag &= ~CSTOPB;

		uart_config.c_cflag &= ~CRTSCTS;

		uart_config.c_iflag &= ~IGNBRK;

		uart_config.c_iflag &= ~ICRNL;

		uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);

		// echo and echo NL off, canonical mode off (raw mode)
		// extended input processing off, signal chars off
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

		uart_config.c_oflag = 0;

		uart_config.c_cc[VMIN] = 0;

		uart_config.c_cc[VTIME] = 1;

		unsigned speed = B921600;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
		}

	}

	if (_collect_phase) {

		/* perform collection */
		int collect_ret = collect();

		if (collect_ret == -EAGAIN) {
			/* reschedule to grab the missing bits, time to transmit 8 bytes @ 9600 bps */
			ScheduleDelayed(1042 * 8);

			return;
		}

		if (OK != collect_ret) {
			/* we know the sensor needs about four seconds to initialize */
			if (hrt_absolute_time() > 5 * 1000 * 1000LL && _consecutive_fail_count < 5) {
				PX4_ERR("collection error #%u", _consecutive_fail_count);
			}

			_consecutive_fail_count++;

			/* restart the measurement state machine */
			start();
			return;

		} else {
			/* apparently success */
			_consecutive_fail_count = 0;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* measurement phase */

	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(_interval);

}

void SF45LaserSerial::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void SF45LaserSerial::sf45_request_handle(int return_val, uint8_t *input_buf)
{

	// SF45 protocol
	// Start byte is 0xAA and is the start of packet
	// Payload length sanity check (0-1023) bytes
	// and represented by 16-bit integer (payload +
	// read/write status)
	// ID byte precedes the data in the payload
	// CRC comprised of 16-bit checksum (not included in checksum calc.)

	uint16_t recv_crc = 0;
	bool restart_flag = false;

	while (restart_flag != true) {

		switch (_parsed_state) {
		case 0: {
				if (input_buf[0] == 0xAA) {
					// start of frame is valid, continue
					_sop_valid = true;
					_calc_crc = sf45_format_crc(_calc_crc, _start_of_frame);
					_parsed_state = 1;
					break;

				} else {
					_sop_valid = false;
					_crc_valid = false;
					_parsed_state = 0;
					restart_flag = true;
					_calc_crc = 0;
					PX4_INFO("INFO: start of packet not valid");
					break;
				} // end else
			} // end case 0

		case 1: {
				rx_field.flags_lo = input_buf[1];
				_calc_crc = sf45_format_crc(_calc_crc, rx_field.flags_lo);
				_parsed_state = 2;
				break;
			}

		case 2: {
				rx_field.flags_hi = input_buf[2];
				rx_field.data_len = (rx_field.flags_hi << 2) | (rx_field.flags_lo >> 6);
				_calc_crc = sf45_format_crc(_calc_crc, rx_field.flags_hi);

				// Check payload length against known max value
				if (rx_field.data_len > 17) {
					PX4_INFO("INFO: payload length invalid, restarting data request");
					_parsed_state = 0;
					restart_flag = true;
					_calc_crc = 0;
					break;

				} else {
					_parsed_state = 3;
					break;
				}
			}

		case 3: {

				rx_field.msg_id = input_buf[3];

				if (rx_field.msg_id == SF_PRODUCT_NAME || rx_field.msg_id == SF_UPDATE_RATE || rx_field.msg_id == SF_DISTANCE_OUTPUT
				    || rx_field.msg_id == SF_STREAM || rx_field.msg_id == SF_DISTANCE_DATA_CM) {

					if (rx_field.msg_id == SF_DISTANCE_DATA_CM && rx_field.data_len > 1) {
						_sensor_ready = true;

					} else {
						_sensor_ready = false;
					}

					_calc_crc = sf45_format_crc(_calc_crc, rx_field.msg_id);

					_parsed_state = 4;
					break;
				}

				// Ignore message ID's that aren't defined
				else {
					_parsed_state = 0;
					_calc_crc = 0;
					restart_flag = true;
					break;

				}
			}

		// Data
		case 4: {
				// Process commands with  & w/out data bytes
				if (rx_field.data_len > 1) {
					for (uint8_t i = 4; i < 3 + rx_field.data_len; ++i) {

						rx_field.data[_data_bytes_recv] = input_buf[i];
						_calc_crc = sf45_format_crc(_calc_crc, rx_field.data[_data_bytes_recv]);
						_data_bytes_recv = _data_bytes_recv + 1;

					}  // end for
				} //end if

				else {

					_parsed_state = 5;
					_data_bytes_recv = 0;
					_calc_crc = sf45_format_crc(_calc_crc, _data_bytes_recv);

				}

				_parsed_state = 5;
				_data_bytes_recv = 0;
				break;
			}

		// CRC low byte
		case 5: {
				rx_field.crc[0] = input_buf[3 + rx_field.data_len];
				_parsed_state = 6;
				break;
			}

		// CRC high byte
		case 6: {
				rx_field.crc[1] = input_buf[4 + rx_field.data_len];
				recv_crc = (rx_field.crc[1] << 8) | rx_field.crc[0];

				// Check the received crc bytes from the sf45 against our own CRC calcuation
				// If it matches, we can check if sensor ready
				// Only if crc match is valid and sensor ready (transmitting distance data) do we flag _init_complete
				if (recv_crc == _calc_crc) {
					_crc_valid = true;

					// Sensor is ready if we read msg ID 44: SF_DISTANCE_DATA_CM
					if (_sensor_ready) {
						_init_complete = true;

					} else {
						_init_complete = false;
					}

					_parsed_state = 0;
					_calc_crc = 0;
					restart_flag = true;
					break;

				} else {
					PX4_INFO("INFO: CRC mismatch");
					_crc_valid = false;
					_init_complete = false;
					_parsed_state = 0;
					_calc_crc = 0;
					restart_flag = true;
					break;
				}
			}
		} // end switch
	} //end while
}

void SF45LaserSerial::sf45_send(uint8_t msg_id, bool write, int *data, uint8_t data_len)
{
	uint16_t crc_val = 0;
	uint8_t packet_buff[SF45_MAX_PAYLOAD];
	uint8_t data_inc = 4;
	int ret = 0;
	uint8_t packet_len = 0;
	// Include payload ID byte in payload len
	uint16_t flags = (data_len + 1) << 6;

	// If writing to the device, lsb is 1
	if (write) {
		flags |= 0x01;
	}

	else {
		flags |= 0x0;
	}

	uint8_t flags_lo = flags & 0xFF;
	uint8_t flags_hi = (flags >> 8) & 0xFF;

	// Add packet bytes to format into crc based on CRC-16-CCITT 0x1021/XMODEM
	crc_val = sf45_format_crc(crc_val, _start_of_frame);
	crc_val = sf45_format_crc(crc_val, flags_lo);
	crc_val = sf45_format_crc(crc_val, flags_hi);
	crc_val = sf45_format_crc(crc_val, msg_id);

	// Write the packet header contents + payload msg ID to the output buffer
	packet_buff[0] = _start_of_frame;
	packet_buff[1] = flags_lo;
	packet_buff[2] = flags_hi;
	packet_buff[3] = msg_id;

	if (msg_id == SF_DISTANCE_OUTPUT) {
		uint8_t data_convert = data[0] & 0x00FF;
		// write data bytes to the output buffer
		packet_buff[data_inc] =  data_convert;
		// Add data bytes to crc add function
		crc_val = sf45_format_crc(crc_val, data_convert);
		data_inc = data_inc + 1;
		data_convert = data[0] >> 8;
		packet_buff[data_inc] = data_convert;
		crc_val = sf45_format_crc(crc_val, data_convert);
		data_inc = data_inc + 1;
		packet_buff[data_inc] = 0;
		crc_val = sf45_format_crc(crc_val, packet_buff[data_inc]);
		data_inc = data_inc + 1;
		packet_buff[data_inc] = 0;
		crc_val = sf45_format_crc(crc_val, packet_buff[data_inc]);
		data_inc = data_inc + 1;
	}

	else if (msg_id == SF_STREAM) {
		packet_buff[data_inc] = data[0];
		//pad zeroes
		crc_val = sf45_format_crc(crc_val, data[0]);
		data_inc = data_inc + 1;
		packet_buff[data_inc] = 0;
		crc_val = sf45_format_crc(crc_val, packet_buff[data_inc]);
		data_inc = data_inc + 1;
		packet_buff[data_inc] = 0;
		crc_val = sf45_format_crc(crc_val, packet_buff[data_inc]);
		data_inc = data_inc + 1;
		packet_buff[data_inc] = 0;
		crc_val = sf45_format_crc(crc_val, packet_buff[data_inc]);
		data_inc = data_inc + 1;
	}

	else if (msg_id == SF_UPDATE_RATE) {
		// Update Rate
		packet_buff[data_inc] = (uint8_t)data[0];
		// Add data bytes to crc add function
		crc_val = sf45_format_crc(crc_val, data[0]);
		data_inc = data_inc + 1;

	}

	else {
		// Product Name
		PX4_INFO("INFO: Product name");
	}

	uint8_t crc_lo = crc_val & 0xFF;
	uint8_t crc_hi = (crc_val >> 8) & 0xFF;

	packet_buff[data_inc] = crc_lo;
	data_inc = data_inc + 1;
	packet_buff[data_inc] = crc_hi;

	size_t len = sizeof(packet_buff[0]) * (data_inc + 1);
	packet_len = (uint8_t)len;

	// DEBUG
	for (uint8_t i = 0; i < packet_len; ++i) {
		PX4_INFO("INFO: Send byte: %d", packet_buff[i]);
	}

	ret = ::write(_fd, packet_buff, packet_len);

	if (ret != packet_len) {
		perf_count(_comms_errors);
		PX4_DEBUG("write fail %d", ret);
		//return ret;
	}
}

void SF45LaserSerial::sf45_process_replies(float *distance_m)
{

	switch (rx_field.msg_id) {
	case SF_DISTANCE_DATA_CM: {

			uint16_t obstacle_dist_cm = 0;
			const float raw_distance = (rx_field.data[0] << 0) | (rx_field.data[1] << 8);
			int16_t raw_yaw = ((rx_field.data[2] << 0) | (rx_field.data[3] << 8));
			int16_t scaled_yaw = 0;

			// The sensor scans from 0 to -160, so extract negative angle from int16 and represent as if a float
			if (raw_yaw > 32000) {
				raw_yaw = raw_yaw - 65535;

			}

			// The sensor is facing downward, so the sensor is flipped about it's x-axis -inverse of each yaw angle
			if (_orient_cfg == 1) {
				raw_yaw = raw_yaw * -1;
			}

			switch (_yaw_cfg) {
			case 0:
				break;

			case 1:
				if (raw_yaw > 180) {
					raw_yaw = raw_yaw - 180;

				} else {
					raw_yaw = raw_yaw + 180; // rotation facing aft
				}

				break;

			case 2:
				raw_yaw = raw_yaw + 90; // rotation facing right
				break;

			case 3:
				raw_yaw = raw_yaw - 90; // rotation facing left
				break;

			default:
				break;
			}

			scaled_yaw = raw_yaw * SF45_SCALE_FACTOR;

			// Convert to meters for rangefinder update
			*distance_m = raw_distance * SF45_SCALE_FACTOR;
			obstacle_dist_cm = (uint16_t)raw_distance;

			uint8_t current_bin = sf45_convert_angle(scaled_yaw);

			// If we have moved to a new bin

			if (current_bin != _previous_bin) {

				// update the current bin to the distance sensor reading
				// readings in cm
				_obstacle_map_msg.distances[current_bin] = obstacle_dist_cm;
				_obstacle_map_msg.timestamp = hrt_absolute_time();

			}

			_previous_bin = current_bin;

			_obstacle_distance_pub.publish(_obstacle_map_msg);

			break;
		}

	default:
		// add case for future use
		break;
	}
}

uint8_t SF45LaserSerial::sf45_convert_angle(const int16_t yaw)
{

	uint8_t mapped_sector = 0;
	float adjusted_yaw = sf45_wrap_360(yaw - _obstacle_map_msg.angle_offset);
	mapped_sector = round(adjusted_yaw / _obstacle_map_msg.increment);

	return mapped_sector;
}

float SF45LaserSerial::sf45_wrap_360(float f)
{
	return matrix::wrap(f, 0.f, 360.f);
}

uint16_t SF45LaserSerial::sf45_format_crc(uint16_t crc, uint8_t data_val)
{
	uint32_t i;
	const uint16_t poly = 0x1021u;
	crc ^= (uint16_t)((uint16_t) data_val << 8u);

	for (i = 0; i < 8; i++) {
		if (crc & (1u << 15u)) {
			crc = (uint16_t)((crc << 1u) ^ poly);

		} else {
			crc = (uint16_t)(crc << 1u);
		}
	}

	return crc;
}
