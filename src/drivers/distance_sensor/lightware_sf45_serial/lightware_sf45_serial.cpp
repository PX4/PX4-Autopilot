/**************************************************************************
 *
 *   Copyright (c) 2022-2024 PX4 Development Team. All rights reserved.
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

#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>
#include <lib/crc/crc.h>
#include <lib/mathlib/mathlib.h>

#include <float.h>

using namespace time_literals;

/* Configuration Constants */
#define SF45_SCALE_FACTOR 0.01f

SF45LaserSerial::SF45LaserSerial(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
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

	// populate obstacle map members
	_obstacle_map_msg.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_map_msg.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_LASER;
	_obstacle_map_msg.increment = 5;
	_obstacle_map_msg.min_distance = 20;
	_obstacle_map_msg.max_distance = 5000;
	_obstacle_map_msg.angle_offset = 0;

	for (uint32_t i = 0 ; i < BIN_COUNT; i++) {
		_obstacle_map_msg.distances[i] = UINT16_MAX;
	}
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

	start();
	return PX4_OK;
}

int SF45LaserSerial::measure()
{
	int32_t rate = (int32_t)_update_rate;
	_data_output = 0x101; // raw distance (first return) + yaw readings
	_stream_data = 5; // enable constant streaming

	// send packets to the sensor depending on the state
	switch (_sensor_state) {

	case STATE_UNINIT:
		// Used to probe if the sensor is alive
		sf45_send(SF_PRODUCT_NAME, false, &_product_name[0], 0);
		break;

	case STATE_ACK_PRODUCT_NAME:
		// Update rate default to 50 readings/s
		sf45_send(SF_UPDATE_RATE, true, &rate, sizeof(uint8_t));
		break;

	case STATE_ACK_UPDATE_RATE:
		// Configure the data that the sensor shall output
		sf45_send(SF_DISTANCE_OUTPUT, true, &_data_output, sizeof(_data_output));
		break;

	case STATE_ACK_DISTANCE_OUTPUT:
		// Configure the sensor to automatically output data at the configured update rate
		sf45_send(SF_STREAM, true, &_stream_data, sizeof(_stream_data));
		_sensor_state = STATE_SEND_STREAM;
		break;

	default:
		break;
	}

	return PX4_OK;
}

int SF45LaserSerial::collect()
{
	float distance_m = -1.0f;

	if (_sensor_state == STATE_UNINIT) {

		perf_begin(_sample_perf);
		const int payload_length = 22;

		_crc_valid = false;
		sf45_get_and_handle_request(payload_length, SF_PRODUCT_NAME);

		if (_crc_valid) {
			_sensor_state = STATE_ACK_PRODUCT_NAME;
			perf_end(_sample_perf);
			return PX4_OK;
		}

		return -EAGAIN;

	} else if (_sensor_state == STATE_ACK_PRODUCT_NAME) {

		perf_begin(_sample_perf);
		const int payload_length = 7;

		_crc_valid = false;
		sf45_get_and_handle_request(payload_length, SF_UPDATE_RATE);

		if (_crc_valid) {
			_sensor_state = STATE_ACK_UPDATE_RATE;
			perf_end(_sample_perf);
			return PX4_OK;
		}

		return -EAGAIN;

	} else if (_sensor_state == STATE_ACK_UPDATE_RATE) {

		perf_begin(_sample_perf);
		const int payload_length = 10;

		_crc_valid = false;
		sf45_get_and_handle_request(payload_length, SF_DISTANCE_OUTPUT);

		if (_crc_valid) {
			_sensor_state = STATE_ACK_DISTANCE_OUTPUT;
			perf_end(_sample_perf);
			return PX4_OK;
		}

		return -EAGAIN;

	} else {

		// Stream data from sensor
		perf_begin(_sample_perf);
		const int payload_length = 10;

		_crc_valid = false;
		sf45_get_and_handle_request(payload_length, SF_DISTANCE_DATA_CM);

		if (_crc_valid) {
			sf45_process_replies(&distance_m);
			PX4_DEBUG("val (float): %8.4f, valid: %s", (double)distance_m, ((_crc_valid) ? "OK" : "NO"));
			perf_end(_sample_perf);
			return PX4_OK;
		}

		return -EAGAIN;
	}
}

void SF45LaserSerial::start()
{
	/* reset the sensor state */
	_sensor_state = STATE_UNINIT;

	/* reset the report ring */
	_collect_phase = false;

	/* reset the UART receive buffer size */
	_linebuf_size = 0;

	/* reset the fail counter */
	_last_received_time = hrt_absolute_time();

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
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) {
			PX4_ERR("serial open failed (%i)", errno);
			return;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* no parity, one stop bit */
		uart_config.c_cflag &= ~(CSTOPB | PARENB);

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
		if (hrt_absolute_time() - _last_received_time >= 1_s) {
			start();
			return;
		}

		/* perform collection */
		if (collect() != PX4_OK && errno != EAGAIN) {
			PX4_DEBUG("collect error");
		}

		if (_sensor_state != STATE_SEND_STREAM) {
			/* next phase is measurement */
			_collect_phase = false;
		}

	} else {
		/* measurement phase */

		if (measure() != PX4_OK) {
			PX4_DEBUG("measure error");
		}

		/* next phase is collection */
		_collect_phase = true;
	}

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(_interval);
}

void SF45LaserSerial::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void SF45LaserSerial::sf45_get_and_handle_request(const int payload_length, const SF_SERIAL_CMD msg_id)
{
	// SF45 protocol
	// Start byte is 0xAA and is the start of packet
	// Payload length sanity check (0-1023) bytes
	// and represented by 16-bit integer (payload +
	// read/write status)
	// ID byte precedes the data in the payload
	// CRC comprised of 16-bit checksum (not included in checksum calc.)

	int ret;
	size_t max_read = sizeof(_linebuf) - _linebuf_size;
	ret = ::read(_fd, &_linebuf[_linebuf_size], max_read);

	if (ret < 0 && errno != EAGAIN) {
		PX4_ERR("ERROR (ack from streaming distance data): %d", ret);
		_linebuf_size = 0;
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return;
	}

	if (ret > 0) {
		_last_received_time = hrt_absolute_time();
		_linebuf_size += ret;
	}

	// Not enough data to parse a complete packet. Gather more data in the next cycle.
	if (_linebuf_size < payload_length) {
		return;
	}

	int index = 0;

	while (index <= _linebuf_size - payload_length && _crc_valid == false) {
		bool restart_flag = false;

		while (restart_flag != true) {
			switch (_parsed_state) {
			case 0: {
					if (_linebuf[index] == 0xAA) {
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
						PX4_DEBUG("Start of packet not valid: %d", _sensor_state);
						break;
					} // end else
				} // end case 0

			case 1: {
					rx_field.flags_lo = _linebuf[index + 1];
					_calc_crc = sf45_format_crc(_calc_crc, rx_field.flags_lo);
					_parsed_state = 2;
					break;
				}

			case 2: {
					rx_field.flags_hi = _linebuf[index + 2];
					rx_field.data_len = (rx_field.flags_hi << 2) | (rx_field.flags_lo >> 6);
					_calc_crc = sf45_format_crc(_calc_crc, rx_field.flags_hi);

					// Check payload length against known max value
					if (rx_field.data_len > 17) {
						_parsed_state = 0;
						restart_flag = true;
						_calc_crc = 0;
						PX4_DEBUG("Payload length error: %d", _sensor_state);
						break;

					} else {
						_parsed_state = 3;
						break;
					}
				}

			case 3: {
					rx_field.msg_id = _linebuf[index + 3];

					if (rx_field.msg_id == msg_id) {
						_calc_crc = sf45_format_crc(_calc_crc, rx_field.msg_id);
						_parsed_state = 4;
						break;
					}

					// Ignore message ID's that aren't searched
					else {
						_parsed_state = 0;
						_calc_crc = 0;
						restart_flag = true;
						PX4_DEBUG("Non needed message ID: %d", _sensor_state);
						break;
					}
				}

			// Data
			case 4: {
					// Process commands with  & w/out data bytes
					if (rx_field.data_len > 1) {
						for (uint8_t i = 4; i < 3 + rx_field.data_len; ++i) {

							rx_field.data[_data_bytes_recv] = _linebuf[index + i];
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
					rx_field.crc[0] = _linebuf[index + 3 + rx_field.data_len];
					_parsed_state = 6;
					break;
				}

			// CRC high byte
			case 6: {
					rx_field.crc[1] = _linebuf[index + 4 + rx_field.data_len];
					uint16_t recv_crc = (rx_field.crc[1] << 8) | rx_field.crc[0];

					// Check the received crc bytes from the sf45 against our own CRC calcuation
					// If it matches, we can check if sensor ready
					// Only if crc match is valid and sensor ready (transmitting distance data) do we flag _init_complete
					if (recv_crc == _calc_crc) {
						_crc_valid = true;
						_parsed_state = 0;
						_calc_crc = 0;
						restart_flag = true;
						break;

					} else {

						_crc_valid = false;
						_parsed_state = 0;
						_calc_crc = 0;
						restart_flag = true;
						perf_count(_comms_errors);
						PX4_DEBUG("CRC mismatch: %d", _sensor_state);
						break;
					}
				}
			} // end switch
		} //end while

		index++;
	}

	// If we parsed successfully, remove the parsed part from the buffer if it is still large enough
	if (_crc_valid && index + payload_length < _linebuf_size) {
		unsigned next_after_index = index + payload_length;
		memmove(&_linebuf[0], &_linebuf[next_after_index], _linebuf_size - next_after_index);
		_linebuf_size -= next_after_index;
	}

	// The buffer is filled. Either we can't keep up with the stream and/or it contains only invalid data. Reset to try again.
	if ((unsigned)_linebuf_size >= sizeof(_linebuf)) {
		_linebuf_size = 0;
		perf_count(_comms_errors);
	}
}

void SF45LaserSerial::sf45_send(uint8_t msg_id, bool write, int32_t *data, uint8_t data_len)
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
		packet_buff[data_inc] = data_convert;
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
		PX4_DEBUG("DEBUG: Product name");
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
		PX4_DEBUG("DEBUG: Send byte: %d", packet_buff[i]);
	}

	ret = ::write(_fd, packet_buff, packet_len);

	if (ret != packet_len) {
		perf_count(_comms_errors);
		PX4_ERR("serial write fail %d", ret);
		// Flush data written, not transmitted
		tcflush(_fd, TCOFLUSH);
	}
}

void SF45LaserSerial::sf45_process_replies(float *distance_m)
{
	switch (rx_field.msg_id) {
	case SF_DISTANCE_DATA_CM: {
			const float raw_distance = (rx_field.data[0] << 0) | (rx_field.data[1] << 8);
			int16_t raw_yaw = ((rx_field.data[2] << 0) | (rx_field.data[3] << 8));
			int16_t scaled_yaw = 0;

			// The sensor scans from 0 to -160, so extract negative angle from int16 and represent as if a float
			if (raw_yaw > 32000) {
				raw_yaw = raw_yaw - 65535;
			}

			// The sensor is facing downward, so the sensor is flipped about it's x-axis -inverse of each yaw angle
			if (_orient_cfg == ROTATION_DOWNWARD_FACING) {
				raw_yaw = raw_yaw * -1;
			}

			// SF45/B product guide {Data output bit: 8 Description: "Yaw angle [1/100 deg] size: int16}"
			scaled_yaw = raw_yaw * SF45_SCALE_FACTOR;

			switch (_yaw_cfg) {
			case ROTATION_FORWARD_FACING:
				break;

			case ROTATION_BACKWARD_FACING:
				if (scaled_yaw > 180) {
					scaled_yaw = scaled_yaw - 180;

				} else {
					scaled_yaw = scaled_yaw + 180; // rotation facing aft
				}

				break;

			case ROTATION_RIGHT_FACING:
				scaled_yaw = scaled_yaw + 90; // rotation facing right
				break;

			case ROTATION_LEFT_FACING:
				scaled_yaw = scaled_yaw - 90; // rotation facing left
				break;

			default:
				break;
			}

			// Convert to meters for the debug message
			*distance_m = raw_distance * SF45_SCALE_FACTOR;
			_current_bin_dist = ((uint16_t)raw_distance < _current_bin_dist) ? (uint16_t)raw_distance : _current_bin_dist;

			uint8_t current_bin = sf45_convert_angle(scaled_yaw);

			if (current_bin != _previous_bin) {
				PX4_DEBUG("scaled_yaw: \t %d, \t current_bin: \t %d, \t distance: \t %8.4f\n", scaled_yaw, current_bin,
					  (double)*distance_m);

				if (_current_bin_dist > _obstacle_map_msg.max_distance) {
					_current_bin_dist = _obstacle_map_msg.max_distance + 1; // As per ObstacleDistance.msg definition
				}

				// if the sensor has its cycle delay configured for a low value like 5, it can happen that not every bin gets a measurement.
				// in this case we assume the measurement to be valid for all bins between the previous and the current bin. win
				uint8_t start;
				uint8_t end;

				if (abs(current_bin - _previous_bin) > BIN_COUNT /
				    4) {	// wrap-around case is assumed to have happend when the distance between the bins is larger than 1/4 of all Bins
					// TODO: differentiate direction of wrap-around, currently it overwrites a previous measurement.
					start = math::max(_previous_bin, current_bin);
					end = math::min(_previous_bin, current_bin);

				} else if (_previous_bin < current_bin) {	// Scanning clockwise
					start = _previous_bin + 1;
					end = current_bin;

				} else { 					// scanning counter-clockwise
					start = current_bin;
					end = _previous_bin - 1;
				}

				if (start <= end) {
					for (uint8_t i = start; i <= end; i++) {_obstacle_map_msg.distances[i] = _current_bin_dist;}

				} else { // wrap-around case
					for (uint8_t i = start; i < BIN_COUNT; i++) {_obstacle_map_msg.distances[i] = _current_bin_dist;}

					for (uint8_t i = 0; i <= end; i++) {_obstacle_map_msg.distances[i] = _current_bin_dist;}
				}

				_obstacle_map_msg.timestamp = hrt_absolute_time();
				_obstacle_distance_pub.publish(_obstacle_map_msg);

				// reset the values for the next measurement
				if (start <= end) {
					for (uint8_t i = start; i <= end; i++) {_obstacle_map_msg.distances[i] = UINT16_MAX;}

				} else { // wrap-around case
					for (uint8_t i = start; i < BIN_COUNT; i++) {_obstacle_map_msg.distances[i] = UINT16_MAX;}

					for (uint8_t i = 0; i <= end; i++) {_obstacle_map_msg.distances[i] = UINT16_MAX;}
				}

				_current_bin_dist = UINT16_MAX;
				_previous_bin = current_bin;
			}

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
