/****************************************************************************
 *
 *   Copyright (C) 2017-2019 Intel Corporation. All rights reserved.
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

#include "MR72.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

MR72::MR72(const char *serial_port, uint8_t device_orientation):
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port)),
	_px4_rangefinder(0 /* device id not yet used */, ORB_PRIO_DEFAULT, device_orientation)
{
	_serial_port = strdup(serial_port);

	_px4_rangefinder.set_max_distance(ULANDING_MAX_DISTANCE);
	_px4_rangefinder.set_min_distance(ULANDING_MIN_DISTANCE);
	_px4_rangefinder.set_fov(0.0488692f);

	_last_value = 0;
	_valid_orign_distance = 0;
	_keep_valid_time = 0;
	_mf_cycle_counter = 0;
	_var_cycle_couter = 0;



}

MR72::~MR72()
{
	stop();

	free((char *)_serial_port);
	perf_free(_comms_error);
	perf_free(_sample_perf);
}


int
MR72::collect()
{
	perf_begin(_sample_perf);

	read_uart_data(_file_descriptor, &_uartbuf);
	parse_uart_data(&_uartbuf, &_packet);

	uint16_t packet_message_id;
	packet_message_id = _packet.message_id2 * 256 + _packet.message_id1;
	public_distance_sensor(packet_message_id);

	perf_end(_sample_perf);
	return true;
}

int
MR72::init()
{
	if (open_serial_port() != PX4_OK) {
		return PX4_ERROR;
	}


	return PX4_OK;
}


int
MR72::open_serial_port(const speed_t speed)
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
	uart_config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// Set the input baud rate in the uart_config struct.
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







int
MR72::read_uart_data(int uart_fd, UART_BUF *const uart_buf)
{
	uint8_t tmp_serial_buf[UART_BUFFER_SIZE];

	int len =::read(uart_fd, tmp_serial_buf, sizeof(tmp_serial_buf));

	if (len > 0 && (uart_buf->dat_cnt + len < UART_BUFFER_SIZE)) {
		for (int i = 0; i < len; i++) {
			uart_buf->tx_rx_uart_buf[uart_buf->tail++] = tmp_serial_buf[i];
			uart_buf->dat_cnt++;

			if (uart_buf->tail >= UART_BUFFER_SIZE) {
				uart_buf->tail = 0;
			}
		}

	} else if (len < 0) {
		PX4_DEBUG("error reading radar");
		return len;
	}

	return 0;
}


int
MR72::parse_uart_data(UART_BUF *const serial_buf, Packet *const packetdata)
{
	static PARSR_mr72_STATE state = HEAD1;
	static uint8_t data_index = 0;
	uint16_t packet_tmp_data;

	if (serial_buf->dat_cnt > 0) {
		int count = serial_buf->dat_cnt;

		for (int i = 0; i < count; i++) {
			switch (state) {
			case HEAD1:

				packet_tmp_data = serial_buf->tx_rx_uart_buf[serial_buf->head];

				if (packet_tmp_data == NAR15_HEAD1) {
					packetdata->start_sequence1 = NAR15_HEAD1; //just_keep the format
					state = HEAD2;
				}

				break;

			case HEAD2:

				packet_tmp_data = serial_buf->tx_rx_uart_buf[serial_buf->head];

				if (packet_tmp_data == NAR15_HEAD2) {
					packetdata->start_sequence2 = NAR15_HEAD2;
					state = ID1;

				} else {
					state = HEAD1;
				}

				break;

			case ID1:

				packet_tmp_data = serial_buf->tx_rx_uart_buf[serial_buf->head];

				if (packet_tmp_data == SENSOR_STATUS_L_8_BIT || packet_tmp_data == TARGET_STATUS_L_8_BIT
				    || packet_tmp_data == TARGET_INFO_L_8_BIT) {
					packetdata->message_id1 = packet_tmp_data;
					state = ID2;

				} else {
					state = HEAD1;
				}

				break;

			case ID2:

				packet_tmp_data = serial_buf->tx_rx_uart_buf[serial_buf->head];

				packetdata->message_id2 = packet_tmp_data;
				data_index = 0;
				state = DATA;

				break;

			case DATA:
				packetdata->payload.bytes[data_index++] = serial_buf->tx_rx_uart_buf[serial_buf->head];

				if (data_index >= DATA_PAYLOAD_NUM) {
					state = END1;
				}

				break;

			case END1:

				packet_tmp_data = serial_buf->tx_rx_uart_buf[serial_buf->head];

				if (packet_tmp_data == NAR15_END1) {
					packetdata->stop_sequence1 = NAR15_END1; //just_keep the format
					state = END2;

				}

				break;

			case END2:
				// packet_tmp_data = ((uint16_t)serial_buf->tx_rx_uart_buf[serial_buf->head + 1] << 8);
				packet_tmp_data = serial_buf->tx_rx_uart_buf[serial_buf->head];

				if (packet_tmp_data == NAR15_END2) {
					packetdata->stop_sequence2 = NAR15_END2; //just_keep the format
					state = HEAD1;

					if (++serial_buf->head >= UART_BUFFER_SIZE) {
						serial_buf->head = 0;
					}

					serial_buf->dat_cnt--;
					return 0;
				}

				state = HEAD1;
				break;

			default:
				state = HEAD1;
				break;

			}

			if (++serial_buf->head >= UART_BUFFER_SIZE) {
				serial_buf->head = 0;
			}

			serial_buf->dat_cnt--;
		}
	}

	return -1;
}



float
MR72::_calc_variance(float value)
{
	float accum = 0.f;
	float variance_sum = 0.f;

	/* this is an unusual special case handling,
	 * because ultrasound does not receive echoes in many cases, so we ignore the data
	 */
	if ((((value - _last_value) > 0.5f) ||  value >= ULANDING_MAX_DISTANCE || value <= ULANDING_MIN_DISTANCE)
	    && (hrt_elapsed_time(&_keep_valid_time) < RANDAR_DATA_ABNORMAL_TIMEOUT)) { // add abnormal timeout handle
		value = _last_value;
		_valid_orign_distance = _last_value;

	} else {
		_valid_orign_distance = value;
		_keep_valid_time = hrt_absolute_time();
	}

	_last_value = value;

	_variance_window[(_var_cycle_couter + 1) % _VAR_WINDOW_SIZE] = value;

	for (int i = 0; i < _VAR_WINDOW_SIZE; i++) {
		variance_sum += _variance_window[i];
	}

	_var_cycle_couter++;

	float average = variance_sum / _VAR_WINDOW_SIZE;

	for (int i = 0; i < _VAR_WINDOW_SIZE; i++) {
		accum  += (_variance_window[i] - average) * (_variance_window[i] - average);
	}

	return (accum / (_VAR_WINDOW_SIZE - 1));
}

int static cmp(const void *a, const void *b)
{
	return (*(const float *)a > *(const float *)b);
}


float
MR72::_median_filter(float value)
{
	/* TODO: replace with ring buffer */
	_mf_window[(_mf_cycle_counter + 1) % _MF_WINDOW_SIZE] = value;

	for (int i = 0; i < _MF_WINDOW_SIZE; ++i) {
		_mf_window_sorted[i] = _mf_window[i];
	}

	qsort(_mf_window_sorted, _MF_WINDOW_SIZE, sizeof(float), cmp);

	_mf_cycle_counter++;

	return _mf_window_sorted[_MF_WINDOW_SIZE / 2];
}




bool
MR72::public_distance_sensor(uint16_t message_id)
{
	if (message_id == TARGET_INFO) {
				float range = 0;
				uint16_t tmp;

				TargetInfo &feed_back_data = _packet.payload.runinfo;
				tmp  = (uint16_t)(feed_back_data.RangeH << 8);
				tmp |= (uint16_t)(feed_back_data.RangeL);
				range = tmp * 0.01;
				float variance_after_filter;
				float range_value_after_filter;
				variance_after_filter = _calc_variance(range);
				range_value_after_filter = _median_filter(_valid_orign_distance);
				range_value_after_filter = range_value_after_filter > ULANDING_MAX_DISTANCE ? ULANDING_MAX_DISTANCE :
							  range_value_after_filter;
				range_value_after_filter = range_value_after_filter < ULANDING_MIN_DISTANCE ? ULANDING_MIN_DISTANCE :
							  range_value_after_filter;
							  _px4_rangefinder.set_covariance(variance_after_filter);
				const hrt_abstime timestamp_sample = hrt_absolute_time();
				_px4_rangefinder.update(timestamp_sample, range_value_after_filter);
				return true;


	}

		return false;
}


void
MR72::print_info()
{
	perf_print_counter(_comms_error);
	perf_print_counter(_sample_perf);
}

void
MR72::Run()
{

	// Ensure the serial port is open.
	open_serial_port();

	collect();
}

void
MR72::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(MR72_MEASURE_INTERVAL, MR72_MEASURE_INTERVAL);
}

void
MR72::stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);
	_file_descriptor = -1;

	// Clear the work queue schedule.
	ScheduleClear();
}
