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
 * @file sony_asdt1.cpp
 * @author Andrew Brahim <brahim@ascendengineer.com>
 * @author Apoorv Thapliyal
 *
 * Driver for the Sony ASDT1 lidar
 */

#include "sony_asdt1.hpp"
#include <px4_platform_common/defines.h>
#include <lib/parameters/param.h>
#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>
#include <cmath>
#include <cstdio>
#include <cstring>

AS_DT1::AS_DT1(const char *device) : ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
{
	if (device != nullptr) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}

	_obstacle_distance.timestamp = hrt_absolute_time();
	_obstacle_distance.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_distance.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_LASER;
	_obstacle_distance.increment = static_cast<uint8_t>(OBSTACLE_INCREMENT_DEG);
	_obstacle_distance.min_distance = 30;
	_obstacle_distance.max_distance = 2000;
	_obstacle_distance.angle_offset = 0.0f;

	for (uint32_t i = 0 ; i < BIN_COUNT; i++) { // Initialize all distances to max (no obstacle)
		_obstacle_distance.distances[i] = UINT16_MAX;
	}
}
AS_DT1::~AS_DT1()
{

	// if (_uart.isOpen()) {
	// 	_uart.close();
	// }

	stop();
	perf_free(_comms_errors);
	perf_free(_sample_perf);
	perf_free(_bytes_rx);
	perf_free(_no_data);
	perf_free(_frames_rx);
	perf_free(_frames_pub);
	perf_free(_parser_resets);
	perf_free(_backlog_flushes);

}
int AS_DT1::init()
{
	return start();
}


// TODO:
void AS_DT1::print_info()
{
	PX4_INFO("AS-DT1 on %s, baud %u", _device, _baud);
	PX4_INFO("parser state: %u, begin match: %u, end match: %u, frame buffer: %u bytes, latest frame: %u bytes",
		 static_cast<unsigned>(_parser_state),
		 static_cast<unsigned>(_begin_match_index),
		 static_cast<unsigned>(_end_match_index),
		 static_cast<unsigned>(_frame_buffer_len),
		 static_cast<unsigned>(_latest_frame_len));
	PX4_INFO("rx: total %llu bytes, last available %lld, last read %lld, last read age %llu us",
		 static_cast<unsigned long long>(_bytes_rx_total),
		 static_cast<long long>(_last_bytes_available),
		 static_cast<long long>(_last_bytes_read),
		 static_cast<unsigned long long>(_last_read == 0 ? 0 : hrt_elapsed_time(&_last_read)));
	PX4_INFO("last processed: frame %u bytes, samples %u, valid bins %u, closest %u cm",
		 static_cast<unsigned>(_last_frame_processed_len),
		 static_cast<unsigned>(_last_sample_count),
		 static_cast<unsigned>(_last_valid_bins),
		 static_cast<unsigned>(_last_closest_distance));
	PX4_INFO("rx markers: B %llu, E %llu, CR %llu, LF %llu, prompt %llu",
		 static_cast<unsigned long long>(_rx_b_count),
		 static_cast<unsigned long long>(_rx_e_count),
		 static_cast<unsigned long long>(_rx_cr_count),
		 static_cast<unsigned long long>(_rx_lf_count),
		 static_cast<unsigned long long>(_rx_prompt_count));
	PX4_INFO("payload END: confirmed %u bytes, probe %u bytes, rejected candidate %u bytes",
		 static_cast<unsigned>(_last_end_payload_len),
		 static_cast<unsigned>(_last_end_probe_payload_len),
		 static_cast<unsigned>(_last_rejected_candidate_len));
	PX4_INFO("payload sizes: 720 %llu, 1440 %llu, unexpected END %llu, candidate miss %llu",
		 static_cast<unsigned long long>(_payload_len_720_count),
		 static_cast<unsigned long long>(_payload_len_1440_count),
		 static_cast<unsigned long long>(_payload_len_unexpected_count),
		 static_cast<unsigned long long>(_end_mismatch_count));

	print_debug_buffer("first rx", _first_rx, _first_rx_len);

	uint8_t last_rx_ordered[DEBUG_RX_CAPTURE_SIZE] {};
	const size_t last_rx_len = math::min(_last_rx_len, DEBUG_RX_CAPTURE_SIZE);

	for (size_t i = 0; i < last_rx_len; i++) {
		const size_t index = (_last_rx_len < DEBUG_RX_CAPTURE_SIZE) ? i : ((_last_rx_pos + i) % DEBUG_RX_CAPTURE_SIZE);
		last_rx_ordered[i] = _last_rx[index];
	}

	print_debug_buffer("last rx", last_rx_ordered, last_rx_len);
	print_debug_buffer("last B probe", _last_begin_probe, _last_begin_probe_len);
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
	perf_print_counter(_bytes_rx);
	perf_print_counter(_no_data);
	perf_print_counter(_frames_rx);
	perf_print_counter(_frames_pub);
	perf_print_counter(_parser_resets);
	perf_print_counter(_backlog_flushes);
}
int AS_DT1::start()
{
	if (!_uart.setPort(_device)) {
		PX4_ERR("failed to set serial port %s", _device);
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	if (!_uart.setBaudrate(_baud)) {
		PX4_ERR("failed to set UART %lu baud", static_cast<unsigned long>(_baud));
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	(void)_uart.setBytesize(ByteSize::EightBits);
	(void)_uart.setParity(Parity::None);
	(void)_uart.setStopbits(StopBits::One);
	(void)_uart.setFlowcontrol(FlowControl::Disabled);

	if (!_uart.isOpen() && !_uart.open()) {
		PX4_ERR("failed to open AS-DT1 on %s", _device);
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	// _uart.flush();
	_first_rx_len = 0;
	_last_rx_len = 0;
	_last_rx_pos = 0;
	_last_begin_probe_len = 0;
	_last_begin_probe_remaining = 0;
	_payload_end_probe_match_index = 0;
	_rx_b_count = 0;
	_rx_e_count = 0;
	_rx_cr_count = 0;
	_rx_lf_count = 0;
	_rx_prompt_count = 0;
	_last_end_payload_len = 0;
	_last_end_probe_payload_len = 0;
	_last_rejected_candidate_len = 0;
	_payload_len_720_count = 0;
	_payload_len_1440_count = 0;
	_payload_len_unexpected_count = 0;
	_end_mismatch_count = 0;

	// Start polling before command setup so command echoes and immediate streams
	// from a simulator or real sensor cannot be missed during startup.

	// Minimal command probe: send exactly one command and log whatever comes back.
	// if (writeCommandPadded("flshow") != PX4_OK) {
	// 	PX4_ERR("failed to request AS-DT1 flash settings");
	// 	perf_count(_comms_errors);
	// 	return PX4_ERROR;
	// }

	// readAndLogCommandResponse("flshow response", 2_s);
	// return PX4_OK;

	// readAndLogCommandResponse("fsync 0 response", 500_ms);
	// _uart.flush();

	// if (writeCommandPadded("flshow") != PX4_OK) {
	// 	PX4_ERR("failed to request AS-DT1 flash settings");
	// 	perf_count(_comms_errors);
	// 	return PX4_ERROR;
	// }

	// px4_usleep(250_ms);


	// readAndLogCommandResponse("flshow response", 2_s);
	// return PX4_OK;

	// Normal measurement startup:
	//
	// if (writeCommandPadded("fsync 0") != PX4_OK) {
	// 	PX4_ERR("failed to disable AS-DT1 self trigger");
	// 	perf_count(_comms_errors);
	// 	return PX4_ERROR;
	// }

	// px4_usleep(250_ms);
	// _uart.flush();

	if (writeCommandPadded("format binz") != PX4_OK) {
		PX4_ERR("failed to configure AS-DT1 binz output");
		perf_count(_comms_errors);
		ScheduleClear();
		return PX4_ERROR;
	}

	if (writeCommandPadded("fsync 200") != PX4_OK) {
		PX4_ERR("failed to start AS-DT1 frame stream");
		perf_count(_comms_errors);
		ScheduleClear();
		return PX4_ERROR;
	}

	return PX4_OK;


}

void AS_DT1::Run()
{
	if (!_uart.isOpen()) {
		PX4_ERR("serial port is not open");
		perf_count(_comms_errors);
		ScheduleClear();
		return;
	}

	collect();
}

void
AS_DT1::stop()
{
	ScheduleClear();

	if (_uart.isOpen()) {
		// Leave the minimal command probe quiet on stop. If the port is already invalid,
		// writing here can hide the useful result from the actual probe.
		// (void)writeCommandPadded("fsync 0");
		_uart.close();
	}
}

int AS_DT1::writeCommand(const uint8_t *data, size_t length)
{
	//TODO:

	// If write fails, print error and return false
	if (_uart.write(data, length) != static_cast<ssize_t>(length)) {
		PX4_ERR("Failed to write to AS-DT1");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int AS_DT1::writeCommandPadded(const char *command)
{
	if (command == nullptr) {
		return PX4_ERROR;
	}

	char buffer[32] {};
	const size_t command_len = strnlen(command, sizeof(buffer) - 1);

	if (command_len == 0 || command_len >= sizeof(buffer) - 1) {
		return PX4_ERROR;
	}

	memcpy(buffer, command, command_len);
	size_t length = command_len + 1; // include trailing CR
	buffer[command_len] = '\r';

	const size_t remainder = length % 16;

	if (remainder != 0) {
		const size_t padded_length = length + (16 - remainder);

		if (padded_length > sizeof(buffer)) {
			return PX4_ERROR;
		}

		memmove(&buffer[padded_length - 1], &buffer[length - 1], 1);
		memset(&buffer[length - 1], ' ', padded_length - length);
		length = padded_length;
	}

	const ssize_t written = _uart.writeBlocking(reinterpret_cast<const uint8_t *>(buffer), length, 100);

	if (written != static_cast<ssize_t>(length)) {
		PX4_ERR("failed to write AS-DT1 command");
		return PX4_ERROR;
	}

	return PX4_OK;
}

void AS_DT1::readAndLogCommandResponse(const char *label, hrt_abstime timeout)
{
	const hrt_abstime start = hrt_absolute_time();
	uint8_t buffer[96] {};
	size_t total_read = 0;

	while (hrt_elapsed_time(&start) < timeout) {
		const ssize_t bytes_available = _uart.bytesAvailable();

		if (bytes_available <= 0) {
			px4_usleep(10_ms);
			continue;
		}

		const size_t bytes_to_read = math::min(static_cast<size_t>(bytes_available), sizeof(buffer) - 1);
		const ssize_t bytes_read = _uart.read(buffer, bytes_to_read);

		if (bytes_read <= 0) {
			px4_usleep(10_ms);
			continue;
		}

		total_read += static_cast<size_t>(bytes_read);

		char text[sizeof(buffer)] {};

		for (ssize_t i = 0; i < bytes_read; i++) {
			const uint8_t byte = buffer[i];
			text[i] = (byte >= 32 && byte <= 126) ? static_cast<char>(byte) : '.';
		}

		PX4_INFO("%s: %s", label, text);
	}

	if (total_read == 0) {
		PX4_WARN("%s: no response", label);
	}
}


int AS_DT1::collect()
{
	if (!_uart.isOpen()) {
		return PX4_ERROR;
	}

	perf_begin(_sample_perf);

	uint8_t read_buffer[READ_BUFFER_SIZE] {};
	bool received_data = false;
	bool backlog_flushed = false;

	for (uint8_t read_count = 0; read_count < MAX_READS_PER_COLLECT; read_count++) {
		ssize_t bytes_available = _uart.bytesAvailable();
		_last_bytes_available = bytes_available;

		if (bytes_available < 0) {
			PX4_ERR("failed checking UART bytes available");
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return PX4_ERROR;
		}

		if (static_cast<size_t>(bytes_available) > ASDT1_MAX_BACKLOG) {
			PX4_WARN("AS-DT1 UART backlog, flushing");
			_uart.flush();
			_parser_state = ParserState::FindBegin;
			_frame_buffer_len = 0;
			_begin_match_index = 0;
			_end_match_index = 0;
			_candidate_frame_len = 0;
			_payload_end_probe_match_index = 0;
			backlog_flushed = true;
			perf_count(_backlog_flushes);
			break;
		}

		const size_t bytes_to_read = (bytes_available > 0) ?
					     math::min(static_cast<size_t>(bytes_available), sizeof(read_buffer)) :
					     sizeof(read_buffer);
		const ssize_t bytes_read = _uart.read(read_buffer, bytes_to_read);

		if (bytes_read < 0) {
			if (bytes_available == 0) {
				break;
			}

			PX4_ERR("UART read failed");
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return PX4_ERROR;
		}

		if (bytes_read == 0) {
			_last_bytes_read = 0;
			break;
		}

		received_data = true;
		_last_read = hrt_absolute_time();
		_last_bytes_read = bytes_read;
		_bytes_rx_total += static_cast<uint64_t>(bytes_read);
		perf_set_count(_bytes_rx, _bytes_rx_total);

		for (ssize_t i = 0; i < bytes_read; i++) {
			capture_debug_byte(read_buffer[i]);
			parse_byte(read_buffer[i]);
		}
	}


	if (_have_latest_frame) {
		process_frame(_latest_frame, _latest_frame_len);
		_have_latest_frame = false;
	}

	perf_end(_sample_perf);

	if (!received_data && !backlog_flushed) {
		perf_count(_no_data);
	}

	return (received_data || backlog_flushed) ? PX4_OK : -EAGAIN;
}

void AS_DT1::capture_debug_byte(uint8_t byte)
{
	if (_first_rx_len < DEBUG_RX_CAPTURE_SIZE) {
		_first_rx[_first_rx_len++] = byte;
	}

	_last_rx[_last_rx_pos] = byte;
	_last_rx_pos = (_last_rx_pos + 1) % DEBUG_RX_CAPTURE_SIZE;

	if (_last_rx_len < DEBUG_RX_CAPTURE_SIZE) {
		_last_rx_len++;
	}

	if (byte == 'B') {
		_rx_b_count++;
		_last_begin_probe_len = 0;
		_last_begin_probe_remaining = DEBUG_RX_CAPTURE_SIZE;

	} else if (byte == 'E') {
		_rx_e_count++;
	}

	if (byte == '\r') {
		_rx_cr_count++;

	} else if (byte == '\n') {
		_rx_lf_count++;

	} else if (byte == '>') {
		_rx_prompt_count++;
	}

	if (_last_begin_probe_remaining > 0) {
		_last_begin_probe[_last_begin_probe_len++] = byte;
		_last_begin_probe_remaining--;
	}
}

void AS_DT1::print_debug_buffer(const char *label, const uint8_t *buffer, size_t length)
{
	if (length == 0) {
		PX4_INFO("%s: empty", label);
		return;
	}

	for (size_t offset = 0; offset < length; offset += 16) {
		const size_t chunk_len = math::min(static_cast<size_t>(16), length - offset);
		char ascii[17] {};

		for (size_t i = 0; i < chunk_len; i++) {
			const uint8_t byte = buffer[offset + i];
			ascii[i] = (byte >= 32 && byte <= 126) ? static_cast<char>(byte) : '.';
		}

		PX4_INFO("%s[%u..%u] ascii: %s",
			 label,
			 static_cast<unsigned>(offset),
			 static_cast<unsigned>(offset + chunk_len - 1),
			 ascii);
	}
}

bool AS_DT1::parse_byte(uint8_t byte)
{
	constexpr char begin_marker[] = "BEGIN MP";
	constexpr size_t begin_marker_len = sizeof(begin_marker) - 1;
	constexpr char end_marker[] = "END";
	constexpr size_t end_marker_len = sizeof(end_marker) - 1;

	const auto reset_parser = [this]() {
		_parser_state = ParserState::FindBegin;
		_frame_buffer_len = 0;
		_begin_match_index = 0;
		_end_match_index = 0;
		_candidate_frame_len = 0;
		_payload_end_probe_match_index = 0;
	};

	const auto complete_frame = [this, &reset_parser](size_t length) {
		memcpy(_latest_frame, _frame_buffer, length);
		_latest_frame_len = length;
		_have_latest_frame = true;
		_last_end_payload_len = length;

		if (length == ASDT1_BINZ_SHORT_FRAME_SIZE) {
			_payload_len_720_count++;

		} else if (length == ASDT1_BINZ_FRAME_SIZE) {
			_payload_len_1440_count++;
		}

		perf_count(_frames_rx);
		reset_parser();
		return true;
	};

	const auto arm_end_check_if_candidate = [this]() {
		if (_frame_buffer_len == ASDT1_BINZ_SHORT_FRAME_SIZE || _frame_buffer_len == ASDT1_BINZ_FRAME_SIZE) {
			_parser_state = ParserState::FindEnd;
			_candidate_frame_len = _frame_buffer_len;
			_end_match_index = 0;
			_payload_end_probe_match_index = 0;
		}
	};

	const auto probe_unexpected_end = [this, &end_marker, end_marker_len, &reset_parser](uint8_t value) {
		if (value == static_cast<uint8_t>(end_marker[_payload_end_probe_match_index])) {
			_payload_end_probe_match_index++;

			if (_payload_end_probe_match_index == end_marker_len) {
				_last_end_probe_payload_len = (_frame_buffer_len >= end_marker_len) ?
							      _frame_buffer_len - end_marker_len : 0;
				_payload_len_unexpected_count++;
				perf_count(_parser_resets);
				reset_parser();
				return true;
			}

		} else {
			_payload_end_probe_match_index = (value == static_cast<uint8_t>(end_marker[0])) ? 1 : 0;
		}

		return false;
	};

	switch (_parser_state) {
	case ParserState::FindBegin:
		if (byte == static_cast<uint8_t>(begin_marker[_begin_match_index])) {
			_begin_match_index++;

			if (_begin_match_index == begin_marker_len) {
				_parser_state = ParserState::ReadPayload;
				_frame_buffer_len = 0;
				_begin_match_index = 0;
				_end_match_index = 0;
				_candidate_frame_len = 0;
				_payload_end_probe_match_index = 0;
			}

		} else {
			_begin_match_index = (byte == static_cast<uint8_t>(begin_marker[0])) ? 1 : 0;
		}

		break;

	case ParserState::ReadPayload:
		if (_frame_buffer_len == 0 && (byte == '\r' || byte == '\n')) {
			break;
		}

		if (_frame_buffer_len >= ASDT1_BINZ_FRAME_SIZE) {
			perf_count(_parser_resets);
			reset_parser();
			_begin_match_index = (byte == static_cast<uint8_t>(begin_marker[0])) ? 1 : 0;
			break;
		}

		_frame_buffer[_frame_buffer_len++] = byte;

		arm_end_check_if_candidate();

		if (_parser_state != ParserState::FindEnd && probe_unexpected_end(byte)) {
			break;
		}

		break;

	case ParserState::FindEnd:
		if (byte == static_cast<uint8_t>(end_marker[_end_match_index])) {
			_end_match_index++;

			if (_end_match_index == end_marker_len) {
				return complete_frame(_candidate_frame_len);
			}

		} else {
			_end_mismatch_count++;
			_last_rejected_candidate_len = _candidate_frame_len;

			if (_candidate_frame_len >= ASDT1_BINZ_FRAME_SIZE) {
				perf_count(_parser_resets);
				reset_parser();
				_begin_match_index = (byte == static_cast<uint8_t>(begin_marker[0])) ? 1 : 0;
				break;
			}

			for (size_t i = 0; i < _end_match_index && _frame_buffer_len < ASDT1_BINZ_FRAME_SIZE; i++) {
				_frame_buffer[_frame_buffer_len++] = static_cast<uint8_t>(end_marker[i]);
			}

			if (_frame_buffer_len < ASDT1_BINZ_FRAME_SIZE) {
				_frame_buffer[_frame_buffer_len++] = byte;
			}

			_parser_state = ParserState::ReadPayload;
			_end_match_index = 0;
			_candidate_frame_len = 0;

			arm_end_check_if_candidate();

			if (_parser_state != ParserState::FindEnd && probe_unexpected_end(byte)) {
				break;
			}
		}

		break;
	}

	return false;
}

int AS_DT1::process_frame(const uint8_t *frame, size_t length)
{
	if (frame == nullptr || (length != ASDT1_BINZ_SHORT_FRAME_SIZE && length != ASDT1_BINZ_FRAME_SIZE)) {
		PX4_ERR("invalid AS-DT1 binz frame length: %u", static_cast<unsigned>(length));
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	const size_t sample_count = (length == ASDT1_BINZ_SHORT_FRAME_SIZE) ? ASDT1_SHORT_SAMPLE_COUNT : ASDT1_MAX_SAMPLE_COUNT;
	_last_frame_processed_len = length;
	_last_sample_count = sample_count;
	_last_valid_bins = 0;
	_last_closest_distance = UINT16_MAX;

	for (uint32_t i = 0; i < BIN_COUNT; i++) {
		_obstacle_distance.distances[i] = UINT16_MAX;
	}

	for (size_t sample = 0; sample < sample_count; sample++) {
		const int layout_index = sample_to_layout_index(sample);
		int row = 0;
		int col = 0;

		if (!layout_index_to_row_col(layout_index, row, col)) {
			continue;
		}

		if (row < MIN_USED_ROW || row > MAX_USED_ROW) {
			continue;
		}

		const int bin = col_to_obstacle_bin(col);

		if (bin < 0 || bin >= BIN_COUNT) {
			continue;
		}

		const uint32_t z_raw = decode_20bit_raw(frame, sample);

		if (z_raw == 0) {
			continue;
		}

		const uint16_t distance_cm = z_raw_to_distance_cm(z_raw);
		const uint16_t obstacle_distance_cm = (distance_cm > _obstacle_distance.max_distance) ?
						     _obstacle_distance.max_distance + 1 : distance_cm;

		if (obstacle_distance_cm < _obstacle_distance.distances[bin]) {
			if (_obstacle_distance.distances[bin] == UINT16_MAX) {
				_last_valid_bins++;
			}

			_obstacle_distance.distances[bin] = obstacle_distance_cm;
			_last_closest_distance = math::min(_last_closest_distance, obstacle_distance_cm);
		}
	}

	_obstacle_distance.timestamp = hrt_absolute_time();
	_obstacle_distance_pub.publish(_obstacle_distance);
	perf_count(_frames_pub);

	return PX4_OK;
}

int32_t AS_DT1::decode_20bit_signed(uint32_t raw)
{
	raw &= 0x000fffff;

	if (raw & 0x00080000) {
		return -static_cast<int32_t>(0x00100000 - raw);
	}

	return static_cast<int32_t>(raw);
}

uint32_t AS_DT1::decode_20bit_raw(const uint8_t *data, size_t sample_index)
{
	const size_t offset = (sample_index / 2) * 5;

	if ((sample_index % 2) == 0) {
		return (static_cast<uint32_t>(data[offset]) << 12)
		       | (static_cast<uint32_t>(data[offset + 1]) << 4)
		       | (static_cast<uint32_t>(data[offset + 2]) >> 4);
	}

	return ((static_cast<uint32_t>(data[offset + 2]) & 0x0f) << 16)
	       | (static_cast<uint32_t>(data[offset + 3]) << 8)
	       | static_cast<uint32_t>(data[offset + 4]);
}

uint16_t AS_DT1::z_raw_to_distance_cm(uint32_t z_raw)
{
	const float z_mm = static_cast<float>(z_raw) / 4.0f;
	const float z_cm = z_mm / 10.0f;

	if (!PX4_ISFINITE(z_cm)) {
		return UINT16_MAX;
	}

	return static_cast<uint16_t>(math::min(roundf(z_cm), static_cast<float>(UINT16_MAX)));
}

int AS_DT1::sample_to_layout_index(size_t sample_index)
{
	static constexpr int MPDATATBL[ASDT1_MAX_SAMPLE_COUNT] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
		48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71,
		288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311,
		336, 337, 338, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359,
		96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
		144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167,
		384, 385, 386, 387, 388, 389, 390, 391, 392, 393, 394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405, 406, 407,
		432, 433, 434, 435, 436, 437, 438, 439, 440, 441, 442, 443, 444, 445, 446, 447, 448, 449, 450, 451, 452, 453, 454, 455,
		192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215,
		240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263,
		480, 481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 495, 496, 497, 498, 499, 500, 501, 502, 503,
		528, 529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539, 540, 541, 542, 543, 544, 545, 546, 547, 548, 549, 550, 551,
		24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
		72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95,
		312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335,
		360, 361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 381, 382, 383,
		120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143,
		168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191,
		408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419, 420, 421, 422, 423, 424, 425, 426, 427, 428, 429, 430, 431,
		456, 457, 458, 459, 460, 461, 462, 463, 464, 465, 466, 467, 468, 469, 470, 471, 472, 473, 474, 475, 476, 477, 478, 479,
		216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
		264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287,
		504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 514, 515, 516, 517, 518, 519, 520, 521, 522, 523, 524, 525, 526, 527,
		552, 553, 554, 555, 556, 557, 558, 559, 560, 561, 562, 563, 564, 565, 566, 567, 568, 569, 570, 571, 572, 573, 574, 575
	};

	if (sample_index >= ASDT1_MAX_SAMPLE_COUNT) {
		return -1;
	}

	return MPDATATBL[sample_index];
}

bool AS_DT1::layout_index_to_row_col(int layout_index, int &row, int &col)
{
	if (layout_index < 0 || layout_index >= static_cast<int>(ASDT1_MAX_SAMPLE_COUNT)) {
		return false;
	}

	row = layout_index / ASDT1_COLS;
	col = layout_index % ASDT1_COLS;
	return true;
}

int AS_DT1::col_to_obstacle_bin(int col)
{
	if (col < 0 || col >= ASDT1_COLS) {
		return -1;
	}

	const float col_fraction = static_cast<float>(col) / static_cast<float>(ASDT1_COLS - 1);
	const float angle_deg = LEFT_EDGE_DEG + col_fraction * HORIZONTAL_FOV_DEG;
	int bin = static_cast<int>(roundf(angle_deg / OBSTACLE_INCREMENT_DEG));

	while (bin < 0) {
		bin += BIN_COUNT;
	}

	return bin % BIN_COUNT;
}


// void AS_DT1::readThreadFunction()
// {
// 	while (reading) {
// 		char buf[50000];
// 		int n = SERIAL_.read(buf, sizeof(buf));
//
// 		if (n > 0) {
// 			read_buffer.append(buf, n);
// 			// std::cout.write(buf, n);
// 			// std::cout.flush();
//
// 			// If "END" found, clear buffer and send command
// 			if (read_buffer.find("END") != std::string::npos) {
// 				// writeCommand("");
//
// 				// Store the latest read data
// 				latest_read_data = read_buffer;
//
// 				// Clear the buffer for the next read
// 				read_buffer.clear();
// 			}
// 		}
//
// 		// std::this_thread::sleep_for(std::chrono::milliseconds(1));
// 	}
// }




// bool AS_DT1::convertBinaryToPCD()
// {
// 	// Thread-safe copy of latest read data
// 	std::string binary_pcd_data;
// 	{
// 		std::lock_guard<std::mutex> lock(buffer_mutex);
// 		binary_pcd_data = latest_read_data;
// 	}

// 	std::cout << "Latest read data length: " << binary_pcd_data.size() << "\n";
// 	// std::cout << "Latest read data : \n" << binary_pcd_data << "\n";

// 	// If the data is empty, print error and return false
// 	if (binary_pcd_data.empty()) {
// 		std::cerr << "No binary data to convert to PCD\n";
// 		return false;
// 	}

// 	// Find "BEGIN MP" and "END"
// 	size_t begin_pos = binary_pcd_data.find("BEGIN MP");
// 	size_t end_pos = binary_pcd_data.find("END");

// 	// Print the string before BEGIN MP
// 	// std::cout << "Data before BEGIN MP: \n" << binary_pcd_data.substr(0, begin_pos) << "\n";

// 	std::cout << "BEGIN MP position: " << begin_pos << "\n";
// 	std::cout << "END position: " << end_pos << "\n";
// 	// std::cout << "Data between BEGIN MP and END: \n" << binary_pcd_data.substr(begin_pos+8, end_pos-begin_pos-8) << "\n";

// 	// If either "BEGIN MP" or "END" is not found, print error and return false
// 	if (begin_pos == std::string::npos || end_pos == std::string::npos || end_pos <= begin_pos) {
// 		std::cerr << "Invalid binary data format: missing BEGIN MP or END\n";
// 		return false;
// 	}

// 	// Extract the binary data between "BEGIN MP" and "END"
// 	size_t data_start = begin_pos + std::string("BEGIN MP\r\n").length();
// 	size_t data_length = end_pos - data_start;
// 	std::cout << "Binary data length: " << data_length << "\n";
// 	const uint8_t *data_ptr = (const uint8_t *)binary_pcd_data.c_str() + data_start;

// 	// Read length is set to 4320
// 	size_t read_length = 4320;

// 	//TODO:
// 	// Temporary points, fill it with 0s first, then we will fill it with the extracted points
// 	// matrix::Vector3f pcd_raw(2);
// 	// pcd_raw[0].reserve(576);
// 	// pcd_raw[1].reserve(576);

// 	// // Fill temporary points with 0s
// 	// for (int i = 0; i < 576; ++i) {
// 	//     pcd_raw[0].emplace_back(Point3D{0, 0, 0});
// 	//     pcd_raw[1].emplace_back(Point3D{0, 0, 0});
// 	// }

// 	// int count = 0;
// 	for (size_t dr_cnt = 0; dr_cnt + 15 <= 4320; dr_cnt += 15) {
// 		try {
// 			// Extract 20-bit values from 15 bytes
// 			// XXXXX YYYYY ZZZZZ XXXXX YYYYY ZZZZZ
// 			// XX XX XY YY YY ZZ ZZ ZX XX XX YY YY YZ ZZ ZZ
// 			//  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14
// 			int32_t x1 = (data_ptr[dr_cnt + 0] << 12) | (data_ptr[dr_cnt + 1] << 4) | (data_ptr[dr_cnt + 2] >> 4);
// 			int32_t y1 = ((data_ptr[dr_cnt + 2] & 0x0f) << 16) | (data_ptr[dr_cnt + 3] << 8) | data_ptr[dr_cnt + 4];
// 			int32_t z1 = (data_ptr[dr_cnt + 5] << 12) | (data_ptr[dr_cnt + 6] << 4) | (data_ptr[dr_cnt + 7] >> 4);
// 			int32_t x2 = ((data_ptr[dr_cnt + 7] & 0x0f) << 16) | (data_ptr[dr_cnt + 8] << 8) | data_ptr[dr_cnt + 9];
// 			int32_t y2 = (data_ptr[dr_cnt + 10] << 12) | (data_ptr[dr_cnt + 11] << 4) | (data_ptr[dr_cnt + 12] >> 4);
// 			int32_t z2 = ((data_ptr[dr_cnt + 12] & 0x0f) << 16) | (data_ptr[dr_cnt + 13] << 8) | data_ptr[dr_cnt + 14];

// 			// Convert from 20-bit two's complement to signed
// 			if (x1 & 0x80000) { x1 = -(0x100000 - x1); }

// 			if (y1 & 0x80000) { y1 = -(0x100000 - y1); }

// 			if (x2 & 0x80000) { x2 = -(0x100000 - x2); }

// 			if (y2 & 0x80000) { y2 = -(0x100000 - y2); }

// 			// Store pcd_raw[0][count] and pcd_raw[1][count]
// 			Eigen::Vector3f p0, p1;
// 			p0 << x1 / 4.0f, y1 / 4.0f, z1 / 4.0f;
// 			p1 << x2 / 4.0f, y2 / 4.0f, z2 / 4.0f;

// 			pcd_raw[0].push_back(p0);
// 			pcd_raw[1].push_back(p1);
// 			// count++;

// 		} catch (...) {
// 			std::cerr << "Error processing data\n";
// 		}
// 	}

// 	// Make the PCD from the temp points
// 	std::vector<std::vector<Eigen::Vector3f>> pcd(2);
// 	pcd[0].resize(576);
// 	pcd[1].resize(576);

// 	float thrMin = range_min;
// 	float thrMax = range_max;

// 	// Iterate over 576 points
// 	for (int cnt = 0; cnt < 576; cnt++) {
// 		// tx order to picture order
// 		int mp = xyDataTbl[cnt];

// 		// pcd[1][cnt] = pcd_raw[1][mp];
// 		Eigen::Vector3f x1_y1_z1 = pcd_raw[1][mp];
// 		pcd[1][cnt] = x1_y1_z1;

// 		// Range filtering
// 		Eigen::Vector3f x_y_z = pcd_raw[0][mp];
// 		float z = x_y_z(2);
// 		float z1 = x1_y1_z1(2);

// 		if (std::abs(z) < thrMin || std::abs(z) > thrMax) {
// 			if (std::abs(z1) < thrMin || std::abs(z1) > thrMax) {
// 				// ignore both z and z1
// 				pcd[0][cnt] = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

// 			} else {
// 				// z is invalid, z1 is valid
// 				pcd[0][cnt] = x1_y1_z1;
// 			}

// 		} else {
// 			// z is valid
// 			pcd[0][cnt] = x_y_z;
// 		}
// 	}

// 	// Step 3: Convert to meters (divide by 1000)
// 	for (int i = 0; i < 2; i++) {
// 		for (auto &p : pcd[i]) {
// 			p(0) /= 1000.0f;
// 			p(1) /= 1000.0f;
// 			p(2) /= 1000.0f;
// 		}
// 	}

// 	// Rotate -90 degrees around y-axis
// 	for (int i = 0; i < 2; i++) {
// 		for (auto &p : pcd[i]) {
// 			float x_new = p(2);
// 			float z_new = -p(0);
// 			p(0) = x_new;
// 			p(2) = z_new;
// 		}
// 	}

// 	std::cout << "Successfully converted " << pcd[0].size() << " points\n";

// 	// Print all the points
// 	for (size_t i = 0; i < pcd[0].size(); ++i) {
// 		const Eigen::Vector3f &p = pcd[0][i];
// 		std::cout << "Point " << i << ": (" << p(0) << ", " << p(1) << ", " << p(2) << ")\n";
// 	}

// 	// Update points
// 	// {
// 	//     std::lock_guard<std::mutex> lock(buffer_mutex);
// 	//     points = pcd;
// 	// }

// 	return true;
// }
