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
 * @file sony_asdt1_copy.cpp
 *
 * Experimental AS-DT1 binz/index-based obstacle_distance driver copy.
 */

#include "sony_asdt1_copy.hpp"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

AS_DT1_COPY::AS_DT1_COPY(const char *device) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
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

	for (uint8_t i = 0; i < BIN_COUNT; i++) {
		_obstacle_distance.distances[i] = UINT16_MAX;
	}
}

AS_DT1_COPY::~AS_DT1_COPY()
{
	stop();
	perf_free(_comms_errors);
	perf_free(_sample_perf);
	perf_free(_bytes_rx);
}

int AS_DT1_COPY::init()
{
	return start();
}

void AS_DT1_COPY::print_info()
{
	PX4_INFO("AS-DT1 copy on %s, baud %u", _device, _baud);
	PX4_INFO("fd: %d, bytes_rx: %llu, last_read: %lld, last_read_age: %llu us",
		 _fd,
		 static_cast<unsigned long long>(_bytes_rx_total),
		 static_cast<long long>(_last_bytes_read),
		 static_cast<unsigned long long>(_last_read == 0 ? 0 : hrt_elapsed_time(&_last_read)));

	if (_first_rx_len == 0) {
		PX4_INFO("first rx: empty");

	} else {
		char ascii[DEBUG_RX_CAPTURE_SIZE + 1]{};

		for (size_t i = 0; i < _first_rx_len; i++) {
			ascii[i] = (_first_rx[i] >= 32 && _first_rx[i] <= 126) ? static_cast<char>(_first_rx[i]) : '.';
		}

		PX4_INFO("first rx: %s", ascii);
	}

	if (_last_rx_len == 0) {
		PX4_INFO("last rx: empty");

	} else {
		char ascii[DEBUG_RX_CAPTURE_SIZE + 1]{};

		for (size_t i = 0; i < _last_rx_len; i++) {
			const size_t index = (_last_rx_len < DEBUG_RX_CAPTURE_SIZE) ? i : ((_last_rx_pos + i) % DEBUG_RX_CAPTURE_SIZE);
			const uint8_t byte = _last_rx[index];
			ascii[i] = (byte >= 32 && byte <= 126) ? static_cast<char>(byte) : '.';
		}

		PX4_INFO("last rx: %s", ascii);
	}

	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
	perf_print_counter(_bytes_rx);
}

int AS_DT1_COPY::start()
{
	_fd = ::open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("failed to open AS-DT1 copy on %s", _device);
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	struct termios uart_config {};

	if (tcgetattr(_fd, &uart_config) < 0) {
		PX4_ERR("failed to get UART config");
		perf_count(_comms_errors);
		::close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS | CSIZE);
	uart_config.c_cflag |= (CLOCAL | CREAD | CS8);
	uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);
	uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	uart_config.c_oflag &= ~ONLCR;
	uart_config.c_cc[VMIN] = 0;
	uart_config.c_cc[VTIME] = 0;

	speed_t speed = B115200;

	if (_baud == 921600) {
		speed = B921600;
	}

	(void)cfsetispeed(&uart_config, speed);
	(void)cfsetospeed(&uart_config, speed);

	if (tcsetattr(_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("failed to set UART config");
		perf_count(_comms_errors);
		::close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	(void)tcflush(_fd, TCIOFLUSH);

	_first_rx_len = 0;
	_last_rx_len = 0;
	_last_rx_pos = 0;
	_bytes_rx_total = 0;
	_last_bytes_read = 0;

	if (write_command_padded("format binz") != PX4_OK) {
		PX4_ERR("failed to configure AS-DT1 binz output");
		perf_count(_comms_errors);
		::close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	if (write_command_padded("fsync 200") != PX4_OK) {
		PX4_ERR("failed to configure AS-DT1 frame trigger");
		perf_count(_comms_errors);
		::close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	ScheduleOnInterval(7_ms);
	return PX4_OK;
}

void AS_DT1_COPY::stop()
{
	ScheduleClear();

	if (_fd >= 0) {
		(void)write_command_padded("fsync 0");
		::close(_fd);
		_fd = -1;
	}
}

void AS_DT1_COPY::Run()
{
	if (_fd < 0) {
		PX4_ERR("serial port is not open");
		perf_count(_comms_errors);
		ScheduleClear();
		return;
	}

	collect();
}

int AS_DT1_COPY::write_command_padded(const char *command)
{
	if (_fd < 0) {
		return PX4_ERROR;
	}

	if (command == nullptr) {
		return PX4_ERROR;
	}

	char buffer[32]{};
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

	const ssize_t written = ::write(_fd, buffer, length);

	if (written != static_cast<ssize_t>(length)) {
		PX4_ERR("failed to write AS-DT1 command");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int AS_DT1_COPY::collect()
{
	if (_fd < 0) {
		return PX4_ERROR;
	}

	perf_begin(_sample_perf);

	uint8_t read_buffer[READ_BUFFER_SIZE]{};
	bool received_data = false;
	for (uint8_t read_count = 0; read_count < MAX_READS_PER_COLLECT; read_count++) {
		const ssize_t bytes_read = ::read(_fd, read_buffer, sizeof(read_buffer));

		if (bytes_read < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				break;
			}

			PX4_ERR("UART read failed");
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return PX4_ERROR;
		}

		if (bytes_read == 0) {
			break;
		}

		received_data = true;
		_last_read = hrt_absolute_time();
		_last_bytes_read = bytes_read;
		_bytes_rx_total += static_cast<uint64_t>(bytes_read);
		perf_set_count(_bytes_rx, _bytes_rx_total);

		for (ssize_t i = 0; i < bytes_read; i++) {
			const uint8_t byte = read_buffer[i];

			if (_first_rx_len < DEBUG_RX_CAPTURE_SIZE) {
				_first_rx[_first_rx_len++] = byte;
			}

			_last_rx[_last_rx_pos] = byte;
			_last_rx_pos = (_last_rx_pos + 1) % DEBUG_RX_CAPTURE_SIZE;

			if (_last_rx_len < DEBUG_RX_CAPTURE_SIZE) {
				_last_rx_len++;
			}
		}
	}

	perf_end(_sample_perf);
	return received_data ? PX4_OK : -EAGAIN;
}

bool AS_DT1_COPY::parse_byte(uint8_t byte)
{
	constexpr char begin_marker[] = "BEGIN MP\r\n";
	constexpr size_t begin_marker_len = sizeof(begin_marker) - 1;
	constexpr char end_marker[] = "END";
	constexpr size_t end_marker_len = sizeof(end_marker) - 1;

	switch (_parser_state) {
	case ParserState::FindBegin:
		if (byte == static_cast<uint8_t>(begin_marker[_begin_match_index])) {
			_begin_match_index++;

			if (_begin_match_index == begin_marker_len) {
				_parser_state = ParserState::ReadPayload;
				_frame_buffer_len = 0;
				_begin_match_index = 0;
				_candidate_frame_len = 0;
			}

		} else {
			_begin_match_index = (byte == static_cast<uint8_t>(begin_marker[0])) ? 1 : 0;
		}

		break;

	case ParserState::ReadPayload:
		_frame_buffer[_frame_buffer_len++] = byte;

		if (_frame_buffer_len == ASDT1_BINZ_SHORT_FRAME_SIZE || _frame_buffer_len == ASDT1_BINZ_FRAME_SIZE) {
			_parser_state = ParserState::FindEnd;
			_end_match_index = 0;
			_candidate_frame_len = _frame_buffer_len;
		}

		break;

	case ParserState::FindEnd:
		if (byte == static_cast<uint8_t>(end_marker[_end_match_index])) {
			_end_match_index++;

			if (_end_match_index == end_marker_len) {
				memcpy(_latest_frame, _frame_buffer, _candidate_frame_len);
				_latest_frame_len = _candidate_frame_len;
				_have_latest_frame = true;

				_parser_state = ParserState::FindBegin;
				_frame_buffer_len = 0;
				_begin_match_index = 0;
				_end_match_index = 0;
				_candidate_frame_len = 0;
				return true;
			}

		} else {
			if (_candidate_frame_len == ASDT1_BINZ_SHORT_FRAME_SIZE && _frame_buffer_len < ASDT1_BINZ_FRAME_SIZE) {
				for (size_t i = 0; i < _end_match_index && _frame_buffer_len < ASDT1_BINZ_FRAME_SIZE; i++) {
					_frame_buffer[_frame_buffer_len++] = static_cast<uint8_t>(end_marker[i]);
				}

				if (_frame_buffer_len < ASDT1_BINZ_FRAME_SIZE) {
					_frame_buffer[_frame_buffer_len++] = byte;
				}

				_parser_state = ParserState::ReadPayload;
				_end_match_index = 0;
				_candidate_frame_len = 0;

				if (_frame_buffer_len == ASDT1_BINZ_FRAME_SIZE) {
					_parser_state = ParserState::FindEnd;
					_candidate_frame_len = _frame_buffer_len;
				}

			} else {
				_parser_state = ParserState::FindBegin;
				_frame_buffer_len = 0;
				_begin_match_index = (byte == static_cast<uint8_t>(begin_marker[0])) ? 1 : 0;
				_end_match_index = 0;
				_candidate_frame_len = 0;
			}
		}

		break;
	}

	return false;
}

int AS_DT1_COPY::process_frame(const uint8_t *frame, size_t length)
{
	if (frame == nullptr || (length != ASDT1_BINZ_SHORT_FRAME_SIZE && length != ASDT1_BINZ_FRAME_SIZE)) {
		PX4_ERR("invalid AS-DT1 binz frame length: %u", static_cast<unsigned>(length));
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	const size_t sample_count = (length == ASDT1_BINZ_SHORT_FRAME_SIZE) ? ASDT1_SHORT_SAMPLE_COUNT : ASDT1_MAX_SAMPLE_COUNT;

	for (uint8_t i = 0; i < BIN_COUNT; i++) {
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

		const int32_t z_raw = decode_20bit_signed(decode_20bit_raw(frame, sample));

		if (z_raw == 0) {
			continue;
		}

		const uint16_t distance_cm = z_raw_to_distance_cm(z_raw);

		const uint16_t obstacle_distance_cm = (distance_cm > _obstacle_distance.max_distance) ?
						     _obstacle_distance.max_distance + 1 : distance_cm;

		if (obstacle_distance_cm < _obstacle_distance.distances[bin]) {
			_obstacle_distance.distances[bin] = obstacle_distance_cm;
		}
	}

	_obstacle_distance.timestamp = hrt_absolute_time();
	_obstacle_distance_pub.publish(_obstacle_distance);

	return PX4_OK;
}

int32_t AS_DT1_COPY::decode_20bit_signed(uint32_t raw)
{
	raw &= 0x000fffff;

	if (raw & 0x00080000) {
		return -static_cast<int32_t>(0x00100000 - raw);
	}

	return static_cast<int32_t>(raw);
}

uint32_t AS_DT1_COPY::decode_20bit_raw(const uint8_t *data, size_t sample_index)
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

uint16_t AS_DT1_COPY::z_raw_to_distance_cm(int32_t z_raw)
{
	if (z_raw < 0) {
		z_raw = -z_raw;
	}

	const float z_mm = static_cast<float>(z_raw) / 4.0f;
	const float z_cm = z_mm / 10.0f;

	if (!PX4_ISFINITE(z_cm)) {
		return UINT16_MAX;
	}

	return static_cast<uint16_t>(math::min(roundf(z_cm), static_cast<float>(UINT16_MAX)));
}

int AS_DT1_COPY::sample_to_layout_index(size_t sample_index)
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

bool AS_DT1_COPY::layout_index_to_row_col(int layout_index, int &row, int &col)
{
	if (layout_index < 0 || layout_index >= static_cast<int>(ASDT1_MAX_SAMPLE_COUNT)) {
		return false;
	}

	row = layout_index / ASDT1_COLS;
	col = layout_index % ASDT1_COLS;
	return true;
}

int AS_DT1_COPY::col_to_obstacle_bin(int col)
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
