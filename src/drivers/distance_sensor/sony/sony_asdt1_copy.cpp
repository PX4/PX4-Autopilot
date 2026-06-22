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
 * Minimal AS-DT1 serial write probe.
 */

#include "sony_asdt1_copy.hpp"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <px4_platform_common/defines.h>

AS_DT1_COPY::AS_DT1_COPY(const char *device, bool one_shot) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device)),
	_one_shot(one_shot)
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
}

int AS_DT1_COPY::init()
{
	if (_one_shot) {
		if (open_port() != PX4_OK) {
			return PX4_ERROR;
		}

		if (write_start_command() != PX4_OK) {
			close_port();
			return PX4_ERROR;
		}

		px4_usleep(1000000);
		(void)read_once();

	} else {
		start();
	}

	return PX4_OK;
}

void AS_DT1_COPY::print_info()
{
	PX4_INFO("AS-DT1 copy on %s, baud %u, mode %s", _device, _baud, _one_shot ? "one-shot" : "scheduled");
	PX4_INFO("fd: %d, last command: %s, command bytes: %u, last write: %lld",
		 _fd, _last_command, static_cast<unsigned>(_last_command_len), static_cast<long long>(_last_write));
	PX4_INFO("read: attempts %llu, total %llu bytes, no data %llu, errors %llu, last read %lld, last read age %llu us",
		 static_cast<unsigned long long>(_read_attempts),
		 static_cast<unsigned long long>(_bytes_read_total),
		 static_cast<unsigned long long>(_no_data_reads),
		 static_cast<unsigned long long>(_read_errors),
		 static_cast<long long>(_last_read_size),
		 static_cast<unsigned long long>(_last_read == 0 ? 0 : hrt_elapsed_time(&_last_read)));
	PX4_INFO("parser: state %u, begin match %u, frame buffer %u bytes, frames rx %llu, frames pub %llu, resets %llu",
		 static_cast<unsigned>(_parser_state),
		 static_cast<unsigned>(_begin_match_index),
		 static_cast<unsigned>(_frame_buffer_len),
		 static_cast<unsigned long long>(_frames_rx),
		 static_cast<unsigned long long>(_frames_pub),
		 static_cast<unsigned long long>(_parser_resets));
	PX4_INFO("last processed: frame %u bytes, samples %u, valid bins %u, closest %u cm",
		 static_cast<unsigned>(_last_frame_processed_len),
		 static_cast<unsigned>(_last_sample_count),
		 static_cast<unsigned>(_last_valid_bins),
		 static_cast<unsigned>(_last_closest_distance));

	if (_last_read_bytes_len == 0) {
		PX4_INFO("last read bytes: empty");

	} else {
		for (size_t offset = 0; offset < _last_read_bytes_len; offset += 16) {
			const size_t chunk_len = math::min(static_cast<size_t>(16), _last_read_bytes_len - offset);
			char hex[16 * 3]{};
			char ascii[17]{};
			size_t hex_pos = 0;

			for (size_t i = 0; i < chunk_len; i++) {
				const uint8_t byte = _last_read_bytes[offset + i];
				hex_pos += snprintf(&hex[hex_pos], sizeof(hex) - hex_pos, "%02x%s", byte, (i + 1 < chunk_len) ? " " : "");
				ascii[i] = (byte >= 32 && byte <= 126) ? static_cast<char>(byte) : '.';
			}

			PX4_INFO("last read[%u..%u] hex: %s ascii: %s",
				 static_cast<unsigned>(offset),
				 static_cast<unsigned>(offset + chunk_len - 1),
				 hex,
				 ascii);
		}
	}
}

void AS_DT1_COPY::start()
{
	ScheduleNow();
}

void AS_DT1_COPY::stop()
{
	ScheduleClear();
	close_port();
}

void AS_DT1_COPY::Run()
{
	if (_fd < 0) {
		if (open_port() != PX4_OK) {
			return;
		}

		if (write_start_command() != PX4_OK) {
			close_port();
			return;
		}

		ScheduleDelayed(_interval);
		return;
	}

	(void)read_once();
	ScheduleDelayed(_interval);
}

int AS_DT1_COPY::open_port()
{
	_fd = ::open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	struct termios uart_config {};

	if (tcgetattr(_fd, &uart_config) < 0) {
		PX4_ERR("tcgetattr failed (%i)", errno);
		close_port();
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
		PX4_ERR("tcsetattr failed (%i)", errno);
		close_port();
		return PX4_ERROR;
	}

	(void)tcflush(_fd, TCIOFLUSH);
	return PX4_OK;
}

void AS_DT1_COPY::close_port()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

int AS_DT1_COPY::write_start_command()
{
	if (write_command_padded("format binz") != PX4_OK) {
		return PX4_ERROR;
	}
	tcdrain(_fd);
	px4_usleep(200000);

	if (write_command_padded("fsync 200") != PX4_OK) {
		return PX4_ERROR;
	}
	tcdrain(_fd);

	return PX4_OK;
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
	_last_write = written;
	_last_command_len = length;
	strncpy(_last_command, command, sizeof(_last_command) - 1);
	_last_command[sizeof(_last_command) - 1] = '\0';

	if (written != static_cast<ssize_t>(length)) {
		PX4_ERR("write failed (%i)", errno);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int AS_DT1_COPY::read_once()
{
	_read_attempts++;

	uint8_t buffer[READ_BUFFER_SIZE]{};
	const ssize_t bytes_read = ::read(_fd, buffer, sizeof(buffer));

	if (bytes_read <= 0) {
		if (bytes_read == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
			_no_data_reads++;
			return PX4_OK;
		}

		_read_errors++;
		return PX4_ERROR;
	}

	_last_read = hrt_absolute_time();
	_last_read_size = bytes_read;
	_bytes_read_total += static_cast<uint64_t>(bytes_read);
	_last_read_bytes_len = math::min(static_cast<size_t>(bytes_read), LAST_READ_CAPTURE_SIZE);
	memcpy(_last_read_bytes, buffer, _last_read_bytes_len);

	for (ssize_t i = 0; i < bytes_read; i++) {
		parse_byte(buffer[i]);
	}

	return PX4_OK;
}

bool AS_DT1_COPY::parse_byte(uint8_t byte)
{
	constexpr char begin_marker[] = "BEGIN MP\r\n";
	constexpr size_t begin_marker_len = sizeof(begin_marker) - 1;

	const auto reset_parser = [this]() {
		_parser_state = ParserState::FindBegin;
		_frame_buffer_len = 0;
		_begin_match_index = 0;
	};

	switch (_parser_state) {
	case ParserState::FindBegin:
		if (byte == static_cast<uint8_t>(begin_marker[_begin_match_index])) {
			_begin_match_index++;

			if (_begin_match_index == begin_marker_len) {
				_parser_state = ParserState::ReadPayload;
				_frame_buffer_len = 0;
				_begin_match_index = 0;
			}

		} else {
			_begin_match_index = (byte == static_cast<uint8_t>(begin_marker[0])) ? 1 : 0;
		}

		break;

	case ParserState::ReadPayload:
		if (_frame_buffer_len >= ASDT1_BINZ_FRAME_SIZE) {
			_parser_resets++;
			reset_parser();
			_begin_match_index = (byte == static_cast<uint8_t>(begin_marker[0])) ? 1 : 0;
			break;
		}

		_frame_buffer[_frame_buffer_len++] = byte;

		if (_frame_buffer_len == ASDT1_BINZ_FRAME_SIZE) {
			_frames_rx++;
			process_frame(_frame_buffer, _frame_buffer_len);
			reset_parser();
			return true;
		}

		break;
	}

	return false;
}

int AS_DT1_COPY::process_frame(const uint8_t *frame, size_t length)
{
	if (frame == nullptr || length != ASDT1_BINZ_FRAME_SIZE) {
		PX4_ERR("invalid AS-DT1 binz frame length: %u", static_cast<unsigned>(length));
		_parser_resets++;
		return PX4_ERROR;
	}

	_last_frame_processed_len = length;
	_last_sample_count = ASDT1_MAX_SAMPLE_COUNT;
	_last_valid_bins = 0;
	_last_closest_distance = UINT16_MAX;

	for (uint8_t i = 0; i < BIN_COUNT; i++) {
		_obstacle_distance.distances[i] = UINT16_MAX;
	}

	for (size_t sample = 0; sample < ASDT1_MAX_SAMPLE_COUNT; sample++) {
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
	_frames_pub++;

	return PX4_OK;
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

uint16_t AS_DT1_COPY::z_raw_to_distance_cm(uint32_t z_raw)
{
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
