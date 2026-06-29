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
 *
 * Sony AS-DT1 serial rangefinder driver.
 */

#include "sony_asdt1.hpp"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <lib/parameters/param.h>
#include <px4_platform_common/defines.h>

namespace
{
// Sony shell commands used during setup. They are padded before write.
constexpr const char *CMD_FSYNC_STOP = "fsync 0";          // Stop periodic frames.
constexpr const char *CMD_FORMAT_BINZ = "format binz";     // Select binary MP output.
constexpr const char *CMD_FSYNC_START = "fsync 30";        // Stream at the minimum interval.
constexpr const char *CMD_FLUART_921600 = "fluart 921600"; // Saved and applied after reboot.
constexpr const char *CMD_REBOOT = "reboot";               // Apply saved mode and UART settings.
constexpr const char *CMD_FLSHOW = "flshow";               // Print saved sensor configuration.
}

AS_DT1::AS_DT1(const char *device, bool flshow_only, float yaw_offset_deg) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device)),
	_flshow_only(flshow_only)
{
	if (device != nullptr) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}

	_obstacle_distance.timestamp = hrt_absolute_time();
	_obstacle_distance.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_distance.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_LASER;
	_obstacle_distance.increment = static_cast<uint8_t>(OBSTACLE_INCREMENT_DEG);
	_obstacle_distance.min_distance = MIN_VALID_DISTANCE_CM;

	(void)param_get(param_find("SENS_ASDT1_MODE"), &_mode);
	_obstacle_distance.max_distance = max_distance_for_mode(_mode);
	_obstacle_distance.angle_offset = yaw_offset_deg;

	for (uint8_t i = 0; i < BIN_COUNT; i++) {
		_obstacle_distance.distances[i] = UINT16_MAX;
	}
}

AS_DT1::~AS_DT1()
{
	stop();
}

int AS_DT1::init()
{
	if (_flshow_only) {
		if (open_port() != PX4_OK) {
			return PX4_ERROR;
		}

		(void)drain_input();
		(void)write_prompt_sync();
		tcdrain(_fd);
		px4_usleep(PROMPT_SYNC_SETTLE_INTERVAL);
		(void)drain_input();

		if (write_command_padded(CMD_FSYNC_STOP) != PX4_OK) {
			close_port();
			return PX4_ERROR;
		}

		tcdrain(_fd);
		px4_usleep(PROMPT_SYNC_SETTLE_INTERVAL);
		(void)drain_input();

		bool flshow_complete = false;

		for (uint8_t attempt = 0; attempt < FLSHOW_RETRY_LIMIT; attempt++) {
			if (write_command_padded(CMD_FLSHOW) != PX4_OK) {
				close_port();
				return PX4_ERROR;
			}

			tcdrain(_fd);
			px4_usleep(1000000);
			const int response = read_and_print_response(CMD_FLSHOW);

			if (response < 0) {
				close_port();
				return PX4_ERROR;
			}

			if (response > 0) {
				flshow_complete = true;
				break;
			}
		}

		if (!flshow_complete) {
			PX4_WARN("flshow: no complete response after %u tries", static_cast<unsigned>(FLSHOW_RETRY_LIMIT));
		}

	} else {
		start();
	}

	return PX4_OK;
}

void AS_DT1::print_info()
{
	const char *mode = _flshow_only ? "flshow" : "scheduled";
	PX4_INFO("AS-DT1 on %s, baud %u, mode %s, yaw offset %.1f deg",
		 _device, _baud, mode, static_cast<double>(_obstacle_distance.angle_offset));
	PX4_INFO("fd: %d, last command: %s, command bytes: %u, last write: %lld",
		 _fd, _last_command, static_cast<unsigned>(_last_command_len), static_cast<long long>(_last_write));
	PX4_INFO("read: attempts %llu, total %llu bytes, no data %llu, errors %llu, last read %lld, last read age %llu us",
		 static_cast<unsigned long long>(_read_attempts),
		 static_cast<unsigned long long>(_bytes_read_total),
		 static_cast<unsigned long long>(_no_data_reads),
		 static_cast<unsigned long long>(_read_errors),
		 static_cast<long long>(_last_read_size),
		 static_cast<unsigned long long>(_last_read == 0 ? 0 : hrt_elapsed_time(&_last_read)));
	PX4_INFO("startup: state %u (%s), prompt seen %u, sync tries %u, stop tries %u, format tries %u, fsync tries %llu",
		 static_cast<unsigned>(_startup_state),
		 startup_state_name(),
		 static_cast<unsigned>(_startup_prompt_seen),
		 static_cast<unsigned>(_startup_sync_attempts),
		 static_cast<unsigned>(_startup_stop_attempts),
		 static_cast<unsigned>(_startup_format_attempts),
		 static_cast<unsigned long long>(_startup_fsync_attempts));
	PX4_INFO("startup: prompt timeouts %llu, discarded %llu bytes",
		 static_cast<unsigned long long>(_startup_prompt_timeouts),
		 static_cast<unsigned long long>(_startup_discarded_bytes));
	PX4_INFO("startup: mode command %s, preamble done %u, mode tries %llu, reboot tries %llu",
		 mode_command_for_param(_mode),
		 static_cast<unsigned>(_startup_mode_preamble_done),
		 static_cast<unsigned long long>(_startup_mode_attempts),
		 static_cast<unsigned long long>(_startup_reboot_attempts));
	PX4_INFO("startup: baud target %u, probe done %u, probe tries %llu, set tries %llu",
		 ASDT1_DESIRED_BAUD,
		 static_cast<unsigned>(_startup_baud_probe_done),
		 static_cast<unsigned long long>(_startup_baud_probe_attempts),
		 static_cast<unsigned long long>(_startup_baud_set_attempts));
	PX4_INFO("parser: state %u, begin match %u, end match %u, frame buffer %u bytes",
		 static_cast<unsigned>(_parser_state),
		 static_cast<unsigned>(_begin_match_index),
		 static_cast<unsigned>(_end_match_index),
		 static_cast<unsigned>(_frame_buffer_len));
	PX4_INFO("parser counters: frames rx %llu, frames pub %llu, resets %llu, end fails %llu",
		 static_cast<unsigned long long>(_frames_rx),
		 static_cast<unsigned long long>(_frames_pub),
		 static_cast<unsigned long long>(_parser_resets),
		 static_cast<unsigned long long>(_end_marker_failures));
	PX4_INFO("filter: below %u cm samples %llu",
		 static_cast<unsigned>(MIN_VALID_DISTANCE_CM),
		 static_cast<unsigned long long>(_below_min_distance_samples));
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
			char hex[16 * 3] {};
			char ascii[17] {};
			size_t hex_pos = 0;

			for (size_t i = 0; i < chunk_len; i++) {
				const uint8_t byte = _last_read_bytes[offset + i];
				hex_pos += snprintf(&hex[hex_pos], sizeof(hex) - hex_pos, "%02x%s", byte, (i + 1 < chunk_len) ? " " : "");
				ascii[i] = (byte >= 32 && byte <= 126) ? static_cast<char>(byte) : '.';
			}

			PX4_INFO("last read[%u..%u] ascii: %s",
				 static_cast<unsigned>(offset),
				 static_cast<unsigned>(offset + chunk_len - 1),
				 ascii);
		}
	}
}

void AS_DT1::start()
{
	_baud = ASDT1_DESIRED_BAUD;
	ScheduleNow();
}

void AS_DT1::stop()
{
	ScheduleClear();
	close_port();
}

void AS_DT1::Run()
{
	if (_fd < 0) {
		if (open_port() != PX4_OK) {
			return;
		}

		_startup_state = !_startup_baud_probe_done ? StartupState::BaudProbeDrain :
				 (_startup_mode_preamble_done ? StartupState::SyncDrain : StartupState::ModeSyncDrain);
		reset_startup_prompt();
		_startup_deadline = 0;
		_startup_sync_attempts = 0;
		_startup_stop_attempts = 0;
		_startup_format_attempts = 0;
		_startup_fsync_attempts = 0;
		_startup_frame_wait_baseline = 0;
		_startup_prompt_timeouts = 0;
		_startup_discarded_bytes = 0;
		reset_parser();
		ScheduleNow();
		return;
	}

	if (_startup_state != StartupState::Streaming) {
		// The sensor may boot in command mode or already streaming. Bring it
		// to a known state before accepting measurement frames.
		// Command echoes/prompts can be missed, so retries keep startup moving
		// and the final check is a valid frame.
		switch (_startup_state) {
		case StartupState::BaudProbeDrain:
			if (drain_input() < 0) {
				close_port();
				return;
			}

			reset_startup_prompt();
			_startup_state = StartupState::BaudProbeSendPromptSync;
			ScheduleNow();
			return;

		case StartupState::BaudProbeSendPromptSync:
			if (_startup_baud_probe_attempts >= BAUD_PROBE_ATTEMPT_LIMIT) {
				_baud = (_baud == ASDT1_DESIRED_BAUD) ? ASDT1_FALLBACK_BAUD : ASDT1_DESIRED_BAUD;
				_startup_baud_probe_attempts = 0;
				reset_startup_prompt();
				close_port();
				ScheduleNow();
				return;
			}

			if (write_prompt_sync() != PX4_OK) {
				close_port();
				return;
			}

			tcdrain(_fd);
			_startup_baud_probe_attempts++;
			reset_startup_prompt();
			set_startup_deadline(PROMPT_SYNC_INTERVAL);
			_startup_state = StartupState::BaudProbeWaitResponse;
			break;

		case StartupState::BaudProbeWaitResponse: {
				const ssize_t bytes_read = read_baud_probe_response();

				if (bytes_read < 0) {
					close_port();
					return;
				}

				if (_startup_prompt_seen || _startup_begin_seen) {
					_startup_baud_probe_done = true;
					reset_startup_prompt();
					_startup_state = StartupState::ModeSyncDrain;
					ScheduleNow();
					return;
				}

				if (startup_deadline_elapsed()) {
					_startup_state = StartupState::BaudProbeSendPromptSync;
					ScheduleNow();
					return;
				}
			}

			break;

		case StartupState::ModeSyncDrain:
			if (drain_input() < 0) {
				close_port();
				return;
			}

			_startup_sync_attempts = 0;
			_startup_state = StartupState::ModeSendPromptSync;
			ScheduleNow();
			return;

		case StartupState::ModeSendPromptSync:
			if (_startup_sync_attempts >= PROMPT_SYNC_ATTEMPT_LIMIT) {
				_startup_state = StartupState::ModeDrainAfterSync;
				ScheduleDelayed(PROMPT_SYNC_SETTLE_INTERVAL);
				return;
			}

			if (write_prompt_sync() != PX4_OK) {
				close_port();
				return;
			}

			tcdrain(_fd);
			_startup_sync_attempts++;
			reset_startup_prompt();
			set_startup_deadline(PROMPT_SYNC_INTERVAL);
			_startup_state = StartupState::ModeWaitPromptSync;
			break;

		case StartupState::ModeWaitPromptSync: {
				const ssize_t bytes_read = read_startup_response();

				if (bytes_read < 0) {
					close_port();
					return;
				}

				if (_startup_prompt_seen || bytes_read > 0) {
					reset_startup_prompt();
					_startup_state = StartupState::ModeDrainAfterSync;
					ScheduleDelayed(PROMPT_SYNC_SETTLE_INTERVAL);
					return;
				}

				if (startup_deadline_elapsed()) {
					_startup_prompt_timeouts++;
					_startup_state = StartupState::ModeSendPromptSync;
					ScheduleNow();
					return;
				}
			}

			break;

		case StartupState::ModeDrainAfterSync:
			if (drain_input() < 0) {
				close_port();
				return;
			}

			reset_startup_prompt();
			_startup_stop_attempts = 0;
			_startup_state = StartupState::ModeSendStop;
			ScheduleNow();
			return;

		case StartupState::ModeSendStop:
			// Stop an old stream before changing persistent mode or baud settings.
			if (write_command_padded(CMD_FSYNC_STOP) != PX4_OK) {
				close_port();
				return;
			}

			tcdrain(_fd);
			_startup_stop_attempts++;
			reset_startup_prompt();
			set_startup_deadline(COMMAND_RESPONSE_TIMEOUT);
			_startup_state = StartupState::ModeWaitStopPrompt;
			break;

		case StartupState::ModeWaitStopPrompt:
			if (read_startup_response() < 0) {
				close_port();
				return;
			}

			if (_startup_prompt_seen) {
				reset_startup_prompt();
				_startup_mode_attempts = 0;
				_startup_state = StartupState::SendMode;
				ScheduleDelayed(FORMAT_SETTLE_INTERVAL);
				return;
			}

			if (startup_deadline_elapsed()) {
				_startup_prompt_timeouts++;

				if (_startup_stop_attempts < STARTUP_COMMAND_RETRY_LIMIT) {
					_startup_state = StartupState::ModeSendStop;
					ScheduleNow();
					return;
				}

				PX4_WARN("no pre-mode fsync 0 prompt after %u tries, continuing",
					 static_cast<unsigned>(_startup_stop_attempts));
				_startup_mode_attempts = 0;
				_startup_state = StartupState::SendMode;
				ScheduleDelayed(FORMAT_SETTLE_INTERVAL);
				return;
			}

			break;

		case StartupState::SendMode:
			if (drain_input() < 0) {
				close_port();
				return;
			}

			// flmode selects the range and point-count layout used below.
			if (write_command_padded(mode_command_for_param(_mode)) != PX4_OK) {
				close_port();
				return;
			}

			tcdrain(_fd);
			_startup_mode_attempts++;
			reset_startup_prompt();
			set_startup_deadline(COMMAND_RESPONSE_TIMEOUT);
			_startup_state = StartupState::WaitModePrompt;
			break;

		case StartupState::WaitModePrompt:
			if (read_startup_response() < 0) {
				close_port();
				return;
			}

			if (_startup_prompt_seen) {
				reset_startup_prompt();
				_startup_state = StartupState::SendBaud;
				ScheduleDelayed(FORMAT_SETTLE_INTERVAL);
				return;
			}

			if (startup_deadline_elapsed()) {
				_startup_prompt_timeouts++;

				if (_startup_mode_attempts < STARTUP_COMMAND_RETRY_LIMIT) {
					_startup_state = StartupState::SendMode;
					ScheduleNow();
					return;
				}

				PX4_WARN("no flmode prompt after %llu tries, setting baud anyway",
					 static_cast<unsigned long long>(_startup_mode_attempts));
				_startup_state = StartupState::SendBaud;
				ScheduleNow();
				return;
			}

			break;

		case StartupState::SendBaud:
			if (drain_input() < 0) {
				close_port();
				return;
			}

			// fluart is stored by the sensor; reopen at the new rate after reboot.
			if (write_command_padded(CMD_FLUART_921600) != PX4_OK) {
				close_port();
				return;
			}

			tcdrain(_fd);
			_startup_baud_set_attempts++;
			reset_startup_prompt();
			set_startup_deadline(COMMAND_RESPONSE_TIMEOUT);
			_startup_state = StartupState::WaitBaudPrompt;
			break;

		case StartupState::WaitBaudPrompt:
			if (read_startup_response() < 0) {
				close_port();
				return;
			}

			if (_startup_prompt_seen) {
				reset_startup_prompt();
				_startup_state = StartupState::SendReboot;
				ScheduleDelayed(FORMAT_SETTLE_INTERVAL);
				return;
			}

			if (startup_deadline_elapsed()) {
				_startup_prompt_timeouts++;

				if (_startup_baud_set_attempts < STARTUP_COMMAND_RETRY_LIMIT) {
					_startup_state = StartupState::SendBaud;
					ScheduleNow();
					return;
				}

				PX4_WARN("no fluart prompt after %llu tries, rebooting anyway",
					 static_cast<unsigned long long>(_startup_baud_set_attempts));
				_startup_state = StartupState::SendReboot;
				ScheduleNow();
				return;
			}

			break;

		case StartupState::SendReboot:
			if (write_command_padded(CMD_REBOOT) != PX4_OK) {
				close_port();
				return;
			}

			tcdrain(_fd);
			_startup_reboot_attempts++;
			_startup_mode_preamble_done = true;
			_startup_baud_probe_done = true;
			_baud = ASDT1_DESIRED_BAUD;
			reset_startup_prompt();
			close_port();
			ScheduleDelayed(REBOOT_SETTLE_INTERVAL);
			return;

		case StartupState::SyncDrain:
			if (drain_input() < 0) {
				close_port();
				return;
			}

			_startup_sync_attempts = 0;
			_startup_state = StartupState::SendPromptSync;
			ScheduleNow();
			return;

		case StartupState::SendPromptSync:
			if (_startup_sync_attempts >= PROMPT_SYNC_ATTEMPT_LIMIT) {
				_startup_state = StartupState::DrainAfterSync;
				ScheduleDelayed(PROMPT_SYNC_SETTLE_INTERVAL);
				return;
			}

			if (write_prompt_sync() != PX4_OK) {
				close_port();
				return;
			}

			tcdrain(_fd);
			_startup_sync_attempts++;
			reset_startup_prompt();
			set_startup_deadline(PROMPT_SYNC_INTERVAL);
			_startup_state = StartupState::WaitPromptSync;
			break;

		case StartupState::WaitPromptSync: {
				const ssize_t bytes_read = read_startup_response();

				if (bytes_read < 0) {
					close_port();
					return;
				}

				if (_startup_prompt_seen || bytes_read > 0) {
					reset_startup_prompt();
					_startup_state = StartupState::DrainAfterSync;
					ScheduleDelayed(PROMPT_SYNC_SETTLE_INTERVAL);
					return;
				}

				if (startup_deadline_elapsed()) {
					_startup_prompt_timeouts++;
					_startup_state = StartupState::SendPromptSync;
					ScheduleNow();
					return;
				}
			}

			break;

		case StartupState::DrainAfterSync:
			if (drain_input() < 0) {
				close_port();
				return;
			}

			reset_startup_prompt();
			_startup_stop_attempts = 0;
			_startup_state = StartupState::SendStop;
			ScheduleNow();
			return;

		case StartupState::SendStop:
			if (write_command_padded(CMD_FSYNC_STOP) != PX4_OK) {
				close_port();
				return;
			}

			tcdrain(_fd);
			_startup_stop_attempts++;
			reset_startup_prompt();
			set_startup_deadline(COMMAND_RESPONSE_TIMEOUT);
			_startup_state = StartupState::WaitStopPrompt;
			break;

		case StartupState::WaitStopPrompt:
			if (read_startup_response() < 0) {
				close_port();
				return;
			}

			if (_startup_prompt_seen) {
				reset_startup_prompt();
				_startup_format_attempts = 0;
				_startup_state = StartupState::SendFormat;
				ScheduleDelayed(FORMAT_SETTLE_INTERVAL);
				return;
			}

			if (startup_deadline_elapsed()) {
				_startup_prompt_timeouts++;

				if (_startup_stop_attempts < STARTUP_COMMAND_RETRY_LIMIT) {
					_startup_state = StartupState::SendStop;
					ScheduleNow();
					return;
				}

				PX4_WARN("no fsync 0 prompt after %u tries, continuing",
					 static_cast<unsigned>(_startup_stop_attempts));
				_startup_format_attempts = 0;
				_startup_state = StartupState::SendFormat;
				ScheduleNow();
				return;
			}

			break;

		case StartupState::SendFormat:
			if (drain_input() < 0) {
				close_port();
				return;
			}

			// binz is the binary MP format parsed by parse_byte().
			if (write_command_padded(CMD_FORMAT_BINZ) != PX4_OK) {
				close_port();
				return;
			}

			tcdrain(_fd);
			_startup_format_attempts++;
			reset_startup_prompt();
			set_startup_deadline(COMMAND_RESPONSE_TIMEOUT);
			_startup_state = StartupState::WaitFormatPrompt;
			break;

		case StartupState::WaitFormatPrompt:
			if (read_startup_response() < 0) {
				close_port();
				return;
			}

			if (_startup_prompt_seen) {
				reset_startup_prompt();
				_startup_state = StartupState::SendFsync;
				ScheduleDelayed(FORMAT_SETTLE_INTERVAL);
				return;
			}

			if (startup_deadline_elapsed()) {
				_startup_prompt_timeouts++;

				if (_startup_format_attempts < STARTUP_COMMAND_RETRY_LIMIT) {
					_startup_state = StartupState::SendFormat;
					ScheduleNow();
					return;
				}

				PX4_WARN("no format prompt after %u tries, sending fsync",
					 static_cast<unsigned>(_startup_format_attempts));
				_startup_state = StartupState::SendFsync;
				ScheduleNow();
				return;
			}

			break;

		case StartupState::SendFsync:
			if (drain_input() < 0) {
				close_port();
				return;
			}

			// If no frame follows, resend fsync rather than trusting the echo.
			if (write_command_padded(CMD_FSYNC_START) != PX4_OK) {
				close_port();
				return;
			}

			tcdrain(_fd);
			_startup_fsync_attempts++;
			_startup_frame_wait_baseline = _frames_rx;
			reset_parser();
			reset_startup_prompt();
			set_startup_deadline(FIRST_FRAME_TIMEOUT);
			_startup_state = StartupState::WaitFirstFrame;
			break;

		case StartupState::Streaming:
			break;

		case StartupState::WaitFirstFrame:
			if (read_and_parse_available() != PX4_OK) {
				close_port();
				return;
			}

			if (_frames_rx > _startup_frame_wait_baseline) {
				_startup_state = StartupState::Streaming;
				ScheduleDelayed(_interval);
				return;
			}

			if (startup_deadline_elapsed()) {
				_startup_state = StartupState::SendFsync;
				ScheduleNow();
				return;
			}

			break;

		}

		ScheduleDelayed(_interval);
		return;
	}

	(void)read_and_parse_available();
	ScheduleDelayed(_interval);
}

int AS_DT1::open_port()
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

void AS_DT1::close_port()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

int AS_DT1::write_command_padded(const char *command)
{
	if (_fd < 0) {
		return PX4_ERROR;
	}

	if (command == nullptr) {
		return PX4_ERROR;
	}

	char buffer[COMMAND_BUFFER_SIZE] {};
	const size_t command_len = strnlen(command, sizeof(buffer) - 1);

	if (command_len == 0 || command_len >= sizeof(buffer) - 1) {
		return PX4_ERROR;
	}

	memcpy(buffer, command, command_len);
	size_t length = command_len + 1; // include trailing CR
	buffer[command_len] = '\r';

	const size_t remainder = length % COMMAND_PADDING_BYTES;

	if (remainder != 0) {
		const size_t padded_length = length + (COMMAND_PADDING_BYTES - remainder);

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

int AS_DT1::write_prompt_sync()
{
	if (_fd < 0) {
		return PX4_ERROR;
	}

	const char prompt_sync[] = "\r";
	const ssize_t written = ::write(_fd, prompt_sync, sizeof(prompt_sync) - 1);
	_last_write = written;
	_last_command_len = sizeof(prompt_sync) - 1;
	strncpy(_last_command, "prompt sync", sizeof(_last_command) - 1);
	_last_command[sizeof(_last_command) - 1] = '\0';

	if (written != static_cast<ssize_t>(sizeof(prompt_sync) - 1)) {
		PX4_ERR("write failed (%i)", errno);
		return PX4_ERROR;
	}

	return PX4_OK;
}

ssize_t AS_DT1::drain_input()
{
	ssize_t total_read = 0;
	uint8_t buffer[READ_BUFFER_SIZE] {};

	for (uint8_t i = 0; i < STARTUP_DRAIN_READ_LIMIT; i++) {
		_read_attempts++;
		const ssize_t bytes_read = ::read(_fd, buffer, sizeof(buffer));

		if (bytes_read > 0) {
			record_read(buffer, bytes_read);
			_startup_discarded_bytes += static_cast<uint64_t>(bytes_read);
			total_read += bytes_read;
			continue;
		}

		if (bytes_read == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
			return total_read;
		}

		_read_errors++;
		return PX4_ERROR;
	}

	return total_read;
}

ssize_t AS_DT1::read_startup_response()
{
	_read_attempts++;

	uint8_t buffer[READ_BUFFER_SIZE] {};
	const ssize_t bytes_read = ::read(_fd, buffer, sizeof(buffer));

	if (bytes_read <= 0) {
		if (bytes_read == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
			_no_data_reads++;
			return 0;
		}

		_read_errors++;
		return PX4_ERROR;
	}

	record_read(buffer, bytes_read);

	constexpr char prompt[] = "\r\n> ";
	constexpr size_t prompt_len = sizeof(prompt) - 1;

	for (ssize_t i = 0; i < bytes_read; i++) {
		const char byte = static_cast<char>(buffer[i]);

		if (byte == prompt[_startup_prompt_match_index]) {
			_startup_prompt_match_index++;

			if (_startup_prompt_match_index == prompt_len) {
				_startup_prompt_seen = true;
				_startup_prompt_match_index = 0;
			}

		} else {
			_startup_prompt_match_index = (byte == prompt[0]) ? 1 : 0;
		}
	}

	return bytes_read;
}

ssize_t AS_DT1::read_baud_probe_response()
{
	_read_attempts++;

	uint8_t buffer[READ_BUFFER_SIZE] {};
	const ssize_t bytes_read = ::read(_fd, buffer, sizeof(buffer));

	if (bytes_read <= 0) {
		if (bytes_read == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
			_no_data_reads++;
			return 0;
		}

		_read_errors++;
		return PX4_ERROR;
	}

	record_read(buffer, bytes_read);

	constexpr char prompt[] = "\r\n> ";
	constexpr size_t prompt_len = sizeof(prompt) - 1;
	constexpr char begin_marker[] = "BEGIN MP\r\n";
	constexpr size_t begin_marker_len = sizeof(begin_marker) - 1;

	for (ssize_t i = 0; i < bytes_read; i++) {
		const char byte = static_cast<char>(buffer[i]);

		if (byte == prompt[_startup_prompt_match_index]) {
			_startup_prompt_match_index++;

			if (_startup_prompt_match_index == prompt_len) {
				_startup_prompt_seen = true;
				_startup_prompt_match_index = 0;
			}

		} else {
			_startup_prompt_match_index = (byte == prompt[0]) ? 1 : 0;
		}

		if (byte == begin_marker[_startup_begin_match_index]) {
			_startup_begin_match_index++;

			if (_startup_begin_match_index == begin_marker_len) {
				_startup_begin_seen = true;
				_startup_begin_match_index = 0;
			}

		} else {
			_startup_begin_match_index = (byte == begin_marker[0]) ? 1 : 0;
		}
	}

	return bytes_read;
}

void AS_DT1::record_read(const uint8_t *buffer, ssize_t bytes_read)
{
	if (buffer == nullptr || bytes_read <= 0) {
		return;
	}

	_last_read = hrt_absolute_time();
	_last_read_size = bytes_read;
	_bytes_read_total += static_cast<uint64_t>(bytes_read);
	_last_read_bytes_len = math::min(static_cast<size_t>(bytes_read), LAST_READ_CAPTURE_SIZE);
	memcpy(_last_read_bytes, buffer, _last_read_bytes_len);
}

void AS_DT1::reset_parser()
{
	_parser_state = ParserState::FindBegin;
	_frame_buffer_len = 0;
	_begin_match_index = 0;
	_end_match_index = 0;
}

void AS_DT1::reset_startup_prompt()
{
	_startup_prompt_seen = false;
	_startup_prompt_match_index = 0;
	_startup_begin_seen = false;
	_startup_begin_match_index = 0;
}

void AS_DT1::set_startup_deadline(hrt_abstime timeout_us)
{
	_startup_deadline = hrt_absolute_time() + timeout_us;
}

bool AS_DT1::startup_deadline_elapsed() const
{
	return _startup_deadline != 0 && hrt_absolute_time() >= _startup_deadline;
}

const char *AS_DT1::startup_state_name() const
{
	switch (_startup_state) {
	case StartupState::SendFormat:
		return "send format";

	case StartupState::WaitFormatPrompt:
		return "wait format prompt";

	case StartupState::SendFsync:
		return "send fsync";

	case StartupState::Streaming:
		return "streaming";

	case StartupState::WaitFirstFrame:
		return "wait first frame";

	case StartupState::SyncDrain:
		return "sync drain";

	case StartupState::SendPromptSync:
		return "send prompt sync";

	case StartupState::WaitPromptSync:
		return "wait prompt sync";

	case StartupState::DrainAfterSync:
		return "drain after sync";

	case StartupState::SendStop:
		return "send stop";

	case StartupState::WaitStopPrompt:
		return "wait stop prompt";

	case StartupState::SendMode:
		return "send mode";

	case StartupState::SendReboot:
		return "send reboot";

	case StartupState::ModeSyncDrain:
		return "mode sync drain";

	case StartupState::ModeSendPromptSync:
		return "mode send prompt sync";

	case StartupState::ModeWaitPromptSync:
		return "mode wait prompt sync";

	case StartupState::ModeDrainAfterSync:
		return "mode drain after sync";

	case StartupState::ModeSendStop:
		return "mode send stop";

	case StartupState::ModeWaitStopPrompt:
		return "mode wait stop prompt";

	case StartupState::WaitModePrompt:
		return "wait mode prompt";

	case StartupState::BaudProbeDrain:
		return "baud probe drain";

	case StartupState::BaudProbeSendPromptSync:
		return "baud probe send prompt sync";

	case StartupState::BaudProbeWaitResponse:
		return "baud probe wait response";

	case StartupState::SendBaud:
		return "send baud";

	case StartupState::WaitBaudPrompt:
		return "wait baud prompt";
	}

	return "unknown";
}

int AS_DT1::read_and_print_response(const char *label)
{
	char line[97] {};
	size_t line_len = 0;
	bool got_response = false;
	bool got_real_response = false;

	const auto print_line = [&]() {
		if (line_len > 0) {
			line[line_len] = '\0';

			if (!strncmp(line, "fl", 2) && strncmp(line, "flshow", 6)) {
				got_real_response = true;
			}

			PX4_INFO("%s: %s", label, line);
			line_len = 0;
		}
	};

	while (true) {
		_read_attempts++;

		uint8_t buffer[READ_BUFFER_SIZE] {};
		const ssize_t bytes_read = ::read(_fd, buffer, sizeof(buffer));

		if (bytes_read <= 0) {
			if (bytes_read == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
				_no_data_reads++;
				print_line();

				if (!got_response) {
					PX4_WARN("%s: no response", label);
				}

				return got_real_response ? 1 : PX4_OK;
			}

			_read_errors++;
			return PX4_ERROR;
		}

		got_response = true;
		record_read(buffer, bytes_read);

		for (ssize_t i = 0; i < bytes_read; i++) {
			const uint8_t byte = buffer[i];

			if (byte == '\r') {
				continue;
			}

			if (byte == '\n') {
				print_line();
				continue;
			}

			line[line_len++] = (byte >= 32 && byte <= 126) ? static_cast<char>(byte) : '.';

			if (line_len >= sizeof(line) - 1) {
				print_line();
			}
		}
	}
}

int AS_DT1::read_and_parse_available()
{
	_read_attempts++;

	uint8_t buffer[READ_BUFFER_SIZE] {};
	bool got_data = false;

	for (uint8_t read_count = 0; read_count < READ_DRAIN_LIMIT; read_count++) {
		const ssize_t bytes_read = ::read(_fd, buffer, sizeof(buffer));

		if (bytes_read <= 0) {
			if (bytes_read == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
				if (!got_data) {
					_no_data_reads++;
				}

				return PX4_OK;
			}

			_read_errors++;
			return PX4_ERROR;
		}

		got_data = true;
		record_read(buffer, bytes_read);

		for (ssize_t i = 0; i < bytes_read; i++) {
			if (_startup_state == StartupState::WaitFormatPrompt && buffer[i] == '>') {
				_startup_prompt_seen = true;
			}

			parse_byte(buffer[i]);
		}
	}

	return PX4_OK;
}

bool AS_DT1::parse_byte(uint8_t byte)
{
	constexpr char begin_marker[] = "BEGIN MP\r\n";
	constexpr size_t begin_marker_len = sizeof(begin_marker) - 1;
	constexpr char end_marker[] = "END";
	constexpr size_t end_marker_len = sizeof(end_marker) - 1;

	// AS-DT1 binz frames are still bookended by ASCII markers.
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

	case ParserState::ReadPayload: {
			const size_t expected_frame_size = frame_size_for_mode(_mode);

			if (_frame_buffer_len >= expected_frame_size || _frame_buffer_len >= ASDT1_FRAME_BUFFER_SIZE) {
				_parser_resets++;
				reset_parser();
				_begin_match_index = (byte == static_cast<uint8_t>(begin_marker[0])) ? 1 : 0;
				break;
			}

			_frame_buffer[_frame_buffer_len++] = byte;

			if (_frame_buffer_len == expected_frame_size) {
				_parser_state = ParserState::ReadEnd;
				_end_match_index = 0;
			}

			break;
		}

	case ParserState::ReadEnd:
		if (byte == static_cast<uint8_t>(end_marker[_end_match_index])) {
			_end_match_index++;

			if (_end_match_index == end_marker_len) {
				_frames_rx++;
				process_frame(_frame_buffer, _frame_buffer_len);
				reset_parser();
				return true;
			}

		} else {
			_parser_resets++;
			_end_marker_failures++;
			reset_parser();
			_begin_match_index = (byte == static_cast<uint8_t>(begin_marker[0])) ? 1 : 0;
		}

		break;
	}

	return false;
}

int AS_DT1::process_frame(const uint8_t *frame, size_t length)
{
	const size_t expected_frame_size = frame_size_for_mode(_mode);
	const size_t sample_count = sample_count_for_mode(_mode);
	const int min_used_row = min_used_row_for_mode(_mode);
	const int max_used_row = max_used_row_for_mode(_mode);

	if (frame == nullptr || length != expected_frame_size) {
		PX4_ERR("invalid AS-DT1 binz frame length: %u", static_cast<unsigned>(length));
		_parser_resets++;
		return PX4_ERROR;
	}

	_last_frame_processed_len = length;
	_last_sample_count = sample_count;
	_last_valid_bins = 0;
	_last_closest_distance = UINT16_MAX;

	for (uint8_t i = 0; i < BIN_COUNT; i++) {
		_obstacle_distance.distances[i] = UINT16_MAX;
	}

	// Only publish the rows that cover the horizontal band used by obstacle_distance.
	for (size_t sample = 0; sample < sample_count; sample++) {
		const int layout_index = sample_to_layout_index(sample, _mode);
		int row = 0;
		int col = 0;

		if (!layout_index_to_row_col(layout_index, row, col)) {
			continue;
		}

		if (row < min_used_row || row > max_used_row) {
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

		// Very close returns are below the sensor's usable range and can flicker.
		if (distance_cm < MIN_VALID_DISTANCE_CM) {
			_below_min_distance_samples++;
			continue;
		}

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

uint32_t AS_DT1::decode_20bit_raw(const uint8_t *data, size_t sample_index)
{
	// Two 20-bit Z samples are packed into each 5-byte group.
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

uint16_t AS_DT1::max_distance_for_mode(int32_t mode)
{
	switch (mode) {
	case 3: // 20M
		return 2000;

	case 4: // 40M
		return 4000;

	case 0: // 30MSTD
	case 1: // 30M15F
	case 2: // 30M30F
	default:
		return 3000;
	}
}

const char *AS_DT1::mode_command_for_param(int32_t mode)
{
	switch (mode) {
	case 0:
		return "flmode 30mstd";

	case 1:
		return "flmode 30m15f";

	case 2:
		return "flmode 30m30f";

	case 3:
		return "flmode 20m";

	case 4:
		return "flmode 40m";

	default:
		return "flmode 30mstd";
	}
}

bool AS_DT1::short_frame_mode(int32_t mode)
{
	return mode == 2;
}

size_t AS_DT1::sample_count_for_mode(int32_t mode)
{
	return short_frame_mode(mode) ? ASDT1_SHORT_SAMPLE_COUNT : ASDT1_MAX_SAMPLE_COUNT;
}

size_t AS_DT1::frame_size_for_mode(int32_t mode)
{
	return short_frame_mode(mode) ? ASDT1_BINZ_SHORT_FRAME_SIZE : ASDT1_BINZ_FRAME_SIZE;
}

int AS_DT1::min_used_row_for_mode(int32_t mode)
{
	return short_frame_mode(mode) ? MIN_USED_SHORT_ROW : MIN_USED_ROW;
}

int AS_DT1::max_used_row_for_mode(int32_t mode)
{
	return short_frame_mode(mode) ? MAX_USED_SHORT_ROW : MAX_USED_ROW;
}

int AS_DT1::sample_to_layout_index(size_t sample_index)
{
	return sample_to_layout_index(sample_index, 0);
}

int AS_DT1::sample_to_layout_index(size_t sample_index, int32_t mode)
{
	// The MP stream is not row-major; these tables restore the 24x24 grid order.
	static constexpr int MPDATATBL_FULL[ASDT1_MAX_SAMPLE_COUNT] = {
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

	static constexpr int MPDATATBL_SHORT[ASDT1_SHORT_SAMPLE_COUNT] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
		24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
		96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
		120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143,
		192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215,
		216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
		48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71,
		72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95,
		144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167,
		168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191,
		240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263,
		264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287,
	};

	if (short_frame_mode(mode)) {
		if (sample_index >= ASDT1_SHORT_SAMPLE_COUNT) {
			return -1;
		}

		return MPDATATBL_SHORT[sample_index];
	}

	if (sample_index >= ASDT1_MAX_SAMPLE_COUNT) {
		return -1;
	}

	return MPDATATBL_FULL[sample_index];
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
	// obstacle_distance is a 72-bin circle at 5 degrees per bin.
	int bin = static_cast<int>(roundf(angle_deg / OBSTACLE_INCREMENT_DEG));

	while (bin < 0) {
		bin += BIN_COUNT;
	}

	return bin % BIN_COUNT;
}
