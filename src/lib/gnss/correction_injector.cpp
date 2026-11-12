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


#include "correction_injector.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>

using namespace time_literals;

namespace gnss
{

// Without a chunk from the selected source for this long, another source may be taken
static constexpr hrt_abstime kSourceTimeout = 5_s;
// The injection rate is frames per second over windows this long
static constexpr hrt_abstime kRateWindow = 5_s;
// Start bit, eight data bits, stop bit
static constexpr uint32_t kBitsPerByteOnTheWire = 10;
// A full TX buffer is retried no sooner than this, and this late when the baudrate is unknown
static constexpr uint32_t kMinRetryDelayUs = 1_ms;
static constexpr uint32_t kFallbackRetryDelayUs = 10_ms;

CorrectionInjector::CorrectionInjector(const char *name, const char *port) :
	ScheduledWorkItem(name, px4::serial_port_to_wq(port))
{
	strncpy(_port, port, sizeof(_port) - 1);
}

CorrectionInjector::~CorrectionInjector()
{
	_moving_baseline_sub.unregisterCallback();

	for (auto &sub : _corrections_sub) {
		sub.unregisterCallback();
	}

	// Cancels a pending retry, then waits for a Run() in progress, which uses the members destroyed after this body
	ScheduleClear();
	Deinit();

	perf_free(_injected);
	perf_free(_tx_buffer_full);
	perf_free(_framer_full);
	perf_free(_unsupported);
	perf_free(_short_write);
}

void CorrectionInjector::start(const Config &config)
{
	_rate_window_start = hrt_absolute_time();
	_rate_window_count = perf_event_count(_injected);
	_injection_rate_hz = 0.f;

	// Only the stream in use schedules runs: a base would otherwise wake for each moving-baseline chunk it publishes
	if (config.stream == Stream::MovingBaseline) {
		_moving_baseline_sub.registerCallback();

	} else {
		for (auto &sub : _corrections_sub) {
			sub.registerCallback();
		}
	}

	pthread_mutex_lock(&_mutex);
	_config = config;
	_active = true;
	_write_error_reported = false;
	pthread_mutex_unlock(&_mutex);

	// What queued up during configuration still goes out: a base sends its position (1005/1006) only every few seconds
	ScheduleNow();
}

void CorrectionInjector::stop()
{
	// Returns only after a write in progress, so the owner's next command can't land inside an RTCM frame
	pthread_mutex_lock(&_mutex);
	_active = false;
	pthread_mutex_unlock(&_mutex);

	// Chunks still queue up until start(), which drains them
	_moving_baseline_sub.unregisterCallback();

	for (auto &sub : _corrections_sub) {
		sub.unregisterCallback();
	}

	// The descriptor is closed where it was opened. Bounded: a starved work queue leaks it rather than blocking the owner.
	ScheduleNow();

	for (int i = 0; i < 20; i++) {
		pthread_mutex_lock(&_mutex);
		const bool closed = (_fd < 0);
		pthread_mutex_unlock(&_mutex);

		if (closed) {
			break;
		}

		px4_usleep(5_ms);
	}
}

float CorrectionInjector::injection_rate_hz()
{
	const hrt_abstime now = hrt_absolute_time();

	if (now - _rate_window_start >= kRateWindow) {
		const uint64_t count = perf_event_count(_injected);
		_injection_rate_hz = (float)(count - _rate_window_count) * 1e6f / (float)(now - _rate_window_start);
		_rate_window_count = count;
		_rate_window_start = now;
	}

	return _injection_rate_hz;
}

void CorrectionInjector::Run()
{
	pthread_mutex_lock(&_mutex);

	if (!_active) {
		if (_fd >= 0) {
			::close(_fd);
			_fd = -1;
		}

	} else {
		if (_fd < 0) {
			_fd = ::open(_port, O_WRONLY | O_NONBLOCK | O_NOCTTY);

			if (_fd < 0 && !_write_error_reported) {
				PX4_ERR("%s open for injection failed: %d", _port, errno);
				_write_error_reported = true;
			}
		}

		if (_fd >= 0) {
			if (_config.stream == Stream::MovingBaseline) {
				drain_moving_baseline();

			} else {
				drain_corrections();
			}

			inject_frames();
		}
	}

	pthread_mutex_unlock(&_mutex);
}

void CorrectionInjector::drain_corrections()
{
	rtcm_data_s chunk;

	if (select_corrections_instance(chunk)) {
		add_chunk(chunk);
	}

	const int8_t selected = _selected_instance.load();

	if (selected < 0) {
		return;
	}

	for (unsigned n = 0; n < rtcm_data_s::ORB_QUEUE_LENGTH && _corrections_sub[selected].update(&chunk); n++) {
		if (chunk.device_id != _config.own_device_id) {
			add_chunk(chunk);
		}
	}
}

void CorrectionInjector::drain_moving_baseline()
{
	rtcm_data_s chunk;

	for (unsigned n = 0; n < rtcm_data_s::ORB_QUEUE_LENGTH && _moving_baseline_sub.update(&chunk); n++) {
		if (chunk.device_id != _config.own_device_id) {
			add_chunk(chunk);
		}
	}
}

// Once the selected source has gone quiet, takes the first instance with a fresh chunk from another device. update()
// hands out the oldest queued chunk, so a backlog from before start() is walked through here in one go rather than
// one chunk per publication. Returns true with the chunk that selected the source, which goes out like any other.
bool CorrectionInjector::select_corrections_instance(rtcm_data_s &chunk)
{
	if (_selected_instance.load() >= 0 && hrt_elapsed_time(&_last_chunk_time) < kSourceTimeout) {
		return false;
	}

	_selected_instance.store(-1);

	for (int8_t instance = 0; instance < rtcm_data_s::MAX_INSTANCES; instance++) {
		for (unsigned n = 0; n < rtcm_data_s::ORB_QUEUE_LENGTH && _corrections_sub[instance].update(&chunk); n++) {
			if (chunk.device_id != _config.own_device_id && hrt_elapsed_time(&chunk.timestamp) < kSourceTimeout) {
				_selected_instance.store(instance);
				return true;
			}
		}
	}

	return false;
}

void CorrectionInjector::add_chunk(const rtcm_data_s &chunk)
{
	const size_t len = math::min<size_t>(chunk.len, sizeof(chunk.data));

	if (_framer.addData(chunk.data, len) < len) {
		perf_count(_framer_full);
	}

	_last_chunk_time = hrt_absolute_time();
}

void CorrectionInjector::inject_frames()
{
	size_t frame_len = 0;
	CorrectionProtocol protocol = CorrectionProtocol::Rtcm3;
	const uint8_t *frame = nullptr;

	while ((frame = _framer.getNextMessage(&frame_len, &protocol)) != nullptr) {
		if ((_config.protocols & protocol_bit(protocol)) == 0) {
			_framer.consumeMessage(frame_len);
			perf_count(_unsupported);
			continue;
		}

		const ssize_t tx_space = tx_space_available();

		// Unknown on POSIX, where the write itself is the check
		if (tx_space >= 0 && tx_space < (ssize_t)frame_len) {
			perf_count(_tx_buffer_full);
			ScheduleDelayed(drain_time_us(frame_len - tx_space));
			break;
		}

		const ssize_t written = ::write(_fd, frame, frame_len);

		if (written < 0 && errno == EAGAIN) {
			// The buffer filled between the check and the write: the frame stays for the retry
			perf_count(_tx_buffer_full);
			ScheduleDelayed(drain_time_us(frame_len));
			break;
		}

		if (written != (ssize_t)frame_len) {
			perf_count(_short_write);

			if (written < 0 && !_write_error_reported) {
				PX4_ERR("%s injection write failed: %d", _port, errno);
				_write_error_reported = true;
			}
		}

		_framer.consumeMessage(frame_len);
		perf_count(_injected);
	}
}

ssize_t CorrectionInjector::tx_space_available() const
{
#if defined(FIONSPACE)
	int space = 0;

	if (::ioctl(_fd, FIONSPACE, &space) == 0) {
		return space;
	}

#endif
	return -1;
}

uint32_t CorrectionInjector::drain_time_us(size_t bytes) const
{
	const uint32_t baudrate = _config.baudrate;

	if (baudrate == 0) {
		return kFallbackRetryDelayUs;
	}

	return math::max<uint32_t>(kMinRetryDelayUs, (uint64_t)bytes * kBitsPerByteOnTheWire * 1_s / baudrate);
}

void CorrectionInjector::print_status() const
{
	PX4_INFO("injecting %s: %6.2f Hz, source instance %d",
		 _config.stream == Stream::MovingBaseline ? "moving baseline" : "corrections", (double)_injection_rate_hz,
		 (int)_selected_instance.load());
	perf_print_counter(_injected);
	perf_print_counter(_tx_buffer_full);
	perf_print_counter(_framer_full);
	perf_print_counter(_unsupported);
	perf_print_counter(_short_write);

	const CorrectionFramerStats stats = _framer.getStats();

	if (stats.messages_parsed > 0 || stats.crc_errors > 0 || stats.bytes_discarded > 0) {
		PX4_INFO("framed: %u RTCM3, %u SPARTN, %u UBX, %u CRC errors, %u bytes discarded",
			 (unsigned)stats.rtcm3_messages, (unsigned)stats.spartn_messages, (unsigned)stats.ubx_messages,
			 (unsigned)stats.crc_errors, (unsigned)stats.bytes_discarded);
	}
}

} // namespace gnss
