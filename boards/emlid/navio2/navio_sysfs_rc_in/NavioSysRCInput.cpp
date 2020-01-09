/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

#include "NavioSysRCInput.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

namespace navio_sysfs_rc_in
{

#define RCINPUT_DEVICE_PATH_BASE "/sys/kernel/rcio/rcin"

#define RCINPUT_MEASURE_INTERVAL_US 20000 // microseconds

NavioSysRCInput::NavioSysRCInput() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_isRunning = true;
};

NavioSysRCInput::~NavioSysRCInput()
{
	ScheduleClear();

	_isRunning = false;

	for (int i = 0; i < _channels; ++i) {
		if (_ch_fd[i] != -1) {
			::close(_ch_fd[i]);
		}
	}
}

int NavioSysRCInput::navio_rc_init()
{
	int i;
	char buf[64];

	for (i = 0; i < _channels; ++i) {
		::snprintf(buf, sizeof(buf), "%s/ch%d", RCINPUT_DEVICE_PATH_BASE, i);
		int fd = ::open(buf, O_RDONLY);

		if (fd < 0) {
			PX4_WARN("error: open %d failed", i);
			break;
		}

		_ch_fd[i] = fd;
	}

	for (; i < input_rc_s::RC_INPUT_MAX_CHANNELS; ++i) {
		_data.values[i] = UINT16_MAX;
	}

	return 0;
}

int NavioSysRCInput::start()
{
	int result = navio_rc_init();

	if (result != 0) {
		PX4_WARN("error: RC initialization failed");
		return -1;
	}

	_should_exit.store(false);
	ScheduleOnInterval(RCINPUT_MEASURE_INTERVAL_US);

	return result;
}

void NavioSysRCInput::stop()
{
	_should_exit.store(true);
}

void NavioSysRCInput::Run()
{
	if (_should_exit.load()) {
		ScheduleClear();
		return;
	}

	uint64_t ts = 0;
	char buf[12] {};

	for (int i = 0; i < _channels; ++i) {
		int res;

		if ((res = ::pread(_ch_fd[i], buf, sizeof(buf) - 1, 0)) < 0) {
			_data.values[i] = UINT16_MAX;
			continue;
		}

		ts = hrt_absolute_time();

		buf[sizeof(buf) - 1] = '\0';

		_data.values[i] = atoi(buf);
	}

	if (ts > 0) {
		_data.timestamp_last_signal = ts;
		_data.channel_count = _channels;
		_data.rssi = 100;
		_data.rc_lost_frame_count = 0;
		_data.rc_total_frame_count = 1;
		_data.rc_ppm_frame_length = 100;
		_data.rc_failsafe = false;
		_data.rc_lost = false;
		_data.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM;
		_data.timestamp = hrt_absolute_time();

		_input_rc_pub.publish(_data);
	}
}

}; // namespace navio_sysfs_rc_in
