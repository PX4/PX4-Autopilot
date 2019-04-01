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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include <px4_config.h>
#include <px4_workqueue.h>
#include <px4_defines.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>

namespace navio_sysfs_rc_in
{

extern "C" __EXPORT int navio_sysfs_rc_in_main(int argc, char *argv[]);

#define RCINPUT_DEVICE_PATH_BASE "/sys/kernel/rcio/rcin"

#define RCINPUT_MEASURE_INTERVAL_US 20000 // microseconds


class RcInput
{
public:
	RcInput() :
		_shouldExit(false),
		_isRunning(false),
		_work{},
		_rcinput_pub(nullptr),
		_channels(8), //D8R-II plus
		_data{}
	{
		memset(_ch_fd, 0, sizeof(_ch_fd));
	}
	~RcInput()
	{
		work_cancel(HPWORK, &_work);
		_isRunning = false;
	}

	/* @return 0 on success, -errno on failure */
	int start();

	/* @return 0 on success, -errno on failure */
	void stop();

	/* Trampoline for the work queue. */
	static void cycle_trampoline(void *arg);

	bool isRunning() { return _isRunning; }

private:
	void _cycle();
	void _measure();

	bool _shouldExit;
	bool _isRunning;
	struct work_s _work;

	orb_advert_t _rcinput_pub;

	int _channels;
	int _ch_fd[input_rc_s::RC_INPUT_MAX_CHANNELS];
	struct input_rc_s _data;

	int navio_rc_init();
};

int RcInput::navio_rc_init()
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

	_rcinput_pub = orb_advertise(ORB_ID(input_rc), &_data);

	if (_rcinput_pub == nullptr) {
		PX4_WARN("error: advertise failed");
		return -1;
	}

	return 0;
}

int RcInput::start()
{
	int result = 0;

	result = navio_rc_init();

	if (result != 0) {
		PX4_WARN("error: RC initialization failed");
		return -1;
	}

	_isRunning = true;
	result = work_queue(HPWORK, &_work, (worker_t)&RcInput::cycle_trampoline, this, 0);

	if (result == -1) {
		_isRunning = false;
	}

	return result;
}

void RcInput::stop()
{
	_shouldExit = true;
}

void RcInput::cycle_trampoline(void *arg)
{
	RcInput *dev = reinterpret_cast<RcInput *>(arg);
	dev->_cycle();
}

void RcInput::_cycle()
{
	_measure();

	if (!_shouldExit) {
		work_queue(HPWORK, &_work, (worker_t)&RcInput::cycle_trampoline, this,
			   USEC2TICK(RCINPUT_MEASURE_INTERVAL_US));
	}
}

void RcInput::_measure(void)
{
	uint64_t ts;
	char buf[12];

	for (int i = 0; i < _channels; ++i) {
		int res;

		if ((res = ::pread(_ch_fd[i], buf, sizeof(buf) - 1, 0)) < 0) {
			_data.values[i] = UINT16_MAX;
			continue;
		}

		buf[sizeof(buf) - 1] = '\0';

		_data.values[i] = atoi(buf);
	}

	ts = hrt_absolute_time();
	_data.timestamp = ts;
	_data.timestamp_last_signal = ts;
	_data.channel_count = _channels;
	_data.rssi = 100;
	_data.rc_lost_frame_count = 0;
	_data.rc_total_frame_count = 1;
	_data.rc_ppm_frame_length = 100;
	_data.rc_failsafe = false;
	_data.rc_lost = false;
	_data.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM;

	orb_publish(ORB_ID(input_rc), _rcinput_pub, &_data);
}

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PX4_INFO("usage: navio_sysfs_rc_in {start|stop|status}");
}

static RcInput *rc_input = nullptr;

int navio_sysfs_rc_in_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (rc_input != nullptr && rc_input->isRunning()) {
			PX4_WARN("already running");
			/* this is not an error */
			return 0;
		}

		rc_input = new RcInput();

		// Check if alloc worked.
		if (rc_input == nullptr) {
			PX4_ERR("alloc failed");
			return -1;
		}

		int ret = rc_input->start();

		if (ret != 0) {
			PX4_ERR("start failed");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {

		if (rc_input == nullptr || !rc_input->isRunning()) {
			PX4_WARN("not running");
			/* this is not an error */
			return 0;
		}

		rc_input->stop();

		// Wait for task to die
		int i = 0;

		do {
			/* wait up to 3s */
			usleep(100000);

		} while (rc_input->isRunning() && ++i < 30);

		delete rc_input;
		rc_input = nullptr;

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (rc_input != nullptr && rc_input->isRunning()) {
			PX4_INFO("running");

		} else {
			PX4_INFO("not running\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;

}

}; // namespace navio_sysfs_rc_in
