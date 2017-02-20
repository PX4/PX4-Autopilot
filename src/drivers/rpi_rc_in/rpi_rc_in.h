/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charl. All rights reserved.
 *   Copyright (C) 2017 Fan.zhang. All rights reserved. 421395590@qq.com
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
#include <sys/ipc.h>
#include <sys/shm.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>

#include <px4_config.h>
#include <px4_workqueue.h>
#include <px4_defines.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>
#define RCINPUT_MEASURE_INTERVAL_US 20000
namespace rpi_rc_in {
class RcInput {
public:
	int *mem;
	key_t key;
	int shmid;
	RcInput() :
			mem(nullptr),key(4096),shmid(0),_shouldExit(false), _isRunning(false), _work { }, _rcinput_pub(
					nullptr), _channels(8),//D8R-II plus
			_data { } {
		//memset(_ch_fd, 0, sizeof(_ch_fd));
	}
	~RcInput() {
		this->mem = nullptr;
		work_cancel(HPWORK, &_work);
		_isRunning = false;
	}

	/* @return 0 on success, -errno on failure */
	int start();

	/* @return 0 on success, -errno on failure */
	void stop();

	/* Trampoline for the work queue. */
	static void cycle_trampoline(void *arg);

	bool isRunning() {
		return _isRunning;
	}

private:
	void _cycle();
	void _measure();

	bool _shouldExit;bool _isRunning;
	struct work_s _work;
	orb_advert_t _rcinput_pub;
	int _channels;
	//int _ch_fd[input_rc_s::RC_INPUT_MAX_CHANNELS];
	struct input_rc_s _data;

	int rpi_rc_init();
};

static void usage(const char *reason);
static RcInput *rc_input = nullptr;
}
extern "C" __EXPORT int rpi_rc_in_main(int argc, char **argv);
