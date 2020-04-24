/****************************************************************************
 *
 *   Copyright (c) 2017 Fan.zhang. All rights reserved. 421395590@qq.com
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#pragma once

/*! @file rpi_rc_in.h
 * Raspberry Pi driver to publish RC input from shared memory.
 * It requires the ppmdecode program (https://github.com/crossa/raspberry-pi-ppm-rc-in)
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/defines.h>

#include <drivers/drv_hrt.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/input_rc.h>

#define RCINPUT_MEASURE_INTERVAL_US 20000

namespace rpi_rc_in
{
class RcInput : public px4::ScheduledWorkItem
{
public:
	RcInput() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) {}

	~RcInput();

	/** @return 0 on success, -errno on failure */
	int start();

	/** @return 0 on success, -errno on failure */
	void stop();

	bool is_running()
	{
		return _is_running;
	}

private:
	void Run() override;
	void _measure();

	int rpi_rc_init();

	bool _should_exit = false;
	bool _is_running = false;
	uORB::PublicationMulti<input_rc_s>	_rcinput_pub{ORB_ID(input_rc)};
	int _channels = 8; //D8R-II plus
	input_rc_s _data{};

	int *_mem = nullptr;
	key_t _key = 4096; ///< shared memory key (matches the ppmdecode program's key)
	int _shmid = 0;
};

static void usage(const char *reason);
static RcInput *rc_input = nullptr;
}
extern "C" __EXPORT int rpi_rc_in_main(int argc, char **argv);
