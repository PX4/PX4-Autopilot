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
#include "rpi_rc_in.h"

using namespace rpi_rc_in;
//---------------------------------------------------------------------------------------------------------//
int RcInput::rpi_rc_init() {
	int i;
	//--------------初始化共享内存映射----------------------------//
	if ((this->shmid = shmget(this->key, sizeof(int) * this->_channels, 0666))
			< 0) {
		PX4_WARN("无法访问共享内存");
		return -1;
	}

	if ((this->mem = (int*) shmat(this->shmid, NULL, 0)) == (void *) -1) {
		PX4_WARN("无法映射共享内存");
		return -1;
	}

	//--------------发布所有通道的数据------------------------//
	for (i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; ++i) {
		_data.values[i] = UINT16_MAX;
	}

	_rcinput_pub = orb_advertise(ORB_ID(input_rc), &_data);

	if (_rcinput_pub == nullptr) {
		PX4_WARN("error: advertise failed");
		return -1;
	}

	return 0;
}
//---------------------------------------------------------------------------------------------------------//
int RcInput::start() {
	int result = 0;

	result = rpi_rc_init();

	if (result != 0) {
		PX4_WARN("error: RC initialization failed");
		return -1;
	}

	_isRunning = true;
	result = work_queue(HPWORK, &_work, (worker_t) & RcInput::cycle_trampoline,
			this, 0);

	if (result == -1) {
		_isRunning = false;
	}

	return result;
}
//---------------------------------------------------------------------------------------------------------//
void RcInput::stop() {
	_shouldExit = true;
}
//---------------------------------------------------------------------------------------------------------//
void RcInput::cycle_trampoline(void *arg) {
	RcInput *dev = reinterpret_cast<RcInput *>(arg);
	dev->_cycle();
}
//---------------------------------------------------------------------------------------------------------//
void RcInput::_cycle() {
	_measure();

	if (!_shouldExit) {
		work_queue(HPWORK, &_work, (worker_t) & RcInput::cycle_trampoline, this,
				USEC2TICK(RCINPUT_MEASURE_INTERVAL_US));
	}
}
//---------------------------------------------------------------------------------------------------------//
void RcInput::_measure(void) {
	uint64_t ts;
	// PWM  数据发布
	int i=0;
	for(i=0;i<_channels;++i){
		_data.values[i] = *(this->mem+i);
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
//---------------------------------------------------------------------------------------------------------//
/**
 * Print the correct usage.
 */

static void rpi_rc_in::usage(const char *reason) {
	if (reason) {
		PX4_ERR("%s", reason);
	}
	PX4_INFO("用法: rpi_rc_in {start|stop|status}");
}
//---------------------------------------------------------------------------------------------------------//
int rpi_rc_in_main(int argc, char **argv) {
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (rc_input != nullptr && rc_input->isRunning()) {
			PX4_WARN("运行中。");
			/* this is not an error */
			return 0;
		}

		rc_input = new RcInput();

		// Check if alloc worked.
		if (nullptr == rc_input) {
			PX4_ERR("遥控输入模块初始化错误。");
			return -1;
		}

		int ret = rc_input->start();

		if (ret != 0) {
			PX4_ERR("遥控输入模块未能启动");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {

		if (rc_input == nullptr || !rc_input->isRunning()) {
			PX4_WARN("模块未运行");
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
			PX4_INFO("运行中");

		} else {
			PX4_INFO("未运行\n");
		}

		return 0;
	}

	usage("不知道你要做什么");
	return 1;

}
//---------------------------------------------------------------------------------------------------------//
