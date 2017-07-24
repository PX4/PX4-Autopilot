
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
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>     /*Unix 标准函数定义*/
#include <fcntl.h>      /*文件控制定义*/
#include <sys/ioctl.h>
#include <asm-generic/termbits.h>
#include <errno.h>      /*错误号定义*/
#include <px4_config.h>
#include <px4_workqueue.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>
#define RCINPUT_MEASURE_INTERVAL_US 4700
/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

namespace linux_sbus {
class RcInput {
public:
	RcInput() :
			_shouldExit(false), _isRunning(false), _work { }, _rcinput_pub(
					nullptr), //D8R-II plus
			_data { }, _sbusData { 0x0f, 0x01, 0x04, 0x20, 0x00, 0xff, 0x07,
					0x40, 0x00, 0x02, 0x10, 0x80, 0x2c, 0x64, 0x21, 0x0b, 0x59,
					0x08, 0x40, 0x00, 0x02, 0x10, 0x80, 0x00, 0x00 } {
		//memset(_ch_fd, 0, sizeof(_ch_fd));
	}
	~RcInput() {
		work_cancel(HPWORK, &_work);
		_isRunning = false;
		close(_device_fd);
	}

	/* @return 0 on success, -errno on failure */
	int start(char *device, int channels);

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
	bool _shouldExit;
	bool _isRunning;
	struct work_s _work;
	orb_advert_t _rcinput_pub;
	struct input_rc_s _data;
	uint8_t _sbusData[25];
	int _channels;
	int _device_fd;  // serial port device to read SBUS;
	int _channels_data[16]; // 16 channels support;
	uint8_t _buffer[25];
	char _device[30];
	bool _failsafe;
	bool _rc_loss;
	int init();
};

static void usage(const char *reason);
static RcInput *rc_input = nullptr;
}
extern "C" __EXPORT int linux_sbus_main(int argc, char **argv);
