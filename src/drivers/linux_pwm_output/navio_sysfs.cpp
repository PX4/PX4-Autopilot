/****************************************************************************
 *
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

#include "navio_sysfs.h"

#include <fcntl.h>
#include <cerrno>
#include <unistd.h>
#include <px4_platform_common/log.h>

using namespace linux_pwm_output;

NavioSysfsPWMOut::~NavioSysfsPWMOut()
{
	for (int i = 0; i < MAX_NUM_PWM; ++i) {
		if (_pwm_fd[i] != -1) {
			::close(_pwm_fd[i]);
		}
	}
}

int NavioSysfsPWMOut::init()
{
	int i;
	char path[128];

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s/export", _device);

		if (pwm_write_sysfs(path, i) < 0) {
			PX4_ERR("PWM export failed");
		}
	}

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s/pwm%u/enable", _device, i);

		if (pwm_write_sysfs(path, 1) < 0) {
			PX4_ERR("PWM enable failed");
		}
	}

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s/pwm%u/period", _device, i);

		if (pwm_write_sysfs(path, (int)1e9 / FREQUENCY_PWM)) {
			PX4_ERR("PWM period failed");
		}
	}

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s/pwm%u/duty_cycle", _device, i);
		_pwm_fd[i] = ::open(path, O_WRONLY | O_CLOEXEC);

		if (_pwm_fd[i] == -1) {
			PX4_ERR("PWM: Failed to open duty_cycle.");
			return -errno;
		}
	}

	return 0;
}

int NavioSysfsPWMOut::send_output_pwm(const uint16_t *pwm, int num_outputs)
{
	char data[16];

	if (num_outputs > _pwm_num) {
		num_outputs = _pwm_num;
	}

	int ret = 0;

	//convert this to duty_cycle in ns
	for (int i = 0; i < num_outputs; ++i) {
		int n = ::snprintf(data, sizeof(data), "%u", pwm[i] * 1000);
		int write_ret = ::write(_pwm_fd[i], data, n);

		if (n != write_ret) {
			ret = -1;
		}
	}

	return ret;
}

int NavioSysfsPWMOut::pwm_write_sysfs(char *path, int value)
{
	int fd = ::open(path, O_WRONLY | O_CLOEXEC);
	int n;
	char data[16];

	if (fd == -1) {
		return -errno;
	}

	n = ::snprintf(data, sizeof(data), "%u", value);

	if (n > 0) {
		n = ::write(fd, data, n);	// This n is not used, but to avoid a compiler error.
	}

	::close(fd);

	return 0;
}

int NavioSysfsPWMOut::FREQUENCY_PWM = 400;

int NavioSysfsPWMOut::deviceConfigure(int argc, char **argv)
{
	int ret = 0;
	int ch;
	optind = 1;
	opterr = 0; // turn off parse error output

	while ((ch = getopt(argc, argv, "d:")) != -1) {
		switch (ch) {
		case 'd':
			strncpy(_device, optarg, 64);
			break;

		case 'n':
			_pwm_num = atoi(optarg);
			_pwm_num = _pwm_num > MAX_NUM_PWM ? MAX_NUM_PWM : _pwm_num;
			break;

		case '?':
			PX4_WARN("Unsupported arg: %c", optopt);
			ret = -EINVAL;

		default:
			break;
		}
	}

	return ret;
}

int NavioSysfsPWMOut::deviceInit()
{
	return init();
}

int NavioSysfsPWMOut::deviceDeinit()
{
	return 0;
}

int NavioSysfsPWMOut::updatePWM(const uint16_t *outputs, unsigned num_outputs)
{
	return send_output_pwm(outputs, num_outputs);
}

int NavioSysfsPWMOut::setFreq(int freq)
{
	FREQUENCY_PWM = freq;
	//init(); // TODO should update pwm frequency here?
	return 0;
}