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

#include "ocpoc_mmap.h"

#include <px4_platform_common/log.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

using namespace linux_pwm_out;

#define RCOUT_ZYNQ_PWM_BASE	    0x43c00000
static const int TICK_PER_US   =  50;
static const int FREQUENCY_PWM = 400;
static const int TICK_PER_S  = 50000000;

OcpocMmapPWMOut::OcpocMmapPWMOut(int max_num_outputs)
{
	_num_outputs = max_num_outputs;

	if (_num_outputs > MAX_ZYNQ_PWMS) {
		PX4_WARN("number of outputs too large. Setting to %i", MAX_ZYNQ_PWMS);
		_num_outputs = MAX_ZYNQ_PWMS;
	}
}

OcpocMmapPWMOut::~OcpocMmapPWMOut()
{
	if (_shared_mem_cmd) {
		munmap((void *)_shared_mem_cmd, 0x1000);
	}
}

unsigned long OcpocMmapPWMOut::freq2tick(uint16_t freq_hz)
{
	unsigned long duty = TICK_PER_S / (unsigned long)freq_hz;
	return duty;
}

int OcpocMmapPWMOut::init()
{
	uint32_t mem_fd = open(_device, O_RDWR | O_SYNC);
	_shared_mem_cmd = (struct pwm_cmd *) mmap(0, 0x1000, PROT_READ | PROT_WRITE,
			  MAP_SHARED, mem_fd, RCOUT_ZYNQ_PWM_BASE);
	close(mem_fd);

	if (_shared_mem_cmd == nullptr) {
		PX4_ERR("initialize pwm pointer failed.");
		return -1;
	}

	for (int i = 0; i < _num_outputs; ++i) {
		_shared_mem_cmd->periodhi[i].period   =  freq2tick(FREQUENCY_PWM);
		_shared_mem_cmd->periodhi[i].hi = freq2tick(FREQUENCY_PWM) / 2;
		PX4_DEBUG("Output values: %d, %d", _shared_mem_cmd->periodhi[i].period, _shared_mem_cmd->periodhi[i].hi);
	}

	return 0;
}

int OcpocMmapPWMOut::send_output_pwm(const uint16_t *pwm, int num_outputs)
{
	if (num_outputs > _num_outputs) {
		num_outputs = _num_outputs;
	}

	//convert this to duty_cycle in ns
	for (int i = 0; i < num_outputs; ++i) {
		//n = ::asprintf(&data, "%u", pwm[i] * 1000);
		//::write(_pwm_fd[i], data, n);
		_shared_mem_cmd->periodhi[i].hi = TICK_PER_US * pwm[i];
		//printf("ch:%d, val:%d*%d ", ch, period_us, TICK_PER_US);
	}

	return 0;
}

