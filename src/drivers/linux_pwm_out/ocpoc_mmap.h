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

#pragma once

#include "common.h"

namespace linux_pwm_out
{

/**
 ** class OcpocMmapPWMOut
 * PWM output class for Aerotenna OcPoC via mmap
 */
class OcpocMmapPWMOut : public PWMOutBase
{
public:
	OcpocMmapPWMOut(int max_num_outputs);
	virtual ~OcpocMmapPWMOut();

	int init() override;

	int send_output_pwm(const uint16_t *pwm, int num_outputs) override;

private:
	static unsigned long freq2tick(uint16_t freq_hz);

	static constexpr int MAX_ZYNQ_PWMS = 8;	/**< maximum number of pwm channels */

	// Period|Hi 32 bits each
	struct s_period_hi {
		uint32_t period;
		uint32_t hi;
	};

	struct pwm_cmd {
		struct s_period_hi periodhi[MAX_ZYNQ_PWMS];
	};

	volatile struct pwm_cmd *_shared_mem_cmd = nullptr;
	static constexpr const char *_device = "/dev/mem";
	int _num_outputs;
};

}
