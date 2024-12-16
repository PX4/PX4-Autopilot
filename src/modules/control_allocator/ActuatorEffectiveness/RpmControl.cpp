/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "RpmControl.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

RpmControl::RpmControl(ModuleParams *parent) : ModuleParams(parent)
{
	_pid.setOutputLimit(PID_OUTPUT_LIMIT);
	_pid.setIntegralLimit(PID_OUTPUT_LIMIT);
};

void RpmControl::setSpoolupProgress(float spoolup_progress)
{
	_spoolup_progress = spoolup_progress;
	_pid.setSetpoint(_spoolup_progress * _param_ca_heli_rpm_sp.get());

	if (_spoolup_progress < SPOOLUP_PROGRESS_WITH_CONTROLLER_ENGAGED) {
		_pid.resetIntegral();
	}
}

float RpmControl::getActuatorCorrection()
{
	hrt_abstime now = hrt_absolute_time();

	// RPM measurement update
	if (_rpm_sub.updated()) {
		rpm_s rpm{};

		if (_rpm_sub.copy(&rpm)) {
			const float dt = math::min((now - _timestamp_last_measurement) * 1e-6f, 1.f);
			_timestamp_last_measurement = rpm.timestamp;

			const float gain_scale = math::interpolate(_spoolup_progress, .8f, 1.f, 0.f, 1e-3f);
			_pid.setGains(_param_ca_heli_rpm_p.get() * gain_scale, _param_ca_heli_rpm_i.get() * gain_scale, 0.f);
			_actuator_correction = _pid.update(rpm.rpm_estimate, dt, true);
		}
	}

	// Timeout
	if (now > _timestamp_last_measurement + 1_s) {
		_pid.resetIntegral();
		_actuator_correction = 0.f;
	}

	return _actuator_correction;
}
