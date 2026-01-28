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

/**
 * @file RpmControl.hpp
 *
 * Control rpm of a helicopter rotor.
 * Input: PWM input pulse period from an rpm sensor
 * Output: Duty cycle command for the ESC
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <lib/pid/PID.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rpm.h>

class RpmControl : public ModuleParams
{
public:
	RpmControl(ModuleParams *parent);
	~RpmControl() = default;

	void setSpoolupProgress(float spoolup_progress);
	float getActuatorCorrection();

private:
	static constexpr float SPOOLUP_PROGRESS_WITH_CONTROLLER_ENGAGED = .8f; // [0,1]
	static constexpr float PID_OUTPUT_LIMIT = .5f; // [0,1]

	uORB::Subscription _rpm_sub{ORB_ID(rpm)};
	bool _rpm_invalid{true};
	PID _pid;
	float _spoolup_progress{0.f}; // [0,1]
	hrt_abstime _timestamp_last_measurement{0}; // for dt and timeout
	float _actuator_correction{0.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_HELI_RPM_SP>) _param_ca_heli_rpm_sp,
		(ParamFloat<px4::params::CA_HELI_RPM_P>) _param_ca_heli_rpm_p,
		(ParamFloat<px4::params::CA_HELI_RPM_I>) _param_ca_heli_rpm_i
	)
};
