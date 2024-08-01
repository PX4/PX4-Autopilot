/****************************************************************************
*
*   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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

#include "input.h"
#include <px4_platform_common/atomic.h>

namespace gimbal
{

class InputTest : public InputBase
{
public:
	InputTest() = delete;
	explicit InputTest(Parameters &parameters);
	virtual ~InputTest() = default;

	UpdateResult update(unsigned int timeout_ms, ControlData &control_data, bool already_active) override;
	int initialize() override;
	void print_status() const override;

	void set_test_input_angles(
		float roll_deg, float pitch_deg, float yaw_deg);
	void set_test_input_angle_rates(float rollrate_deg_s, float pitchrate_deg_s, float yawrate_deg_s);

private:
	float _roll_deg {NAN};
	float _pitch_deg {NAN};
	float _yaw_deg {NAN};
	float _rollrate_deg_s {NAN};
	float _pitchrate_deg_s {NAN};
	float _yawrate_deg_s {NAN};

	px4::atomic<bool> _has_been_set {false};
};


} /* namespace gimbal */
