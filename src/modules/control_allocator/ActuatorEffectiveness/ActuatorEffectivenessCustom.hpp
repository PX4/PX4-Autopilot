/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessCustom.hpp
 *
 * Actuator effectiveness computed from rotors position and orientation
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include "ActuatorEffectiveness.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

using namespace time_literals;

class ActuatorEffectivenessCustom: public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessCustom(ModuleParams *parent);
	virtual ~ActuatorEffectivenessCustom() = default;

	bool getEffectivenessMatrix(matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix_0,
				    matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix_1, bool force) override;

	void setCombinedTilt(float tilt) override;

	int numActuators0() const override { return _acutator_index_0; }
	int numActuators1() const override { return _acutator_index_1; }

	int numActuatorsInMatrix_0() const override { return _num_actuators_0; };
	int numActuatorsInMatrix_1() const override { return _num_actuators_1; };

	int get_actuator_type_0(uint8_t idx) const override {return _actuator_0_type[idx]; }
	int get_actuator_type_1(uint8_t idx) const override {return _actuator_1_type[idx]; }

private:
	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> _effectiveness_0{};
	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> _effectiveness_1{};

	int _num_actuators_0{0};
	int _num_actuators_1{0};

	int _acutator_index_0{0};
	int _acutator_index_1{0};

	int _actuator_0_type [NUM_ACTUATORS] {};
	int _actuator_1_type [NUM_ACTUATORS] {};

	bool _updated{true};

	hrt_abstime _last_effectiveness_update{0};

	int actuatorFunction(uint8_t idx) const { return (int)OutputFunction::Motor1 + idx; }; /// TODO: This is wrong

	bool _is_VTOL{false}; // set to true if both MC motors and either aileron, elevator or rudder is present
	bool _has_MC{false};
};
