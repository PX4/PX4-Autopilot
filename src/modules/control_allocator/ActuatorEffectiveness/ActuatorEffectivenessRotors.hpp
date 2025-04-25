/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessRotors.hpp
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

class ActuatorEffectivenessTilts;

using namespace time_literals;

class ActuatorEffectivenessRotors : public ModuleParams, public ActuatorEffectiveness
{
public:
	enum class AxisConfiguration {
		Configurable, ///< axis can be configured
		FixedForward, ///< axis is fixed, pointing forwards (positive X)
		FixedUpwards, ///< axis is fixed, pointing upwards (negative Z)
	};

	static constexpr int NUM_ROTORS_MAX = 12;

	struct RotorGeometry {
		matrix::Vector3f position;
		matrix::Vector3f axis;
		float thrust_coef;
		float moment_ratio;
		int tilt_index;
	};

	struct Geometry {
		RotorGeometry rotors[NUM_ROTORS_MAX];
		int num_rotors{0};
		bool propeller_torque_disabled{false};
		bool propeller_torque_disabled_non_upwards{false}; ///< keeps propeller torque enabled for upward facing motors
	};

	ActuatorEffectivenessRotors(ModuleParams *parent, AxisConfiguration axis_config = AxisConfiguration::Configurable,
				    bool tilt_support = false);
	virtual ~ActuatorEffectivenessRotors() = default;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	static int computeEffectivenessMatrix(const Geometry &geometry,
					      EffectivenessMatrix &effectiveness, int actuator_start_index = 0);

	bool addActuators(Configuration &configuration);

	const char *name() const override { return "Multirotor"; }

	/**
	 * Sets the motor axis from tilt configurations and current tilt control.
	 * @param tilts configured tilt servos
	 * @param tilt_control current tilt control in [-1, 1] (can be NAN)
	 * @return the motors as bitset which are not tiltable
	 */
	uint32_t updateAxisFromTilts(const ActuatorEffectivenessTilts &tilts, float tilt_control);

	const Geometry &geometry() const { return _geometry; }

	/**
	 * Get the tilted axis {0, 0, -1} rotated by -tilt_angle around y, then
	 * rotated by tilt_direction around z.
	 */
	static matrix::Vector3f tiltedAxis(float tilt_angle, float tilt_direction);

	void enablePropellerTorque(bool enable) { _geometry.propeller_torque_disabled = !enable; }

	void enablePropellerTorqueNonUpwards(bool enable) { _geometry.propeller_torque_disabled_non_upwards = !enable; }

	uint32_t getUpwardsMotors() const;

private:
	void updateParams() override;
	const AxisConfiguration _axis_config;
	const bool _tilt_support; ///< if true, tilt servo assignment params are loaded

	struct ParamHandles {
		param_t position_x;
		param_t position_y;
		param_t position_z;
		param_t axis_x;
		param_t axis_y;
		param_t axis_z;
		param_t thrust_coef;
		param_t moment_ratio;
		param_t tilt_index;
	};
	ParamHandles _param_handles[NUM_ROTORS_MAX];
	param_t _count_handle;

	Geometry _geometry{};
};
