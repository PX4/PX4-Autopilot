/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessRotorsINDI.hpp
 *
 * Actuator effectivness for model based INDI rate control
 * Modified from ActuatorEffectivenessRotorsINDI.hpp
 *
 * @author Rohan Inamdar <rninamdar@wpi.edu>
 */

#pragma once

#include <control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

class ActuatorEffectivenessTilts;

using namespace time_literals;
using namespace matrix;

class ActuatorEffectivenessRotorsINDI : public ModuleParams, public ActuatorEffectiveness
{
public:
	enum class AxisConfiguration {
		Configurable, ///< axis can be configured
		FixedForward, ///< axis is fixed, pointing forwards (positive X)
		FixedUpwards, ///< axis is fixed, pointing upwards (negative Z)
	};

	struct RotorGeometry {
		Vector3f position;
		Vector3f axis;
		float thrust_coef;
		float moment_ratio;
		float moment_inertia;
	};

	struct Geometry {
		RotorGeometry rotors[NUM_ACTUATORS];
		int num_rotors{0};
		bool propeller_torque_disabled{false};
		bool yaw_by_differential_thrust_disabled{false};
		bool propeller_torque_disabled_non_upwards{false}; ///< keeps propeller torque enabled for upward facing motors
	};

	ActuatorEffectivenessRotorsINDI(ModuleParams *parent, AxisConfiguration axis_config = AxisConfiguration::Configurable);
	virtual ~ActuatorEffectivenessRotorsINDI() = default;

	bool initializeEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update);

	void adaptEffectivenessMatrix(Configuration &configuration, Vector<float, NUM_ACTUATORS> &delta_motor_speeds, Vector<float, NUM_ACTUATORS> &delta_dot_motor_speeds, Vector3f &filtered_angular_accel);

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	static int computeEffectivenessMatrix(const Geometry &geometry,
					      EffectivenessMatrix &G1, EffectivenessMatrix &G2, int actuator_start_index = 0);

	Slice<float, 3, NUM_ACTUATORS, NUM_AXES, NUM_ACTUATORS> getG1(Configuration &configuration) const {return configuration.effectiveness_matrices[0].slice<3, NUM_ACTUATORS>(3, 0);}
	Slice<float, 3, NUM_ACTUATORS, NUM_AXES, NUM_ACTUATORS> getG2(Configuration &configuration) const {return configuration.effectiveness_matrices[1].slice<3, NUM_ACTUATORS>(3, 0);}

	bool addActuators(Configuration &configuration);

	const char *name() const override { return "RotorsINDI"; }

	const Geometry &geometry() const { return _geometry; }

	uint32_t getMotors() const;
	uint32_t getUpwardsMotors() const;
	uint32_t getForwardsMotors() const;

private:
	void updateParams() override;
	const AxisConfiguration _axis_config;

	struct ParamHandles {
		param_t position_x;
		param_t position_y;
		param_t position_z;
		param_t axis_x;
		param_t axis_y;
		param_t axis_z;
		param_t thrust_coef;
		param_t moment_ratio;
		param_t moment_inertia;
	};
	ParamHandles _param_handles[NUM_ACTUATORS];

	// for adaptive effectiveness matrix from paper: "Adaptive Incremental nonlinear Dynamic Inversion for Attitude Contol of Micro Air Vehicles" by Smeur et al
	// these are the adaptive constants for the G1 and G2 matrices, and the apative constants per axis
	// these are the constants that are used to adapt the effectiveness matrix to the current motor speeds and angular accelerations to account for changes in the system (weigth changes, battery voltage changes, etc.)
	// mu1 is split between the two matricies: g1_adaptive_constants and g2_adaptive_constants
	// mu2 = apative_constants_per_axis
	// larger values = fasater adaptation rate but too large values can cause instability
	// tuning is required to find the optimal values
	SquareMatrix<float, NUM_ACTUATORS> _G1_adaptive_constants;
	SquareMatrix<float, NUM_ACTUATORS> _G2_adaptive_constants;

	SquareMatrix<float, 3> _adaptive_constants_per_axis = diag(Vector3f(0.2f, 0.2f, 0.2f));

	Geometry _geometry{};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MET_ROTOR_COUNT>) _param_met_rotor_count
	)
};
