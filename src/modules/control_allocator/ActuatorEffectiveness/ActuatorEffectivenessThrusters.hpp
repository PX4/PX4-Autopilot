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
 * @file ActuatorEffectivenessThrusters.hpp
 *
 * Actuator effectiveness computed from thrusters position and orientation
 *
 * @author Pedro Roque, <padr@kth.se>
 */

#pragma once

#include "ActuatorEffectiveness.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

class ActuatorEffectivenessTilts;

using namespace time_literals;

class ActuatorEffectivenessThrusters : public ModuleParams, public ActuatorEffectiveness
{
public:
	enum class AxisConfiguration {
		Configurable, ///< axis can be configured
	};

	static constexpr int NUM_THRUSTERS_MAX = 12;

	struct ThrusterGeometry {
		matrix::Vector3f position;
		matrix::Vector3f axis;
		float thrust_coef;
	};

	struct Geometry {
		ThrusterGeometry thrusters[NUM_THRUSTERS_MAX];
		int num_thrusters{0};
	};

	ActuatorEffectivenessThrusters(ModuleParams *parent, AxisConfiguration axis_config = AxisConfiguration::Configurable,
				       bool tilt_support = false);
	virtual ~ActuatorEffectivenessThrusters() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] =
			AllocationMethod::SEQUENTIAL_DESATURATION;  // TODO(@Jaeyoung-Lim): needs to be updated based on metric mixer
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		// TODO(@Jaeyoung-Lim): needs to be updated based on metric mixer
		normalize[0] = true;
	}

	static int computeEffectivenessMatrix(const Geometry &geometry,
					      EffectivenessMatrix &effectiveness, int actuator_start_index = 0);

	bool addActuators(Configuration &configuration);

	const char *name() const override { return "Thrusters"; }

	const Geometry &geometry() const { return _geometry; }

	uint32_t getThrusters() const;

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
	};
	ParamHandles _param_handles[NUM_THRUSTERS_MAX];
	param_t _count_handle;

	Geometry _geometry{};
};
