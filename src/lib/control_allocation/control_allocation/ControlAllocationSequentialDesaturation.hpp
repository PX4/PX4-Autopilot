/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationSequentialDesaturation.hpp
 *
 * Control Allocation Algorithm which sequentially modifies control demands in order to
 * eliminate the saturation of the actuator setpoint vector.
 *
 *
 * @author Roman Bapst <bapstroman@gmail.com>
 */

#pragma once

#include "ControlAllocationPseudoInverse.hpp"

#include <px4_platform_common/module_params.h>

class ControlAllocationSequentialDesaturation: public ControlAllocationPseudoInverse, public ModuleParams
{
public:

	ControlAllocationSequentialDesaturation() : ModuleParams(nullptr) {}
	virtual ~ControlAllocationSequentialDesaturation() = default;

	void allocate() override;

	void updateParameters() override;

	// This is the minimum actuator yaw granted when the controller is saturated.
	// In the yaw-only case where outputs are saturated, thrust is reduced by up to this amount.
	static constexpr float MINIMUM_YAW_MARGIN{0.15f};
private:

	/**
	 * Minimize the saturation of the actuators by adding or substracting a fraction of desaturation_vector.
	 * desaturation_vector is the vector that added to the output outputs, modifies the thrust or angular
	 * acceleration on a specific axis.
	 * For example, if desaturation_vector is given to slide along the vertical thrust axis (thrust_scale), the
	 * saturation will be minimized by shifting the vertical thrust setpoint, without changing the
	 * roll/pitch/yaw accelerations.
	 *
	 * Note that as we only slide along the given axis, in extreme cases outputs can still contain values
	 * outside of [min_output, max_output].
	 *
	 * @param actuator_sp Actuator setpoint, vector that is modified
	 * @param desaturation_vector vector that is added to the outputs, e.g. thrust_scale
	 * @param increase_limit fraction in [0,1] of the desaturation gain that is allowed to RAISE
	 *                       the actuator setpoint. 0 forbids any upward adjustment (legacy "no
	 *                       airmode"); 1 allows the full unsaturating gain (legacy full airmode).
	 *                       Intermediate values cap the upward excursion proportionally.
	 */
	void desaturateActuators(ActuatorVector &actuator_sp, const ActuatorVector &desaturation_vector,
				 float increase_limit = 1.f);

	/**
	 * Computes the gain k by which desaturation_vector has to be multiplied
	 * in order to unsaturate the output that has the greatest saturation.
	 *
	 * @return desaturation gain
	 */
	float computeDesaturationGain(const ActuatorVector &desaturation_vector, const ActuatorVector &actuator_sp);

	/**
	 * Decide whether folding the yaw demand into the pre-thrust mix reduces the airmode thrust
	 * bump. When true, yaw is folded into the initial sum so the collective-thrust desaturation
	 * can credit yaw relieving the saturating actuator, instead of deferring yaw until after the
	 * thrust step (which over-adds thrust). Compares the unconstrained thrust desaturation gain
	 * of the roll/pitch-only mix against the roll/pitch+yaw mix.
	 *
	 * @return true if including yaw needs a strictly smaller thrust shift to unsaturate.
	 */
	bool yawReducesAirmodeThrust();

	/**
	 * Mix roll, pitch, yaw, thrust and set the actuator setpoint.
	 *
	 * @param roll_pitch_limit fraction in [0,1] of per-actuator headroom the
	 *                         desaturator may consume to raise thrust in order
	 *                         to preserve roll/pitch authority.
	 * @param yaw_limit        fraction in [0,1] for the analogous yaw budget.
	 *                         When 0, yaw is added after roll/pitch resolution
	 *                         and desaturated against an upper bound inflated
	 *                         by MINIMUM_YAW_MARGIN (legacy deferred-yaw path).
	 *                         When > 0, yaw is mixed into the initial
	 *                         accumulation and the desaturator's upward gain
	 *                         is capped by yaw_limit (legacy RPY path at 1).
	 *
	 * Legacy equivalence: (0, 0) = airmode disabled,
	 *                     (1, 0) = roll/pitch airmode,
	 *                     (1, 1) = roll/pitch/yaw airmode.
	 */
	void mix(float roll_pitch_limit, float yaw_limit);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_AIRMODE_LIM>) _param_mc_airmode_lim,         ///< roll/pitch airmode authority limit
		(ParamFloat<px4::params::MC_AIRMODE_YLIM>) _param_mc_airmode_yaw_lim  ///< yaw airmode authority limit
	);
};
