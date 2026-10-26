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
	 * @param increase_limit fraction in [0,1] of the upward (thrust-raising) desaturation gain
	 *                       allowed: 0 = no airmode, 1 = full airmode. Only meaningful for the
	 *                       collective-thrust vector, whose entries all share one sign; on a torque
	 *                       axis a negative gain is not an increase and the limit would depend on
	 *                       the sign of the command.
	 */
	void desaturateActuators(ActuatorVector &actuator_sp, const ActuatorVector &desaturation_vector,
				 float increase_limit = 1.f);

	/**
	 * Desaturate along the yaw axis like desaturateActuators(), but only by shrinking the commanded
	 * yaw toward zero: never past zero (yaw reversal) and never away from the command (yaw nobody
	 * asked for, spent to relieve a roll/pitch saturation).
	 *
	 * @param actuator_sp Actuator setpoint, vector that is modified
	 * @param yaw yaw column of the mixing matrix
	 */
	void desaturateYaw(ActuatorVector &actuator_sp, const ActuatorVector &yaw);

	/**
	 * Computes the gain k by which desaturation_vector has to be multiplied
	 * in order to unsaturate the output that has the greatest saturation.
	 *
	 * @return desaturation gain
	 */
	float computeDesaturationGain(const ActuatorVector &desaturation_vector, const ActuatorVector &actuator_sp);

	/**
	 * @return the largest shift along desaturation_vector that any single saturated output needs to
	 *         reach its bound (0 if nothing saturates). Unlike computeDesaturationGain(), saturation
	 *         on opposite bounds adds up instead of cancelling out.
	 */
	float computeWorstSaturation(const ActuatorVector &desaturation_vector, const ActuatorVector &actuator_sp);

	/**
	 * @return true if folding yaw into the pre-thrust mix reduces the worst saturation along the
	 *         thrust axis by more than YAW_FOLD_MIN_IMPROVEMENT, i.e. yaw relieves the saturating
	 *         actuator.
	 */
	bool yawReducesAirmodeThrust();

	/**
	 * Mix roll, pitch, yaw, thrust and set the actuator setpoint.
	 *
	 * @param roll_pitch_limit airmode limit in [0,1]: the collective-thrust increase allowed to keep
	 *                         roll/pitch authority, and yaw authority too with yaw_airmode.
	 * @param yaw_airmode      also spend the roll_pitch_limit budget on yaw. No effect at
	 *                         roll_pitch_limit == 0, which is always airmode disabled.
	 *
	 * Endpoints: (0,false) disabled, (1,false) roll/pitch, (1,true) roll/pitch/yaw.
	 */
	void mix(float roll_pitch_limit, bool yaw_airmode);

	// Minimum reduction of the worst thrust-axis saturation for yawReducesAirmodeThrust() to fold yaw
	// in. Keeps exact ties, whose outcome would otherwise be decided by float rounding, deferred.
	static constexpr float YAW_FOLD_MIN_IMPROVEMENT{1e-3f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_AIRMODE_LIM>) _param_mc_airmode_lim,  ///< airmode thrust increase limit
		(ParamBool<px4::params::MC_AIRMODE_YAW>) _param_mc_airmode_yaw    ///< include yaw in airmode
	);
};
