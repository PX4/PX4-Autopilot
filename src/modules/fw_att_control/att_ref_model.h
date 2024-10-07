/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include <cstdint>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

#include <drivers/drv_hrt.h>

#include <px4_platform_common/module_params.h>

#include <lib/mathlib/math/filter/second_order_reference_model.hpp>
#include <lib/slew_rate/SlewRateYaw.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

using namespace time_literals;

class AttitudeReferenceModel : public math::SecondOrderReferenceModel<float>
{
public:
	AttitudeReferenceModel()
	{
		setDiscretizationMethod(math::SecondOrderReferenceModel<float>::DiscretizationMethod::kForwardEuler);
	};

	/**
	 * Set the system parameters
	 *
	 * Calculates the damping coefficient, spring constant, and maximum allowed
	 * time step based on the natural frequency.
	 *
	 * @param[in] natural_freq The desired undamped natural frequency of the system [rad/s]
	 * @param[in] damping_ratio The desired damping ratio of the system
	 * @param[in] vel_limit the limit for the velocity [rad/s]
	 * @param[in] acc_limit the limit for the acceleration [rad/s^2]
	 * @param[in] jerk_limit set maximum jerk [rad/s^3]. Optional, only used in kForwardEuler mode
	 * @return Whether or not the param set was successful
	 */
	bool setParameters(const float natural_freq, const float damping_ratio, float vel_limit = INFINITY,
			   float acc_limit = INFINITY)
	{
		if (acc_limit < FLT_EPSILON) {
			acc_limit_ = INFINITY;

		} else {
			acc_limit_ = acc_limit;
		}

		if (vel_limit < FLT_EPSILON) {
			vel_limit_ = INFINITY;

		} else {
			vel_limit_ = vel_limit;
		}

		return math::SecondOrderReferenceModel<float>::setParameters(natural_freq, damping_ratio);
	}

private:
	float vel_limit_; 	/** velocity limit [rad/s]*/
	float acc_limit_; 	/** acceleration limit [rad/s^2]*/

	/**
	 * Take one integration step using Euler-forward integration
	 *
	 * @param[in] time_step Integration time [s]
	 * @param[in] state_sample [rad]
	 * @param[in] rate_sample [rad/s]
	 */
	void integrateStatesForwardEuler(const float time_step, const float &state_sample, const float &rate_sample) override
	{
		filter_accel_ = calculateInstantaneousAcceleration(state_sample, rate_sample);

		if (filter_accel_ > 0.f) {
			filter_accel_ = math::min(math::min(filter_accel_, (vel_limit_ - filter_rate_) / time_step), acc_limit_);

		} else {
			filter_accel_ = math::max(math::max(filter_accel_, (-vel_limit_ - filter_rate_) / time_step), -acc_limit_);
		}

		const float new_rate = filter_rate_ + filter_accel_ * time_step;
		const float new_state = filter_state_ + filter_rate_ * time_step;

		filter_state_ = new_state;
		filter_rate_ = new_rate;
	}
};

class FixedwingAttitudeReferenceModel : public ModuleParams
{
public:
	FixedwingAttitudeReferenceModel();
	~FixedwingAttitudeReferenceModel() = default;

	void update();

	void reset();

	const vehicle_attitude_setpoint_s &getOutput() {return _attitude_setpoint_output;};

	const matrix::Vector2f &getRateFeedforward() {return _attitiude_rate_feedforward_output;};
	const matrix::Vector2f &getTorqueFeedforward() {return _attitiude_torque_feedforward_output;};

private:

	enum class ModelMode {
		MODELMODE_THROUGHPUT,
		MODELMODE_FILTERING
	} _mode{ModelMode::MODELMODE_THROUGHPUT}; /*< Mode in which the attitude reference model works on */

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();

	AttitudeReferenceModel _roll_ref_model;  /*< Second order reference filter for the roll angle */
	AttitudeReferenceModel _pitch_ref_model; /*< Second order reference filter for the pitch angle */
	bool _is_initialized{false}; /*< Flag indicating if the reference model is already initialized */
	uint64_t _last_att_setpoint_timestamp{UINT64_C(0)}; /*< Timestamp of the last own published vehicle attitude setpoint topic */
	hrt_abstime _last_update_timestamp{0U}; 	/*< Timestamp of the last update*/
	vehicle_attitude_setpoint_s _last_attitude_reference_setpoint; /*< Last attitude reference setpoint input */
	vehicle_attitude_setpoint_s _attitude_setpoint_output; /*< Attitude setpoint to be set by output*/
	matrix::Vector2f 	_attitiude_rate_feedforward_output; /*< Attitude rate feedforward output */
	matrix::Vector2f 	_attitiude_torque_feedforward_output; /*< Attitude torque feedforward output */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; /*< parameter update subscription */
	uORB::SubscriptionData<vehicle_attitude_setpoint_s> _att_ref_sp_sub{ORB_ID(vehicle_attitude_reference_setpoint)};	/*< vehicle attitude reference setpoint */
	uORB::SubscriptionData<vehicle_attitude_setpoint_s> _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};	/*< vehicle attitude setpoint */
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)}; /*< vehicle attitude setpoint publication when reference model is active for logging */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_REF_R_FREQ>) _param_ref_r_freq,
		(ParamFloat<px4::params::FW_REF_R_V_LIM>) _param_ref_r_vel_limit,
		(ParamFloat<px4::params::FW_REF_R_A_LIM>) _param_ref_r_acc_limit,
		(ParamBool<px4::params::FW_REF_R_EN>) _param_ref_r_en,
		(ParamFloat<px4::params::FW_REF_P_FREQ>) _param_ref_p_freq,
		(ParamFloat<px4::params::FW_REF_P_V_LIM>) _param_ref_p_vel_limit,
		(ParamFloat<px4::params::FW_REF_P_A_LIM>) _param_ref_p_acc_limit,
		(ParamBool<px4::params::FW_REF_P_EN>) _param_ref_p_en
	)
};
