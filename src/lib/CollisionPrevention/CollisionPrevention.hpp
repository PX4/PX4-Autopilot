/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file CollisionPrevention.hpp
 * @author Tanja Baumann <tanja@auterion.com>
 *
 * CollisionPrevention controller.
 *
 */

#pragma once

#include <px4_module_params.h>
#include <float.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/collision_constraints.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <systemlib/mavlink_log.h>
#include <commander/px4_custom_mode.h>

class CollisionPrevention : public ModuleParams
{
public:
	CollisionPrevention(ModuleParams *parent);

	~CollisionPrevention();

	bool is_active() { return _param_mpc_col_prev_d.get() > 0; }

	void modifySetpoint(matrix::Vector2f &original_setpoint, const float max_speed,
			    const matrix::Vector2f &curr_pos, const matrix::Vector2f &curr_vel);

private:

	bool _interfering{false};		/**< states if the collision prevention interferes with the user input */

	orb_advert_t _constraints_pub{nullptr};  	/**< constraints publication */
	orb_advert_t _mavlink_log_pub{nullptr};	 	/**< Mavlink log uORB handle */
	orb_advert_t _obstacle_distance_pub{nullptr}; /**< obstacle_distance publication */
	orb_advert_t _pub_vehicle_command{nullptr}; /**< vehicle command do publication */

	uORB::SubscriptionData<obstacle_distance_s> _sub_obstacle_distance{ORB_ID(obstacle_distance)}; /**< obstacle distances received form a range sensor */
	uORB::Subscription _sub_distance_sensor[ORB_MULTI_MAX_INSTANCES] {{ORB_ID(distance_sensor), 0}, {ORB_ID(distance_sensor), 1}, {ORB_ID(distance_sensor), 2}, {ORB_ID(distance_sensor), 3}}; /**< distance data received from onboard rangefinders */
	uORB::SubscriptionData<vehicle_attitude_s> _sub_vehicle_attitude{ORB_ID(vehicle_attitude)};

	static constexpr uint64_t RANGE_STREAM_TIMEOUT_US{500000};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_COL_PREV_D>) _param_mpc_col_prev_d, /**< collision prevention keep minimum distance */
		(ParamFloat<px4::params::MPC_XY_P>) _param_mpc_xy_p, /**< p gain from position controller*/
		(ParamFloat<px4::params::MPC_COL_PREV_DLY>) _param_mpc_col_prev_dly /**< delay of the range measurement data*/
	)

	void update();

	inline float sensorOrientationToYawOffset(const distance_sensor_s &distance_sensor, float angle_offset)
	{

		float offset = angle_offset > 0.f ? math::radians(angle_offset) : 0.0f;

		switch (distance_sensor.orientation) {
		case distance_sensor_s::ROTATION_RIGHT_FACING:
			offset = M_PI_F / 2.0f;
			break;

		case distance_sensor_s::ROTATION_LEFT_FACING:
			offset = -M_PI_F / 2.0f;
			break;

		case distance_sensor_s::ROTATION_BACKWARD_FACING:
			offset = M_PI_F;
			break;

		case distance_sensor_s::ROTATION_CUSTOM:
			offset = matrix::Eulerf(matrix::Quatf(distance_sensor.q)).psi();
			break;
		}

		return offset;
	}

	void calculateConstrainedSetpoint(matrix::Vector2f &setpoint, const matrix::Vector2f &curr_pos,
					  const matrix::Vector2f &curr_vel);

	void publishConstrainedSetpoint(const matrix::Vector2f &original_setpoint, const matrix::Vector2f &adapted_setpoint);
	void publishObstacleDistance(obstacle_distance_s &obstacle);

	void updateOffboardObstacleDistance(obstacle_distance_s &obstacle);
	void updateDistanceSensor(obstacle_distance_s &obstacle);
	void _publishVehicleCmdDoLoiter();

};
