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

#include "DifferentialAutoMode.hpp"

using namespace time_literals;

DifferentialAutoMode::DifferentialAutoMode(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_position_setpoint_pub.advertise();
}

void DifferentialAutoMode::updateParams()
{
	ModuleParams::updateParams();
}

void DifferentialAutoMode::autoControl()
{
	if (_position_setpoint_triplet_sub.updated()) {
		position_setpoint_triplet_s position_setpoint_triplet{};
		_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);
		int curr_wp_type = position_setpoint_triplet.current.type;

		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);

		MapProjection global_ned_proj_ref{};

		if (!global_ned_proj_ref.isInitialized()
		    || (global_ned_proj_ref.getProjectionReferenceTimestamp() != vehicle_local_position.ref_timestamp)) {
			global_ned_proj_ref.initReference(vehicle_local_position.ref_lat, vehicle_local_position.ref_lon,
							  vehicle_local_position.ref_timestamp);
		}

		Vector2f curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
		Vector2f curr_wp_ned{NAN, NAN};
		Vector2f prev_wp_ned{NAN, NAN};
		Vector2f next_wp_ned{NAN, NAN};

		RoverControl::globalToLocalSetpointTriplet(curr_wp_ned, prev_wp_ned, next_wp_ned, position_setpoint_triplet,
				curr_pos_ned, global_ned_proj_ref);

		float waypoint_transition_angle = RoverControl::calcWaypointTransitionAngle(prev_wp_ned, curr_wp_ned, next_wp_ned);

		// Waypoint cruising speed
		float cruising_speed = position_setpoint_triplet.current.cruising_speed > 0.f ? math::constrain(
					       position_setpoint_triplet.current.cruising_speed, 0.f, _param_ro_speed_limit.get()) : _param_ro_speed_limit.get();

		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = curr_wp_ned(0);
		rover_position_setpoint.position_ned[1] = curr_wp_ned(1);
		rover_position_setpoint.start_ned[0] = prev_wp_ned(0);
		rover_position_setpoint.start_ned[1] = prev_wp_ned(1);
		rover_position_setpoint.arrival_speed = arrivalSpeed(cruising_speed, waypoint_transition_angle,
							_param_ro_speed_limit.get(), _param_rd_trans_drv_trn.get(), _param_ro_speed_red.get(), curr_wp_type);
		rover_position_setpoint.cruising_speed = cruising_speed;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);
	}
}

float DifferentialAutoMode::arrivalSpeed(const float cruising_speed, const float waypoint_transition_angle,
		const float max_speed, const float trans_drv_trn, const float speed_red, int curr_wp_type)
{
	// Upcoming stop
	if (!PX4_ISFINITE(waypoint_transition_angle) || waypoint_transition_angle < M_PI_F - trans_drv_trn
	    || curr_wp_type == position_setpoint_s::SETPOINT_TYPE_LAND || curr_wp_type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
		return 0.f;
	}

	// Straight line speed
	if (speed_red > FLT_EPSILON) {
		const float speed_reduction = math::constrain(speed_red * math::interpolate(M_PI_F - waypoint_transition_angle,
					      0.f, M_PI_F, 0.f, 1.f), 0.f, 1.f);
		return max_speed * (1.f - speed_reduction);
	}

	return cruising_speed; // Fallthrough

}
