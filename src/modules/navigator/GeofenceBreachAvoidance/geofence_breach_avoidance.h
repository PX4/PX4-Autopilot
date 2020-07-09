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

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>
#include "../geofence.h"
#include <px4_platform_common/defines.h>


class Geofence;

union geofence_violation_type_u {
	struct {
		bool dist_to_home_exceeded: 1;	///< 0 - distance to home exceeded
		bool max_altitude_exceeded: 1;	///< 1 - maximum altitude exceeded
		bool fence_violation: 1;	///< 2- violation of user defined fence
	} flags;
	uint8_t value;
};

class GeofenceBreachAvoidance : public ModuleParams
{
public:
	GeofenceBreachAvoidance();

	~GeofenceBreachAvoidance();

	matrix::Vector2<double> waypointFromBearingAndDistance(matrix::Vector2<double> current_pos_lat_lon,
			float test_point_bearing, float test_point_distance);

	matrix::Vector2<double> getFenceViolationTestPoint();

	matrix::Vector2<double>
	generateLoiterPointForFixedWing(geofence_violation_type_u violation_type, Geofence *geofence);

	float computeBrakingDistanceMultirotor();

	float computeVerticalBrakingDistanceMultirotor();

	matrix::Vector2<double> generateLoiterPointForMultirotor(geofence_violation_type_u violation_type, Geofence *geofence);

	float generateLoiterAltitudeForFixedWing(geofence_violation_type_u violation_type);

	float generateLoiterAltitudeForMulticopter(geofence_violation_type_u violation_type);

	float getMinDistToFenceMulticopter() {return _min_hor_dist_to_fence_mc;}

	float getMinVertDistToFenceMultirotor() {return _min_vert_dist_to_fence_mc;}

	void setTestPointBearing(float test_point_bearing);

	void setTestPointDistance(float test_point_distance);

	void setVerticalTestPointDistance(float distance);

	void setHorizontalVelocity(float velocity_hor_abs);

	void setClimbRate(float climbrate);

	void setCurrentPosition(double lat, double lon, float alt);

	void updateParameters();


private:
	struct {
		param_t param_mpc_jerk_max;
		param_t param_mpc_acc_hor;
		param_t param_mpc_acc_hor_max;
		param_t param_mpc_jerk_auto;
		param_t param_mpc_acc_up_max;
		param_t param_mpc_acc_down_max;

	} _paramHandle;

	struct {
		float param_mpc_jerk_max;
		float param_mpc_acc_hor;
		float param_mpc_acc_hor_max;
		float param_mpc_jerk_auto;
		float param_mpc_acc_up_max;
		float param_mpc_acc_down_max;

	} _params;

	float _test_point_bearing{0.0f};
	float _test_point_distance{0.0f};
	float _vertical_test_point_distance{0.0f};
	float _velocity_hor_abs{0.0f};
	float _climbrate{0.0f};
	float _current_alt_amsl{0.0f};
	float _min_hor_dist_to_fence_mc{0.0f};
	float _min_vert_dist_to_fence_mc{0.0f};

	float _multirotor_braking_distance{0.0f};
	float _multirotor_vertical_braking_distance{0.0f};

	matrix::Vector2<double> _current_pos_lat_lon{};

	void updateMinHorDistToFenceMultirotor();

	void updateMinVertDistToFenceMultirotor();

};