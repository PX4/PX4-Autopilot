/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file MulticopterLandDetector.cpp
 * Land detection algorithm
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 */

#include "MulticopterLandDetector.h"

#include <cmath>
#include <drivers/drv_hrt.h>

MulticopterLandDetector::MulticopterLandDetector() : LandDetector(),
	_vehicleGlobalPositionSub(-1),
	_sensorsCombinedSub(-1),
	_waypointSub(-1),
	_actuatorsSub(-1),
	_armingSub(-1),

	_vehicleGlobalPosition({}),
	_sensors({}),
	_waypoint({}),
	_actuators({}),
	_arming({}),

	_landTimer(0)
{
	//ctor
}

void MulticopterLandDetector::initialize()
{
	//Subscribe to position, attitude, arming and velocity changes
	_vehicleGlobalPositionSub = orb_subscribe(ORB_ID(vehicle_global_position));
	_sensorsCombinedSub = orb_subscribe(ORB_ID(sensor_combined));
	_waypointSub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_actuatorsSub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	_armingSub = orb_subscribe(ORB_ID(actuator_armed));
}

void MulticopterLandDetector::updateSubscriptions()
{
	orb_update(ORB_ID(vehicle_global_position), _vehicleGlobalPositionSub, &_vehicleGlobalPosition);
	orb_update(ORB_ID(sensor_combined), _sensorsCombinedSub, &_sensors);
	orb_update(ORB_ID(position_setpoint_triplet), _waypointSub, &_waypoint);
	orb_update(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, _actuatorsSub, &_actuators);
	orb_update(ORB_ID(actuator_armed), _armingSub, &_arming);
}

bool MulticopterLandDetector::update()
{
	//First poll for new data from our subscriptions
	updateSubscriptions();

	//Only trigger flight conditions if we are armed
	if(!_arming.armed) {
		return true;
	}

	const uint64_t now = hrt_absolute_time();

	//Check if we are moving vertically
	bool verticalMovement = fabsf(_vehicleGlobalPosition.vel_d) > MC_LAND_DETECTOR_CLIMBRATE_MAX;

	//Check if we are moving horizontally
	bool horizontalMovement = sqrtf(_vehicleGlobalPosition.vel_n * _vehicleGlobalPosition.vel_n
					+ _vehicleGlobalPosition.vel_e * _vehicleGlobalPosition.vel_e) > MC_LAND_DETECTOR_VELOCITY_MAX;

	//Next look if all rotation angles are not moving
	bool rotating = sqrtf(_sensors.gyro_rad_s[0] * _sensors.gyro_rad_s[0] +
			      _sensors.gyro_rad_s[1] * _sensors.gyro_rad_s[1] +
			      _sensors.gyro_rad_s[2] * _sensors.gyro_rad_s[2]) > MC_LAND_DETECTOR_ROTATION_MAX;

	//Check if thrust output is minimal (about half of default)
	bool minimalThrust = _actuators.control[3] <= MC_LAND_DETECTOR_THRUST_MAX;

	if (verticalMovement || rotating || !minimalThrust || horizontalMovement) {
		//Sensed movement, so reset the land detector
		_landTimer = now;
		return false;
	}

	return now - _landTimer > MC_LAND_DETECTOR_TRIGGER_TIME;
}
