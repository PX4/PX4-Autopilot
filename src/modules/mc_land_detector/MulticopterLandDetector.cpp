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
 * @file MultiCopterLandDetector.cpp
 * Land detection algorithm for multicopters
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 */

#include "MulticopterLandDetector.h"

#include <stdio.h>
#include <cmath>
#include <drivers/drv_hrt.h>
#include <unistd.h>                 //usleep

MulticopterLandDetector::MulticopterLandDetector() :
    _landDetectedPub(-1),
    _landDetected({0,false}),
    
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

    _taskShouldExit(false),
    _taskIsRunning(false),
    _landTimer(0)
{
    //Advertise the first land detected uORB
    _landDetected.timestamp = hrt_absolute_time();
    _landDetected.landed = false;
    _landDetectedPub = orb_advertise(ORB_ID(vehicle_land_detected), &_landDetected);
}

MulticopterLandDetector::~MulticopterLandDetector()
{
    _taskShouldExit = true;
    close(_landDetectedPub);
}

/**
* @brief Convinience function for polling uORB subscriptions
* @return true if there was new data and it was successfully copied
**/
static bool orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
    bool newData = false;

    //Check if there is new data to grab
    if(orb_check(handle, &newData) != OK) {
        return false;
    }

    if(!newData) {
        return false;
    }

    if(orb_copy(meta, handle, buffer) != OK) {
        return false;
    }

    return true;
}

void MulticopterLandDetector::shutdown()
{
    _taskShouldExit = true;
}

void MulticopterLandDetector::updateSubscriptions()
{
    orb_update(ORB_ID(vehicle_global_position), _vehicleGlobalPositionSub, &_vehicleGlobalPosition);
    orb_update(ORB_ID(sensor_combined), _sensorsCombinedSub, &_sensors);
    orb_update(ORB_ID(position_setpoint_triplet), _waypointSub, &_waypoint);
    orb_update(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, _actuatorsSub, &_actuators);
    orb_update(ORB_ID(actuator_armed), _armingSub, &_arming);
}

void MulticopterLandDetector::landDetectorLoop()
{
    //This should never happen!
    if(_taskIsRunning) return;

    //Subscribe to position, attitude, arming and velocity changes
    _vehicleGlobalPositionSub = orb_subscribe(ORB_ID(vehicle_global_position));
    _sensorsCombinedSub = orb_subscribe(ORB_ID(sensor_combined));
    _waypointSub = orb_subscribe(ORB_ID(position_setpoint_triplet));
    _actuatorsSub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
    _armingSub = orb_subscribe(ORB_ID(actuator_armed));

    //Begin task
    _taskIsRunning = true;
    _taskShouldExit = false;
    while (!_taskShouldExit) {

        //First poll for new data from our subscriptions
        updateSubscriptions();

        const uint64_t now = hrt_absolute_time();

        //only detect landing if the autopilot is actively trying to land
        if(!_waypoint.current.valid || _waypoint.current.type != SETPOINT_TYPE_LAND) {
            _landTimer = now;
        }
        else {

            //Check if we are moving vertically
            bool verticalMovement = fabsf(_vehicleGlobalPosition.vel_d) > MC_LAND_DETECTOR_CLIMBRATE_MAX;

            //Check if we are moving horizontally
            bool horizontalMovement = sqrtf(_vehicleGlobalPosition.vel_n*_vehicleGlobalPosition.vel_n 
                + _vehicleGlobalPosition.vel_e*_vehicleGlobalPosition.vel_e) > MC_LAND_DETECTOR_VELOCITY_MAX;

            //Next look if all rotation angles are not moving
            bool rotating = sqrtf(_sensors.gyro_rad_s[0]*_sensors.gyro_rad_s[0]+
                          _sensors.gyro_rad_s[1]*_sensors.gyro_rad_s[1]+
                          _sensors.gyro_rad_s[2]*_sensors.gyro_rad_s[2]) > MC_LAND_DETECTOR_ROTATION_MAX;

            //Check if thrust output is minimal (about half of default)
            bool minimalThrust = _arming.armed && _actuators.control[3] <= MC_LAND_DETECTOR_THRUST_MAX;

            if(verticalMovement || rotating || !minimalThrust || horizontalMovement) {
                //Sensed movement, so reset the land detector
                _landTimer = now;
            }

        }

        // if we have detected a landing for 2 continuous seconds
        if(now-_landTimer > MC_LAND_DETECTOR_TRIGGER_TIME) {
            if(!_landDetected.landed)
            {
                _landDetected.timestamp = now;
                _landDetected.landed = true;

                /* publish the land detected broadcast */
                orb_publish(ORB_ID(vehicle_land_detected), _landDetectedPub, &_landDetected);
            }
        }
        else {
            // if we currently think we have landed, but the latest data says we are flying
            if(_landDetected.landed)
            {
                _landDetected.timestamp = now;
                _landDetected.landed = false;

                /* publish the land detected broadcast */
                orb_publish(ORB_ID(vehicle_land_detected), _landDetectedPub, &_landDetected);
            }
        }

        //Limit loop rate
        usleep(1000000 / MC_LAND_DETECTOR_UPDATE_RATE);
    }

    _taskIsRunning = false;
    _exit(0);
}

bool MulticopterLandDetector::isRunning() const
{
    return _taskIsRunning;
}
