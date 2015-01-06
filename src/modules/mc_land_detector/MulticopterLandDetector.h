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
 * @file MultiCopterLandDetector.h
 * Land detection algorithm for multicopters
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <Morten@mycptr.com>
 */

#ifndef __MULTICOPTER_LAND_DETECTOR_H__
#define __MULTICOPTER_LAND_DETECTOR_H__

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

class MulticopterLandDetector {
public:
        MulticopterLandDetector();
        ~MulticopterLandDetector();

        /**
        * @brief Executes the main loop of the land detector in a separate deamon thread
        * @returns OK if task was successfully launched
        **/
        int createDeamonThread();

        /**
        * @returns true if this task is currently running
        **/
        bool isRunning() const;

        /**
        * @brief  blocking loop, should be run in a separate thread or task. Runs at 50Hz
        **/
        void landDetectorLoop();

        /**
        * @brief  Tells the Land Detector task that it should exit
        **/
        void shutdown();

protected:
        /**
        * @brief  polls all subscriptions and pulls any data that has changed
        **/
        void updateSubscriptions();

        //Algorithm parameters (TODO: should probably be externalized)
        static constexpr float MC_LAND_DETECTOR_CLIMBRATE_MAX = 0.30f;      /**< max +- climb rate in m/s */
        static constexpr float MC_LAND_DETECTOR_ROTATION_MAX = 0.5f;        /**< max rotation in rad/sec (= 30 deg/s) */
        static constexpr float MC_LAND_DETECTOR_THRUST_MAX = 0.2f;
        static constexpr float MC_LAND_DETECTOR_VELOCITY_MAX = 1.0f;        /**< max +- horizontal movement in m/s */
        static constexpr uint32_t MC_LAND_DETECTOR_UPDATE_RATE = 50;        /**< Run algorithm at 50Hz */
        static constexpr uint32_t MC_LAND_DETECTOR_TRIGGER_TIME = 2000000;  /**< usec that landing conditions have to hold before triggering a land */

private:
        orb_advert_t                            _landDetectedPub;           /**< publisher for position in local frame */
        struct vehicle_land_detected_s          _landDetected;              /**< local vehicle position */

        int _vehicleGlobalPositionSub;                                      /**< notification of global position */
        int _sensorsCombinedSub;
        int _waypointSub;
        int _actuatorsSub;
        int _armingSub;

        struct vehicle_global_position_s          _vehicleGlobalPosition;   /**< the result from global position subscription */
        struct sensor_combined_s                  _sensors;                 /**< subscribe to sensor readings */
        struct position_setpoint_triplet_s        _waypoint;                /**< subscribe to autopilot navigation */
        struct actuator_controls_s                _actuators;
        struct actuator_armed_s                   _arming;
        
        bool _taskShouldExit;                                               /**< true if it is requested that this task should exit */
        bool _taskIsRunning;                                                /**< task has reached main loop and is currently running */

        uint64_t _landTimer;                                                /**< timestamp in microseconds since a possible land was detected*/
};

#endif //__MULTICOPTER_LAND_DETECTOR_H__
