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
 * @file MulticopterLandDetector.h
 * Land detection algorithm for multicopters
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 */

#ifndef __MULTICOPTER_LAND_DETECTOR_H__
#define __MULTICOPTER_LAND_DETECTOR_H__

#include "LandDetector.h"
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>

class MulticopterLandDetector : public LandDetector
{
public:
	MulticopterLandDetector();

protected:
	/**
	* @brief  polls all subscriptions and pulls any data that has changed
	**/
	void updateSubscriptions();

	/**
	* @brief Runs one iteration of the land detection algorithm
	**/
	bool update() override;

	/**
	* @brief Initializes the land detection algorithm
	**/
	void initialize() override;


private:
	/**
	* @brief download and update local parameter cache
	**/
	void updateParameterCache(const bool force);

	/**
	* @brief Handles for interesting parameters
	**/
	struct {
		param_t maxClimbRate;
		param_t maxVelocity;
		param_t maxRotation;
		param_t maxThrottle;
	}		_paramHandle;

	struct {
		float maxClimbRate;
		float maxVelocity;
		float maxRotation;
		float maxThrottle;
	} _params;

private:
	int _vehicleGlobalPositionSub;                                      /**< notification of global position */
	int _vehicleStatusSub;
	int _actuatorsSub;
	int _armingSub;
	int _parameterSub;
    int _attitudeSub;

	struct vehicle_global_position_s          _vehicleGlobalPosition;   /**< the result from global position subscription */
	struct vehicle_status_s 			      _vehicleStatus;
	struct actuator_controls_s                _actuators;
	struct actuator_armed_s                   _arming;
    struct vehicle_attitude_s                 _vehicleAttitude;

	uint64_t _landTimer;                                                /**< timestamp in microseconds since a possible land was detected*/
};

#endif //__MULTICOPTER_LAND_DETECTOR_H__
