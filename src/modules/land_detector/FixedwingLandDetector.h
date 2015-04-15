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
 * @file FixedwingLandDetector.h
 * Land detection algorithm for fixedwing
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 */

#ifndef __FIXED_WING_LAND_DETECTOR_H__
#define __FIXED_WING_LAND_DETECTOR_H__

#include "LandDetector.h"
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>

class FixedwingLandDetector : public LandDetector
{
public:
	FixedwingLandDetector();

protected:
	/**
	* @brief  blocking loop, should be run in a separate thread or task. Runs at 50Hz
	**/
	bool update() override;

	/**
	* @brief Initializes the land detection algorithm
	**/
	void initialize() override;

	/**
	* @brief  polls all subscriptions and pulls any data that has changed
	**/
	void updateSubscriptions();

private:
	/**
	* @brief download and update local parameter cache
	**/
	void updateParameterCache(const bool force);

	/**
	* @brief Handles for interesting parameters
	**/
	struct {
		param_t maxVelocity;
		param_t maxClimbRate;
		param_t maxAirSpeed;
	}		_paramHandle;

	struct {
		float maxVelocity;
		float maxClimbRate;
		float maxAirSpeed;
	} _params;

private:
	int                                     _vehicleLocalPositionSub;   /**< notification of local position */
	struct vehicle_local_position_s         _vehicleLocalPosition;      /**< the result from local position subscription */
	int                                     _airspeedSub;
	struct airspeed_s                       _airspeed;
	int 									_parameterSub;

	float _velocity_xy_filtered;
	float _velocity_z_filtered;
	float _airspeed_filtered;
	uint64_t _landDetectTrigger;
};

#endif //__FIXED_WING_LAND_DETECTOR_H__
