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

	//Algorithm parameters (TODO: should probably be externalized)
	static constexpr uint64_t FW_LAND_DETECTOR_TRIGGER_TIME = 2000000;  /**< usec that landing conditions have to hold
                                                                             before triggering a land */
	static constexpr float FW_LAND_DETECTOR_VELOCITY_MAX = 5.0f;        /**< maximum horizontal movement m/s*/
	static constexpr float FW_LAND_DETECTOR_CLIMBRATE_MAX = 10.00f;     /**< +- climb rate in m/s*/
	static constexpr float FW_LAND_DETECTOR_AIRSPEED_MAX = 10.00f;      /**< airspeed max m/s*/

private:
	int                                     _vehicleLocalPositionSub;   /**< notification of local position */
	struct vehicle_local_position_s         _vehicleLocalPosition;      /**< the result from local position subscription */
	int                                     _airspeedSub;
	struct airspeed_s                       _airspeed;

	float _velocity_xy_filtered;
	float _velocity_z_filtered;
	float _airspeed_filtered;
	uint64_t _landDetectTrigger;
};

#endif //__FIXED_WING_LAND_DETECTOR_H__
