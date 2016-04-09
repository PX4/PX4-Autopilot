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
 * @file VtolLandDetector.h
 * Land detection algorithm for vtol
 *
 * @author Roman Bapst <bapstr@gmail.com>
 */

#ifndef __VTOL_LAND_DETECTOR_H__
#define __VTOL_LAND_DETECTOR_H__

#include "MulticopterLandDetector.h"
#include <uORB/topics/airspeed.h>

class VtolLandDetector : public MulticopterLandDetector
{
public:
	VtolLandDetector();


private:
	/**
	* @brief  polls all subscriptions and pulls any data that has changed
	**/
	void updateSubscriptions() override;

	/**
	* @brief Runs one iteration of the land detection algorithm
	**/
	bool update() override;

	/**
	* @brief Initializes the land detection algorithm
	**/
	void initialize() override;

	/**
	* @brief download and update local parameter cache
	**/
	void updateParameterCache(const bool force) override;

	/**
	* @brief Handles for interesting parameters
	**/
	struct {
		param_t maxAirSpeed;
	}		_paramHandle;

	struct {
		float maxAirSpeed;
	} _params;

	int _airspeedSub;
	int _parameterSub;

	struct airspeed_s _airspeed;

	bool _was_in_air; /**< indicates whether the vehicle was in the air in the previous iteration */
	float _airspeed_filtered; /**< low pass filtered airspeed */
};

#endif
