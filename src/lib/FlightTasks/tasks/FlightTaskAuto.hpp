/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAuto.hpp
 *
 * Map from global triplet to local quadruple.
 *
 */

#pragma once

#include "FlightTask.hpp"
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>

/* This enum has to agree with position_setpoint_s type definition */
enum class WaypointType : int {
	position = 0,
	velocity,
	loiter,
	takeoff,
	land,
	idle
};

class FlightTaskAuto : public FlightTask
{
public:
	FlightTaskAuto(control::SuperBlock *parent, const char *name);

	virtual ~FlightTaskAuto() = default;

	bool initializeSubscriptions(SubscriptionArray &subscription_array) override;

	bool activate() override;

	bool updateInitialize() override;

protected:

	matrix::Vector3f _prev_prev_wp{}; /**< Triplet previous-previous triplet. This will be used for smoothing trajectories -> not used yet. */
	matrix::Vector3f _prev_wp{}; /**< Triplet previous setpoint in local frame. If not previous triplet is available, the prev_wp is set to current position. */
	matrix::Vector3f _target{}; /**< Triplet target setpoint in local frame. */
	matrix::Vector3f _next_wp{}; /**< Triplet setpoint in local frame. If no next setpoint is available, next is set to target. */
	float _yaw_wp{0.0f}; /**< Triplet yaw waypoint. Unfortunately navigator sends yaw setpoint continuously. It would be better if a yaw setpoint is attached
	to triplet waypoint. This way it would be easy for multicopter to implement features where yaw does not matter. */
	float _mc_cruise_speed{0.0f}; /**< Cruise speed with which multicopter flies and gets set by triplet. If no valid, default cruise speed is used. */
	WaypointType _type{WaypointType::idle}; /**< Type of current target triplet. */

private:
	control::BlockParamFloat _mc_cruise_default; /**< Default mc cruise speed*/
	map_projection_reference_s _reference; /**< Reference frame from global to local */
	uORB::Subscription<position_setpoint_triplet_s> *_sub_triplet_setpoint{nullptr};
	map_projection_reference_s _reference_position{}; /**< Structure used to project lat/lon setpoint into local frame */
	float _reference_altitude = 0.0f;  /**< Altitude relative to ground */
	hrt_abstime _time_stamp_reference = 0; /**< time stamp when last reference update */

	bool _evaluateTriplets(); /**< Checks and sets triplets */
	bool _isFinite(const position_setpoint_s sp);
	void _updateReference();

	bool _evaluateVehiclePosition() override; /**< Required for reference update */
};
