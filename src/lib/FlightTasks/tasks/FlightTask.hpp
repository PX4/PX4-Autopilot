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
 * @file FlightTask.hpp
 *
 * Abstract base class for different advanced flight tasks like orbit, follow me, ...
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <controllib/blocks.hpp>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>


class FlightTask : public control::SuperBlock
{
public:
	FlightTask(SuperBlock *parent, const char *name) :
		SuperBlock(parent, name),
		_sub_vehicle_local_position(ORB_ID(vehicle_local_position), 0, 0, &getSubscriptions())
	{ };
	virtual ~FlightTask() {};

	/**
	 * Call once on the event where you switch to the task
	 * @return 0 on success, >0 on error
	 */
	virtual int activate()
	{
		_starting_time_stamp = hrt_absolute_time();
		FlightTask::update();
		return 0;
	};

	/**
	 * Call once on the event of switching away from the task
	 * @return 0 on success, >0 on error
	 */
	virtual int disable() { return 0; };

	/**
	 * To be called regularly in the control loop cycle to execute the task
	 * @return 0 on success, >0 on error
	 */
	virtual int update();

	/**
	 * Get the output data
	 */
	const vehicle_local_position_setpoint_s &get_position_setpoint()
	{
		return _vehicle_local_position_setpoint;
	};

protected:
	static constexpr uint64_t _timeout = 500000; /*< maximal time in us before a loop or data times out */

	float _time = 0; /**< passed time in seconds since the task was activated */
	float _deltatime = 0; /**< passed time in seconds since the task was last updated */

	/* Current vehicle position for every task */
	matrix::Vector3f _position; /**< current vehicle position */
	matrix::Vector3f _velocity; /**< current vehicle velocity */
	float _yaw = 0.f;

	/**
	 * Put the position vector produced by the task into the setpoint message
	 */
	void _set_position_setpoint(const matrix::Vector3f position_setpoint);

	/**
	 * Put the velocity vector produced by the task into the setpoint message
	 */
	void _set_velocity_setpoint(const matrix::Vector3f velocity_setpoint);

	/**
	 * Put the acceleration vector produced by the task into the setpoint message
	 */
	void _set_acceleration_setpoint(const matrix::Vector3f acceleration_setpoint);

	/**
	 * Put the yaw angle produced by the task into the setpoint message
	 */
	void _set_yaw_setpoint(float yaw) { _vehicle_local_position_setpoint.yaw = yaw; };

private:
	uORB::Subscription<vehicle_local_position_s> _sub_vehicle_local_position;

	hrt_abstime _starting_time_stamp = 0; /**< time stamp when task was activated */
	hrt_abstime _last_time_stamp = 0; /**< time stamp when task was last updated */

	vehicle_local_position_setpoint_s _vehicle_local_position_setpoint; /**< Output position setpoint that every task has */

	void _evaluate_vehicle_position();
};
