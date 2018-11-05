/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file CollisionAvoidance.hpp
 * @author Tanja Baumann <tanja@auterion.com>
 *
 * CollisionAvoidance controller.
 *
 */

#pragma once

#include <px4_module_params.h>
#include <float.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/collision_constraints.h>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/Publication.hpp>
using uORB::Publication;
#include <uORB/uORB.h>
#include <systemlib/mavlink_log.h>

using namespace matrix;
using namespace time_literals;

class CollisionAvoidance : public ModuleParams
{
public:
	CollisionAvoidance();

	~CollisionAvoidance() = default;

	void activate() {_is_active = true;}

	void deactivate() {_is_active = false;}

	bool is_active() {return _is_active;}

	bool collision_avoidance_enabled() { return MPC_COL_AVOID.get(); }

	void update(const obstacle_distance_s &distance_measurements);

	void update_range_constraints();

	void reset_constraints();

	void publish_constraints(const Vector2f &original_setpoint, const Vector2f &adapted_setpoint);

	void modifySetpoint(Vector2f &original_setpoint, const float max_speed);

private:

	obstacle_distance_s			_obstacle_distance{};	/**< obstacle distances received form a range sensor */

	bool _is_active = true;
	bool _interfering = false;		/**< states if the collision avoidance interferes with the user input */

	orb_advert_t _constraints_pub{nullptr};		/**< constraints publication */
	orb_advert_t _mavlink_log_pub = nullptr;	 	/**< Mavlink log uORB handle */

	static constexpr uint64_t RANGE_STREAM_TIMEOUT_US = 500000;
	static constexpr uint64_t MESSAGE_THROTTLE_US = 5_s;

	hrt_abstime _last_message;

	Vector2f _move_constraints_x_normalized;
	Vector2f _move_constraints_y_normalized;
	Vector2f _move_constraints_x;
	Vector2f _move_constraints_y;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MPC_COL_AVOID>) MPC_COL_AVOID, /**< use range sensor measurements to avoid collision */
		(ParamFloat<px4::params::MPC_COL_AVOID_D>) MPC_COL_AVOID_D /**< collision avoidance keep minimum distance */
	)

};
