/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>
// #include "rs_remote_control.hpp"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/drone_task.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/uORB.h>
#include "../rs_motor_control/rs_motor_control.hpp"


using matrix::Eulerf;
using matrix::Quatf;
using matrix::Matrix3f;
using matrix::Vector3f;
using matrix::Dcmf;

using uORB::SubscriptionData;

using namespace time_literals;

class RobosubPosControl: public ModuleBase<RobosubPosControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RobosubPosControl();
	~RobosubPosControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	// PID state
	float _integral = 0.0f;
	float _previous_error = 0.0f;

	// PID gains
	float _Kp = 2.0f;
	float _Ki = 0.5f;
	float _Kd = 0.1f;

	// Constants
	const float SURFACE_PRESSURE = 1.0f;     // bar
	const float METER_PER_BAR = 10.0f;       // 1 bar = 10 meters
	const float SETPOINT = 0.5f;             // meters


	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _drone_task_sub{ORB_ID(drone_task)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	static RobosubMotorControl robosub_motor_control;


	drone_task_s _drone_task{};

	perf_counter_t	_loop_perf;

	enum class AutoMoveState {
		INIT,
		MOVE_FORWARD,
		WAIT_FORWARD,
		MOVE_ORIGIN,
		WAIT_ORIGIN,
		MOVE_BACKWARD,
		WAIT_BACKWARD,
		DONE
	};

	AutoMoveState _auto_move_state{AutoMoveState::INIT};
	matrix::Vector3f _origin_position{};
	hrt_abstime _state_entry_time{0};
	uORB::Publication<trajectory_setpoint_s> trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};

	void Run() override;
	/**
	* Update our local parameter cache.
	*/
	void parameters_update(bool force = false);
	void posControl();
	void AltitudeControl();

};
