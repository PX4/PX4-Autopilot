/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "offboard_switch.hpp"

using namespace time_literals;
using namespace matrix;

OffboardSwitch::OffboardSwitch() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();
}

OffboardSwitch::~OffboardSwitch()
{
	perf_free(_loop_perf);
}

bool
OffboardSwitch::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
OffboardSwitch::parameters_update()
{

	return PX4_OK;
}

// void OffboardSwitch::trajectorySetpointCheck() {
// 	if ()
// }

void OffboardSwitch::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if angular velocity changed

	///TODO: Timeout if setpoints fall out of sync
	///TODO: Package offboard control mode messages to the correct level, depending on the setpoint
	offboard_control_mode_s ocm{};
	trajectory_setpoint_s trajectory_setpoint;

	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);

	bool data_is_recent{false};

	if (_offboard_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
		data_is_recent = data_is_recent || (hrt_absolute_time() < trajectory_setpoint.timestamp
						    + static_cast<hrt_abstime>(_param_com_of_loss_t.get() * 1_s));

		if (data_is_recent) {
			if (PX4_ISFINITE(trajectory_setpoint.position[0]) && PX4_ISFINITE(trajectory_setpoint.position[1])
			    && PX4_ISFINITE(trajectory_setpoint.position[2])) {
				ocm.position = true;
			}

			if (PX4_ISFINITE(trajectory_setpoint.velocity[0]) && PX4_ISFINITE(trajectory_setpoint.velocity[1])
			    && PX4_ISFINITE(trajectory_setpoint.velocity[2])) {
				ocm.velocity = true;
			}

			if (PX4_ISFINITE(trajectory_setpoint.acceleration[0]) && PX4_ISFINITE(trajectory_setpoint.acceleration[1])
			    && PX4_ISFINITE(trajectory_setpoint.acceleration[2])) {
				ocm.acceleration = true;
			}

			if ((ocm.position || ocm.velocity || ocm.acceleration)
			    && (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD)) {
				_trajectory_setpoint_pub.publish(trajectory_setpoint);
			}
		}
	}

	vehicle_attitude_setpoint_s attitude_setpoint;

	if (_offboard_vehicle_attitude_setpoint_sub.update(&attitude_setpoint)) {
		data_is_recent = data_is_recent || (hrt_absolute_time() < attitude_setpoint.timestamp
						    + static_cast<hrt_abstime>(_param_com_of_loss_t.get() * 1_s));
		PX4_INFO("Data is recent: %f", double(data_is_recent));

		if (data_is_recent) {
			if (PX4_ISFINITE(attitude_setpoint.q_d[0]) && PX4_ISFINITE(attitude_setpoint.q_d[1])
			    && PX4_ISFINITE(attitude_setpoint.q_d[2]) && PX4_ISFINITE(attitude_setpoint.q_d[3])) {
				ocm.attitude = true;
			}

			if (ocm.attitude
			    && (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD)) {
				_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
			}
		}
	}

	vehicle_rates_setpoint_s rates_setpoint;

	if (_offboard_vehicle_rates_setpoint_sub.update(&rates_setpoint)) {
		data_is_recent = data_is_recent || (hrt_absolute_time() < rates_setpoint.timestamp
						    + static_cast<hrt_abstime>(_param_com_of_loss_t.get() * 1_s));

		if (data_is_recent) {
			if (PX4_ISFINITE(rates_setpoint.roll) && PX4_ISFINITE(rates_setpoint.pitch)
			    && PX4_ISFINITE(rates_setpoint.yaw)) {
				ocm.body_rate = true;
			}

			if (ocm.body_rate
			    && (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD)) {
				_vehicle_rates_setpoint_pub.publish(rates_setpoint);
			}
		}
	}

	///TODO: Handle corner cases where ill-posed setpoints are mixed
	bool valid_position_setpoint = ocm.position || ocm.velocity || ocm.acceleration;
	bool valid_attitude_setpoint = ocm.attitude || ocm.body_rate;
	bool valid_actuator_setpooint = ocm.thrust_and_torque || ocm.direct_actuator;

	if (data_is_recent && (valid_position_setpoint || valid_attitude_setpoint || valid_actuator_setpooint)) {
		// publish offboard_control_mode
		ocm.timestamp = hrt_absolute_time();
		_offboard_control_mode_pub.publish(ocm);
	}

	perf_end(_loop_perf);
}

int OffboardSwitch::task_spawn(int argc, char *argv[])
{
	OffboardSwitch *instance = new OffboardSwitch();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int OffboardSwitch::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int OffboardSwitch::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
offboard_switch filters and regulated offboard setpoints.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("offboard_switch", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int offboard_switch_main(int argc, char *argv[])
{
	return OffboardSwitch::main(argc, argv);
}
