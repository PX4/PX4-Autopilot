/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "VehicleAngularVelocity.hpp"

#include <px4_log.h>

using namespace time_literals;
using namespace matrix;

VehicleAngularVelocity::VehicleAngularVelocity() :
	ModuleParams(nullptr),
	WorkItem(px4::wq_configurations::rate_ctrl),
	_cycle_perf(perf_alloc(PC_ELAPSED, "vehicle_angular_velocity: cycle time")),
	_sensor_gyro_latency_perf(perf_alloc(PC_ELAPSED, "vehicle_angular_velocity: sensor gyro latency"))
{
	_gyro_count = math::constrain(orb_group_count(ORB_ID(sensor_gyro)), 1, MAX_GYRO_COUNT);

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}

	parameters_updated();
}

VehicleAngularVelocity::~VehicleAngularVelocity()
{
	perf_free(_cycle_perf);
	perf_free(_sensor_gyro_latency_perf);
}

bool
VehicleAngularVelocity::init()
{
	_sensor_correction_sub.register_callback();

	return selected_gyro_update();
}

void
VehicleAngularVelocity::parameters_updated()
{
	/* get transformation matrix from sensor/board to body frame */
	_board_rotation = get_rot_matrix((enum Rotation)_param_sens_board_rot.get());

	/* fine tune the rotation */
	Dcmf board_rotation_offset(Eulerf(
					   M_DEG_TO_RAD_F * _param_sens_board_x_off.get(),
					   M_DEG_TO_RAD_F * _param_sens_board_y_off.get(),
					   M_DEG_TO_RAD_F * _param_sens_board_z_off.get()));

	_board_rotation = board_rotation_offset * _board_rotation;
}

void
VehicleAngularVelocity::parameter_update_poll()
{
	/* Check if parameters have changed */
	parameter_update_s param_update;

	if (_params_sub.update(&param_update)) {
		updateParams();
		parameters_updated();
	}
}

bool
VehicleAngularVelocity::selected_gyro_update()
{
	// check if the selected gyro has updated first
	_sensor_correction_sub.update(&_sensor_correction);

	/* update the latest gyro selection */
	if (_selected_gyro != _sensor_correction.selected_gyro_instance) {
		if (_sensor_correction.selected_gyro_instance < _gyro_count) {
			// clear all registered callbacks
			for (auto sub : _sensor_gyro_sub) {
				sub.unregister_callback();
			}

			const int gyro_new = _sensor_correction.selected_gyro_instance;

			if (_sensor_gyro_sub[gyro_new].register_callback()) {
				PX4_WARN("selected gyro changed %d -> %d", _selected_gyro, gyro_new);
				_selected_gyro = gyro_new;

				return true;
			}
		}
	}

	return false;
}

void
VehicleAngularVelocity::Run()
{
	if (should_exit()) {
		// clear all registered callbacks
		for (auto sub : _sensor_gyro_sub) {
			sub.unregister_callback();
		}

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	parameter_update_poll();
	selected_gyro_update();

	_sensor_bias_sub.update(&_sensor_bias);

	sensor_gyro_s gyro;

	if (_sensor_gyro_sub[_selected_gyro].update(&gyro)) {
		perf_set_elapsed(_sensor_gyro_latency_perf, hrt_elapsed_time(&gyro.timestamp));

		// get the raw gyro data and correct for thermal errors
		Vector3f rates;

		if (_selected_gyro == 0) {
			rates(0) = (gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
			rates(1) = (gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
			rates(2) = (gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

		} else if (_selected_gyro == 1) {
			rates(0) = (gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
			rates(1) = (gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
			rates(2) = (gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

		} else if (_selected_gyro == 2) {
			rates(0) = (gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
			rates(1) = (gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
			rates(2) = (gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

		} else {
			rates(0) = gyro.x;
			rates(1) = gyro.y;
			rates(2) = gyro.z;
		}

		// rotate corrected measurements from sensor to body frame
		rates = _board_rotation * rates;

		// correct for in-run bias errors
		rates(0) -= _sensor_bias.gyro_x_bias;
		rates(1) -= _sensor_bias.gyro_y_bias;
		rates(2) -= _sensor_bias.gyro_z_bias;

		vehicle_angular_velocity_s vrates{};
		vrates.timestamp_sample = gyro.timestamp;
		vrates.rollspeed = rates(0);
		vrates.pitchspeed = rates(1);
		vrates.yawspeed = rates(2);
		vrates.timestamp = hrt_absolute_time();

		_vehicle_angular_velocity_pub.publish(vrates);
	}

	perf_end(_cycle_perf);
}

int VehicleAngularVelocity::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VehicleAngularVelocity::print_status()
{
	PX4_INFO("Running");

	PX4_INFO("selected gyro: %d", _selected_gyro);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_sensor_gyro_latency_perf);

	return 0;
}

int
VehicleAngularVelocity::task_spawn(int argc, char *argv[])
{
	VehicleAngularVelocity *instance = new VehicleAngularVelocity();

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

int VehicleAngularVelocity::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module does the RC input parsing and auto-selecting the method. Supported methods are:

### Implementation
By default the module runs on the work queue, to reduce RAM usage. It can also be run in its own thread,
specified via start flag -t, to reduce latency.
When running on the work queue, it schedules at a fixed frequency.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_input", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task (without any mode set, use any of the mode_* cmds)");
	PRINT_MODULE_USAGE_PARAM_FLAG('t', "Run as separate task instead of the work queue", true);

#if defined(SPEKTRUM_POWER)
	PRINT_MODULE_USAGE_COMMAND_DESCR("bind", "Send a DSM bind command (module must be running)");
#endif /* SPEKTRUM_POWER */

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int vehicle_angular_velocity_main(int argc, char *argv[]);

int
vehicle_angular_velocity_main(int argc, char *argv[])
{
	return VehicleAngularVelocity::main(argc, argv);
}
