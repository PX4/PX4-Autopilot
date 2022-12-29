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

#include "DragEstimator.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>

DragEstimator::DragEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl) // TODO: Edit
{
}

DragEstimator::~DragEstimator()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool DragEstimator::init()
{
	// execute Run() on every _vehicle_attitude_setpoint publication
	if (!_vehicle_acceleration_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude_setpoint callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(10000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void DragEstimator::ResetFilterParams()
{
	// Update the filter cutoff, and reset the filter to the last output
	_lp_filter1.set_cutoff_frequency(_param_de_cutoff.get());
	_lp_filter2.set_cutoff_frequency(_param_de_cutoff.get());
	_lp_filter1.reset(_drag_acc_filtered);
	_lp_filter2.reset(_drag_acc_filtered);
}

void DragEstimator::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
		ResetFilterParams();
	}

	// Only run if vehicle_attitude is updated
	// Note - we use this as if it hasn't been updated after startup then quaternion will be zero
	// Hence the maths generates NaNs
	if (_vehicle_acceleration_sub.update(&_vehicle_acceleration)) {

		//Update data
		_vehicle_attitude_sub.update(&_vehicle_attitude);
		_vehicle_attitude_setpoint_sub.update(&_vehicle_attitude_setpoint);
		hover_thrust_estimate_s hover{};

		if (_hover_thrust_estimate_sub.update(&hover)) {
			_hover_thrust = math::max(0.1f, hover.hover_thrust);
		}

		// Work out delta time since last update for filter
		hrt_abstime acc_timestamp = _vehicle_acceleration.timestamp_sample;
		float dt = float(acc_timestamp - _timestamp_prev) * 1e-6f;
		_timestamp_prev = acc_timestamp;

		// Update rate warnings - disabled for now but needs attention if we wish to move back to Filter2p
		// if (dt > 0) {
		// 	float freq = 1.f / dt;

		// 	if (fabs(_filter_sample_freq - freq) > 20.0f) {
		// 		_filter_sample_freq = freq;
		// 		hrt_abstime now = hrt_absolute_time();

		// 		if (now - _last_warning_time > 3_s) {
		// 			mavlink_log_info(&_mavlink_log_pub, "Drag Estimator input rate %f Hz", double(_filter_sample_freq));
		// 			_last_warning_time = now;
		// 		}
		// 	}
		// }

		// Body attitude
		const float &qw = _vehicle_attitude.q[0];
		const float &qx = _vehicle_attitude.q[1];
		const float &qy = _vehicle_attitude.q[2];
		const float &qz = _vehicle_attitude.q[3];
		Quatf att_quat(qw, qx, qy, qz);

		// Acceleration from accelerometer (using vehicle_acceleration as this has bias and filter applied)
		const float &ax = _vehicle_acceleration.xyz[0];
		const float &ay = _vehicle_acceleration.xyz[1];
		const float &az = _vehicle_acceleration.xyz[2];
		Vector3f acc_measured_body(ax, ay, az);
		Vector3f acc_measured = att_quat.rotateVector(acc_measured_body);

		// Expected acceleration from thrust (note thrust_body[0] and thrust_body[1] are always zero for a multicopter)
		const float &acc_expected_x = 0.f;
		const float &acc_expected_y = 0.f;
		const float &acc_expected_z = (_vehicle_attitude_setpoint.thrust_body[2]) * 9.81f / _hover_thrust;
		Vector3f acc_expected_body(acc_expected_x, acc_expected_y, acc_expected_z);
		Vector3f acc_expected = att_quat.rotateVector(acc_expected_body);

		// Subtract expected acceleration from measured acceleration to estimate drag acceleration.
		// (assuming measured = expected + drag)
		Vector3f drag_acc = acc_measured - acc_expected;

		// Force the above to zero when landed or maybe_landed
		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		if (_landed || _maybe_landed) {
			drag_acc = Vector3f(0.f, 0.f, 0.f);
		}

		// Check that things are not NAN before filtering, so as to avoid the filter becoming NAN
		Vector3f drag_acc_filtered_body;
		Vector3f drag_acc_moment_body;

		if (PX4_ISFINITE(drag_acc(0)) && PX4_ISFINITE(drag_acc(1)) && PX4_ISFINITE(drag_acc(2))) {
			// Filter the drag acceleration
			_drag_acc_filtered = _lp_filter2.apply(_lp_filter1.apply(drag_acc, dt), dt);

			// Convert drag_acceleration_filtered back into body fram
			drag_acc_filtered_body = att_quat.rotateVectorInverse(_drag_acc_filtered);

			// Cross product with Centre of Pressure (CoP) offset z to get moment acting on Centre of Gravity (CoG) in body frame.
			// We use a fixed 0.1m offset above the CoG
			// This can then be scaled accordingly with the gain in the rate controller
			drag_acc_moment_body = drag_acc_filtered_body.cross(Vector3f(0.f, 0.f, _param_de_z_offset.get()));

		} else {
			// Set all outputs to zero if any NANs
			_drag_acc_filtered = Vector3f(0.f, 0.f, 0.f);
			drag_acc_filtered_body = Vector3f(0.f, 0.f, 0.f);
			drag_acc_moment_body = Vector3f(0.f, 0.f, 0.f);
		}

		// Populate drag for publishing
		drag_estimator_s drag{};
		acc_measured.copyTo(drag.acc_measured);
		acc_expected.copyTo(drag.acc_expected);
		drag_acc.copyTo(drag.drag_acc);
		_drag_acc_filtered.copyTo(drag.drag_acc_filtered);
		drag_acc_filtered_body.copyTo(drag.drag_acc_filtered_body);
		drag_acc_moment_body.copyTo(drag.drag_acceleration_moment_body);
		drag.timestamp = hrt_absolute_time();
		_drag_estimator_pub.publish(drag);
	}

	perf_end(_loop_perf);
}

int DragEstimator::task_spawn(int argc, char *argv[])
{
	DragEstimator *instance = new DragEstimator();

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


int DragEstimator::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}


int DragEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int DragEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.
This is a template for a module running as a task in the background with start/stop/status functionality.
### Implementation
Section describing the high-level implementation of this module.
### Examples
CLI usage example:
$ module start -f -p 42
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("DragEstimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int drag_estimator_main(int argc, char *argv[])
{
	return DragEstimator::main(argc, argv);
}
