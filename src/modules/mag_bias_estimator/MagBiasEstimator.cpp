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

#include "MagBiasEstimator.hpp"

using namespace time_literals;
using matrix::Vector3f;

namespace mag_bias_estimator
{

MagBiasEstimator::MagBiasEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

MagBiasEstimator::~MagBiasEstimator()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}

int MagBiasEstimator::task_spawn(int argc, char *argv[])
{
	MagBiasEstimator *obj = new MagBiasEstimator();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	/* Schedule a cycle to start things. */
	obj->start();

	return 0;
}

void MagBiasEstimator::start()
{
	ScheduleOnInterval(20_ms); // 50 Hz
}

void MagBiasEstimator::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			if (_arming_state != vehicle_status.arming_state) {
				_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

				// reset on any arming state change
				for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
					_reset_field_estimator[mag_index] = true;
				}

				_arming_state = vehicle_status.arming_state;
			}
		}
	}

	// only run when disarmed
	if (_armed) {
		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
			const auto calibration_count = _calibration[mag_index].calibration_count();
			_calibration[mag_index].ParametersUpdate();

			if (calibration_count != _calibration[mag_index].calibration_count()) {
				_reset_field_estimator[mag_index] = true;
			}
		}
	}

	if (_vehicle_status_flags_sub.updated()) {
		vehicle_status_flags_s vehicle_status_flags;

		if (_vehicle_status_flags_sub.copy(&vehicle_status_flags)) {

			// do nothing during regular sensor calibration
			_system_calibrating = vehicle_status_flags.condition_calibration_enabled;
			_system_sensors_initialized = vehicle_status_flags.condition_system_sensors_initialized
						      && vehicle_status_flags.condition_system_hotplug_timeout;
		}
	}

	perf_begin(_cycle_perf);

	bool calibration_updated = false;

	Vector3f angular_velocity{};

	for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
		sensor_mag_s sensor_mag;

		while (_sensor_mag_subs[mag_index].update(&sensor_mag)) {

			{
				// grab latest angular velocity
				// TODO: time synchronize?
				vehicle_angular_velocity_s vehicle_angular_velocity;

				if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
					angular_velocity = Vector3f{vehicle_angular_velocity.xyz};
				}
			}

			// apply existing mag calibration
			_calibration[mag_index].set_device_id(sensor_mag.device_id, sensor_mag.is_external);
			const Vector3f raw_mag{sensor_mag.x, sensor_mag.y, sensor_mag.z};
			const Vector3f mag_calibrated = _calibration[mag_index].Correct(raw_mag);

			float dt = (sensor_mag.timestamp_sample - _timestamp_last_update[mag_index]) * 1e-6f;
			_timestamp_last_update[mag_index] = sensor_mag.timestamp_sample;

			if (dt < 0.001f || dt > 0.2f) {
				_reset_field_estimator[mag_index] = true;
			}

			if (_reset_field_estimator[mag_index]) {
				// reset
				_bias_estimator[mag_index].setBias(Vector3f{});
				_bias_estimator[mag_index].setField(mag_calibrated);
				_timestamp_last_save[mag_index] = sensor_mag.timestamp_sample;

				_reset_field_estimator[mag_index] = false;

			} else {
				_bias_estimator[mag_index].updateEstimate(angular_velocity, mag_calibrated, dt);

				const Vector3f &bias = _bias_estimator[mag_index].getBias();

				if (!PX4_ISFINITE(bias(0)) || !PX4_ISFINITE(bias(1)) || !PX4_ISFINITE(bias(2)) || bias.longerThan(5.f)) {
					_reset_field_estimator[mag_index] = true;

				} else {
					// save the bias to the parameters to apply it given the right circumstances
					const bool longer_than_10_sec = (sensor_mag.timestamp_sample - _timestamp_last_save[mag_index]) > 10_s;
					const bool bias_significant = bias.longerThan(0.01f);

					// TODO: require sufficient movement? check validity? range? quality?
					if (bias_significant && longer_than_10_sec) {
						const Vector3f old_offset = _calibration[mag_index].offset();
						const Vector3f new_offset = _calibration[mag_index].CalculateSensorOffsetFromCalibratedBias(bias);

						PX4_DEBUG("%d (%d) updating offset: [%.6f %.6f %.6f] -> [%.6f %.6f %.6f] (Bias [%.6f %.6f %.6f])",
							  mag_index, _calibration[mag_index].device_id(),
							  (double)old_offset(0),
							  (double)old_offset(1),
							  (double)old_offset(2),
							  (double)new_offset(0),
							  (double)new_offset(1),
							  (double)new_offset(2),
							  (double)bias(0),
							  (double)bias(1),
							  (double)bias(2)
							 );

						// update calibration with new offset
						_calibration[mag_index].set_offset(new_offset);

						// zero bias estimate
						_bias_estimator[mag_index].setField(_calibration[mag_index].Correct(raw_mag));
						_bias_estimator[mag_index].setBias(Vector3f{});

						calibration_updated = true;
					}
				}
			}
		}
	}

	if (calibration_updated && !_system_calibrating && _system_sensors_initialized) {
		bool notify = false;

		for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
			if (_calibration[mag_index].device_id() != 0) {
				_calibration[mag_index].set_calibration_index(mag_index);

				if (_calibration[mag_index].ParametersSave()) {
					_timestamp_last_save[mag_index] = hrt_absolute_time();
					notify = true;
				}
			}
		}

		if (notify) {
			param_notify_changes();
		}
	}

	perf_end(_cycle_perf);
}

int MagBiasEstimator::print_status()
{
	for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
		if (_calibration[mag_index].device_id() != 0) {

			_calibration[mag_index].PrintStatus();

			const Vector3f &bias = _bias_estimator[mag_index].getBias();

			PX4_INFO("%d (%d) bias: [% 05.3f % 05.3f % 05.3f]",
				 mag_index, _calibration[mag_index].device_id(),
				 (double)bias(0),
				 (double)bias(1),
				 (double)bias(2));
		}
	}

	return 0;
}

int MagBiasEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Online magnetometer bias estimator.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mag_bias_estimator", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mag_bias_estimator_main(int argc, char *argv[])
{
	return MagBiasEstimator::main(argc, argv);
}

} // namespace load_mon
