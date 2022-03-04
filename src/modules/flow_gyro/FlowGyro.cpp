/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "FlowGyro.hpp"

FlowGyro::FlowGyro() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_gyro_buffer(25)
{
	// We must be the first instance to publish because uavcan selects first instance at subscription.
	_optical_flow_pub.advertise();
}

FlowGyro::~FlowGyro()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	_sensor_gyro_sub.unregisterCallback();
}

bool FlowGyro::init()
{
	if (!_sensor_gyro_sub.registerCallback()) {
		PX4_ERR("sensor_gyro callback registration failed");
		return false;
	}

	return true;
}

void FlowGyro::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	sensor_gyro_s gyro;

	while (_sensor_gyro_sub.update(&gyro)) {
		_gyro_buffer.push(gyro);
	}

	if (_optical_flow_sub.updated()) {
		if (_flow_available) {
			PX4_WARN("New flow available while previous not published, publishing without gyros");
			_optical_flow.gyro_x_rate_integral = NAN;
			_optical_flow.gyro_y_rate_integral = NAN;
			_optical_flow.gyro_z_rate_integral = NAN;
			_optical_flow_pub.publish(_optical_flow);
		}

		_flow_available = _optical_flow_sub.copy(&_optical_flow);
	}

	if (_flow_available && _gyro_buffer.get_newest().timestamp_sample > _optical_flow.timestamp) {
		// Integrate gyros on <timespan> period up to <timestamp>
		_optical_flow.gyro_x_rate_integral = 0;
		_optical_flow.gyro_y_rate_integral = 0;
		_optical_flow.gyro_z_rate_integral = 0;
		hrt_abstime start = _optical_flow.timestamp - _optical_flow.integration_timespan;
		hrt_abstime stop = _optical_flow.timestamp;
		const int i = _gyro_buffer.get_oldest_index();
		hrt_abstime last_time = _gyro_buffer[i].timestamp_sample;
		bool start_ok = false;
		bool stop_ok = false;

		for (int k = 1; k < _gyro_buffer.get_length(); k++) {
			sensor_gyro_s &current_gyro = _gyro_buffer[(i + k) % _gyro_buffer.get_length()];
			float dt = (current_gyro.timestamp_sample - last_time) * 1e-6f;
			float ratio = 1.0f;

			// First add - integrate only on relevant period
			if (!start_ok && current_gyro.timestamp_sample > start) {
				if (last_time > start) {
					PX4_WARN("Gyro buffer too short, dropping sample! %.3fs missing on %.3f timespan", (last_time - start) * 1e-6,
						 _optical_flow.integration_timespan * 1e-6);
					break;
				}

				ratio = (current_gyro.timestamp_sample - start) * 1e-6f / dt;
				start_ok = true;
			}

			// Last add - integrate only on relevant period
			else if (!stop_ok && current_gyro.timestamp_sample > stop) {
				ratio = (stop - last_time) * 1e-6f / dt;
				stop_ok = true;
			}

			if (start_ok) {
				_optical_flow.gyro_x_rate_integral += current_gyro.x * dt * ratio;
				_optical_flow.gyro_y_rate_integral += current_gyro.y * dt * ratio;
				_optical_flow.gyro_z_rate_integral += current_gyro.z * dt * ratio;
			}

			last_time = current_gyro.timestamp_sample;

			if (stop_ok) {
				break;
			}
		}

		if (start_ok && stop_ok) {
			_optical_flow_pub.publish(_optical_flow);
			_flow_available = false;
		}
	}

	perf_end(_loop_perf);
}

int FlowGyro::task_spawn(int argc, char *argv[])
{
	FlowGyro *instance = new FlowGyro();

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

int FlowGyro::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int FlowGyro::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlowGyro::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Simple module that uses boards'gyros to populate optical_flow messages.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flow_gyro", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flow_gyro_main(int argc, char *argv[])
{
	return FlowGyro::main(argc, argv);
}
