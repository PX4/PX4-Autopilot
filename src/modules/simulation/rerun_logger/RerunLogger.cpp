/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "RerunLogger.hpp"

ModuleBase::Descriptor RerunLogger::desc{task_spawn, custom_command, print_usage};

RerunLogger::RerunLogger(const char *sim_name) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_rec(std::string(sim_name) + "-viewer", std::string("px4-") + sim_name)
{
}

bool RerunLogger::init()
{
	auto err = _rec.connect_grpc("rerun+http://127.0.0.1:9876/proxy");

	if (err.is_err()) {
		PX4_WARN("Rerun gRPC connection failed, visualization disabled");

	} else {
		_rerun_connected = true;
	}

	ScheduleOnInterval(SAMPLE_INTERVAL_US);
	return true;
}

void RerunLogger::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	if (!_rerun_connected) {
		return;
	}

	// Log console messages (PX4_INFO/WARN/ERR)
	log_message_s log_msg{};

	while (_log_message_sub.update(&log_msg)) {
		px4_rerun::log_text(_rec, log_msg.timestamp, log_msg.text, log_msg.severity);
	}

	// Log estimated drone pose (every cycle)
	vehicle_local_position_s vlp{};

	if (_vehicle_local_position_sub.copy(&vlp)) {
		if (!_map_ref_initialized && vlp.xy_global) {
			_map_projection.initReference(vlp.ref_lat, vlp.ref_lon, vlp.timestamp);
			_ref_alt = vlp.ref_alt;
			_map_ref_initialized = true;
		}

		if (vlp.xy_valid && vlp.z_valid) {
			px4_rerun::set_vehicle_position(vlp.x, vlp.y, vlp.z);
		}
	}

	// Log attitude independently (always available from IMU)
	vehicle_attitude_s att{};

	if (_vehicle_attitude_sub.copy(&att)) {
		px4_rerun::set_vehicle_attitude(att.q[0], att.q[1], att.q[2], att.q[3]);
	}

	// Log combined pose (position + attitude)
	px4_rerun::log_vehicle_pose(_rec, att.timestamp);

	// Log home position (only when updated)
	if (_home_position_sub.updated()) {
		home_position_s home{};

		if (_home_position_sub.copy(&home) && home.valid_lpos) {
			px4_rerun::log_home_position(_rec, home.timestamp, home.x, home.y, home.z);
		}
	}

	// Log raw magnetometer readings
	for (uint8_t i = 0; i < _sensor_mag_subs.size(); i++) {
		sensor_mag_s sensor_mag{};

		if (_sensor_mag_subs[i].update(&sensor_mag)) {
			px4_rerun::log_sensor_mag(_rec, sensor_mag.timestamp, i, sensor_mag.x, sensor_mag.y, sensor_mag.z);
		}
	}

	// Accumulate mag calibration samples; batch-log when a side completes
	mag_worker_data_s mwd{};

	if (_mag_worker_data_sub.update(&mwd)) {
		for (uint8_t i = 0; i < MAX_MAGS; i++) {
			if (mwd.calibration_counter_total[i] > _mag_cal_counter[i]) {
				_mag_cal_samples[i].push_back({mwd.x[i], mwd.y[i], mwd.z[i]});
				_mag_cal_counter[i] = mwd.calibration_counter_total[i];
				_mag_cal_last_timestamp = mwd.timestamp;
			}
		}

		// Detect side completion from sample count rather than done_count,
		// which is incremented after the last sample publish and never
		// propagated for the final side.
		if (mwd.calibration_points_perside > 0) {
			uint32_t max_counter = 0;

			for (uint8_t i = 0; i < MAX_MAGS; i++) {
				if (mwd.calibration_counter_total[i] > max_counter) {
					max_counter = mwd.calibration_counter_total[i];
				}
			}

			uint32_t completed_sides = max_counter / mwd.calibration_points_perside;

			if (completed_sides > _mag_cal_sides_flushed) {
				_mag_cal_sides_flushed = completed_sides;

				for (uint8_t i = 0; i < MAX_MAGS; i++) {
					if (!_mag_cal_samples[i].empty()) {
						px4_rerun::log_mag_cal_samples(_rec, _mag_cal_last_timestamp, i,
									       _mag_cal_samples[i]);
					}
				}
			}
		}
	}

	// Log current mission item (only when updated)
	if (_navigator_mission_item_sub.updated()) {
		navigator_mission_item_s mission_item{};

		if (_navigator_mission_item_sub.copy(&mission_item) && _map_ref_initialized) {
			float local_x, local_y;
			_map_projection.project(
				static_cast<double>(mission_item.latitude),
				static_cast<double>(mission_item.longitude),
				local_x, local_y);
			float local_z = mission_item.altitude_is_relative
				       ? -mission_item.altitude
				       : -(mission_item.altitude - _ref_alt);

			px4_rerun::log_mission_item(_rec, mission_item.timestamp,
						    local_x, local_y, local_z, mission_item.nav_cmd);
		}
	}
}

int RerunLogger::task_spawn(int argc, char *argv[])
{
	const char *sim_name = "sim";

	int ch;

	while ((ch = getopt(argc, argv, "s:")) != EOF) {
		switch (ch) {
		case 's':
			sim_name = optarg;
			break;

		default:
			break;
		}
	}

	RerunLogger *instance = new RerunLogger(sim_name);

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int RerunLogger::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RerunLogger::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Logs PX4 state data to a Rerun visualization instance for SITL.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rerun_logger", "simulation");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('s', "sim", nullptr, "Simulator name (used for Rerun recording ID)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rerun_logger_main(int argc, char *argv[])
{
	return ModuleBase::main(RerunLogger::desc, argc, argv);
}
