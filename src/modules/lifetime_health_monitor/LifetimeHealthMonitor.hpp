/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/drv_hrt.h>
#include <sys/stat.h>
#include <math.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

#include <uORB/uORB.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/esc_status.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/lifetime_health_stats.h>
#include <uORB/topics/sensor_gps.h>

#include "lifetime_health_data.h"

/* path to lifetime health data storage */
#define LIFETIME_HEALTH_DATA_PATH PX4_STORAGEDIR "/lifetime_health_data"

/* path prefix to each record */
#define LIFETIME_HEALTH_DATA_NUM_RECORDS 2
#define LIFETIME_HEALTH_DATA_RECORD_1_PATH LIFETIME_HEALTH_DATA_PATH "/record_1.bson"
#define LIFETIME_HEALTH_DATA_RECORD_2_PATH LIFETIME_HEALTH_DATA_PATH "/record_2.bson"

// check that the size of record paths is less than 50 characters
static_assert(strlen(LIFETIME_HEALTH_DATA_RECORD_1_PATH) < 50, "LIFETIME_HEALTH_DATA_RECORD_1_PATH is too long");
static_assert(strlen(LIFETIME_HEALTH_DATA_RECORD_2_PATH) < 50, "LIFETIME_HEALTH_DATA_RECORD_2_PATH is too long");

char record_paths[LIFETIME_HEALTH_DATA_NUM_RECORDS][50] = {LIFETIME_HEALTH_DATA_RECORD_1_PATH, LIFETIME_HEALTH_DATA_RECORD_2_PATH};

lifetime_health_data_s _latest_health_data;

/**
 * flag to signal we are loading data for the purpose of verifying written record,
 * not to update in-memory data, which is only done at initialization
 */
bool _is_loading_to_verify_record{false};

using namespace time_literals;

class LifetimeHealthMonitor : public ModuleBase, public ModuleParams
{
public:
	static Descriptor desc;

	LifetimeHealthMonitor();
	~LifetimeHealthMonitor() override = default;

	static LifetimeHealthMonitor *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int run_trampoline(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** @see ModuleBase::run() */
	void run() override;

private:
	static constexpr uint64_t TASK_UPDATE_INTERVAL = 100_ms;

	// cap integration interval size to an upperbound that's greater than expected expected interval (TASKS_UPDATE_INTERVAL)
	static constexpr uint32_t MAX_INTEGRATION_DELTA_MS = 500;

	static constexpr uint16_t UORB_PUBLISH_INTERVAL_MS = 10000;
	static constexpr uint16_t WAIT_TIME_AFTER_DISARM_TO_WRITE_MS = 3000;
	static constexpr uint16_t SD_WRITE_INTERVAL_MS = 15000;

	// write operation methods
	bool _lifetime_health_data_dir_exists()
	{
		struct stat st;
		return stat(LIFETIME_HEALTH_DATA_PATH, &st) == 0 && S_ISDIR(st.st_mode);
	}

	bool _mkdir_health_data_path()
	{
		return mkdir(LIFETIME_HEALTH_DATA_PATH, 0777) == 0;
	}

	bool _persistent_storage_accessible()
	{
		return access(PX4_STORAGEDIR, F_OK) != -1;
	}


	void _determine_threshold_values();

	void _set_initial_timestamp();

	// Add subscribers, publishers, etc. as needed
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _wind_sub{ORB_ID(wind)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_imu_status_sub{ORB_ID(vehicle_imu_status)};
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};

	uORB::Publication<lifetime_health_stats_s> _lifetime_health_stats_pub{ORB_ID(lifetime_health_stats)};

	void _update_electrical_history(const esc_report_s &esc_report, uint16_t esc_index, uint32_t delta_ms);
	void _update_esc_temp_history(const esc_report_s &esc_report, uint16_t esc_index, uint32_t delta_ms);
	void _update_motor_temp_history(const esc_report_s &esc_report, uint16_t esc_index, uint32_t delta_ms);
	void _update_rpm_history(const esc_report_s &esc_report, uint16_t esc_index, uint32_t delta_ms);
	void _update_prop_dynamics_history(const esc_report_s &esc_report, const vehicle_odometry_s &vehicle_odometry,
					   uint16_t esc_index, uint32_t delta_ms);
	void _update_all_esc_and_motor_history(const esc_status_s &esc_status, const vehicle_odometry_s &vehicle_odometry);
	void _update_flight_history();
	void _update_structural_history();

	void _publish_lifetime_health_stats(lifetime_health_data_s &health_data);

	// BSON decoding methods
	int _lifetime_health_data_import();
	static int bson_decode_lifetime_health_data_callback(bson_decoder_t decoder, bson_node_t node);
	static int handle_bson_int32_key(bson_decoder_t decoder, bson_node_t node, uint32_t bson_key_hash);
	static int handle_bson_int64_key(bson_decoder_t decoder, bson_node_t node, uint32_t bson_key_hash);
	static int handle_bson_binary_key(bson_decoder_t decoder, bson_node_t node, uint32_t bson_key_hash);

	// BSON encoding methods
	int _verify_record(int fd);
	int _write_to_record(int fd, lifetime_health_data_s *data);
	int _bson_encode_and_write_lifetime_health_data(lifetime_health_data_s *data);

	// Threshold values
	float _motor_max_current{60.f};
	float _motor_warn_current{40.f};

	float _motor_warn_temp{120.f};
	float _motor_max_temp{150.f};
	float _esc_warn_temp{95.f};
	float _esc_max_temp{105.f};

	int _med_rpm{4000};
	int _high_rpm{6000};

	//  prop flop load for a motor is defined as (angular velocity of propeller (rad/s) * angular velocity magnitude of drone in x-y (rad/s)
	float _high_gyroscopic_torque_threshold{80.f};
	float _moderate_gyroscopic_torque_threshold{40.f};

	// asymmetric lift load for a motor is defined as (angular velocity of propeller (rad/s) * air speed magnitude (m/s))
	float _high_prop_asymmetric_lift_threshold{2500.f};
	float _moderate_prop_asymmetric_lift_threshold{1500.f};

	float _altitude_m_med_threshold{1200.f};
	float _altitude_m_high_threshold{2400.f};
	float _altitude_m_very_high_threshold{3600.f};

	float _ambient_temp_c_low_threshold{-20.f};
	float _ambient_temp_c_high_threshold{50.f};

	float _vibration_severity_high{20.f};
	float _vibration_severity_moderate{10.f};
};
