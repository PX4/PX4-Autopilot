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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/Serial.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

#include "mathlib/math/filter/AlphaFilter.hpp"

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_baro.h>

#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_optical_flow_vel.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/estimator_status.h>


#include "mip_sdk/src/mip/mip_all.h"
#include "mip_sdk/src/mip/definitions/commands_aiding.h"

using namespace mip::C;

using namespace time_literals;

using matrix::Vector2f;

#define MS_PX4_ERROR(res, ...) \
	do \
	{ \
		PX4_ERR(__VA_ARGS__); \
		PX4_ERR(" Error: %s", mip_cmd_result_to_string(res)); \
	} while (0)

static constexpr float sq(float x) { return x * x; };

class MicroStrain : public ModuleBase<MicroStrain>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MicroStrain(const char *_device);
	~MicroStrain() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	/* Callbacks */

	// Sensor Callbacks
	static void sensorCallback(void *user, const mip_packet *packet, mip::Timestamp timestamp);

	static void filterCallback(void *user, const mip_packet *packet, mip::Timestamp timestamp);

	static void gnssCallback(void *user, const mip_packet *packet, mip::Timestamp timestamp);

private:
	/** @see ModuleBase */
	void Run() override;

	/// @brief Attempt to connect to the Sensor and set in known configuration
	bool initializeIns();

	int connectAtBaud(int32_t baud);

	mip_cmd_result forceIdle();

	mip_cmd_result getSupportedDescriptors();

	bool supportsDescriptor(uint8_t descriptor_set, uint8_t field_descriptor);

	bool supportsDescriptorSet(uint8_t descriptor_set);

	mip_cmd_result writeBaudRate(uint32_t baudrate, uint8_t port);

	mip_cmd_result configureImuRange();

	mip_cmd_result getBaseRate(uint8_t descriptor_set, uint16_t *base_rate);

	mip_cmd_result configureImuMessageFormat();

	mip_cmd_result configureFilterMessageFormat();

	mip_cmd_result configureGnssMessageFormat(uint8_t descriptor_set);

	mip_cmd_result writeMessageFormat(uint8_t descriptor_set, uint8_t num_descriptors,
					  const mip::DescriptorRate *descriptors);

	mip_cmd_result configureAidingMeasurement(uint16_t aiding_source, bool enable);

	mip_cmd_result configureAidingSources();

	mip_cmd_result writeFilterInitConfig();

	void initializeRefPos();

	void updateGeoidHeight(float geoid_height, float t);

	void sendGPSAiding();

	void sendMagAiding();

	void sendOpticalFlowAiding();

	void sendAidingMeasurements();

	bool init();

	uint32_t _dev_id{0};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	mip_cmd_result MIP_PX4_ERROR = MIP_STATUS_USER_START;
	uint8_t _parse_buffer[2048];
	bool _is_initialized{false};
	int _ms_schedule_rate_us{0};

	bool _ext_pos_vel_aiding{false};
	bool _ext_heading_aiding{false};
	bool _ext_mag_aiding{false};
	bool _ext_optical_flow_aiding{false};

	float gnss_antenna_offset1[3] = {0};
	float gnss_antenna_offset2[3] = {0};
	float ext_mag_offset[3] = {0};
	float optical_flow_offset[3] = {0};
	mip_aiding_frame_config_command_rotation rotation_sens = {0};
	mip_aiding_frame_config_command_rotation rotation_gnss = {0};
	mip_aiding_frame_config_command_rotation rotation_ext_mag = {0};
	mip_aiding_frame_config_command_rotation rotation_oflow = {0};

	float ext_mag_uncert = 0.0;
	float opt_flow_uncert = 0.0;

	AlphaFilter<float> _geoid_height_lpf;
	uint64_t _last_geoid_height_update_us{0};
	static constexpr float kGeoidHeightLpfTimeConstant = 10.f;

	MapProjection _pos_ref{};
	double _ref_alt = 0;

	template <typename T>
	struct SensorSample {
		T sample;
		bool updated = false;
	};

	mip_filter_gnss_dual_antenna_status_data dual_ant_stat{0};

	uint16_t _supported_descriptors[1024] = {0};
	uint16_t _supported_desc_len = 0;
	uint16_t _supported_descriptor_sets[1024] = {0};
	uint16_t _supported_desc_set_len = 0;

	mip::C::mip_interface _device;

	// Handlers
	mip_dispatch_handler _sensor_data_handler;
	mip_dispatch_handler _filter_data_handler;
	mip_dispatch_handler _gnss_data_handler[2];

	char _port[128];

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MS_MODE>) _param_ms_mode,
		(ParamInt<px4::params::MS_IMU_RATE_HZ>) _param_ms_imu_rate_hz,
		(ParamInt<px4::params::MS_MAG_RATE_HZ>) _param_ms_mag_rate_hz,
		(ParamInt<px4::params::MS_BARO_RATE_HZ>) _param_ms_baro_rate_hz,
		(ParamInt<px4::params::MS_FILT_RATE_HZ>) _param_ms_filter_rate_hz,
		(ParamInt<px4::params::MS_GNSS_RATE_HZ>) _param_ms_gnss_rate_hz,
		(ParamInt<px4::params::MS_ALIGNMENT>) _param_ms_alignment,
		(ParamInt<px4::params::MS_GNSS_AID_SRC>) _param_ms_gnss_aid_src_ctrl,
		(ParamInt<px4::params::MS_INT_MAG_EN>) _param_ms_int_mag_en,
		(ParamInt<px4::params::MS_EXT_MAG_EN>) _param_ms_ext_mag_en,
		(ParamInt<px4::params::MS_INT_HEAD_EN>) _param_ms_int_heading_en,
		(ParamInt<px4::params::MS_EXT_HEAD_EN>) _param_ms_ext_heading_en,
		(ParamInt<px4::params::MS_OPT_FLOW_EN>) _param_ms_ext_opt_flow_en,
		(ParamInt<px4::params::MS_SVT_EN>) _param_ms_svt_en,
		(ParamInt<px4::params::MS_ACCEL_RANGE>) _param_ms_accel_range_setting,
		(ParamInt<px4::params::MS_GYRO_RANGE>) _param_ms_gyro_range_setting,
		(ParamFloat<px4::params::MS_GNSS_OFF1_X>) _param_ms_gnss_offset1_x,
		(ParamFloat<px4::params::MS_GNSS_OFF1_Y>) _param_ms_gnss_offset1_y,
		(ParamFloat<px4::params::MS_GNSS_OFF1_Z>) _param_ms_gnss_offset1_z,
		(ParamFloat<px4::params::MS_GNSS_OFF2_X>) _param_ms_gnss_offset2_x,
		(ParamFloat<px4::params::MS_GNSS_OFF2_Y>) _param_ms_gnss_offset2_y,
		(ParamFloat<px4::params::MS_GNSS_OFF2_Z>) _param_ms_gnss_offset2_z,
		(ParamFloat<px4::params::MS_SENSOR_ROLL>) _param_ms_sensor_roll,
		(ParamFloat<px4::params::MS_SENSOR_PTCH>) _param_ms_sensor_pitch,
		(ParamFloat<px4::params::MS_SENSOR_YAW>) _param_ms_sensor_yaw,
		(ParamFloat<px4::params::MS_GNSS_ROLL>) _param_ms_gnss_roll,
		(ParamFloat<px4::params::MS_GNSS_PTCH>) _param_ms_gnss_pitch,
		(ParamFloat<px4::params::MS_GNSS_YAW>) _param_ms_gnss_yaw,
		(ParamFloat<px4::params::MS_EMAG_OFF_X>) _param_ms_emag_offset_x,
		(ParamFloat<px4::params::MS_EMAG_OFF_Y>) _param_ms_emag_offset_y,
		(ParamFloat<px4::params::MS_EMAG_OFF_Z>) _param_ms_emag_offset_z,
		(ParamFloat<px4::params::MS_EMAG_ROLL>) _param_ms_emag_roll,
		(ParamFloat<px4::params::MS_EMAG_PTCH>) _param_ms_emag_pitch,
		(ParamFloat<px4::params::MS_EMAG_YAW>) _param_ms_emag_yaw,
		(ParamFloat<px4::params::MS_OFLW_OFF_X>) _param_ms_oflow_offset_x,
		(ParamFloat<px4::params::MS_OFLW_OFF_Y>) _param_ms_oflow_offset_y,
		(ParamFloat<px4::params::MS_OFLW_OFF_Z>) _param_ms_oflow_offset_z,
		(ParamFloat<px4::params::MS_EMAG_UNCERT>) _param_ms_emag_uncert,
		(ParamFloat<px4::params::MS_OFLW_UNCERT>) _param_ms_oflow_uncert
	)

	// Sensor types needed for message creation / updating / publishing
	PX4Accelerometer _px4_accel{0};
	PX4Gyroscope _px4_gyro{0};
	PX4Magnetometer _px4_mag{0};
	sensor_baro_s _sensor_baro{0};

	// Must publish to prevent sensor stale failure (sensors module)
	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};
	uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub[2] {ORB_ID(sensor_gps), ORB_ID(sensor_gps)};
	uORB::Publication<sensor_selection_s> _sensor_selection_pub{ORB_ID(sensor_selection)};

	uORB::Publication<vehicle_global_position_s> _vehicle_global_position_pub;
	uORB::Publication<vehicle_attitude_s> _vehicle_attitude_pub;
	uORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub;
	uORB::Publication<vehicle_odometry_s> _vehicle_odometry_pub{ORB_ID(vehicle_odometry)};
	uORB::Publication<vehicle_angular_velocity_s> _vehicle_angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};

	// Needed for health checks
	uORB::Publication<estimator_status_s> _estimator_status_pub{ORB_ID(estimator_status)};

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_magnetometer_sub{ORB_ID(vehicle_magnetometer)};
	uORB::Subscription _vehicle_optical_flow_vel_sub{ORB_ID(vehicle_optical_flow_vel)};
};
