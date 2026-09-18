/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

// C Headers
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <cstdint>

// C++ Headers
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Publication.hpp>

// Other Headers
#include "naviguider_consts.hpp"
#include "naviguider_structs.hpp"

using namespace time_literals;

class Naviguider : public device::I2C,
//public ModuleBase<Naviguider>,
	public ModuleBase,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	Naviguider(int bus, uint8_t address);
	~Naviguider() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	// public static descriptor
	static ModuleBase::Descriptor module_desc;

	int init() override;
	int print_status() override;

private:
	void Run() override;

	/* ---- Data Handling ---- */
	uint32_t read_fifo(uint8_t *buffer);
	uint32_t parse_fifo(uint8_t *buffer, uint32_t len);
	uint32_t parse_next_fifo_block(uint8_t *buffer, uint32_t size);
	uint32_t handle_sensor_data_3axis(uint32_t remaining, uint8_t sensor_id, uint8_t *data_start);
	uint32_t handle_pressure(uint32_t remaining, uint8_t sensor_id, uint8_t *data_start);
	uint32_t handle_rotation_vector(uint32_t remaining, uint8_t sensor_id, uint8_t *data_start);
	uint8_t _fifo_buf[FIFO_BUF_SIZE] {};

	/* ---- Publishing ---- */
	void publish_sensor_selection();
	void publish_accel(const hrt_abstime &timestmap, float x, float y, float z);
	void publish_gyro(const hrt_abstime &timestmap, float x, float y, float z);
	void publish_mag(const hrt_abstime &timestmap, float x, float y, float z);
	void publish_baro(const hrt_abstime &timestmap, float pressure_pa, float temperature_c);
	void publish_attitude(const hrt_abstime &timestmap, float qw, float qx, float qy, float qz);
	void publish_synthetic_mag(const hrt_abstime &timestmap, float qw, float qx, float qy, float qz);
	hrt_abstime _last_sens_select_time;
	uORB::PublicationMulti<vehicle_attitude_s> *_att_pub;
	PX4Accelerometer *_px4_accel;
	PX4Gyroscope *_px4_gyro;
	PX4Magnetometer *_px4_mag;
	PX4Magnetometer *_px4_mag_synth;
	uORB::PublicationMulti<sensor_baro_s> _baro_pub{ORB_ID(sensor_baro)};
	uORB::PublicationMulti<estimator_status_s> _est_status_pub{ORB_ID(estimator_status)};
	uORB::PublicationMulti<sensor_selection_s> _sens_select_pub{ORB_ID(sensor_selection)};

	/* ---- Initialization ---- */
	// Stuff needed for setting the synthetic magnetometer output to be
	// consistent with the local magnetic field when disarmed and stationary
	uORB::SubscriptionInterval _vehicle_status_sub{ORB_ID(vehicle_status), 1_s};
	uORB::SubscriptionInterval _vehicle_local_pos_sub{ORB_ID(vehicle_local_position), 1_s};
	float _last_mag_body_vec[3];
	float _last_att_quat[4];
	float _last_mag_ned_vec[3];
	int32_t _synth_mag_mode;
	void update_synthetic_mag_field_from_readings();
	void update_synthetic_mag_field_from_params();

	/* ---- I2C ---- */
	int read_reg(uint8_t reg, void *data, uint32_t len);
	int read_reg_u8(uint8_t reg, uint8_t &v);
	int write_reg(uint8_t reg, const void *data, uint32_t len);
	int write_reg_u8(uint8_t reg, uint8_t v);
	uint8_t _addr{0};

	/* ---- Configuration ---- */
	int update_sensor_rates();
	int set_sensor_rate(uint8_t sensorId, uint16_t rate);
	void set_scale_factors();
	void set_ned_frame(bool ned_not_enu);
	float _scale[128] {};
	uint16_t _last_rate_acc;
	uint16_t _last_rate_gyro;
	uint16_t _last_rate_mag;
	uint16_t _last_rate_baro;
	uint16_t _last_rate_rotvec;

	/* ---- Interval Counters ---- */
	void print_sensor_counts();
	void reset_sensor_counts();
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _accel_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": accel interval")};
	perf_counter_t _gyro_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": gyro interval")};
	perf_counter_t _mag_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": mag interval")};
	perf_counter_t _baro_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": baro interval")};
	hrt_abstime _count_start_time;
	hrt_abstime _last_print_time;
	uint64_t _accel_count;
	uint64_t _gyro_count;
	uint64_t _mag_count;
	uint64_t _baro_count;
	uint64_t _attitude_count;

	/* ---- Debug ---- */
	void debug_pub();
	void pub_last_phys_sensor_status();
	void pub_last_phys_sensors_present();
	void pub_last_sensor_info();
	void pub_last_phys_sensor_info();
	void pub_last_sensor_config();
	void pub_last_warm_start_cal_score();
	uint16_t _dbg_round_robin_sensor;
	uORB::PublicationMulti<debug_array_s> _dbg_arr_pub{ORB_ID(debug_array)};

	/* ---- Sensor Info ---- */
	int param_read(uint8_t page, uint8_t param_no, void *data, uint8_t size);
	int param_write(uint8_t page, uint8_t param_no, const void *data, uint8_t size);
	int get_phys_sensor_status();
	int get_phys_sensors_present();
	int get_sensor_info(uint8_t sensorId);
	int get_phys_sensor_info(uint8_t sensorId);
	int get_sensor_config(uint8_t sensorId);
	// TODO(js1195): Add functions for accessing warm start param data
	PhysSensorStatus _last_phys_sensor_status;
	PhysSensorsPresent _last_phys_sensors_present;
	PhysSensorInfo _last_phys_sensor_info;
	SensorInfo _last_sensor_info;
	SensorConfig _last_sensor_config;
	uint8_t  _last_sensor_config_sensor_id;

	/* ---- Warm Start ---- */
	int get_warm_start_cal_score();
	WarmStartCalScore _last_warm_start_cal_score;

	/* ---- Meta Events ---- */
	uint32_t handle_meta_event(uint32_t remainig, uint8_t *data_start);
	void handle_meta_sample_rate_changed(uint8_t *data_start);
	void handle_meta_magnetic_transient(uint8_t *data_start);
	void handle_meta_cal_status_changed(uint8_t *data_start);
	void handle_meta_calibration_stable(uint8_t *data_start);
	void handle_meta_sensor_error(uint8_t *data_start);
	void handle_meta_dynamic_range_changed(uint8_t *data_start);
	void handle_meta_self_test_results(uint8_t *data_start);
	void handle_meta_initialized(uint8_t *data_start);
	void handle_meta_power_mode_changed(uint8_t *data_start);
	// MetaEvents _meta_event;

	/* ---- Parameters ---- */
	void parameters_update();
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_NG_MODE>)    _param_ng_mode,
		(ParamInt<px4::params::NG_ROT>)          _param_ng_rot,
		(ParamInt<px4::params::NG_RATE_ACC>)     _param_ng_rate_acc,
		(ParamInt<px4::params::NG_RATE_GYRO>)    _param_ng_rate_gyro,
		(ParamInt<px4::params::NG_RATE_MAG>)     _param_ng_rate_mag,
		(ParamInt<px4::params::NG_RATE_BARO>)    _param_ng_rate_baro,
		(ParamInt<px4::params::NG_RATE_ROTVEC>)  _param_ng_rate_rotvec,
		(ParamInt<px4::params::NG_SYN_MAG_MODE>) _param_synth_mag_mode,
		(ParamFloat<px4::params::NG_SYN_MAG_N>)  _param_ng_local_north_mag_field,
		(ParamFloat<px4::params::NG_SYN_MAG_E>)  _param_ng_local_east_mag_field,
		(ParamBool<px4::params::SENS_EN_NG>)     _param_sens_en_ng,
		(ParamFloat<px4::params::NG_SYN_MAG_D>)  _param_ng_local_down_mag_field
	);
	uint32_t _param_ng_mode_initial_value;
	uint32_t _param_ng_rot_initial_value;
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
};
