/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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


/**
 * @file simulator.h
 *
 * This module interfaces via MAVLink to a software in the loop simulator (SITL)
 * such as jMAVSim or Gazebo.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/bitmask.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/esc_report.h>
#include <uORB/topics/irlock_report.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_optical_flow.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/rpm.h>

#include <random>

#include <mavlink.h>
#include <mavlink_types.h>

using namespace time_literals;

//! Enumeration to use on the bitmask in HIL_SENSOR
enum class SensorSource {
	ACCEL		= 0b111,
	GYRO		= 0b111000,
	MAG		= 0b111000000,
	BARO		= 0b1101000000000,
	DIFF_PRESS	= 0b10000000000
};
ENABLE_BIT_OPERATORS(SensorSource)

//! AND operation for the enumeration and unsigned types that returns the bitmask
template<typename A, typename B>
static inline SensorSource operator &(A lhs, B rhs)
{
	// make it type safe
	static_assert((std::is_same<A, uint32_t>::value || std::is_same<A, SensorSource>::value),
		      "first argument is not uint32_t or SensorSource enum type");
	static_assert((std::is_same<B, uint32_t>::value || std::is_same<B, SensorSource>::value),
		      "second argument is not uint32_t or SensorSource enum type");

	typedef typename std::underlying_type<SensorSource>::type underlying;

	return static_cast<SensorSource>(
		       static_cast<underlying>(lhs) &
		       static_cast<underlying>(rhs)
	       );
}

class SimulatorMavlink : public ModuleParams
{
public:
	static SimulatorMavlink *getInstance() { return _instance; }

	enum class InternetProtocol {
		TCP,
		UDP
	};

	static int start(int argc, char *argv[]);

	void set_ip(InternetProtocol ip) { _ip = ip; }
	void set_port(unsigned port) { _port = port; }
	void set_hostname(const char *hostname) { _hostname = hostname; }
	void set_tcp_remote_ipaddr(char *tcp_remote_ipaddr) { _tcp_remote_ipaddr = tcp_remote_ipaddr; }

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	bool has_initialized() { return _has_initialized.load(); }
#endif

private:
	SimulatorMavlink();

	~SimulatorMavlink()
	{
		// free perf counters
		perf_free(_perf_sim_delay);
		perf_free(_perf_sim_interval);

		for (size_t i = 0; i < sizeof(_dist_pubs) / sizeof(_dist_pubs[0]); i++) {
			delete _dist_pubs[i];
		}

		px4_lockstep_unregister_component(_lockstep_component);

		for (size_t i = 0; i < sizeof(_sensor_gps_pubs) / sizeof(_sensor_gps_pubs[0]); i++) {
			delete _sensor_gps_pubs[i];
		}

		_instance = nullptr;
	}


	void check_failure_injections();

	int publish_distance_topic(const mavlink_distance_sensor_t *dist);

	static SimulatorMavlink *_instance;

	// simulated sensor instances
	static constexpr uint8_t ACCEL_COUNT_MAX = 3;
	PX4Accelerometer _px4_accel[ACCEL_COUNT_MAX] {
		{1310988, ROTATION_NONE}, // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
		{1310996, ROTATION_NONE}, // 1310996: DRV_IMU_DEVTYPE_SIM, BUS: 2, ADDR: 1, TYPE: SIMULATION
		{1311004, ROTATION_NONE}, // 1311004: DRV_IMU_DEVTYPE_SIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
	};

	static constexpr uint8_t GYRO_COUNT_MAX = 3;
	PX4Gyroscope _px4_gyro[GYRO_COUNT_MAX] {
		{1310988, ROTATION_NONE}, // 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
		{1310996, ROTATION_NONE}, // 1310996: DRV_IMU_DEVTYPE_SIM, BUS: 2, ADDR: 1, TYPE: SIMULATION
		{1311004, ROTATION_NONE}, // 1311004: DRV_IMU_DEVTYPE_SIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
	};

	static constexpr uint8_t MAG_COUNT_MAX = 2;
	PX4Magnetometer _px4_mag[MAG_COUNT_MAX] {
		{197388, ROTATION_NONE},
		{197644, ROTATION_NONE},
	};

	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pubs[2] {{ORB_ID(sensor_baro)}, {ORB_ID(sensor_baro)}};

	float _sensors_temperature{0};

	perf_counter_t _perf_sim_delay{perf_alloc(PC_ELAPSED, MODULE_NAME": network delay")};
	perf_counter_t _perf_sim_interval{perf_alloc(PC_INTERVAL, MODULE_NAME": network interval")};

	// uORB publisher handlers
	uORB::Publication<differential_pressure_s>	_differential_pressure_pub{ORB_ID(differential_pressure)};
	uORB::PublicationMulti<sensor_optical_flow_s>	_sensor_optical_flow_pub{ORB_ID(sensor_optical_flow)};
	uORB::Publication<irlock_report_s>		_irlock_report_pub{ORB_ID(irlock_report)};
	uORB::Publication<esc_status_s>			_esc_status_pub{ORB_ID(esc_status)};
	uORB::Publication<vehicle_odometry_s>		_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
	uORB::Publication<vehicle_odometry_s>		_mocap_odometry_pub{ORB_ID(vehicle_mocap_odometry)};

	uORB::Publication<vehicle_command_ack_s>	_command_ack_pub{ORB_ID(vehicle_command_ack)};

	uORB::PublicationMulti<distance_sensor_s>	*_dist_pubs[ORB_MULTI_MAX_INSTANCES] {};
	uint32_t _dist_sensor_ids[ORB_MULTI_MAX_INSTANCES] {};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	unsigned int _port{14560};

	InternetProtocol _ip{InternetProtocol::UDP};

	std::string _hostname{""};

	char *_tcp_remote_ipaddr{nullptr};

	double _realtime_factor{1.0};		///< How fast the simulation runs in comparison to real system time

	hrt_abstime _last_sim_timestamp{0};
	hrt_abstime _last_sitl_timestamp{0};

	void run();

	void handle_message(const mavlink_message_t *msg);
	void handle_message_distance_sensor(const mavlink_message_t *msg);
	void handle_message_hil_gps(const mavlink_message_t *msg);
	void handle_message_hil_sensor(const mavlink_message_t *msg);
	void handle_message_hil_state_quaternion(const mavlink_message_t *msg);
	void handle_message_landing_target(const mavlink_message_t *msg);
	void handle_message_odometry(const mavlink_message_t *msg);
	void handle_message_optical_flow(const mavlink_message_t *msg);
	void handle_message_rc_channels(const mavlink_message_t *msg);
	void handle_message_vision_position_estimate(const mavlink_message_t *msg);

	void parameters_update(bool force);
	void poll_for_MAVLink_messages();
	void request_hil_state_quaternion();
	void send();
	void send_controls();
	void send_heartbeat();
	void send_esc_telemetry(mavlink_hil_actuator_controls_t hil_act_control);
	void send_mavlink_message(const mavlink_message_t &aMsg);
	void update_sensors(const hrt_abstime &time, const mavlink_hil_sensor_t &sensors);

	static void *sending_trampoline(void *);

	void actuator_controls_from_outputs(mavlink_hil_actuator_controls_t *msg);

	// uORB publisher handlers
	uORB::Publication<vehicle_angular_velocity_s>	_vehicle_angular_velocity_ground_truth_pub{ORB_ID(vehicle_angular_velocity_groundtruth)};
	uORB::Publication<vehicle_attitude_s>		_attitude_ground_truth_pub{ORB_ID(vehicle_attitude_groundtruth)};
	uORB::Publication<vehicle_global_position_s>	_gpos_ground_truth_pub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Publication<vehicle_local_position_s>	_lpos_ground_truth_pub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::Publication<input_rc_s>			_input_rc_pub{ORB_ID(input_rc)};

	//rpm
	uORB::Publication<rpm_s>			_rpm_pub{ORB_ID(rpm)};

	// HIL GPS
	static constexpr int MAX_GPS = 3;
	uORB::PublicationMulti<sensor_gps_s>	*_sensor_gps_pubs[MAX_GPS] {};
	uint8_t _gps_ids[MAX_GPS] {};
	std::default_random_engine _gen{};

	// uORB subscription handlers
	orb_sub_t _actuator_outputs_sub{ORB_SUB_INVALID};
	actuator_outputs_s _actuator_outputs{};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};

	// hil map_ref data
	MapProjection _global_local_proj_ref{};
	float _global_local_alt0{NAN};

	vehicle_status_s _vehicle_status{};
	battery_status_s _battery_status{};

	bool _accel_blocked[ACCEL_COUNT_MAX] {};
	bool _accel_stuck[ACCEL_COUNT_MAX] {};
	sensor_accel_fifo_s _last_accel_fifo{};
	matrix::Vector3f _last_accel[GYRO_COUNT_MAX] {};

	bool _gyro_blocked[GYRO_COUNT_MAX] {};
	bool _gyro_stuck[GYRO_COUNT_MAX] {};
	sensor_gyro_fifo_s _last_gyro_fifo{};
	matrix::Vector3f _last_gyro[GYRO_COUNT_MAX] {};

	bool _baro_blocked{false};
	bool _baro_stuck{false};

	bool _mag_blocked[MAG_COUNT_MAX] {};
	bool _mag_stuck[MAG_COUNT_MAX] {};

	bool _gps_blocked{false};
	bool _airspeed_blocked{false};
	bool _vio_blocked{false};

	float _last_magx[MAG_COUNT_MAX] {};
	float _last_magy[MAG_COUNT_MAX] {};
	float _last_magz[MAG_COUNT_MAX] {};

	float _last_baro_pressure{0.0f};
	float _last_baro_temperature{0.0f};

	int32_t _output_functions[actuator_outputs_s::NUM_ACTUATOR_OUTPUTS] {};

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	px4::atomic<bool> _has_initialized {false};
#endif

	int _lockstep_component{-1};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MAV_TYPE>) _param_mav_type,
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id,
		(ParamInt<px4::params::MAV_COMP_ID>) _param_mav_comp_id
	)
};
