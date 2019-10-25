/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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


/**
 * @file simulator.h
 *
 * This module interfaces via MAVLink to a software in the loop simulator (SITL)
 * such as jMAVSim or Gazebo.
 */

#pragma once

#include <battery/battery.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/ecl/geo/geo.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/irlock_report.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>

#include <v2.0/common/mavlink.h>
#include <v2.0/mavlink_types.h>

namespace simulator
{

#pragma pack(push, 1)
struct RawGPSData {
	uint64_t timestamp;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	uint16_t eph;
	uint16_t epv;
	uint16_t vel;
	int16_t vn;
	int16_t ve;
	int16_t vd;
	uint16_t cog;
	uint8_t fix_type;
	uint8_t satellites_visible;
};
#pragma pack(pop)

template <typename RType> class Report
{
public:
	Report(int readers) :
		_readidx(0),
		_max_readers(readers),
		_report_len(sizeof(RType))
	{
		memset(_buf, 0, sizeof(_buf));
		px4_sem_init(&_lock, 0, _max_readers);
	}

	~Report() {}

	bool copyData(void *outbuf, int len)
	{
		if (len != _report_len) {
			return false;
		}

		read_lock();
		memcpy(outbuf, &_buf[_readidx], _report_len);
		read_unlock();
		return true;
	}
	void writeData(void *inbuf)
	{
		write_lock();
		memcpy(&_buf[!_readidx], inbuf, _report_len);
		_readidx = !_readidx;
		write_unlock();
	}

protected:
	void read_lock() { px4_sem_wait(&_lock); }
	void read_unlock() { px4_sem_post(&_lock); }
	void write_lock()
	{
		for (int i = 0; i < _max_readers; i++) {
			px4_sem_wait(&_lock);
		}
	}
	void write_unlock()
	{
		for (int i = 0; i < _max_readers; i++) {
			px4_sem_post(&_lock);
		}
	}

	int _readidx;
	px4_sem_t _lock;
	const int _max_readers;
	const int _report_len;
	RType _buf[2];
};

} // namespace simulator


class Simulator : public ModuleParams
{
public:
	static Simulator *getInstance();

	enum class InternetProtocol {
		TCP,
		UDP
	};

	static int start(int argc, char *argv[]);

	bool getGPSSample(uint8_t *buf, int len);

	void write_gps_data(void *buf);

	void set_ip(InternetProtocol ip);
	void set_port(unsigned port);

private:
	Simulator() :
		ModuleParams(nullptr)
	{
		simulator::RawGPSData gps_data{};
		gps_data.eph = UINT16_MAX;
		gps_data.epv = UINT16_MAX;

		_gps.writeData(&gps_data);

		_battery_status.timestamp = hrt_absolute_time();

		_px4_accel.set_sample_rate(250);
		_px4_gyro.set_sample_rate(250);
	}

	~Simulator()
	{
		// free perf counters
		perf_free(_perf_gps);
		perf_free(_perf_sim_delay);
		perf_free(_perf_sim_interval);

		_instance = nullptr;
	}

	// class methods

	int publish_flow_topic(const mavlink_hil_optical_flow_t *flow);
	int publish_odometry_topic(const mavlink_message_t *odom_mavlink);
	int publish_distance_topic(const mavlink_distance_sensor_t *dist);

	static Simulator *_instance;

	// simulated sensor instances
	PX4Accelerometer	_px4_accel{1311244, ORB_PRIO_DEFAULT, ROTATION_NONE}; // 1311244: DRV_ACC_DEVTYPE_ACCELSIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	PX4Gyroscope		_px4_gyro{2294028, ORB_PRIO_DEFAULT, ROTATION_NONE}; // 2294028: DRV_GYR_DEVTYPE_GYROSIM, BUS: 1, ADDR: 2, TYPE: SIMULATION
	PX4Magnetometer		_px4_mag{197388, ORB_PRIO_DEFAULT, ROTATION_NONE}; // 197388: DRV_MAG_DEVTYPE_MAGSIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
	PX4Barometer		_px4_baro{6620172, ORB_PRIO_DEFAULT}; // 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION

	simulator::Report<simulator::RawGPSData>	_gps{1};

	perf_counter_t _perf_gps{perf_alloc_once(PC_ELAPSED, "sim_gps_delay")};
	perf_counter_t _perf_sim_delay{perf_alloc_once(PC_ELAPSED, "sim_network_delay")};
	perf_counter_t _perf_sim_interval{perf_alloc(PC_INTERVAL, "sim_network_interval")};

	// uORB publisher handlers
	orb_advert_t _battery_pub{nullptr};
	orb_advert_t _differential_pressure_pub{nullptr};
	orb_advert_t _dist_pub{nullptr};
	orb_advert_t _flow_pub{nullptr};
	orb_advert_t _irlock_report_pub{nullptr};
	orb_advert_t _visual_odometry_pub{nullptr};

	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};

	unsigned int _port{14560};

	InternetProtocol _ip{InternetProtocol::UDP};

	double _realtime_factor{1.0};		///< How fast the simulation runs in comparison to real system time

	hrt_abstime _last_sim_timestamp{0};
	hrt_abstime _last_sitl_timestamp{0};

	// Lib used to do the battery calculations.
	Battery _battery {};
	battery_status_s _battery_status{};

#ifndef __PX4_QURT

	mavlink_hil_actuator_controls_t actuator_controls_from_outputs(const actuator_outputs_s &actuators);

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
	void poll_topics();
	void poll_for_MAVLink_messages();
	void request_hil_state_quaternion();
	void send();
	void send_controls();
	void send_heartbeat();
	void send_mavlink_message(const mavlink_message_t &aMsg);
	void update_sensors(const hrt_abstime &time, const mavlink_hil_sensor_t &imu);
	void update_gps(const mavlink_hil_gps_t *gps_sim);

	static void *sending_trampoline(void *);

	// uORB publisher handlers
	orb_advert_t _vehicle_angular_velocity_pub{nullptr};
	orb_advert_t _attitude_pub{nullptr};
	orb_advert_t _gpos_pub{nullptr};
	orb_advert_t _lpos_pub{nullptr};
	orb_advert_t _rc_channels_pub{nullptr};

	// uORB subscription handlers
	int _actuator_outputs_sub{-1};
	int _vehicle_status_sub{-1};

	// hil map_ref data
	struct map_projection_reference_s _hil_local_proj_ref {};

	bool _hil_local_proj_inited{false};

	double _hil_ref_lat{0};
	double _hil_ref_lon{0};
	float _hil_ref_alt{0.0f};
	uint64_t _hil_ref_timestamp{0};

	// uORB data containers
	input_rc_s _rc_input {};
	manual_control_setpoint_s _manual {};
	vehicle_attitude_s _attitude {};
	vehicle_status_s _vehicle_status {};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SIM_BAT_DRAIN>) _param_sim_bat_drain, ///< battery drain interval
		(ParamFloat<px4::params::SIM_BAT_MIN_PCT>) _battery_min_percentage, //< minimum battery percentage
		(ParamInt<px4::params::MAV_TYPE>) _param_mav_type,
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id,
		(ParamInt<px4::params::MAV_COMP_ID>) _param_mav_comp_id
	)

#endif
};
