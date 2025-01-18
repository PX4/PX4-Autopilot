/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (c) 2016 Anton Matosov. All rights reserved.
 *   Copyright (c) 2017-2020 PX4 Development Team. All rights reserved.
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

#include "SimulatorMavlink.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/tasks.h>
#include <lib/geo/geo.h>
#include <drivers/device/Device.hpp>
#include <drivers/drv_pwm_output.h>
#include <conversion/rotation.h>
#include <mathlib/mathlib.h>
#include <lib/drivers/device/Device.hpp>

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <sys/socket.h>
#include <termios.h>
#include <arpa/inet.h>

#include <limits>

static int _fd;
static unsigned char _buf[2048];
static sockaddr_in _srcaddr;
static unsigned _addrlen = sizeof(_srcaddr);

const unsigned mode_flag_armed = 128;
const unsigned mode_flag_custom = 1;

using namespace time_literals;

static px4_task_t g_sim_task = -1;

SimulatorMavlink *SimulatorMavlink::_instance = nullptr;

static constexpr vehicle_odometry_s vehicle_odometry_empty {
	.timestamp = 0,
	.timestamp_sample = 0,
	.position = {NAN, NAN, NAN},
	.q = {NAN, NAN, NAN, NAN},
	.velocity = {NAN, NAN, NAN},
	.angular_velocity = {NAN, NAN, NAN},
	.position_variance = {NAN, NAN, NAN},
	.orientation_variance = {NAN, NAN, NAN},
	.velocity_variance = {NAN, NAN, NAN},
	.pose_frame = vehicle_odometry_s::POSE_FRAME_UNKNOWN,
	.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_UNKNOWN,
	.reset_counter = 0,
	.quality = 0
};

SimulatorMavlink::SimulatorMavlink() :
	ModuleParams(nullptr)
{
	for (int i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; ++i) {
		char param_name[17];
		snprintf(param_name, sizeof(param_name), "%s_%s%d", "PWM_MAIN", "FUNC", i + 1);
		param_get(param_find(param_name), &_output_functions[i]);
	}

	_esc_status_pub.advertise();
}

void SimulatorMavlink::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

void SimulatorMavlink::actuator_controls_from_outputs(mavlink_hil_actuator_controls_t *msg)
{
	memset(msg, 0, sizeof(mavlink_hil_actuator_controls_t));

	msg->time_usec = hrt_absolute_time();

	bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	if (armed) {
		for (unsigned i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
			msg->controls[i] = _actuator_outputs.output[i];
		}
	}

	msg->mode = mode_flag_custom;
	msg->mode |= (armed) ? mode_flag_armed : 0;
	msg->flags = 0;

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	msg->flags |= 1;
#endif
}

void SimulatorMavlink::send_esc_telemetry(mavlink_hil_actuator_controls_t hil_act_control)
{
	esc_status_s esc_status{};
	esc_status.timestamp = hrt_absolute_time();
	const int max_esc_count = math::min(actuator_outputs_s::NUM_ACTUATOR_OUTPUTS, esc_status_s::CONNECTED_ESC_MAX);

	const bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	int max_esc_index = 0;

	for (int i = 0; i < max_esc_count; i++) {
		if (_output_functions[i] != 0) {
			max_esc_index = i;
		}

		esc_status.esc[i].actuator_function = _output_functions[i]; // TODO: this should be in pwm_sim...
		esc_status.esc[i].timestamp = esc_status.timestamp;
		esc_status.esc[i].esc_errorcount = 0; // TODO
		esc_status.esc[i].esc_voltage = _battery_status.voltage_v;
		esc_status.esc[i].esc_current = armed ? 1.0f + math::abs_t(hil_act_control.controls[i]) * 15.0f :
						0.0f; // TODO: magic number
		esc_status.esc[i].esc_rpm = hil_act_control.controls[i] * 6000;  // TODO: magic number
		esc_status.esc[i].esc_temperature = 20.0 + math::abs_t(hil_act_control.controls[i]) * 40.0;
	}

	esc_status.esc_count = max_esc_index + 1;
	esc_status.esc_armed_flags = (1u << esc_status.esc_count) - 1;
	esc_status.esc_online_flags = (1u << esc_status.esc_count) - 1;

	_esc_status_pub.publish(esc_status);
}

void SimulatorMavlink::send_controls()
{
	orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuator_outputs);

	if (_actuator_outputs.timestamp > 0) {
		mavlink_hil_actuator_controls_t hil_act_control;
		actuator_controls_from_outputs(&hil_act_control);

		mavlink_message_t message{};
		mavlink_msg_hil_actuator_controls_encode(_param_mav_sys_id.get(), _param_mav_comp_id.get(), &message, &hil_act_control);

		PX4_DEBUG("sending controls t=%ld (%ld)", _actuator_outputs.timestamp, hil_act_control.time_usec);

		send_mavlink_message(message);

		send_esc_telemetry(hil_act_control);
	}
}

void SimulatorMavlink::update_sensors(const hrt_abstime &time, const mavlink_hil_sensor_t &sensors)
{
	// temperature only updated with baro
	if ((sensors.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		if (PX4_ISFINITE(sensors.temperature)) {
			_sensors_temperature = sensors.temperature;
		}
	}

	// accel
	if ((sensors.fields_updated & SensorSource::ACCEL) == SensorSource::ACCEL) {
		if (sensors.id >= ACCEL_COUNT_MAX) {
			PX4_ERR("Number of simulated accelerometer %d out of range. Max: %d", sensors.id, ACCEL_COUNT_MAX);
			return;
		}

		if (sensors.id == 0) {
			// accel 0 is simulated FIFO
			static constexpr float ACCEL_FIFO_SCALE = CONSTANTS_ONE_G / 2048.f;
			static constexpr float ACCEL_FIFO_RANGE = 16.f * CONSTANTS_ONE_G;

			_px4_accel[sensors.id].set_scale(ACCEL_FIFO_SCALE);
			_px4_accel[sensors.id].set_range(ACCEL_FIFO_RANGE);

			if (_accel_stuck[sensors.id]) {
				_px4_accel[sensors.id].updateFIFO(_last_accel_fifo);

			} else if (!_accel_blocked[sensors.id]) {
				_px4_accel[sensors.id].set_temperature(_sensors_temperature);

				_last_accel_fifo.samples = 1;
				_last_accel_fifo.dt = time - _last_accel_fifo.timestamp_sample;
				_last_accel_fifo.timestamp_sample = time;
				_last_accel_fifo.x[0] = sensors.xacc / ACCEL_FIFO_SCALE;
				_last_accel_fifo.y[0] = sensors.yacc / ACCEL_FIFO_SCALE;
				_last_accel_fifo.z[0] = sensors.zacc / ACCEL_FIFO_SCALE;

				_px4_accel[sensors.id].updateFIFO(_last_accel_fifo);
			}

		} else {
			if (_accel_stuck[sensors.id]) {
				_px4_accel[sensors.id].update(time, _last_accel[sensors.id](0), _last_accel[sensors.id](1), _last_accel[sensors.id](2));

			} else if (!_accel_blocked[sensors.id]) {
				_px4_accel[sensors.id].set_temperature(_sensors_temperature);
				_px4_accel[sensors.id].update(time, sensors.xacc, sensors.yacc, sensors.zacc);
				_last_accel[sensors.id] = matrix::Vector3f{sensors.xacc, sensors.yacc, sensors.zacc};
			}
		}
	}

	// gyro
	if ((sensors.fields_updated & SensorSource::GYRO) == SensorSource::GYRO) {
		if (sensors.id >= GYRO_COUNT_MAX) {
			PX4_ERR("Number of simulated gyroscope %d out of range. Max: %d", sensors.id, GYRO_COUNT_MAX);
			return;
		}

		if (sensors.id == 0) {
			// gyro 0 is simulated FIFO
			static constexpr float GYRO_FIFO_SCALE = math::radians(2000.f / 32768.f);
			static constexpr float GYRO_FIFO_RANGE = math::radians(2000.f);

			_px4_gyro[sensors.id].set_scale(GYRO_FIFO_SCALE);
			_px4_gyro[sensors.id].set_range(GYRO_FIFO_RANGE);

			if (_gyro_stuck[sensors.id]) {
				_px4_gyro[sensors.id].updateFIFO(_last_gyro_fifo);

			} else if (!_gyro_blocked[sensors.id]) {
				_px4_gyro[sensors.id].set_temperature(_sensors_temperature);

				_last_gyro_fifo.samples = 1;
				_last_gyro_fifo.dt = time - _last_gyro_fifo.timestamp_sample;
				_last_gyro_fifo.timestamp_sample = time;
				_last_gyro_fifo.x[0] = sensors.xgyro / GYRO_FIFO_SCALE;
				_last_gyro_fifo.y[0] = sensors.ygyro / GYRO_FIFO_SCALE;
				_last_gyro_fifo.z[0] = sensors.zgyro / GYRO_FIFO_SCALE;

				_px4_gyro[sensors.id].updateFIFO(_last_gyro_fifo);
			}

		} else {
			if (_gyro_stuck[sensors.id]) {
				_px4_gyro[sensors.id].update(time, _last_gyro[sensors.id](0), _last_gyro[sensors.id](1), _last_gyro[sensors.id](2));

			} else if (!_gyro_blocked[sensors.id]) {
				_px4_gyro[sensors.id].set_temperature(_sensors_temperature);
				_px4_gyro[sensors.id].update(time, sensors.xgyro, sensors.ygyro, sensors.zgyro);
				_last_gyro[sensors.id] = matrix::Vector3f{sensors.xgyro, sensors.ygyro, sensors.zgyro};
			}
		}
	}

	// magnetometer
	if ((sensors.fields_updated & SensorSource::MAG) == SensorSource::MAG) {
		if (sensors.id >= MAG_COUNT_MAX) {
			PX4_ERR("Number of simulated magnetometer %d out of range. Max: %d", sensors.id, MAG_COUNT_MAX);
			return;
		}

		if (_mag_stuck[sensors.id]) {
			_px4_mag[sensors.id].update(time, _last_magx[sensors.id], _last_magy[sensors.id], _last_magz[sensors.id]);

		} else if (!_mag_blocked[sensors.id]) {
			_px4_mag[sensors.id].set_temperature(_sensors_temperature);
			_px4_mag[sensors.id].update(time, sensors.xmag, sensors.ymag, sensors.zmag);
			_last_magx[sensors.id] = sensors.xmag;
			_last_magy[sensors.id] = sensors.ymag;
			_last_magz[sensors.id] = sensors.zmag;
		}
	}

	// baro
	if ((sensors.fields_updated & SensorSource::BARO) == SensorSource::BARO && !_baro_blocked) {

		if (!_baro_stuck) {
			_last_baro_pressure = sensors.abs_pressure * 100.f; // hPa to Pa
			_last_baro_temperature = sensors.temperature;
		}

		// publish
		sensor_baro_s sensor_baro{};
		sensor_baro.timestamp_sample = time;
		sensor_baro.pressure = _last_baro_pressure;
		sensor_baro.temperature = _last_baro_temperature;

		// publish 1st baro
		sensor_baro.device_id = 6620172; // 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
		sensor_baro.timestamp = hrt_absolute_time();
		_sensor_baro_pubs[0].publish(sensor_baro);

		// publish 2nd baro
		sensor_baro.device_id = 6620428; // 6620428: DRV_BARO_DEVTYPE_BAROSIM, BUS: 2, ADDR: 4, TYPE: SIMULATION
		sensor_baro.timestamp = hrt_absolute_time();
		_sensor_baro_pubs[1].publish(sensor_baro);
	}

	// differential pressure
	if ((sensors.fields_updated & SensorSource::DIFF_PRESS) == SensorSource::DIFF_PRESS && !_airspeed_disconnected) {

		const float blockage_fraction = 0.7; // defines max blockage (fully ramped)
		const float airspeed_blockage_rampup_time = 1_s; // time it takes to go max blockage, linear ramp

		float airspeed_blockage_scale = 1.f;

		if (_airspeed_blocked_timestamp > 0) {
			airspeed_blockage_scale = math::constrain(1.f - (hrt_absolute_time() - _airspeed_blocked_timestamp) /
						  airspeed_blockage_rampup_time, 1.f - blockage_fraction, 1.f);
		}

		differential_pressure_s report{};
		report.timestamp_sample = time;
		report.device_id = 1377548; // 1377548: DRV_DIFF_PRESS_DEVTYPE_SIM, BUS: 1, ADDR: 5, TYPE: SIMULATION
		report.differential_pressure_pa = sensors.diff_pressure * 100.f * airspeed_blockage_scale; // hPa to Pa;
		report.temperature = _sensors_temperature;
		report.timestamp = hrt_absolute_time();
		_differential_pressure_pub.publish(report);
	}
}

void SimulatorMavlink::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HIL_SENSOR:
		handle_message_hil_sensor(msg);
		break;

	case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
		handle_message_optical_flow(msg);
		break;

	case MAVLINK_MSG_ID_ODOMETRY:
		handle_message_odometry(msg);
		break;

	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		handle_message_vision_position_estimate(msg);
		break;

	case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		handle_message_distance_sensor(msg);
		break;

	case MAVLINK_MSG_ID_HIL_GPS:
		handle_message_hil_gps(msg);
		break;

	case MAVLINK_MSG_ID_RC_CHANNELS:
		handle_message_rc_channels(msg);
		break;

	case MAVLINK_MSG_ID_LANDING_TARGET:
		handle_message_landing_target(msg);
		break;

	case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
		handle_message_hil_state_quaternion(msg);
		break;

	case MAVLINK_MSG_ID_RAW_RPM:
		mavlink_raw_rpm_t rpm_mavlink;
		mavlink_msg_raw_rpm_decode(msg, &rpm_mavlink);
		rpm_s rpm_uorb{};
		rpm_uorb.timestamp = hrt_absolute_time();
		rpm_uorb.rpm_estimate = rpm_mavlink.frequency;
		_rpm_pub.publish(rpm_uorb);
		break;
	}
}

void SimulatorMavlink::handle_message_distance_sensor(const mavlink_message_t *msg)
{
	mavlink_distance_sensor_t dist;
	mavlink_msg_distance_sensor_decode(msg, &dist);
	publish_distance_topic(&dist);
}

void SimulatorMavlink::handle_message_hil_gps(const mavlink_message_t *msg)
{
	mavlink_hil_gps_t hil_gps;
	mavlink_msg_hil_gps_decode(msg, &hil_gps);

	if (!_gps_blocked) {
		sensor_gps_s gps{};

		gps.latitude_deg = hil_gps.lat / 1e7;
		gps.longitude_deg = hil_gps.lon / 1e7;
		gps.altitude_msl_m = hil_gps.alt / 1e3;
		gps.altitude_ellipsoid_m = hil_gps.alt / 1e3;

		gps.s_variance_m_s = 0.25f;
		gps.c_variance_rad = 0.5f;
		gps.fix_type = hil_gps.fix_type;

		gps.eph = (float)hil_gps.eph * 1e-2f; // cm -> m
		gps.epv = (float)hil_gps.epv * 1e-2f; // cm -> m

		gps.hdop = 0; // TODO
		gps.vdop = 0; // TODO

		gps.noise_per_ms = 0;
		gps.automatic_gain_control = 0;
		gps.jamming_indicator = 0;
		gps.jamming_state = 0;
		gps.spoofing_state = 0;

		gps.vel_m_s = (float)(hil_gps.vel) / 100.0f; // cm/s -> m/s
		gps.vel_n_m_s = (float)(hil_gps.vn) / 100.0f; // cm/s -> m/s
		gps.vel_e_m_s = (float)(hil_gps.ve) / 100.0f; // cm/s -> m/s
		gps.vel_d_m_s = (float)(hil_gps.vd) / 100.0f; // cm/s -> m/s
		gps.cog_rad = ((hil_gps.cog == 65535) ? NAN : matrix::wrap_2pi(math::radians(hil_gps.cog * 1e-2f))); // cdeg -> rad
		gps.vel_ned_valid = true;

		gps.timestamp_time_relative = 0;
		gps.time_utc_usec = hil_gps.time_usec;

		gps.satellites_used = hil_gps.satellites_visible;

		gps.heading = NAN;
		gps.heading_offset = NAN;

		gps.timestamp = hrt_absolute_time();

		// New publishers will be created based on the HIL_GPS ID's being different or not
		for (size_t i = 0; i < sizeof(_gps_ids) / sizeof(_gps_ids[0]); i++) {
			if (_sensor_gps_pubs[i] && _gps_ids[i] == hil_gps.id) {
				_sensor_gps_pubs[i]->publish(gps);
				break;
			}

			if (_sensor_gps_pubs[i] == nullptr) {
				_sensor_gps_pubs[i] = new uORB::PublicationMulti<sensor_gps_s> {ORB_ID(sensor_gps)};
				_gps_ids[i] = hil_gps.id;

				device::Device::DeviceId device_id;
				device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
				device_id.devid_s.bus = 0;
				device_id.devid_s.address = i;
				device_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;
				gps.device_id = device_id.devid;

				_sensor_gps_pubs[i]->publish(gps);
				break;
			}
		}
	}
}

void SimulatorMavlink::handle_message_hil_sensor(const mavlink_message_t *msg)
{
	mavlink_hil_sensor_t imu;
	mavlink_msg_hil_sensor_decode(msg, &imu);

	// Assume imu with id 0 is the primary imu an base lockstep based on this.
	if (imu.id == 0) {
		if (_lockstep_component == -1) {
			_lockstep_component = px4_lockstep_register_component();
		}

		struct timespec ts;

		abstime_to_ts(&ts, imu.time_usec);

		px4_clock_settime(CLOCK_MONOTONIC, &ts);
	}

	hrt_abstime now_us = hrt_absolute_time();

#if 0
	// This is just for to debug missing HIL_SENSOR messages.
	static hrt_abstime last_time = 0;
	hrt_abstime diff = now_us - last_time;
	float step = diff / 4000.0f;

	if (step > 1.1f || step < 0.9f) {
		PX4_INFO("HIL_SENSOR: imu time_usec: %lu, time_usec: %lu, diff: %lu, step: %.2f", imu.time_usec, now_us, diff, step);
	}

	last_time = now_us;
#endif

	update_sensors(now_us, imu);

	if (imu.id == 0) {
#if defined(ENABLE_LOCKSTEP_SCHEDULER)

		if (!_has_initialized.load()) {
			_has_initialized.store(true);
		}

#endif

		px4_lockstep_progress(_lockstep_component);
	}
}

void SimulatorMavlink::handle_message_hil_state_quaternion(const mavlink_message_t *msg)
{
	mavlink_hil_state_quaternion_t hil_state;
	mavlink_msg_hil_state_quaternion_decode(msg, &hil_state);

	uint64_t timestamp = hrt_absolute_time();

	/* angular velocity */
	vehicle_angular_velocity_s hil_angular_velocity{};
	{
		hil_angular_velocity.timestamp = timestamp;

		hil_angular_velocity.xyz[0] = hil_state.rollspeed;
		hil_angular_velocity.xyz[1] = hil_state.pitchspeed;
		hil_angular_velocity.xyz[2] = hil_state.yawspeed;

		// always publish ground truth attitude message
		_vehicle_angular_velocity_ground_truth_pub.publish(hil_angular_velocity);
	}

	/* attitude */
	vehicle_attitude_s hil_attitude{};
	{
		hil_attitude.timestamp = timestamp;

		matrix::Quatf q(hil_state.attitude_quaternion);
		q.copyTo(hil_attitude.q);

		// always publish ground truth attitude message
		_attitude_ground_truth_pub.publish(hil_attitude);
	}

	/* global position */
	vehicle_global_position_s hil_gpos{};
	{
		hil_gpos.timestamp = timestamp;

		hil_gpos.lat = hil_state.lat / 1E7;//1E7
		hil_gpos.lon = hil_state.lon / 1E7;//1E7
		hil_gpos.alt = hil_state.alt / 1E3;//1E3

		// always publish ground truth attitude message
		_gpos_ground_truth_pub.publish(hil_gpos);
	}

	/* local position */
	vehicle_local_position_s hil_lpos{};
	{
		hil_lpos.timestamp = timestamp;

		double lat = hil_state.lat * 1e-7;
		double lon = hil_state.lon * 1e-7;

		if (!_global_local_proj_ref.isInitialized()) {
			_global_local_proj_ref.initReference(lat, lon, timestamp);
			_global_local_alt0 = hil_state.alt / 1000.f;
		}

		hil_lpos.timestamp = timestamp;
		hil_lpos.xy_valid = true;
		hil_lpos.z_valid = true;
		hil_lpos.v_xy_valid = true;
		hil_lpos.v_z_valid = true;
		_global_local_proj_ref.project(lat, lon, hil_lpos.x, hil_lpos.y);
		hil_lpos.z = _global_local_alt0 - hil_state.alt / 1000.0f;
		hil_lpos.vx = hil_state.vx / 100.0f;
		hil_lpos.vy = hil_state.vy / 100.0f;
		hil_lpos.vz = hil_state.vz / 100.0f;
		matrix::Eulerf euler = matrix::Quatf(hil_attitude.q);
		matrix::Vector3f acc(hil_state.xacc / 1000.f, hil_state.yacc / 1000.f,  hil_state.zacc / 1000.f);
		hil_lpos.ax = acc(0);
		hil_lpos.ay = acc(1);
		hil_lpos.az = acc(2);
		hil_lpos.heading = euler.psi();
		hil_lpos.xy_global = true;
		hil_lpos.z_global = true;
		hil_lpos.ref_timestamp = _global_local_proj_ref.getProjectionReferenceTimestamp();
		hil_lpos.ref_lat = _global_local_proj_ref.getProjectionReferenceLat();
		hil_lpos.ref_lon = _global_local_proj_ref.getProjectionReferenceLon();
		hil_lpos.ref_alt = _global_local_alt0;
		hil_lpos.vxy_max = std::numeric_limits<float>::infinity();
		hil_lpos.vz_max = std::numeric_limits<float>::infinity();
		hil_lpos.hagl_min = std::numeric_limits<float>::infinity();
		hil_lpos.hagl_max = std::numeric_limits<float>::infinity();

		// always publish ground truth attitude message
		_lpos_ground_truth_pub.publish(hil_lpos);
	}
}

void SimulatorMavlink::handle_message_landing_target(const mavlink_message_t *msg)
{
	mavlink_landing_target_t landing_target_mavlink;
	mavlink_msg_landing_target_decode(msg, &landing_target_mavlink);

	if (landing_target_mavlink.position_valid) {
		PX4_WARN("Only landing targets relative to captured images are supported");

	} else {
		irlock_report_s report{};
		report.timestamp = hrt_absolute_time();
		report.signature = landing_target_mavlink.target_num;
		report.pos_x = landing_target_mavlink.angle_x;
		report.pos_y = landing_target_mavlink.angle_y;
		report.size_x = landing_target_mavlink.size_x;
		report.size_y = landing_target_mavlink.size_y;

		_irlock_report_pub.publish(report);
	}
}

void SimulatorMavlink::handle_message_odometry(const mavlink_message_t *msg)
{
	mavlink_odometry_t odom_in;
	mavlink_msg_odometry_decode(msg, &odom_in);

	// fill vehicle_odometry from Mavlink ODOMETRY
	vehicle_odometry_s odom{vehicle_odometry_empty};

	odom.timestamp_sample = hrt_absolute_time(); // _mavlink_timesync.sync_stamp(odom_in.time_usec);

	// position x/y/z (m)
	matrix::Vector3f odom_in_p(odom_in.x, odom_in.y, odom_in.z);

	if (odom_in_p.isAllFinite()) {
		// frame_id: Coordinate frame of reference for the pose data.
		switch (odom_in.frame_id) {
		case MAV_FRAME_LOCAL_NED:
			// NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
			odom_in_p.copyTo(odom.position);
			break;

		case MAV_FRAME_LOCAL_ENU:
			// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
			odom.position[0] =  odom_in.y; // y: North
			odom.position[1] =  odom_in.x; // x: East
			odom.position[2] = -odom_in.z; // z: Up
			break;

		case MAV_FRAME_LOCAL_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_FRD;
			odom_in_p.copyTo(odom.position);
			break;

		case MAV_FRAME_LOCAL_FLU:
			// FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_FRD;
			odom.position[0] =  odom_in.x; // x: Forward
			odom.position[1] = -odom_in.y; // y: Left
			odom.position[2] = -odom_in.z; // z: Up
			break;

		default:
			break;
		}

		// pose_covariance
		//  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw)
		//  first six entries are the first ROW, next five entries are the second ROW, etc.
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			switch (odom_in.frame_id) {
			case MAV_FRAME_LOCAL_NED:
			case MAV_FRAME_LOCAL_FRD:
			case MAV_FRAME_LOCAL_FLU:
				// position variances copied directly
				odom.position_variance[0] = odom_in.pose_covariance[0];  // X  row 0, col 0
				odom.position_variance[1] = odom_in.pose_covariance[6];  // Y  row 1, col 1
				odom.position_variance[2] = odom_in.pose_covariance[11]; // Z  row 2, col 2
				break;

			case MAV_FRAME_LOCAL_ENU:
				// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
				odom.position_variance[0] = odom_in.pose_covariance[6];  // Y  row 1, col 1
				odom.position_variance[1] = odom_in.pose_covariance[0];  // X  row 0, col 0
				odom.position_variance[2] = odom_in.pose_covariance[11]; // Z  row 2, col 2
				break;

			default:
				break;
			}
		}
	}

	// q: the quaternion of the ODOMETRY msg represents a rotation from body frame to a local frame
	if (matrix::Quatf(odom_in.q).isAllFinite()) {
		odom.q[0] = odom_in.q[0];
		odom.q[1] = odom_in.q[1];
		odom.q[2] = odom_in.q[2];
		odom.q[3] = odom_in.q[3];

		// pose_covariance (roll, pitch, yaw)
		//  states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.
		//  TODO: fix pose_covariance for MAV_FRAME_LOCAL_ENU, MAV_FRAME_LOCAL_FLU
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			odom.orientation_variance[0] = odom_in.pose_covariance[15]; // R  row 3, col 3
			odom.orientation_variance[1] = odom_in.pose_covariance[18]; // P  row 4, col 4
			odom.orientation_variance[2] = odom_in.pose_covariance[20]; // Y  row 5, col 5
		}
	}

	// velocity vx/vy/vz (m/s)
	matrix::Vector3f odom_in_v(odom_in.vx, odom_in.vy, odom_in.vz);

	if (odom_in_v.isAllFinite()) {
		// child_frame_id: Coordinate frame of reference for the velocity in free space (twist) data.
		switch (odom_in.child_frame_id) {
		case MAV_FRAME_LOCAL_NED:
			// NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
			odom_in_v.copyTo(odom.velocity);
			break;

		case MAV_FRAME_LOCAL_ENU:
			// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
			odom.velocity[0] =  odom_in.vy; // y: North
			odom.velocity[1] =  odom_in.vx; // x: East
			odom.velocity[2] = -odom_in.vz; // z: Up
			break;

		case MAV_FRAME_LOCAL_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_FRD;
			odom_in_v.copyTo(odom.velocity);
			break;

		case MAV_FRAME_LOCAL_FLU:
			// FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_FRD;
			odom.velocity[0] =  odom_in.vx; // x: Forward
			odom.velocity[1] = -odom_in.vy; // y: Left
			odom.velocity[2] = -odom_in.vz; // z: Up
			break;

		case MAV_FRAME_BODY_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
		case MAV_FRAME_BODY_OFFSET_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
		case MAV_FRAME_BODY_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin that travels with vehicle.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD;
			odom_in_v.copyTo(odom.velocity);
			break;

		default:
			// unsupported child_frame_id
			break;
		}

		// velocity_covariance (vx, vy, vz)
		//  states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.
		//  TODO: fix velocity_covariance for MAV_FRAME_LOCAL_ENU, MAV_FRAME_LOCAL_FLU, MAV_FRAME_LOCAL_FLU
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			switch (odom_in.child_frame_id) {
			case MAV_FRAME_LOCAL_NED:
			case MAV_FRAME_LOCAL_FRD:
			case MAV_FRAME_LOCAL_FLU:
			case MAV_FRAME_BODY_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
			case MAV_FRAME_BODY_OFFSET_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
			case MAV_FRAME_BODY_FRD:
				// velocity covariances copied directly
				odom.velocity_variance[0] = odom_in.velocity_covariance[0];  // X  row 0, col 0
				odom.velocity_variance[1] = odom_in.velocity_covariance[6];  // Y  row 1, col 1
				odom.velocity_variance[2] = odom_in.velocity_covariance[11]; // Z  row 2, col 2
				break;

			case MAV_FRAME_LOCAL_ENU:
				// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
				odom.velocity_variance[0] = odom_in.velocity_covariance[6];  // Y  row 1, col 1
				odom.velocity_variance[1] = odom_in.velocity_covariance[0];  // X  row 0, col 0
				odom.velocity_variance[2] = odom_in.velocity_covariance[11]; // Z  row 2, col 2
				break;

			default:
				// unsupported child_frame_id
				break;
			}
		}
	}

	// Roll/Pitch/Yaw angular speed (rad/s)
	if (PX4_ISFINITE(odom_in.rollspeed)
	    && PX4_ISFINITE(odom_in.pitchspeed)
	    && PX4_ISFINITE(odom_in.yawspeed)) {

		odom.angular_velocity[0] = odom_in.rollspeed;
		odom.angular_velocity[1] = odom_in.pitchspeed;
		odom.angular_velocity[2] = odom_in.yawspeed;
	}

	odom.reset_counter = odom_in.reset_counter;
	odom.quality = odom_in.quality;

	switch (odom_in.estimator_type) {
	case MAV_ESTIMATOR_TYPE_UNKNOWN: // accept MAV_ESTIMATOR_TYPE_UNKNOWN for legacy support
	case MAV_ESTIMATOR_TYPE_NAIVE:
	case MAV_ESTIMATOR_TYPE_VISION:
	case MAV_ESTIMATOR_TYPE_VIO:
		if (!_vio_blocked) {
			odom.timestamp = hrt_absolute_time();
			_visual_odometry_pub.publish(odom);
		}

		break;

	case MAV_ESTIMATOR_TYPE_MOCAP:
		odom.timestamp = hrt_absolute_time();
		_mocap_odometry_pub.publish(odom);
		break;

	case MAV_ESTIMATOR_TYPE_GPS:
	case MAV_ESTIMATOR_TYPE_GPS_INS:
	case MAV_ESTIMATOR_TYPE_LIDAR:
	case MAV_ESTIMATOR_TYPE_AUTOPILOT:
	default:
		PX4_ERR("ODOMETRY: estimator_type %" PRIu8 " unsupported", odom_in.estimator_type);
		return;
	}
}

void SimulatorMavlink::handle_message_optical_flow(const mavlink_message_t *msg)
{
	mavlink_hil_optical_flow_t flow;
	mavlink_msg_hil_optical_flow_decode(msg, &flow);

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = 0;
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_FLOW_DEVTYPE_SIM;

	sensor_optical_flow_s sensor_optical_flow{};

	sensor_optical_flow.timestamp_sample = hrt_absolute_time();
	sensor_optical_flow.device_id = device_id.devid;

	sensor_optical_flow.pixel_flow[0] = flow.integrated_x;
	sensor_optical_flow.pixel_flow[1] = flow.integrated_y;

	sensor_optical_flow.integration_timespan_us = flow.integration_time_us;
	sensor_optical_flow.quality = flow.quality;

	matrix::Vector3f integrated_gyro(flow.integrated_xgyro, flow.integrated_ygyro, flow.integrated_zgyro);

	if (integrated_gyro.isAllFinite()) {
		integrated_gyro.copyTo(sensor_optical_flow.delta_angle);
		sensor_optical_flow.delta_angle_available = true;
	}

	sensor_optical_flow.max_flow_rate       = NAN;
	sensor_optical_flow.min_ground_distance = NAN;
	sensor_optical_flow.max_ground_distance = NAN;

	// Use distance value for distance sensor topic
	if (PX4_ISFINITE(flow.distance) && (flow.distance >= 0.f)) {
		// Positive value (including zero): distance known. Negative value: Unknown distance.
		sensor_optical_flow.distance_m = flow.distance;
		sensor_optical_flow.distance_available = true;
	}

	sensor_optical_flow.timestamp = hrt_absolute_time();

	_sensor_optical_flow_pub.publish(sensor_optical_flow);
}

void SimulatorMavlink::handle_message_rc_channels(const mavlink_message_t *msg)
{
	mavlink_rc_channels_t rc_channels;
	mavlink_msg_rc_channels_decode(msg, &rc_channels);

	input_rc_s rc_input{};
	rc_input.timestamp_last_signal = hrt_absolute_time();
	rc_input.channel_count = rc_channels.chancount;
	rc_input.rssi = rc_channels.rssi;
	rc_input.values[0] = rc_channels.chan1_raw;
	rc_input.values[1] = rc_channels.chan2_raw;
	rc_input.values[2] = rc_channels.chan3_raw;
	rc_input.values[3] = rc_channels.chan4_raw;
	rc_input.values[4] = rc_channels.chan5_raw;
	rc_input.values[5] = rc_channels.chan6_raw;
	rc_input.values[6] = rc_channels.chan7_raw;
	rc_input.values[7] = rc_channels.chan8_raw;
	rc_input.values[8] = rc_channels.chan9_raw;
	rc_input.values[9] = rc_channels.chan10_raw;
	rc_input.values[10] = rc_channels.chan11_raw;
	rc_input.values[11] = rc_channels.chan12_raw;
	rc_input.values[12] = rc_channels.chan13_raw;
	rc_input.values[13] = rc_channels.chan14_raw;
	rc_input.values[14] = rc_channels.chan15_raw;
	rc_input.values[15] = rc_channels.chan16_raw;
	rc_input.values[16] = rc_channels.chan17_raw;
	rc_input.values[17] = rc_channels.chan18_raw;

	rc_input.link_quality = -1;
	rc_input.rssi_dbm = NAN;

	rc_input.timestamp = hrt_absolute_time();

	// publish message
	_input_rc_pub.publish(rc_input);
}

void SimulatorMavlink::handle_message_vision_position_estimate(const mavlink_message_t *msg)
{
	mavlink_vision_position_estimate_t vpe;
	mavlink_msg_vision_position_estimate_decode(msg, &vpe);

	// fill vehicle_odometry from Mavlink VISION_POSITION_ESTIMATE
	vehicle_odometry_s odom{vehicle_odometry_empty};

	odom.timestamp_sample = hrt_absolute_time(); // _mavlink_timesync.sync_stamp(vpe.usec);

	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	odom.position[0] = vpe.x;
	odom.position[1] = vpe.y;
	odom.position[2] = vpe.z;

	const matrix::Quatf q(matrix::Eulerf(vpe.roll, vpe.pitch, vpe.yaw));
	q.copyTo(odom.q);

	// VISION_POSITION_ESTIMATE covariance
	//  Row-major representation of pose 6x6 cross-covariance matrix upper right triangle
	//  (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.).
	//  If unknown, assign NaN value to first element in the array.
	odom.position_variance[0] = vpe.covariance[0];  // X  row 0, col 0
	odom.position_variance[1] = vpe.covariance[6];  // Y  row 1, col 1
	odom.position_variance[2] = vpe.covariance[11]; // Z  row 2, col 2

	odom.orientation_variance[0] = vpe.covariance[15]; // R  row 3, col 3
	odom.orientation_variance[1] = vpe.covariance[18]; // P  row 4, col 4
	odom.orientation_variance[2] = vpe.covariance[20]; // Y  row 5, col 5

	odom.reset_counter = vpe.reset_counter;

	odom.timestamp = hrt_absolute_time();

	_visual_odometry_pub.publish(odom);
}

void SimulatorMavlink::send_mavlink_message(const mavlink_message_t &aMsg)
{
	uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t bufLen = 0;

	bufLen = mavlink_msg_to_send_buffer(buf, &aMsg);

	ssize_t len;

	if (_ip == InternetProtocol::UDP) {
		len = ::sendto(_fd, buf, bufLen, 0, (struct sockaddr *)&_srcaddr, sizeof(_srcaddr));

	} else {
		len = ::send(_fd, buf, bufLen, 0);
	}

	if (len <= 0) {
		PX4_WARN("Failed sending mavlink message: %s", strerror(errno));
	}
}

void *SimulatorMavlink::sending_trampoline(void * /*unused*/)
{
	_instance->send();
	return nullptr;
}

void SimulatorMavlink::send()
{
#ifdef __PX4_DARWIN
	pthread_setname_np("sim_send");
#else
	pthread_setname_np(pthread_self(), "sim_send");
#endif

	// Subscribe to topics.
	// Only subscribe to the first actuator_outputs to fill a single HIL_ACTUATOR_CONTROLS.
	_actuator_outputs_sub = orb_subscribe_multi(ORB_ID(actuator_outputs_sim), 0);

	// Before starting, we ought to send a heartbeat to initiate the SITL
	// simulator to start sending sensor data which will set the time and
	// get everything rolling.
	// Without this, we get stuck at px4_poll which waits for a time update.
	send_heartbeat();

	px4_pollfd_struct_t fds_actuator_outputs[1] = {};
	fds_actuator_outputs[0].fd = _actuator_outputs_sub;
	fds_actuator_outputs[0].events = POLLIN;

	while (true) {

		// Wait for up to 100ms for data.
		int pret = px4_poll(&fds_actuator_outputs[0], 1, 100);

		if (pret == 0) {
			// Timed out, try again.
			continue;
		}

		if (pret < 0) {
			PX4_ERR("poll error %s", strerror(errno));
			continue;
		}

		if (fds_actuator_outputs[0].revents & POLLIN) {
			// Got new data to read, update all topics.
			parameters_update(false);
			check_failure_injections();
			_vehicle_status_sub.update(&_vehicle_status);
			_battery_status_sub.update(&_battery_status);

			// Wait for other modules, such as logger or ekf2
			px4_lockstep_wait_for_components();

			send_controls();
		}
	}

	orb_unsubscribe(_actuator_outputs_sub);
}

void SimulatorMavlink::request_hil_state_quaternion()
{
	mavlink_command_long_t cmd_long = {};
	mavlink_message_t message = {};
	cmd_long.command = MAV_CMD_SET_MESSAGE_INTERVAL;
	cmd_long.param1 = MAVLINK_MSG_ID_HIL_STATE_QUATERNION;
	cmd_long.param2 = 5e3;
	mavlink_msg_command_long_encode(_param_mav_sys_id.get(), _param_mav_comp_id.get(), &message, &cmd_long);
	send_mavlink_message(message);
}

void SimulatorMavlink::send_heartbeat()
{
	mavlink_heartbeat_t hb = {};
	mavlink_message_t message = {};
	hb.autopilot = 12;
	hb.base_mode |= (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 128 : 0;
	mavlink_msg_heartbeat_encode(_param_mav_sys_id.get(), _param_mav_comp_id.get(), &message, &hb);
	send_mavlink_message(message);
}

void SimulatorMavlink::run()
{
#ifdef __PX4_DARWIN
	pthread_setname_np("sim_rcv");
#else
	pthread_setname_np(pthread_self(), "sim_rcv");
#endif

	struct sockaddr_in _myaddr {};
	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(_port);

	if (_tcp_remote_ipaddr != nullptr) {
		_myaddr.sin_addr.s_addr = inet_addr(_tcp_remote_ipaddr);

	} else if (!_hostname.empty()) {
		/* resolve hostname */
		struct hostent *host;
		host = gethostbyname(_hostname.c_str());
		memcpy(&_myaddr.sin_addr, host->h_addr_list[0], host->h_length);

		char ip[30];
		strcpy(ip, (char *)inet_ntoa((struct in_addr)_myaddr.sin_addr));
		PX4_INFO("Resolved host '%s' to address: %s", _hostname.c_str(), ip);
	}


	if (_ip == InternetProtocol::UDP) {

		if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
			PX4_ERR("Creating UDP socket failed: %s", strerror(errno));
			return;
		}

		if (bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
			PX4_ERR("bind for UDP port %i failed (%i)", _port, errno);
			::close(_fd);
			return;
		}

		PX4_INFO("Waiting for simulator to connect on UDP port %u", _port);

		while (true) {
			// Once we receive something, we're most probably good and can carry on.
			int len = ::recvfrom(_fd, _buf, sizeof(_buf), 0,
					     (struct sockaddr *)&_srcaddr, (socklen_t *)&_addrlen);

			if (len > 0) {
				break;

			} else {
				system_usleep(500);
			}
		}

		PX4_INFO("Simulator connected on UDP port %u.", _port);

	} else {

		PX4_INFO("Waiting for simulator to accept connection on TCP port %u", _port);

		while (true) {
			if ((_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
				PX4_ERR("Creating TCP socket failed: %s", strerror(errno));
				return;
			}

			int yes = 1;
			int ret = setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, (char *) &yes, sizeof(int));

			if (ret != 0) {
				PX4_ERR("setsockopt failed: %s", strerror(errno));
			}

			socklen_t myaddr_len = sizeof(_myaddr);
			ret = connect(_fd, (struct sockaddr *)&_myaddr, myaddr_len);

			if (ret == 0) {
				break;

			} else {
				::close(_fd);
				system_usleep(500);
			}
		}

		PX4_INFO("Simulator connected on TCP port %u.", _port);

	}

	// Create a thread for sending data to the simulator.
	pthread_t sender_thread;

	pthread_attr_t sender_thread_attr;
	pthread_attr_init(&sender_thread_attr);
	pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(8000));

	struct sched_param param;
	(void)pthread_attr_getschedparam(&sender_thread_attr, &param);

	// sender thread should run immediately after new outputs are available
	//  to send the lockstep update to the simulation
	param.sched_priority = SCHED_PRIORITY_ACTUATOR_OUTPUTS + 1;
	(void)pthread_attr_setschedparam(&sender_thread_attr, &param);

	struct pollfd fds[2] = {};
	unsigned fd_count = 1;
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	// got data from simulator, now activate the sending thread
	pthread_create(&sender_thread, &sender_thread_attr, SimulatorMavlink::sending_trampoline, nullptr);
	pthread_attr_destroy(&sender_thread_attr);

	mavlink_status_t mavlink_status = {};

	// Request HIL_STATE_QUATERNION for ground truth.
	request_hil_state_quaternion();

	while (true) {

		// wait for new mavlink messages to arrive
		int pret = ::poll(&fds[0], fd_count, 1000);

		if (pret == 0) {
			// Timed out.
			PX4_ERR("poll timeout %d, %d", pret, errno);
			continue;
		}

		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			continue;
		}

		if (fds[0].revents & POLLIN) {

			int len = ::recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, (socklen_t *)&_addrlen);

			if (len > 0) {
				mavlink_message_t msg;

				for (int i = 0; i < len; i++) {
					if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &mavlink_status)) {
						handle_message(&msg);
					}
				}
			}
		}
	}
}

void SimulatorMavlink::check_failure_injections()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE) {
			continue;
		}

		bool handled = false;
		bool supported = false;

		const int failure_unit = static_cast<int>(vehicle_command.param1 + 0.5f);
		const int failure_type = static_cast<int>(vehicle_command.param2 + 0.5f);
		const int instance = static_cast<int>(vehicle_command.param3 + 0.5f);

		if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_GPS) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, GPS off");
				supported = true;
				_gps_blocked = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, GPS ok");
				supported = true;
				_gps_blocked = false;
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_ACCEL) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < ACCEL_COUNT_MAX; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, accel %d off", i);
						_accel_blocked[i] = true;
						_accel_stuck[i] = false;
					}

				} else if (instance >= 1 && instance <= ACCEL_COUNT_MAX) {
					PX4_WARN("CMD_INJECT_FAILURE, accel %d off", instance - 1);
					_accel_blocked[instance - 1] = true;
					_accel_stuck[instance - 1] = false;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < ACCEL_COUNT_MAX; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, accel %d stuck", i);
						_accel_blocked[i] = false;
						_accel_stuck[i] = true;
					}

				} else if (instance >= 1 && instance <= ACCEL_COUNT_MAX) {
					PX4_WARN("CMD_INJECT_FAILURE, accel %d stuck", instance - 1);
					_accel_blocked[instance - 1] = false;
					_accel_stuck[instance - 1] = true;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < ACCEL_COUNT_MAX; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, accel %d ok", i);
						_accel_blocked[i] = false;
						_accel_stuck[i] = false;
					}

				} else if (instance >= 1 && instance <= ACCEL_COUNT_MAX) {
					PX4_INFO("CMD_INJECT_FAILURE, accel %d ok", instance - 1);
					_accel_blocked[instance - 1] = false;
					_accel_stuck[instance - 1] = false;
				}
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_GYRO) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < GYRO_COUNT_MAX; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, gyro %d off", i);
						_gyro_blocked[i] = true;
						_gyro_stuck[i] = false;
					}

				} else if (instance >= 1 && instance <= GYRO_COUNT_MAX) {
					PX4_WARN("CMD_INJECT_FAILURE, gyro %d off", instance - 1);
					_gyro_blocked[instance - 1] = true;
					_gyro_stuck[instance - 1] = false;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < GYRO_COUNT_MAX; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, gyro %d stuck", i);
						_gyro_blocked[i] = false;
						_gyro_stuck[i] = true;
					}

				} else if (instance >= 1 && instance <= GYRO_COUNT_MAX) {
					PX4_INFO("CMD_INJECT_FAILURE, gyro %d stuck", instance - 1);
					_gyro_blocked[instance - 1] = false;
					_gyro_stuck[instance - 1] = true;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < GYRO_COUNT_MAX; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, gyro %d ok", i);
						_gyro_blocked[i] = false;
						_gyro_stuck[i] = false;
					}

				} else if (instance >= 1 && instance <= GYRO_COUNT_MAX) {
					PX4_INFO("CMD_INJECT_FAILURE, gyro %d ok", instance - 1);
					_gyro_blocked[instance - 1] = false;
					_gyro_stuck[instance - 1] = false;
				}
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_MAG) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < MAG_COUNT_MAX; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, mag %d off", i);
						_mag_blocked[i] = true;
						_mag_stuck[i] = false;
					}

				} else if (instance >= 1 && instance <= MAG_COUNT_MAX) {
					PX4_WARN("CMD_INJECT_FAILURE, mag %d off", instance - 1);
					_mag_blocked[instance - 1] = true;
					_mag_stuck[instance - 1] = false;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < MAG_COUNT_MAX; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, mag %d stuck", i);
						_mag_blocked[i] = false;
						_mag_stuck[i] = true;
					}

				} else if (instance >= 1 && instance <= MAG_COUNT_MAX) {
					PX4_WARN("CMD_INJECT_FAILURE, mag %d stuck", instance - 1);
					_mag_blocked[instance - 1] = false;
					_mag_stuck[instance - 1] = true;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < MAG_COUNT_MAX; i++) {
						PX4_WARN("CMD_INJECT_FAILURE, mag %d ok", i);
						_mag_blocked[i] = false;
						_mag_stuck[i] = false;
					}

				} else if (instance >= 1 && instance <= MAG_COUNT_MAX) {
					PX4_WARN("CMD_INJECT_FAILURE, mag %d ok", instance - 1);
					_mag_blocked[instance - 1] = false;
					_mag_stuck[instance - 1] = false;
				}
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_BARO) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, baro off");
				supported = true;
				_baro_blocked = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				PX4_WARN("CMD_INJECT_FAILURE, baro stuck");
				supported = true;
				_baro_stuck = true;
				_baro_blocked = false;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, baro ok");
				supported = true;
				_baro_blocked = false;
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_AIRSPEED) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, airspeed off");
				supported = true;
				_airspeed_disconnected = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_WRONG) {
				PX4_WARN("CMD_INJECT_FAILURE, airspeed wrong (simulate pitot blockage)");
				supported = true;
				_airspeed_blocked_timestamp = hrt_absolute_time();

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, airspeed ok");
				supported = true;
				_airspeed_disconnected = false;
				_airspeed_blocked_timestamp = 0;
			}

		} else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_VIO) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, vio off");
				supported = true;
				_vio_blocked = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, vio ok");
				supported = true;
				_vio_blocked = false;
			}
		}

		if (handled) {
			vehicle_command_ack_s ack{};
			ack.command = vehicle_command.command;
			ack.from_external = false;
			ack.result = supported ?
				     vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
				     vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
			ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(ack);
		}
	}
}

int SimulatorMavlink::publish_distance_topic(const mavlink_distance_sensor_t *dist_mavlink)
{
	distance_sensor_s dist{};
	dist.timestamp = hrt_absolute_time();
	dist.min_distance = dist_mavlink->min_distance / 100.0f;
	dist.max_distance = dist_mavlink->max_distance / 100.0f;
	dist.current_distance = dist_mavlink->current_distance / 100.0f;
	dist.type = dist_mavlink->type;
	dist.variance = dist_mavlink->covariance * 1e-4f; // cm^2 to m^2

	device::Device::DeviceId device_id {};
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SIMULATION;
	device_id.devid_s.address = dist_mavlink->id;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_SIM;

	dist.device_id = device_id.devid;

	// MAVLink DISTANCE_SENSOR signal_quality value of 0 means unset/unknown
	// quality value. Also it comes normalised between 1 and 100 while the uORB
	// signal quality is normalised between 0 and 100.
	dist.signal_quality = dist_mavlink->signal_quality == 0 ? -1 : 100 * (dist_mavlink->signal_quality - 1) / 99;

	switch (dist_mavlink->orientation) {
	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_270:
		dist.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
		break;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_90:
		dist.orientation = distance_sensor_s::ROTATION_UPWARD_FACING;
		break;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_180:
		dist.orientation = distance_sensor_s::ROTATION_BACKWARD_FACING;
		break;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_NONE:
		dist.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
		break;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_270:
		dist.orientation = distance_sensor_s::ROTATION_LEFT_FACING;
		break;

	case MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_90:
		dist.orientation = distance_sensor_s::ROTATION_RIGHT_FACING;
		break;

	default:
		dist.orientation = distance_sensor_s::ROTATION_CUSTOM;
	}

	dist.h_fov = dist_mavlink->horizontal_fov;
	dist.v_fov = dist_mavlink->vertical_fov;
	dist.q[0] = dist_mavlink->quaternion[0];
	dist.q[1] = dist_mavlink->quaternion[1];
	dist.q[2] = dist_mavlink->quaternion[2];
	dist.q[3] = dist_mavlink->quaternion[3];

	// New publishers will be created based on the sensor ID's being different or not
	for (size_t i = 0; i < sizeof(_dist_sensor_ids) / sizeof(_dist_sensor_ids[0]); i++) {
		if (_dist_pubs[i] && _dist_sensor_ids[i] == dist.device_id) {
			_dist_pubs[i]->publish(dist);
			break;

		}

		if (_dist_pubs[i] == nullptr) {
			_dist_pubs[i] = new uORB::PublicationMulti<distance_sensor_s> {ORB_ID(distance_sensor)};
			_dist_sensor_ids[i] = dist.device_id;
			_dist_pubs[i]->publish(dist);
			break;
		}
	}

	return PX4_OK;
}

int SimulatorMavlink::start(int argc, char *argv[])
{
	_instance = new SimulatorMavlink();

	if (_instance) {

		if (argc == 5 && strcmp(argv[3], "-u") == 0) {
			_instance->set_ip(InternetProtocol::UDP);
			_instance->set_port(atoi(argv[4]));
		}

		if (argc == 5 && strcmp(argv[3], "-c") == 0) {
			_instance->set_ip(InternetProtocol::TCP);
			_instance->set_port(atoi(argv[4]));
		}

		if (argc == 6 && strcmp(argv[3], "-t") == 0) {
			PX4_INFO("using TCP on remote host %s port %s", argv[4], argv[5]);
			PX4_WARN("Please ensure port %s is not blocked by a firewall.", argv[5]);
			_instance->set_ip(InternetProtocol::TCP);
			_instance->set_tcp_remote_ipaddr(argv[4]);
			_instance->set_port(atoi(argv[5]));
		}

		if (argc == 6 && strcmp(argv[3], "-h") == 0) {
			PX4_INFO("using TCP on remote host %s port %s", argv[4], argv[5]);
			PX4_WARN("Please ensure port %s is not blocked by a firewall.", argv[5]);
			_instance->set_ip(InternetProtocol::TCP);
			_instance->set_hostname(argv[4]);
			_instance->set_port(atoi(argv[5]));
		}

		_instance->run();

		return 0;

	} else {
		PX4_WARN("creation failed");
		return 1;
	}
}

static void usage()
{
	PX4_INFO("Usage: simulator_mavlink {start -[spt] [-u udp_port / -c tcp_port] |stop|status}");
	PX4_INFO("Start simulator:     simulator_mavlink start");
	PX4_INFO("Connect using UDP: simulator_mavlink start -u udp_port");
	PX4_INFO("Connect using TCP: simulator_mavlink start -c tcp_port");
	PX4_INFO("Connect to a remote server using TCP: simulator_mavlink start -t ip_addr tcp_port");
	PX4_INFO("Connect to a remote server via hostname using TCP: simulator_mavlink start -h hostname tcp_port");
}

__BEGIN_DECLS
extern int simulator_mavlink_main(int argc, char *argv[]);
__END_DECLS

int simulator_mavlink_main(int argc, char *argv[])
{
	if (argc > 1 && strcmp(argv[1], "start") == 0) {

		if (g_sim_task >= 0) {
			PX4_WARN("Simulator already started");
			return 0;
		}

		g_sim_task = px4_task_spawn_cmd("simulator_mavlink",
						SCHED_DEFAULT,
						SCHED_PRIORITY_MAX,
						1500,
						SimulatorMavlink::start,
						argv);

#if defined(ENABLE_LOCKSTEP_SCHEDULER)

		// We want to prevent the rest of the startup script from running until time
		// is initialized by the HIL_SENSOR messages from the simulator.
		while (true) {
			if (SimulatorMavlink::getInstance() && SimulatorMavlink::getInstance()->has_initialized()) {
				break;
			}

			system_usleep(100);
		}

#endif

	} else if (argc == 2 && strcmp(argv[1], "stop") == 0) {
		if (g_sim_task < 0) {
			PX4_WARN("Simulator not running");
			return 1;

		} else {
			px4_task_delete(g_sim_task);
			g_sim_task = -1;
		}

	} else if (argc == 2 && strcmp(argv[1], "status") == 0) {
		if (g_sim_task < 0) {
			PX4_WARN("Simulator not running");
			return 1;

		} else {
			PX4_INFO("running");
		}

	} else {
		usage();
		return 1;
	}

	return 0;
}
