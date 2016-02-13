/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file ekf2_main.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>

#include <arch/board/board.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <mavlink/mavlink_log.h>
#include <platforms/px4_defines.h>
#include <drivers/drv_hrt.h>
#include <controllib/uorb/blocks.hpp>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/vehicle_control_mode.h>

#include <ecl/EKF/ekf.h>


extern "C" __EXPORT int ekf2_main(int argc, char *argv[]);


class Ekf2;

namespace ekf2
{
Ekf2 *instance = nullptr;
}


class Ekf2 : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	Ekf2();

	/**
	 * Destructor, also kills task.
	 */
	~Ekf2();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	static void	task_main_trampoline(int argc, char *argv[]);

	void		task_main();

	void print_status();

	void exit() { _task_should_exit = true; }

private:
	static constexpr float _dt_max = 0.02;
	bool		_task_should_exit = false;		/**< if true, task should exit */
	int		_control_task = -1;			/**< task handle for task */

	int		_sensors_sub = -1;
	int		_gps_sub = -1;
	int		_airspeed_sub = -1;
	int		_params_sub = -1;
	int		_control_mode_sub = -1;

	orb_advert_t _att_pub;
	orb_advert_t _lpos_pub;
	orb_advert_t _control_state_pub;
	orb_advert_t _vehicle_global_position_pub;
	orb_advert_t _estimator_status_pub;
	orb_advert_t _estimator_innovations_pub;

	/* Low pass filter for attitude rates */
	math::LowPassFilter2p _lp_roll_rate;
	math::LowPassFilter2p _lp_pitch_rate;
	math::LowPassFilter2p _lp_yaw_rate;

	EstimatorInterface *_ekf;

	parameters *_params;	// pointer to ekf parameter struct (located in _ekf class instance)

	control::BlockParamFloat *_mag_delay_ms;
	control::BlockParamFloat *_baro_delay_ms;
	control::BlockParamFloat *_gps_delay_ms;
	control::BlockParamFloat *_airspeed_delay_ms;

	control::BlockParamFloat *_gyro_noise;
	control::BlockParamFloat *_accel_noise;

	// process noise
	control::BlockParamFloat *_gyro_bias_p_noise;
	control::BlockParamFloat *_accel_bias_p_noise;
	control::BlockParamFloat *_gyro_scale_p_noise;
	control::BlockParamFloat *_mag_p_noise;
	control::BlockParamFloat *_wind_vel_p_noise;

	control::BlockParamFloat *_gps_vel_noise;
	control::BlockParamFloat *_gps_pos_noise;
	control::BlockParamFloat *_pos_noaid_noise;
	control::BlockParamFloat *_baro_noise;
	control::BlockParamFloat *_baro_innov_gate;     // innovation gate for barometric height innovation test (std dev)
	control::BlockParamFloat *_posNE_innov_gate;    // innovation gate for GPS horizontal position innovation test (std dev)
	control::BlockParamFloat *_vel_innov_gate;      // innovation gate for GPS velocity innovation test (std dev)

	control::BlockParamFloat *_mag_heading_noise;	// measurement noise used for simple heading fusion
	control::BlockParamFloat *_mag_noise;           // measurement noise used for 3-axis magnetoemter fusion (Gauss)
	control::BlockParamFloat *_mag_declination_deg;	// magnetic declination in degrees
	control::BlockParamFloat *_heading_innov_gate;	// innovation gate for heading innovation test
	control::BlockParamFloat *_mag_innov_gate;	// innovation gate for magnetometer innovation test

	control::BlockParamInt *_gps_check_mask;        // bitmasked integer used to activate the different GPS quality checks
	control::BlockParamFloat *_requiredEph;         // maximum acceptable horiz position error (m)
	control::BlockParamFloat *_requiredEpv;         // maximum acceptable vert position error (m)
	control::BlockParamFloat *_requiredSacc;        // maximum acceptable speed error (m/s)
	control::BlockParamInt *_requiredNsats;         // minimum acceptable satellite count
	control::BlockParamFloat *_requiredGDoP;        // maximum acceptable geometric dilution of precision
	control::BlockParamFloat *_requiredHdrift;      // maximum acceptable horizontal drift speed (m/s)
	control::BlockParamFloat *_requiredVdrift;      // maximum acceptable vertical drift speed (m/s)

	int update_subscriptions();

};

Ekf2::Ekf2():
	SuperBlock(NULL, "EKF"),
	_att_pub(nullptr),
	_lpos_pub(nullptr),
	_control_state_pub(nullptr),
	_vehicle_global_position_pub(nullptr),
	_estimator_status_pub(nullptr),
	_estimator_innovations_pub(nullptr),
	_lp_roll_rate(250.0f, 30.0f),
	_lp_pitch_rate(250.0f, 30.0f),
	_lp_yaw_rate(250.0f, 20.0f),
	_ekf(new Ekf()),
	_params(_ekf->getParamHandle()),
	_mag_delay_ms(new control::BlockParamFloat(this, "EKF2_MAG_DELAY", false, &_params->mag_delay_ms)),
	_baro_delay_ms(new control::BlockParamFloat(this, "EKF2_BARO_DELAY", false, &_params->baro_delay_ms)),
	_gps_delay_ms(new control::BlockParamFloat(this, "EKF2_GPS_DELAY", false, &_params->gps_delay_ms)),
	_airspeed_delay_ms(new control::BlockParamFloat(this, "EKF2_ASP_DELAY", false, &_params->airspeed_delay_ms)),
	_gyro_noise(new control::BlockParamFloat(this, "EKF2_GYR_NOISE", false, &_params->gyro_noise)),
	_accel_noise(new control::BlockParamFloat(this, "EKF2_ACC_NOISE", false, &_params->accel_noise)),
	_gyro_bias_p_noise(new control::BlockParamFloat(this, "EKF2_GYR_B_NOISE", false, &_params->gyro_bias_p_noise)),
	_accel_bias_p_noise(new control::BlockParamFloat(this, "EKF2_ACC_B_NOISE", false, &_params->accel_bias_p_noise)),
	_gyro_scale_p_noise(new control::BlockParamFloat(this, "EKF2_GYR_S_NOISE", false, &_params->gyro_scale_p_noise)),
	_mag_p_noise(new control::BlockParamFloat(this, "EKF2_MAG_B_NOISE", false, &_params->mag_p_noise)),
	_wind_vel_p_noise(new control::BlockParamFloat(this, "EKF2_WIND_NOISE", false, &_params->wind_vel_p_noise)),
	_gps_vel_noise(new control::BlockParamFloat(this, "EKF2_GPS_V_NOISE", false, &_params->gps_vel_noise)),
	_gps_pos_noise(new control::BlockParamFloat(this, "EKF2_GPS_P_NOISE", false, &_params->gps_pos_noise)),
	_pos_noaid_noise(new control::BlockParamFloat(this, "EKF2_NOAID_NOISE", false, &_params->pos_noaid_noise)),
	_baro_noise(new control::BlockParamFloat(this, "EKF2_BARO_NOISE", false, &_params->baro_noise)),
	_baro_innov_gate(new control::BlockParamFloat(this, "EKF2_BARO_GATE", false, &_params->baro_innov_gate)),
	_posNE_innov_gate(new control::BlockParamFloat(this, "EKF2_GPS_P_GATE", false, &_params->posNE_innov_gate)),
	_vel_innov_gate(new control::BlockParamFloat(this, "EKF2_GPS_V_GATE", false, &_params->vel_innov_gate)),
	_mag_heading_noise(new control::BlockParamFloat(this, "EKF2_HEAD_NOISE", false, &_params->mag_heading_noise)),
	_mag_noise(new control::BlockParamFloat(this, "EKF2_MAG_NOISE", false, &_params->mag_noise)),
	_mag_declination_deg(new control::BlockParamFloat(this, "EKF2_MAG_DECL", false, &_params->mag_declination_deg)),
	_heading_innov_gate(new control::BlockParamFloat(this, "EKF2_HDG_GATE", false, &_params->heading_innov_gate)),
	_mag_innov_gate(new control::BlockParamFloat(this, "EKF2_MAG_GATE", false, &_params->mag_innov_gate)),
	_gps_check_mask(new control::BlockParamInt(this, "EKF2_GPS_CHECK", false, &_params->gps_check_mask)),
	_requiredEph(new control::BlockParamFloat(this, "EKF2_REQ_EPH", false, &_params->req_hacc)),
	_requiredEpv(new control::BlockParamFloat(this, "EKF2_REQ_EPV", false, &_params->req_vacc)),
	_requiredSacc(new control::BlockParamFloat(this, "EKF2_REQ_SACC", false, &_params->req_sacc)),
	_requiredNsats(new control::BlockParamInt(this, "EKF2_REQ_NSATS", false, &_params->req_nsats)),
	_requiredGDoP(new control::BlockParamFloat(this, "EKF2_REQ_GDOP", false, &_params->req_gdop)),
	_requiredHdrift(new control::BlockParamFloat(this, "EKF2_REQ_HDRIFT", false, &_params->req_hdrift)),
	_requiredVdrift(new control::BlockParamFloat(this, "EKF2_REQ_VDRIFT", false, &_params->req_vdrift))
{

}

Ekf2::~Ekf2()
{

}

void Ekf2::print_status()
{
	warnx("position OK %s", (_ekf->position_is_valid()) ? "[YES]" : "[NO]");
}

void Ekf2::task_main()
{
	// subscribe to relevant topics
	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

	px4_pollfd_struct_t fds[2] = {};
	fds[0].fd = _sensors_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _params_sub;
	fds[1].events = POLLIN;

	// initialise parameter cache
	updateParams();

	vehicle_gps_position_s gps = {};

	while (!_task_should_exit) {
		int ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		if (fds[1].revents & POLLIN) {
			// read from param to clear updated flag
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);
			updateParams();

			// fetch sensor data in next loop
			continue;

		} else if (!(fds[0].revents & POLLIN)) {
			// no new data
			continue;
		}

		bool gps_updated = false;
		bool airspeed_updated = false;
		bool control_mode_updated = false;

		sensor_combined_s sensors = {};
		airspeed_s airspeed = {};
		vehicle_control_mode_s vehicle_control_mode = {};

		orb_copy(ORB_ID(sensor_combined), _sensors_sub, &sensors);

		// update all other topics if they have new data
		orb_check(_gps_sub, &gps_updated);

		if (gps_updated) {
			orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &gps);
		}

		orb_check(_airspeed_sub, &airspeed_updated);

		if (airspeed_updated) {
			orb_copy(ORB_ID(airspeed), _airspeed_sub, &airspeed);
		}

		// Use the control model data to determine if the motors are armed as a surrogate for an on-ground vs in-air status
		// TODO implement a global vehicle on-ground/in-air check
		orb_check(_control_mode_sub, &control_mode_updated);

		if (control_mode_updated) {
			orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &vehicle_control_mode);
			_ekf->set_arm_status(vehicle_control_mode.flag_armed);
		}

		hrt_abstime now = hrt_absolute_time();
		// push imu data into estimator
		_ekf->setIMUData(now, sensors.gyro_integral_dt[0], sensors.accelerometer_integral_dt[0],
				 &sensors.gyro_integral_rad[0], &sensors.accelerometer_integral_m_s[0]);

		// read mag data
		_ekf->setMagData(sensors.magnetometer_timestamp[0], &sensors.magnetometer_ga[0]);

		// read baro data
		_ekf->setBaroData(sensors.baro_timestamp[0], &sensors.baro_alt_meter[0]);

		// read gps data if available
		if (gps_updated) {
			struct gps_message gps_msg = {};
			gps_msg.time_usec = gps.timestamp_position;
			gps_msg.lat = gps.lat;
			gps_msg.lon = gps.lon;
			gps_msg.alt = gps.alt;
			gps_msg.fix_type = gps.fix_type;
			gps_msg.eph = gps.eph;
			gps_msg.epv = gps.epv;
			gps_msg.sacc = gps.s_variance_m_s;
			gps_msg.time_usec_vel = gps.timestamp_velocity;
			gps_msg.vel_m_s = gps.vel_m_s;
			gps_msg.vel_ned[0] = gps.vel_n_m_s;
			gps_msg.vel_ned[1] = gps.vel_e_m_s;
			gps_msg.vel_ned[2] = gps.vel_d_m_s;
			gps_msg.vel_ned_valid = gps.vel_ned_valid;
			gps_msg.nsats = gps.satellites_used;
			//TODO add gdop to gps topic
			gps_msg.gdop = 0.0f;

			_ekf->setGpsData(gps.timestamp_position, &gps_msg);
		}

		// read airspeed data if available
		if (airspeed_updated) {
			_ekf->setAirspeedData(airspeed.timestamp, &airspeed.indicated_airspeed_m_s);
		}

		// run the EKF update
		_ekf->update();

		// generate vehicle attitude data
		struct vehicle_attitude_s att = {};
		att.timestamp = hrt_absolute_time();

		_ekf->copy_quaternion(att.q);
		matrix::Quaternion<float> q(att.q[0], att.q[1], att.q[2], att.q[3]);
		matrix::Euler<float> euler(q);
		att.roll = euler(0);
		att.pitch = euler(1);
		att.yaw = euler(2);

		// generate vehicle local position data
		struct vehicle_local_position_s lpos = {};
		float pos[3] = {};
		float vel[3] = {};

		lpos.timestamp = hrt_absolute_time();

		// Position in local NED frame
		_ekf->copy_position(pos);
		lpos.x = pos[0];
		lpos.y = pos[1];
		lpos.z = pos[2];

		// Velocity in NED frame (m/s)
		_ekf->copy_velocity(vel);
		lpos.vx = vel[0];
		lpos.vy = vel[1];
		lpos.vz = vel[2];

		// TODO: better status reporting
		lpos.xy_valid = _ekf->position_is_valid();
		lpos.z_valid = true;
		lpos.v_xy_valid = _ekf->position_is_valid();
		lpos.v_z_valid = true;

		// Position of local NED origin in GPS / WGS84 frame
		struct map_projection_reference_s ekf_origin = {};
		_ekf->get_ekf_origin(&lpos.ref_timestamp, &ekf_origin, &lpos.ref_alt);
		lpos.xy_global =
			_ekf->position_is_valid();          // true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon)
		lpos.z_global = true;                                // true if z is valid and has valid global reference (ref_alt)
		lpos.ref_lat = ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
		lpos.ref_lon = ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees

		// The rotation of the tangent plane vs. geographical north
		lpos.yaw = 0.0f;

		lpos.dist_bottom = 0.0f; // Distance to bottom surface (ground) in meters
		lpos.dist_bottom_rate = 0.0f; // Distance to bottom surface (ground) change rate
		lpos.surface_bottom_timestamp	= 0; // Time when new bottom surface found
		lpos.dist_bottom_valid = false; // true if distance to bottom surface is valid

		// TODO: uORB definition does not define what thes variables are. We have assumed them to be horizontal and vertical 1-std dev accuracy in metres
		// TODO: Should use sqrt of filter position variances
		lpos.eph = gps.eph;
		lpos.epv = gps.epv;

		// publish vehicle local position data
		if (_lpos_pub == nullptr) {
			_lpos_pub = orb_advertise(ORB_ID(vehicle_local_position), &lpos);

		} else {
			orb_publish(ORB_ID(vehicle_local_position), _lpos_pub, &lpos);
		}

		// generate control state data
		control_state_s ctrl_state = {};
		ctrl_state.timestamp = hrt_absolute_time();
		ctrl_state.roll_rate = _lp_roll_rate.apply(sensors.gyro_rad_s[0]);
		ctrl_state.pitch_rate = _lp_pitch_rate.apply(sensors.gyro_rad_s[1]);
		ctrl_state.yaw_rate = _lp_yaw_rate.apply(sensors.gyro_rad_s[2]);

		ctrl_state.q[0] = q(0);
		ctrl_state.q[1] = q(1);
		ctrl_state.q[2] = q(2);
		ctrl_state.q[3] = q(3);

		// publish control state data
		if (_control_state_pub == nullptr) {
			_control_state_pub = orb_advertise(ORB_ID(control_state), &ctrl_state);

		} else {
			orb_publish(ORB_ID(control_state), _control_state_pub, &ctrl_state);
		}

		// generate vehicle attitude data
		att.q[0] = q(0);
		att.q[1] = q(1);
		att.q[2] = q(2);
		att.q[3] = q(3);
		att.q_valid = true;

		att.rollspeed = sensors.gyro_rad_s[0];
		att.pitchspeed = sensors.gyro_rad_s[1];
		att.yawspeed = sensors.gyro_rad_s[2];

		// publish vehicle attitude data
		if (_att_pub == nullptr) {
			_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

		} else {
			orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
		}

		// generate and publish global position data
		struct vehicle_global_position_s global_pos = {};

		if (_ekf->position_is_valid()) {
			// TODO: local origin is currenlty at GPS height origin - this is different to ekf_att_pos_estimator

			global_pos.timestamp = hrt_absolute_time(); // Time of this estimate, in microseconds since system start
			global_pos.time_utc_usec = gps.time_utc_usec; // GPS UTC timestamp in microseconds

			double est_lat, est_lon;
			map_projection_reproject(&ekf_origin, lpos.x, lpos.y, &est_lat, &est_lon);
			global_pos.lat = est_lat; // Latitude in degrees
			global_pos.lon = est_lon; // Longitude in degrees

			global_pos.alt = -pos[2] + lpos.ref_alt; // Altitude AMSL in meters

			global_pos.vel_n = vel[0]; // Ground north velocity, m/s
			global_pos.vel_e = vel[1]; // Ground east velocity, m/s
			global_pos.vel_d = vel[2]; // Ground downside velocity, m/s

			global_pos.yaw = euler(2); // Yaw in radians -PI..+PI.

			global_pos.eph = gps.eph; // Standard deviation of position estimate horizontally
			global_pos.epv = gps.epv; // Standard deviation of position vertically

			// TODO: implement terrain estimator
			global_pos.terrain_alt = 0.0f; // Terrain altitude in m, WGS84
			global_pos.terrain_alt_valid = false; // Terrain altitude estimate is valid
			// TODO use innovatun consistency check timouts to set this
			global_pos.dead_reckoning = false; // True if this position is estimated through dead-reckoning

			global_pos.pressure_alt = sensors.baro_alt_meter[0]; // Pressure altitude AMSL (m)

			if (_vehicle_global_position_pub == nullptr) {
				_vehicle_global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_pos);

			} else {
				orb_publish(ORB_ID(vehicle_global_position), _vehicle_global_position_pub, &global_pos);
			}
		}

		// publish estimator status
		struct estimator_status_s status = {};
		status.timestamp = hrt_absolute_time();
		_ekf->get_state_delayed(status.states);
		_ekf->get_covariances(status.covariances);
		//status.gps_check_fail_flags = _ekf->_gps_check_fail_status.value;

		if (_estimator_status_pub == nullptr) {
			_estimator_status_pub = orb_advertise(ORB_ID(estimator_status), &status);

		} else {
			orb_publish(ORB_ID(estimator_status), _estimator_status_pub, &status);
		}

		// publish estimator innovation data
		struct ekf2_innovations_s innovations = {};
		innovations.timestamp = hrt_absolute_time();
		_ekf->get_vel_pos_innov(&innovations.vel_pos_innov[0]);
		_ekf->get_mag_innov(&innovations.mag_innov[0]);
		_ekf->get_heading_innov(&innovations.heading_innov);

		_ekf->get_vel_pos_innov_var(&innovations.vel_pos_innov_var[0]);
		_ekf->get_mag_innov_var(&innovations.mag_innov_var[0]);
		_ekf->get_heading_innov_var(&innovations.heading_innov_var);

		if (_estimator_innovations_pub == nullptr) {
			_estimator_innovations_pub = orb_advertise(ORB_ID(ekf2_innovations), &innovations);

		} else {
			orb_publish(ORB_ID(ekf2_innovations), _estimator_innovations_pub, &innovations);
		}

	}

	delete ekf2::instance;
	ekf2::instance = nullptr;
}

void Ekf2::task_main_trampoline(int argc, char *argv[])
{
	ekf2::instance->task_main();
}

int Ekf2::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("ekf2",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   9000,
					   (px4_main_t)&Ekf2::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

int ekf2_main(int argc, char *argv[])
{
	if (argc < 1) {
		PX4_WARN("usage: ekf2 {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (ekf2::instance != nullptr) {
			PX4_WARN("already running");
			return 1;
		}

		ekf2::instance = new Ekf2();

		if (ekf2::instance == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		if (OK != ekf2::instance->start()) {
			delete ekf2::instance;
			ekf2::instance = nullptr;
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (ekf2::instance == nullptr) {
			PX4_WARN("not running");
			return 1;
		}

		ekf2::instance->exit();

		// wait for the destruction of the instance
		while (ekf2::instance != nullptr) {
			usleep(50000);
		}

		return 0;
	}

	if (!strcmp(argv[1], "print")) {
		if (ekf2::instance != nullptr) {

			return 0;
		}

		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		if (ekf2::instance) {
			PX4_WARN("running");
			ekf2::instance->print_status();
			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}

	PX4_WARN("unrecognized command");
	return 1;
}
