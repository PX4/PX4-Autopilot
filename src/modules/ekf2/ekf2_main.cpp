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

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/parameter_update.h>

#include <ecl/EKF/ekf.h>


extern "C" __EXPORT int ekf2_main(int argc, char *argv[]);


class Ekf2;

namespace ekf2
{
Ekf2 *instance;
}


class Ekf2
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

	void print();

private:
	static constexpr float _dt_max = 0.02;
	bool		_task_should_exit = false;		/**< if true, task should exit */
	int		_control_task = -1;			/**< task handle for task */

	int		_sensors_sub = -1;
	int		_gps_sub = -1;
	int		_airspeed_sub = -1;
	int 	_params_sub = -1;

	orb_advert_t _att_pub;
	orb_advert_t _lpos_pub;
	orb_advert_t _control_state_pub;

	struct {
		param_t mag_delay_ms;
		param_t baro_delay_ms;
		param_t gps_delay_ms;
		param_t airspeed_delay_ms;
		param_t 	requiredEph;
		param_t 	requiredEpv;

		param_t gyro_noise;
		param_t accel_noise;

		// process noise
		param_t gyro_bias_p_noise;
		param_t accel_bias_p_noise;
		param_t gyro_scale_p_noise;
		param_t mag_p_noise;
		param_t wind_vel_p_noise;

		param_t gps_vel_noise;
		param_t gps_pos_noise;
		param_t baro_noise;

		param_t mag_heading_noise;	// measurement noise used for simple heading fusion
		param_t mag_declination_deg;	// magnetic declination in degrees
		param_t heading_innov_gate;	// innovation gate for heading innovation test
	} _param_handles;

	/* Low pass filter for attitude rates */
	math::LowPassFilter2p _lp_roll_rate;
	math::LowPassFilter2p _lp_pitch_rate;
	math::LowPassFilter2p _lp_yaw_rate;

	EstimatorBase *_ekf;


	int update_subscriptions();

	void update_parameters();

};

Ekf2::Ekf2():
	_lp_roll_rate(250.0f, 30.0f),
	_lp_pitch_rate(250.0f, 30.0f),
	_lp_yaw_rate(250.0f, 20.0f)
{
	_ekf = new Ekf();
	_att_pub = nullptr;
	_lpos_pub = nullptr;
	_control_state_pub = nullptr;

	_param_handles.mag_delay_ms = param_find("EKF2_MAG_DELAY");
	_param_handles.baro_delay_ms = param_find("EKF2_BARO_DELAY");
	_param_handles.gps_delay_ms = param_find("EKF2_GPS_DELAY");
	_param_handles.airspeed_delay_ms = param_find("EKF2_ASP_DELAY");
	_param_handles.requiredEph = param_find("EKF2_REQ_EPH");
	_param_handles.requiredEpv = param_find("EKF2_REQ_EPV");

	_param_handles.gyro_noise = param_find("EKF2_G_NOISE");
	_param_handles.accel_noise = param_find("EKF2_ACC_NOISE");

	_param_handles.gyro_bias_p_noise = param_find("EKF2_GB_NOISE");
	_param_handles.accel_bias_p_noise = param_find("EKF2_ACCB_NOISE");
	_param_handles.gyro_scale_p_noise = param_find("EKF2_GS_NOISE");
	_param_handles.mag_p_noise = param_find("EKF2_MAG_NOISE");
	_param_handles.wind_vel_p_noise = param_find("EKF2_WIND_NOISE");

	_param_handles.gps_vel_noise = param_find("EKF2_GPS_V_NOISE");
	_param_handles.gps_pos_noise = param_find("EKF2_GPS_P_NOISE");
	_param_handles.baro_noise = param_find("EKF2_BARO_NOISE");

	_param_handles.mag_heading_noise = param_find("EKF2_HEAD_NOISE");
	_param_handles.mag_declination_deg = param_find("EKF2_MAG_DECL");
	_param_handles.heading_innov_gate = param_find("EKF2_H_INOV_GATE");
}

Ekf2::~Ekf2()
{

}

void Ekf2::print()
{
	_ekf->printStoredGps();
	_ekf->printStoredBaro();
	_ekf->printStoredMag();
	_ekf->printStoredIMU();
}

void Ekf2::update_parameters()
{
	parameters *params = _ekf->getParamHandle();
	param_get(_param_handles.mag_delay_ms, &params->mag_delay_ms);
	param_get(_param_handles.baro_delay_ms, &params->baro_delay_ms);
	param_get(_param_handles.gps_delay_ms, &params->gps_delay_ms);
	param_get(_param_handles.airspeed_delay_ms, &params->airspeed_delay_ms);
	param_get(_param_handles.requiredEph, &params->requiredEph);
	param_get(_param_handles.requiredEpv, &params->requiredEpv);
	param_get(_param_handles.gyro_noise, &params->gyro_noise);
	param_get(_param_handles.accel_noise, &params->accel_noise);
	param_get(_param_handles.gyro_bias_p_noise, &params->gyro_bias_p_noise);
	param_get(_param_handles.accel_bias_p_noise, &params->accel_bias_p_noise);
	param_get(_param_handles.gyro_scale_p_noise, &params->gyro_scale_p_noise);
	param_get(_param_handles.mag_p_noise, &params->mag_p_noise);
	param_get(_param_handles.wind_vel_p_noise, &params->wind_vel_p_noise);
	param_get(_param_handles.gps_vel_noise, &params->gps_vel_noise);
	param_get(_param_handles.gps_pos_noise, &params->gps_pos_noise);
	param_get(_param_handles.baro_noise, &params->baro_noise);
	param_get(_param_handles.mag_heading_noise, &params->mag_heading_noise);
	param_get(_param_handles.mag_declination_deg, &params->mag_declination_deg);
	param_get(_param_handles.heading_innov_gate, &params->heading_innov_gate);
}

void Ekf2::task_main()
{
	// subscribe to relevant topics
	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));


	// initialize parameters
	update_parameters();

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _sensors_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		int ret = px4_poll(fds, 1, 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0 || fds[0].revents != POLLIN) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		bool gps_updated = false;
		bool airspeed_updated = false;
		bool params_updated = false;

		sensor_combined_s sensors = {};
		vehicle_gps_position_s gps = {};
		airspeed_s airspeed = {};

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

		orb_check(_params_sub, &params_updated);

		if (params_updated) {
			struct parameter_update_s param_update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
			update_parameters();
		}

		// push imu data into estimator
		_ekf->setIMUData(sensors.gyro_timestamp[0], sensors.gyro_integral_dt[0], sensors.accelerometer_integral_dt[0],
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
			gps_msg.time_usec_vel = gps.timestamp_velocity;
			gps_msg.vel_m_s = gps.vel_m_s;
			gps_msg.vel_ned[0] = gps.vel_n_m_s;
			gps_msg.vel_ned[1] = gps.vel_e_m_s;
			gps_msg.vel_ned[2] = gps.vel_d_m_s;
			gps_msg.vel_ned_valid = gps.vel_ned_valid;

			_ekf->setGpsData(gps.timestamp_position, &gps_msg);
		}

		// read airspeed data if available
		if (airspeed_updated) {
			_ekf->setAirspeedData(airspeed.timestamp, &airspeed.indicated_airspeed_m_s);
		}

		struct vehicle_attitude_s att;

		struct vehicle_local_position_s lpos;

		att.timestamp = hrt_absolute_time();

		lpos.timestamp = hrt_absolute_time();

		_ekf->update();

		_ekf->copy_quaternion(att.q);

		matrix::Quaternion<float> q(att.q[0], att.q[1], att.q[2], att.q[3]);

		matrix::Euler<float> euler(q);

		att.roll = euler(0);

		att.pitch = euler(1);

		att.yaw = euler(2);

		float pos[3] = {};

		float vel[3] = {};

		_ekf->copy_position(pos);

		lpos.x = pos[0];

		lpos.y = pos[1];

		lpos.z = pos[2];

		_ekf->copy_velocity(vel);

		lpos.vx = vel[0];

		lpos.vy = vel[1];

		lpos.vz = vel[2];

		control_state_s ctrl_state = {};

		ctrl_state.timestamp = hrt_absolute_time();

		ctrl_state.roll_rate = _lp_roll_rate.apply(sensors.gyro_rad_s[0]);

		ctrl_state.pitch_rate = _lp_pitch_rate.apply(sensors.gyro_rad_s[1]);

		ctrl_state.yaw_rate = _lp_yaw_rate.apply(sensors.gyro_rad_s[2]);

		ctrl_state.q[0] = q(0);

		ctrl_state.q[1] = q(1);

		ctrl_state.q[2] = q(2);

		ctrl_state.q[3] = q(3);

		att.q[0] = q(0);

		att.q[1] = q(1);

		att.q[2] = q(2);

		att.q[3] = q(3);

		att.q_valid = true;

		att.rollspeed = sensors.gyro_rad_s[0];

		att.pitchspeed = sensors.gyro_rad_s[1];

		att.yawspeed = sensors.gyro_rad_s[2];

		if (_control_state_pub == nullptr) {
			_control_state_pub = orb_advertise(ORB_ID(control_state), &ctrl_state);

		} else {
			orb_publish(ORB_ID(control_state), _control_state_pub, &ctrl_state);
		}

		if (_att_pub == nullptr) {
			_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

		} else {
			orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
		}

		if (_lpos_pub == nullptr) {
			_lpos_pub = orb_advertise(ORB_ID(vehicle_local_position), &lpos);

		} else {
			orb_publish(ORB_ID(vehicle_local_position), _lpos_pub, &lpos);
		}


	}
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

		delete ekf2::instance;
		ekf2::instance = nullptr;
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
			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}

	PX4_WARN("unrecognized command");
	return 1;
}