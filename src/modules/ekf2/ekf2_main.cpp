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

	orb_advert_t _att_pub;
	orb_advert_t _lpos_pub;
	orb_advert_t _control_state_pub;

	/* Low pass filter for attitude rates */
	math::LowPassFilter2p _lp_roll_rate;
	math::LowPassFilter2p _lp_pitch_rate;
	math::LowPassFilter2p _lp_yaw_rate;

	EstimatorBase *_ekf;


	void update_parameters(bool force);

	int update_subscriptions();

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

void Ekf2::task_main()
{
	// subscribe to relevant topics
	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));

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