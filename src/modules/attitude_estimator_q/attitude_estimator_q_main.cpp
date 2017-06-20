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

/*
 * @file attitude_estimator_q_main.cpp
 *
 * Attitude estimator (quaternion based)
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <float.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/estimator_status.h>
#include <drivers/drv_hrt.h>

#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/geo/geo.h>

#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

extern "C" __EXPORT int attitude_estimator_q_main(int argc, char *argv[]);

using math::Vector;
using math::Matrix;
using math::Quaternion;

class AttitudeEstimatorQ;

namespace attitude_estimator_q
{
AttitudeEstimatorQ *instance;
}


class AttitudeEstimatorQ
{
public:
	/**
	 * Constructor
	 */
	AttitudeEstimatorQ();

	/**
	 * Destructor, also kills task.
	 */
	~AttitudeEstimatorQ();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	static void	task_main_trampoline(int argc, char *argv[]);

	void		task_main();

	void		print();

private:
	static constexpr float _dt_max = 0.02;
	bool		_task_should_exit = false;		/**< if true, task should exit */
	int		_control_task = -1;			/**< task handle for task */

	int		_sensors_sub = -1;
	int		_params_sub = -1;
	int		_vision_sub = -1;
	int		_mocap_sub = -1;
	int		_airspeed_sub = -1;
	int		_global_pos_sub = -1;
	orb_advert_t	_att_pub = nullptr;
	orb_advert_t	_est_state_pub = nullptr;

	struct {
		param_t	w_acc;
		param_t	w_mag;
		param_t	w_ext_hdg;
		param_t	w_gyro_bias;
		param_t	mag_decl;
		param_t	mag_decl_auto;
		param_t	acc_comp;
		param_t	bias_max;
		param_t	ext_hdg_mode;
		param_t airspeed_mode;
	}		_params_handles;		/**< handles for interesting parameters */

	float		_w_accel = 0.0f;
	float		_w_mag = 0.0f;
	float		_w_ext_hdg = 0.0f;
	float		_w_gyro_bias = 0.0f;
	float		_mag_decl = 0.0f;
	bool		_mag_decl_auto = false;
	bool		_acc_comp = false;
	float		_bias_max = 0.0f;
	int		_ext_hdg_mode = 0;
	int 	_airspeed_mode = 0;

	Vector<3>	_gyro;
	Vector<3>	_accel;
	Vector<3>	_mag;

	vehicle_attitude_s _vision = {};
	Vector<3>	_vision_hdg;

	att_pos_mocap_s _mocap = {};
	Vector<3>	_mocap_hdg;

	airspeed_s _airspeed = {};

	Quaternion	_q;
	Vector<3>	_rates;
	Vector<3>	_gyro_bias;

	vehicle_global_position_s _gpos = {};
	Vector<3>	_vel_prev;
	Vector<3>	_pos_acc;

	/* Low pass filter for accel/gyro */
	math::LowPassFilter2p _lp_accel_x;
	math::LowPassFilter2p _lp_accel_y;
	math::LowPassFilter2p _lp_accel_z;

	hrt_abstime _vel_prev_t = 0;

	bool		_inited = false;
	bool		_data_good = false;
	bool		_ext_hdg_good = false;

	orb_advert_t	_mavlink_log_pub = nullptr;

	void update_parameters(bool force);

	int update_subscriptions();

	bool init();

	bool update(float dt);

	// Update magnetic declination (in rads) immediately changing yaw rotation
	void update_mag_declination(float new_declination);
};


AttitudeEstimatorQ::AttitudeEstimatorQ() :
	_vel_prev(0, 0, 0),
	_pos_acc(0, 0, 0),
	_lp_accel_x(250.0f, 30.0f),
	_lp_accel_y(250.0f, 30.0f),
	_lp_accel_z(250.0f, 30.0f)
{
	_params_handles.w_acc		= param_find("ATT_W_ACC");
	_params_handles.w_mag		= param_find("ATT_W_MAG");
	_params_handles.w_ext_hdg	= param_find("ATT_W_EXT_HDG");
	_params_handles.w_gyro_bias	= param_find("ATT_W_GYRO_BIAS");
	_params_handles.mag_decl	= param_find("ATT_MAG_DECL");
	_params_handles.mag_decl_auto	= param_find("ATT_MAG_DECL_A");
	_params_handles.acc_comp	= param_find("ATT_ACC_COMP");
	_params_handles.bias_max	= param_find("ATT_BIAS_MAX");
	_params_handles.ext_hdg_mode	= param_find("ATT_EXT_HDG_M");
	_params_handles.airspeed_mode = param_find("FW_ARSP_MODE");
}

/**
 * Destructor, also kills task.
 */
AttitudeEstimatorQ::~AttitudeEstimatorQ()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	attitude_estimator_q::instance = nullptr;
}

int AttitudeEstimatorQ::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("attitude_estimator_q",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   2000,
					   (px4_main_t)&AttitudeEstimatorQ::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void AttitudeEstimatorQ::print()
{
}

void AttitudeEstimatorQ::task_main_trampoline(int argc, char *argv[])
{
	attitude_estimator_q::instance->task_main();
}

void AttitudeEstimatorQ::task_main()
{

#ifdef __PX4_POSIX
	perf_counter_t _perf_accel(perf_alloc_once(PC_ELAPSED, "sim_accel_delay"));
	perf_counter_t _perf_mpu(perf_alloc_once(PC_ELAPSED, "sim_mpu_delay"));
	perf_counter_t _perf_mag(perf_alloc_once(PC_ELAPSED, "sim_mag_delay"));
#endif

	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));

	_vision_sub = orb_subscribe(ORB_ID(vehicle_vision_attitude));
	_mocap_sub = orb_subscribe(ORB_ID(att_pos_mocap));

	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));

	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	update_parameters(true);

	hrt_abstime last_time = 0;

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _sensors_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		int ret = px4_poll(fds, 1, 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			PX4_WARN("Q POLL ERROR");
			continue;

		} else if (ret == 0) {
			// Poll timeout, do nothing
			PX4_WARN("Q POLL TIMEOUT");
			continue;
		}

		update_parameters(false);

		// Update sensors
		sensor_combined_s sensors;

		if (!orb_copy(ORB_ID(sensor_combined), _sensors_sub, &sensors)) {
			// Feed validator with recent sensor data

			if (sensors.timestamp > 0) {
				_gyro(0) = sensors.gyro_rad[0];
				_gyro(1) = sensors.gyro_rad[1];
				_gyro(2) = sensors.gyro_rad[2];
			}

			if (sensors.accelerometer_timestamp_relative != sensor_combined_s::RELATIVE_TIMESTAMP_INVALID) {
				// Filter accel signal since it is not filtered in the drivers.
				_accel(0) = _lp_accel_x.apply(sensors.accelerometer_m_s2[0]);
				_accel(1) = _lp_accel_y.apply(sensors.accelerometer_m_s2[1]);
				_accel(2) = _lp_accel_z.apply(sensors.accelerometer_m_s2[2]);

				if (_accel.length() < 0.01f) {
					PX4_DEBUG("WARNING: degenerate accel!");
					continue;
				}
			}

			if (sensors.magnetometer_timestamp_relative != sensor_combined_s::RELATIVE_TIMESTAMP_INVALID) {
				_mag(0) = sensors.magnetometer_ga[0];
				_mag(1) = sensors.magnetometer_ga[1];
				_mag(2) = sensors.magnetometer_ga[2];

				if (_mag.length() < 0.01f) {
					PX4_DEBUG("WARNING: degenerate mag!");
					continue;
				}
			}

			_data_good = true;
		}

		// Update vision and motion capture heading
		bool vision_updated = false;
		orb_check(_vision_sub, &vision_updated);

		bool mocap_updated = false;
		orb_check(_mocap_sub, &mocap_updated);

		if (vision_updated) {
			orb_copy(ORB_ID(vehicle_vision_attitude), _vision_sub, &_vision);
			math::Quaternion q(_vision.q);

			math::Matrix<3, 3> Rvis = q.to_dcm();
			math::Vector<3> v(1.0f, 0.0f, 0.4f);

			// Rvis is Rwr (robot respect to world) while v is respect to world.
			// Hence Rvis must be transposed having (Rwr)' * Vw
			// Rrw * Vw = vn. This way we have consistency
			_vision_hdg = Rvis.transposed() * v;
		}

		if (mocap_updated) {
			orb_copy(ORB_ID(att_pos_mocap), _mocap_sub, &_mocap);
			math::Quaternion q(_mocap.q);
			math::Matrix<3, 3> Rmoc = q.to_dcm();

			math::Vector<3> v(1.0f, 0.0f, 0.4f);

			// Rmoc is Rwr (robot respect to world) while v is respect to world.
			// Hence Rmoc must be transposed having (Rwr)' * Vw
			// Rrw * Vw = vn. This way we have consistency
			_mocap_hdg = Rmoc.transposed() * v;
		}

		// Update airspeed
		bool airspeed_updated = false;
		orb_check(_airspeed_sub, &airspeed_updated);

		if (airspeed_updated) {
			orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
		}

		// Check for timeouts on data
		if (_ext_hdg_mode == 1) {
			_ext_hdg_good = _vision.timestamp > 0 && (hrt_elapsed_time(&_vision.timestamp) < 500000);

		} else if (_ext_hdg_mode == 2) {
			_ext_hdg_good = _mocap.timestamp > 0 && (hrt_elapsed_time(&_mocap.timestamp) < 500000);
		}

		bool gpos_updated;
		orb_check(_global_pos_sub, &gpos_updated);

		if (gpos_updated) {
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_gpos);

			if (_mag_decl_auto && _gpos.eph < 20.0f && hrt_elapsed_time(&_gpos.timestamp) < 1000000) {
				/* set magnetic declination automatically */
				update_mag_declination(math::radians(get_mag_declination(_gpos.lat, _gpos.lon)));
			}
		}

		if (_acc_comp && _gpos.timestamp != 0 && hrt_absolute_time() < _gpos.timestamp + 20000 && _gpos.eph < 5.0f && _inited) {
			/* position data is actual */
			if (gpos_updated) {
				Vector<3> vel(_gpos.vel_n, _gpos.vel_e, _gpos.vel_d);

				/* velocity updated */
				if (_vel_prev_t != 0 && _gpos.timestamp != _vel_prev_t) {
					float vel_dt = (_gpos.timestamp - _vel_prev_t) / 1000000.0f;
					/* calculate acceleration in body frame */
					_pos_acc = _q.conjugate_inversed((vel - _vel_prev) / vel_dt);
				}

				_vel_prev_t = _gpos.timestamp;
				_vel_prev = vel;
			}

		} else {
			/* position data is outdated, reset acceleration */
			_pos_acc.zero();
			_vel_prev.zero();
			_vel_prev_t = 0;
		}

		/* time from previous iteration */
		hrt_abstime now = hrt_absolute_time();
		float dt = (last_time > 0) ? ((now  - last_time) / 1000000.0f) : 0.00001f;
		last_time = now;

		if (dt > _dt_max) {
			dt = _dt_max;
		}

		if (!update(dt)) {
			continue;
		}

		vehicle_attitude_s att = {
			.timestamp = sensors.timestamp,
			.rollspeed = _rates(0),
			.pitchspeed = _rates(1),
			.yawspeed = _rates(2),

			/* TODO get bias estimates from estimator */
			.roll_rate_bias = 0.0f,
			.pitch_rate_bias = 0.0f,
			.yaw_rate_bias = 0.0f,

			.q = {_q(0), _q(1), _q(2), _q(3)},
			.delta_q_reset = {},
			.quat_reset_counter = 0,
		};

		/* the instance count is not used here */
		int att_inst;
		orb_publish_auto(ORB_ID(vehicle_attitude), &_att_pub, &att, &att_inst, ORB_PRIO_HIGH);

		{
			//struct estimator_status_s est = {};

			//est.timestamp = sensors.timestamp;

			/* the instance count is not used here */
			//int est_inst;
			/* publish to control state topic */
			// TODO handle attitude states in position estimators instead so we can publish all data at once
			// or we need to enable more thatn just one estimator_status topic
			// orb_publish_auto(ORB_ID(estimator_status), &_est_state_pub, &est, &est_inst, ORB_PRIO_HIGH);
		}
	}

#ifdef __PX4_POSIX
	perf_end(_perf_accel);
	perf_end(_perf_mpu);
	perf_end(_perf_mag);
#endif

	orb_unsubscribe(_sensors_sub);
	orb_unsubscribe(_vision_sub);
	orb_unsubscribe(_mocap_sub);
	orb_unsubscribe(_airspeed_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_global_pos_sub);
}

void AttitudeEstimatorQ::update_parameters(bool force)
{
	bool updated = force;

	if (!updated) {
		orb_check(_params_sub, &updated);
	}

	if (updated) {
		parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);

		param_get(_params_handles.w_acc, &_w_accel);
		param_get(_params_handles.w_mag, &_w_mag);
		param_get(_params_handles.w_ext_hdg, &_w_ext_hdg);
		param_get(_params_handles.w_gyro_bias, &_w_gyro_bias);
		float mag_decl_deg = 0.0f;
		param_get(_params_handles.mag_decl, &mag_decl_deg);
		update_mag_declination(math::radians(mag_decl_deg));
		int32_t mag_decl_auto_int;
		param_get(_params_handles.mag_decl_auto, &mag_decl_auto_int);
		_mag_decl_auto = mag_decl_auto_int != 0;
		int32_t acc_comp_int;
		param_get(_params_handles.acc_comp, &acc_comp_int);
		_acc_comp = acc_comp_int != 0;
		param_get(_params_handles.bias_max, &_bias_max);
		param_get(_params_handles.ext_hdg_mode, &_ext_hdg_mode);
		param_get(_params_handles.airspeed_mode, &_airspeed_mode);
	}
}

bool AttitudeEstimatorQ::init()
{
	// Rotation matrix can be easily constructed from acceleration and mag field vectors
	// 'k' is Earth Z axis (Down) unit vector in body frame
	Vector<3> k = -_accel;
	k.normalize();

	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
	Vector<3> i = (_mag - k * (_mag * k));
	i.normalize();

	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
	Vector<3> j = k % i;

	// Fill rotation matrix
	Matrix<3, 3> R;
	R.set_row(0, i);
	R.set_row(1, j);
	R.set_row(2, k);

	// Convert to quaternion
	_q.from_dcm(R);

	// Compensate for magnetic declination
	Quaternion decl_rotation;
	decl_rotation.from_yaw(_mag_decl);
	_q = decl_rotation * _q;

	_q.normalize();

	if (PX4_ISFINITE(_q(0)) && PX4_ISFINITE(_q(1)) &&
	    PX4_ISFINITE(_q(2)) && PX4_ISFINITE(_q(3)) &&
	    _q.length() > 0.95f && _q.length() < 1.05f) {
		_inited = true;

	} else {
		_inited = false;
	}

	return _inited;
}

bool AttitudeEstimatorQ::update(float dt)
{
	if (!_inited) {

		if (!_data_good) {
			return false;
		}

		return init();
	}

	Quaternion q_last = _q;

	// Angular rate of correction
	Vector<3> corr;
	float spinRate = _gyro.length();

	if (_ext_hdg_mode > 0 && _ext_hdg_good) {
		if (_ext_hdg_mode == 1) {
			// Vision heading correction
			// Project heading to global frame and extract XY component
			Vector<3> vision_hdg_earth = _q.conjugate(_vision_hdg);
			float vision_hdg_err = _wrap_pi(atan2f(vision_hdg_earth(1), vision_hdg_earth(0)));
			// Project correction to body frame
			corr += _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, -vision_hdg_err)) * _w_ext_hdg;
		}

		if (_ext_hdg_mode == 2) {
			// Mocap heading correction
			// Project heading to global frame and extract XY component
			Vector<3> mocap_hdg_earth = _q.conjugate(_mocap_hdg);
			float mocap_hdg_err = _wrap_pi(atan2f(mocap_hdg_earth(1), mocap_hdg_earth(0)));
			// Project correction to body frame
			corr += _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, -mocap_hdg_err)) * _w_ext_hdg;
		}
	}

	if (_ext_hdg_mode == 0  || !_ext_hdg_good) {
		// Magnetometer correction
		// Project mag field vector to global frame and extract XY component
		Vector<3> mag_earth = _q.conjugate(_mag);
		float mag_err = _wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);
		float gainMult = 1.0f;
		const float fifty_dps = 0.873f;

		if (spinRate > fifty_dps) {
			gainMult = math::min(spinRate / fifty_dps, 10.0f);
		}

		// Project magnetometer correction to body frame
		corr += _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, -mag_err)) * _w_mag * gainMult;
	}

	_q.normalize();


	// Accelerometer correction
	// Project 'k' unit vector of earth frame to body frame
	// Vector<3> k = _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, 1.0f));
	// Optimized version with dropped zeros
	Vector<3> k(
		2.0f * (_q(1) * _q(3) - _q(0) * _q(2)),
		2.0f * (_q(2) * _q(3) + _q(0) * _q(1)),
		(_q(0) * _q(0) - _q(1) * _q(1) - _q(2) * _q(2) + _q(3) * _q(3))
	);

	corr += (k % (_accel - _pos_acc).normalized()) * _w_accel;

	// Gyro bias estimation
	if (spinRate < 0.175f) {
		_gyro_bias += corr * (_w_gyro_bias * dt);

		for (int i = 0; i < 3; i++) {
			_gyro_bias(i) = math::constrain(_gyro_bias(i), -_bias_max, _bias_max);
		}

	}

	_rates = _gyro + _gyro_bias;

	// Feed forward gyro
	corr += _rates;

	// Apply correction to state
	_q += _q.derivative(corr) * dt;

	// Normalize quaternion
	_q.normalize();

	if (!(PX4_ISFINITE(_q(0)) && PX4_ISFINITE(_q(1)) &&
	      PX4_ISFINITE(_q(2)) && PX4_ISFINITE(_q(3)))) {
		// Reset quaternion to last good state
		_q = q_last;
		_rates.zero();
		_gyro_bias.zero();
		return false;
	}

	return true;
}

void AttitudeEstimatorQ::update_mag_declination(float new_declination)
{
	// Apply initial declination or trivial rotations without changing estimation
	if (!_inited || fabsf(new_declination - _mag_decl) < 0.0001f) {
		_mag_decl = new_declination;

	} else {
		// Immediately rotate current estimation to avoid gyro bias growth
		Quaternion decl_rotation;
		decl_rotation.from_yaw(new_declination - _mag_decl);
		_q = decl_rotation * _q;
		_mag_decl = new_declination;
	}
}

int attitude_estimator_q_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: attitude_estimator_q {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (attitude_estimator_q::instance != nullptr) {
			warnx("already running");
			return 1;
		}

		attitude_estimator_q::instance = new AttitudeEstimatorQ;

		if (attitude_estimator_q::instance == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != attitude_estimator_q::instance->start()) {
			delete attitude_estimator_q::instance;
			attitude_estimator_q::instance = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (attitude_estimator_q::instance == nullptr) {
			warnx("not running");
			return 1;
		}

		delete attitude_estimator_q::instance;
		attitude_estimator_q::instance = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (attitude_estimator_q::instance) {
			attitude_estimator_q::instance->print();
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
