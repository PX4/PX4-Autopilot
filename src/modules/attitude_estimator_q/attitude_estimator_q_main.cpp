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
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>

#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/geo/geo.h>
#include <lib/ecl/validation/data_validator_group.h>
#include <mavlink/mavlink_log.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

extern "C" __EXPORT int attitude_estimator_q_main(int argc, char *argv[]);

using math::Vector;
using math::Matrix;
using math::Quaternion;

class AttitudeEstimatorQ;

namespace attitude_estimator_q {
AttitudeEstimatorQ *instance;
}


class AttitudeEstimatorQ {
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
	int		_global_pos_sub = -1;
	orb_advert_t	_att_pub = nullptr;

	struct {
		param_t	w_acc;
		param_t	w_mag;
		param_t	w_gyro_bias;
		param_t	mag_decl;
		param_t	mag_decl_auto;
		param_t	acc_comp;
		param_t	bias_max;
		param_t vibe_thresh;
	}		_params_handles;		/**< handles for interesting parameters */

	float		_w_accel = 0.0f;
	float		_w_mag = 0.0f;
	float		_w_gyro_bias = 0.0f;
	float		_mag_decl = 0.0f;
	bool		_mag_decl_auto = false;
	bool		_acc_comp = false;
	float		_bias_max = 0.0f;
	float		_vibration_warning_threshold = 1.0f;
	hrt_abstime	_vibration_warning_timestamp = 0;

	Vector<3>	_gyro;
	Vector<3>	_accel;
	Vector<3>	_mag;

	Quaternion	_q;
	Vector<3>	_rates;
	Vector<3>	_gyro_bias;

	vehicle_global_position_s _gpos = {};
	Vector<3>	_vel_prev;
	Vector<3>	_pos_acc;

	DataValidatorGroup _voter_gyro;
	DataValidatorGroup _voter_accel;
	DataValidatorGroup _voter_mag;

	/* Low pass filter for attitude rates */
	math::LowPassFilter2p _lp_roll_rate;
	math::LowPassFilter2p _lp_pitch_rate;
	math::LowPassFilter2p _lp_yaw_rate;

	hrt_abstime _vel_prev_t = 0;

	bool		_inited = false;
	bool		_data_good = false;
	bool		_failsafe = false;
	bool		_vibration_warning = false;

	int		_mavlink_fd = -1;

	perf_counter_t _update_perf;
	perf_counter_t _loop_perf;

	void update_parameters(bool force);

	int update_subscriptions();

	bool init();

	bool update(float dt);
};


AttitudeEstimatorQ::AttitudeEstimatorQ() :
	_voter_gyro(3),
	_voter_accel(3),
	_voter_mag(3),
	_lp_roll_rate(250.0f, 18.0f),
	_lp_pitch_rate(250.0f, 18.0f),
	_lp_yaw_rate(250, 10.0f)
{
	_voter_mag.set_timeout(200000);
	
	_params_handles.w_acc		= param_find("ATT_W_ACC");
	_params_handles.w_mag		= param_find("ATT_W_MAG");
	_params_handles.w_gyro_bias	= param_find("ATT_W_GYRO_BIAS");
	_params_handles.mag_decl	= param_find("ATT_MAG_DECL");
	_params_handles.mag_decl_auto	= param_find("ATT_MAG_DECL_A");
	_params_handles.acc_comp	= param_find("ATT_ACC_COMP");
	_params_handles.bias_max	= param_find("ATT_BIAS_MAX");
	_params_handles.vibe_thresh	= param_find("ATT_VIBE_THRESH");
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
				       2100,
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
	warnx("gyro status:");
	_voter_gyro.print();
	warnx("accel status:");
	_voter_accel.print();
	warnx("mag status:");
	_voter_mag.print();
}

void AttitudeEstimatorQ::task_main_trampoline(int argc, char *argv[])
{
	attitude_estimator_q::instance->task_main();
}

void AttitudeEstimatorQ::task_main()
{
	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	update_parameters(true);

	hrt_abstime last_time = 0;

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _sensors_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		int ret = px4_poll(fds, 1, 1000);

		if (_mavlink_fd < 0) {
			_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
		}

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;
		} else if (ret == 0) {
			// Poll timeout, do nothing
			continue;
		}

		update_parameters(false);

		// Update sensors
		sensor_combined_s sensors;
		if (!orb_copy(ORB_ID(sensor_combined), _sensors_sub, &sensors)) {
			// Feed validator with recent sensor data

			for (unsigned i = 0; i < (sizeof(sensors.gyro_timestamp) / sizeof(sensors.gyro_timestamp[0])); i++) {

				/* ignore empty fields */
				if (sensors.gyro_timestamp[i] > 0) {

					float gyro[3];

					for (unsigned j = 0; j < 3; j++) {
						if (sensors.gyro_integral_dt[i] > 0) {
							gyro[j] = (double)sensors.gyro_integral_rad[i * 3 + j] / (sensors.gyro_integral_dt[i] / 1e6);
						} else {
							/* fall back to angular rate */
							gyro[j] = sensors.gyro_rad_s[i * 3 + j];
						}
					}

					_voter_gyro.put(i, sensors.gyro_timestamp[i], &gyro[0], sensors.gyro_errcount[i], sensors.gyro_priority[i]);
				}
				_voter_accel.put(i, sensors.accelerometer_timestamp[i], &sensors.accelerometer_m_s2[i * 3],
					sensors.accelerometer_errcount[i], sensors.accelerometer_priority[i]);
				_voter_mag.put(i, sensors.magnetometer_timestamp[i], &sensors.magnetometer_ga[i * 3],
					sensors.magnetometer_errcount[i], sensors.magnetometer_priority[i]);
			}

			int best_gyro, best_accel, best_mag;

			// Get best measurement values
			hrt_abstime curr_time = hrt_absolute_time();
			_gyro.set(_voter_gyro.get_best(curr_time, &best_gyro));
			_accel.set(_voter_accel.get_best(curr_time, &best_accel));
			_mag.set(_voter_mag.get_best(curr_time, &best_mag));

			if (_accel.length() < 0.01f || _mag.length() < 0.01f) {
				warnx("WARNING: degenerate accel / mag!");
				continue;
			}

			_data_good = true;

			if (!_failsafe && (_voter_gyro.failover_count() > 0 ||
				_voter_accel.failover_count() > 0 ||
				_voter_mag.failover_count() > 0)) {

				_failsafe = true;
				mavlink_and_console_log_emergency(_mavlink_fd, "SENSOR FAILSAFE! RETURN TO LAND IMMEDIATELY");
			}

			if (!_vibration_warning && (_voter_gyro.get_vibration_factor(curr_time) > _vibration_warning_threshold ||
				_voter_accel.get_vibration_factor(curr_time) > _vibration_warning_threshold ||
				_voter_mag.get_vibration_factor(curr_time) > _vibration_warning_threshold)) {

				if (_vibration_warning_timestamp == 0) {
					_vibration_warning_timestamp = curr_time;
				} else if (hrt_elapsed_time(&_vibration_warning_timestamp) > 10000000) {
					_vibration_warning = true;
					mavlink_and_console_log_critical(_mavlink_fd, "HIGH VIBRATION! g: %d a: %d m: %d",
						(int)(100 * _voter_gyro.get_vibration_factor(curr_time)),
						(int)(100 * _voter_accel.get_vibration_factor(curr_time)),
						(int)(100 * _voter_mag.get_vibration_factor(curr_time)));
				}
			} else {
				_vibration_warning_timestamp = 0;
			}
		}

		bool gpos_updated;
		orb_check(_global_pos_sub, &gpos_updated);
		if (gpos_updated) {
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_gpos);
			if (_mag_decl_auto && _gpos.eph < 20.0f && hrt_elapsed_time(&_gpos.timestamp) < 1000000) {
				/* set magnetic declination automatically */
				_mag_decl = math::radians(get_mag_declination(_gpos.lat, _gpos.lon));
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

		// Time from previous iteration
		hrt_abstime now = hrt_absolute_time();
		float dt = (last_time > 0) ? ((now  - last_time) / 1000000.0f) : 0.00001f;
		last_time = now;

		if (dt > _dt_max) {
			dt = _dt_max;
		}

		if (!update(dt)) {
			continue;
		}

		Vector<3> euler = _q.to_euler();

		struct vehicle_attitude_s att = {};
		att.timestamp = sensors.timestamp;

		att.roll = euler(0);
		att.pitch = euler(1);
		att.yaw = euler(2);

		/* the complimentary filter should reflect the true system
		 * state, but we need smoother outputs for the control system
		 */
		att.rollspeed = _lp_roll_rate.apply(_rates(0));
		att.pitchspeed = _lp_pitch_rate.apply(_rates(1));
		att.yawspeed = _lp_yaw_rate.apply(_rates(2));

		for (int i = 0; i < 3; i++) {
			att.g_comp[i] = _accel(i) - _pos_acc(i);
		}

		/* copy offsets */
		memcpy(&att.rate_offsets, _gyro_bias.data, sizeof(att.rate_offsets));

		Matrix<3, 3> R = _q.to_dcm();

		/* copy rotation matrix */
		memcpy(&att.R[0], R.data, sizeof(att.R));
		att.R_valid = true;

		att.rate_vibration = _voter_gyro.get_vibration_factor(hrt_absolute_time());
		att.accel_vibration = _voter_accel.get_vibration_factor(hrt_absolute_time());
		att.mag_vibration = _voter_mag.get_vibration_factor(hrt_absolute_time());

		if (_att_pub == nullptr) {
			_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);
		} else {
			orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
		}
	}
}

void AttitudeEstimatorQ::update_parameters(bool force) {
	bool updated = force;
	if (!updated) {
		orb_check(_params_sub, &updated);
	}
	if (updated) {
		parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);

		param_get(_params_handles.w_acc, &_w_accel);
		param_get(_params_handles.w_mag, &_w_mag);
		param_get(_params_handles.w_gyro_bias, &_w_gyro_bias);
		float mag_decl_deg = 0.0f;
		param_get(_params_handles.mag_decl, &mag_decl_deg);
		_mag_decl = math::radians(mag_decl_deg);
		int32_t mag_decl_auto_int;
		param_get(_params_handles.mag_decl_auto, &mag_decl_auto_int);
		_mag_decl_auto = mag_decl_auto_int != 0;
		int32_t acc_comp_int;
		param_get(_params_handles.acc_comp, &acc_comp_int);
		_acc_comp = acc_comp_int != 0;
		param_get(_params_handles.bias_max, &_bias_max);
		param_get(_params_handles.vibe_thresh, &_vibration_warning_threshold);
	}
}

bool AttitudeEstimatorQ::init() {
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

bool AttitudeEstimatorQ::update(float dt) {
	if (!_inited) {

		if (!_data_good) {
			return false;
		}

		return init();
	}

	Quaternion q_last = _q;

	// Angular rate of correction
	Vector<3> corr;

	// Magnetometer correction
	// Project mag field vector to global frame and extract XY component
	Vector<3> mag_earth = _q.conjugate(_mag);
	float mag_err = _wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);
	// Project magnetometer correction to body frame
	corr += _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, -mag_err)) * _w_mag;

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
	_gyro_bias += corr * (_w_gyro_bias * dt);
	for (int i = 0; i < 3; i++) {
		_gyro_bias(i) = math::constrain(_gyro_bias(i), -_bias_max, _bias_max);
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


int attitude_estimator_q_main(int argc, char *argv[]) {
	if (argc < 1) {
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
