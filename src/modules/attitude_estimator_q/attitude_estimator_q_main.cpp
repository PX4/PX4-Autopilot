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

#include <cfloat>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <lib/ecl/geo_lookup/geo_mag_declination.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_odometry.h>

extern "C" __EXPORT int attitude_estimator_q_main(int argc, char *argv[]);

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::wrap_pi;

class AttitudeEstimatorQ;

namespace attitude_estimator_q
{
AttitudeEstimatorQ *instance;
} // namespace attitude_estimator_q


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

	static int	task_main_trampoline(int argc, char *argv[]);

	void		task_main();

private:
	const float _eo_max_std_dev = 100.0f;		/**< Maximum permissible standard deviation for estimated orientation */
	const float _dt_min = 0.00001f;
	const float _dt_max = 0.02f;

	bool		_task_should_exit = false;	/**< if true, task should exit */
	int		_control_task = -1;		/**< task handle for task */

	int		_params_sub = -1;
	int		_sensors_sub = -1;
	int		_global_pos_sub = -1;
	int		_vision_odom_sub = -1;
	int		_mocap_odom_sub = -1;
	int		_magnetometer_sub = -1;

	orb_advert_t	_att_pub = nullptr;

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
		param_t	has_mag;
	} _params_handles{};		/**< handles for interesting parameters */

	float		_w_accel = 0.0f;
	float		_w_mag = 0.0f;
	float		_w_ext_hdg = 0.0f;
	float		_w_gyro_bias = 0.0f;
	float		_mag_decl = 0.0f;
	bool		_mag_decl_auto = false;
	bool		_acc_comp = false;
	float		_bias_max = 0.0f;
	int32_t		_ext_hdg_mode = 0;

	Vector3f	_gyro;
	Vector3f	_accel;
	Vector3f	_mag;

	Vector3f	_vision_hdg;
	Vector3f	_mocap_hdg;

	Quatf		_q;
	Vector3f	_rates;
	Vector3f	_gyro_bias;

	Vector3f	_vel_prev;
	hrt_abstime	_vel_prev_t = 0;

	Vector3f	_pos_acc;

	bool		_inited = false;
	bool		_data_good = false;
	bool		_ext_hdg_good = false;

	void update_parameters(bool force);

	int update_subscriptions();

	bool init();

	bool update(float dt);

	// Update magnetic declination (in rads) immediately changing yaw rotation
	void update_mag_declination(float new_declination);
};


AttitudeEstimatorQ::AttitudeEstimatorQ()
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
	_params_handles.has_mag		= param_find("SYS_HAS_MAG");

	_vel_prev.zero();
	_pos_acc.zero();

	_gyro.zero();
	_accel.zero();
	_mag.zero();

	_vision_hdg.zero();
	_mocap_hdg.zero();

	_q.zero();
	_rates.zero();
	_gyro_bias.zero();
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
			px4_usleep(20000);

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
					   SCHED_PRIORITY_ESTIMATOR,
					   2000,
					   (px4_main_t)&AttitudeEstimatorQ::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int AttitudeEstimatorQ::task_main_trampoline(int argc, char *argv[])
{
	attitude_estimator_q::instance->task_main();
	return 0;
}

void AttitudeEstimatorQ::task_main()
{

#ifdef __PX4_POSIX
	perf_counter_t _perf_accel(perf_alloc_once(PC_ELAPSED, "sim_accel_delay"));
	perf_counter_t _perf_mpu(perf_alloc_once(PC_ELAPSED, "sim_mpu_delay"));
	perf_counter_t _perf_mag(perf_alloc_once(PC_ELAPSED, "sim_mag_delay"));
#endif

	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_vision_odom_sub = orb_subscribe(ORB_ID(vehicle_visual_odometry));
	_mocap_odom_sub = orb_subscribe(ORB_ID(vehicle_mocap_odometry));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_magnetometer_sub = orb_subscribe(ORB_ID(vehicle_magnetometer));

	update_parameters(true);

	hrt_abstime last_time = 0;

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _sensors_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		int ret = px4_poll(fds, 1, 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			px4_usleep(10000);
			PX4_WARN("POLL ERROR");
			continue;

		} else if (ret == 0) {
			// Poll timeout, do nothing
			PX4_WARN("POLL TIMEOUT");
			continue;
		}

		update_parameters(false);

		// Update sensors
		sensor_combined_s sensors;

		if (orb_copy(ORB_ID(sensor_combined), _sensors_sub, &sensors) == PX4_OK) {
			// Feed validator with recent sensor data

			if (sensors.timestamp > 0) {
				_gyro(0) = sensors.gyro_rad[0];
				_gyro(1) = sensors.gyro_rad[1];
				_gyro(2) = sensors.gyro_rad[2];
			}

			if (sensors.accelerometer_timestamp_relative != sensor_combined_s::RELATIVE_TIMESTAMP_INVALID) {
				_accel(0) = sensors.accelerometer_m_s2[0];
				_accel(1) = sensors.accelerometer_m_s2[1];
				_accel(2) = sensors.accelerometer_m_s2[2];

				if (_accel.length() < 0.01f) {
					PX4_ERR("degenerate accel!");
					continue;
				}
			}

			_data_good = true;
		}

		// Update magnetometer
		bool magnetometer_updated = false;
		orb_check(_magnetometer_sub, &magnetometer_updated);

		if (magnetometer_updated) {
			vehicle_magnetometer_s magnetometer;

			if (orb_copy(ORB_ID(vehicle_magnetometer), _magnetometer_sub, &magnetometer) == PX4_OK) {
				_mag(0) = magnetometer.magnetometer_ga[0];
				_mag(1) = magnetometer.magnetometer_ga[1];
				_mag(2) = magnetometer.magnetometer_ga[2];

				if (_mag.length() < 0.01f) {
					PX4_ERR("degenerate mag!");
					continue;
				}
			}

		}

		// Update vision and motion capture heading
		bool vision_updated = false;
		orb_check(_vision_odom_sub, &vision_updated);

		if (vision_updated) {
			vehicle_odometry_s vision;

			if (orb_copy(ORB_ID(vehicle_visual_odometry), _vision_odom_sub, &vision) == PX4_OK) {
				// validation check for vision attitude data
				bool vision_att_valid = PX4_ISFINITE(vision.q[0])
							&& (PX4_ISFINITE(vision.pose_covariance[vision.COVARIANCE_MATRIX_ROLL_VARIANCE]) ? sqrtf(fmaxf(
									vision.pose_covariance[vision.COVARIANCE_MATRIX_ROLL_VARIANCE],
									fmaxf(vision.pose_covariance[vision.COVARIANCE_MATRIX_PITCH_VARIANCE],
											vision.pose_covariance[vision.COVARIANCE_MATRIX_YAW_VARIANCE]))) <= _eo_max_std_dev : true);

				if (vision_att_valid) {
					Dcmf Rvis = Quatf(vision.q);
					Vector3f v(1.0f, 0.0f, 0.4f);

					// Rvis is Rwr (robot respect to world) while v is respect to world.
					// Hence Rvis must be transposed having (Rwr)' * Vw
					// Rrw * Vw = vn. This way we have consistency
					_vision_hdg = Rvis.transpose() * v;

					// vision external heading usage (ATT_EXT_HDG_M 1)
					if (_ext_hdg_mode == 1) {
						// Check for timeouts on data
						_ext_hdg_good = vision.timestamp > 0 && (hrt_elapsed_time(&vision.timestamp) < 500000);
					}
				}
			}
		}

		bool mocap_updated = false;
		orb_check(_mocap_odom_sub, &mocap_updated);

		if (mocap_updated) {
			vehicle_odometry_s mocap;

			if (orb_copy(ORB_ID(vehicle_mocap_odometry), _mocap_odom_sub, &mocap) == PX4_OK) {
				// validation check for mocap attitude data
				bool mocap_att_valid = PX4_ISFINITE(mocap.q[0])
						       && (PX4_ISFINITE(mocap.pose_covariance[mocap.COVARIANCE_MATRIX_ROLL_VARIANCE]) ? sqrtf(fmaxf(
								       mocap.pose_covariance[mocap.COVARIANCE_MATRIX_ROLL_VARIANCE],
								       fmaxf(mocap.pose_covariance[mocap.COVARIANCE_MATRIX_PITCH_VARIANCE],
										       mocap.pose_covariance[mocap.COVARIANCE_MATRIX_YAW_VARIANCE]))) <= _eo_max_std_dev : true);

				if (mocap_att_valid) {
					Dcmf Rmoc = Quatf(mocap.q);
					Vector3f v(1.0f, 0.0f, 0.4f);

					// Rmoc is Rwr (robot respect to world) while v is respect to world.
					// Hence Rmoc must be transposed having (Rwr)' * Vw
					// Rrw * Vw = vn. This way we have consistency
					_mocap_hdg = Rmoc.transpose() * v;

					// Motion Capture external heading usage (ATT_EXT_HDG_M 2)
					if (_ext_hdg_mode == 2) {
						// Check for timeouts on data
						_ext_hdg_good = mocap.timestamp > 0 && (hrt_elapsed_time(&mocap.timestamp) < 500000);
					}
				}
			}
		}

		bool gpos_updated = false;
		orb_check(_global_pos_sub, &gpos_updated);

		if (gpos_updated) {
			vehicle_global_position_s gpos;

			if (orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &gpos) == PX4_OK) {
				if (_mag_decl_auto && gpos.eph < 20.0f && hrt_elapsed_time(&gpos.timestamp) < 1000000) {
					/* set magnetic declination automatically */
					update_mag_declination(math::radians(get_mag_declination(gpos.lat, gpos.lon)));
				}

				if (_acc_comp && gpos.timestamp != 0 && hrt_absolute_time() < gpos.timestamp + 20000 && gpos.eph < 5.0f && _inited) {
					/* position data is actual */
					Vector3f vel(gpos.vel_n, gpos.vel_e, gpos.vel_d);

					/* velocity updated */
					if (_vel_prev_t != 0 && gpos.timestamp != _vel_prev_t) {
						float vel_dt = (gpos.timestamp - _vel_prev_t) / 1e6f;
						/* calculate acceleration in body frame */
						_pos_acc = _q.conjugate_inversed((vel - _vel_prev) / vel_dt);
					}

					_vel_prev_t = gpos.timestamp;
					_vel_prev = vel;

				} else {
					/* position data is outdated, reset acceleration */
					_pos_acc.zero();
					_vel_prev.zero();
					_vel_prev_t = 0;
				}
			}
		}

		/* time from previous iteration */
		hrt_abstime now = hrt_absolute_time();
		const float dt = math::constrain((now  - last_time) / 1e6f, _dt_min, _dt_max);
		last_time = now;

		if (update(dt)) {
			vehicle_attitude_s att = {};
			att.timestamp = sensors.timestamp;
			att.rollspeed = _rates(0);
			att.pitchspeed = _rates(1);
			att.yawspeed = _rates(2);
			_q.copyTo(att.q);

			/* the instance count is not used here */
			int att_inst;
			orb_publish_auto(ORB_ID(vehicle_attitude), &_att_pub, &att, &att_inst, ORB_PRIO_HIGH);
		}
	}

#ifdef __PX4_POSIX
	perf_end(_perf_accel);
	perf_end(_perf_mpu);
	perf_end(_perf_mag);
#endif

	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_sensors_sub);
	orb_unsubscribe(_global_pos_sub);
	orb_unsubscribe(_vision_odom_sub);
	orb_unsubscribe(_mocap_odom_sub);
	orb_unsubscribe(_magnetometer_sub);
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

		// disable mag fusion if the system does not have a mag
		if (_params_handles.has_mag != PARAM_INVALID) {
			int32_t has_mag;

			if (param_get(_params_handles.has_mag, &has_mag) == 0 && has_mag == 0) {
				_w_mag = 0.f;
			}
		}

		if (_w_mag < FLT_EPSILON) { // if the weight is zero (=mag disabled), make sure the estimator initializes
			_mag(0) = 1.f;
			_mag(1) = 0.f;
			_mag(2) = 0.f;
		}

		param_get(_params_handles.w_ext_hdg, &_w_ext_hdg);
		param_get(_params_handles.w_gyro_bias, &_w_gyro_bias);

		float mag_decl_deg = 0.0f;
		param_get(_params_handles.mag_decl, &mag_decl_deg);
		update_mag_declination(math::radians(mag_decl_deg));

		int32_t mag_decl_auto_int;
		param_get(_params_handles.mag_decl_auto, &mag_decl_auto_int);
		_mag_decl_auto = (mag_decl_auto_int != 0);

		int32_t acc_comp_int;
		param_get(_params_handles.acc_comp, &acc_comp_int);
		_acc_comp = (acc_comp_int != 0);

		param_get(_params_handles.bias_max, &_bias_max);
		param_get(_params_handles.ext_hdg_mode, &_ext_hdg_mode);
	}
}

bool AttitudeEstimatorQ::init()
{
	// Rotation matrix can be easily constructed from acceleration and mag field vectors
	// 'k' is Earth Z axis (Down) unit vector in body frame
	Vector3f k = -_accel;
	k.normalize();

	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
	Vector3f i = (_mag - k * (_mag * k));
	i.normalize();

	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
	Vector3f j = k % i;

	// Fill rotation matrix
	Dcmf R;
	R.setRow(0, i);
	R.setRow(1, j);
	R.setRow(2, k);

	// Convert to quaternion
	_q = R;

	// Compensate for magnetic declination
	Quatf decl_rotation = Eulerf(0.0f, 0.0f, _mag_decl);
	_q = _q * decl_rotation;

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

	Quatf q_last = _q;

	// Angular rate of correction
	Vector3f corr;
	float spinRate = _gyro.length();

	if (_ext_hdg_mode > 0 && _ext_hdg_good) {
		if (_ext_hdg_mode == 1) {
			// Vision heading correction
			// Project heading to global frame and extract XY component
			Vector3f vision_hdg_earth = _q.conjugate(_vision_hdg);
			float vision_hdg_err = wrap_pi(atan2f(vision_hdg_earth(1), vision_hdg_earth(0)));
			// Project correction to body frame
			corr += _q.conjugate_inversed(Vector3f(0.0f, 0.0f, -vision_hdg_err)) * _w_ext_hdg;
		}

		if (_ext_hdg_mode == 2) {
			// Mocap heading correction
			// Project heading to global frame and extract XY component
			Vector3f mocap_hdg_earth = _q.conjugate(_mocap_hdg);
			float mocap_hdg_err = wrap_pi(atan2f(mocap_hdg_earth(1), mocap_hdg_earth(0)));
			// Project correction to body frame
			corr += _q.conjugate_inversed(Vector3f(0.0f, 0.0f, -mocap_hdg_err)) * _w_ext_hdg;
		}
	}

	if (_ext_hdg_mode == 0 || !_ext_hdg_good) {
		// Magnetometer correction
		// Project mag field vector to global frame and extract XY component
		Vector3f mag_earth = _q.conjugate(_mag);
		float mag_err = wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);
		float gainMult = 1.0f;
		const float fifty_dps = 0.873f;

		if (spinRate > fifty_dps) {
			gainMult = math::min(spinRate / fifty_dps, 10.0f);
		}

		// Project magnetometer correction to body frame
		corr += _q.conjugate_inversed(Vector3f(0.0f, 0.0f, -mag_err)) * _w_mag * gainMult;
	}

	_q.normalize();


	// Accelerometer correction
	// Project 'k' unit vector of earth frame to body frame
	// Vector3f k = _q.conjugate_inversed(Vector3f(0.0f, 0.0f, 1.0f));
	// Optimized version with dropped zeros
	Vector3f k(
		2.0f * (_q(1) * _q(3) - _q(0) * _q(2)),
		2.0f * (_q(2) * _q(3) + _q(0) * _q(1)),
		(_q(0) * _q(0) - _q(1) * _q(1) - _q(2) * _q(2) + _q(3) * _q(3))
	);

	// If we are not using acceleration compensation based on GPS velocity,
	// fuse accel data only if its norm is close to 1 g (reduces drift).
	const float accel_norm_sq = _accel.norm_squared();
	const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
	const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;

	if (_acc_comp || (accel_norm_sq > lower_accel_limit * lower_accel_limit &&
			  accel_norm_sq < upper_accel_limit * upper_accel_limit)) {
		corr += (k % (_accel - _pos_acc).normalized()) * _w_accel;
	}

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
	_q += _q.derivative1(corr) * dt;

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
		Quatf decl_rotation = Eulerf(0.0f, 0.0f, new_declination - _mag_decl);
		_q = _q * decl_rotation;
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
