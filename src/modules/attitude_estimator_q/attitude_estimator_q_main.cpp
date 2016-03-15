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
 *
 * Update to matrix and controllib.
 * @author James Goppert <james.goppert@gmail.com>
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
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/estimator_status.h>
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

#include <matrix/math.hpp>

#include <controllib/blocks.hpp>

extern "C" __EXPORT int attitude_estimator_q_main(int argc, char *argv[]);

using namespace matrix;
using namespace control;

class AttitudeEstimatorQ;

namespace attitude_estimator_q
{
AttitudeEstimatorQ *instance;
}


class AttitudeEstimatorQ: public control::SuperBlock

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

	// subscriptions
	uORB::Subscription<parameter_update_s> _sub_param_update;
	uORB::Subscription<sensor_combined_s> _sub_sensor;
	uORB::Subscription<vision_position_estimate_s> _sub_vision;
	uORB::Subscription<att_pos_mocap_s> _sub_mocap;
	uORB::Subscription<vehicle_global_position_s> _sub_gpos;
	uORB::Subscription<airspeed_s> _sub_airspeed;

	// publications
	uORB::Publication<vehicle_attitude_s> _pub_att;
	uORB::Publication<control_state_s> _pub_control_state;
	uORB::Publication<estimator_status_s> _pub_est_status;

	// parameters
	BlockParamFloat _w_acc;
	BlockParamFloat _w_mag;
	BlockParamFloat _w_ext_hdg;
	BlockParamFloat _w_gyro_bias;
	BlockParamFloat _param_mag_decl;
	BlockParamBool _mag_decl_auto;
	BlockParamBool _acc_comp;
	BlockParamFloat _bias_max;
	BlockParamFloat _vibe_thresh;
	BlockParamInt _ext_hdg_mode;

	hrt_abstime	_vibration_warning_timestamp = 0;

	float _mag_decl;
	Vector3f	_gyro;
	Vector3f	_accel;
	Vector3f	_mag;

	vision_position_estimate_s _vision = {};
	Vector3f	_vision_hdg;

	Vector3f	_mocap_hdg;

	Quatf	_q;
	Vector3f	_rates;
	Vector3f	_gyro_bias;

	Vector3f	_vel_prev;
	Vector3f	_pos_acc;

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
	bool		_ext_hdg_good = false;

	int		_mavlink_fd = -1;

	perf_counter_t _update_perf;
	perf_counter_t _loop_perf;

	void updateParams();

	int update_subscriptions();

	bool init();

	bool update(float dt);

	// Update magnetic declination (in rads) immediately changing yaw rotation
	void update_mag_declination(float new_declination);
};


AttitudeEstimatorQ::AttitudeEstimatorQ() :
	SuperBlock(NULL, "ATT"),

	// subscriptions
	_sub_param_update(ORB_ID(parameter_update), 0, 0, &getSubscriptions()),
	_sub_sensor(ORB_ID(sensor_combined), 0, 0, &getSubscriptions()),
	_sub_vision(ORB_ID(vision_position_estimate), 0, 0, &getSubscriptions()),
	_sub_mocap(ORB_ID(att_pos_mocap), 0, 0, &getSubscriptions()),
	_sub_gpos(ORB_ID(vehicle_global_position), 0, 0, &getSubscriptions()),
	_sub_airspeed(ORB_ID(airspeed), 0, 0, &getSubscriptions()),

	// publications
	_pub_att(ORB_ID(vehicle_attitude), -1, &getPublications()),
	_pub_control_state(ORB_ID(control_state), -1, &getPublications()),
	_pub_est_status(ORB_ID(estimator_status), -1, &getPublications()),

	// params
	_w_acc(this, "W_ACC"),
	_w_mag(this, "W_MAG"),
	_w_ext_hdg(this, "W_EXT_HDG"),
	_w_gyro_bias(this, "W_GYRO_BIAS"),
	_param_mag_decl(this, "MAG_DECL"),
	_mag_decl_auto(this, "MAG_DECL_A"),
	_acc_comp(this, "ACC_COMP"),
	_bias_max(this, "BIAS_MAX"),
	_vibe_thresh(this, "VIBE_THRESH"),
	_ext_hdg_mode(this, "EXT_HDG_M"),

	// states
	_vel_prev(0, 0, 0),
	_pos_acc(0, 0, 0),

	// voters
	_voter_gyro(3),
	_voter_accel(3),
	_voter_mag(3),

	// filters
	_lp_roll_rate(250.0f, 30.0f),
	_lp_pitch_rate(250.0f, 30.0f),
	_lp_yaw_rate(250.0f, 20.0f)
{
	_voter_mag.set_timeout(200000);
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
					   2500,
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

#ifdef __PX4_POSIX
	perf_counter_t _perf_accel(perf_alloc_once(PC_ELAPSED, "sim_accel_delay"));
	perf_counter_t _perf_mpu(perf_alloc_once(PC_ELAPSED, "sim_mpu_delay"));
	perf_counter_t _perf_mag(perf_alloc_once(PC_ELAPSED, "sim_mag_delay"));
#endif

	updateSubscriptions();
	updateParams();

	hrt_abstime last_time = 0;

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _sub_sensor.getHandle();
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		int ret = px4_poll(fds, 1, 1000);

		// see which updates are available
		bool sensorsUpdated = _sub_sensor.updated();
		bool visionUpdated = _sub_vision.updated();
		bool mocapUpdated = _sub_mocap.updated();
		//bool airspeedUpdated = _sub_airspeed.updated();
		bool paramsUpdated = _sub_param_update.updated();
		bool gposUpdated = _sub_gpos.updated();

		// get new data
		updateSubscriptions();

		// update parameters
		if (paramsUpdated) {
			updateParams();
		}

#ifndef __PX4_QURT

		if (_mavlink_fd < 0) {
			/* TODO: This call currently stalls the thread on QURT */
			_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
		}

#endif

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

		// Update sensors
		sensor_combined_s sensors = _sub_sensor.get();

		int best_gyro = 0;
		int best_accel = 0;
		int best_mag = 0;

		if (sensorsUpdated) {
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

				/* ignore empty fields */
				if (sensors.accelerometer_timestamp[i] > 0) {
					_voter_accel.put(i, sensors.accelerometer_timestamp[i], &sensors.accelerometer_m_s2[i * 3],
							 sensors.accelerometer_errcount[i], sensors.accelerometer_priority[i]);
				}

				/* ignore empty fields */
				if (sensors.magnetometer_timestamp[i] > 0) {
					_voter_mag.put(i, sensors.magnetometer_timestamp[i], &sensors.magnetometer_ga[i * 3],
						       sensors.magnetometer_errcount[i], sensors.magnetometer_priority[i]);
				}
			}

			// Get best measurement values
			hrt_abstime curr_time = hrt_absolute_time();
			_gyro = Vector3f(_voter_gyro.get_best(curr_time, &best_gyro));
			_accel = Vector3f(_voter_accel.get_best(curr_time, &best_accel));
			_mag = Vector3f(_voter_mag.get_best(curr_time, &best_mag));

			if (_accel.norm() < 0.01f) {
				warnx("WARNING: degenerate accel!");
				continue;
			}

			if (_mag.norm() < 0.01f) {
				warnx("WARNING: degenerate mag!");
				continue;
			}

			_data_good = true;

			if (!_failsafe) {
				uint32_t flags = DataValidator::ERROR_FLAG_NO_ERROR;

#ifdef __PX4_POSIX
				perf_end(_perf_accel);
				perf_end(_perf_mpu);
				perf_end(_perf_mag);
#endif

				if (_voter_gyro.failover_count() > 0) {
					_failsafe = true;
					flags = _voter_gyro.failover_state();
					mavlink_and_console_log_emergency(_mavlink_fd, "Gyro #%i failure :%s%s%s%s%s!",
									  _voter_gyro.failover_index(),
									  ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " No data" : ""),
									  ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " Stale data" : ""),
									  ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " Data timeout" : ""),
									  ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " High error count" : ""),
									  ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " High error density" : ""));
				}

				if (_voter_accel.failover_count() > 0) {
					_failsafe = true;
					flags = _voter_accel.failover_state();
					mavlink_and_console_log_emergency(_mavlink_fd, "Accel #%i failure :%s%s%s%s%s!",
									  _voter_accel.failover_index(),
									  ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " No data" : ""),
									  ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " Stale data" : ""),
									  ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " Data timeout" : ""),
									  ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " High error count" : ""),
									  ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " High error density" : ""));
				}

				if (_voter_mag.failover_count() > 0) {
					_failsafe = true;
					flags = _voter_mag.failover_state();
					mavlink_and_console_log_emergency(_mavlink_fd, "Mag #%i failure :%s%s%s%s%s!",
									  _voter_mag.failover_index(),
									  ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " No data" : ""),
									  ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " Stale data" : ""),
									  ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " Data timeout" : ""),
									  ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " High error count" : ""),
									  ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " High error density" : ""));
				}

				if (_failsafe) {
					mavlink_and_console_log_emergency(_mavlink_fd, "SENSOR FAILSAFE! RETURN TO LAND IMMEDIATELY");
				}
			}

			if (!_vibration_warning && (_voter_gyro.get_vibration_factor(curr_time) > _vibe_thresh.get() ||
						    _voter_accel.get_vibration_factor(curr_time) > _vibe_thresh.get() ||
						    _voter_mag.get_vibration_factor(curr_time) > _vibe_thresh.get())) {

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

		// Update vision
		if (visionUpdated) {
			Quatf q(_sub_vision.get().q);

			Dcmf Rvis(q);
			Vector3f v(1.0f, 0.0f, 0.4f);

			// Rvis is Rwr (robot respect to world) while v is respect to world.
			// Hence Rvis must be transposed having (Rwr)' * Vw
			// Rrw * Vw = vn. This way we have consistency
			_vision_hdg = Rvis.transpose() * v;
		}

		// Update motion capture heading
		if (mocapUpdated) {
			Quatf q(_sub_mocap.get().q);
			Dcmf Rmoc(q);

			Vector3f v(1.0f, 0.0f, 0.4f);

			// Rmoc is Rwr (robot respect to world) while v is respect to world.
			// Hence Rmoc must be transposed having (Rwr)' * Vw
			// Rrw * Vw = vn. This way we have consistency
			_mocap_hdg = Rmoc.transpose() * v;
		}

		// Check for timeouts on data
		if (_ext_hdg_mode.get() == 1) {
			_ext_hdg_good = _vision.timestamp_boot > 0
					&& (hrt_elapsed_time(&_vision.timestamp_boot) < 500000);

		} else if (_ext_hdg_mode.get() == 2) {
			_ext_hdg_good = _sub_mocap.get().timestamp_boot > 0
					&& (hrt_elapsed_time(&_sub_mocap.get().timestamp_boot) < 500000);
		}

		// Global position
		if (gposUpdated) {
			if (_mag_decl_auto.get() && _sub_gpos.get().eph < 20.0f
			    && hrt_elapsed_time(&_sub_gpos.get().timestamp) < 1000000) {
				/* set magnetic declination automatically */
				update_mag_declination(math::radians(get_mag_declination(_sub_gpos.get().lat, _sub_gpos.get().lon)));
			}
		}

		Dcmf C_nb(_q); // rotates from body frame to nav frame

		if (_acc_comp.get() && _sub_gpos.get().timestamp != 0
		    && hrt_absolute_time() < _sub_gpos.get().timestamp + 20000
		    && _sub_gpos.get().eph < 5.0f && _inited) {
			/* position data is actual */
			if (gposUpdated) {
				Vector3f vel(_sub_gpos.get().vel_n,
					     _sub_gpos.get().vel_e,
					     _sub_gpos.get().vel_d);

				/* velocity updated */
				if (_vel_prev_t != 0 && _sub_gpos.get().timestamp != _vel_prev_t) {
					float vel_dt = (_sub_gpos.get().timestamp - _vel_prev_t) / 1000000.0f;
					/* calculate acceleration in body frame */
					_pos_acc = C_nb.transpose() * ((vel - _vel_prev) / vel_dt);
				}

				_vel_prev_t = _sub_gpos.get().timestamp;
				_vel_prev = vel;
			}

		} else {
			/* position data is outdated, reset acceleration */
			_pos_acc.setZero();
			_vel_prev.setZero();
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

		// publish attitude
		{
			struct vehicle_attitude_s &att = _pub_att.get();
			att.timestamp = sensors.timestamp;

			Eulerf euler(_q);
			att.roll = euler(0);
			att.pitch = euler(1);
			att.yaw = euler(2);

			att.rollspeed = _rates(0);
			att.pitchspeed = _rates(1);
			att.yawspeed = _rates(2);

			for (int i = 0; i < 3; i++) {
				att.g_comp[i] = _accel(i) - _pos_acc(i);
			}

			/* copy offsets */
			memcpy(&att.rate_offsets, _gyro_bias.data(),
			       sizeof(att.rate_offsets));

			/* copy rotation matrix */
			memcpy(&att.R[0], C_nb.data(), sizeof(att.R));
			att.R_valid = true;
			memcpy(&att.q[0], _q.data(), sizeof(att.q));
			att.q_valid = true;

			att.rate_vibration = _voter_gyro.get_vibration_factor(hrt_absolute_time());
			att.accel_vibration = _voter_accel.get_vibration_factor(hrt_absolute_time());
			att.mag_vibration = _voter_mag.get_vibration_factor(hrt_absolute_time());

			_pub_att.update();
		}

		// publish control state
		{
			struct control_state_s &ctrl_state = _pub_control_state.get();

			ctrl_state.timestamp = sensors.timestamp;

			/* attitude quaternions for control state */
			ctrl_state.q[0] = _q(0);
			ctrl_state.q[1] = _q(1);
			ctrl_state.q[2] = _q(2);
			ctrl_state.q[3] = _q(3);

			/* attitude rates for control state */
			ctrl_state.roll_rate = _lp_roll_rate.apply(_rates(0));

			ctrl_state.pitch_rate = _lp_pitch_rate.apply(_rates(1));

			ctrl_state.yaw_rate = _lp_yaw_rate.apply(_rates(2));

			/* Airspeed - take airspeed measurement directly here as no wind is estimated */
			if (PX4_ISFINITE(_sub_airspeed.get().indicated_airspeed_m_s)
			    && hrt_absolute_time() - _sub_airspeed.get().timestamp < 1e6
			    && _sub_airspeed.get().timestamp > 0) {
				ctrl_state.airspeed = _sub_airspeed.get().indicated_airspeed_m_s;
				ctrl_state.airspeed_valid = true;

			} else {
				ctrl_state.airspeed_valid = false;
			}

			_pub_control_state.update();
		}

		// publish estimator status
		{
			struct estimator_status_s est = {};

			est.timestamp = sensors.timestamp;
			est.vibe[0] = _voter_accel.get_vibration_offset(est.timestamp, 0);
			est.vibe[1] = _voter_accel.get_vibration_offset(est.timestamp, 1);
			est.vibe[2] = _voter_accel.get_vibration_offset(est.timestamp, 2);

			/* the instance count is not used here */
			//int est_inst;
			/* publish to control state topic */
			// publishing this conflicts with LPE
			// better to publish in INAV and LPE
			//orb_publish_auto(ORB_ID(estimator_status), &_est_state_pub, &est, &est_inst, ORB_PRIO_HIGH);
		}
	}
}

void AttitudeEstimatorQ::updateParams()
{
	SuperBlock::updateParams();
	update_mag_declination(math::radians(_param_mag_decl.get()));
}

bool AttitudeEstimatorQ::init()
{
	// Rotation matrix can be easily constructed from acceleration and mag field vectors
	// 'k' is Earth Z axis (Down) unit vector in body frame
	Vector3f k = -_accel;
	k.normalize();

	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
	Vector3f i = Vector3f(_mag - k * _mag.dot(k)).unit();
	i.normalize();

	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
	Vector3f j = k.cross(i);
	j.normalize();

	// Fill rotation matrix
	Dcmf R;
	R.setRow(0, i);
	R.setRow(1, j);
	R.setRow(2, k);

	// Convert to quaternion
	_q = Quatf(R);

	// Compensate for magnetic declination
	Quatf decl_rotation(Eulerf(0, 0, _mag_decl));
	_q = decl_rotation * _q;

	_q.normalize();

	if (PX4_ISFINITE(_q(0)) && PX4_ISFINITE(_q(1)) &&
	    PX4_ISFINITE(_q(2)) && PX4_ISFINITE(_q(3)) &&
	    _q.norm() > 0.95f && _q.norm() < 1.05f) {
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

	Dcmf C_nb(_q);

	Quatf q_last = _q;

	// Angular rate of correction
	Vector3f corr;

	if (_ext_hdg_mode.get() > 0 && _ext_hdg_good) {
		if (_ext_hdg_mode.get() == 1) {
			// Vision heading correction
			// Project heading to global frame and extract XY component
			Vector3f vision_hdg_earth = C_nb * _vision_hdg;
			float vision_hdg_err = _wrap_pi(atan2f(vision_hdg_earth(1), vision_hdg_earth(0)));
			// Project correction to body frame
			corr += C_nb.transpose() * Vector3f(0.0f, 0.0f, -vision_hdg_err) * _w_ext_hdg.get();
		}

		if (_ext_hdg_mode.get() == 2) {
			// Mocap heading correction
			// Project heading to global frame and extract XY component
			Vector3f mocap_hdg_earth = C_nb * _mocap_hdg;
			float mocap_hdg_err = _wrap_pi(atan2f(mocap_hdg_earth(1), mocap_hdg_earth(0)));
			// Project correction to body frame
			corr += C_nb.transpose() * Vector3f(0.0f, 0.0f, -mocap_hdg_err) * _w_ext_hdg.get();
		}
	}

	if (_ext_hdg_mode.get() == 0  || !_ext_hdg_good) {
		// Magnetometer correction
		// Project mag field vector to global frame and extract XY component
		Vector3f mag_earth = C_nb * _mag;
		float mag_err = _wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);
		// Project magnetometer correction to body frame
		corr += C_nb.transpose() * Vector3f(0.0f, 0.0f, -mag_err) * _w_mag.get();
	}

	_q.normalize();

	// Accelerometer correction
	// Project 'k' unit vector of earth frame to body frame
	// Vector3f k = C_nb.transpose() * Vector3f(0.0f, 0.0f, 1.0f);
	// Optimized version with dropped zeros
	Vector3f k(
		2.0f * (_q(1) * _q(3) - _q(0) * _q(2)),
		2.0f * (_q(2) * _q(3) + _q(0) * _q(1)),
		(_q(0) * _q(0) - _q(1) * _q(1) - _q(2) * _q(2) + _q(3) * _q(3))
	);

	corr += Vector3f(k % Vector3f(_accel - _pos_acc)).unit() * _w_acc.get();

	// Gyro bias estimation
	_gyro_bias += corr * (_w_gyro_bias.get() * dt);

	for (int i = 0; i < 3; i++) {
		_gyro_bias(i) = math::constrain(_gyro_bias(i), -_bias_max.get(), _bias_max.get());
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
		_rates.setZero();
		_gyro_bias.setZero();
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
		Quatf decl_rotation(Eulerf(
					    0, 0, new_declination - _mag_decl));
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
