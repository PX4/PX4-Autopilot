/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file BeaconPositionEstimator.cpp
 *
 * @author Nicolas de Palezieux <ndepal@gmail.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>

#include "BeaconPositionEstimator.h"

#define SEC2USEC 1000000.0f


namespace beacon_position_estimator
{

BeaconPositionEstimator::BeaconPositionEstimator() :
	_beaconPositionPub(nullptr),
	_paramHandle(),
	_vehicleLocalPosition_valid(false),
	_vehicleAttitude_valid(false),
	_sensorCombined_valid(false),
	_new_irlockReport(false),
	_estimator_initialized(false),
	_last_predict(0),
	_last_update(0),
	_taskShouldExit(false),
	_taskIsRunning(false),
	_work{}
{
	_paramHandle.acc_unc = param_find("BEST_ACC_UNC");
	_paramHandle.meas_unc = param_find("BEST_MEAS_UNC");
	_paramHandle.pos_unc_init = param_find("BEST_POS_UNC_IN");
	_paramHandle.vel_unc_init = param_find("BEST_VEL_UNC_IN");
	_paramHandle.mode = param_find("BEST_MODE");
}

BeaconPositionEstimator::~BeaconPositionEstimator()
{
	work_cancel(HPWORK, &_work);
	_taskShouldExit = true;
}

int BeaconPositionEstimator::start()
{
	_taskShouldExit = false;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&BeaconPositionEstimator::_cycle_trampoline, this, 0);

	return 0;
}

void BeaconPositionEstimator::stop()
{
	_taskShouldExit = true;
}

void
BeaconPositionEstimator::_cycle_trampoline(void *arg)
{
	BeaconPositionEstimator *dev = reinterpret_cast<BeaconPositionEstimator *>(arg);

	dev->_cycle();
}

void BeaconPositionEstimator::_cycle()
{
	if (!_taskIsRunning) {
		// Initialize uORB topics.
		_initialize_topics();

		_check_params(true);

		// Task is now running, keep doing so until we need to stop.
		_taskIsRunning = true;
	}

	_check_params(false);

	_update_topics();

	if (_estimator_initialized) {
		/* predict */
		// only run prediction if filter has been initialized
		// use the best estimate of the vehicle acceleration to predict the beacon position

		if (hrt_absolute_time() - _last_update > beacon_position_estimator_TIMEOUT_US) {
			PX4_WARN("Timeout");
			_estimator_initialized = false;

		} else if (_vehicleLocalPosition_valid
			   && _vehicleLocalPosition_last_valid
			   && _vehicleLocalPosition.v_xy_valid
			   && _vehicleLocalPosition_last.v_xy_valid) {
			float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC;

			// predict beacon position with the help of accel data
			matrix::Vector3f a;

			if (_sensorCombined_valid) {
				matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
				_R_att = matrix::Dcm<float>(q_att);
				a = _sensorCombined.accelerometer_m_s2;
				a = _R_att * a;

			} else {
				a.zero();
			}

			_kalman_filter_x.predict(dt, -a(0), _params.acc_unc);
			_kalman_filter_y.predict(dt, -a(1), _params.acc_unc);

			_last_predict = hrt_absolute_time();
		}
	}

	if (_new_irlockReport
	    && _vehicleAttitude_valid
	    && _vehicleLocalPosition_valid
	    && _vehicleLocalPosition.xy_valid
	    && _vehicleLocalPosition.v_xy_valid
	    && _vehicleLocalPosition.dist_bottom_valid) {

		// TODO account for sensor orientation as set by parameter
		// default orientation has camera x pointing in body y, camera y in body -x

		matrix::Vector<float, 3> sensor_ray; // ray pointing towards beacon in body frame
		sensor_ray(0) = -_irlockReport.pos_y;//_irlockReport.pos_x; // forward
		sensor_ray(1) = _irlockReport.pos_x;//_irlockReport.pos_y; // right
		sensor_ray(2) = 1.0f;

		// rotate the unit ray into the navigation frame, assume sensor frame = body frame
		matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
		_R_att = matrix::Dcm<float>(q_att);
		sensor_ray = _R_att * sensor_ray;

		if (fabs(sensor_ray(2)) < 1e-6) {
			PX4_WARN("z component of measurement unsafe: %f %f %f", (double)sensor_ray(0), (double)sensor_ray(1),
				 (double)sensor_ray(2));

		} else {
			// scale the ray s.t. the z component has length of HAGL
			_rel_pos(0) = sensor_ray(0) / sensor_ray(2) * _vehicleLocalPosition.dist_bottom;
			_rel_pos(1) = sensor_ray(1) / sensor_ray(2) * _vehicleLocalPosition.dist_bottom;

			if (!_estimator_initialized) {
				// too long since last measurement, reset filter
				PX4_WARN("Init");
				_kalman_filter_x.init(_rel_pos(0), 0, _params.pos_unc_init, _params.vel_unc_init);
				_kalman_filter_y.init(_rel_pos(1), 0, _params.pos_unc_init, _params.vel_unc_init);

				_estimator_initialized = true;
				_last_predict = hrt_absolute_time();

			} else {
				// update
				_kalman_filter_x.update(_rel_pos(0), _params.meas_unc);
				_kalman_filter_y.update(_rel_pos(1), _params.meas_unc);

				_beacon_position.timestamp = hrt_absolute_time();
				float x, xvel, y, yvel, covx, covx_v, covy, covy_v;
				_kalman_filter_x.getState(x, xvel);
				_kalman_filter_x.getCovariance(covx, covx_v);

				_kalman_filter_y.getState(y, yvel);
				_kalman_filter_y.getCovariance(covy, covy_v);

				_beacon_position.rel_pos_valid = true; // relative position is always valid, independent of mode
				_beacon_position.rel_vel_valid = true; // relative velocity is always valid, independent of mode
				_beacon_position.x_rel = x;
				_beacon_position.y_rel = y;
				_beacon_position.vx_rel = xvel;
				_beacon_position.vy_rel = yvel;
				_beacon_position.x_unfilt = _rel_pos(0);
				_beacon_position.y_unfilt = _rel_pos(1);

				_beacon_position.cov_x_rel = covx;
				_beacon_position.cov_y_rel = covy;

				_beacon_position.cov_vx_rel = covx_v;
				_beacon_position.cov_vy_rel = covy_v;

				if (_vehicleLocalPosition_valid && _vehicleLocalPosition.xy_valid && _vehicleLocalPosition.v_xy_valid) {
					_beacon_position.local_pos_valid = true;
					_beacon_position.local_vel_valid = true;

					// In stationary mode, relative velocity is used in the position estimator, so vx/vy_local should always be 0
					// Mark it as invalid to signal that it should not be used, write the values anyway for debugging
					if (_params.mode > BeaconMode::Moving) {
						_beacon_position.local_vel_valid = false;
					}

					// In known location mode, relative position ins used in the position estimator, so v/y_local should always correspond to BEST_LAT/BEST_LON
					// mark it as invalid to signal that it should not be used, write the values anyway for debugging
					if (_params.mode > BeaconMode::Stationary) {
						_beacon_position.local_pos_valid = false;
					}

					_beacon_position.x_local = x + _vehicleLocalPosition.x;
					_beacon_position.vx_local = xvel + _vehicleLocalPosition.vx;

					_beacon_position.y_local = y + _vehicleLocalPosition.y;
					_beacon_position.vy_local = yvel + _vehicleLocalPosition.vy;

				} else {
					_beacon_position.local_pos_valid = false;
					_beacon_position.local_vel_valid = false;
				}

				if (_beaconPositionPub == nullptr) {
					_beaconPositionPub = orb_advertise(ORB_ID(beacon_position), &_beacon_position);

				} else {
					orb_publish(ORB_ID(beacon_position), _beaconPositionPub, &_beacon_position);
				}
			}

			_last_update = hrt_absolute_time();
		}

		_new_irlockReport = false;

	}

	if (!_taskShouldExit) {

		// Schedule next cycle.
		work_queue(HPWORK, &_work, (worker_t)&BeaconPositionEstimator::_cycle_trampoline, this,
			   USEC2TICK(1000000 / beacon_position_estimator_UPDATE_RATE_HZ));

	} else {
		_taskIsRunning = false;
	}
}

void BeaconPositionEstimator::_check_params(const bool force)
{
	bool updated;
	parameter_update_s paramUpdate;

	orb_check(_parameterSub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameterSub, &paramUpdate);
	}

	if (updated || force) {
		_update_params();
	}
}

void BeaconPositionEstimator::_initialize_topics()
{
	// subscribe to position, attitude, arming and velocity changes
	_vehicleLocalPositionSub = orb_subscribe(ORB_ID(vehicle_local_position));
	_attitudeSub = orb_subscribe(ORB_ID(vehicle_attitude));
	_sensorCombinedSub = orb_subscribe(ORB_ID(sensor_combined));
	_irlockReportSub = orb_subscribe(ORB_ID(irlock_report));
	_parameterSub = orb_subscribe(ORB_ID(parameter_update));
}

void BeaconPositionEstimator::_update_topics()
{
	if (_vehicleLocalPosition_valid) {
		_vehicleLocalPosition_last = _vehicleLocalPosition;
		_vehicleLocalPosition_last_valid = true;
	}

	_vehicleLocalPosition_valid = _orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub,
				      &_vehicleLocalPosition);
	_vehicleAttitude_valid = _orb_update(ORB_ID(vehicle_attitude), _attitudeSub, &_vehicleAttitude);
	_sensorCombined_valid = _orb_update(ORB_ID(sensor_combined), _sensorCombinedSub, &_sensorCombined);

	_new_irlockReport = _orb_update(ORB_ID(irlock_report), _irlockReportSub, &_irlockReport);
}


bool BeaconPositionEstimator::_orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
	bool newData = false;

	// check if there is new data to grab
	if (orb_check(handle, &newData) != OK) {
		return false;
	}

	if (!newData) {
		return false;
	}

	if (orb_copy(meta, handle, buffer) != OK) {
		return false;
	}

	return true;
}

void BeaconPositionEstimator::_update_params()
{
	param_get(_paramHandle.acc_unc, &_params.acc_unc);
	param_get(_paramHandle.meas_unc, &_params.meas_unc);
	param_get(_paramHandle.pos_unc_init, &_params.pos_unc_init);
	param_get(_paramHandle.vel_unc_init, &_params.vel_unc_init);
	param_get(_paramHandle.mode, &_params.mode);
}


} // namespace beacon_position_estimator
