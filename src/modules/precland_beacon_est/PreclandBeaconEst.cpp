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
 * @file PreclandBeaconEst.cpp
 *
 * @author Nicolas de Palezieux <ndepal@gmail.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>

#include "PreclandBeaconEst.h"

#define SEC2USEC 1000000.0f


namespace precland_beacon_est
{

PreclandBeaconEst::PreclandBeaconEst() :
	_preclandBeaconRelposPub(nullptr),
	_vehicleLocalPosition_valid(false),
	_vehicleAttitude_valid(false),
	_new_irlockReport(false),
	_estimator_initialized(false),
	_last_predict(0),
	_last_update(0),
	_taskShouldExit(false),
	_taskIsRunning(false),
	_work{}
{
}

PreclandBeaconEst::~PreclandBeaconEst()
{
	work_cancel(HPWORK, &_work);
	_taskShouldExit = true;
}

int PreclandBeaconEst::start()
{
	_taskShouldExit = false;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&PreclandBeaconEst::_cycle_trampoline, this, 0);

	return 0;
}

void PreclandBeaconEst::stop()
{
	_taskShouldExit = true;
}

void
PreclandBeaconEst::_cycle_trampoline(void *arg)
{
	PreclandBeaconEst *dev = reinterpret_cast<PreclandBeaconEst *>(arg);

	dev->_cycle();
}

void PreclandBeaconEst::_cycle()
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
		// TODO this is only valid if the beacon is stationary

		if (hrt_absolute_time() - _last_update > PRECLAND_BEACON_EST_TIMEOUT_US) {
			PX4_WARN("Timeout");
			_estimator_initialized = false;

		} else if (_vehicleLocalPosition_valid
			   && _vehicleLocalPosition_last_valid
			   && _vehicleLocalPosition.v_xy_valid
			   && _vehicleLocalPosition_last.v_xy_valid) {
			double dt = (hrt_absolute_time() - _last_predict) / SEC2USEC;
			double dt_vehicleLocalPosition = (_vehicleLocalPosition.timestamp - _vehicleLocalPosition_last.timestamp) / SEC2USEC;
			double acc_x = (_vehicleLocalPosition.vx - _vehicleLocalPosition_last.vx) / dt_vehicleLocalPosition;
			double acc_y = (_vehicleLocalPosition.vy - _vehicleLocalPosition_last.vy) / dt_vehicleLocalPosition;
			double acc_unc = 10; // Variance of acceleration [(m/s^2)^2] TODO what should this be?
			_kalman_filter_x.predict(dt, -acc_x, acc_unc);
			_kalman_filter_y.predict(dt, -acc_y, acc_unc);

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

		if (std::abs(sensor_ray(2)) < 1e-6) {
			PX4_WARN("z component of measurement unsafe: %f %f %f", (double)sensor_ray(0), (double)sensor_ray(1),
				 (double)sensor_ray(2));

		} else {
			// scale the ray s.t. the z component has length of HAGL
			_rel_pos(0) = sensor_ray(0) / sensor_ray(2) * _vehicleLocalPosition.dist_bottom;
			_rel_pos(1) = sensor_ray(1) / sensor_ray(2) * _vehicleLocalPosition.dist_bottom;

			if (!_estimator_initialized) {
				// too long since last measurement, reset filter
				PX4_WARN("Init");
				// TODO what whould these be?
				double covInit00 = 0.01; // initial variance of position [m^2]
				double covInit11 = 1; // initial variance of velocity [(m/2)^2]
				_kalman_filter_x.init(_rel_pos(0), 0, covInit00, covInit11);
				_kalman_filter_y.init(_rel_pos(1), 0, covInit00, covInit11);

				_estimator_initialized = true;
				_last_predict = hrt_absolute_time();

			} else {
				// update
				double measUnc = 0.01; // measurement variance [m^2] TODO what should this be?
				_kalman_filter_x.update(_rel_pos(0), measUnc);
				_kalman_filter_y.update(_rel_pos(1), measUnc);
			}

			_last_update = hrt_absolute_time();
		}

		_new_irlockReport = false;

	}

	if (_estimator_initialized) {
		// always publish, even if only predicted, unless timeout was exceeded
		_preclandBeaconRelpos.timestamp = hrt_absolute_time();
		double x, xvel, y, yvel, covx, covx_v, covy, covy_v;
		_kalman_filter_x.getState(x, xvel);
		_kalman_filter_x.getCovariance(covx, covx_v);

		_kalman_filter_y.getState(y, yvel);
		_kalman_filter_y.getCovariance(covy, covy_v);

		_preclandBeaconRelpos.x = x;
		_preclandBeaconRelpos.vx = xvel;
		_preclandBeaconRelpos.x_unfilt = _rel_pos(0);
		_preclandBeaconRelpos.covx = covx;
		_preclandBeaconRelpos.x_lpos = x + _vehicleLocalPosition.x;

		_preclandBeaconRelpos.y = y;
		_preclandBeaconRelpos.vy = yvel;
		_preclandBeaconRelpos.y_unfilt = _rel_pos(1);
		_preclandBeaconRelpos.covy = covy;
		_preclandBeaconRelpos.y_lpos = y + _vehicleLocalPosition.y;

		if (_preclandBeaconRelposPub == nullptr) {
			_preclandBeaconRelposPub = orb_advertise(ORB_ID(precland_beacon_relpos), &_preclandBeaconRelpos);

		} else {
			orb_publish(ORB_ID(precland_beacon_relpos), _preclandBeaconRelposPub, &_preclandBeaconRelpos);
		}
	}

	if (!_taskShouldExit) {

		// Schedule next cycle.
		work_queue(HPWORK, &_work, (worker_t)&PreclandBeaconEst::_cycle_trampoline, this,
			   USEC2TICK(1000000 / PRECLAND_BEACON_EST_UPDATE_RATE_HZ));

	} else {
		_taskIsRunning = false;
	}
}

void PreclandBeaconEst::_check_params(const bool force)
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

void PreclandBeaconEst::_initialize_topics()
{
	// subscribe to position, attitude, arming and velocity changes
	_vehicleLocalPositionSub = orb_subscribe(ORB_ID(vehicle_local_position));
	_attitudeSub = orb_subscribe(ORB_ID(vehicle_attitude));
	_irlockReportSub = orb_subscribe(ORB_ID(irlock_report));
}

void PreclandBeaconEst::_update_topics()
{
	if (_vehicleLocalPosition_valid) {
		_vehicleLocalPosition_last = _vehicleLocalPosition;
		_vehicleLocalPosition_last_valid = true;
	}

	_vehicleLocalPosition_valid = _orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub,
				      &_vehicleLocalPosition);
	_vehicleAttitude_valid = _orb_update(ORB_ID(vehicle_attitude), _attitudeSub, &_vehicleAttitude);

	_new_irlockReport = _orb_update(ORB_ID(irlock_report), _irlockReportSub, &_irlockReport);
}


bool PreclandBeaconEst::_orb_update(const struct orb_metadata *meta, int handle, void *buffer)
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

void PreclandBeaconEst::_update_params()
{

}


} // namespace precland_beacon_est
