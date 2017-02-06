/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>

#include "PreclandBeaconEst.h"


namespace precland_beacon_est
{


PreclandBeaconEst::PreclandBeaconEst() :
	_preclandBeaconRelposPub(nullptr),
	_new_vehicleLocalPosition(false),
	_new_vehicleAttitude(false),
	_new_irlockReport(false),
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
		// Advertise the first land detected uORB.
		// _landDetected.timestamp = hrt_absolute_time();

		// Initialize uORB topics.
		_initialize_topics();

		_check_params(true);

		// Task is now running, keep doing so until we need to stop.
		_taskIsRunning = true;
	}

	_check_params(false);

	_update_topics();

	if (_new_irlockReport && _new_vehicleAttitude && _new_vehicleLocalPosition) {

		matrix::Vector<float, 3> sensor_ray; // ray pointing towards beacon in sensor frame
		sensor_ray(0) = _irlockReport.pos_x;
		sensor_ray(1) = _irlockReport.pos_y;
		sensor_ray(2) = 1.0f;

		// TODO account for sensor orientation

		// rotate the unit ray into the navigation frame, assume sensor frame = body frame
		matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
		_R_att = matrix::Dcm<float>(q_att);
		sensor_ray = _R_att * sensor_ray;

		if (std::abs(sensor_ray(2)) < 1e-6) {
			PX4_WARN("z component unsafe: %f %f %f", sensor_ray(0), sensor_ray(1), sensor_ray(2));
		}

		// scale the ray s.t. the z component has length of HAGL
		// TODO check for dist_bottom_valid
		matrix::Vector<float, 3> rel_pos;
		rel_pos(0) = sensor_ray(0) / sensor_ray(2);
		rel_pos(1) = sensor_ray(1) / sensor_ray(2);
		rel_pos(2) = _vehicleLocalPosition.dist_bottom;

		// TODO filter this in a simple EKF
		_preclandBeaconRelpos.timestamp = hrt_absolute_time();
		_preclandBeaconRelpos.x = rel_pos(0);
		_preclandBeaconRelpos.y = rel_pos(1);

		if (_preclandBeaconRelposPub == nullptr) {
			_preclandBeaconRelposPub = orb_advertise(ORB_ID(precland_beacon_relpos), &_preclandBeaconRelpos);
		} else {
			orb_publish(ORB_ID(precland_beacon_relpos), _preclandBeaconRelposPub, &_preclandBeaconRelpos);
		}

		_new_irlockReport = false;
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
	_new_vehicleLocalPosition = _orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub, &_vehicleLocalPosition);
	_new_vehicleAttitude = _orb_update(ORB_ID(vehicle_attitude), _attitudeSub, &_vehicleAttitude);
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
