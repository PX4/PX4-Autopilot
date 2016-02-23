/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file estimator_interface.cpp
 * Definition of base class for attitude estimators
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth B Purohit <siddharthbharatpurohit@gmail.com>
 */

#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <math.h>
#include "estimator_interface.h"
#include "mathlib.h"


EstimatorInterface::EstimatorInterface()
{
}

EstimatorInterface::~EstimatorInterface()
{

}

// Accumulate imu data and store to buffer at desired rate
void EstimatorInterface::setIMUData(uint64_t time_usec, uint64_t delta_ang_dt, uint64_t delta_vel_dt, float *delta_ang,
				    float *delta_vel)
{
	if (!_initialised) {
		init(time_usec);
		_initialised = true;
	}

	float dt = (float)(time_usec - _time_last_imu) / 1000 / 1000;
	dt = math::max(dt, 1.0e-4f);
	dt = math::min(dt, 0.02f);

	_time_last_imu = time_usec;

	if (_time_last_imu > 0) {
		_dt_imu_avg = 0.8f * _dt_imu_avg + 0.2f * dt;
	}

	// copy data
	imuSample imu_sample_new = {};
	memcpy(&imu_sample_new.delta_ang._data[0], delta_ang, sizeof(imu_sample_new.delta_ang._data));
	memcpy(&imu_sample_new.delta_vel._data[0], delta_vel, sizeof(imu_sample_new.delta_vel._data));

	//convert time from us to secs
	imu_sample_new.delta_ang_dt = delta_ang_dt / 1e6f;
	imu_sample_new.delta_vel_dt = delta_vel_dt / 1e6f;
	imu_sample_new.time_us = time_usec;
	_imu_ticks++;


	if (collect_imu(imu_sample_new)) {
		_imu_buffer.push(imu_sample_new);
		_imu_ticks = 0;
		_imu_updated = true;

	} else {
		_imu_updated = false;
	}


	_imu_sample_delayed = _imu_buffer.get_oldest();
}

void EstimatorInterface::setMagData(uint64_t time_usec, float *data)
{

	if (time_usec - _time_last_mag > 70000) {

		magSample mag_sample_new = {};
		mag_sample_new.time_us = time_usec  - _params.mag_delay_ms * 1000;

		mag_sample_new.time_us -= FILTER_UPDATE_PERRIOD_MS * 1000 / 2;
		_time_last_mag = time_usec;


		memcpy(&mag_sample_new.mag._data[0], data, sizeof(mag_sample_new.mag._data));

		_mag_buffer.push(mag_sample_new);
	}
}

void EstimatorInterface::setGpsData(uint64_t time_usec, struct gps_message *gps)
{
	if (!collect_gps(time_usec, gps) || !_initialised) {
		return;
	}

	// Only use GPS data if we have a 3D fix and limit the GPS data rate to a maximum of 14Hz
	if (time_usec - _time_last_gps > 70000 && gps->fix_type >= 3) {
		gpsSample gps_sample_new = {};
		gps_sample_new.time_us = gps->time_usec - _params.gps_delay_ms * 1000;

		gps_sample_new.time_us -= FILTER_UPDATE_PERRIOD_MS * 1000 / 2;
		_time_last_gps = time_usec;

		gps_sample_new.time_us = math::max(gps_sample_new.time_us, _imu_sample_delayed.time_us);

		memcpy(gps_sample_new.vel._data[0], gps->vel_ned, sizeof(gps_sample_new.vel._data));

		_gps_speed_valid = gps->vel_ned_valid;
		_gps_speed_accuracy = gps->sacc;

		float lpos_x = 0.0f;
		float lpos_y = 0.0f;
		map_projection_project(&_pos_ref, (gps->lat / 1.0e7), (gps->lon / 1.0e7), &lpos_x, &lpos_y);
		gps_sample_new.pos(0) = lpos_x;
		gps_sample_new.pos(1) = lpos_y;
		gps_sample_new.hgt = gps->alt / 1e3f;

		_gps_buffer.push(gps_sample_new);
	}
}

void EstimatorInterface::setBaroData(uint64_t time_usec, float *data)
{
	if (!collect_baro(time_usec, data) || !_initialised) {
		return;
	}

	if (time_usec - _time_last_baro > 70000) {

		baroSample baro_sample_new;
		baro_sample_new.hgt = *data;
		baro_sample_new.time_us = time_usec - _params.baro_delay_ms * 1000;

		baro_sample_new.time_us -= FILTER_UPDATE_PERRIOD_MS * 1000 / 2;
		_time_last_baro = time_usec;

		baro_sample_new.time_us = math::max(baro_sample_new.time_us, _imu_sample_delayed.time_us);

		_baro_buffer.push(baro_sample_new);
	}
}

void EstimatorInterface::setAirspeedData(uint64_t time_usec, float *data)
{
	if (!collect_airspeed(time_usec, data) || !_initialised) {
		return;
	}

	if (time_usec > _time_last_airspeed) {
		airspeedSample airspeed_sample_new;
		airspeed_sample_new.airspeed = *data;
		airspeed_sample_new.time_us = time_usec -_params.airspeed_delay_ms * 1000;
		airspeed_sample_new.time_us -= FILTER_UPDATE_PERRIOD_MS * 1000 / 2;
		_time_last_airspeed = time_usec;

		_airspeed_buffer.push(airspeed_sample_new);
	}
}

// set range data
void EstimatorInterface::setRangeData(uint64_t time_usec, float *data)
{
	if (!collect_range(time_usec, data) || !_initialised) {
		return;
	}
}

// set optical flow data
void EstimatorInterface::setOpticalFlowData(uint64_t time_usec, float *data)
{
	if (!collect_opticalflow(time_usec, data) || !_initialised) {
		return;
	}
}

bool EstimatorInterface::initialise_interface(uint64_t timestamp)
{

	if (!(_imu_buffer.allocate(IMU_BUFFER_LENGTH) &&
	      _gps_buffer.allocate(OBS_BUFFER_LENGTH) &&
	      _mag_buffer.allocate(OBS_BUFFER_LENGTH) &&
	      _baro_buffer.allocate(OBS_BUFFER_LENGTH) &&
	      _range_buffer.allocate(OBS_BUFFER_LENGTH) &&
	      _airspeed_buffer.allocate(OBS_BUFFER_LENGTH) &&
	      _flow_buffer.allocate(OBS_BUFFER_LENGTH) &&
	      _output_buffer.allocate(IMU_BUFFER_LENGTH))) {
		printf("Estimator Buffer Allocation failed!");
		unallocate_buffers();
		return false;
	}


	_dt_imu_avg = 0.0f;

	_imu_sample_delayed.delta_ang.setZero();
	_imu_sample_delayed.delta_vel.setZero();
	_imu_sample_delayed.delta_ang_dt = 0.0f;
	_imu_sample_delayed.delta_vel_dt = 0.0f;
	_imu_sample_delayed.time_us = timestamp;

	_imu_ticks = 0;

	_initialised = false;

	_time_last_imu = 0;
	_time_last_gps = 0;
	_time_last_mag = 0;
	_time_last_baro = 0;
	_time_last_range = 0;
	_time_last_airspeed = 0;

	memset(&_fault_status, 0, sizeof(_fault_status));
	return true;
}

void EstimatorInterface::unallocate_buffers()
{
	_imu_buffer.unallocate();
	_gps_buffer.unallocate();
	_mag_buffer.unallocate();
	_baro_buffer.unallocate();
	_range_buffer.unallocate();
	_airspeed_buffer.unallocate();
	_flow_buffer.unallocate();
	_output_buffer.unallocate();

}

bool EstimatorInterface::position_is_valid()
{
	// return true if the position estimate is valid
	// TOTO implement proper check based on published GPS accuracy, innovaton consistency checks and timeout status
	return _NED_origin_initialised && (_time_last_imu - _time_last_gps) < 5e6;
}
