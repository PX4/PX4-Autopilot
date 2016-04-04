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


EstimatorInterface::EstimatorInterface():
	_dt_imu_avg(0.0f),
	_imu_ticks(0),
	_imu_updated(false),
	_initialised(false),
	_vehicle_armed(false),
	_in_air(false),
	_NED_origin_initialised(false),
	_gps_speed_valid(false),
	_gps_origin_eph(0.0f),
	_gps_origin_epv(0.0f),
	_mag_healthy(false),
	_yaw_test_ratio(0.0f),
	_time_last_imu(0),
	_time_last_gps(0),
	_time_last_mag(0),
	_time_last_baro(0),
	_time_last_range(0),
	_time_last_airspeed(0),
	_mag_declination_gps(0.0f),
	_mag_declination_to_save_deg(0.0f)
{
	_pos_ref = {};
	memset(_mag_test_ratio, 0, sizeof(_mag_test_ratio));
	memset(_vel_pos_test_ratio, 0, sizeof(_vel_pos_test_ratio));
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
	// Limit the GPS data rate to a maximum of 14Hz
	if (time_usec - _time_last_gps > 70000) {
		gpsSample gps_sample_new = {};
		gps_sample_new.time_us = gps->time_usec - _params.gps_delay_ms * 1000;

		gps_sample_new.time_us -= FILTER_UPDATE_PERRIOD_MS * 1000 / 2;
		_time_last_gps = time_usec;

		gps_sample_new.time_us = math::max(gps_sample_new.time_us, _imu_sample_delayed.time_us);

		memcpy(gps_sample_new.vel._data[0], gps->vel_ned, sizeof(gps_sample_new.vel._data));

		_gps_speed_valid = gps->vel_ned_valid;
		gps_sample_new.sacc = gps->sacc;
		gps_sample_new.hacc = gps->eph;
		gps_sample_new.vacc = gps->epv;

		gps_sample_new.hgt = (float)gps->alt * 1e-3f;

		// Only calculate the relative position if the WGS-84 location of the origin is set
		if (collect_gps(time_usec, gps)) {
			float lpos_x = 0.0f;
			float lpos_y = 0.0f;
			map_projection_project(&_pos_ref, (gps->lat / 1.0e7), (gps->lon / 1.0e7), &lpos_x, &lpos_y);
			gps_sample_new.pos(0) = lpos_x;
			gps_sample_new.pos(1) = lpos_y;

		} else {
			gps_sample_new.pos(0) = 0.0f;
			gps_sample_new.pos(1) = 0.0f;
		}

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

	if (time_usec - _time_last_airspeed > 80000) {
		airspeedSample airspeed_sample_new;
		airspeed_sample_new.true_airspeed = *data;
		airspeed_sample_new.time_us = time_usec - _params.airspeed_delay_ms * 1000;
		airspeed_sample_new.time_us -= FILTER_UPDATE_PERRIOD_MS * 1000 / 2; //typo PeRRiod
		_time_last_airspeed = time_usec;

		_airspeed_buffer.push(airspeed_sample_new);
	}
}
static float rng;
// set range data
void EstimatorInterface::setRangeData(uint64_t time_usec, float *data)
{
	if (!collect_range(time_usec, data) || !_initialised) {
		return;
	}

	if (time_usec > _time_last_range) {
		rangeSample range_sample_new;
		range_sample_new.rng = *data;
		rng = *data;
		range_sample_new.time_us -= _params.range_delay_ms * 1000;

		range_sample_new.time_us = time_usec;
		_time_last_range = time_usec;

		_range_buffer.push(range_sample_new);
	}
}

// set optical flow data
void EstimatorInterface::setOpticalFlowData(uint64_t time_usec, flow_message *flow)
{
	if (!collect_opticalflow(time_usec, flow) || !_initialised) {
		return;
	}

	// if data passes checks, push to buffer
	if (time_usec > _time_last_optflow) {
		// check if enough integration time
		float delta_time = 1e-6f * (float)flow->dt;
		bool delta_time_good = (delta_time >= 0.05f);

		// check magnitude is within sensor limits
		float flow_rate_magnitude;
		bool flow_magnitude_good = false;

		if (delta_time_good) {
			flow_rate_magnitude = flow->flowdata.norm() / delta_time;
			flow_magnitude_good = (flow_rate_magnitude <= _params.flow_rate_max);
		}

		// check quality metric
		bool flow_quality_good = (flow->quality >= _params.flow_qual_min);

		if (delta_time_good && flow_magnitude_good && flow_quality_good) {
			flowSample optflow_sample_new;
			// calculate the system time-stamp for the mid point of the integration period
			optflow_sample_new.time_us = time_usec - _params.flow_delay_ms * 1000 - flow->dt / 2;
			// copy the quality metric returned by the PX4Flow sensor
			optflow_sample_new.quality = flow->quality;
			// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate is produced by a RH rotation of the image about the sensor axis.
			// copy the optical and gyro measured delta angles
			optflow_sample_new.flowRadXY = - flow->flowdata;
			optflow_sample_new.gyroXY = - flow->gyrodata;
			// compensate for body motion to give a LOS rate
			optflow_sample_new.flowRadXYcomp = optflow_sample_new.flowRadXY - optflow_sample_new.gyroXY;
			// convert integraton interval to seconds
			optflow_sample_new.dt = 1e-6f * (float)flow->dt;
			_time_last_optflow = time_usec;
			// push to buffer
			_flow_buffer.push(optflow_sample_new);
		}
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
		printf("EKF buffer allocation failed!");
		unallocate_buffers();
		return false;
	}

	// zero the data in the observation buffers
	for (int index=0; index < OBS_BUFFER_LENGTH; index++) {
		gpsSample gps_sample_init = {};
		_gps_buffer.push(gps_sample_init);
		magSample mag_sample_init = {};
		_mag_buffer.push(mag_sample_init);
		baroSample baro_sample_init = {};
		_baro_buffer.push(baro_sample_init);
		rangeSample range_sample_init = {};
		_range_buffer.push(range_sample_init);
		airspeedSample airspeed_sample_init = {};
		_airspeed_buffer.push(airspeed_sample_init);
		flowSample flow_sample_init = {};
		_flow_buffer.push(flow_sample_init);
	}

	// zero the data in the imu data and output observer state buffers
	for (int index=0; index < IMU_BUFFER_LENGTH; index++) {
		imuSample imu_sample_init = {};
		_imu_buffer.push(imu_sample_init);
		outputSample output_sample_init = {};
		_output_buffer.push(output_sample_init);
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
	_time_last_optflow = 0;

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

bool EstimatorInterface::local_position_is_valid()
{
	// return true if the position estimate is valid
	return ((_time_last_imu - _time_last_optflow) < 5e6) || global_position_is_valid();
}
