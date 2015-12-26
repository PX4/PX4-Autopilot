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
 * @file estimator_base.cpp
 * Definition of base class for attitude estimators
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include <math.h>
#include "estimator_base.h"
#include <mathlib/mathlib.h>


EstimatorBase::EstimatorBase()
{
}

EstimatorBase::~EstimatorBase()
{

}

// Accumulate imu data and store to buffer at desired rate
void EstimatorBase::setIMUData(uint64_t time_usec, uint64_t delta_ang_dt, uint64_t delta_vel_dt, float *delta_ang,
			       float *delta_vel)
{
	if (!_initialised) {
		initialiseVariables(time_usec);
		_initialised = true;
		_start_predict_enabled = true;
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

	imu_sample_new.delta_ang_dt = delta_ang_dt / 1e6f;
	imu_sample_new.delta_vel_dt = delta_vel_dt / 1e6f;

	imu_sample_new.time_us = time_usec;

	imu_sample_new.delta_ang(0) = imu_sample_new.delta_ang(0) * _state.gyro_scale(0);
	imu_sample_new.delta_ang(1) = imu_sample_new.delta_ang(1) * _state.gyro_scale(1);
	imu_sample_new.delta_ang(2) = imu_sample_new.delta_ang(2) * _state.gyro_scale(2);

	imu_sample_new.delta_ang -= _state.gyro_bias * imu_sample_new.delta_ang_dt / (_dt_imu_avg > 0 ? _dt_imu_avg : 0.01f);
	imu_sample_new.delta_vel(2) -= _state.accel_z_bias * imu_sample_new.delta_vel_dt / (_dt_imu_avg > 0 ? _dt_imu_avg : 0.01f);;

	// store the new sample for the complementary filter prediciton
	_imu_sample_new = imu_sample_new;

	_imu_down_sampled.delta_ang_dt += imu_sample_new.delta_ang_dt;
	_imu_down_sampled.delta_vel_dt += imu_sample_new.delta_vel_dt;


	Quaternion delta_q;
	delta_q.rotate(imu_sample_new.delta_ang);
	_q_down_sampled =  _q_down_sampled * delta_q;
	_q_down_sampled.normalize();

	matrix::Dcm<float> delta_R(delta_q.inversed());
	_imu_down_sampled.delta_vel = delta_R * _imu_down_sampled.delta_vel;
	_imu_down_sampled.delta_vel += imu_sample_new.delta_vel;

	_imu_ticks++;

	if ((_dt_imu_avg * _imu_ticks >= (float)(FILTER_UPDATE_PERRIOD_MS) / 1000 && _start_predict_enabled)
	    || (_dt_imu_avg * _imu_ticks >= 0.02f)) {
		_imu_down_sampled.delta_ang = _q_down_sampled.to_axis_angle();
		_imu_down_sampled.time_us = time_usec;

		_imu_buffer.push(_imu_down_sampled);

		_imu_down_sampled.delta_ang.setZero();
		_imu_down_sampled.delta_vel.setZero();
		_imu_down_sampled.delta_ang_dt = 0.0f;
		_imu_down_sampled.delta_vel_dt = 0.0f;
		_q_down_sampled(0) = 1.0f;
		_q_down_sampled(1) = _q_down_sampled(2) = _q_down_sampled(3) = 0.0f;

		_imu_ticks = 0;

		_imu_updated = true;

	} else {

		_imu_updated = false;
	}


	_imu_sample_delayed = _imu_buffer.get_oldest();
}

void EstimatorBase::setMagData(uint64_t time_usec, float *data)
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

void EstimatorBase::setGpsData(uint64_t time_usec, struct gps_message *gps)
{

	if (!_gps_initialised) {
		initialiseGPS(gps);
		return;
	}

	if (time_usec - _time_last_gps > 70000 && gps_is_good(gps)) {
		gpsSample gps_sample_new = {};
		gps_sample_new.time_us = gps->time_usec - _params.gps_delay_ms * 1000;


		gps_sample_new.time_us -= FILTER_UPDATE_PERRIOD_MS * 1000 / 2;
		_time_last_gps = time_usec;

		_last_valid_gps_time_us = hrt_absolute_time();

		gps_sample_new.time_us = math::max(gps_sample_new.time_us, _imu_sample_delayed.time_us);


		memcpy(gps_sample_new.vel._data[0], gps->vel_ned, sizeof(gps_sample_new.vel._data));

		_gps_speed_valid = gps->vel_ned_valid;


		float lpos_x = 0.0f;
		float lpos_y = 0.0f;
		map_projection_project(&_posRef, (gps->lat / 1.0e7), (gps->lon / 1.0e7), &lpos_x, &lpos_y);
		gps_sample_new.pos(0) = lpos_x;
		gps_sample_new.pos(1) = lpos_y;
		gps_sample_new.hgt = gps->alt / 1e3f;

		_gps_buffer.push(gps_sample_new);
	}
}

void EstimatorBase::setBaroData(uint64_t time_usec, float *data)
{
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

void EstimatorBase::setAirspeedData(uint64_t time_usec, float *data)
{
	if (time_usec > _time_last_airspeed) {
		airspeedSample airspeed_sample_new;
		airspeed_sample_new.airspeed = *data;
		airspeed_sample_new.time_us -= _params.airspeed_delay_ms * 1000;

		airspeed_sample_new.time_us = time_usec -= FILTER_UPDATE_PERRIOD_MS * 1000 / 2;
		_time_last_airspeed = time_usec;

		_airspeed_buffer.push(airspeed_sample_new);
	}
}

// set range data
void EstimatorBase::setRangeData(uint64_t time_usec, float *data)
{

}

// set optical flow data
void EstimatorBase::setOpticalFlowData(uint64_t time_usec, float *data)
{

}

void EstimatorBase::initialiseVariables(uint64_t time_usec)
{
	_imu_buffer.allocate(IMU_BUFFER_LENGTH);
	_gps_buffer.allocate(OBS_BUFFER_LENGTH);
	_mag_buffer.allocate(OBS_BUFFER_LENGTH);
	_baro_buffer.allocate(OBS_BUFFER_LENGTH);
	_range_buffer.allocate(OBS_BUFFER_LENGTH);
	_airspeed_buffer.allocate(OBS_BUFFER_LENGTH);
	_flow_buffer.allocate(OBS_BUFFER_LENGTH);
	_output_buffer.allocate(IMU_BUFFER_LENGTH);

	_state.ang_error.setZero();
	_state.vel.setZero();
	_state.pos.setZero();
	_state.gyro_bias.setZero();
	_state.gyro_scale(0) = 1.0f;
	_state.gyro_scale(1) = 1.0f;
	_state.gyro_scale(2) = 1.0f;
	_state.accel_z_bias = 0.0f;
	_state.mag_I.setZero();
	_state.mag_B.setZero();
	_state.wind_vel.setZero();
	_state.quat_nominal.setZero();
	_state.quat_nominal(0) = 1.0f;

	_dt_imu_avg = 0.0f;
	_imu_time_last = time_usec;

	_imu_sample_delayed.delta_ang.setZero();
	_imu_sample_delayed.delta_vel.setZero();
	_imu_sample_delayed.delta_ang_dt = 0.0f;
	_imu_sample_delayed.delta_vel_dt = 0.0f;
	_imu_sample_delayed.time_us = time_usec;

	_output_new.vel.setZero();
	_output_new.pos.setZero();
	_output_new.quat_nominal = matrix::Quaternion<float>();

	_imu_down_sampled.delta_ang.setZero();
	_imu_down_sampled.delta_vel.setZero();
	_imu_down_sampled.delta_ang_dt = 0.0f;
	_imu_down_sampled.delta_vel_dt = 0.0f;
	_imu_down_sampled.time_us = time_usec;

	_q_down_sampled(0) = 1.0f;
	_q_down_sampled(1) = 0.0f;
	_q_down_sampled(2) = 0.0f;
	_q_down_sampled(3) = 0.0f;

	_imu_ticks = 0;

	_imu_updated = false;
	_start_predict_enabled = false;
	_initialised = false;
	_gps_initialised = false;
	_gps_speed_valid = false;

	_mag_healthy = false;
	_in_air = false;			// XXX get this flag from the application

	_time_last_imu = 0;
	_time_last_gps = 0;
	_time_last_mag = 0;
	_time_last_baro = 0;
	_time_last_range = 0;
	_time_last_airspeed = 0;

	memset(&_fault_status, 0, sizeof(_fault_status));

}

void EstimatorBase::initialiseGPS(struct gps_message *gps)
{
	//Check if the GPS fix is good enough for us to use
	if (gps_is_good(gps)) {
		printf("gps is good\n");
		// Initialise projection
		double lat = gps->lat / 1.0e7;
		double lon = gps->lon / 1.0e7;
		map_projection_init(&_posRef, lat, lon);
		_gps_alt_ref = gps->alt / 1e3f;
		_gps_initialised = true;
		_last_gps_origin_time_us = hrt_absolute_time();
	}
}

bool EstimatorBase::gps_is_good(struct gps_message *gps)
{
	// go through apm implementation of calcGpsGoodToAlign for fancier checks
	// Use a stricter check for initialisation than during flight to avoid complete loss of GPS
	if (_gps_initialised) {
		if ((gps->fix_type >= 3) && (gps->eph < _params.requiredEph * 2) && (gps->epv < _params.requiredEpv * 2)) {
			return true;

		} else {
			return false;
		}
	} else {
		if ((gps->fix_type >= 3) && (gps->eph < _params.requiredEph) && (gps->epv < _params.requiredEpv)) {
			return true;

		} else {
			return false;
		}
	}
}

bool EstimatorBase::position_is_valid()
{
	// return true if the position estimate is valid
	// TOTO implement proper check based on published GPS accuracy, innovaton consistency checks and timeout status
	return _gps_initialised &&  (hrt_absolute_time() - _last_valid_gps_time_us) < 5e6;
}

void EstimatorBase::printStoredIMU()
{
	printf("---------Printing IMU data buffer------------\n");

	for (int i = 0; i < IMU_BUFFER_LENGTH; i++) {
		printIMU(&_imu_buffer[i]);
	}
}

void EstimatorBase::printIMU(struct imuSample *data)
{
	printf("time %llu\n", data->time_us);
	printf("delta_ang_dt %.5f\n", (double)data->delta_ang_dt);
	printf("delta_vel_dt %.5f\n", (double)data->delta_vel_dt);
	printf("dA: %.5f %.5f %.5f \n", (double)data->delta_ang(0), (double)data->delta_ang(1), (double)data->delta_ang(2));
	printf("dV: %.5f %.5f %.5f \n\n", (double)data->delta_vel(0), (double)data->delta_vel(1), (double)data->delta_vel(2));
}

void EstimatorBase::printQuaternion(Quaternion &q)
{
	printf("q1 %.5f q2 %.5f q3 %.5f q4 %.5f\n", (double)q(0), (double)q(1), (double)q(2), (double)q(3));
}

void EstimatorBase::print_imu_avg_time()
{
	printf("dt_avg: %.5f\n", (double)_dt_imu_avg);
}

void EstimatorBase::printStoredMag()
{
	printf("---------Printing mag data buffer------------\n");

	for (int i = 0; i < OBS_BUFFER_LENGTH; i++) {
		printMag(&_mag_buffer[i]);
	}
}

void EstimatorBase::printMag(struct magSample *data)
{
	printf("time %llu\n", data->time_us);
	printf("mag: %.5f %.5f %.5f \n\n", (double)data->mag(0), (double)data->mag(1), (double)data->mag(2));

}

void EstimatorBase::printBaro(struct baroSample *data)
{
	printf("time %llu\n", data->time_us);
	printf("baro: %.5f\n\n", (double)data->hgt);
}

void EstimatorBase::printStoredBaro()
{
	printf("---------Printing baro data buffer------------\n");

	for (int i = 0; i < OBS_BUFFER_LENGTH; i++) {
		printBaro(&_baro_buffer[i]);
	}
}

void EstimatorBase::printGps(struct gpsSample *data)
{
	printf("time %llu\n", data->time_us);
	printf("gps pos: %.5f %.5f %.5f\n", (double)data->pos(0), (double)data->pos(1), (double)data->hgt);
	printf("gps vel %.5f %.5f %.5f\n\n", (double)data->vel(0), (double)data->vel(1), (double)data->vel(2));
}

void EstimatorBase::printStoredGps()
{
	printf("---------Printing GPS data buffer------------\n");

	for (int i = 0; i < OBS_BUFFER_LENGTH; i++) {
		printGps(&_gps_buffer[i]);
	}
}
