/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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

/**
 * @file estimator_interface.cpp
 * Definition of base class for attitude estimators
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth B Purohit <siddharthbharatpurohit@gmail.com>
 */

#include "estimator_interface.h"

#include <mathlib/mathlib.h>

EstimatorInterface::~EstimatorInterface()
{
#if defined(CONFIG_EKF2_GNSS)
	delete _gps_buffer;
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_MAGNETOMETER)
	delete _mag_buffer;
#endif // CONFIG_EKF2_MAGNETOMETER
#if defined(CONFIG_EKF2_BAROMETER)
	delete _baro_buffer;
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_RANGE_FINDER)
	delete _range_buffer;
#endif // CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_AIRSPEED)
	delete _airspeed_buffer;
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	delete _flow_buffer;
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	delete _ext_vision_buffer;
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_DRAG_FUSION)
	delete _drag_buffer;
#endif // CONFIG_EKF2_DRAG_FUSION
#if defined(CONFIG_EKF2_AUXVEL)
	delete _auxvel_buffer;
#endif // CONFIG_EKF2_AUXVEL
}

// Accumulate imu data and store to buffer at desired rate
void EstimatorInterface::setIMUData(const imuSample &imu_sample)
{
	// TODO: resolve misplaced responsibility
	if (!_initialised) {
		_initialised = init(imu_sample.time_us);
	}

	_time_latest_us = imu_sample.time_us;

	// the output observer always runs
	_output_predictor.calculateOutputStates(imu_sample.time_us, imu_sample.delta_ang, imu_sample.delta_ang_dt,
						imu_sample.delta_vel, imu_sample.delta_vel_dt);

	// accumulate and down-sample imu data and push to the buffer when new downsampled data becomes available
	if (_imu_down_sampler.update(imu_sample)) {

		_imu_updated = true;

		imuSample imu_downsampled = _imu_down_sampler.getDownSampledImuAndTriggerReset();

		// as a precaution constrain the integration delta time to prevent numerical problems
		const float filter_update_period_s = _params.filter_update_interval_us * 1e-6f;
		const float imu_min_dt = 0.5f * filter_update_period_s;
		const float imu_max_dt = 2.0f * filter_update_period_s;

		imu_downsampled.delta_ang_dt = math::constrain(imu_downsampled.delta_ang_dt, imu_min_dt, imu_max_dt);
		imu_downsampled.delta_vel_dt = math::constrain(imu_downsampled.delta_vel_dt, imu_min_dt, imu_max_dt);

		_imu_buffer.push(imu_downsampled);

		// get the oldest data from the buffer
		_time_delayed_us = _imu_buffer.get_oldest().time_us;

		// calculate the minimum interval between observations required to guarantee no loss of data
		// this will occur if data is overwritten before its time stamp falls behind the fusion time horizon
		_min_obs_interval_us = (imu_sample.time_us - _time_delayed_us) / (_obs_buffer_length - 1);
	}

#if defined(CONFIG_EKF2_DRAG_FUSION)
	setDragData(imu_sample);
#endif // CONFIG_EKF2_DRAG_FUSION
}

#if defined(CONFIG_EKF2_MAGNETOMETER)
void EstimatorInterface::setMagData(const magSample &mag_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_mag_buffer == nullptr) {
		_mag_buffer = new RingBuffer<magSample>(_obs_buffer_length);

		if (_mag_buffer == nullptr || !_mag_buffer->valid()) {
			delete _mag_buffer;
			_mag_buffer = nullptr;
			printBufferAllocationFailed("mag");
			return;
		}
	}

	const int64_t time_us = mag_sample.time_us
				- static_cast<int64_t>(_params.mag_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_mag_buffer->get_newest().time_us + _min_obs_interval_us)) {

		magSample mag_sample_new{mag_sample};
		mag_sample_new.time_us = time_us;

		_mag_buffer->push(mag_sample_new);
		_time_last_mag_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("mag data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _mag_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS)
void EstimatorInterface::setGpsData(const gnssSample &gnss_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_gps_buffer == nullptr) {
		_gps_buffer = new RingBuffer<gnssSample>(_obs_buffer_length);

		if (_gps_buffer == nullptr || !_gps_buffer->valid()) {
			delete _gps_buffer;
			_gps_buffer = nullptr;
			printBufferAllocationFailed("GPS");
			return;
		}
	}

	const int64_t time_us = gnss_sample.time_us
				- static_cast<int64_t>(_params.gps_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	if (time_us >= static_cast<int64_t>(_gps_buffer->get_newest().time_us + _min_obs_interval_us)) {

		gnssSample gnss_sample_new(gnss_sample);

		gnss_sample_new.time_us = time_us;

		_gps_buffer->push(gnss_sample_new);
		_time_last_gps_buffer_push = _time_latest_us;

#if defined(CONFIG_EKF2_GNSS_YAW)

		if (PX4_ISFINITE(gnss_sample.yaw)) {
			_time_last_gnss_yaw_buffer_push = _time_latest_us;
		}

#endif // CONFIG_EKF2_GNSS_YAW

	} else {
		ECL_WARN("GPS data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _gps_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_BAROMETER)
void EstimatorInterface::setBaroData(const baroSample &baro_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_baro_buffer == nullptr) {
		_baro_buffer = new RingBuffer<baroSample>(_obs_buffer_length);

		if (_baro_buffer == nullptr || !_baro_buffer->valid()) {
			delete _baro_buffer;
			_baro_buffer = nullptr;
			printBufferAllocationFailed("baro");
			return;
		}
	}

	const int64_t time_us = baro_sample.time_us
				- static_cast<int64_t>(_params.baro_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_baro_buffer->get_newest().time_us + _min_obs_interval_us)) {

		baroSample baro_sample_new{baro_sample};
		baro_sample_new.time_us = time_us;

		_baro_buffer->push(baro_sample_new);
		_time_last_baro_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("baro data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _baro_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AIRSPEED)
void EstimatorInterface::setAirspeedData(const airspeedSample &airspeed_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_airspeed_buffer == nullptr) {
		_airspeed_buffer = new RingBuffer<airspeedSample>(_obs_buffer_length);

		if (_airspeed_buffer == nullptr || !_airspeed_buffer->valid()) {
			delete _airspeed_buffer;
			_airspeed_buffer = nullptr;
			printBufferAllocationFailed("airspeed");
			return;
		}
	}

	const int64_t time_us = airspeed_sample.time_us
				- static_cast<int64_t>(_params.airspeed_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_airspeed_buffer->get_newest().time_us + _min_obs_interval_us)) {

		airspeedSample airspeed_sample_new{airspeed_sample};
		airspeed_sample_new.time_us = time_us;

		_airspeed_buffer->push(airspeed_sample_new);

	} else {
		ECL_WARN("airspeed data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _airspeed_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_RANGE_FINDER)
void EstimatorInterface::setRangeData(const sensor::rangeSample &range_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_range_buffer == nullptr) {
		_range_buffer = new RingBuffer<sensor::rangeSample>(_obs_buffer_length);

		if (_range_buffer == nullptr || !_range_buffer->valid()) {
			delete _range_buffer;
			_range_buffer = nullptr;
			printBufferAllocationFailed("range");
			return;
		}
	}

	const int64_t time_us = range_sample.time_us
				- static_cast<int64_t>(_params.range_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_range_buffer->get_newest().time_us + _min_obs_interval_us)) {

		sensor::rangeSample range_sample_new{range_sample};
		range_sample_new.time_us = time_us;

		_range_buffer->push(range_sample_new);
		_time_last_range_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("range data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _range_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
void EstimatorInterface::setOpticalFlowData(const flowSample &flow)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_flow_buffer == nullptr) {
		_flow_buffer = new RingBuffer<flowSample>(_imu_buffer_length);

		if (_flow_buffer == nullptr || !_flow_buffer->valid()) {
			delete _flow_buffer;
			_flow_buffer = nullptr;
			printBufferAllocationFailed("flow");
			return;
		}
	}

	const int64_t time_us = flow.time_us
				- static_cast<int64_t>(_params.flow_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_flow_buffer->get_newest().time_us + _min_obs_interval_us)) {

		flowSample optflow_sample_new{flow};
		optflow_sample_new.time_us = time_us;

		_flow_buffer->push(optflow_sample_new);

	} else {
		ECL_WARN("optical flow data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _flow_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
void EstimatorInterface::setExtVisionData(const extVisionSample &evdata)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_ext_vision_buffer == nullptr) {
		_ext_vision_buffer = new RingBuffer<extVisionSample>(_obs_buffer_length);

		if (_ext_vision_buffer == nullptr || !_ext_vision_buffer->valid()) {
			delete _ext_vision_buffer;
			_ext_vision_buffer = nullptr;
			printBufferAllocationFailed("vision");
			return;
		}
	}

	// calculate the system time-stamp for the mid point of the integration period
	const int64_t time_us = evdata.time_us
				- static_cast<int64_t>(_params.ev_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_ext_vision_buffer->get_newest().time_us + _min_obs_interval_us)) {

		extVisionSample ev_sample_new{evdata};
		ev_sample_new.time_us = time_us;

		_ext_vision_buffer->push(ev_sample_new);
		_time_last_ext_vision_buffer_push = _time_latest_us;

	} else {
		ECL_WARN("EV data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _ext_vision_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
void EstimatorInterface::setAuxVelData(const auxVelSample &auxvel_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_auxvel_buffer == nullptr) {
		_auxvel_buffer = new RingBuffer<auxVelSample>(_obs_buffer_length);

		if (_auxvel_buffer == nullptr || !_auxvel_buffer->valid()) {
			delete _auxvel_buffer;
			_auxvel_buffer = nullptr;
			printBufferAllocationFailed("aux vel");
			return;
		}
	}

	const int64_t time_us = auxvel_sample.time_us
				- static_cast<int64_t>(_params.auxvel_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_auxvel_buffer->get_newest().time_us + _min_obs_interval_us)) {

		auxVelSample auxvel_sample_new{auxvel_sample};
		auxvel_sample_new.time_us = time_us;

		_auxvel_buffer->push(auxvel_sample_new);

	} else {
		ECL_WARN("aux velocity data too fast %" PRIi64 " < %" PRIu64 " + %d", time_us, _auxvel_buffer->get_newest().time_us,
			 _min_obs_interval_us);
	}
}
#endif // CONFIG_EKF2_AUXVEL

void EstimatorInterface::setSystemFlagData(const systemFlagUpdate &system_flags)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_system_flag_buffer == nullptr) {
		_system_flag_buffer = new RingBuffer<systemFlagUpdate>(_obs_buffer_length);

		if (_system_flag_buffer == nullptr || !_system_flag_buffer->valid()) {
			delete _system_flag_buffer;
			_system_flag_buffer = nullptr;
			printBufferAllocationFailed("system flag");
			return;
		}
	}

	const int64_t time_us = system_flags.time_us
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_system_flag_buffer->get_newest().time_us + _min_obs_interval_us)) {

		systemFlagUpdate system_flags_new{system_flags};
		system_flags_new.time_us = time_us;

		_system_flag_buffer->push(system_flags_new);

	} else {
		ECL_DEBUG("system flag update too fast %" PRIi64 " < %" PRIu64 " + %d", time_us,
			  _system_flag_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}

#if defined(CONFIG_EKF2_DRAG_FUSION)
void EstimatorInterface::setDragData(const imuSample &imu)
{
	// down-sample the drag specific force data by accumulating and calculating the mean when
	// sufficient samples have been collected
	if (_params.drag_ctrl > 0) {

		// Allocate the required buffer size if not previously done
		if (_drag_buffer == nullptr) {
			_drag_buffer = new RingBuffer<dragSample>(_obs_buffer_length);

			if (_drag_buffer == nullptr || !_drag_buffer->valid()) {
				delete _drag_buffer;
				_drag_buffer = nullptr;
				printBufferAllocationFailed("drag");
				return;
			}
		}

		// don't use any accel samples that are clipping
		if (imu.delta_vel_clipping[0] || imu.delta_vel_clipping[1] || imu.delta_vel_clipping[2]) {
			// reset accumulators
			_drag_sample_count = 0;
			_drag_down_sampled.accelXY.zero();
			_drag_down_sampled.time_us = 0;
			_drag_sample_time_dt = 0.0f;

			return;
		}

		_drag_sample_count++;
		// note acceleration is accumulated as a delta velocity
		_drag_down_sampled.accelXY(0) += imu.delta_vel(0);
		_drag_down_sampled.accelXY(1) += imu.delta_vel(1);
		_drag_down_sampled.time_us += imu.time_us;
		_drag_sample_time_dt += imu.delta_vel_dt;

		// calculate the downsample ratio for drag specific force data
		uint8_t min_sample_ratio = (uint8_t) ceilf((float)_imu_buffer_length / _obs_buffer_length);

		if (min_sample_ratio < 5) {
			min_sample_ratio = 5;
		}

		// calculate and store means from accumulated values
		if (_drag_sample_count >= min_sample_ratio) {
			// note conversion from accumulated delta velocity to acceleration
			_drag_down_sampled.accelXY(0) /= _drag_sample_time_dt;
			_drag_down_sampled.accelXY(1) /= _drag_sample_time_dt;
			_drag_down_sampled.time_us /= _drag_sample_count;

			// write to buffer
			_drag_buffer->push(_drag_down_sampled);

			// reset accumulators
			_drag_sample_count = 0;
			_drag_down_sampled.accelXY.zero();
			_drag_down_sampled.time_us = 0;
			_drag_sample_time_dt = 0.0f;
		}
	}
}
#endif // CONFIG_EKF2_DRAG_FUSION

bool EstimatorInterface::initialise_interface(uint64_t timestamp)
{
	const float filter_update_period_ms = _params.filter_update_interval_us / 1000.f;

	// calculate the IMU buffer length required to accomodate the maximum delay with some allowance for jitter
	_imu_buffer_length = math::max(2, (int)ceilf(_params.delay_max_ms / filter_update_period_ms));

	// set the observation buffer length to handle the minimum time of arrival between observations in combination
	// with the worst case delay from current time to ekf fusion time
	// allow for worst case 50% extension of the ekf fusion time horizon delay due to timing jitter
	const float ekf_delay_ms = _params.delay_max_ms * 1.5f;
	_obs_buffer_length = roundf(ekf_delay_ms / filter_update_period_ms);

	// limit to be no longer than the IMU buffer (we can't process data faster than the EKF prediction rate)
	_obs_buffer_length = math::min(_obs_buffer_length, _imu_buffer_length);

	ECL_DEBUG("EKF max time delay %.1f ms, OBS length %d\n", (double)ekf_delay_ms, _obs_buffer_length);

	if (!_imu_buffer.allocate(_imu_buffer_length) || !_output_predictor.allocate(_imu_buffer_length)) {

		printBufferAllocationFailed("IMU and output");
		return false;
	}

	_time_delayed_us = timestamp;
	_time_latest_us = timestamp;

	_fault_status.value = 0;

	return true;
}

Vector3f EstimatorInterface::getPosition() const
{
	LatLonAlt lla = _output_predictor.getLatLonAlt();
	float x;
	float y;

	if (_local_origin_lat_lon.isInitialized()) {
		_local_origin_lat_lon.project(lla.latitude_deg(), lla.longitude_deg(), x, y);

	} else {
		MapProjection zero_ref;
		zero_ref.initReference(0.0, 0.0);
		zero_ref.project(lla.latitude_deg(), lla.longitude_deg(), x, y);
	}

	const float z = -(lla.altitude() - getEkfGlobalOriginAltitude());

	return Vector3f(x, y, z);
}

bool EstimatorInterface::isOnlyActiveSourceOfHorizontalAiding(const bool aiding_flag) const
{
	return aiding_flag && !isOtherSourceOfHorizontalAidingThan(aiding_flag);
}

bool EstimatorInterface::isOtherSourceOfHorizontalAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveHorizontalAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

int EstimatorInterface::getNumberOfActiveHorizontalAidingSources() const
{
	return getNumberOfActiveHorizontalPositionAidingSources() + getNumberOfActiveHorizontalVelocityAidingSources();
}

bool EstimatorInterface::isOnlyActiveSourceOfHorizontalPositionAiding(const bool aiding_flag) const
{
	return aiding_flag && !isOtherSourceOfHorizontalPositionAidingThan(aiding_flag);
}

bool EstimatorInterface::isOtherSourceOfHorizontalPositionAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveHorizontalPositionAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

int EstimatorInterface::getNumberOfActiveHorizontalPositionAidingSources() const
{
	return int(_control_status.flags.gnss_pos)
	       + int(_control_status.flags.ev_pos)
	       + int(_control_status.flags.aux_gpos);
}

int EstimatorInterface::getNumberOfActiveHorizontalVelocityAidingSources() const
{
	return int(_control_status.flags.gnss_vel)
	       + int(_control_status.flags.opt_flow)
	       + int(_control_status.flags.ev_vel)
	       // Combined airspeed and sideslip fusion allows sustained wind relative dead reckoning
	       // and so is treated as a single aiding source.
	       + int(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta);
}

bool EstimatorInterface::isHorizontalAidingActive() const
{
	return getNumberOfActiveHorizontalAidingSources() > 0;
}

bool EstimatorInterface::isOtherSourceOfVerticalPositionAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveVerticalPositionAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

bool EstimatorInterface::isVerticalPositionAidingActive() const
{
	return getNumberOfActiveVerticalPositionAidingSources() > 0;
}

bool EstimatorInterface::isOnlyActiveSourceOfVerticalPositionAiding(const bool aiding_flag) const
{
	return aiding_flag && !isOtherSourceOfVerticalPositionAidingThan(aiding_flag);
}

int EstimatorInterface::getNumberOfActiveVerticalPositionAidingSources() const
{
	return int(_control_status.flags.gps_hgt)
	       + int(_control_status.flags.baro_hgt)
	       + int(_control_status.flags.rng_hgt)
	       + int(_control_status.flags.ev_hgt);
}

bool EstimatorInterface::isVerticalAidingActive() const
{
	return isVerticalPositionAidingActive() || isVerticalVelocityAidingActive();
}

bool EstimatorInterface::isVerticalVelocityAidingActive() const
{
	return getNumberOfActiveVerticalVelocityAidingSources() > 0;
}

int EstimatorInterface::getNumberOfActiveVerticalVelocityAidingSources() const
{
	return int(_control_status.flags.gnss_vel)
	       + int(_control_status.flags.ev_vel);
}

bool EstimatorInterface::isNorthEastAidingActive() const
{
	return _control_status.flags.gnss_pos
	       || _control_status.flags.gnss_vel
	       || _control_status.flags.aux_gpos;
}

void EstimatorInterface::printBufferAllocationFailed(const char *buffer_name)
{
	if (buffer_name) {
		ECL_ERR("%s buffer allocation failed", buffer_name);
	}
}
