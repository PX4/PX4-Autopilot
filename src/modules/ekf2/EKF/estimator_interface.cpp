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

#include "estimator_interface.h"

#include <mathlib/mathlib.h>

EstimatorInterface::~EstimatorInterface()
{
	delete _gps_buffer;
	delete _gnss_heading_buffer;
	delete _mag_buffer;
	delete _baro_buffer;
	delete _range_buffer;
	delete _airspeed_buffer;
	delete _flow_buffer;
	delete _ext_vision_buffer;
	delete _drag_buffer;
	delete _auxvel_buffer;
}

// Accumulate imu data and store to buffer at desired rate
void EstimatorInterface::setIMUData(const imuSample &imu_sample)
{
	// TODO: resolve misplaced responsibility
	if (!_initialised) {
		_initialised = init(imu_sample.time_us);
	}

	const float dt = math::constrain((imu_sample.time_us - _time_last_imu) / 1e6f, 1.0e-4f, 0.02f);

	_time_last_imu = imu_sample.time_us;

	if (_time_last_imu > 0) {
		_dt_imu_avg = 0.8f * _dt_imu_avg + 0.2f * dt;
	}

	_newest_high_rate_imu_sample = imu_sample;

	_imu_updated = _imu_down_sampler.update(imu_sample);

	// accumulate and down-sample imu data and push to the buffer when new downsampled data becomes available
	if (_imu_updated) {

		_imu_buffer.push(_imu_down_sampler.getDownSampledImuAndTriggerReset());

		// get the oldest data from the buffer
		_imu_sample_delayed = _imu_buffer.get_oldest();

		// calculate the minimum interval between observations required to guarantee no loss of data
		// this will occur if data is overwritten before its time stamp falls behind the fusion time horizon
		_min_obs_interval_us = (imu_sample.time_us - _imu_sample_delayed.time_us) / (_obs_buffer_length - 1);

		setDragData(imu_sample);
	}
}

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

	// limit data rate to prevent data being lost
	if ((mag_sample.time_us - _time_last_mag) > _min_obs_interval_us) {
		_time_last_mag = mag_sample.time_us;

		magSample mag_sample_new;

		mag_sample_new.time_us = mag_sample.time_us;
		mag_sample_new.time_us -= static_cast<uint64_t>(_params.mag_delay_ms * 1000);
		mag_sample_new.time_us -= static_cast<uint64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

		mag_sample_new.mag = mag_sample.mag;

		_mag_buffer->push(mag_sample_new);
	} else {
		ECL_ERR("mag data too fast %" PRIu64, mag_sample.time_us - _time_last_mag);
	}
}

void EstimatorInterface::setGpsData(const gps_message &gps)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_gps_buffer == nullptr) {
		_gps_buffer = new RingBuffer<gpsSample>(_obs_buffer_length);

		if (_gps_buffer == nullptr || !_gps_buffer->valid()) {
			delete _gps_buffer;
			_gps_buffer = nullptr;
			printBufferAllocationFailed("GPS");
			return;
		}
	}

	if ((gps.time_usec - _time_last_gps) > _min_obs_interval_us) {

		if (!gps.vel_ned_valid || (gps.fix_type == 0)) {
			return;
		}

		_time_last_gps = gps.time_usec;

		gpsSample gps_sample_new;

		gps_sample_new.time_us = gps.time_usec - static_cast<uint64_t>(_params.gps_delay_ms * 1000);
		gps_sample_new.time_us -= static_cast<uint64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

		gps_sample_new.vel = gps.vel_ned;

		_gps_speed_valid = gps.vel_ned_valid;
		gps_sample_new.sacc = gps.sacc;
		gps_sample_new.hacc = gps.eph;
		gps_sample_new.vacc = gps.epv;

		gps_sample_new.hgt = (float)gps.alt * 1e-3f;

		gps_sample_new.yaw = gps.yaw;

		if (PX4_ISFINITE(gps.yaw_offset)) {
			_gps_yaw_offset = gps.yaw_offset;

		} else {
			_gps_yaw_offset = 0.0f;
		}

		// Only calculate the relative position if the WGS-84 location of the origin is set
		if (collect_gps(gps)) {
			gps_sample_new.pos = _pos_ref.project((gps.lat / 1.0e7), (gps.lon / 1.0e7));

		} else {
			gps_sample_new.pos(0) = 0.0f;
			gps_sample_new.pos(1) = 0.0f;
		}

		_gps_buffer->push(gps_sample_new);
	} else {
		ECL_ERR("GPS data too fast %" PRIu64, gps.time_usec - _time_last_gps);
	}
}

void EstimatorInterface::setGnssHeadingData(const gnss_heading_message &gnss_heading)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_gnss_heading_buffer == nullptr) {
		_gnss_heading_buffer = new RingBuffer<gnssHeadingSample>(_obs_buffer_length);

		if (_gnss_heading_buffer == nullptr || !_gnss_heading_buffer->valid()) {
			delete _gnss_heading_buffer;
			_gnss_heading_buffer = nullptr;
			printBufferAllocationFailed("GNSS heading");
			return;
		}
	}

	if ((gnss_heading.time_usec - _time_last_gnss_heading) > _min_obs_interval_us) {
		_time_last_gnss_heading = gnss_heading.time_usec;

		gnssHeadingSample gnss_heading_sample_new;

		gnss_heading_sample_new.time_us = gnss_heading.time_usec - static_cast<uint64_t>(_params.gps_delay_ms * 1000);
		gnss_heading_sample_new.time_us -= static_cast<uint64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

		gnss_heading_sample_new.heading = gnss_heading.heading;
		gnss_heading_sample_new.heading_accuracy = gnss_heading.heading_accuracy;

		_gnss_heading_buffer->push(gnss_heading_sample_new);
	} else {
		ECL_ERR("GNSS heading data too fast %" PRIu64, gnss_heading.time_usec - _time_last_gnss_heading);
	}
}

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

	// limit data rate to prevent data being lost
	if ((baro_sample.time_us - _time_last_baro) > _min_obs_interval_us) {
		_time_last_baro = baro_sample.time_us;

		baroSample baro_sample_new;
		baro_sample_new.hgt = compensateBaroForDynamicPressure(baro_sample.hgt);

		baro_sample_new.time_us = baro_sample.time_us;
		baro_sample_new.time_us -= static_cast<uint64_t>(_params.baro_delay_ms * 1000);
		baro_sample_new.time_us -= static_cast<uint64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

		_baro_buffer->push(baro_sample_new);
	} else {
		ECL_ERR("baro data too fast %" PRIu64, baro_sample.time_us - _time_last_baro);
	}
}

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

	// limit data rate to prevent data being lost
	if ((airspeed_sample.time_us - _time_last_airspeed) > _min_obs_interval_us) {
		_time_last_airspeed = airspeed_sample.time_us;

		airspeedSample airspeed_sample_new = airspeed_sample;

		airspeed_sample_new.time_us -= static_cast<uint64_t>(_params.airspeed_delay_ms * 1000);
		airspeed_sample_new.time_us -= static_cast<uint64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

		_airspeed_buffer->push(airspeed_sample_new);
	}
}

void EstimatorInterface::setRangeData(const rangeSample &range_sample)
{
	if (!_initialised) {
		return;
	}

	// Allocate the required buffer size if not previously done
	if (_range_buffer == nullptr) {
		_range_buffer = new RingBuffer<rangeSample>(_obs_buffer_length);

		if (_range_buffer == nullptr || !_range_buffer->valid()) {
			delete _range_buffer;
			_range_buffer = nullptr;
			printBufferAllocationFailed("range");
			return;
		}
	}

	// limit data rate to prevent data being lost
	if ((range_sample.time_us - _time_last_range) > _min_obs_interval_us) {
		_time_last_range = range_sample.time_us;

		rangeSample range_sample_new = range_sample;
		range_sample_new.time_us -= static_cast<uint64_t>(_params.range_delay_ms * 1000);
		range_sample_new.time_us -= static_cast<uint64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

		_range_buffer->push(range_sample_new);
	}
}

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

	// limit data rate to prevent data being lost
	if ((flow.time_us - _time_last_optflow) > _min_obs_interval_us) {
		_time_last_optflow = flow.time_us;

		flowSample optflow_sample_new = flow;

		optflow_sample_new.time_us -= static_cast<uint64_t>(_params.flow_delay_ms * 1000);
		optflow_sample_new.time_us -= static_cast<uint64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

		_flow_buffer->push(optflow_sample_new);
	}
}

// set attitude and position data derived from an external vision system
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

	// limit data rate to prevent data being lost
	if ((evdata.time_us - _time_last_ext_vision) > _min_obs_interval_us) {
		_time_last_ext_vision = evdata.time_us;

		extVisionSample ev_sample_new = evdata;
		// calculate the system time-stamp for the mid point of the integration period
		ev_sample_new.time_us -= static_cast<uint64_t>(_params.ev_delay_ms * 1000);
		ev_sample_new.time_us -= static_cast<uint64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

		_ext_vision_buffer->push(ev_sample_new);
	}
}

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

	// limit data rate to prevent data being lost
	if ((auxvel_sample.time_us - _time_last_auxvel) > _min_obs_interval_us) {
		_time_last_auxvel = auxvel_sample.time_us;

		auxVelSample auxvel_sample_new = auxvel_sample;

		auxvel_sample_new.time_us -= static_cast<uint64_t>(_params.auxvel_delay_ms * 1000);
		auxvel_sample_new.time_us -= static_cast<uint64_t>(_dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

		_auxvel_buffer->push(auxvel_sample_new);
	}
}

void EstimatorInterface::setDragData(const imuSample &imu)
{
	// down-sample the drag specific force data by accumulating and calculating the mean when
	// sufficient samples have been collected
	if ((_params.fusion_mode & MASK_USE_DRAG)) {

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

		_drag_sample_count ++;
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

bool EstimatorInterface::initialise_interface(uint64_t timestamp)
{
	// find the maximum time delay the buffers are required to handle

	// it's reasonable to assume that aux velocity device has low delay. TODO: check the delay only if the aux device is used
	float max_time_delay_ms = math::max((float)_params.sensor_interval_max_ms, _params.auxvel_delay_ms);

	// using baro
	if (_params.vdist_sensor_type == 0) {
		max_time_delay_ms = math::max(_params.baro_delay_ms, max_time_delay_ms);
	}

	// using airspeed
	if (_params.arsp_thr > FLT_EPSILON) {
		max_time_delay_ms = math::max(_params.airspeed_delay_ms, max_time_delay_ms);
	}

	// mag mode
	if (_params.mag_fusion_type != MAG_FUSE_TYPE_NONE) {
		max_time_delay_ms = math::max(_params.mag_delay_ms, max_time_delay_ms);
	}

	// range aid or range height
	if (_params.range_aid || (_params.vdist_sensor_type == VDIST_SENSOR_RANGE)) {
		max_time_delay_ms = math::max(_params.range_delay_ms, max_time_delay_ms);
	}

	if (_params.fusion_mode & MASK_USE_GPS) {
		max_time_delay_ms = math::max(_params.gps_delay_ms, max_time_delay_ms);
	}

	if (_params.fusion_mode & MASK_USE_OF) {
		max_time_delay_ms = math::max(_params.flow_delay_ms, max_time_delay_ms);
	}

	if (_params.fusion_mode & (MASK_USE_EVPOS | MASK_USE_EVYAW | MASK_USE_EVVEL)) {
		max_time_delay_ms = math::max(_params.ev_delay_ms, max_time_delay_ms);
	}

	const float filter_update_period_ms = _params.filter_update_interval_us / 1000.f;

	// calculate the IMU buffer length required to accomodate the maximum delay with some allowance for jitter
	_imu_buffer_length = ceilf(max_time_delay_ms / filter_update_period_ms);

	// set the observation buffer length to handle the minimum time of arrival between observations in combination
	// with the worst case delay from current time to ekf fusion time
	// allow for worst case 50% extension of the ekf fusion time horizon delay due to timing jitter
	const float ekf_delay_ms = max_time_delay_ms * 1.5f;
	_obs_buffer_length = roundf(ekf_delay_ms / filter_update_period_ms);

	// limit to be no longer than the IMU buffer (we can't process data faster than the EKF prediction rate)
	_obs_buffer_length = math::min(_obs_buffer_length, _imu_buffer_length);

	ECL_DEBUG("EKF max time delay %.1f ms, OBS length %d\n", (double)ekf_delay_ms, _obs_buffer_length);

	if (!_imu_buffer.allocate(_imu_buffer_length) || !_output_buffer.allocate(_imu_buffer_length)
	    || !_output_vert_buffer.allocate(_imu_buffer_length)) {

		printBufferAllocationFailed("IMU and output");
		return false;
	}

	_imu_sample_delayed.time_us = timestamp;
	_imu_sample_delayed.delta_vel_clipping[0] = false;
	_imu_sample_delayed.delta_vel_clipping[1] = false;
	_imu_sample_delayed.delta_vel_clipping[2] = false;

	_fault_status.value = 0;

	return true;
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
	return int(_control_status.flags.gps)
	       + int(_control_status.flags.opt_flow)
	       + int(_control_status.flags.ev_pos)
	       + int(_control_status.flags.ev_vel)
	       // Combined airspeed and sideslip fusion allows sustained wind relative dead reckoning
	       // and so is treated as a single aiding source.
	       + int(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta);
}

bool EstimatorInterface::isHorizontalAidingActive() const
{
	return getNumberOfActiveHorizontalAidingSources() > 0;
}

bool EstimatorInterface::isVerticalVelocityAidingActive() const
{
	return getNumberOfActiveVerticalVelocityAidingSources() > 0;
}

int EstimatorInterface::getNumberOfActiveVerticalVelocityAidingSources() const
{
	return int(_control_status.flags.gps)
	       + int(_control_status.flags.ev_vel);
}

void EstimatorInterface::printBufferAllocationFailed(const char *buffer_name)
{
	if (buffer_name) {
		ECL_ERR("%s buffer allocation failed", buffer_name);
	}
}

void EstimatorInterface::print_status()
{
	printf("IMU average dt: %.6f seconds\n", (double)_dt_imu_avg);
	printf("EKF average dt: %.6f seconds\n", (double)_dt_ekf_avg);

	printf("IMU buffer: %d (%d Bytes)\n", _imu_buffer.get_length(), _imu_buffer.get_total_size());

	printf("minimum observation interval %d us\n", _min_obs_interval_us);

	if (_gps_buffer) {
		printf("gps buffer: %d/%d (%d Bytes)\n", _gps_buffer->entries(), _gps_buffer->get_length(), _gps_buffer->get_total_size());
	}

	if (_gnss_heading_buffer) {
		printf("gnss heading buffer: %d/%d (%d Bytes)\n", _gnss_heading_buffer->entries(), _gnss_heading_buffer->get_length(), _gnss_heading_buffer->get_total_size());
	}

	if (_mag_buffer) {
		printf("mag buffer: %d/%d (%d Bytes)\n", _mag_buffer->entries(), _mag_buffer->get_length(), _mag_buffer->get_total_size());
	}

	if (_baro_buffer) {
		printf("baro buffer: %d/%d (%d Bytes)\n", _baro_buffer->entries(), _baro_buffer->get_length(), _baro_buffer->get_total_size());
	}

	if (_range_buffer) {
		printf("range buffer: %d/%d (%d Bytes)\n", _range_buffer->entries(), _range_buffer->get_length(), _range_buffer->get_total_size());
	}

	if (_airspeed_buffer) {
		printf("airspeed buffer: %d/%d (%d Bytes)\n", _airspeed_buffer->entries(), _airspeed_buffer->get_length(), _airspeed_buffer->get_total_size());
	}

	if (_flow_buffer) {
		printf("flow buffer: %d/%d (%d Bytes)\n", _flow_buffer->entries(), _flow_buffer->get_length(), _flow_buffer->get_total_size());
	}

	if (_ext_vision_buffer) {
		printf("vision buffer: %d/%d (%d Bytes)\n", _ext_vision_buffer->entries(), _ext_vision_buffer->get_length(), _ext_vision_buffer->get_total_size());
	}

	if (_drag_buffer) {
		printf("drag buffer: %d/%d (%d Bytes)\n", _drag_buffer->entries(), _drag_buffer->get_length(), _drag_buffer->get_total_size());
	}

	printf("output buffer: %d/%d (%d Bytes)\n", _output_buffer.entries(), _output_buffer.get_length(), _output_buffer.get_total_size());
	printf("output vert buffer: %d/%d (%d Bytes)\n", _output_vert_buffer.entries(), _output_vert_buffer.get_length(), _output_vert_buffer.get_total_size());
}
