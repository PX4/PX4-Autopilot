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

#include "ekf.h"

#include <mathlib/mathlib.h>

Ekf::~Ekf()
{
	delete _gps_buffer;
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
void Ekf::setIMUData(const imuSample &imu_sample)
{
	// TODO: resolve misplaced responsibility
	if (!_initialised) {
		_initialised = init(imu_sample.time_us);
	}

	_time_latest_us = imu_sample.time_us;

	// the output observer always runs
	_output_predictor.calculateOutputStates(imu_sample.time_us,
						imu_sample.delta_ang, imu_sample.delta_ang_dt,
						imu_sample.delta_vel, imu_sample.delta_vel_dt);

	// accumulate and down-sample imu data and push to the buffer when new downsampled data becomes available
	if (_imu_down_sampler.update(imu_sample)) {

		_imu_updated = true;

		_imu_buffer.push(_imu_down_sampler.getDownSampledImuAndTriggerReset());

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

void Ekf::setSystemFlagData(const systemFlagUpdate &system_flags)
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
		ECL_DEBUG("system flag update too fast %" PRIi64 " < %" PRIu64 " + %d",
			  time_us, _system_flag_buffer->get_newest().time_us, _min_obs_interval_us);
	}
}

bool Ekf::initialise_interface(uint64_t timestamp)
{
	// find the maximum time delay the buffers are required to handle

	// it's reasonable to assume that aux velocity device has low delay. TODO: check the delay only if the aux device is used
	float max_time_delay_ms = _params.sensor_interval_max_ms;

	// aux vel
#if defined(CONFIG_EKF2_AUXVEL)
	max_time_delay_ms = math::max(_params.auxvel_delay_ms, max_time_delay_ms);
#endif // CONFIG_EKF2_AUXVEL

	// using baro
	if (_params.baro_ctrl > 0) {
		max_time_delay_ms = math::max(_params.baro_delay_ms, max_time_delay_ms);
	}

#if defined(CONFIG_EKF2_AIRSPEED)

	// using airspeed
	if (_params.arsp_thr > FLT_EPSILON) {
		max_time_delay_ms = math::max(_params.airspeed_delay_ms, max_time_delay_ms);
	}

#endif // CONFIG_EKF2_AIRSPEED

	// mag mode
	if (_params.mag_fusion_type != MagFuseType::NONE) {
		max_time_delay_ms = math::max(_params.mag_delay_ms, max_time_delay_ms);
	}

#if defined(CONFIG_EKF2_RANGE_FINDER)

	// using range finder
	if ((_params.rng_ctrl != RngCtrl::DISABLED)) {
		max_time_delay_ms = math::max(_params.range_delay_ms, max_time_delay_ms);
	}

#endif // CONFIG_EKF2_RANGE_FINDER

	if (_params.gnss_ctrl > 0) {
		max_time_delay_ms = math::max(_params.gps_delay_ms, max_time_delay_ms);
	}

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (_params.flow_ctrl > 0) {
		max_time_delay_ms = math::max(_params.flow_delay_ms, max_time_delay_ms);
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_params.ev_ctrl > 0) {
		max_time_delay_ms = math::max(_params.ev_delay_ms, max_time_delay_ms);
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

	const float filter_update_period_ms = _params.filter_update_interval_us / 1000.f;

	// calculate the IMU buffer length required to accomodate the maximum delay with some allowance for jitter
	_imu_buffer_length = math::max(2, (int)ceilf(max_time_delay_ms / filter_update_period_ms));

	// set the observation buffer length to handle the minimum time of arrival between observations in combination
	// with the worst case delay from current time to ekf fusion time
	// allow for worst case 50% extension of the ekf fusion time horizon delay due to timing jitter
	const float ekf_delay_ms = max_time_delay_ms * 1.5f;
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

bool Ekf::isOtherSourceOfHorizontalAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveHorizontalAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

int Ekf::getNumberOfActiveHorizontalAidingSources() const
{
	return int(_control_status.flags.gps)
	       + int(_control_status.flags.opt_flow)
	       + int(_control_status.flags.ev_pos)
	       + int(_control_status.flags.ev_vel)
	       // Combined airspeed and sideslip fusion allows sustained wind relative dead reckoning
	       // and so is treated as a single aiding source.
	       + int(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta);
}

bool Ekf::isOtherSourceOfVerticalPositionAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveVerticalPositionAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

int Ekf::getNumberOfActiveVerticalPositionAidingSources() const
{
	return int(_control_status.flags.gps_hgt)
	       + int(_control_status.flags.baro_hgt)
	       + int(_control_status.flags.rng_hgt)
	       + int(_control_status.flags.ev_hgt);
}

void Ekf::printBufferAllocationFailed(const char *buffer_name)
{
	if (buffer_name) {
		ECL_ERR("%s buffer allocation failed", buffer_name);
	}
}

void Ekf::print_status()
{
	printf("EKF average dt: %.6f seconds\n", (double)_dt_ekf_avg);

	printf("IMU buffer: %d (%d Bytes)\n", _imu_buffer.get_length(), _imu_buffer.get_total_size());

	printf("minimum observation interval %d us\n", _min_obs_interval_us);

	if (_gps_buffer) {
		printf("gps buffer: %d/%d (%d Bytes)\n",
		       _gps_buffer->entries(), _gps_buffer->get_length(), _gps_buffer->get_total_size());
	}

#if defined(CONFIG_EKF2_MAGNETOMETER)

	if (_mag_buffer) {
		printf("mag buffer: %d/%d (%d Bytes)\n",
		       _mag_buffer->entries(), _mag_buffer->get_length(), _mag_buffer->get_total_size());
	}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_BAROMETER)

	if (_baro_buffer) {
		printf("baro buffer: %d/%d (%d Bytes)\n",
		       _baro_buffer->entries(), _baro_buffer->get_length(), _baro_buffer->get_total_size());
	}

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_range_buffer) {
		printf("range buffer: %d/%d (%d Bytes)\n", _range_buffer->entries(), _range_buffer->get_length(),
		       _range_buffer->get_total_size());
	}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_AIRSPEED)

	if (_airspeed_buffer) {
		printf("airspeed buffer: %d/%d (%d Bytes)\n",
		       _airspeed_buffer->entries(), _airspeed_buffer->get_length(), _airspeed_buffer->get_total_size());
	}

#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (_flow_buffer) {
		printf("flow buffer: %d/%d (%d Bytes)\n",
		       _flow_buffer->entries(), _flow_buffer->get_length(), _flow_buffer->get_total_size());
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_ext_vision_buffer) {
		printf("vision buffer: %d/%d (%d Bytes)\n",
		       _ext_vision_buffer->entries(), _ext_vision_buffer->get_length(), _ext_vision_buffer->get_total_size());
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_DRAG_FUSION)

	if (_drag_buffer) {
		printf("drag buffer: %d/%d (%d Bytes)\n",
		       _drag_buffer->entries(), _drag_buffer->get_length(), _drag_buffer->get_total_size());
	}

#endif // CONFIG_EKF2_DRAG_FUSION

	_output_predictor.print_status();
}
