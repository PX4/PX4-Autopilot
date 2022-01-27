/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * @file terrain_estimator.cpp
 * Function for fusing rangefinder measurements to estimate terrain vertical position/
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

void Ekf::initHagl()
{
	resetHaglFake();
}

void Ekf::controlHaglFusion()
{
	// If we are on ground, store the local position and time to use as a reference
	if (!_control_status.flags.in_air) {
		_last_on_ground_posD = _state.pos(2);
		_control_status.flags.rng_fault = false;
	}

	controlHaglRngFusion();
	controlHaglFlowFusion();
	controlHaglFakeFusion();

	// constrain terrain position to be a minimum of _params.rng_gnd_clearance larger than _state.pos(2)
	if (_state.posd_terrain - _state.pos(2) < _params.rng_gnd_clearance) {
		_state.posd_terrain = _params.rng_gnd_clearance + _state.pos(2);
	}

	updateTerrainValidity();
}

void Ekf::controlHaglRngFusion()
{
	if (!(_params.terrain_fusion_mode & TerrainFusionMask::TerrainFuseRangeFinder)
	    || _control_status.flags.rng_fault) {

		stopHaglRngFusion();
		return;
	}

	if (_range_sensor.isDataHealthy()) {
		const bool continuing_conditions_passing = _control_status.flags.in_air;
		//const bool continuing_conditions_passing = _control_status.flags.in_air && !_control_status.flags.rng_hgt; // TODO: should not be fused when using range height
		const bool starting_conditions_passing = continuing_conditions_passing && _range_sensor.isRegularlySendingData();

		_time_last_healthy_rng_data = _time_last_imu;

		if (_hagl_sensor_status.flags.range_finder) {
			if (continuing_conditions_passing) {
				/* fuseHaglRng(); */ // done when fusing range finder

				// We have been rejecting range data for too long
				const uint64_t timeout = static_cast<uint64_t>(_params.terrain_timeout * 1e6f);
				const bool is_fusion_failing = isTimedOut(_time_last_hagl_fuse, timeout);

				if (is_fusion_failing) {
					if (_range_sensor.getDistBottom() > 2.f * _params.rng_gnd_clearance) {
						// Data seems good, attempt a reset
						resetHaglRng();

					} else if (starting_conditions_passing) {
						// The sensor can probably not detect the ground properly
						// declare the sensor faulty and stop the fusion
						_control_status.flags.rng_fault = true;
						_range_sensor.setFaulty();
						stopHaglRngFusion();

					} else {
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopHaglRngFusion();
					}
				}

			} else {
				stopHaglRngFusion();
			}

		} else {
			if (starting_conditions_passing) {
				startHaglRngFusion();
			}
		}

	} else if (_hagl_sensor_status.flags.range_finder && isTimedOut(_time_last_healthy_rng_data, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopHaglRngFusion();
	}
}

void Ekf::startHaglRngFusion()
{
	_hagl_sensor_status.flags.range_finder = true;
	resetHaglRngIfNeeded();
}

void Ekf::resetHaglRngIfNeeded()
{
	if (_hagl_sensor_status.flags.flow) {
		const float meas_hagl = _range_sensor.getDistBottom();
		const float pred_hagl = _state.posd_terrain - _state.pos(2);
		const float hagl_innov = pred_hagl - meas_hagl;
		const float obs_variance = getRngVar();

		const float hagl_innov_var = fmaxf(P(24, 24) + obs_variance, obs_variance);

		const float gate_size = fmaxf(_params.range_innov_gate, 1.0f);
		const float hagl_test_ratio = sq(hagl_innov) / (sq(gate_size) * hagl_innov_var);

		// Reset the state to the measurement only if the test ratio is large,
		// otherwise let it converge through the fusion
		if (hagl_test_ratio > 0.2f) {
			resetHaglRng();

		} else {
			/* fuseHaglRng(); */
		}

	} else {
		resetHaglRng();
	}
}

float Ekf::getRngVar()
{
	return fmaxf(P(9, 9) * _params.vehicle_variance_scaler, 0.0f)
	       + sq(_params.range_noise)
	       + sq(_params.range_noise_scaler * _range_sensor.getRange());
}

void Ekf::resetHaglRng()
{
	_state.posd_terrain = _state.pos(2) + _range_sensor.getDistBottom();
	P(24, 24) = getRngVar();
	_terrain_vpos_reset_counter++;
	_time_last_hagl_fuse = _time_last_imu;
}

void Ekf::stopHaglRngFusion()
{
	_hagl_sensor_status.flags.range_finder = false;
}

void Ekf::controlHaglFlowFusion()
{
	if (!(_params.terrain_fusion_mode & TerrainFusionMask::TerrainFuseOpticalFlow)) {
		stopHaglFlowFusion();
		return;
	}

	if (_flow_data_ready) {
		const bool continuing_conditions_passing = _control_status.flags.in_air
		                                           && !_control_status.flags.opt_flow
							   && _control_status.flags.gps
							   && isTimedOut(_time_last_hagl_fuse, 5e6f); // TODO: check for range_finder hagl aiding instead?
							   /* && !_hagl_sensor_status.flags.range_finder; */
		const bool starting_conditions_passing = continuing_conditions_passing;

		if (_hagl_sensor_status.flags.flow) {
			if (continuing_conditions_passing) {

				// TODO: wait until the midpoint of the flow sample has fallen behind the fusion time horizon
				/* fuseFlowForTerrain(); */
				/* _flow_data_ready = false; */ // done in flow fusion

				// TODO: do something when failing continuously the innovation check
				/* const bool is_fusion_failing = isTimedOut(_time_last_flow_terrain_fuse, _params.reset_timeout_max); */

				/* if (is_fusion_failing) { */
				/* 	resetHaglFlow(); */
				/* } */

			} else {
				stopHaglFlowFusion();
			}

		} else {
			if (starting_conditions_passing) {
				startHaglFlowFusion();
			}
		}

	} else if (_hagl_sensor_status.flags.flow
		   && (_imu_sample_delayed.time_us >  _flow_sample_delayed.time_us + (uint64_t)5e6)) {
		// No data anymore. Stop until it comes back.
		stopHaglFlowFusion();
	}
}

void Ekf::startHaglFlowFusion()
{
	_hagl_sensor_status.flags.flow = true;
	// TODO: do a reset instead of trying to fuse the data?
	/* fuseFlowForTerrain(); */
	/* _flow_data_ready = false; */
}

void Ekf::stopHaglFlowFusion()
{
	_hagl_sensor_status.flags.flow = false;
}

void Ekf::resetHaglFlow()
{
	// TODO: use the flow data
	_state.posd_terrain = fmaxf(0.0f,  _state.pos(2));
	P(24, 24) = 100.0f;
	_terrain_vpos_reset_counter++;
}

void Ekf::controlHaglFakeFusion()
{
	if (!_control_status.flags.in_air
	    && !_hagl_sensor_status.flags.range_finder
	    && !_hagl_sensor_status.flags.flow) {
		resetHaglFake();
	}
}

void Ekf::resetHaglFake()
{
	// assume a ground clearance
	_state.posd_terrain = _state.pos(2) + _params.rng_gnd_clearance;
	// use the ground clearance value as our uncertainty
	P(24, 24) = sq(_params.rng_gnd_clearance);
	_time_last_hagl_fuse = _time_last_imu;
}

void Ekf::updateTerrainValidity()
{
	// we have been fusing range finder measurements in the last 5 seconds
	const bool recent_range_fusion = isRecent(_time_last_hagl_fuse, (uint64_t)5e6);

	// we have been fusing optical flow measurements for terrain estimation within the last 5 seconds
	// this can only be the case if the main filter does not fuse optical flow
	const bool recent_flow_for_terrain_fusion = isRecent(_time_last_flow_terrain_fuse, (uint64_t)5e6);

	_hagl_valid = (recent_range_fusion || recent_flow_for_terrain_fusion);
}

void Ekf::fuseHaglAllStates()
{
	// get a height above ground measurement from the range finder assuming a flat earth
	const float meas_hagl = _range_sensor.getDistBottom();

	// predict the hagl from the vehicle position and terrain height
	const float pred_hagl = _state.posd_terrain - _state.pos(2);

	// calculate the innovation
	_hagl_innov = pred_hagl - meas_hagl;

	// calculate the observation variance adding the variance of the vehicles own height uncertainty
	const float obs_variance = fmaxf(P(9, 9) * _params.vehicle_variance_scaler, 0.0f)
				   + sq(_params.range_noise)
				   + sq(_params.range_noise_scaler * _range_sensor.getRange());

	// calculate the innovation variance - limiting it to prevent a badly conditioned fusion
	_hagl_innov_var = fmaxf(P(24,24) - 2*P(9,24) + P(9,9) + obs_variance, obs_variance);

	// perform an innovation consistency check and only fuse data if it passes
	const float gate_size = fmaxf(_params.range_innov_gate, 1.0f);
	_hagl_test_ratio = sq(_hagl_innov) / (sq(gate_size) * _hagl_innov_var);

	bool is_fused = false;
	if (_hagl_test_ratio <= 1.0f) {
		// calculate the Kalman gain
		const float HK0 = 1.0f/_hagl_innov_var;

		// calculate the observation Jacobians and Kalman gains
		SparseVector25f<9,24> Hfusion;
		Vector25f Kfusion;

		if (_control_status.flags.rng_hgt) {
			Hfusion.at<9>() = -1.0f;
			if (_hagl_sensor_status.flags.range_finder || _hagl_sensor_status.flags.flow) {
				for (uint8_t index=0; index<=23; index++) {
					Kfusion(index) = HK0*(P(index,24) - P(index,9));
				}

			} else {
				for (uint8_t index=0; index<=23; index++) {
					Kfusion(index) = - HK0*P(index,9);
				}
			}

		} else {
			for (unsigned row=0; row<=23; row++) {
				// update of all states other than terrain is inhibited
				Kfusion(row) = 0.0f;
			}
		}

		if (_hagl_sensor_status.flags.range_finder) {
			Hfusion.at<24>() = 1.0f;

			if (_control_status.flags.rng_hgt) {
				Kfusion(24) = HK0*(P(24,24) - P(24,9));

			} else {
				Kfusion(24) = HK0*P(24,24);
			}

		} else {
			Kfusion(24) = 0.0f;
		}

		is_fused = measurementUpdate(Kfusion, Hfusion, _hagl_innov);
	}

	if (is_fused) {
		// record last successful fusion event
		if (_hagl_sensor_status.flags.range_finder) {
			_time_last_hagl_fuse = _time_last_imu;
			_innov_check_fail_status.flags.reject_hagl = false;
		}

		if (_control_status.flags.rng_hgt) {
			_time_last_hgt_fuse = _time_last_imu;
			_innov_check_fail_status.flags.reject_ver_pos = false;
		}

	} else {
		if (_hagl_sensor_status.flags.range_finder) {
			_innov_check_fail_status.flags.reject_hagl = true;
		}

		if (_control_status.flags.rng_hgt) {
			_innov_check_fail_status.flags.reject_ver_pos = true;
		}
	}
	updateTerrainValidity();
}
