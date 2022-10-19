/****************************************************************************
 *
 *   Copyright (c) 2015-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * @file control.cpp
 * Control functions for ekf attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */


#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlFusionModes()
{
	// Store the status to enable change detection
	_control_status_prev.value = _control_status.value;

	// monitor the tilt alignment
	if (!_control_status.flags.tilt_align) {
		// whilst we are aligning the tilt, monitor the variances
		const Vector3f angle_err_var_vec = calcRotVecVariances();

		// Once the tilt variances have reduced to equivalent of 3deg uncertainty
		// and declare the tilt alignment complete
		if ((angle_err_var_vec(0) + angle_err_var_vec(1)) < sq(math::radians(3.0f))) {
			_control_status.flags.tilt_align = true;

			// send alignment status message to the console
			const char *height_source = nullptr;

			if (_control_status.flags.baro_hgt) {
				height_source = "baro";

			} else if (_control_status.flags.ev_hgt) {
				height_source = "ev";

			} else if (_control_status.flags.gps_hgt) {
				height_source = "gps";

			} else if (_control_status.flags.rng_hgt) {
				height_source = "rng";

			} else {
				height_source = "unknown";

			}

			if (height_source) {
				ECL_INFO("%llu: EKF aligned, (%s hgt, IMU buf: %i, OBS buf: %i)",
					 (unsigned long long)_imu_sample_delayed.time_us, height_source, (int)_imu_buffer_length, (int)_obs_buffer_length);
			}
		}
	}

	if (_gps_buffer) {
		_gps_intermittent = !isNewestSampleRecent(_time_last_gps_buffer_push, 2 * GPS_MAX_INTERVAL);

		// check for arrival of new sensor data at the fusion time horizon
		_time_prev_gps_us = _gps_sample_delayed.time_us;
		_gps_data_ready = _gps_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &_gps_sample_delayed);

		if (_gps_data_ready) {
			// correct velocity for offset relative to IMU
			const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
			const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
			const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
			_gps_sample_delayed.vel -= vel_offset_earth;

			// correct position and height for offset relative to IMU
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_gps_sample_delayed.pos -= pos_offset_earth.xy();
			_gps_sample_delayed.hgt += pos_offset_earth(2);

			_gps_sample_delayed.sacc = fmaxf(_gps_sample_delayed.sacc, _params.gps_vel_noise);
		}
	}

	if (_range_buffer) {
		// Get range data from buffer and check validity
		_rng_data_ready = _range_buffer->pop_first_older_than(_imu_sample_delayed.time_us, _range_sensor.getSampleAddress());
		_range_sensor.setDataReadiness(_rng_data_ready);

		// update range sensor angle parameters in case they have changed
		_range_sensor.setPitchOffset(_params.rng_sens_pitch);
		_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
		_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

		_range_sensor.runChecks(_imu_sample_delayed.time_us, _R_to_earth);

		if (_range_sensor.isDataHealthy()) {
			// correct the range data for position offset relative to the IMU
			const Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_range_sensor.setRange(_range_sensor.getRange() + pos_offset_earth(2) / _range_sensor.getCosTilt());

			// Run the kinematic consistency check when not moving horizontally
			if (_control_status.flags.in_air && !_control_status.flags.fixed_wing
			    && (sq(_state.vel(0)) + sq(_state.vel(1)) < fmaxf(P(4, 4) + P(5, 5), 0.1f))) {

				const float dist_dependant_var = sq(_params.range_noise_scaler * _range_sensor.getDistBottom());
				const float var = sq(_params.range_noise) + dist_dependant_var;

				_rng_consistency_check.setGate(_params.range_kin_consistency_gate);
				_rng_consistency_check.update(_range_sensor.getDistBottom(), math::max(var, 0.001f), _state.vel(2), P(6, 6), _imu_sample_delayed.time_us);
			}

		} else {
			// If we are supposed to be using range finder data as the primary height sensor, have bad range measurements
			// and are on the ground, then synthesise a measurement at the expected on ground value
			if (!_control_status.flags.in_air
			&& _range_sensor.isRegularlySendingData()
			&& _range_sensor.isDataReady()) {

				_range_sensor.setRange(_params.rng_gnd_clearance);
				_range_sensor.setValidity(true); // bypass the checks
			}
		}

		_control_status.flags.rng_kin_consistent = _rng_consistency_check.isKinematicallyConsistent();
	}

	if (_flow_buffer) {
		// We don't fuse flow data immediately because we have to wait for the mid integration point to fall behind the fusion time horizon.
		// This means we stop looking for new data until the old data has been fused, unless we are not fusing optical flow,
		// in this case we need to empty the buffer
		if (!_flow_data_ready || (!_control_status.flags.opt_flow && !_hagl_sensor_status.flags.flow)) {
			_flow_data_ready = _flow_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &_flow_sample_delayed);
		}
	}

	if (_ext_vision_buffer) {
		_ev_data_ready = _ext_vision_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &_ev_sample_delayed);
	}

	if (_airspeed_buffer) {
		_tas_data_ready = _airspeed_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &_airspeed_sample_delayed);
	}

	// run EKF-GSF yaw estimator once per _imu_sample_delayed update after all main EKF data samples available
	runYawEKFGSF();

	// control use of observations for aiding
	controlMagFusion();
	controlOpticalFlowFusion();
	controlGpsFusion();
	controlAirDataFusion();
	controlBetaFusion();
	controlDragFusion();
	controlHeightFusion();

	// Additional data odoemtery data from an external estimator can be fused.
	controlExternalVisionFusion();

	// Additional horizontal velocity data from an auxiliary sensor can be fused
	controlAuxVelFusion();

	controlZeroInnovationHeadingUpdate();

	controlZeroVelocityUpdate();

	// Fake position measurement for constraining drift when no other velocity or position measurements
	controlFakePosFusion();
	controlFakeHgtFusion();

	// check if we are no longer fusing measurements that directly constrain velocity drift
	updateDeadReckoningStatus();
}

void Ekf::controlExternalVisionFusion()
{
	// Check for new external vision data
	if (_ev_data_ready) {

		bool reset = false;

		if (_ev_sample_delayed.reset_counter != _ev_sample_delayed_prev.reset_counter) {
			reset = true;
		}

		if (_inhibit_ev_yaw_use) {
			stopEvYawFusion();
		}

		// if the ev data is not in a NED reference frame, then the transformation between EV and EKF navigation frames
		// needs to be calculated and the observations rotated into the EKF frame of reference
		if ((_params.fusion_mode & SensorFusionMask::ROTATE_EXT_VIS) && ((_params.fusion_mode & SensorFusionMask::USE_EXT_VIS_POS)
				|| (_params.fusion_mode & SensorFusionMask::USE_EXT_VIS_VEL)) && !_control_status.flags.ev_yaw) {

			// rotate EV measurements into the EKF Navigation frame
			calcExtVisRotMat();
		}

		// external vision aiding selection logic
		if (_control_status.flags.tilt_align && _control_status.flags.yaw_align) {

			// check for a external vision measurement that has fallen behind the fusion time horizon
			if (isNewestSampleRecent(_time_last_ext_vision_buffer_push, 2 * EV_MAX_INTERVAL)) {
				// turn on use of external vision measurements for position
				if (_params.fusion_mode & SensorFusionMask::USE_EXT_VIS_POS && !_control_status.flags.ev_pos) {
					startEvPosFusion();
				}

				// turn on use of external vision measurements for velocity
				if (_params.fusion_mode & SensorFusionMask::USE_EXT_VIS_VEL && !_control_status.flags.ev_vel) {
					startEvVelFusion();
				}
			}
		}

		// external vision yaw aiding selection logic
		if (!_inhibit_ev_yaw_use && (_params.fusion_mode & SensorFusionMask::USE_EXT_VIS_YAW) && !_control_status.flags.ev_yaw
		    && _control_status.flags.tilt_align) {

			// don't start using EV data unless data is arriving frequently
			if (isNewestSampleRecent(_time_last_ext_vision_buffer_push, 2 * EV_MAX_INTERVAL)) {
				if (resetYawToEv()) {
					_control_status.flags.yaw_align = true;
					startEvYawFusion();
				}
			}
		}

		// determine if we should use the horizontal position observations
		if (_control_status.flags.ev_pos) {
			resetEstimatorAidStatus(_aid_src_ev_pos);

			if (reset && _control_status_prev.flags.ev_pos) {
				if (!_fuse_hpos_as_odom) {
					resetHorizontalPositionToVision();
				}
			}

			Vector3f ev_pos_obs_var;

			// correct position and height for offset relative to IMU
			const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_ev_sample_delayed.pos -= pos_offset_earth;

			// Use an incremental position fusion method for EV position data if GPS is also used
			if (_params.gnss_ctrl & GnssCtrl::HPOS) {
				_fuse_hpos_as_odom = true;

			} else {
				_fuse_hpos_as_odom = false;
			}

			if (_fuse_hpos_as_odom) {
				if (!_hpos_prev_available) {
					// no previous observation available to calculate position change
					_hpos_prev_available = true;

				} else {
					// calculate the change in position since the last measurement
					// rotate measurement into body frame is required when fusing with GPS
					Vector3f ev_delta_pos = _R_ev_to_ekf * Vector3f(_ev_sample_delayed.pos - _ev_sample_delayed_prev.pos);

					// use the change in position since the last measurement
					_aid_src_ev_pos.observation[0] = ev_delta_pos(0);
					_aid_src_ev_pos.observation[1] = ev_delta_pos(1);

					_aid_src_ev_pos.innovation[0] = _state.pos(0) - _hpos_pred_prev(0) - ev_delta_pos(0);
					_aid_src_ev_pos.innovation[1] = _state.pos(1) - _hpos_pred_prev(1) - ev_delta_pos(1);

					// observation 1-STD error, incremental pos observation is expected to have more uncertainty
					Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);
					ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();
					ev_pos_obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.5f));
					ev_pos_obs_var(1) = fmaxf(ev_pos_var(1, 1), sq(0.5f));

					_aid_src_ev_pos.observation_variance[0] = ev_pos_obs_var(0);
					_aid_src_ev_pos.observation_variance[1] = ev_pos_obs_var(1);

					_aid_src_ev_pos.innovation_variance[0] = P(7, 7) + _aid_src_ev_pos.observation_variance[0];
					_aid_src_ev_pos.innovation_variance[1] = P(8, 8) + _aid_src_ev_pos.observation_variance[1];
				}
			} else {
				// use the absolute position
				Vector3f ev_pos_meas = _ev_sample_delayed.pos;
				Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);

				if (_params.fusion_mode & SensorFusionMask::ROTATE_EXT_VIS) {
					ev_pos_meas = _R_ev_to_ekf * ev_pos_meas;
					ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();
				}

				_aid_src_ev_pos.observation[0] = ev_pos_meas(0);
				_aid_src_ev_pos.observation[1] = ev_pos_meas(1);

				_aid_src_ev_pos.observation_variance[0] = fmaxf(ev_pos_var(0, 0), sq(0.01f));
				_aid_src_ev_pos.observation_variance[1] = fmaxf(ev_pos_var(1, 1), sq(0.01f));

				_aid_src_ev_pos.innovation[0] = _state.pos(0) - _aid_src_ev_pos.observation[0];
				_aid_src_ev_pos.innovation[1] = _state.pos(1) - _aid_src_ev_pos.observation[1];

				_aid_src_ev_pos.innovation_variance[0] = P(7, 7) + _aid_src_ev_pos.observation_variance[0];
				_aid_src_ev_pos.innovation_variance[1] = P(8, 8) + _aid_src_ev_pos.observation_variance[1];

				// check if we have been deadreckoning too long
				if (isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)) {
					// only reset velocity if we have no another source of aiding constraining it
					if (isTimedOut(_aid_src_optical_flow.time_last_fuse, (uint64_t)1E6) &&
					    isTimedOut(_time_last_hor_vel_fuse, (uint64_t)1E6)) {

						if (_control_status.flags.ev_vel) {
							resetVelocityToVision();
						}
					}

					resetHorizontalPositionToVision();
				}
			}

			// innovation gate size
			const float ev_pos_innov_gate = fmaxf(_params.ev_pos_innov_gate, 1.0f);
			setEstimatorAidStatusTestRatio(_aid_src_ev_pos, ev_pos_innov_gate);

			_aid_src_ev_pos.timestamp_sample = _ev_sample_delayed.time_us;
			_aid_src_ev_pos.fusion_enabled = true;

			fuseHorizontalPosition(_aid_src_ev_pos);
		}

		// determine if we should use the velocity observations
		if (_control_status.flags.ev_vel) {
			if (reset && _control_status_prev.flags.ev_vel) {
				resetVelocityToVision();
			}

			// check if we have been deadreckoning too long
			if (isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)) {
				// only reset velocity if we have no another source of aiding constraining it
				if (isTimedOut(_aid_src_optical_flow.time_last_fuse, (uint64_t)1E6) &&
				    isTimedOut(_time_last_hor_pos_fuse, (uint64_t)1E6)) {
					resetVelocityToVision();
				}
			}

			resetEstimatorAidStatus(_aid_src_ev_vel);

			const Vector3f obs_var = matrix::max(getVisionVelocityVarianceInEkfFrame(), sq(0.05f));

			const float innov_gate = fmaxf(_params.ev_vel_innov_gate, 1.f);

			updateVelocityAidSrcStatus(_ev_sample_delayed.time_us, getVisionVelocityInEkfFrame(), obs_var, innov_gate, _aid_src_ev_vel);

			_aid_src_ev_vel.fusion_enabled = true;

			fuseVelocity(_aid_src_ev_vel);
		}

		// determine if we should use the yaw observation
		resetEstimatorAidStatus(_aid_src_ev_yaw);
		const float measured_hdg = getEulerYaw(_ev_sample_delayed.quat);
		const float ev_yaw_obs_var = fmaxf(_ev_sample_delayed.angVar, 1.e-4f);

		if (PX4_ISFINITE(measured_hdg)) {
			_aid_src_ev_yaw.timestamp_sample = _ev_sample_delayed.time_us;
			_aid_src_ev_yaw.observation = measured_hdg;
			_aid_src_ev_yaw.observation_variance = ev_yaw_obs_var;
			_aid_src_ev_yaw.fusion_enabled = _control_status.flags.ev_yaw;

			if (_control_status.flags.ev_yaw) {
				if (reset && _control_status_prev.flags.ev_yaw) {
					resetYawToEv();
				}

				const float innovation = wrap_pi(getEulerYaw(_R_to_earth) - measured_hdg);

				fuseYaw(innovation, ev_yaw_obs_var, _aid_src_ev_yaw);

			} else {
				// populate estimator_aid_src_ev_yaw with delta heading innovations for logging
				// use the change in yaw since the last measurement
				const float measured_hdg_prev = getEulerYaw(_ev_sample_delayed_prev.quat);

				// calculate the change in yaw since the last measurement
				const float ev_delta_yaw = wrap_pi(measured_hdg - measured_hdg_prev);

				_aid_src_ev_yaw.innovation = wrap_pi(getEulerYaw(_R_to_earth) - _yaw_pred_prev - ev_delta_yaw);
			}
		}

		// record observation and estimate for use next time
		_ev_sample_delayed_prev = _ev_sample_delayed;
		_hpos_pred_prev = _state.pos.xy();
		_yaw_pred_prev = getEulerYaw(_state.quat_nominal);

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel ||  _control_status.flags.ev_yaw)
		   && !isRecent(_ev_sample_delayed.time_us, (uint64_t)_params.reset_timeout_max)) {

		// Turn off EV fusion mode if no data has been received
		stopEvFusion();
		_warning_events.flags.vision_data_stopped = true;
		ECL_WARN("vision data stopped");
	}
}

void Ekf::controlGpsYawFusion(const gpsSample &gps_sample, bool gps_checks_passing, bool gps_checks_failing)
{
	if (!(_params.gnss_ctrl & GnssCtrl::YAW)
	    || _control_status.flags.gps_yaw_fault) {

		stopGpsYawFusion();
		return;
	}

	const bool is_new_data_available = PX4_ISFINITE(gps_sample.yaw);

	if (is_new_data_available) {

		const bool continuing_conditions_passing = !gps_checks_failing;

		const bool is_gps_yaw_data_intermittent = !isNewestSampleRecent(_time_last_gps_yaw_buffer_push, 2 * GPS_MAX_INTERVAL);

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _control_status.flags.tilt_align
				&& gps_checks_passing
				&& !is_gps_yaw_data_intermittent
				&& !_gps_intermittent;

		if (_control_status.flags.gps_yaw) {

			if (continuing_conditions_passing) {

				fuseGpsYaw(gps_sample);

				const bool is_fusion_failing = isTimedOut(_aid_src_gnss_yaw.time_last_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (_nb_gps_yaw_reset_available > 0) {
						// Data seems good, attempt a reset
						resetYawToGps(gps_sample.yaw);

						if (_control_status.flags.in_air) {
							_nb_gps_yaw_reset_available--;
						}

					} else if (starting_conditions_passing) {
						// Data seems good, but previous reset did not fix the issue
						// something else must be wrong, declare the sensor faulty and stop the fusion
						_control_status.flags.gps_yaw_fault = true;
						stopGpsYawFusion();

					} else {
						// A reset did not fix the issue but all the starting checks are not passing
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopGpsYawFusion();
					}

					// TODO: should we give a new reset credit when the fusion does not fail for some time?
				}

			} else {
				// Stop GPS yaw fusion but do not declare it faulty
				stopGpsYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate GPS yaw fusion
				startGpsYawFusion(gps_sample);

				if (_control_status.flags.gps_yaw) {
					_nb_gps_yaw_reset_available = 1;
				}
			}
		}

	} else if (_control_status.flags.gps_yaw && !isNewestSampleRecent(_time_last_gps_yaw_buffer_push, _params.reset_timeout_max)) {
		// No yaw data in the message anymore. Stop until it comes back.
		stopGpsYawFusion();
	}

	// Before takeoff, we do not want to continue to rely on the current heading
	// if we had to stop the fusion
	if (!_control_status.flags.in_air
	    && !_control_status.flags.gps_yaw
	    && _control_status_prev.flags.gps_yaw) {
		_control_status.flags.yaw_align = false;
	}
}

void Ekf::controlAirDataFusion()
{
	// control activation and initialisation/reset of wind states required for airspeed fusion

	// If both airspeed and sideslip fusion have timed out and we are not using a drag observation model then we no longer have valid wind estimates
	const bool airspeed_timed_out = isTimedOut(_aid_src_airspeed.time_last_fuse, (uint64_t)10e6);
	const bool sideslip_timed_out = isTimedOut(_aid_src_sideslip.time_last_fuse, (uint64_t)10e6);

	if (_control_status.flags.fake_pos || (airspeed_timed_out && sideslip_timed_out && !(_params.fusion_mode & SensorFusionMask::USE_DRAG))) {
		_control_status.flags.wind = false;
	}

	if (_params.arsp_thr <= 0.f) {
		stopAirspeedFusion();
		return;
	}

	if (_tas_data_ready) {
		updateAirspeed(_airspeed_sample_delayed, _aid_src_airspeed);

		_innov_check_fail_status.flags.reject_airspeed = _aid_src_airspeed.innovation_rejected; // TODO: remove this redundant flag

		const bool continuing_conditions_passing = _control_status.flags.in_air && _control_status.flags.fixed_wing && !_control_status.flags.fake_pos;
		const bool is_airspeed_significant = _airspeed_sample_delayed.true_airspeed > _params.arsp_thr;
		const bool is_airspeed_consistent = (_aid_src_airspeed.test_ratio > 0.f && _aid_src_airspeed.test_ratio < 1.f);
		const bool starting_conditions_passing = continuing_conditions_passing && is_airspeed_significant
		                                         && (is_airspeed_consistent || !_control_status.flags.wind); // if wind isn't already estimated, the states are reset when starting airspeed fusion

		if (_control_status.flags.fuse_aspd) {
			if (continuing_conditions_passing) {
				if (is_airspeed_significant) {
					fuseAirspeed(_aid_src_airspeed);
				}

				const bool is_fusion_failing = isTimedOut(_aid_src_airspeed.time_last_fuse, (uint64_t)10e6);

				if (is_fusion_failing) {
					stopAirspeedFusion();
				}

			} else {
				stopAirspeedFusion();
			}

		} else if (starting_conditions_passing) {
			startAirspeedFusion();
		}

	} else if (_control_status.flags.fuse_aspd && !isRecent(_airspeed_sample_delayed.time_us, (uint64_t)1e6)) {
		ECL_WARN("Airspeed data stopped");
		stopAirspeedFusion();
	}
}

void Ekf::controlBetaFusion()
{
	if (_control_status.flags.fake_pos) {
		return;
	}

	// Perform synthetic sideslip fusion at regular intervals when in-air and sideslip fuson had been enabled externally:
	const bool beta_fusion_time_triggered = isTimedOut(_aid_src_sideslip.time_last_fuse, _params.beta_avg_ft_us);

	if (beta_fusion_time_triggered &&
	    _control_status.flags.fuse_beta &&
	    _control_status.flags.in_air) {
		updateSideslip(_aid_src_sideslip);
		_innov_check_fail_status.flags.reject_sideslip = _aid_src_sideslip.innovation_rejected;

		// If starting wind state estimation, reset the wind states and covariances before fusing any data
		if (!_control_status.flags.wind) {
			// activate the wind states
			_control_status.flags.wind = true;
			// reset the timeout timers to prevent repeated resets
			_aid_src_sideslip.time_last_fuse = _imu_sample_delayed.time_us;
			resetWind();
		}

		if (Vector2f(Vector2f(_state.vel) - _state.wind_vel).longerThan(7.f)) {
			fuseSideslip(_aid_src_sideslip);
		}
	}
}

void Ekf::controlDragFusion()
{
	if ((_params.fusion_mode & SensorFusionMask::USE_DRAG) && _drag_buffer &&
	    !_control_status.flags.fake_pos && _control_status.flags.in_air && !_mag_inhibit_yaw_reset_req) {

		if (!_control_status.flags.wind) {
			// reset the wind states and covariances when starting drag accel fusion
			_control_status.flags.wind = true;
			resetWind();

		}


		dragSample drag_sample;

		if (_drag_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &drag_sample)) {
			fuseDrag(drag_sample);
		}
	}
}

void Ekf::controlAuxVelFusion()
{
	if (_auxvel_buffer) {
		auxVelSample auxvel_sample_delayed;

		if (_auxvel_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &auxvel_sample_delayed)) {

			resetEstimatorAidStatus(_aid_src_aux_vel);

			updateVelocityAidSrcStatus(auxvel_sample_delayed.time_us, auxvel_sample_delayed.vel, auxvel_sample_delayed.velVar, fmaxf(_params.auxvel_gate, 1.f), _aid_src_aux_vel);

			if (isHorizontalAidingActive()) {
				_aid_src_aux_vel.fusion_enabled = true;
				fuseVelocity(_aid_src_aux_vel);
			}
		}
	}
}

bool Ekf::hasHorizontalAidingTimedOut() const
{
	return isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
	       && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
	       && isTimedOut(_aid_src_optical_flow.time_last_fuse, _params.reset_timeout_max);
}
