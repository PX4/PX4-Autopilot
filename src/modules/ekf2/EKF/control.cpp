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

	if (_baro_buffer) {
		// check for intermittent data
		_baro_hgt_intermittent = !isRecent(_time_last_baro, 2 * BARO_MAX_INTERVAL);

		const uint64_t baro_time_prev = _baro_sample_delayed.time_us;
		_baro_data_ready = _baro_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed);

		// if we have a new baro sample save the delta time between this sample and the last sample which is
		// used below for baro offset calculations
		if (_baro_data_ready && baro_time_prev != 0) {
			_delta_time_baro_us = _baro_sample_delayed.time_us - baro_time_prev;
		}
	}


	if (_gps_buffer) {
		_gps_intermittent = !isRecent(_time_last_gps, 2 * GPS_MAX_INTERVAL);

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
			if ((sq(_state.vel(0)) + sq(_state.vel(1)) < fmaxf(P(4, 4) + P(5, 5), 0.1f))) {
				_rng_consistency_check.setGate(_params.range_kin_consistency_gate);
				_rng_consistency_check.update(_range_sensor.getDistBottom(), getRngHeightVariance(), _state.vel(2), P(6, 6), _time_last_imu);
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

	// check for height sensor timeouts and reset and change sensor if necessary
	controlHeightSensorTimeouts();

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

	controlZeroVelocityUpdate();

	// Fake position measurement for constraining drift when no other velocity or position measurements
	controlFakePosFusion();

	// check if we are no longer fusing measurements that directly constrain velocity drift
	update_deadreckoning_status();
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
			if (isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)) {
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
			if (isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)) {
				if (resetYawToEv()) {
					_control_status.flags.yaw_align = true;
					startEvYawFusion();
				}
			}
		}

		// determine if we should use the horizontal position observations
		if (_control_status.flags.ev_pos) {
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
			if (_params.fusion_mode & SensorFusionMask::USE_GPS) {
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
					_ev_pos_innov(0) = _state.pos(0) - _hpos_pred_prev(0) - ev_delta_pos(0);
					_ev_pos_innov(1) = _state.pos(1) - _hpos_pred_prev(1) - ev_delta_pos(1);

					// observation 1-STD error, incremental pos observation is expected to have more uncertainty
					Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);
					ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();
					ev_pos_obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.5f));
					ev_pos_obs_var(1) = fmaxf(ev_pos_var(1, 1), sq(0.5f));
				}
			} else {
				// use the absolute position
				Vector3f ev_pos_meas = _ev_sample_delayed.pos;
				Matrix3f ev_pos_var = matrix::diag(_ev_sample_delayed.posVar);

				if (_params.fusion_mode & SensorFusionMask::ROTATE_EXT_VIS) {
					ev_pos_meas = _R_ev_to_ekf * ev_pos_meas;
					ev_pos_var = _R_ev_to_ekf * ev_pos_var * _R_ev_to_ekf.transpose();
				}

				_ev_pos_innov(0) = _state.pos(0) - ev_pos_meas(0);
				_ev_pos_innov(1) = _state.pos(1) - ev_pos_meas(1);

				ev_pos_obs_var(0) = fmaxf(ev_pos_var(0, 0), sq(0.01f));
				ev_pos_obs_var(1) = fmaxf(ev_pos_var(1, 1), sq(0.01f));

				// check if we have been deadreckoning too long
				if (isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)) {
					// only reset velocity if we have no another source of aiding constraining it
					if (isTimedOut(_time_last_of_fuse, (uint64_t)1E6) &&
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

			fuseHorizontalPosition(_ev_pos_innov, ev_pos_innov_gate, ev_pos_obs_var, _ev_pos_innov_var, _ev_pos_test_ratio);
		}

		// determine if we should use the velocity observations
		if (_control_status.flags.ev_vel) {
			if (reset && _control_status_prev.flags.ev_vel) {
				resetVelocityToVision();
			}

			_ev_vel_innov = _state.vel - getVisionVelocityInEkfFrame();

			// check if we have been deadreckoning too long
			if (isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)) {
				// only reset velocity if we have no another source of aiding constraining it
				if (isTimedOut(_time_last_of_fuse, (uint64_t)1E6) &&
				    isTimedOut(_time_last_hor_pos_fuse, (uint64_t)1E6)) {
					resetVelocityToVision();
				}
			}

			const Vector3f obs_var = matrix::max(getVisionVelocityVarianceInEkfFrame(), sq(0.05f));

			const float innov_gate = fmaxf(_params.ev_vel_innov_gate, 1.f);

			fuseHorizontalVelocity(_ev_vel_innov, innov_gate, obs_var, _ev_vel_innov_var, _ev_vel_test_ratio);
			fuseVerticalVelocity(_ev_vel_innov, innov_gate, obs_var, _ev_vel_innov_var, _ev_vel_test_ratio);
		}

		// determine if we should use the yaw observation
		if (_control_status.flags.ev_yaw) {
			if (reset && _control_status_prev.flags.ev_yaw) {
				resetYawToEv();
			}

			if (shouldUse321RotationSequence(_R_to_earth)) {
				float measured_hdg = getEuler321Yaw(_ev_sample_delayed.quat);
				fuseYaw321(measured_hdg, _ev_sample_delayed.angVar);

			} else {
				float measured_hdg = getEuler312Yaw(_ev_sample_delayed.quat);
				fuseYaw312(measured_hdg, _ev_sample_delayed.angVar);
			}
		}

		// record observation and estimate for use next time
		_ev_sample_delayed_prev = _ev_sample_delayed;
		_hpos_pred_prev = _state.pos.xy();

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel ||  _control_status.flags.ev_yaw)
		   && isTimedOut(_time_last_ext_vision, (uint64_t)_params.reset_timeout_max)) {

		// Turn off EV fusion mode if no data has been received
		stopEvFusion();
		_warning_events.flags.vision_data_stopped = true;
		ECL_WARN("vision data stopped");
	}
}

void Ekf::controlOpticalFlowFusion()
{
	// Check if on ground motion is un-suitable for use of optical flow
	if (!_control_status.flags.in_air) {
		updateOnGroundMotionForOpticalFlowChecks();

	} else {
		resetOnGroundMotionForOpticalFlowChecks();
	}

	// Accumulate autopilot gyro data across the same time interval as the flow sensor
	_imu_del_ang_of += _imu_sample_delayed.delta_ang - _state.delta_ang_bias;
	_delta_time_of += _imu_sample_delayed.delta_ang_dt;

	if (_flow_data_ready) {
		const bool is_quality_good = (_flow_sample_delayed.quality >= _params.flow_qual_min);
		const bool is_magnitude_good = !_flow_sample_delayed.flow_xy_rad.longerThan(_flow_sample_delayed.dt * _flow_max_rate);
		const bool is_tilt_good = (_R_to_earth(2, 2) > _params.range_cos_max_tilt);

		const float delta_time_min = fmaxf(0.7f * _delta_time_of, 0.001f);
		const float delta_time_max = fminf(1.3f * _delta_time_of, 0.2f);
		const bool is_delta_time_good = _flow_sample_delayed.dt >= delta_time_min && _flow_sample_delayed.dt <= delta_time_max;
		const bool is_body_rate_comp_available = calcOptFlowBodyRateComp();

		if (is_quality_good
		    && is_magnitude_good
		    && is_tilt_good
		    && is_body_rate_comp_available
		    && is_delta_time_good) {
			// compensate for body motion to give a LOS rate
			_flow_compensated_XY_rad = _flow_sample_delayed.flow_xy_rad - _flow_sample_delayed.gyro_xyz.xy();

		} else if (!_control_status.flags.in_air) {

			if (!is_delta_time_good) {
				// handle special case of SITL and PX4Flow where dt is forced to
				// zero when the quaity is 0
				_flow_sample_delayed.dt = delta_time_min;
			}

			// don't allow invalid flow gyro_xyz to propagate
			if (!is_body_rate_comp_available) {
				if (!PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(0)) || !PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(1)) || !PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(2))) {
					_flow_sample_delayed.gyro_xyz.zero();
				}
			}

			// when on the ground with poor flow quality,
			// assume zero ground relative velocity and LOS rate
			_flow_compensated_XY_rad.setZero();

		} else {
			// don't use this flow data and wait for the next data to arrive
			_flow_data_ready = false;
			_flow_compensated_XY_rad.setZero();
		}
	} else {
		_flow_compensated_XY_rad.setZero();
	}

	// New optical flow data is available and is ready to be fused when the midpoint of the sample falls behind the fusion time horizon
	if (_flow_data_ready) {
		// Inhibit flow use if motion is un-suitable or we have good quality GPS
		// Apply hysteresis to prevent rapid mode switching
		const float gps_err_norm_lim = _control_status.flags.opt_flow ? 0.7f : 1.0f;

		// Check if we are in-air and require optical flow to control position drift
		const bool is_flow_required = _control_status.flags.in_air
					      && (_control_status.flags.inertial_dead_reckoning // is doing inertial dead-reckoning so must constrain drift urgently
						  || isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)
						  || (_control_status.flags.gps && (_gps_error_norm > gps_err_norm_lim))); // is using GPS, but GPS is bad


		// inhibit use of optical flow if motion is unsuitable and we are not reliant on it for flight navigation
		const bool preflight_motion_not_ok = !_control_status.flags.in_air
						     && ((_imu_sample_delayed.time_us > (_time_good_motion_us + (uint64_t)1E5))
								     || (_imu_sample_delayed.time_us < (_time_bad_motion_us + (uint64_t)5E6)));
		const bool flight_condition_not_ok = _control_status.flags.in_air && !isTerrainEstimateValid();

		_inhibit_flow_use = ((preflight_motion_not_ok || flight_condition_not_ok) && !is_flow_required)
				    || !_control_status.flags.tilt_align;

		// Handle cases where we are using optical flow but we should not use it anymore
		if (_control_status.flags.opt_flow) {
			if (!(_params.fusion_mode & SensorFusionMask::USE_OPT_FLOW)
			    || _inhibit_flow_use) {

				stopFlowFusion();
				return;
			}
		}

		// optical flow fusion mode selection logic
		if ((_params.fusion_mode & SensorFusionMask::USE_OPT_FLOW) // optical flow has been selected by the user
		    && !_control_status.flags.opt_flow // we are not yet using flow data
		    && !_inhibit_flow_use) {
			// If the heading is valid and use is not inhibited , start using optical flow aiding
			if (_control_status.flags.yaw_align || _params.mag_fusion_type == MagFuseType::NONE) {
				// set the flag and reset the fusion timeout
				ECL_INFO("starting optical flow fusion");
				_control_status.flags.opt_flow = true;
				_time_last_of_fuse = _time_last_imu;

				// if we are not using GPS or external vision aiding, then the velocity and position states and covariances need to be set
				const bool flow_aid_only = !isOtherSourceOfHorizontalAidingThan(_control_status.flags.opt_flow);

				if (flow_aid_only) {
					resetHorizontalVelocityToOpticalFlow();
					resetHorizontalPositionToOpticalFlow();
				}
			}
		}

		if (_control_status.flags.opt_flow) {
			// Wait until the midpoint of the flow sample has fallen behind the fusion time horizon
			if (_imu_sample_delayed.time_us > (_flow_sample_delayed.time_us - uint32_t(1e6f * _flow_sample_delayed.dt) / 2)) {
				// Fuse optical flow LOS rate observations into the main filter only if height above ground has been updated recently
				// but use a relaxed time criteria to enable it to coast through bad range finder data
				if (isRecent(_time_last_hagl_fuse, (uint64_t)10e6)) {
					fuseOptFlow();
					_last_known_posNE = _state.pos.xy();
				}

				_flow_data_ready = false;
			}

			// handle the case when we have optical flow, are reliant on it, but have not been using it for an extended period
			if (isTimedOut(_time_last_of_fuse, _params.reset_timeout_max)
			    && !isOtherSourceOfHorizontalAidingThan(_control_status.flags.opt_flow)) {

				resetHorizontalVelocityToOpticalFlow();
				resetHorizontalPositionToOpticalFlow();
			}
		}

	} else if (_control_status.flags.opt_flow
		   && (_imu_sample_delayed.time_us > _flow_sample_delayed.time_us + (uint64_t)10e6)) {

		stopFlowFusion();
	}
}

void Ekf::updateOnGroundMotionForOpticalFlowChecks()
{
	// When on ground check if the vehicle is being shaken or moved in a way that could cause a loss of navigation
	const float accel_norm = _accel_vec_filt.norm();

	const bool motion_is_excessive = ((accel_norm > (CONSTANTS_ONE_G * 1.5f)) // upper g limit
					  || (accel_norm < (CONSTANTS_ONE_G * 0.5f)) // lower g limit
					  || (_ang_rate_magnitude_filt > _flow_max_rate) // angular rate exceeds flow sensor limit
					  || (_R_to_earth(2, 2) < cosf(math::radians(30.0f)))); // tilted excessively

	if (motion_is_excessive) {
		_time_bad_motion_us = _imu_sample_delayed.time_us;

	} else {
		_time_good_motion_us = _imu_sample_delayed.time_us;
	}
}

void Ekf::resetOnGroundMotionForOpticalFlowChecks()
{
	_time_bad_motion_us = 0;
	_time_good_motion_us = _imu_sample_delayed.time_us;
}

void Ekf::controlGpsYawFusion(bool gps_checks_passing, bool gps_checks_failing)
{
	if (!(_params.fusion_mode & SensorFusionMask::USE_GPS_YAW)
	    || _control_status.flags.gps_yaw_fault) {

		stopGpsYawFusion();
		return;
	}

	const bool is_new_data_available = PX4_ISFINITE(_gps_sample_delayed.yaw);

	if (is_new_data_available) {

		const bool continuing_conditions_passing = !gps_checks_failing;

		const bool is_gps_yaw_data_intermittent = !isRecent(_time_last_gps_yaw_data, 2 * GPS_MAX_INTERVAL);

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _control_status.flags.tilt_align
				&& gps_checks_passing
				&& !is_gps_yaw_data_intermittent
				&& !_gps_intermittent;

		_time_last_gps_yaw_data = _time_last_imu;

		if (_control_status.flags.gps_yaw) {

			if (continuing_conditions_passing) {

				fuseGpsYaw();

				const bool is_fusion_failing = isTimedOut(_time_last_gps_yaw_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (_nb_gps_yaw_reset_available > 0) {
						// Data seems good, attempt a reset
						resetYawToGps();

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
				startGpsYawFusion();

				if (_control_status.flags.gps_yaw) {
					_nb_gps_yaw_reset_available = 1;
				}
			}
		}

	} else if (_control_status.flags.gps_yaw && isTimedOut(_time_last_gps_yaw_data, _params.reset_timeout_max)) {
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

void Ekf::controlHeightSensorTimeouts()
{
	/*
	 * Handle the case where we have not fused height measurements recently and
	 * uncertainty exceeds the max allowable. Reset using the best available height
	 * measurement source, continue using it after the reset and declare the current
	 * source failed if we have switched.
	*/

	checkVerticalAccelerationHealth();

	// check if height is continuously failing because of accel errors
	const bool continuous_bad_accel_hgt = isTimedOut(_time_good_vert_accel, (uint64_t)_params.bad_acc_reset_delay_us);

	// check if height has been inertial deadreckoning for too long
	// in vision hgt mode check for vision data
	const bool hgt_fusion_timeout = isTimedOut(_time_last_hgt_fuse, (uint64_t)5e6);

	if (hgt_fusion_timeout || continuous_bad_accel_hgt) {

		const char *failing_height_source = nullptr;
		const char *new_height_source = nullptr;

		if (_control_status.flags.baro_hgt) {
			bool reset_to_gps = false;

			// reset to GPS if adequate GPS data is available and the timeout cannot be blamed on IMU data
			if (!_gps_intermittent) {
				reset_to_gps = (_gps_checks_passed && !_fault_status.flags.bad_acc_vertical) || _baro_hgt_faulty || _baro_hgt_intermittent;
			}

			if (reset_to_gps) {
				// set height sensor health
				_baro_hgt_faulty = true;

				startGpsHgtFusion();

				failing_height_source = "baro";
				new_height_source = "gps";

			} else if (!_baro_hgt_faulty && !_baro_hgt_intermittent) {
				resetHeightToBaro();

				failing_height_source = "baro";
				new_height_source = "baro";
			}

		} else if (_control_status.flags.gps_hgt) {
			bool reset_to_baro = false;

			// if baro data is available and GPS data is inaccurate and the timeout cannot be blamed on IMU data, reset height to baro
			if (!_baro_hgt_faulty && !_baro_hgt_intermittent) {
				reset_to_baro = (!_fault_status.flags.bad_acc_vertical && !_gps_checks_passed) || _gps_intermittent;
			}

			if (reset_to_baro) {
				startBaroHgtFusion();

				failing_height_source = "gps";
				new_height_source = "baro";

			} else if (!_gps_intermittent) {
				resetHeightToGps();

				failing_height_source = "gps";
				new_height_source = "gps";
			}

		} else if (_control_status.flags.rng_hgt) {

			if (_range_sensor.isHealthy()) {
				resetHeightToRng();

				failing_height_source = "rng";
				new_height_source = "rng";

			} else if (!_baro_hgt_faulty && !_baro_hgt_intermittent) {
				startBaroHgtFusion();

				failing_height_source = "rng";
				new_height_source = "baro";
			}

		} else if (_control_status.flags.ev_hgt) {
			// check if vision data is available
			bool ev_data_available = false;

			if (_ext_vision_buffer) {
				const extVisionSample &ev_init = _ext_vision_buffer->get_newest();
				ev_data_available = isRecent(ev_init.time_us, 2 * EV_MAX_INTERVAL);
			}

			if (ev_data_available) {
				resetHeightToEv();

				failing_height_source = "ev";
				new_height_source = "ev";

			} else if (_range_sensor.isHealthy()) {
				// Fallback to rangefinder data if available
				startRngHgtFusion();

				failing_height_source = "ev";
				new_height_source = "rng";

			} else if (!_baro_hgt_faulty && !_baro_hgt_intermittent) {
				startBaroHgtFusion();

				failing_height_source = "ev";
				new_height_source = "baro";
			}
		}

		if (failing_height_source && new_height_source) {
			_warning_events.flags.height_sensor_timeout = true;
			ECL_WARN("%s hgt timeout - reset to %s", failing_height_source, new_height_source);
		}

		// Also reset the vertical velocity
		if (_control_status.flags.gps && !_gps_intermittent && _gps_checks_passed) {
			resetVerticalVelocityToGps(_gps_sample_delayed);

		} else {
			resetVerticalVelocityToZero();
		}
	}
}

void Ekf::checkVerticalAccelerationHealth()
{
	// Check for IMU accelerometer vibration induced clipping as evidenced by the vertical
	// innovations being positive and not stale.
	// Clipping usually causes the average accel reading to move towards zero which makes the INS
	// think it is falling and produces positive vertical innovations.
	// Don't use stale innovation data.
	bool is_inertial_nav_falling = false;
	bool are_vertical_pos_and_vel_independant = false;

	if (_control_status.flags.gps) {
		// GNSS velocity
		const auto &gps_vel = _aid_src_gnss_vel;

		if (gps_vel.time_last_fuse[2] > _vert_vel_fuse_time_us) {
			_vert_vel_fuse_time_us = gps_vel.time_last_fuse[2];
			_vert_vel_innov_ratio = gps_vel.innovation[2] / sqrtf(gps_vel.innovation_variance[2]);
		}

		// GNSS position
		const auto &gps_pos = _aid_src_gnss_pos;

		if (gps_pos.time_last_fuse[2] > _vert_pos_fuse_attempt_time_us) {
			_vert_pos_fuse_attempt_time_us = gps_pos.time_last_fuse[2];
			_vert_pos_innov_ratio = gps_pos.innovation[2] / sqrtf(gps_pos.innovation_variance[2]);
		}
	}

	if (_control_status.flags.baro_hgt) {
		// baro height
		const auto &baro_hgt = _aid_src_baro_hgt;

		if (baro_hgt.time_last_fuse > _vert_pos_fuse_attempt_time_us) {
			_vert_pos_fuse_attempt_time_us = baro_hgt.time_last_fuse;
			_vert_pos_innov_ratio = baro_hgt.innovation / sqrtf(baro_hgt.innovation_variance);
		}
	}

	if (isRecent(_vert_pos_fuse_attempt_time_us, 1000000)) {
		if (isRecent(_vert_vel_fuse_time_us, 1000000)) {
			// If vertical position and velocity come from independent sensors then we can
			// trust them more if they disagree with the IMU, but need to check that they agree
			const bool using_gps_for_both = _control_status.flags.gps_hgt && _control_status.flags.gps;
			const bool using_ev_for_both = _control_status.flags.ev_hgt && _control_status.flags.ev_vel;
			are_vertical_pos_and_vel_independant = !(using_gps_for_both || using_ev_for_both);
			is_inertial_nav_falling |= _vert_vel_innov_ratio > _params.vert_innov_test_lim && _vert_pos_innov_ratio > _params.vert_innov_test_min;
			is_inertial_nav_falling |= _vert_pos_innov_ratio > _params.vert_innov_test_lim && _vert_vel_innov_ratio > _params.vert_innov_test_min;

		} else {
			// only height sensing available
			is_inertial_nav_falling = _vert_pos_innov_ratio > _params.vert_innov_test_lim;
		}
	}

	// Check for more than 50% clipping affected IMU samples within the past 1 second
	const uint16_t clip_count_limit = 1.f / _dt_ekf_avg;
	const bool is_clipping = _imu_sample_delayed.delta_vel_clipping[0] ||
				 _imu_sample_delayed.delta_vel_clipping[1] ||
				 _imu_sample_delayed.delta_vel_clipping[2];

	if (is_clipping && _clip_counter < clip_count_limit) {
		_clip_counter++;

	} else if (_clip_counter > 0) {
		_clip_counter--;
	}

	_fault_status.flags.bad_acc_clipping = _clip_counter > clip_count_limit / 2;

	const bool is_clipping_frequently = _clip_counter > 0;

	// if vertical velocity and position are independent and agree, then do not require evidence of clipping if
	// innovations are large
	const bool bad_vert_accel = (are_vertical_pos_and_vel_independant || is_clipping_frequently) && is_inertial_nav_falling;

	if (bad_vert_accel) {
		_time_bad_vert_accel = _time_last_imu;

	} else {
		_time_good_vert_accel = _time_last_imu;
	}

	// declare a bad vertical acceleration measurement and make the declaration persist
	// for a minimum of BADACC_PROBATION seconds
	if (_fault_status.flags.bad_acc_vertical) {
		_fault_status.flags.bad_acc_vertical = isRecent(_time_bad_vert_accel, BADACC_PROBATION);

	} else {
		_fault_status.flags.bad_acc_vertical = bad_vert_accel;
	}
}

void Ekf::controlHeightFusion()
{
	checkRangeAidSuitability();
	const bool do_range_aid = (_params.range_aid == 1) && _is_range_aid_suitable;

	switch (_params.vdist_sensor_type) {
	default:
		ECL_ERR("Invalid hgt mode: %" PRIi32, _params.vdist_sensor_type);

	// FALLTHROUGH
	case VerticalHeightSensor::BARO:
		if (do_range_aid) {
			if (!_control_status.flags.rng_hgt && _range_sensor.isDataHealthy()) {
				startRngAidHgtFusion();
			}

		} else {
			if (!_control_status.flags.baro_hgt) {
				if (!_baro_hgt_faulty && !_baro_hgt_intermittent) {
					startBaroHgtFusion();

				} else if (!_control_status.flags.gps_hgt && !_gps_intermittent && _gps_checks_passed) {
					// Use GPS as a fallback
					startGpsHgtFusion();
				}
			}
		}

		break;

	case VerticalHeightSensor::RANGE:

		// If we are supposed to be using range finder data as the primary height sensor, have bad range measurements
		// and are on the ground, then synthesise a measurement at the expected on ground value
		if (!_control_status.flags.in_air
		    && !_range_sensor.isDataHealthy()
		    && _range_sensor.isRegularlySendingData()
		    && _range_sensor.isDataReady()) {

			_range_sensor.setRange(_params.rng_gnd_clearance);
			_range_sensor.setValidity(true); // bypass the checks
		}

		if (!_control_status.flags.rng_hgt) {
			if (_range_sensor.isDataHealthy()) {
				startRngHgtFusion();
			}
		}

		break;

	case VerticalHeightSensor::GPS:

		// NOTE: emergency fallback due to extended loss of currently selected sensor data or failure
		// to pass innovation cinsistency checks is handled elsewhere in Ekf::controlHeightSensorTimeouts.
		// Do switching between GPS and rangefinder if using range finder as a height source when close
		// to ground and moving slowly. Also handle switch back from emergency Baro sensor when GPS recovers.
		if (do_range_aid) {
			if (!_control_status_prev.flags.rng_hgt && _range_sensor.isDataHealthy()) {
				startRngAidHgtFusion();
			}

		} else {
			if (!_control_status.flags.gps_hgt) {
				if (!_gps_intermittent && _gps_checks_passed) {
					// In fallback mode and GPS has recovered so start using it
					startGpsHgtFusion();

				} else if (!_control_status.flags.baro_hgt && !_baro_hgt_faulty && !_baro_hgt_intermittent) {
					// Use baro as a fallback
					startBaroHgtFusion();
				}
			}
		}

		break;

	case VerticalHeightSensor::EV:

		// don't start using EV data unless data is arriving frequently
		if (!_control_status.flags.ev_hgt && isRecent(_time_last_ext_vision, 2 * EV_MAX_INTERVAL)) {
			startEvHgtFusion();
		}

		break;
	}

	updateBaroHgtBias();
	updateBaroHgtOffset();
	updateGroundEffect();

	if (_baro_data_ready) {
		updateBaroHgt(_baro_sample_delayed, _aid_src_baro_hgt);

		if (_control_status.flags.baro_hgt && !_baro_hgt_faulty) {
			fuseBaroHgt(_aid_src_baro_hgt);
		}
	}

	if (_rng_data_ready) {
		updateRngHgt(_aid_src_rng_hgt);

		if (_control_status.flags.rng_hgt && _range_sensor.isDataHealthy()) {
			fuseRngHgt(_aid_src_rng_hgt);
		}
	}

	if (_control_status.flags.ev_hgt) {

		if (_control_status.flags.ev_hgt && _ev_data_ready) {
			fuseEvHgt();
		}
	}
}

void Ekf::checkRangeAidSuitability()
{
	if (_control_status.flags.in_air
	    && _range_sensor.isHealthy()
	    && isTerrainEstimateValid()) {
		// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
		// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
		const float range_hagl = _terrain_vpos - _state.pos(2);
		const float range_hagl_max = _is_range_aid_suitable ? _params.max_hagl_for_range_aid : (_params.max_hagl_for_range_aid * 0.7f);
		const bool is_in_range = range_hagl < range_hagl_max;

		const float hagl_test_ratio = (_hagl_innov * _hagl_innov / (sq(_params.range_aid_innov_gate) * _hagl_innov_var));
		const bool is_hagl_stable = _is_range_aid_suitable ? (hagl_test_ratio < 1.f) : (hagl_test_ratio < 0.01f);

		if (isHorizontalAidingActive()) {
			const float max_vel = _is_range_aid_suitable ? _params.max_vel_for_range_aid : (_params.max_vel_for_range_aid * 0.7f);
			const bool is_below_max_speed = !_state.vel.xy().longerThan(max_vel);

			_is_range_aid_suitable = is_in_range && is_hagl_stable && is_below_max_speed;

		} else {
			_is_range_aid_suitable = is_in_range && is_hagl_stable;
		}

	} else {
		_is_range_aid_suitable = false;
	}
}

void Ekf::controlAirDataFusion()
{
	// control activation and initialisation/reset of wind states required for airspeed fusion

	// If both airspeed and sideslip fusion have timed out and we are not using a drag observation model then we no longer have valid wind estimates
	const bool airspeed_timed_out = isTimedOut(_time_last_arsp_fuse, (uint64_t)10e6);
	const bool sideslip_timed_out = isTimedOut(_time_last_beta_fuse, (uint64_t)10e6);

	if (_using_synthetic_position || (airspeed_timed_out && sideslip_timed_out && !(_params.fusion_mode & SensorFusionMask::USE_DRAG))) {
		_control_status.flags.wind = false;
	}

	if (_params.arsp_thr <= 0.f) {
		stopAirspeedFusion();
		return;
	}

	if (_tas_data_ready) {
		const bool continuing_conditions_passing = _control_status.flags.in_air && _control_status.flags.fixed_wing && !_using_synthetic_position;
		const bool is_airspeed_significant = _airspeed_sample_delayed.true_airspeed > _params.arsp_thr;
		const bool starting_conditions_passing = continuing_conditions_passing && is_airspeed_significant;

		if (_control_status.flags.fuse_aspd) {
			if (continuing_conditions_passing) {
				if (is_airspeed_significant) {
					fuseAirspeed();
				}

				const bool is_fusion_failing = isTimedOut(_time_last_arsp_fuse, (uint64_t)10e6);

				if (is_fusion_failing) {
					stopAirspeedFusion();
				}

			} else {
				stopAirspeedFusion();
			}

		} else if (starting_conditions_passing) {
			startAirspeedFusion();
		}

	} else if (_control_status.flags.fuse_aspd && (_imu_sample_delayed.time_us - _airspeed_sample_delayed.time_us > (uint64_t) 1e6)) {
		ECL_WARN("Airspeed data stopped");
		stopAirspeedFusion();
	}
}

void Ekf::controlBetaFusion()
{
	if (_using_synthetic_position) {
		return;
	}

	// Perform synthetic sideslip fusion at regular intervals when in-air and sideslip fuson had been enabled externally:
	const bool beta_fusion_time_triggered = isTimedOut(_time_last_beta_fuse, _params.beta_avg_ft_us);

	if (beta_fusion_time_triggered &&
	    _control_status.flags.fuse_beta &&
	    _control_status.flags.in_air) {
		// If starting wind state estimation, reset the wind states and covariances before fusing any data
		if (!_control_status.flags.wind) {
			// activate the wind states
			_control_status.flags.wind = true;
			// reset the timeout timers to prevent repeated resets
			_time_last_beta_fuse = _time_last_imu;
			resetWind();
		}

		fuseSideslip();
	}
}

void Ekf::controlDragFusion()
{
	if ((_params.fusion_mode & SensorFusionMask::USE_DRAG) && _drag_buffer &&
	    !_using_synthetic_position && _control_status.flags.in_air && !_mag_inhibit_yaw_reset_req) {

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

			if (isHorizontalAidingActive()) {

				const float aux_vel_innov_gate = fmaxf(_params.auxvel_gate, 1.f);

				_aux_vel_innov = _state.vel - auxvel_sample_delayed.vel;

				fuseHorizontalVelocity(_aux_vel_innov, aux_vel_innov_gate, auxvel_sample_delayed.velVar,
						       _aux_vel_innov_var, _aux_vel_test_ratio);

				// Can be enabled after bit for this is added to EKF_AID_MASK
				// fuseVerticalVelocity(_aux_vel_innov, aux_vel_innov_gate, auxvel_sample_delayed.velVar,
				//		_aux_vel_innov_var, _aux_vel_test_ratio);
			}
		}
	}
}

bool Ekf::hasHorizontalAidingTimedOut() const
{
	return isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
	       && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
	       && isTimedOut(_time_last_of_fuse, _params.reset_timeout_max);
}
