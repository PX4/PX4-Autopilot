/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file VTEPosition.cpp
 * @brief Estimate the state of a target by processing and fusing sensor data in a Kalman Filter.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <px4_platform_common/defines.h>

#include <cmath>
#include <cstdint>

#include <lib/geo/geo.h>

#include "VTEPosition.h"

namespace vision_target_estimator
{

using namespace matrix;

static_assert(vtest::Axis::size == 3, "one position filter per NED axis");
#if defined(CONFIG_VTEST_MOVING)
static_assert(vtest::State::size == 5, "moving target state: pos_rel, vel_uav, bias, acc_target, vel_target");
#else
static_assert(vtest::State::size == 3, "static target state: pos_rel, vel_uav, bias");
#endif // CONFIG_VTEST_MOVING

VTEPosition::VTEPosition() :
	ModuleParams(nullptr)
{
	_target_pose_pub.advertise();
	_target_estimator_state_pub.advertise();
	_vte_bias_init_status_pub.advertise();

	for (auto &pub : _vte_aid_pubs) {
		pub.advertise();
	}

	updateParams();
}

VTEPosition::~VTEPosition()
{
	perf_free(_vte_predict_perf);
	perf_free(_vte_update_perf);
	perf_free(_vte_fusion_perf);
}

void VTEPosition::resetFilter(hrt_abstime min_sample_time)
{
	// Keep the frame boundary across subsequent timeout/task resets.
	if (min_sample_time > _min_sample_time) {
		_min_sample_time = min_sample_time;
	}

	if (min_sample_time != 0) {
		// These cached values use the old frame, even if published after the reset's sample time.
		_local_position.valid = false;
		_local_velocity.valid = false;
		_pos_rel_gnss.valid = false;
		_gps_pos_offset_ned.valid = false;
		_velocity_offset_ned.valid = false;
	}

	if (_estimator_initialized) {
		publishInvalidState();
	}

	_estimator_initialized = false;
	_last_relative_meas_fused_time = 0;
	_bias.set = false;
	_pre_bias_reference = PreBiasReference::kUnknown;
	resetBiasAveraging();

	for (int axis = 0; axis < vtest::Axis::size; ++axis) {
		_target_est_pos[axis].resetHistory();
	}
}

void VTEPosition::update(const Vector3f &acc_ned)
{
	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	checkMeasurementInputs();

	// Predict the target state using a constant relative-acceleration model.
	if (_estimator_initialized) {
		const hrt_abstime now_us = nowUs();

		if (now_us < _last_predict) {
			resetFilter();
			PX4_WARN("VTE position invalid _last_predict time");
			return;
		}

		const uint64_t dt_us = now_us - _last_predict;

		// Guard against large dt values for which the constant acceleration assumption does not hold.
		static constexpr uint64_t kMaxPredictionDeltaTimeUs = 100_ms; // Filter runs at 50 Hz, expect dt = 20 ms.

		if (dt_us > kMaxPredictionDeltaTimeUs) {
			resetFilter();
			PX4_WARN("VTE position stale, resetting filter");
			return;
		}

		const float dt = dt_us * kMicrosecondsToSeconds;
		perf_begin(_vte_predict_perf);
		predictionStep(acc_ned, dt);
		perf_end(_vte_predict_perf);

		_last_predict = now_us;
	}

	// Update with the newest observations, fuse if the filter is initialised,
	// and publish any resulting innovations.
	performUpdateStep();

	if (_estimator_initialized) {
		// Store the post-fusion posterior snapshot at the current timestamp for OOSM fusion.
		for (int axis = 0; axis < vtest::Axis::size; ++axis) {
			_target_est_pos[axis].pushHistory(_last_predict);
		}

		publishTarget();
	}
}

bool VTEPosition::initEstimator(const AxisEstimatorStates &state_init, const Vector3f &position_variance)
{
	// Get initial variance from params
	const float state_pos_var = fmaxf(_param_vte_pos_unc_in.get(), kMinVariance);
	const float state_vel_var = fmaxf(_param_vte_vel_unc_in.get(), kMinVariance);
	const float state_bias_var = fmaxf(_param_vte_bias_unc_in.get(), kMinVariance);

	EstimatorState state_var_init{};
	state_var_init(vtest::State::vel_uav) = state_vel_var;
	state_var_init(vtest::State::bias) = state_bias_var;

#if defined(CONFIG_VTEST_MOVING)
	const float state_acc_var = fmaxf(_param_vte_acc_unc_in.get(), kMinVariance);
	const float state_target_vel_var = fmaxf(_param_vte_vel_unc_in.get(), kMinVariance);
	state_var_init(vtest::State::acc_target) = state_acc_var;
	state_var_init(vtest::State::vel_target) = state_target_vel_var;

#endif // CONFIG_VTEST_MOVING

	for (int i = 0; i < vtest::Axis::size; i++) {
		state_var_init(vtest::State::pos_rel) = fmaxf(state_pos_var, position_variance(i));
		_target_est_pos[i].setState(state_init[i]);
		_target_est_pos[i].setStateCovarianceDiag(state_var_init);
	}

	// Debug INFO
	PX4_DEBUG("Rel pos init %.2f %.2f %.2f", (double)state_init[vtest::Axis::x](vtest::State::pos_rel),
		  (double)state_init[vtest::Axis::y](vtest::State::pos_rel),
		  (double)state_init[vtest::Axis::z](vtest::State::pos_rel));
	PX4_DEBUG("Vel uav init %.2f %.2f %.2f", (double)state_init[vtest::Axis::x](vtest::State::vel_uav),
		  (double)state_init[vtest::Axis::y](vtest::State::vel_uav),
		  (double)state_init[vtest::Axis::z](vtest::State::vel_uav));
	PX4_DEBUG("GNSS bias init %.2f %.2f %.2f", (double)state_init[vtest::Axis::x](vtest::State::bias),
		  (double)state_init[vtest::Axis::y](vtest::State::bias),
		  (double)state_init[vtest::Axis::z](vtest::State::bias));

#if defined(CONFIG_VTEST_MOVING)
	PX4_DEBUG("Target acc init %.2f %.2f %.2f", (double)state_init[vtest::Axis::x](vtest::State::acc_target),
		  (double)state_init[vtest::Axis::y](vtest::State::acc_target),
		  (double)state_init[vtest::Axis::z](vtest::State::acc_target));
	PX4_DEBUG("Target vel init %.2f %.2f %.2f", (double)state_init[vtest::Axis::x](vtest::State::vel_target),
		  (double)state_init[vtest::Axis::y](vtest::State::vel_target),
		  (double)state_init[vtest::Axis::z](vtest::State::vel_target));
#endif // CONFIG_VTEST_MOVING

	return true;
}

void VTEPosition::predictionStep(const Vector3f &vehicle_acc_ned, const float dt)
{
	const float bias_var = fmaxf(_param_vte_bias_unc.get(), kMinVariance);
	const float uav_acc_var = fmaxf(_param_vte_acc_d_unc.get(), kMinVariance);
#if defined(CONFIG_VTEST_MOVING)
	const float target_acc_var = fmaxf(_param_vte_acc_t_unc.get(), kMinVariance);
#endif // CONFIG_VTEST_MOVING

	//Decoupled dynamics, we neglect the off diag elements.
	for (int i = 0; i < vtest::Axis::size; i++) {
#if defined(CONFIG_VTEST_MOVING)
		_target_est_pos[i].setTargetAccVar(target_acc_var);
#endif // CONFIG_VTEST_MOVING

		// We assume an isotropic uncertainty (cov = sigma^2 * I)
		// The rotated input covariance from body to NED R*(sigma^2 * I)*R^T = sigma^2*I because R*R^T = I.
		_target_est_pos[i].setBiasVar(bias_var);
		_target_est_pos[i].setInputVar(uav_acc_var);

		_target_est_pos[i].predict(dt, vehicle_acc_ned(i));
	}
}

void VTEPosition::performUpdateStep()
{
	// Gather observations and tag the fusion mask bits for each new valid source.
	resetObservations();
	ObsValidMaskU fusion_mask{};
	processObservations(fusion_mask, _obs_buffer);

	if (fusion_mask.value == 0) {
		return;
	}

	if (!_estimator_initialized) {
		if (hasNewPositionSensorData(fusion_mask)) {
			// Initialize only this cycle to avoid using the same measurements twice.
			initializeEstimator(fusion_mask, _obs_buffer);
		}

		return;
	}

	if (!_bias.set && updateBiasIfObservable(fusion_mask, _obs_buffer)) {
		// The vision sample is the state now and is not fused again, as at init.
		const TargetObs &vision_obs = _obs_buffer[obsIndex(ObsType::kFiducialMarker)];
		fusion_mask.flags.fuse_vision = false;
		publishInitializedAid(vision_obs);
	}

	fuseActiveMeasurements(fusion_mask, _obs_buffer);
}

void VTEPosition::resetObservations()
{
	for (auto &obs : _obs_buffer) {
		obs = {};
	}
}

void VTEPosition::processObservations(ObsValidMaskU &fusion_mask,
				      TargetObs observations[kObsTypeCount])
{
	if (_vte_aid_mask.flags.use_vision_pos) {
		fusion_mask.flags.fuse_vision = processObsVision(observations[obsIndex(ObsType::kFiducialMarker)]);
	}

	if (updateUavGnssData()) {
		if (_vte_aid_mask.flags.use_mission_pos && _mission_land_position.valid && _uav_gps_position.valid) {
			fusion_mask.flags.fuse_mission_pos = processObsGNSSPosMission(observations[obsIndex(ObsType::kMissionGpsPos)]);
		}

		if (_vte_aid_mask.flags.use_uav_gps_vel && _uav_gps_vel.valid) {
			fusion_mask.flags.fuse_uav_gps_vel = processObsGNSSVelUav(observations[obsIndex(ObsType::kUavGpsVel)]);
		}
	}

	target_gnss_s target_gnss{};

	if (pollTargetGnss(target_gnss)) {
		const bool target_gps_position_valid = isTargetGpsPositionValid(target_gnss);

		if (_vte_aid_mask.flags.use_target_gps_pos && _uav_gps_position.valid
		    && target_gps_position_valid && target_gnss.abs_pos_updated) {
			fusion_mask.flags.fuse_target_gps_pos = processObsGNSSPosTarget(target_gnss,
								observations[obsIndex(ObsType::kTargetGpsPos)]);
		}

#if defined(CONFIG_VTEST_MOVING)

		if (isTargetGpsVelocityValid(target_gnss)) {

			updateTargetGpsVelocity(target_gnss);

			if (_vte_aid_mask.flags.use_target_gps_vel && _target_gps_vel.valid) {
				fusion_mask.flags.fuse_target_gps_vel = processObsGNSSVelTarget(target_gnss,
									observations[obsIndex(ObsType::kTargetGpsVel)]);
			}
		}

#endif // CONFIG_VTEST_MOVING

	}
}

bool VTEPosition::isVisionDataValid(const fiducial_marker_pos_report_s &fiducial_marker_pose)
{
	if (!isMeasRecent(fiducial_marker_pose.timestamp_sample)) {
		if (shouldEmitWarning(_vision_pos_warn_last)) {
			PX4_WARN("VTE: Vision data too old!");
		}

		return false;
	}

	const bool finite_measurement = Vector3f(fiducial_marker_pose.rel_pos).isAllFinite();

	if (!finite_measurement) {
		if (shouldEmitWarning(_vision_pos_warn_last)) {
			PX4_WARN("VTE: Vision meas is corrupt!");
		}

		return false;
	}

	const bool finite_cov = Vector3f(fiducial_marker_pose.cov_rel_pos).isAllFinite();

	if (!finite_cov) {
		if (shouldEmitWarning(_vision_pos_warn_last)) {
			PX4_WARN("VTE: Vision cov is corrupt!");
		}

		return false;
	}

	// Rotate vision observation into vc-NED using the reported attitude quaternion
	const bool finite_q = PX4_ISFINITE(fiducial_marker_pose.q[0])
			      && PX4_ISFINITE(fiducial_marker_pose.q[1])
			      && PX4_ISFINITE(fiducial_marker_pose.q[2])
			      && PX4_ISFINITE(fiducial_marker_pose.q[3]);

	if (!finite_q) {
		if (shouldEmitWarning(_vision_pos_warn_last)) {
			PX4_WARN("VTE: Vision attitude quaternion is corrupt!");
		}

		return false;
	}

	return true;
}

bool VTEPosition::updateUavGnssData()
{
	vehicle_gnss_s vehicle_gnss;
	const bool vehicle_gnss_updated = _vehicle_gnss_sub.update(&vehicle_gnss);

	if (vehicle_gnss_updated) {
		// Cache receiver validity independently of lever-arm corrections, which are checked when used.
		const bool sample_usable = vehicle_gnss.usable && isMeasRecent(vehicle_gnss.timestamp_sample);

		// Position
		_uav_gps_position.lat_deg = vehicle_gnss.receiver.latitude;
		_uav_gps_position.lon_deg = vehicle_gnss.receiver.longitude;
		_uav_gps_position.alt_m = (float)vehicle_gnss.receiver.altitude_msl;
		_uav_gps_position.timestamp = vehicle_gnss.timestamp_sample;
		_uav_gps_position.eph = vehicle_gnss.receiver.eph;
		_uav_gps_position.epv = vehicle_gnss.receiver.epv;
		_uav_gps_position.valid = sample_usable
					  && PX4_ISFINITE(_uav_gps_position.eph) && _uav_gps_position.eph >= 0.f
					  && PX4_ISFINITE(_uav_gps_position.epv) && _uav_gps_position.epv >= 0.f
					  && isLatLonAltValid(_uav_gps_position.lat_deg, _uav_gps_position.lon_deg,
							  _uav_gps_position.alt_m, "UAV GPS ", &_uav_gps_pos_warn_last);

		// Velocity
		_uav_gps_vel.timestamp = vehicle_gnss.timestamp_sample;
		_uav_gps_vel.xyz(vtest::Axis::x) = vehicle_gnss.receiver.vel_north;
		_uav_gps_vel.xyz(vtest::Axis::y) = vehicle_gnss.receiver.vel_east;
		_uav_gps_vel.xyz(vtest::Axis::z) = vehicle_gnss.receiver.vel_down;
		_uav_gps_vel.uncertainty = vehicle_gnss.receiver.speed_accuracy;
		_uav_gps_vel.valid = sample_usable && vehicle_gnss.receiver.vel_ned_valid
				     && PX4_ISFINITE(_uav_gps_vel.uncertainty) && _uav_gps_vel.uncertainty >= 0.f;

		if (_uav_gps_vel.valid && !_uav_gps_vel.xyz.isAllFinite()) {
			if (shouldEmitWarning(_uav_gps_vel_warn_last)) {
				PX4_WARN("UAV GPS velocity not finite! vx: %.1f, vy: %.1f, vz: %.1f",
					 (double)_uav_gps_vel.xyz(vtest::Axis::x), (double)_uav_gps_vel.xyz(vtest::Axis::y),
					 (double)_uav_gps_vel.xyz(vtest::Axis::z));
			}

			_uav_gps_vel.valid = false;
		}

	} else {
		// Check if stored data is still valid
		_uav_gps_position.valid = _uav_gps_position.valid && isMeasRecent(_uav_gps_position.timestamp);
		_uav_gps_vel.valid = _uav_gps_vel.valid && isMeasRecent(_uav_gps_vel.timestamp);
	}

	return vehicle_gnss_updated;
}

#if defined(CONFIG_VTEST_MOVING)
void VTEPosition::updateTargetGpsVelocity(const target_gnss_s &target_gnss)
{
	_target_gps_vel.timestamp = target_gnss.timestamp_sample;
	_target_gps_vel.valid = isMeasRecent(target_gnss.timestamp_sample);

	_target_gps_vel.xyz(vtest::Axis::x) = target_gnss.vel_n_m_s;
	_target_gps_vel.xyz(vtest::Axis::y) = target_gnss.vel_e_m_s;
	_target_gps_vel.xyz(vtest::Axis::z) = target_gnss.vel_d_m_s;
}
#endif // CONFIG_VTEST_MOVING

bool VTEPosition::pollTargetGnss(target_gnss_s &target_gnss)
{
	if (!_target_gnss_sub.update(&target_gnss)) {
		return false;
	}

	if (!alignFutureSampleTime(target_gnss.timestamp_sample, nowUs())) {
		if (shouldEmitWarning(_target_gps_pos_warn_last)) {
			PX4_WARN("VTE: Target GNSS timestamp in the future!");
		}

		return false;
	}

	return true;
}

bool VTEPosition::isTargetGpsPositionValid(const target_gnss_s &target_gnss)
{
	if (!isMeasRecent(target_gnss.timestamp_sample)) {
		return false;
	}

	if (!isLatLonAltValid(target_gnss.latitude_deg, target_gnss.longitude_deg,
			      target_gnss.altitude_msl_m, "Target GPS ", &_target_gps_pos_warn_last)) {
		return false;
	}

	return true;
}

bool VTEPosition::isTargetGpsVelocityValid(const target_gnss_s &target_gnss)
{
	if (!isMeasRecent(target_gnss.timestamp_sample)) {
		return false;
	}

	const Vector3f target_vel_ned{target_gnss.vel_n_m_s, target_gnss.vel_e_m_s, target_gnss.vel_d_m_s};

	if (!target_gnss.vel_ned_updated || !target_vel_ned.isAllFinite()) {
		if (shouldEmitWarning(_target_gps_vel_warn_last)) {
			PX4_WARN("Target GPS velocity is corrupt!");
		}

		return false;
	}

	return true;
}

bool VTEPosition::initializeEstimator(const ObsValidMaskU &fusion_mask,
				      const TargetObs observations[kObsTypeCount])
{
	if (!hasNewPositionSensorData(fusion_mask)) {
		return false;
	}

	// Prefer GNSS velocity corrected to the center of mass, with local velocity as fallback.
	Vector3f initial_uav_velocity{};

	if (!getOffsetUavVel(_uav_gps_vel.timestamp, initial_uav_velocity)) {
		if (!_local_velocity.valid || !isMeasRecent(_local_velocity.timestamp)) {
			if (shouldEmitWarning(_init_vel_warn_last)) {
				PX4_WARN("No UAV velocity estimate. Estimator cannot be started.");
			}

			return false;
		}

		initial_uav_velocity = _local_velocity.xyz;
	}

	// If both GNSS and vision are available at startup, only keep the GNSS-first
	// path when bias averaging is explicitly enabled.
	// Otherwise, trust vision and activate the bias immediately.
	const bool has_vision_observation = fusion_mask.flags.fuse_vision;
	const bool can_average_initial_bias = shouldSetBias(has_vision_observation) && isBiasAveragingEnabled();

	ObsType initial_type = ObsType::kFiducialMarker;

	if (can_average_initial_bias || !has_vision_observation) {
		if (fusion_mask.flags.fuse_target_gps_pos) {
			initial_type = ObsType::kTargetGpsPos;

		} else if (fusion_mask.flags.fuse_mission_pos) {
			initial_type = ObsType::kMissionGpsPos;
		}
	}

	const TargetObs &initial_obs = observations[obsIndex(initial_type)];
	const Vector3f &initial_position = initial_obs.meas_xyz;
	const PreBiasReference initial_reference = initial_type == ObsType::kFiducialMarker
			? PreBiasReference::kVision : PreBiasReference::kGnss;
	const bool average_initial_bias = shouldAverageInitialBias(has_vision_observation, initial_reference);

	Vector3f initial_bias{};

	if (shouldSetBias(has_vision_observation) && !average_initial_bias) {
		selectInitialBiasFromGnss(observations, initial_position, initial_bias);
	}

	AxisEstimatorStates state_init{};
	buildInitialStateArray(initial_position, initial_uav_velocity, initial_bias, state_init);

	if (!initEstimator(state_init, initial_obs.meas_unc_xyz)) {
		resetFilter();
		return false;
	}

	PX4_DEBUG("VTE Position Estimator properly initialized.");
	_estimator_initialized = true;
	_pre_bias_reference = _bias.set ? PreBiasReference::kUnknown : initial_reference;
	_last_predict = nowUs();
	_last_update = initial_obs.timestamp;

	if (initial_type == ObsType::kFiducialMarker) {
		_last_relative_meas_fused_time = initial_obs.timestamp;
	}

	for (int axis = 0; axis < vtest::Axis::size; ++axis) {
		_target_est_pos[axis].resetHistory({_last_update, _last_relative_meas_fused_time});
		_target_est_pos[axis].pushHistory(initial_obs.timestamp);
	}

	// No acceleration history exists yet. Propagate with constant velocity and grow the
	// covariance with the normal process noise instead of labelling the raw sample as current.
	predictionStep(Vector3f{}, (_last_predict - initial_obs.timestamp) * kMicrosecondsToSeconds);
	publishInitializedAid(initial_obs);

	return true;
}

void VTEPosition::publishInitializedAid(const TargetObs &obs)
{
	// The sample became the state, so there is no innovation.
	vte_aid_source3d_s aid{};
	aid.timestamp = _last_predict;
	aid.timestamp_sample = obs.timestamp;
	aid.time_last_predict = _last_predict;
	aid.time_since_meas_ms = static_cast<float>(signedTimeDiffUs(_last_predict, obs.timestamp)) * kMicrosecondsToMilliseconds;

	for (int axis = 0; axis < vtest::Axis::size; ++axis) {
		aid.observation[axis] = obs.meas_xyz(axis);
		aid.observation_variance[axis] = obs.meas_unc_xyz(axis);
		aid.innovation[axis] = NAN;
		aid.innovation_variance[axis] = NAN;
		aid.test_ratio[axis] = NAN;
		aid.replay_test_ratio_max[axis] = -1.f;
		aid.fusion_status[axis] = FusionStatus::STATUS_INITIALIZED;
	}

	_vte_aid_pubs[obsIndex(obs.type)].publish(aid);
}

void VTEPosition::selectInitialBiasFromGnss(const TargetObs observations[kObsTypeCount],
		const Vector3f &initial_position, Vector3f &initial_bias)
{
	PX4_DEBUG("VTE Position setting GNSS bias.");
	const hrt_abstime sample_time = observations[obsIndex(ObsType::kFiducialMarker)].timestamp;
	Vector3f gnss_sample{};
	_bias.set = selectBiasGnssSample(sample_time, gnss_sample);

	if (_bias.set) {
		initial_bias = gnss_sample - initial_position;
	}
}

void VTEPosition::buildInitialStateArray(const Vector3f &initial_position,
		const Vector3f &initial_uav_velocity, const Vector3f &initial_bias,
		AxisEstimatorStates &state_init) const
{
#if defined(CONFIG_VTEST_MOVING)
	// Assume null target absolute acceleration; seed target velocity from GNSS when recent.
	Vector3f initial_target_velocity{};

	if (_target_gps_vel.valid && isMeasRecent(_target_gps_vel.timestamp)) {
		initial_target_velocity = _target_gps_vel.xyz;
	}

#endif // CONFIG_VTEST_MOVING

	for (int axis = 0; axis < vtest::Axis::size; ++axis) {
		state_init[axis](vtest::State::pos_rel) = initial_position(axis);
		state_init[axis](vtest::State::vel_uav) = initial_uav_velocity(axis);
		state_init[axis](vtest::State::bias) = initial_bias(axis);

#if defined(CONFIG_VTEST_MOVING)
		state_init[axis](vtest::State::acc_target) = 0.f;
		state_init[axis](vtest::State::vel_target) = initial_target_velocity(axis);
#endif // CONFIG_VTEST_MOVING
	}
}

bool VTEPosition::getOffsetUavVel(const hrt_abstime sample_time, Vector3f &uav_velocity_ned) const
{
	if (!_uav_gps_vel.valid || !isTimeDifferenceWithin(_uav_gps_vel.timestamp, sample_time, _meas_updated_timeout_us)) {
		return false;
	}

	uav_velocity_ned = _uav_gps_vel.xyz;

	if (_gps_pos_is_offset) {
		if (!_velocity_offset_ned.valid
		    || !isMeasUpdated(_velocity_offset_ned.timestamp)) {
			return false;
		}

		uav_velocity_ned -= _velocity_offset_ned.xyz;
	}

	return true;
}

bool VTEPosition::selectBiasGnssSample(const hrt_abstime sample_time, Vector3f &gnss_sample)
{
	// Velocity-propagation below compensates for the lag between the cached GNSS sample and
	// the requested vision sample, so the alignment gate uses recent-measurement
	// tolerance and not the tighter "updated" gate which can reject usable samples.
	if (!_pos_rel_gnss.valid || !isMeasRecent(_pos_rel_gnss.timestamp)
	    || !isTimeDifferenceWithin(_pos_rel_gnss.timestamp, sample_time, _meas_recent_timeout_us)) {
		return false;
	}

	gnss_sample = _pos_rel_gnss.xyz;

	const int64_t dt_us = signedTimeDiffUs(sample_time, _pos_rel_gnss.timestamp);

	if (dt_us != 0) {
		// Use the latest valid velocity for this constant-velocity alignment, even when vision
		// is older than the fast-input timeout.
		Vector3f uav_velocity_ned{};

		if (_local_velocity.valid) {
			uav_velocity_ned = _local_velocity.xyz;

		} else if (!getOffsetUavVel(_uav_gps_vel.timestamp, uav_velocity_ned)) {
			return false;
		}

		Vector3f relative_velocity_ned = -uav_velocity_ned;

#if defined(CONFIG_VTEST_MOVING)

		if (_pos_rel_gnss_is_target) {
			if (!_target_gps_vel.valid || !isMeasRecent(_target_gps_vel.timestamp)) {
				return false;
			}

			relative_velocity_ned += _target_gps_vel.xyz;
		}

#endif // CONFIG_VTEST_MOVING

		gnss_sample += relative_velocity_ned * (static_cast<float>(dt_us) * kMicrosecondsToSeconds);
	}

	return true;
}

bool VTEPosition::updateBiasIfObservable(const ObsValidMaskU &fusion_mask,
		const TargetObs observations[kObsTypeCount])
{
	if (!fusion_mask.flags.fuse_vision) {
		return false;
	}

	const TargetObs &vision_obs = observations[obsIndex(ObsType::kFiducialMarker)];
	const hrt_abstime sample_time = vision_obs.timestamp;
	const Vector3f &initial_position = vision_obs.meas_xyz;

	Vector3f gnss_sample{};

	if (!selectBiasGnssSample(sample_time, gnss_sample)) {
		if (_bias.averaging_active) {
			PX4_WARN("GNSS unavailable, ending bias averaging and switching to vision.");
			activateBiasEstimate(initial_position, _bias.initial_lpf.getState(), vision_obs, true);
			return true;
		}

		if (_pre_bias_reference == PreBiasReference::kGnss) {
			// The state was initialized GNSS-referenced, so vision fusion is currently
			// gated on the GNSS bias being observable. Without an aligned GNSS sample
			// we would never make progress and vision would be blocked indefinitely.
			// Activate the bias with a zero offset so the filter unblocks.
			PX4_WARN("GNSS bias not observable, setting bias to zero.");
			activateBiasEstimate(initial_position, Vector3f{}, vision_obs, true);
			return true;
		}

		return false;
	}

	const Vector3f initial_bias = gnss_sample - initial_position;

	if (!_bias.averaging_active && shouldAverageInitialBias(fusion_mask.flags.fuse_vision)) {
		PX4_INFO("GNSS/vision bias observable, averaging initial GNSS bias.");
		startBiasAveraging(initial_bias, sample_time);
		return false;
	}

	if (_bias.averaging_active) {
		if (!updateBiasAveraging(initial_bias, sample_time)) {
			return false;
		}

		// The averaged bias anchors the position to GNSS, so the vision sample is new information.
		PX4_INFO("GNSS bias averaging complete, setting position and bias.");
		const Vector3f filtered_bias = _bias.initial_lpf.getState();
		activateBiasEstimate(gnss_sample - filtered_bias, filtered_bias, vision_obs, false);
		return false;
	}

	PX4_INFO("GNSS/vision bias observable, setting position and bias.");
	activateBiasEstimate(initial_position, initial_bias, vision_obs, true);
	return true;
}

void VTEPosition::startBiasAveraging(const Vector3f &bias_sample, const hrt_abstime sample_time)
{
	_bias.initial_lpf.reset(bias_sample);
	_bias.averaging_active = true;
	_bias.stable_delta_count = 0;
	_bias.averaging_start_time = sample_time;
	_bias.last_sample_time = sample_time;

	publishBiasInitStatus(bias_sample, bias_sample, 0.f, sample_time);
}

bool VTEPosition::updateBiasAveraging(const Vector3f &bias_sample, const hrt_abstime sample_time)
{
	static constexpr unsigned kInitialBiasLpfMinTimeFactor{2};
	static constexpr uint8_t kRequiredStableBiasDeltas{5};

	if (!_bias.averaging_active) {
		return false;
	}

	// Ignore duplicate or out-of-order samples: the LPF's alpha was set from the previous
	// interval's dt, so reusing it here would re-apply the same time step and double-count
	// the measurement without reflecting any elapsed time.
	if (sample_time <= _bias.last_sample_time) {
		return false;
	}

	const Vector3f filtered_bias_before_update = _bias.initial_lpf.getState();
	const float bias_delta = (bias_sample - filtered_bias_before_update).norm();

	_bias.initial_lpf.update(bias_sample, sample_time - _bias.last_sample_time);
	_bias.last_sample_time = sample_time;

	const Vector3f filtered_bias_logged = _bias.initial_lpf.getState();
	publishBiasInitStatus(bias_sample, filtered_bias_logged, bias_delta, sample_time);

	if (bias_delta <= getBiasAveragingThreshold()) {
		if (_bias.stable_delta_count < UINT8_MAX) {
			++_bias.stable_delta_count;
		}

	} else {
		_bias.stable_delta_count = 0;
	}

	const hrt_abstime min_averaging_time_us = kInitialBiasLpfMinTimeFactor * kInitialBiasLpfTimeConstant;
	const bool min_time_elapsed = (sample_time >= _bias.averaging_start_time)
				      && ((sample_time - _bias.averaging_start_time) >= min_averaging_time_us);
	const bool stable = (_bias.stable_delta_count >= kRequiredStableBiasDeltas) && min_time_elapsed;
	const bool timed_out = (sample_time >= _bias.averaging_start_time)
			       && ((sample_time - _bias.averaging_start_time) >= getBiasAveragingTimeoutUs());

	return stable || timed_out;
}

void VTEPosition::resetBiasAveraging()
{
	_bias.averaging_active = false;
	_bias.stable_delta_count = 0;
	_bias.averaging_start_time = 0;
	_bias.last_sample_time = 0;
	_bias.initial_lpf.reset(Vector3f{});
}

void VTEPosition::activateBiasEstimate(const Vector3f &position, const Vector3f &bias,
				       const TargetObs &vision_obs, bool position_from_vision)
{
	const float pos_var = fmaxf(_param_vte_pos_unc_in.get(), kMinVariance);
	const float bias_var = fmaxf(_param_vte_bias_unc_in.get(), kMinVariance);
	const hrt_abstime reset_time = math::min(vision_obs.timestamp, _last_predict);
	const float latency = (_last_predict - reset_time) * kMicrosecondsToSeconds;

	for (int axis = 0; axis < vtest::Axis::size; ++axis) {
		KF_position &filter = _target_est_pos[axis];
		FusionTimes times = filter.getFusionTimes();

		if (position_from_vision) {
			times.record(FusionSource::Relative, vision_obs.timestamp);
		}

		filter.resetPositionAndBias(position(axis), fmaxf(pos_var, vision_obs.meas_unc_xyz(axis)),
					    bias(axis), fmaxf(bias_var, vision_obs.meas_unc_xyz(axis)), latency);
		filter.resetHistory(times);
		filter.pushHistory(reset_time);
	}

	// Reconstruct this one-time initialization bridge with constant UAV velocity and
	// the retained target acceleration. Grow covariance with the normal process noise.
	predictionStep(Vector3f{}, latency);

	PX4_DEBUG("Rel pos init %.2f %.2f %.2f", (double)position(vtest::Axis::x),
		  (double)position(vtest::Axis::y), (double)position(vtest::Axis::z));
	PX4_DEBUG("GNSS bias init %.2f %.2f %.2f", (double)bias(vtest::Axis::x),
		  (double)bias(vtest::Axis::y), (double)bias(vtest::Axis::z));

	_bias.set = true;
	_pre_bias_reference = PreBiasReference::kUnknown;
	resetBiasAveraging();
}

void VTEPosition::publishBiasInitStatus(const Vector3f &raw_bias, const Vector3f &filtered_bias,
					const float raw_bias_delta_norm, const hrt_abstime sample_time)
{
	vte_bias_init_status_s status{};
	status.timestamp = sample_time;
	status.raw_bias[0] = raw_bias(vtest::Axis::x);
	status.raw_bias[1] = raw_bias(vtest::Axis::y);
	status.raw_bias[2] = raw_bias(vtest::Axis::z);
	status.filtered_bias[0] = filtered_bias(vtest::Axis::x);
	status.filtered_bias[1] = filtered_bias(vtest::Axis::y);
	status.filtered_bias[2] = filtered_bias(vtest::Axis::z);
	status.delta_norm = raw_bias_delta_norm;
	_vte_bias_init_status_pub.publish(status);
}

void VTEPosition::fuseActiveMeasurements(const ObsValidMaskU &fusion_mask, const TargetObs observations[kObsTypeCount])
{
	const bool gnss_ready = !shouldBlockGnssFusionUntilBiasReady();
	const bool vision_ready = !shouldBlockVisionFusionUntilBiasReady();
	const TargetObs *order[kObsTypeCount];
	unsigned count = 0;

	// Source priority for samples fused at now. The batch fusion orders delayed ones by capture time.
	if (fusion_mask.flags.fuse_target_gps_pos && gnss_ready) {
		order[count++] = &observations[obsIndex(ObsType::kTargetGpsPos)];
	}

	if (fusion_mask.flags.fuse_mission_pos && gnss_ready) {
		order[count++] = &observations[obsIndex(ObsType::kMissionGpsPos)];
	}

	if (fusion_mask.flags.fuse_vision && vision_ready) {
		order[count++] = &observations[obsIndex(ObsType::kFiducialMarker)];
	}

	if (fusion_mask.flags.fuse_uav_gps_vel) {
		order[count++] = &observations[obsIndex(ObsType::kUavGpsVel)];
	}

#if defined(CONFIG_VTEST_MOVING)

	if (fusion_mask.flags.fuse_target_gps_vel) {
		order[count++] = &observations[obsIndex(ObsType::kTargetGpsVel)];
	}

#endif // CONFIG_VTEST_MOVING

	// The observation model is the same on every axis, only the value and variance differ.
	KF_position::ScalarMeas meas[kObsTypeCount];

	for (unsigned i = 0; i < count; ++i) {
		const TargetObs &obs = *order[i];
		meas[i] = {obs.timestamp, 0.f, 0.f, {}};
		meas[i].H(obs.state_index) = 1.f;

		if (obs.type == ObsType::kFiducialMarker) {
			meas[i].source = FusionSource::Relative;

		} else if ((obs.type == ObsType::kUavGpsVel) || (obs.type == ObsType::kTargetGpsVel)) {
			meas[i].source = FusionSource::Untracked;
		}

		// Bias can become observable after observations were collected in this update.
		// Build the GNSS model from the state being fused, including on that first cycle.
		if (_bias.set && (obs.type == ObsType::kTargetGpsPos || obs.type == ObsType::kMissionGpsPos)) {
			meas[i].H(vtest::State::bias) = 1.f;
		}
	}

	// One batch per axis, so the delayed observations of this cycle share a single replay.
	const float nis_threshold = fmaxf(_param_vte_pos_nis_thre.get(), kMinNisThreshold);
	AxisResults results{};
	perf_begin(_vte_update_perf);

	for (int axis = 0; axis < vtest::Axis::size; ++axis) {
		for (unsigned i = 0; i < count; ++i) {
			meas[i].val = order[i]->meas_xyz(axis);
			meas[i].unc = order[i]->meas_unc_xyz(axis);
		}

		perf_begin(_vte_fusion_perf);
		_target_est_pos[axis].fuseScalarsAtTime(meas, count, _last_predict, nis_threshold, results[axis]);
		perf_end(_vte_fusion_perf);
	}

	perf_end(_vte_update_perf);

	for (unsigned i = 0; i < count; ++i) {
		const TargetObs &obs = *order[i];
		publishAidSource(obs, results, i);
	}

	// Replay can accept or reject older observations, so freshness can move in either direction.
	_last_update = 0;
	_last_relative_meas_fused_time = 0;

	for (const KF_position &filter : _target_est_pos) {
		const FusionTimes times = filter.getFusionTimes();
		_last_update = math::max(_last_update, times.last_fused);
		_last_relative_meas_fused_time = math::max(_last_relative_meas_fused_time, times.last_relative_fused);
	}
}

/*Vision observation: [rx, ry, rz]*/
bool VTEPosition::processObsVision(TargetObs &obs)
{
	fiducial_marker_pos_report_s report;

	if (!_fiducial_marker_pos_report_sub.update(&report)) {
		return false;
	}

	if (!alignFutureSampleTime(report.timestamp_sample, nowUs())) {
		if (shouldEmitWarning(_vision_pos_warn_last)) {
			PX4_WARN("VTE: Vision timestamp in the future!");
		}

		return false;
	}

	if (!isVisionDataValid(report)) {
		return false;
	}

	// The covariance rotation below assumes a unit quaternion.
	matrix::Quaternionf quat_att(report.q);
	quat_att.normalize();
	const Vector3f vision_rel(report.rel_pos);
	const Vector3f vision_ned = quat_att.rotateVector(vision_rel);
	const float min_ev_pos_var = sq(fmaxf(_param_vte_ev_pos_noise.get(), kMinObservationNoise));

	// Rotate covariance matrix to vc-NED
	const SquareMatrix<float, vtest::Axis::size> cov_mat = diag(Vector3f(
				fmaxf(report.cov_rel_pos[0], min_ev_pos_var),
				fmaxf(report.cov_rel_pos[1], min_ev_pos_var),
				fmaxf(report.cov_rel_pos[2], min_ev_pos_var)));
	const matrix::Dcmf r_att = matrix::Dcm<float>(quat_att);
	const SquareMatrix<float, vtest::Axis::size> cov_rotated = r_att * cov_mat * r_att.transpose();

	// Relative position
	if (!vision_ned.isAllFinite()) {
		if (shouldEmitWarning(_vision_pos_warn_last)) {
			PX4_WARN("VTE: Vision position NED is corrupt!");
		}

		return false;
	}

	obs = {ObsType::kFiducialMarker, report.timestamp_sample, vtest::State::pos_rel, vision_ned,
	       Vector3f(cov_rotated(0, 0), cov_rotated(1, 1), cov_rotated(2, 2))
	      };

	return true;
}

/*Drone GNSS velocity observation: [vel_uav_x, vel_uav_y ,vel_uav_z]*/
bool VTEPosition::processObsGNSSVelUav(TargetObs &obs) const
{
	Vector3f vel_uav_ned{};

	if (!getOffsetUavVel(_uav_gps_vel.timestamp, vel_uav_ned)) {
		return false;
	}

	const float unc = fmaxf(sq(_uav_gps_vel.uncertainty), getMinGpsVelVar());
	obs = {ObsType::kUavGpsVel, _uav_gps_vel.timestamp, vtest::State::vel_uav, vel_uav_ned, Vector3f(unc, unc, unc)};

	return true;
}

#if defined(CONFIG_VTEST_MOVING)

/*Target GNSS velocity observation: [r_dotx, r_doty, r_dotz]*/
bool VTEPosition::processObsGNSSVelTarget(const target_gnss_s &target_gnss, TargetObs &obs) const
{
	const Vector3f vel_ned{target_gnss.vel_n_m_s, target_gnss.vel_e_m_s, target_gnss.vel_d_m_s};
	const float unc = fmaxf(sq(target_gnss.s_acc_m_s), getMinGpsVelVar());
	obs = {ObsType::kTargetGpsVel, target_gnss.timestamp_sample, vtest::State::vel_target, vel_ned, Vector3f(unc, unc, unc)};

	return true;
}

#endif // CONFIG_VTEST_MOVING

/*Target GNSS mission observation: [rx + bx, ry + by, rz + bz]*/
bool VTEPosition::processObsGNSSPosMission(TargetObs &obs)
{
	// Obtain GPS relative measurements in NED as target_global - uav_gps_global followed by global2local transformation
	Vector3f gps_relative_pos;
	get_vector_to_next_waypoint(_uav_gps_position.lat_deg, _uav_gps_position.lon_deg,
				    _mission_land_position.lat_deg, _mission_land_position.lon_deg,
				    &gps_relative_pos(0), &gps_relative_pos(1));

	// Down direction (if the drone is above the target, the relative position is positive)
	gps_relative_pos(2) = _uav_gps_position.alt_m - _mission_land_position.alt_m;

	// Offset gps relative position to the center of mass:
	if (_gps_pos_is_offset) {

		if (!_gps_pos_offset_ned.valid
		    || !isMeasUpdated(_gps_pos_offset_ned.timestamp)) {
			return false;
		}

		gps_relative_pos += _gps_pos_offset_ned.xyz;
	}

	const float min_gps_pos_var = getMinGpsPosVar();
	const float gps_unc_horizontal = fmaxf(sq(_uav_gps_position.eph), min_gps_pos_var);
	const float gps_unc_vertical = fmaxf(sq(_uav_gps_position.epv), min_gps_pos_var);

	if (!gps_relative_pos.isAllFinite() || !PX4_ISFINITE(gps_unc_horizontal) || !PX4_ISFINITE(gps_unc_vertical)) {
		if (shouldEmitWarning(_mission_pos_warn_last)) {
			PX4_WARN("Mission position observation is invalid!");
		}

		return false;
	}

	// GNSS observes pos_rel + bias; fuseActiveMeasurements() adds the bias coefficient once it is active.
	obs = {ObsType::kMissionGpsPos, _uav_gps_position.timestamp, vtest::State::pos_rel, gps_relative_pos,
	       Vector3f(gps_unc_horizontal, gps_unc_horizontal, gps_unc_vertical)
	      };

	// Latest GNSS-relative position, used for bias initialization. Mission and target GNSS are exclusive.
	_pos_rel_gnss.timestamp = obs.timestamp;
	_pos_rel_gnss.valid = true;
	_pos_rel_gnss.xyz = gps_relative_pos;
	_pos_rel_gnss_is_target = false;

	return true;
}

/*Target GNSS observation: [rx + bx, ry + by, rz + bz]*/
bool VTEPosition::processObsGNSSPosTarget(const target_gnss_s &target_gnss, TargetObs &obs)
{
	const int64_t time_diff_us = signedTimeDiffUs(target_gnss.timestamp_sample, _uav_gps_position.timestamp);
	const float dt_sync_us = fabsf(static_cast<float>(time_diff_us));

	if (dt_sync_us > _meas_recent_timeout_us) {
		PX4_DEBUG("Time diff between UAV GNSS and target GNSS too high: %.2f [ms] > timeout: %.2f [ms]",
			  static_cast<double>(dt_sync_us * kMicrosecondsToMilliseconds),
			  static_cast<double>(_meas_recent_timeout_us * kMicrosecondsToMilliseconds));
		return false;
	}

	double uav_lat_deg = _uav_gps_position.lat_deg;
	double uav_lon_deg = _uav_gps_position.lon_deg;
	float uav_alt_m = _uav_gps_position.alt_m;

	const float dt_sync_s = static_cast<float>(time_diff_us) * kMicrosecondsToSeconds;
	bool uav_position_interpolated = false;

	// Move the UAV fix to the target fix time. The receivers are not synchronized, so the fixes rarely
	// line up; the local velocity stands in when the GNSS velocity is unavailable.
	Vector3f uav_vel_ned{};
	float uav_vel_var = getMinGpsVelVar();
	bool has_uav_velocity = getOffsetUavVel(_uav_gps_position.timestamp, uav_vel_ned);

	if (has_uav_velocity) {
		uav_vel_var = fmaxf(sq(_uav_gps_vel.uncertainty), uav_vel_var);

	} else if (_local_velocity.valid) {
		uav_vel_ned = _local_velocity.xyz;
		// Local velocity has no supplied covariance here; use the configured initial variance.
		uav_vel_var = fmaxf(_param_vte_vel_unc_in.get(), uav_vel_var);
		has_uav_velocity = true;
	}

	if (time_diff_us != 0) {
		if (!has_uav_velocity) {
			return false;
		}

		const float delta_n = uav_vel_ned(vtest::Axis::x) * dt_sync_s;
		const float delta_e = uav_vel_ned(vtest::Axis::y) * dt_sync_s;

		double lat_res = uav_lat_deg;
		double lon_res = uav_lon_deg;
		add_vector_to_global_position(uav_lat_deg, uav_lon_deg, delta_n, delta_e, &lat_res, &lon_res);
		uav_lat_deg = lat_res;
		uav_lon_deg = lon_res;
		uav_alt_m = _uav_gps_position.alt_m - uav_vel_ned(vtest::Axis::z) * dt_sync_s;
		uav_position_interpolated = true;
	}

	// Obtain GPS relative measurements in NED as target_global - uav_gps_global followed by global2local transformation
	Vector3f gps_relative_pos;
	get_vector_to_next_waypoint(uav_lat_deg, uav_lon_deg,
				    target_gnss.latitude_deg, target_gnss.longitude_deg,
				    &gps_relative_pos(0), &gps_relative_pos(1));

	// Down direction (if the drone is above the target, the relative position is positive)
	gps_relative_pos(2) = uav_alt_m - target_gnss.altitude_msl_m;

	// Offset gps relative position to the center of mass:
	if (_gps_pos_is_offset) {
		if (!_gps_pos_offset_ned.valid
		    || !isMeasUpdated(_gps_pos_offset_ned.timestamp)) {

			return false;
		}

		gps_relative_pos += _gps_pos_offset_ned.xyz;
	}

	// Grow uncertainty with both velocity error and the normal UAV acceleration process noise.
	const float uav_acc_var = fmaxf(_param_vte_acc_d_unc.get(), kMinVariance);
	const float propagation_unc = uav_position_interpolated
				      ? sq(dt_sync_s) * (uav_vel_var + uav_acc_var * fabsf(dt_sync_s) / 3.f) : 0.f;
	const float min_gps_pos_var = getMinGpsPosVar();
	const float gps_unc_horizontal = fmaxf(sq(_uav_gps_position.eph), min_gps_pos_var)
					 + fmaxf(sq(target_gnss.eph), min_gps_pos_var)
					 + propagation_unc;
	const float gps_unc_vertical = fmaxf(sq(_uav_gps_position.epv), min_gps_pos_var)
				       + fmaxf(sq(target_gnss.epv), min_gps_pos_var)
				       + propagation_unc;

	if (!gps_relative_pos.isAllFinite() || !PX4_ISFINITE(gps_unc_horizontal) || !PX4_ISFINITE(gps_unc_vertical)) {
		if (shouldEmitWarning(_target_gps_pos_warn_last)) {
			PX4_WARN("Target GNSS position observation is invalid!");
		}

		return false;
	}

	// GNSS observes pos_rel + bias; fuseActiveMeasurements() adds the bias coefficient once it is active.
	obs = {ObsType::kTargetGpsPos, target_gnss.timestamp_sample, vtest::State::pos_rel, gps_relative_pos,
	       Vector3f(gps_unc_horizontal, gps_unc_horizontal, gps_unc_vertical)
	      };

	// Latest GNSS-relative position, used for bias initialization.
	_pos_rel_gnss.timestamp = obs.timestamp;
	_pos_rel_gnss.valid = true;
	_pos_rel_gnss.xyz = gps_relative_pos;
	_pos_rel_gnss_is_target = true;

	return true;
}

void VTEPosition::publishAidSource(const TargetObs &target_obs, const AxisResults &results, const unsigned obs_pos)
{
	vte_aid_source3d_s target_innov = {};
	target_innov.timestamp_sample = target_obs.timestamp;
	target_innov.timestamp = nowUs();
	target_innov.time_last_predict = _last_predict;
	target_innov.time_since_meas_ms = static_cast<float>(signedTimeDiffUs(_last_predict, target_obs.timestamp))
					  * kMicrosecondsToMilliseconds;

	uint8_t history_steps = 0;
	uint8_t replayed_count = 0;
	uint8_t replay_revised_count = 0;

	for (int j = 0; j < vtest::Axis::size; j++) {
		const FusionResult &result = results[j][obs_pos];
		target_innov.observation[j] = target_obs.meas_xyz(j);
		target_innov.observation_variance[j] = target_obs.meas_unc_xyz(j);
		target_innov.innovation[j]          = result.innov;
		target_innov.innovation_variance[j] = result.innov_var;
		target_innov.test_ratio[j]          = result.test_ratio;
		target_innov.fusion_status[j]       = static_cast<uint8_t>(result.status);
		target_innov.replay_test_ratio_max[j] = result.replay_test_ratio_max;

		if (result.status == FusionStatus::STATUS_FUSED_OOSM) {
			history_steps = math::max(result.history_steps, history_steps);
			replayed_count = math::max(result.replayed_count, replayed_count);
			replay_revised_count = math::max(result.replay_revised_count, replay_revised_count);
		}
	}

	target_innov.history_steps = history_steps;
	target_innov.replayed_count = replayed_count;
	target_innov.replay_revised_count = replay_revised_count;

	_vte_aid_pubs[obsIndex(target_obs.type)].publish(target_innov);
}

void VTEPosition::publishInvalidState()
{
	// Mark the logged state invalid. landing_target_pose is not published: precision landing keeps
	// approaching the last pose it received and judges its age with its own timeout.
	_vte_state.timestamp = nowUs();
	_vte_state.rel_pos_valid = false;
	_vte_state.rel_vel_valid = false;
	_target_estimator_state_pub.publish(_vte_state);
}

void VTEPosition::publishTarget()
{
	_vte_state = {};
	_target_pose = {};

	_target_pose.timestamp = _last_predict;
	_vte_state.timestamp = _last_predict;
	_target_pose.is_static = kTargetIsStatic;

	Vector3f rel_pos;
	Vector3f rel_pos_var;
	Vector3f rel_vel;
	Vector3f rel_vel_var;

	for (int axis = 0; axis < vtest::Axis::size; ++axis) {
		const KF_position &filter = _target_est_pos[axis];
		const EstimatorState &state = filter.getState();
		const EstimatorState state_var = filter.getStateCovarianceDiag();

		_vte_state.rel_pos[axis] = state(vtest::State::pos_rel);
		_vte_state.cov_rel_pos[axis] = state_var(vtest::State::pos_rel);
		_vte_state.vel_uav[axis] = state(vtest::State::vel_uav);
		_vte_state.cov_vel_uav[axis] = state_var(vtest::State::vel_uav);
		_vte_state.bias[axis] = state(vtest::State::bias);
		_vte_state.cov_bias[axis] = state_var(vtest::State::bias);

		rel_pos(axis) = state(vtest::State::pos_rel);
		rel_pos_var(axis) = state_var(vtest::State::pos_rel);
		rel_vel(axis) = -state(vtest::State::vel_uav);
		rel_vel_var(axis) = state_var(vtest::State::vel_uav);

#if defined(CONFIG_VTEST_MOVING)
		_vte_state.vel_target[axis] = state(vtest::State::vel_target);
		_vte_state.cov_vel_target[axis] = state_var(vtest::State::vel_target);
		_vte_state.acc_target[axis] = state(vtest::State::acc_target);
		_vte_state.cov_acc_target[axis] = state_var(vtest::State::acc_target);

		// Relative velocity is vel_target - vel_uav: Var = Var(vel_target) + Var(vel_uav) - 2 Cov(vel_uav, vel_target)
		rel_vel(axis) += state(vtest::State::vel_target);
		rel_vel_var(axis) += state_var(vtest::State::vel_target)
				     - 2.f * filter.getStateCovariance()(vtest::State::vel_uav, vtest::State::vel_target);
#endif // CONFIG_VTEST_MOVING
	}

	_target_pose.x_rel = rel_pos(0);
	_target_pose.y_rel = rel_pos(1);
	_target_pose.z_rel = rel_pos(2);
	_target_pose.cov_x_rel = rel_pos_var(0);
	_target_pose.cov_y_rel = rel_pos_var(1);
	_target_pose.cov_z_rel = rel_pos_var(2);
	_target_pose.vx_rel = rel_vel(0);
	_target_pose.vy_rel = rel_vel(1);
	_target_pose.vz_rel = rel_vel(2);
	_target_pose.cov_vx_rel = rel_vel_var(0);
	_target_pose.cov_vy_rel = rel_vel_var(1);
	_target_pose.cov_vz_rel = rel_vel_var(2);

	const bool relative_position_recent = !hasTimedOut(_last_update, _target_valid_timeout_us);
	_target_pose.rel_pos_valid = relative_position_recent && rel_pos.isAllFinite();
	_target_pose.rel_vel_valid = _target_pose.rel_pos_valid && rel_vel.isAllFinite(); // Velocity cannot be valid if position invalid

	// EKF2 auxiliary velocity aiding must only use velocity backed by a recent relative
	// measurement (vision). GNSS/mission position can keep the target estimate valid, but it is not a
	// relative measurement and therefore must not enable this aiding path by itself.
	const hrt_abstime now = nowUs();
	const bool recent_relative_fusion = (_last_relative_meas_fused_time != 0)
					    && (now >= _last_relative_meas_fused_time)
					    && ((now - _last_relative_meas_fused_time) < _meas_recent_timeout_us);
	_target_pose.rel_vel_ekf2_valid = _target_pose.is_static && _target_pose.rel_vel_valid && recent_relative_fusion;

	_vte_state.rel_pos_valid = _target_pose.rel_pos_valid;
	_vte_state.rel_vel_valid = _target_pose.rel_vel_valid;

	// Prec land does not check _target_pose.abs_pos_valid. Only send the target if abs pose valid.
	if (_local_position.valid && _target_pose.rel_pos_valid) {
		_target_pose.x_abs = _target_pose.x_rel + _local_position.xyz(0);
		_target_pose.y_abs = _target_pose.y_rel + _local_position.xyz(1);
		_target_pose.z_abs = _target_pose.z_rel + _local_position.xyz(2);
		_target_pose.abs_pos_valid = true;

		_target_pose_pub.publish(_target_pose);

	}

	_target_estimator_state_pub.publish(_vte_state);
}

void VTEPosition::print_status() const
{
	const auto yes_no = [](const bool value) { return value ? "yes" : "no"; };
	const auto age_s = [](const hrt_abstime timestamp) -> double {
		if (timestamp == 0)
		{
			return -1.;
		}

		return signedTimeDiffUs(nowUs(), timestamp) * 1e-6;
	};
	const auto pre_bias_reference_name = [this]() {
		switch (_pre_bias_reference) {
		case PreBiasReference::kVision: return "vision";

		case PreBiasReference::kGnss: return "gnss";

		default: return "unknown";
		}
	};

	const bool relative_position_valid = _estimator_initialized && !hasTimedOut(_last_update, _target_valid_timeout_us);

	PX4_INFO("  initialized: %s, valid: %s, predict/update age: %.3f / %.3f s",
		 yes_no(_estimator_initialized), yes_no(relative_position_valid), age_s(_last_predict), age_s(_last_update));
	PX4_INFO("  output valid: position: %s, velocity: %s, EKF2 velocity: %s",
		 yes_no(_target_pose.rel_pos_valid),
		 yes_no(_target_pose.rel_vel_valid),
		 yes_no(_target_pose.rel_vel_ekf2_valid));

	PX4_INFO("  bias set: %s, averaging: %s, pre-bias reference: %s",
		 yes_no(_bias.set),
		 yes_no(_bias.averaging_active),
		 pre_bias_reference_name());
}

void VTEPosition::checkMeasurementInputs()
{
	// Make sure fast topics are up to date.
	_local_position.valid = _local_position.valid && isMeasUpdated(_local_position.timestamp);
	_local_velocity.valid = _local_velocity.valid && isMeasUpdated(_local_velocity.timestamp);
	_velocity_offset_ned.valid = _velocity_offset_ned.valid && isMeasUpdated(_velocity_offset_ned.timestamp);
	_gps_pos_offset_ned.valid = _gps_pos_offset_ned.valid && isMeasUpdated(_gps_pos_offset_ned.timestamp);

	// Slower observation topics are valid if recent
#if defined(CONFIG_VTEST_MOVING)
	_target_gps_vel.valid = _target_gps_vel.valid && isMeasRecent(_target_gps_vel.timestamp);
#endif // CONFIG_VTEST_MOVING
	_pos_rel_gnss.valid = _pos_rel_gnss.valid && isMeasRecent(_pos_rel_gnss.timestamp);
	_uav_gps_vel.valid = _uav_gps_vel.valid && isMeasRecent(_uav_gps_vel.timestamp);
}

void VTEPosition::setGpsPosOffset(const matrix::Vector3f &xyz, const bool gps_is_offset)
{
	_gps_pos_is_offset = gps_is_offset;
	_gps_pos_offset_ned.xyz = xyz;
	_gps_pos_offset_ned.valid = xyz.isAllFinite();
	// This is the freshness of the current attitude correction, not the GNSS sample time.
	// Using current attitude remains an approximation while the vehicle rotates during GNSS latency.
	_gps_pos_offset_ned.timestamp = nowUs();
}

void VTEPosition::setVelOffset(const matrix::Vector3f &xyz)
{
	_velocity_offset_ned.xyz = xyz;
	_velocity_offset_ned.valid = xyz.isAllFinite();
	_velocity_offset_ned.timestamp = nowUs();
}

void VTEPosition::setLocalVelocity(const matrix::Vector3f &vel_xyz, const bool vel_valid, hrt_abstime timestamp)
{
	_local_velocity.xyz = vel_xyz;
	_local_velocity.valid = vel_valid && vel_xyz.isAllFinite() && isMeasUpdated(timestamp);
	_local_velocity.timestamp = timestamp;
}

void VTEPosition::setLocalPosition(const matrix::Vector3f &xyz, const bool pos_valid, hrt_abstime timestamp)
{
	_local_position.xyz = xyz;
	_local_position.valid = pos_valid && xyz.isAllFinite() && isMeasUpdated(timestamp);
	_local_position.timestamp = timestamp;
}

void VTEPosition::setMissionPosition(const double lat_deg, const double lon_deg, const float alt_m)
{
	// Cache the mission waypoint independently from the current aid mask so the caller
	// can provide it before mission aiding is enabled or while the mask is toggled.
	_mission_land_position.lat_deg = lat_deg;
	_mission_land_position.lon_deg = lon_deg;
	_mission_land_position.alt_m = alt_m;
	_mission_land_position.valid = isLatLonAltValid(_mission_land_position.lat_deg, _mission_land_position.lon_deg,
				       _mission_land_position.alt_m, "Mission position ", &_mission_pos_warn_last);

	if (_mission_land_position.valid) {
		PX4_INFO("Mission position lat %.8f, lon %.8f [deg], alt %.1f [m]", lat_deg,
			 lon_deg, (double)(alt_m));

	} else if (shouldEmitWarning(_mission_pos_status_warn_last)) {
		PX4_WARN("Mission position not used because not valid");
	}
}

void VTEPosition::updateParams()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	ModuleParams::updateParams();
}

float VTEPosition::getMinGpsVelVar() const
{
	return sq(fmaxf(_param_vte_gps_vel_noise.get(), kMinObservationNoise));
}

float VTEPosition::getMinGpsPosVar() const
{
	return sq(fmaxf(_param_vte_gps_pos_noise.get(), kMinObservationNoise));
}

float VTEPosition::getBiasAveragingThreshold() const
{
	return fmaxf(_param_vte_bias_avg_thr.get(), 0.f);
}

hrt_abstime VTEPosition::getBiasAveragingTimeoutUs() const
{
	const float seconds = fmaxf(_param_vte_bias_avg_tout.get(), 0.f);
	const float microseconds = fminf(seconds * 1e6f, static_cast<float>(UINT32_MAX));
	return static_cast<hrt_abstime>(microseconds);
}

bool VTEPosition::isLatLonAltValid(double lat_deg, double lon_deg, float alt_m, const char *who,
				   hrt_abstime *warn_last)
{
	// all finite
	if (!PX4_ISFINITE(lat_deg) || !PX4_ISFINITE(lon_deg) || !PX4_ISFINITE(alt_m)) {
		if (who && (!warn_last || shouldEmitWarning(*warn_last))) {
			PX4_WARN("%s position not finite! lat: %.8f, lon: %.8f, alt: %.1f",
				 who, lat_deg, lon_deg, (double)alt_m);
		}

		return false;
	}

	// latitude/longitude within symmetric geographic ranges
	if ((fabs(lat_deg) > kLatAbsMaxDeg) || (fabs(lon_deg) > kLonAbsMaxDeg)) {
		if (who && (!warn_last || shouldEmitWarning(*warn_last))) {
			PX4_WARN("%s lat/lon out of range! lat: %.8f, lon: %.8f", who, lat_deg, lon_deg);
		}

		return false;
	}

	// (0,0) sentinel is considered invalid
	if ((fabs(lat_deg) < DBL_EPSILON) && (fabs(lon_deg) < DBL_EPSILON)) {
		if (who && (!warn_last || shouldEmitWarning(*warn_last))) {
			PX4_WARN("%s position near (0,0)!", who);
		}

		return false;
	}

	// altitude window
	if ((alt_m < kAltMinM) || (alt_m > kAltMaxM)) {
		if (who && (!warn_last || shouldEmitWarning(*warn_last))) {
			PX4_WARN("%s altitude out of range! alt: %.1f [m] (limits: %.1f;%.1f)",
				 who, (double)alt_m, (double)kAltMinM, (double)kAltMaxM);
		}

		return false;
	}

	return true;
}

} // namespace vision_target_estimator
