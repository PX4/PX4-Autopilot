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

VTEPosition::VTEPosition() :
	ModuleParams(nullptr)
{
	_target_pose_pub.advertise();
	_target_estimator_state_pub.advertise();
	_vte_aid_gps_pos_target_pub.advertise();
	_vte_aid_gps_pos_mission_pub.advertise();
	_vte_aid_gps_vel_uav_pub.advertise();
	_vte_aid_gps_vel_target_pub.advertise();
	_vte_aid_fiducial_marker_pub.advertise();
	_vte_bias_init_status_pub.advertise();

	updateParams();
}

VTEPosition::~VTEPosition()
{
	perf_free(_vte_predict_perf);
	perf_free(_vte_update_perf);
	perf_free(_vte_fusion_perf);
}

bool VTEPosition::shouldEmitWarning(hrt_abstime &last_warn)
{
	const hrt_abstime now = nowUs();

	if ((last_warn == 0) || (now < last_warn) || ((now - last_warn) > kWarnThrottleIntervalUs)) {
		last_warn = now;
		return true;
	}

	return false;
}

bool VTEPosition::init()
{
	// Check valid vtest_derivation/generated/state.h
	if (vtest::Axis::size != kExpectedAxisCount) {
		PX4_ERR("VTE: Invalid axis size: %d, expected %d. Generate vtest_derivation/derivation.py",
			static_cast<int>(vtest::Axis::size), kExpectedAxisCount);
		return false;
	}

#if defined(CONFIG_VTEST_MOVING)
	PX4_INFO("VTE for moving target init");
	PX4_WARN("VTE for moving targets has not been thoroughly tested");

	if (vtest::State::size != kExpectedMovingPositionStateCount) {
		PX4_ERR("VTE: Invalid state size: %d, expected %d. Generate vtest_derivation/derivation.py",
			static_cast<int>(vtest::State::size), kExpectedMovingPositionStateCount);
		return false;
	}

#else
	PX4_INFO("VTE for static target init");

	if (vtest::State::size != kExpectedStaticPositionStateCount) {
		PX4_ERR("VTE: Invalid state size: %d, expected %d. Generate vtest_derivation/derivation.py",
			static_cast<int>(vtest::State::size), kExpectedStaticPositionStateCount);
		return false;
	}

#endif // CONFIG_VTEST_MOVING

	for (int axis = 0; axis < vtest::Axis::size; ++axis) {
		_target_est_pos[axis].resetHistory();
	}

	return true;
}

void VTEPosition::resetFilter()
{
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
	if (performUpdateStep(acc_ned)) {
		_last_update = _last_predict;
	}

	if (_estimator_initialized) {
		// Store the post-fusion posterior snapshot at the current timestamp for OOSM fusion.
		for (int axis = 0; axis < vtest::Axis::size; ++axis) {
			_target_est_pos[axis].pushHistory(_last_predict);
		}

		publishTarget();
	}
}

bool VTEPosition::initEstimator(const AxisEstimatorStates &state_init)
{
	// Get initial variance from params
	const float state_pos_var = fmaxf(_param_vte_pos_unc_in.get(), kMinVariance);
	const float state_vel_var = fmaxf(_param_vte_vel_unc_in.get(), kMinVariance);
	const float state_bias_var = fmaxf(_param_vte_bias_unc_in.get(), kMinVariance);

	EstimatorState state_var_init{};
	state_var_init(vtest::State::pos_rel) = state_pos_var;
	state_var_init(vtest::State::vel_uav) = state_vel_var;
	state_var_init(vtest::State::bias) = state_bias_var;

#if defined(CONFIG_VTEST_MOVING)
	const float state_acc_var = fmaxf(_param_vte_acc_unc_in.get(), kMinVariance);
	const float state_target_vel_var = fmaxf(_param_vte_vel_unc_in.get(), kMinVariance);
	state_var_init(vtest::State::acc_target) = state_acc_var;
	state_var_init(vtest::State::vel_target) = state_target_vel_var;

#endif // CONFIG_VTEST_MOVING

	for (int i = 0; i < vtest::Axis::size; i++) {
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

bool VTEPosition::performUpdateStep(const Vector3f &vehicle_acc_ned)
{
	// Gather observations and tag the fusion mask bits for each new valid source.
	resetObservations();
	ObsValidMaskU fusion_mask{};
	processObservations(fusion_mask, _obs_buffer);

	if (fusion_mask.value == 0) {
		return false;
	}

	if (!_estimator_initialized) {
		if (!hasNewPositionSensorData(fusion_mask)) {
			return false;
		}

		if (!initializeEstimator(fusion_mask, _obs_buffer)) {
			return false;
		}
	}

	if (!_bias.set) {
		updateBiasIfObservable(fusion_mask, _obs_buffer);
	}

	return fuseActiveMeasurements(vehicle_acc_ned, fusion_mask, _obs_buffer);
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

	if (updateUavGpsData()) {
		if (_vte_aid_mask.flags.use_mission_pos && _mission_land_position.valid) {
			fusion_mask.flags.fuse_mission_pos = processObsGNSSPosMission(observations[obsIndex(ObsType::kMissionGpsPos)]);
		}

		if (_vte_aid_mask.flags.use_uav_gps_vel && _uav_gps_vel.valid) {
			fusion_mask.flags.fuse_uav_gps_vel = processObsGNSSVelUav(observations[obsIndex(ObsType::kUavGpsVel)]);
		}
	}

	target_gnss_s target_gnss{};

	if (_target_gnss_sub.update(&target_gnss)) {
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

bool VTEPosition::isUavGpsPositionValid()
{
	if (!isMeasRecent(_uav_gps_position.timestamp)) {
		return false;
	}

	if (!isLatLonAltValid(_uav_gps_position.lat_deg, _uav_gps_position.lon_deg,
			      _uav_gps_position.alt_m, "UAV GPS ", &_uav_gps_pos_warn_last)) {
		return false;
	}

	if (_gps_pos_is_offset && !_gps_pos_offset_ned.valid) {
		PX4_DEBUG("Uav Gps position invalid offset!"); // Debug because this message can spam
		return false;
	}

	return true;
}

bool VTEPosition::isUavGpsVelocityValid()
{
	// Check freshness first to early-out on stale samples before touching the payload.
	if (!isMeasRecent(_uav_gps_vel.timestamp)) {
		return false;
	}

	if (!_uav_gps_vel.xyz.isAllFinite()) {
		if (shouldEmitWarning(_uav_gps_vel_warn_last)) {
			PX4_WARN("UAV GPS velocity not finite! vx: %.1f, vy: %.1f, vz: %.1f",
				 (double)_uav_gps_vel.xyz(vtest::Axis::x), (double)_uav_gps_vel.xyz(vtest::Axis::y),
				 (double)_uav_gps_vel.xyz(vtest::Axis::z));
		}

		return false;
	}

	if (_gps_pos_is_offset && !_velocity_offset_ned.valid) {
		PX4_DEBUG("Uav Gps velocity invalid offset!");
		return false;
	}

	return true;
}

bool VTEPosition::updateUavGpsData()
{
	sensor_gps_s vehicle_gps_position;
	const bool vehicle_gps_position_updated = _vehicle_gps_position_sub.update(&vehicle_gps_position);

	if (vehicle_gps_position_updated) {
		// Position
		_uav_gps_position.lat_deg = vehicle_gps_position.latitude_deg;
		_uav_gps_position.lon_deg = vehicle_gps_position.longitude_deg;
		_uav_gps_position.alt_m = (float)vehicle_gps_position.altitude_msl_m;
		_uav_gps_position.timestamp = vehicle_gps_position.timestamp_sample;
		_uav_gps_position.eph = vehicle_gps_position.eph;
		_uav_gps_position.epv = vehicle_gps_position.epv;
		_uav_gps_position.valid = isUavGpsPositionValid();

		// Velocity
		_uav_gps_vel.timestamp = vehicle_gps_position.timestamp_sample;
		_uav_gps_vel.xyz(vtest::Axis::x) = vehicle_gps_position.vel_n_m_s;
		_uav_gps_vel.xyz(vtest::Axis::y) = vehicle_gps_position.vel_e_m_s;
		_uav_gps_vel.xyz(vtest::Axis::z) = vehicle_gps_position.vel_d_m_s;
		_uav_gps_vel.uncertainty = vehicle_gps_position.s_variance_m_s;
		_uav_gps_vel.valid = vehicle_gps_position.vel_ned_valid && isUavGpsVelocityValid();

	} else {
		// Check if stored data is still valid
		_uav_gps_position.valid = _uav_gps_position.valid && isMeasRecent(_uav_gps_position.timestamp);
		_uav_gps_vel.valid = _uav_gps_vel.valid && isMeasRecent(_uav_gps_vel.timestamp);
	}

	return vehicle_gps_position_updated;
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

	if (!hasInitialVelocityEstimate()) {
		if (shouldEmitWarning(_init_vel_warn_last)) {
			PX4_WARN("No UAV velocity estimate. Estimator cannot be started.");
		}

		return false;
	}

	// If both GNSS and vision are available at startup, only keep the GNSS-first
	// path when bias averaging is explicitly enabled.
	// Otherwise, trust vision and activate the bias immediately.
	const bool has_vision_observation = fusion_mask.flags.fuse_vision;
	const bool can_average_initial_bias = shouldSetBias(has_vision_observation) && isBiasAveragingEnabled();

	Vector3f initial_position{};
	const PreBiasReference initial_reference = selectInitialPosition(fusion_mask, observations, initial_position,
			!can_average_initial_bias);
	const bool average_initial_bias = shouldAverageInitialBias(has_vision_observation, initial_reference);

	Vector3f initial_bias{};

	if (shouldSetBias(has_vision_observation) && !average_initial_bias) {
		selectInitialBiasFromGnss(observations, initial_position, initial_bias);
	}

	AxisEstimatorStates state_init{};
	buildInitialStateArray(initial_position, selectInitialUavVelocity(), initial_bias, state_init);

	if (!initEstimator(state_init)) {
		resetFilter();
		return false;
	}

	PX4_DEBUG("VTE Position Estimator properly initialized.");
	_estimator_initialized = true;
	_pre_bias_reference = _bias.set ? PreBiasReference::kUnknown : initial_reference;
	_last_update = nowUs();
	_last_predict = _last_update;

	for (int axis = 0; axis < vtest::Axis::size; ++axis) {
		_target_est_pos[axis].resetHistory();
	}

	return true;
}

bool VTEPosition::hasInitialVelocityEstimate() const
{
	return (_local_velocity.valid && isMeasRecent(_local_velocity.timestamp))
	       || (_uav_gps_vel.valid && isMeasRecent(_uav_gps_vel.timestamp));
}

Vector3f VTEPosition::selectInitialUavVelocity() const
{
	// Prefer GNSS-derived velocity when available; local velocity is a fallback.
	if (_uav_gps_vel.valid && isMeasRecent(_uav_gps_vel.timestamp)) {
		return _uav_gps_vel.xyz;
	}

	if (_local_velocity.valid && isMeasRecent(_local_velocity.timestamp)) {
		return _local_velocity.xyz;
	}

	return Vector3f{};
}

void VTEPosition::selectInitialBiasFromGnss(const TargetObs observations[kObsTypeCount],
		const Vector3f &initial_position, Vector3f &initial_bias)
{
	PX4_DEBUG("VTE Position setting GNSS bias.");
	const hrt_abstime sample_time = observations[obsIndex(ObsType::kFiducialMarker)].timestamp;
	Vector3f gnss_sample{};

	if (selectBiasGnssSample(sample_time, gnss_sample)) {
		initial_bias = gnss_sample - initial_position;
	}

	_bias.set = true;
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

VTEPosition::PreBiasReference VTEPosition::selectInitialPosition(const ObsValidMaskU &fusion_mask,
		const TargetObs observations[kObsTypeCount], matrix::Vector3f &initial_position,
		const bool prefer_vision)
{
	// Before the GNSS bias is ready, initialize from vision when requested and
	// otherwise fall back to the available GNSS position source.
	if (prefer_vision && fusion_mask.flags.fuse_vision) {
		initial_position = observations[obsIndex(ObsType::kFiducialMarker)].meas_xyz;
		return PreBiasReference::kVision;
	}

	if (fusion_mask.flags.fuse_target_gps_pos) {
		initial_position = observations[obsIndex(ObsType::kTargetGpsPos)].meas_xyz;
		return PreBiasReference::kGnss;
	}

	if (fusion_mask.flags.fuse_mission_pos) {
		initial_position = observations[obsIndex(ObsType::kMissionGpsPos)].meas_xyz;
		return PreBiasReference::kGnss;
	}

	if (!prefer_vision && fusion_mask.flags.fuse_vision) {
		initial_position = observations[obsIndex(ObsType::kFiducialMarker)].meas_xyz;
		return PreBiasReference::kVision;
	}

	return PreBiasReference::kUnknown;
}

bool VTEPosition::getOffsetUavVel(const hrt_abstime sample_time, Vector3f &uav_velocity_ned) const
{
	if (!_uav_gps_vel.valid || !isTimeDifferenceWithin(_uav_gps_vel.timestamp, sample_time, _meas_updated_timeout_us)) {
		return false;
	}

	uav_velocity_ned = _uav_gps_vel.xyz;

	if (_gps_pos_is_offset) {
		if (!_velocity_offset_ned.valid
		    || !isTimeDifferenceWithin(_velocity_offset_ned.timestamp, _uav_gps_vel.timestamp, _meas_updated_timeout_us)) {
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
		const float dt = static_cast<float>(dt_us) * kMicrosecondsToSeconds;
		Vector3f uav_velocity_ned{};
		bool has_velocity_for_propagation = false;

		if (_local_velocity.valid
		    && isTimeDifferenceWithin(_local_velocity.timestamp, sample_time, _meas_updated_timeout_us)) {
			uav_velocity_ned = _local_velocity.xyz;
			has_velocity_for_propagation = true;

		} else if (getOffsetUavVel(sample_time, uav_velocity_ned)) {
			has_velocity_for_propagation = true;
		}

		if (has_velocity_for_propagation) {
			gnss_sample -= uav_velocity_ned * dt;
		}
	}

	return true;
}

void VTEPosition::updateBiasIfObservable(const ObsValidMaskU &fusion_mask,
		const TargetObs observations[kObsTypeCount])
{
	if (!fusion_mask.flags.fuse_vision) {
		return;
	}

	const TargetObs &vision_obs = observations[obsIndex(ObsType::kFiducialMarker)];
	const hrt_abstime sample_time = vision_obs.timestamp;

	if (sample_time == 0) {
		return;
	}

	Vector3f initial_position{};
	selectInitialPosition(fusion_mask, observations, initial_position);

	Vector3f gnss_sample{};

	if (!selectBiasGnssSample(sample_time, gnss_sample)) {
		if (_bias.averaging_active) {
			PX4_WARN("GNSS unavailable, ending bias averaging and switching to vision.");
			activateBiasEstimate(initial_position, _bias.initial_lpf.getState());

		} else if (_pre_bias_reference == PreBiasReference::kGnss) {
			// The state was initialized GNSS-referenced, so vision fusion is currently
			// gated on the GNSS bias being observable. Without an aligned GNSS sample
			// we would never make progress and vision would be blocked indefinitely.
			// Activate the bias with a zero offset so the filter unblocks.
			PX4_WARN("GNSS bias not observable, setting bias to zero.");
			activateBiasEstimate(initial_position, Vector3f{});
		}

		return;
	}

	const Vector3f initial_bias = gnss_sample - initial_position;

	if (!_bias.averaging_active && shouldAverageInitialBias(fusion_mask.flags.fuse_vision)) {
		PX4_INFO("GNSS/vision bias observable, averaging initial GNSS bias.");
		startBiasAveraging(initial_bias, sample_time);
		return;
	}

	if (_bias.averaging_active) {
		if (!updateBiasAveraging(initial_bias, sample_time)) {
			return;
		}

		PX4_INFO("GNSS bias averaging complete, setting position and bias.");
		const Vector3f filtered_bias = _bias.initial_lpf.getState();
		activateBiasEstimate(gnss_sample - filtered_bias, filtered_bias);
		return;
	}

	PX4_INFO("GNSS/vision bias observable, setting position and bias.");
	activateBiasEstimate(initial_position, initial_bias);
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
	static constexpr float kInitialBiasLpfMinTimeFactor{2.f};
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

	const float dt = static_cast<float>(sample_time - _bias.last_sample_time) * kMicrosecondsToSeconds;
	_bias.initial_lpf.update(bias_sample, dt);
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

	const hrt_abstime min_averaging_time_us = static_cast<hrt_abstime>(
				kInitialBiasLpfMinTimeFactor * kInitialBiasLpfTimeConstantS * 1e6f);
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

void VTEPosition::activateBiasEstimate(const Vector3f &initial_position, const Vector3f &initial_bias)
{
	const float pos_var = fmaxf(_param_vte_pos_unc_in.get(), kMinVariance);
	const float bias_var = fmaxf(_param_vte_bias_unc_in.get(), kMinVariance);

	for (int axis = 0; axis < vtest::Axis::size; ++axis) {
		KF_position &filter = _target_est_pos[axis];
		EstimatorState state = filter.getState();
		state(vtest::State::bias) = initial_bias(axis);
		state(vtest::State::pos_rel) = initial_position(axis);
		filter.setState(state);

		EstimatorState state_var = filter.getStateCovarianceDiag();
		state_var(vtest::State::bias) = bias_var;
		state_var(vtest::State::pos_rel) = pos_var;
		filter.setStateCovarianceDiag(state_var);

		// Bias initialization is a state reset: restart OOSM history to avoid mixing pre/post-reset states.
		filter.resetHistory();
	}

	PX4_DEBUG("Rel pos init %.2f %.2f %.2f", (double)initial_position(vtest::Axis::x),
		  (double)initial_position(vtest::Axis::y), (double)initial_position(vtest::Axis::z));

	PX4_DEBUG("GNSS bias init %.2f %.2f %.2f", (double)initial_bias(vtest::Axis::x),
		  (double)initial_bias(vtest::Axis::y), (double)initial_bias(vtest::Axis::z));

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

bool VTEPosition::fuseActiveMeasurements(const matrix::Vector3f &vehicle_acc_ned, ObsValidMaskU &fusion_mask,
		const TargetObs observations[kObsTypeCount])
{
	const bool gnss_ready = !shouldBlockGnssFusionUntilBiasReady();
	const bool vision_ready = !shouldBlockVisionFusionUntilBiasReady();
	bool position_fused = false;

	if (fusion_mask.flags.fuse_target_gps_pos && gnss_ready) {
		const TargetObs &obs = observations[obsIndex(ObsType::kTargetGpsPos)];

		if ((obs.timestamp != 0) && fuseMeas(vehicle_acc_ned, obs)) {
			position_fused = true;

			if (!_bias.set) {
				_pre_bias_reference = PreBiasReference::kGnss;
			}
		}
	}

	if (fusion_mask.flags.fuse_mission_pos && gnss_ready) {
		const TargetObs &obs = observations[obsIndex(ObsType::kMissionGpsPos)];

		if ((obs.timestamp != 0) && fuseMeas(vehicle_acc_ned, obs)) {
			position_fused = true;

			if (!_bias.set) {
				_pre_bias_reference = PreBiasReference::kGnss;
			}
		}
	}

	if (fusion_mask.flags.fuse_vision && vision_ready) {
		const TargetObs &obs = observations[obsIndex(ObsType::kFiducialMarker)];

		if ((obs.timestamp != 0) && fuseMeas(vehicle_acc_ned, obs)) {
			position_fused = true;

			if (!_bias.set) {
				_pre_bias_reference = PreBiasReference::kVision;
			}

			_last_relative_meas_fused_time = nowUs();
		}
	}

	if (fusion_mask.flags.fuse_uav_gps_vel) {
		const TargetObs &obs = observations[obsIndex(ObsType::kUavGpsVel)];

		if (obs.timestamp != 0) {
			fuseMeas(vehicle_acc_ned, obs);
		}
	}

#if defined(CONFIG_VTEST_MOVING)

	if (fusion_mask.flags.fuse_target_gps_vel) {
		const TargetObs &obs = observations[obsIndex(ObsType::kTargetGpsVel)];

		if (obs.timestamp != 0) {
			fuseMeas(vehicle_acc_ned, obs);
		}
	}

#endif // CONFIG_VTEST_MOVING

	return position_fused;
}

/*Vision observation: [rx, ry, rz]*/
bool VTEPosition::processObsVision(TargetObs &obs)
{
	fiducial_marker_pos_report_s report;

	if (!_fiducial_marker_pos_report_sub.update(&report) || !isVisionDataValid(report)) {
		return false;
	}

	const matrix::Quaternionf quat_att(report.q);
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

	obs.timestamp = report.timestamp_sample;

	obs.type = ObsType::kFiducialMarker;
	obs.updated(vtest::Axis::x) = true;
	obs.updated(vtest::Axis::y) = true;
	obs.updated(vtest::Axis::z) = true;

	// Assume noise correlation negligible:
	obs.meas_h_xyz(vtest::Axis::x, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(vtest::Axis::y, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(vtest::Axis::z, vtest::State::pos_rel) = 1;

	obs.meas_xyz = vision_ned;

	// Assume off diag elements ~ 0
	obs.meas_unc_xyz(vtest::Axis::x) = cov_rotated(vtest::Axis::x, vtest::Axis::x);
	obs.meas_unc_xyz(vtest::Axis::y) = cov_rotated(vtest::Axis::y, vtest::Axis::y);
	obs.meas_unc_xyz(vtest::Axis::z) = cov_rotated(vtest::Axis::z, vtest::Axis::z);

	return true;
}

/*Drone GNSS velocity observation: [vel_uav_x, vel_uav_y ,vel_uav_z]*/
bool VTEPosition::processObsGNSSVelUav(TargetObs &obs) const
{
	Vector3f vel_uav_ned{};

	if (!getOffsetUavVel(_uav_gps_vel.timestamp, vel_uav_ned)) {
		return false;
	}

	const float min_gps_vel_var = getMinGpsVelVar();

	obs.meas_xyz = vel_uav_ned;

	obs.meas_h_xyz(vtest::Axis::x, vtest::State::vel_uav) = 1;
	obs.meas_h_xyz(vtest::Axis::y, vtest::State::vel_uav) = 1;
	obs.meas_h_xyz(vtest::Axis::z, vtest::State::vel_uav) = 1;

	const float unc = fmaxf(sq(_uav_gps_vel.uncertainty), min_gps_vel_var);
	obs.meas_unc_xyz(vtest::Axis::x) = unc;
	obs.meas_unc_xyz(vtest::Axis::y) = unc;
	obs.meas_unc_xyz(vtest::Axis::z) = unc;

	obs.timestamp = _uav_gps_vel.timestamp;

	obs.type = ObsType::kUavGpsVel;

	obs.updated(vtest::Axis::x) = true;
	obs.updated(vtest::Axis::y) = true;
	obs.updated(vtest::Axis::z) = true;

	return true;
}

#if defined(CONFIG_VTEST_MOVING)

/*Target GNSS velocity observation: [r_dotx, r_doty, r_dotz]*/
bool VTEPosition::processObsGNSSVelTarget(const target_gnss_s &target_gnss, TargetObs &obs) const
{
	// If the target is moving, the relative velocity is expressed as the drone velocity minus the target velocity.
	obs.meas_xyz(vtest::Axis::x) = target_gnss.vel_n_m_s;
	obs.meas_xyz(vtest::Axis::y) = target_gnss.vel_e_m_s;
	obs.meas_xyz(vtest::Axis::z) = target_gnss.vel_d_m_s;

	const float unc = fmaxf(sq(target_gnss.s_acc_m_s), getMinGpsVelVar());

	obs.meas_unc_xyz(vtest::Axis::x) = unc;
	obs.meas_unc_xyz(vtest::Axis::y) = unc;
	obs.meas_unc_xyz(vtest::Axis::z) = unc;

	obs.meas_h_xyz(vtest::Axis::x, vtest::State::vel_target) = 1;
	obs.meas_h_xyz(vtest::Axis::y, vtest::State::vel_target) = 1;
	obs.meas_h_xyz(vtest::Axis::z, vtest::State::vel_target) = 1;

	obs.timestamp = target_gnss.timestamp_sample;

	obs.type = ObsType::kTargetGpsVel;

	obs.updated(vtest::Axis::x) = true;
	obs.updated(vtest::Axis::y) = true;
	obs.updated(vtest::Axis::z) = true;

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
		    || !isTimeDifferenceWithin(_gps_pos_offset_ned.timestamp, _uav_gps_position.timestamp, _meas_updated_timeout_us)) {
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

	// GPS already in NED, no rotation required.
	// Obs: [pos_rel + bias]
	obs.meas_h_xyz(vtest::Axis::x, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(vtest::Axis::y, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(vtest::Axis::z, vtest::State::pos_rel) = 1;

	obs.timestamp = _uav_gps_position.timestamp;

	obs.meas_xyz = gps_relative_pos;

	obs.meas_unc_xyz(vtest::Axis::x) = gps_unc_horizontal;
	obs.meas_unc_xyz(vtest::Axis::y) = gps_unc_horizontal;
	obs.meas_unc_xyz(vtest::Axis::z) = gps_unc_vertical;

	if (_bias.set) {
		obs.meas_h_xyz(vtest::Axis::x, vtest::State::bias) = 1;
		obs.meas_h_xyz(vtest::Axis::y, vtest::State::bias) = 1;
		obs.meas_h_xyz(vtest::Axis::z, vtest::State::bias) = 1;
	}

	obs.type = ObsType::kMissionGpsPos;

	obs.updated(vtest::Axis::x) = true;
	obs.updated(vtest::Axis::y) = true;
	obs.updated(vtest::Axis::z) = true;

	// Keep track of the latest GNSS-relative position used for bias initialization.
	// Mission and target GPS position fusion are mutually exclusive, so the mission
	// path can refresh this cache unconditionally.
	_pos_rel_gnss.timestamp = obs.timestamp;
	_pos_rel_gnss.valid = gps_relative_pos.isAllFinite();
	_pos_rel_gnss.xyz = gps_relative_pos;

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
	const float dt_sync_s_abs = fabsf(dt_sync_s);
	bool uav_position_interpolated = false;

	// Compensate UAV motion during the latency interval
	static constexpr float kGnssSyncInterpolationMinTimeS = 0.1f;

	Vector3f uav_vel_ned{};

	if ((dt_sync_s_abs > kGnssSyncInterpolationMinTimeS)
	    && getOffsetUavVel(_uav_gps_position.timestamp, uav_vel_ned)) {

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
		    || !isTimeDifferenceWithin(_gps_pos_offset_ned.timestamp, _uav_gps_position.timestamp, _meas_updated_timeout_us)) {

			return false;
		}

		gps_relative_pos += _gps_pos_offset_ned.xyz;
	}

	// Var(aX - bY) = a^2 Var(X) + b^2Var(Y) - 2ab Cov(X,Y)
	const float propagation_unc = uav_position_interpolated
				      ? sq(_uav_gps_vel.uncertainty * dt_sync_s_abs) : 0.f;
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

	// GPS already in NED, no rotation required.
	// Obs: [pos_rel + bias]
	obs.meas_h_xyz(vtest::Axis::x, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(vtest::Axis::y, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(vtest::Axis::z, vtest::State::pos_rel) = 1;

	if (_bias.set) {
		obs.meas_h_xyz(vtest::Axis::x, vtest::State::bias) = 1;
		obs.meas_h_xyz(vtest::Axis::y, vtest::State::bias) = 1;
		obs.meas_h_xyz(vtest::Axis::z, vtest::State::bias) = 1;
	}

	obs.timestamp = target_gnss.timestamp_sample;

	obs.meas_xyz = gps_relative_pos;

	obs.meas_unc_xyz(vtest::Axis::x) = gps_unc_horizontal;
	obs.meas_unc_xyz(vtest::Axis::y) = gps_unc_horizontal;
	obs.meas_unc_xyz(vtest::Axis::z) = gps_unc_vertical;

	obs.type = ObsType::kTargetGpsPos;

	obs.updated(vtest::Axis::x) = true;
	obs.updated(vtest::Axis::y) = true;
	obs.updated(vtest::Axis::z) = true;

	// Keep track of the latest GNSS relative position used for bias initialization.
	_pos_rel_gnss.timestamp = obs.timestamp;
	_pos_rel_gnss.valid = gps_relative_pos.isAllFinite();
	_pos_rel_gnss.xyz = gps_relative_pos;

	return true;
}

bool VTEPosition::fuseMeas(const Vector3f &vehicle_acc_ned, const TargetObs &target_obs)
{
	perf_begin(_vte_update_perf);
	const float nis_threshold = fmaxf(_param_vte_pos_nis_thre.get(), kMinNisThreshold);

	vte_aid_source3d_s target_innov = {};
	target_innov.timestamp_sample = target_obs.timestamp;
	target_innov.timestamp = nowUs();
	target_innov.time_last_predict = _last_predict;
	target_innov.time_since_meas_ms = static_cast<float>(signedTimeDiffUs(_last_predict, target_obs.timestamp))
					  * kMicrosecondsToMilliseconds;

	bool any_axis_fused = false;
	uint8_t history_steps = 0;

	for (int j = 0; j < vtest::Axis::size; j++) {

		if (!target_obs.updated(j)) {
			target_innov.fusion_status[j] = static_cast<uint8_t>(FusionStatus::STATUS_IDLE);
			continue; // nothing to do for this axis
		}

		const KF_position::ScalarMeas meas_input{target_obs.timestamp, target_obs.meas_xyz(j),
				target_obs.meas_unc_xyz(j), target_obs.meas_h_xyz.row(j)};

		target_innov.observation[j] = meas_input.val;
		target_innov.observation_variance[j] = meas_input.unc;

		perf_begin(_vte_fusion_perf);
		const FusionResult result = _target_est_pos[j].fuseScalarAtTime(meas_input, _last_predict, nis_threshold);
		perf_end(_vte_fusion_perf);

		target_innov.innovation[j]          = result.innov;
		target_innov.innovation_variance[j] = result.innov_var;
		target_innov.test_ratio[j]          = result.test_ratio;
		target_innov.fusion_status[j]       = static_cast<uint8_t>(result.status);
		any_axis_fused |= (result.status == FusionStatus::STATUS_FUSED_CURRENT ||
				   result.status == FusionStatus::STATUS_FUSED_OOSM);

		if (result.status == FusionStatus::STATUS_FUSED_OOSM) {
			history_steps = math::max(result.history_steps, history_steps);
		}
	}

	target_innov.history_steps = history_steps;

	perf_end(_vte_update_perf);
	publishInnov(target_innov, target_obs.type);

	return any_axis_fused;
}

void VTEPosition::publishInnov(const vte_aid_source3d_s &target_innov, const ObsType type)
{
	// Publish innovations
	switch (type) {
	case ObsType::kTargetGpsPos:
		_vte_aid_gps_pos_target_pub.publish(target_innov);
		break;

	case ObsType::kMissionGpsPos:
		_vte_aid_gps_pos_mission_pub.publish(target_innov);
		break;

	case ObsType::kUavGpsVel:
		_vte_aid_gps_vel_uav_pub.publish(target_innov);
		break;

	case ObsType::kTargetGpsVel:
		_vte_aid_gps_vel_target_pub.publish(target_innov);
		break;

	case ObsType::kFiducialMarker:
		_vte_aid_fiducial_marker_pub.publish(target_innov);
		break;

	case ObsType::kTypeCount:
		break;
	}
}

void VTEPosition::publishTarget()
{
	_vte_state = {};
	_target_pose = {};

	_target_pose.timestamp = _last_predict;
	_vte_state.timestamp = _last_predict;

#if defined(CONFIG_VTEST_MOVING)
	_target_pose.is_static = false;
#else
	_target_pose.is_static = true;
#endif // CONFIG_VTEST_MOVING

	// Get state
	const EstimatorState &state_x = _target_est_pos[vtest::Axis::x].getState();
	const EstimatorState &state_y = _target_est_pos[vtest::Axis::y].getState();
	const EstimatorState &state_z = _target_est_pos[vtest::Axis::z].getState();

	const EstimatorState state_var_x = _target_est_pos[vtest::Axis::x].getStateCovarianceDiag();
	const EstimatorState state_var_y = _target_est_pos[vtest::Axis::y].getStateCovarianceDiag();
	const EstimatorState state_var_z = _target_est_pos[vtest::Axis::z].getStateCovarianceDiag();

	// Fill target relative pose
	_target_pose.x_rel = state_x(vtest::State::pos_rel);
	_target_pose.y_rel = state_y(vtest::State::pos_rel);
	_target_pose.z_rel = state_z(vtest::State::pos_rel);

	_target_pose.cov_x_rel = state_var_x(vtest::State::pos_rel);
	_target_pose.cov_y_rel = state_var_y(vtest::State::pos_rel);
	_target_pose.cov_z_rel = state_var_z(vtest::State::pos_rel);

	// Fill target relative velocity
	_target_pose.vx_rel = -state_x(vtest::State::vel_uav);
	_target_pose.vy_rel = -state_y(vtest::State::vel_uav);
	_target_pose.vz_rel = -state_z(vtest::State::vel_uav);

	_target_pose.cov_vx_rel = state_var_x(vtest::State::vel_uav);
	_target_pose.cov_vy_rel = state_var_y(vtest::State::vel_uav);
	_target_pose.cov_vz_rel = state_var_z(vtest::State::vel_uav);

#if defined(CONFIG_VTEST_MOVING)
	const EstimatorStateCovariance &state_cov_x = _target_est_pos[vtest::Axis::x].getStateCovariance();
	const EstimatorStateCovariance &state_cov_y = _target_est_pos[vtest::Axis::y].getStateCovariance();
	const EstimatorStateCovariance &state_cov_z = _target_est_pos[vtest::Axis::z].getStateCovariance();

	// If target is moving, relative velocity = vel_target - vel_uav
	_target_pose.vx_rel += state_x(vtest::State::vel_target);
	_target_pose.vy_rel += state_y(vtest::State::vel_target);
	_target_pose.vz_rel += state_z(vtest::State::vel_target);

	// Var(vel_target - vel_uav) = Var(vel_target) + Var(vel_uav) - 2 Cov(vel_uav, vel_target)
	_target_pose.cov_vx_rel += state_var_x(vtest::State::vel_target)
				   - 2.f * state_cov_x(vtest::State::vel_uav, vtest::State::vel_target);
	_target_pose.cov_vy_rel += state_var_y(vtest::State::vel_target)
				   - 2.f * state_cov_y(vtest::State::vel_uav, vtest::State::vel_target);
	_target_pose.cov_vz_rel += state_var_z(vtest::State::vel_target)
				   - 2.f * state_cov_z(vtest::State::vel_uav, vtest::State::vel_target);

#endif // CONFIG_VTEST_MOVING

	// Fill vision target estimator state
	_vte_state.rel_pos[0] = _target_pose.x_rel;
	_vte_state.rel_pos[1] = _target_pose.y_rel;
	_vte_state.rel_pos[2] = _target_pose.z_rel;

	_vte_state.cov_rel_pos[0] = _target_pose.cov_x_rel;
	_vte_state.cov_rel_pos[1] = _target_pose.cov_y_rel;
	_vte_state.cov_rel_pos[2] = _target_pose.cov_z_rel;

	// Fill uav velocity
	_vte_state.vel_uav[0] = state_x(vtest::State::vel_uav);
	_vte_state.vel_uav[1] = state_y(vtest::State::vel_uav);
	_vte_state.vel_uav[2] = state_z(vtest::State::vel_uav);

	_vte_state.cov_vel_uav[0] = state_var_x(vtest::State::vel_uav);
	_vte_state.cov_vel_uav[1] = state_var_y(vtest::State::vel_uav);
	_vte_state.cov_vel_uav[2] = state_var_z(vtest::State::vel_uav);

	_vte_state.bias[0] = state_x(vtest::State::bias);
	_vte_state.bias[1] = state_y(vtest::State::bias);
	_vte_state.bias[2] = state_z(vtest::State::bias);

	_vte_state.cov_bias[0] = state_var_x(vtest::State::bias);
	_vte_state.cov_bias[1] = state_var_y(vtest::State::bias);
	_vte_state.cov_bias[2] = state_var_z(vtest::State::bias);

#if defined(CONFIG_VTEST_MOVING)

	_vte_state.vel_target[0] = state_x(vtest::State::vel_target);
	_vte_state.vel_target[1] = state_y(vtest::State::vel_target);
	_vte_state.vel_target[2] = state_z(vtest::State::vel_target);

	_vte_state.cov_vel_target[0] = state_var_x(vtest::State::vel_target);
	_vte_state.cov_vel_target[1] = state_var_y(vtest::State::vel_target);
	_vte_state.cov_vel_target[2] = state_var_z(vtest::State::vel_target);

	_vte_state.acc_target[0] = state_x(vtest::State::acc_target);
	_vte_state.acc_target[1] = state_y(vtest::State::acc_target);
	_vte_state.acc_target[2] = state_z(vtest::State::acc_target);

	_vte_state.cov_acc_target[0] = state_var_x(vtest::State::acc_target);
	_vte_state.cov_acc_target[1] = state_var_y(vtest::State::acc_target);
	_vte_state.cov_acc_target[2] = state_var_z(vtest::State::acc_target);

#endif // CONFIG_VTEST_MOVING

	const Vector3f rel_pos{_target_pose.x_rel, _target_pose.y_rel, _target_pose.z_rel};
	const Vector3f rel_vel{_target_pose.vx_rel, _target_pose.vy_rel, _target_pose.vz_rel};
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

	PX4_INFO("  position state: initialized: %s, valid: %s, last predict/update age: %.3f / %.3f s",
		 yes_no(_estimator_initialized), yes_no(relative_position_valid), age_s(_last_predict), age_s(_last_update));
	PX4_INFO("    rel pos/vel/ekf2 vel valid: %s / %s / %s",
		 yes_no(_target_pose.rel_pos_valid),
		 yes_no(_target_pose.rel_vel_valid),
		 yes_no(_target_pose.rel_vel_ekf2_valid));

	if (_estimator_initialized) {
		const EstimatorState &state_x = _target_est_pos[vtest::Axis::x].getState();
		const EstimatorState &state_y = _target_est_pos[vtest::Axis::y].getState();
		const EstimatorState &state_z = _target_est_pos[vtest::Axis::z].getState();

		PX4_INFO("    rel pos: [%.3f, %.3f, %.3f]",
			 (double)state_x(vtest::State::pos_rel),
			 (double)state_y(vtest::State::pos_rel),
			 (double)state_z(vtest::State::pos_rel));
		PX4_INFO("    gnss bias: [%.3f, %.3f, %.3f]",
			 (double)state_x(vtest::State::bias),
			 (double)state_y(vtest::State::bias),
			 (double)state_z(vtest::State::bias));

#if defined(CONFIG_VTEST_MOVING)
		PX4_INFO("    target vel: [%.3f, %.3f, %.3f]",
			 (double)state_x(vtest::State::vel_target),
			 (double)state_y(vtest::State::vel_target),
			 (double)state_z(vtest::State::vel_target));
		PX4_INFO("    target acc: [%.3f, %.3f, %.3f]",
			 (double)state_x(vtest::State::acc_target),
			 (double)state_y(vtest::State::acc_target),
			 (double)state_z(vtest::State::acc_target));
#endif // CONFIG_VTEST_MOVING
	}

	PX4_INFO("    bias set: %s, averaging: %s, pre-bias reference: %s",
		 yes_no(_bias.set),
		 yes_no(_bias.averaging_active),
		 pre_bias_reference_name());
	PX4_INFO("  position inputs: local pos: %s age %.3f s, local vel: %s age %.3f s",
		 yes_no(_local_position.valid), age_s(_local_position.timestamp),
		 yes_no(_local_velocity.valid), age_s(_local_velocity.timestamp));
	PX4_INFO("    uav gps pos: %s age %.3f s, uav gps vel: %s age %.3f s, mission pos: %s",
		 yes_no(_uav_gps_position.valid), age_s(_uav_gps_position.timestamp),
		 yes_no(_uav_gps_vel.valid), age_s(_uav_gps_vel.timestamp),
		 yes_no(_mission_land_position.valid));
	PX4_INFO("    gnss rel pos: %s age %.3f s, gps pos offset: %s age %.3f s, vel offset: %s age %.3f s",
		 yes_no(_pos_rel_gnss.valid), age_s(_pos_rel_gnss.timestamp),
		 yes_no(_gps_pos_is_offset && _gps_pos_offset_ned.valid), age_s(_gps_pos_offset_ned.timestamp),
		 yes_no(_velocity_offset_ned.valid), age_s(_velocity_offset_ned.timestamp));

#if defined(CONFIG_VTEST_MOVING)
	PX4_INFO("    target gps vel: %s age %.3f s",
		 yes_no(_target_gps_vel.valid), age_s(_target_gps_vel.timestamp));
#endif // CONFIG_VTEST_MOVING
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
