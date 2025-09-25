/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

#include "VTEPosition.h"

#define SEC2USEC_F 1e6f

namespace vision_target_estimator
{

using namespace matrix;

// Geographic limits
constexpr double kLatAbsMaxDeg =  90.0;
constexpr double kLonAbsMaxDeg = 180.0;
constexpr float kAltMinM = -350.f;
constexpr float kAltMaxM = 10000.f;
constexpr float kMinAbsMpcZVAutoDnMps = 1e-3f;
constexpr float kDefaultVisionUncDistanceM = 10.f;
constexpr float kMinTimesSyncNoInterpolationS = 0.1f;

VTEPosition::VTEPosition() :
	ModuleParams(nullptr)
{
	_targetPosePub.advertise();
	_targetEstimatorStatePub.advertise();
	_vte_aid_gps_pos_target_pub.advertise();
	_vte_aid_gps_pos_mission_pub.advertise();
	_vte_aid_gps_vel_uav_pub.advertise();
	_vte_aid_gps_vel_target_pub.advertise();
	_vte_aid_fiducial_marker_pub.advertise();

	updateParams();
}

VTEPosition::~VTEPosition()
{
	perf_free(_vte_predict_perf);
	perf_free(_vte_update_perf);
}

bool VTEPosition::shouldEmitWarning(hrt_abstime &last_warn)
{
	if ((last_warn == 0) || (hrt_elapsed_time(&last_warn) > kWarnThrottleIntervalUs)) {
		last_warn = hrt_absolute_time();
		return true;
	}

	return false;
}

bool VTEPosition::init()
{
	// Check valid vtest_derivation/generated/state.h
	if (vtest::Axis::size != 3) {
		PX4_ERR("VTE: Invalid axis size: %d, expected 3. Generate vtest_derivation/derivation.py",
			static_cast<int>(vtest::Axis::size));
		return false;
	}

#if defined(CONFIG_VTEST_MOVING)
	PX4_INFO("VTE for moving target init");
	PX4_WARN("VTE for moving targets has not been thoroughly tested");

	if (vtest::State::size != 5) {
		PX4_ERR("VTE: Invalid state size: %d, expected 5. Generate vtest_derivation/derivation.py",
			static_cast<int>(vtest::State::size));
		return false;
	}

	_param_mpc_z_v_auto_dn = param_find("MPC_Z_V_AUTO_DN");
#else
	PX4_INFO("VTE for static target init");

	if (vtest::State::size != 3) {
		PX4_ERR("VTE: Invalid state size: %d, expected 3. Generate vtest_derivation/derivation.py",
			static_cast<int>(vtest::State::size));
		return false;
	}

#endif // CONFIG_VTEST_MOVING

	for (int axis = 0; axis < vtest::Axis::size; ++axis) {
		_target_est_pos[axis] = KF_position{};
	}

	return true;
}

void VTEPosition::resetFilter()
{
	_estimator_initialized = false;
	_last_relative_meas_fused_time = 0;
	_bias_set = false;
	_mission_land_position.valid = false;
}

void VTEPosition::update(const Vector3f &acc_ned)
{
	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	checkMeasurementInputs();

	// Predict the target state using a constant relative-acceleration model.
	if (_estimator_initialized) {
		perf_begin(_vte_predict_perf);
		predictionStep(acc_ned);
		perf_end(_vte_predict_perf);

		_last_predict = hrt_absolute_time();
	}

	// Update with the newest observations and publish any resulting innovations.
	if (performUpdateStep(acc_ned)) {
		_last_update = _last_predict;
	}

	if (_estimator_initialized) {
		publishTarget();
	}
}

bool VTEPosition::initEstimator(const Matrix<float, vtest::Axis::size, vtest::State::size>
				&state_init)
{
	// Get initial variance from params
	const float state_pos_var = _param_vte_pos_unc_in.get();
	const float state_vel_var = _param_vte_vel_unc_in.get();
	const float state_bias_var = _param_vte_bias_unc_in.get();

	const Vector3f state_pos_var_vect(state_pos_var, state_pos_var, state_pos_var);
	const Vector3f state_vel_var_vect(state_vel_var, state_vel_var, state_vel_var);
	const Vector3f state_bias_var_vect(state_bias_var, state_bias_var, state_bias_var);

	matrix::Matrix <float, vtest::Axis::size, vtest::State::size> state_var_init;
	state_var_init.col(vtest::State::pos_rel) = state_pos_var_vect;
	state_var_init.col(vtest::State::vel_uav) = state_vel_var_vect;
	state_var_init.col(vtest::State::bias) = state_bias_var_vect;

#if defined(CONFIG_VTEST_MOVING)
	const float state_acc_var = _param_vte_acc_unc_in.get();
	const float state_target_vel_var = _param_vte_vel_unc_in.get();
	const Vector3f state_acc_var_vect(state_acc_var, state_acc_var, state_acc_var);
	const Vector3f state_target_vel_var_vect(state_target_vel_var, state_target_vel_var, state_target_vel_var);
	state_var_init.col(vtest::State::acc_target) = state_acc_var_vect;
	state_var_init.col(vtest::State::vel_target) = state_target_vel_var_vect;

#endif // CONFIG_VTEST_MOVING

	for (int i = 0; i < vtest::Axis::size; i++) {

		_target_est_pos[i].set_state(state_init.row(i));
		_target_est_pos[i].set_state_covariance(state_var_init.row(i));
	}

	// Debug INFO
	PX4_INFO("Rel pos init %.2f %.2f %.2f", (double)state_init(vtest::Axis::x, vtest::State::pos_rel),
		 (double)state_init(vtest::Axis::y, vtest::State::pos_rel), (double)state_init(vtest::Axis::z,
				 vtest::State::pos_rel));
	PX4_INFO("Vel uav init %.2f %.2f %.2f", (double)state_init(vtest::Axis::x, vtest::State::vel_uav),
		 (double)state_init(vtest::Axis::y, vtest::State::vel_uav), (double)state_init(vtest::Axis::z,
				 vtest::State::vel_uav));
	PX4_INFO("GNSS bias init %.2f %.2f %.2f", (double)state_init(vtest::Axis::x, vtest::State::bias),
		 (double)state_init(vtest::Axis::y, vtest::State::bias), (double)state_init(vtest::Axis::z,
				 vtest::State::bias));

#if defined(CONFIG_VTEST_MOVING)
	PX4_INFO("Target acc init %.2f %.2f %.2f", (double)state_init(vtest::Axis::x,
			vtest::State::acc_target),
		 (double)state_init(vtest::Axis::y, vtest::State::acc_target), (double)state_init(vtest::Axis::z,
				 vtest::State::acc_target));
	PX4_INFO("Target vel init %.2f %.2f %.2f", (double)state_init(vtest::Axis::x,
			vtest::State::vel_target),
		 (double)state_init(vtest::Axis::y, vtest::State::vel_target), (double)state_init(vtest::Axis::z,
				 vtest::State::vel_target));
#endif // CONFIG_VTEST_MOVING

	return true;
}

void VTEPosition::predictionStep(const Vector3f &vehicle_acc_ned)
{
	// Time since the last prediction
	const float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC_F;

	//Decoupled dynamics, we neglect the off diag elements.
	for (int i = 0; i < vtest::Axis::size; i++) {

#if defined(CONFIG_VTEST_MOVING)
		_target_est_pos[i].set_target_acc_var(_target_acc_unc);
#endif // CONFIG_VTEST_MOVING

		// We assume an isotropic uncertainty (cov = sigma^2 * I)
		// The rotated input covariance from body to NED R*(sigma^2 * I)*R^T = sigma^2*I because R*R^T = I.
		_target_est_pos[i].set_bias_var(_bias_unc);
		_target_est_pos[i].set_input_var(_uav_acc_unc);

		_target_est_pos[i].predictState(dt, vehicle_acc_ned(i));
		_target_est_pos[i].predictCov(dt);
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

	if (!_bias_set) {
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
	handleVisionData(fusion_mask, observations[obsIndex(ObsType::Fiducial_marker)]);

	if (updateUavGpsData()) {
		handleUavGpsData(fusion_mask,
				 observations[obsIndex(ObsType::Mission_gps_pos)],
				 observations[obsIndex(ObsType::Uav_gps_vel)]);
	}

	handleTargetGpsData(fusion_mask,
			    observations[obsIndex(ObsType::Target_gps_pos)],
			    observations[obsIndex(ObsType::Target_gps_vel)]);
}

void VTEPosition::handleVisionData(ObsValidMaskU &fusion_mask, TargetObs &vision_obs)
{
	if (!_vte_aid_mask.flags.use_vision_pos) {
		return;
	}

	fiducial_marker_pos_report_s report;

	if (!_fiducial_marker_pos_report_sub.update(&report) || !isVisionDataValid(report)) {
		return;
	}

	if (processObsVision(report, vision_obs)) {
		fusion_mask.flags.fuse_vision = true;
	}
}

bool VTEPosition::isVisionDataValid(const fiducial_marker_pos_report_s &fiducial_marker_pose) const
{
	const bool finite_measurement = PX4_ISFINITE(fiducial_marker_pose.x_rel_body)
					&& PX4_ISFINITE(fiducial_marker_pose.y_rel_body)
					&& PX4_ISFINITE(fiducial_marker_pose.z_rel_body);

	return finite_measurement && isMeasRecent(fiducial_marker_pose.timestamp);
}

void VTEPosition::handleUavGpsData(ObsValidMaskU &fusion_mask,
				   TargetObs &mission_pos_obs,
				   TargetObs &uav_vel_obs)
{
	if (_vte_aid_mask.flags.use_mission_pos
	    && _mission_land_position.valid
	    && processObsGNSSPosMission(mission_pos_obs)) {
		fusion_mask.flags.fuse_mission_pos = true;
	}

	if (_vte_aid_mask.flags.use_uav_gps_vel
	    && _uav_gps_vel.valid
	    && processObsGNSSVelUav(uav_vel_obs)) {
		fusion_mask.flags.fuse_uav_gps_vel = true;
	}
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
	if (!PX4_ISFINITE(_uav_gps_vel.xyz(vtest::Axis::x)) ||
	    !PX4_ISFINITE(_uav_gps_vel.xyz(vtest::Axis::y)) ||
	    !PX4_ISFINITE(_uav_gps_vel.xyz(vtest::Axis::z))) {
		if (shouldEmitWarning(_uav_gps_vel_warn_last)) {
			PX4_WARN("UAV GPS velocity not finite! vx: %.1f, vy: %.1f, vz: %.1f",
				 (double)_uav_gps_vel.xyz(vtest::Axis::x), (double)_uav_gps_vel.xyz(vtest::Axis::y),
				 (double)_uav_gps_vel.xyz(vtest::Axis::z));
		}

		return false;
	}

	if (!isMeasRecent(_uav_gps_vel.timestamp)) {
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
		_uav_gps_position.timestamp = vehicle_gps_position.timestamp;
		_uav_gps_position.eph = vehicle_gps_position.eph;
		_uav_gps_position.epv = vehicle_gps_position.epv;
		_uav_gps_position.valid = isUavGpsPositionValid();

		// Velocity
		_uav_gps_vel.timestamp = vehicle_gps_position.timestamp;
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

void VTEPosition::handleTargetGpsData(ObsValidMaskU &fusion_mask,
				      TargetObs &target_pos_obs,
				      TargetObs &target_vel_obs)
{
	target_gnss_s target_gnss{};

	if (!_target_gnss_sub.update(&target_gnss) || !isTargetGpsPositionValid(target_gnss)) {
		return;
	}

	if (_vte_aid_mask.flags.use_target_gps_pos
	    && _uav_gps_position.valid
	    && target_gnss.abs_pos_updated
	    && processObsGNSSPosTarget(target_gnss, target_pos_obs)) {
		fusion_mask.flags.fuse_target_gps_pos = true;
	}

#if defined(CONFIG_VTEST_MOVING)

	if (!isTargetGpsVelocityValid(target_gnss)) {
		return;
	}

	updateTargetGpsVelocity(target_gnss);

	if (_vte_aid_mask.flags.use_target_gps_vel && _target_gps_vel.valid
	    && ProcessObsGNSSVelTarget(target_gnss, target_vel_obs)) {
		fusion_mask.flags.fuse_target_gps_vel = true;
	}

#endif // CONFIG_VTEST_MOVING
}

#if defined(CONFIG_VTEST_MOVING)
void VTEPosition::updateTargetGpsVelocity(const target_gnss_s &target_gnss)
{
	_target_gps_vel.timestamp = target_gnss.timestamp;
	_target_gps_vel.valid = isMeasRecent(target_gnss.timestamp);

	_target_gps_vel.xyz(vtest::Axis::x) = target_gnss.vel_n_m_s;
	_target_gps_vel.xyz(vtest::Axis::y) = target_gnss.vel_e_m_s;
	_target_gps_vel.xyz(vtest::Axis::z) = target_gnss.vel_d_m_s;
}
#endif // CONFIG_VTEST_MOVING

bool VTEPosition::isTargetGpsPositionValid(const target_gnss_s &target_gnss)
{
	if (!isMeasRecent(target_gnss.timestamp)) {
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
	if (!isMeasRecent(target_gnss.timestamp)) {
		return false;
	}

	if (!target_gnss.vel_ned_updated ||
	    !PX4_ISFINITE(target_gnss.vel_n_m_s) ||
	    !PX4_ISFINITE(target_gnss.vel_e_m_s) ||
	    !PX4_ISFINITE(target_gnss.vel_d_m_s)) {
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

	// Check for initial velocity estimate
	const bool has_initial_velocity_estimate = (_local_velocity.valid && isMeasRecent(_local_velocity.timestamp)) ||
			(_uav_gps_vel.valid && isMeasRecent(_uav_gps_vel.timestamp));

	if (!has_initial_velocity_estimate) {
		if (shouldEmitWarning(_init_vel_warn_last)) {
			PX4_WARN("No UAV velocity estimate. Estimator cannot be started.");
		}

		return false;
	}

	// Initialize state variables
	Vector3f initial_position{};
	Vector3f initial_uav_velocity{};
	Vector3f initial_bias{};
#if defined(CONFIG_VTEST_MOVING)
	Vector3f initial_target_accel {};	// Assume null target absolute acceleration
	Vector3f initial_target_velocity{};
#endif // CONFIG_VTEST_MOVING

	// Get the initial position based on the current valid observations
	selectInitialPosition(fusion_mask, observations, initial_position);

	// Compute initial bias if needed
	if (shouldSetBias(fusion_mask)) {
		PX4_INFO("VTE Position setting GNSS bias.");
		initial_bias = _pos_rel_gnss.xyz - initial_position;
		_bias_set = true;
	}

#if defined(CONFIG_VTEST_MOVING)

	if (_target_gps_vel.valid && (isMeasRecent(_target_gps_vel.timestamp))) {
		initial_target_velocity = _target_gps_vel.xyz;
	}

#endif // CONFIG_VTEST_MOVING

	// Define initial UAV velocity
	if (_uav_gps_vel.valid && isMeasRecent(_uav_gps_vel.timestamp)) {
		initial_uav_velocity = _uav_gps_vel.xyz;

	} else if (_local_velocity.valid && isMeasRecent(_local_velocity.timestamp)) {
		initial_uav_velocity = _local_velocity.xyz;
	}

	// Initialize estimator state
	matrix::Matrix <float, vtest::Axis::size, vtest::State::size> state_init;
	state_init.col(vtest::State::pos_rel) = initial_position;
	state_init.col(vtest::State::vel_uav) = initial_uav_velocity;
	state_init.col(vtest::State::bias) = initial_bias;

#if defined(CONFIG_VTEST_MOVING)
	state_init.col(vtest::State::acc_target) = initial_target_accel;
	state_init.col(vtest::State::vel_target) = initial_target_velocity;
#endif // CONFIG_VTEST_MOVING

	if (!initEstimator(state_init)) {
		resetFilter();
		return false;
	}

	PX4_INFO("VTE Position Estimator properly initialized.");
	_estimator_initialized = true;
	_last_update = hrt_absolute_time();
	_last_predict = _last_update;
	return true;
}

void VTEPosition::selectInitialPosition(const ObsValidMaskU &fusion_mask,
					const TargetObs observations[kObsTypeCount], matrix::Vector3f &initial_position)
{
	// Non-GNSS observations must have priority as this function is also used to get the initial GNSS bias
	if (fusion_mask.flags.fuse_vision) {
		initial_position = observations[obsIndex(ObsType::Fiducial_marker)].meas_xyz;
		return;
	}

	if (fusion_mask.flags.fuse_target_gps_pos) {
		initial_position = observations[obsIndex(ObsType::Target_gps_pos)].meas_xyz;
		return;
	}

	if (fusion_mask.flags.fuse_mission_pos) {
		initial_position = observations[obsIndex(ObsType::Mission_gps_pos)].meas_xyz;
	}
}

void VTEPosition::updateBiasIfObservable(const ObsValidMaskU &fusion_mask,
		const TargetObs observations[kObsTypeCount])
{

	if (!shouldSetBias(fusion_mask)) {
		return;
	}

	PX4_INFO("Second relative position measurement available, setting position and bias.");

	// Get the initial position based on the current valid observations
	Vector3f initial_position{};
	selectInitialPosition(fusion_mask, observations, initial_position);

	// Compute the initial bias
	const Vector3f initial_bias = _pos_rel_gnss.xyz - initial_position;

	// Reset filter's state and variance
	const float pos_var = _param_vte_pos_unc_in.get();
	const float bias_var = _param_vte_bias_unc_in.get();

	for (int axis = 0; axis < vtest::Axis::size; ++axis) {
		KF_position &filter = _target_est_pos[axis];
		matrix::Vector<float, vtest::State::size>  state = filter.get_state();
		state(vtest::State::bias) = initial_bias(axis);
		state(vtest::State::pos_rel) = initial_position(axis);
		filter.set_state(state);

		matrix::Vector<float, vtest::State::size> state_var = filter.get_state_covariance();
		state_var(vtest::State::bias) = bias_var;
		state_var(vtest::State::pos_rel) = pos_var;
		filter.set_state_covariance(state_var);
	}

	PX4_INFO("Rel pos init %.2f %.2f %.2f", (double)initial_position(vtest::Axis::x),
		 (double)initial_position(vtest::Axis::y), (double)initial_position(vtest::Axis::z));

	PX4_INFO("GNSS bias init %.2f %.2f %.2f", (double)initial_bias(vtest::Axis::x),
		 (double)initial_bias(vtest::Axis::y), (double)initial_bias(vtest::Axis::z));

	_bias_set = true;
}

bool VTEPosition::fuseActiveMeasurements(const matrix::Vector3f &vehicle_acc_ned, ObsValidMaskU &fusion_mask,
		const TargetObs observations[kObsTypeCount])
{
	auto fuse_relative_obs = [&](ObsType type) {
		if (!fuseMeas(vehicle_acc_ned, observations[obsIndex(type)])) {
			return false;
		}

		_last_relative_meas_fused_time = hrt_absolute_time();
		return true;
	};

	bool position_fused = false;

	if (fusion_mask.flags.fuse_target_gps_pos) {
		position_fused |= fuseMeas(vehicle_acc_ned, observations[obsIndex(ObsType::Target_gps_pos)]);
	}

	if (fusion_mask.flags.fuse_mission_pos) {
		position_fused |= fuseMeas(vehicle_acc_ned, observations[obsIndex(ObsType::Mission_gps_pos)]);
	}

	if (fusion_mask.flags.fuse_vision) {
		position_fused |= fuse_relative_obs(ObsType::Fiducial_marker);
	}

	if (fusion_mask.flags.fuse_uav_gps_vel) {
		fuseMeas(vehicle_acc_ned, observations[obsIndex(ObsType::Uav_gps_vel)]);
	}

#if defined(CONFIG_VTEST_MOVING)

	if (fusion_mask.flags.fuse_target_gps_vel) {
		fuseMeas(vehicle_acc_ned, observations[obsIndex(ObsType::Target_gps_vel)]);
	}

#endif // CONFIG_VTEST_MOVING

	return position_fused;
}

/*Vision observation: [rx, ry, rz]*/
bool VTEPosition::processObsVision(const fiducial_marker_pos_report_s &report, TargetObs &obs)
{
	// Rotate vision observation from body FRD to vc-NED
	const matrix::Quaternionf quat_att(report.q);
	const Vector3f vision_body(report.x_rel_body, report.y_rel_body, report.z_rel_body);
	const Vector3f vision_ned = quat_att.rotateVector(vision_body);

	// Rotate covariance matrix to vc-NED
	SquareMatrix<float, vtest::Axis::size> cov_rotated;

	// Use uncertainty from parameters or from vision messages
	if (_ev_noise_md) {
		// Uncertainty proportional to the vertical distance
		const float meas_unc = _range_sensor.valid ? (_min_ev_pos_var * fmaxf(_range_sensor.dist_bottom, 1.f)) :
				       (_min_ev_pos_var * kDefaultVisionUncDistanceM);
		cov_rotated = diag(Vector3f(meas_unc, meas_unc, meas_unc));

	} else {
		const SquareMatrix<float, vtest::Axis::size> covMat = diag(Vector3f(
					fmaxf(report.var_x_rel_body, _min_ev_pos_var),
					fmaxf(report.var_y_rel_body, _min_ev_pos_var),
					fmaxf(report.var_z_rel_body, _min_ev_pos_var)));
		const matrix::Dcmf R_att = matrix::Dcm<float>(quat_att);
		cov_rotated = R_att * covMat * R_att.transpose();
	}

	// Relative position
	if (!PX4_ISFINITE(vision_ned(0)) || !PX4_ISFINITE(vision_ned(1)) || !PX4_ISFINITE(vision_ned(2))) {
		if (shouldEmitWarning(_vision_pos_warn_last)) {
			PX4_WARN("VTE: Vision position NED is corrupt!");
		}

		return false;
	}

	obs.timestamp = report.timestamp;

	obs.type = ObsType::Fiducial_marker;
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
	Vector3f vel_uav_ned = _uav_gps_vel.xyz;

	if (_gps_pos_is_offset) {
		if (!_velocity_offset_ned.valid
		    || !isTimeDifferenceWithin(_velocity_offset_ned.timestamp, _uav_gps_vel.timestamp, _meas_updated_timeout_us)) {
			return false;
		}

		vel_uav_ned -= _velocity_offset_ned.xyz;
	}

	obs.meas_xyz = vel_uav_ned;

	obs.meas_h_xyz(vtest::Axis::x, vtest::State::vel_uav) = 1;
	obs.meas_h_xyz(vtest::Axis::y, vtest::State::vel_uav) = 1;
	obs.meas_h_xyz(vtest::Axis::z, vtest::State::vel_uav) = 1;

	const float unc = fmaxf(math::sq(_uav_gps_vel.uncertainty), _min_gps_vel_var);
	obs.meas_unc_xyz(vtest::Axis::x) = unc;
	obs.meas_unc_xyz(vtest::Axis::y) = unc;
	obs.meas_unc_xyz(vtest::Axis::z) = unc;

	obs.timestamp = _uav_gps_vel.timestamp;

	obs.type = ObsType::Uav_gps_vel;

	obs.updated(vtest::Axis::x) = true;
	obs.updated(vtest::Axis::y) = true;
	obs.updated(vtest::Axis::z) = true;

	return true;
}

#if defined(CONFIG_VTEST_MOVING)

/*Target GNSS velocity observation: [r_dotx, r_doty, r_dotz]*/
bool VTEPosition::ProcessObsGNSSVelTarget(const target_gnss_s &target_gnss, TargetObs &obs) const
{

	// If the target is moving, the relative velocity is expressed as the drone verlocity - the target velocity
	obs.meas_xyz(vtest::Axis::x) = target_gnss.vel_n_m_s;
	obs.meas_xyz(vtest::Axis::y) = target_gnss.vel_e_m_s;
	obs.meas_xyz(vtest::Axis::z) = target_gnss.vel_d_m_s;

	const float unc = fmaxf(math::sq(target_gnss.s_variance_m_s), _min_gps_vel_var);

	obs.meas_unc_xyz(vtest::Axis::x) = unc;
	obs.meas_unc_xyz(vtest::Axis::y) = unc;
	obs.meas_unc_xyz(vtest::Axis::z) = unc;

	obs.meas_h_xyz(vtest::Axis::x, vtest::State::vel_target) = 1;
	obs.meas_h_xyz(vtest::Axis::y, vtest::State::vel_target) = 1;
	obs.meas_h_xyz(vtest::Axis::z, vtest::State::vel_target) = 1;

	obs.timestamp = target_gnss.timestamp;

	obs.type = ObsType::Target_gps_vel;

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

	const float gps_unc_horizontal = fmaxf(math::sq(_uav_gps_position.eph), _min_gps_pos_var);
	const float gps_unc_vertical = fmaxf(math::sq(_uav_gps_position.epv), _min_gps_pos_var);

	// GPS already in NED, no rotation required.
	// Obs: [pos_rel + bias]
	obs.meas_h_xyz(vtest::Axis::x, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(vtest::Axis::y, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(vtest::Axis::z, vtest::State::pos_rel) = 1;

	if (_bias_set) {
		obs.meas_h_xyz(vtest::Axis::x, vtest::State::bias) = 1;
		obs.meas_h_xyz(vtest::Axis::y, vtest::State::bias) = 1;
		obs.meas_h_xyz(vtest::Axis::z, vtest::State::bias) = 1;
	}

	obs.timestamp = _uav_gps_position.timestamp;

	obs.meas_xyz = gps_relative_pos;

	obs.meas_unc_xyz(vtest::Axis::x) = gps_unc_horizontal;
	obs.meas_unc_xyz(vtest::Axis::y) = gps_unc_horizontal;
	obs.meas_unc_xyz(vtest::Axis::z) = gps_unc_vertical;

	obs.type = ObsType::Mission_gps_pos;

	obs.updated(vtest::Axis::x) = true;
	obs.updated(vtest::Axis::y) = true;
	obs.updated(vtest::Axis::z) = true;

	// If the target GPS fusion is enabled give it priority over the GPS relative position measurement.
	if (!_vte_aid_mask.flags.use_target_gps_pos || !(isMeasRecent(_pos_rel_gnss.timestamp))) {
		_pos_rel_gnss.timestamp = obs.timestamp;
		_pos_rel_gnss.valid = (PX4_ISFINITE(gps_relative_pos(0)) && PX4_ISFINITE(gps_relative_pos(1))
				       && PX4_ISFINITE(gps_relative_pos(2)));
		_pos_rel_gnss.xyz = gps_relative_pos;
	}

	return true;
}

/*Target GNSS observation: [rx + bx, ry + by, rz + bz]*/
bool VTEPosition::processObsGNSSPosTarget(const target_gnss_s &target_gnss, TargetObs &obs)
{
	const int64_t time_diff_us = static_cast<int64_t>(target_gnss.timestamp)
				     - static_cast<int64_t>(_uav_gps_position.timestamp);
	const float dt_sync_us = fabsf(static_cast<float>(time_diff_us));

	if (dt_sync_us > _meas_recent_timeout_us) {
		PX4_INFO("Time diff between UAV GNSS and target GNNS too high: %.2f [ms] > timeout: %.2f [ms]",
			 (double)(dt_sync_us / 1000), (double)(_meas_recent_timeout_us / 1000));
		return false;
	}

	double uav_lat_deg = _uav_gps_position.lat_deg;
	double uav_lon_deg = _uav_gps_position.lon_deg;
	float uav_alt_m = _uav_gps_position.alt_m;

	const float dt_sync_s = static_cast<float>(time_diff_us) / SEC2USEC_F;
	const float dt_sync_s_abs = fabsf(dt_sync_s);
	bool uav_position_interpolated = false;

	// Compensate UAV motion during the latency interval
	if (dt_sync_s_abs > kMinTimesSyncNoInterpolationS &&
	    _uav_gps_vel.valid
	    && isTimeDifferenceWithin(_uav_gps_vel.timestamp, _uav_gps_position.timestamp, _meas_updated_timeout_us)) {
		const matrix::Vector3f uav_vel_ned = _uav_gps_vel.xyz;
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
				      ? math::sq(_uav_gps_vel.uncertainty * dt_sync_s_abs) : 0.f;
	const float gps_unc_horizontal = fmaxf(math::sq(_uav_gps_position.eph), _min_gps_pos_var)
					 + fmaxf(math::sq(target_gnss.eph), _min_gps_pos_var)
					 + propagation_unc;
	const float gps_unc_vertical = fmaxf(math::sq(_uav_gps_position.epv), _min_gps_pos_var)
				       + fmaxf(math::sq(target_gnss.epv), _min_gps_pos_var)
				       + propagation_unc;

	// GPS already in NED, no rotation required.
	// Obs: [pos_rel + bias]
	obs.meas_h_xyz(vtest::Axis::x, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(vtest::Axis::y, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(vtest::Axis::z, vtest::State::pos_rel) = 1;

	if (_bias_set) {
		obs.meas_h_xyz(vtest::Axis::x, vtest::State::bias) = 1;
		obs.meas_h_xyz(vtest::Axis::y, vtest::State::bias) = 1;
		obs.meas_h_xyz(vtest::Axis::z, vtest::State::bias) = 1;
	}

	obs.timestamp = target_gnss.timestamp;

	obs.meas_xyz = gps_relative_pos;

	obs.meas_unc_xyz(vtest::Axis::x) = gps_unc_horizontal;
	obs.meas_unc_xyz(vtest::Axis::y) = gps_unc_horizontal;
	obs.meas_unc_xyz(vtest::Axis::z) = gps_unc_vertical;

	obs.type = ObsType::Target_gps_pos;

	obs.updated(vtest::Axis::x) = true;
	obs.updated(vtest::Axis::y) = true;
	obs.updated(vtest::Axis::z) = true;

	// Keep track of the gps relative position
	_pos_rel_gnss.timestamp = obs.timestamp;
	_pos_rel_gnss.valid = (PX4_ISFINITE(gps_relative_pos(0)) && PX4_ISFINITE(gps_relative_pos(1))
			       && PX4_ISFINITE(gps_relative_pos(2)));
	_pos_rel_gnss.xyz = gps_relative_pos;

	return true;
}

bool VTEPosition::fuseMeas(const Vector3f &vehicle_acc_ned, const TargetObs &target_obs)
{
	perf_begin(_vte_update_perf);

	_target_innov = {};
	bool all_axis_fused = true;

	_target_innov.time_last_fuse = _last_predict;
	_target_innov.timestamp_sample = target_obs.timestamp;
	_target_innov.timestamp = hrt_absolute_time();

	// Measurement's time delay (difference between state and measurement time on validity)
	const int64_t dt_sync_us = signedTimeDiffUs(_last_predict, target_obs.timestamp);

	// Reject old measurements or measurements in the "future" due to bad time sync
	if (dt_sync_us > static_cast<int64_t>(_meas_recent_timeout_us) || dt_sync_us < 0) {

		PX4_DEBUG("Obs type: %d too old or in the future. Time sync: %.2f [ms] (timeout: %.2f [ms])",
			  static_cast<int>(target_obs.type),
			  static_cast<double>(dt_sync_us) / 1000., static_cast<double>(_meas_recent_timeout_us) / 1000.);

		_target_innov.fused = false;
		perf_end(_vte_update_perf);
		publishInnov(_target_innov, target_obs.type);
		return false;
	}

	if (!target_obs.updated(vtest::Axis::x) && !target_obs.updated(vtest::Axis::y) && !target_obs.updated(vtest::Axis::z)) {
		all_axis_fused = false;
		PX4_DEBUG("Obs i = %d: non-valid", target_obs.type);
		_target_innov.fused = false;
		perf_end(_vte_update_perf);
		publishInnov(_target_innov, target_obs.type);
		return false;
	}

	const float dt_sync_s = static_cast<float>(dt_sync_us) / SEC2USEC_F;

	for (int j = 0; j < vtest::Axis::size; j++) {

		if (!target_obs.updated(j)) {
			continue; // nothing to do for this iteration
		}

		KF_position &est = _target_est_pos[j];

		const float meas_j     = target_obs.meas_xyz(j);
		const float meas_unc_j = target_obs.meas_unc_xyz(j);

		// Move state back to the measurement time of validity.
		est.syncState(dt_sync_s, vehicle_acc_ned(j));

		//Get the corresponding row of the H matrix.
		const Vector<float, vtest::State::size> meas_h_row = target_obs.meas_h_xyz.row(j);
		est.set_H(meas_h_row);

		_target_innov.innovation_variance[j] = est.computeInnovCov(meas_unc_j);
		_target_innov.innovation[j] = est.computeInnov(meas_j);

		// Set the Normalized Innovation Squared (NIS) check threshold. Used to reject outlier measurements
		est.set_nis_threshold(_nis_threshold);

		if (!est.update()) {
			all_axis_fused = false;
			PX4_DEBUG("Obs i = %d : not fused in direction: %d", static_cast<int>(target_obs.type), j);
		}

		_target_innov.observation[j] = meas_j;
		_target_innov.observation_variance[j] = meas_unc_j;
		_target_innov.test_ratio[j] = est.get_test_ratio();
	}

	_target_innov.fused = all_axis_fused;
	_target_innov.innovation_rejected = !all_axis_fused;

	perf_end(_vte_update_perf);
	publishInnov(_target_innov, target_obs.type);

	return all_axis_fused;
}

void VTEPosition::publishInnov(const estimator_aid_source3d_s &target_innov, const ObsType type)
{
	// Publish innovations
	switch (type) {
	case ObsType::Target_gps_pos:
		_vte_aid_gps_pos_target_pub.publish(target_innov);
		break;

	case ObsType::Mission_gps_pos:
		_vte_aid_gps_pos_mission_pub.publish(target_innov);
		break;

	case ObsType::Uav_gps_vel:
		_vte_aid_gps_vel_uav_pub.publish(target_innov);
		break;

	case ObsType::Target_gps_vel:
		_vte_aid_gps_vel_target_pub.publish(target_innov);
		break;

	case ObsType::Fiducial_marker:
		_vte_aid_fiducial_marker_pub.publish(target_innov);
		break;

	case ObsType::Type_count:
		break;
	}
}

void VTEPosition::publishTarget()
{
	_vte_state = {};
	_target_pose = {};

	_target_pose.timestamp = _last_predict;
	_vte_state.timestamp = _last_predict;

	_target_pose.rel_pos_valid = !hasTimedOut(_last_update, _target_valid_timeout_us);

#if defined(CONFIG_VTEST_MOVING)
	_target_pose.is_static = false;
#else
	_target_pose.is_static = true;
#endif // CONFIG_VTEST_MOVING

	// Get state
	matrix::Vector<float, vtest::State::size> state_x = _target_est_pos[vtest::Axis::x].get_state();
	matrix::Vector<float, vtest::State::size> state_y = _target_est_pos[vtest::Axis::y].get_state();
	matrix::Vector<float, vtest::State::size> state_z = _target_est_pos[vtest::Axis::z].get_state();

	matrix::Vector<float, vtest::State::size> state_var_x = _target_est_pos[vtest::Axis::x].get_state_covariance();
	matrix::Vector<float, vtest::State::size> state_var_y = _target_est_pos[vtest::Axis::y].get_state_covariance();
	matrix::Vector<float, vtest::State::size> state_var_z = _target_est_pos[vtest::Axis::z].get_state_covariance();

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
	// If target is moving, relative velocity = vel_target - vel_uav
	_target_pose.vx_rel += state_x(vtest::State::vel_target);
	_target_pose.vy_rel += state_y(vtest::State::vel_target);
	_target_pose.vz_rel += state_z(vtest::State::vel_target);

	// Var(aX + bY) = a^2 Var(x) + b^2 Var(y) + 2abCov(X,Y)
	_target_pose.cov_vx_rel += state_var_x(vtest::State::vel_target);
	_target_pose.cov_vy_rel += state_var_y(vtest::State::vel_target);
	_target_pose.cov_vz_rel += state_var_z(vtest::State::vel_target);

#endif // CONFIG_VTEST_MOVING

	// Fill vision target estimator state
	_vte_state.x_rel = _target_pose.x_rel;
	_vte_state.y_rel = _target_pose.y_rel;
	_vte_state.z_rel = _target_pose.z_rel;

	_vte_state.cov_x_rel = _target_pose.cov_x_rel;
	_vte_state.cov_y_rel = _target_pose.cov_y_rel;
	_vte_state.cov_z_rel = _target_pose.cov_z_rel;

	// Fill uav velocity
	_vte_state.vx_uav = state_x(vtest::State::vel_uav);
	_vte_state.vy_uav = state_y(vtest::State::vel_uav);
	_vte_state.vz_uav = state_z(vtest::State::vel_uav);

	_vte_state.cov_vx_uav = state_var_x(vtest::State::vel_uav);
	_vte_state.cov_vy_uav = state_var_y(vtest::State::vel_uav);
	_vte_state.cov_vz_uav = state_var_z(vtest::State::vel_uav);

	_vte_state.x_bias = state_x(vtest::State::bias);
	_vte_state.y_bias = state_y(vtest::State::bias);
	_vte_state.z_bias = state_z(vtest::State::bias);

	_vte_state.cov_x_bias = state_var_x(vtest::State::bias);
	_vte_state.cov_y_bias = state_var_y(vtest::State::bias);
	_vte_state.cov_z_bias = state_var_z(vtest::State::bias);

#if defined(CONFIG_VTEST_MOVING)

	_vte_state.vx_target = state_x(vtest::State::vel_target);
	_vte_state.vy_target = state_y(vtest::State::vel_target);
	_vte_state.vz_target = state_z(vtest::State::vel_target);

	_vte_state.cov_vx_target = state_var_x(vtest::State::vel_target);
	_vte_state.cov_vy_target = state_var_y(vtest::State::vel_target);
	_vte_state.cov_vz_target = state_var_z(vtest::State::vel_target);

	_vte_state.ax_target = state_x(vtest::State::acc_target);
	_vte_state.ay_target = state_y(vtest::State::acc_target);
	_vte_state.az_target = state_z(vtest::State::acc_target);

	_vte_state.cov_ax_target = state_var_x(vtest::State::acc_target);
	_vte_state.cov_ay_target = state_var_y(vtest::State::acc_target);
	_vte_state.cov_az_target = state_var_z(vtest::State::acc_target);

#endif // CONFIG_VTEST_MOVING

	// If the target is static, valid and vision obs was fused recently, use the relative to aid the EKF2 state estimation.
	// Check performed in EKF2 to use target vel: if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid)
	_target_pose.rel_vel_valid = _target_pose.is_static && _param_vte_ekf_aid.get() && _target_pose.rel_pos_valid &&
				     (hrt_absolute_time() - _last_relative_meas_fused_time) < _meas_recent_timeout_us;

	// Prec land does not check _target_pose.abs_pos_valid. Only send the target if abs pose valid.
	if (_local_position.valid && _target_pose.rel_pos_valid) {
		_target_pose.x_abs = _target_pose.x_rel + _local_position.xyz(0);
		_target_pose.y_abs = _target_pose.y_rel + _local_position.xyz(1);
		_target_pose.z_abs = _target_pose.z_rel + _local_position.xyz(2);
		_target_pose.abs_pos_valid = true;

#if defined(CONFIG_VTEST_MOVING)

		// If the target is moving, move towards its expected location
		float mpc_z_v_auto_dn = _mpc_z_v_auto_dn;

		if (fabsf(mpc_z_v_auto_dn) < kMinAbsMpcZVAutoDnMps) {
			mpc_z_v_auto_dn = (mpc_z_v_auto_dn >= 0.f) ? kMinAbsMpcZVAutoDnMps : -kMinAbsMpcZVAutoDnMps;
		}

		float intersection_time_s = fabsf(_target_pose.z_rel / mpc_z_v_auto_dn);
		intersection_time_s = math::constrain(intersection_time_s, _param_vte_moving_t_min.get(),
						      _param_vte_moving_t_max.get());

		// Anticipate where the target will be
		_target_pose.x_abs += _vte_state.vx_target * intersection_time_s;
		_target_pose.y_abs += _vte_state.vy_target * intersection_time_s;
		_target_pose.z_abs += _vte_state.vz_target * intersection_time_s;

#endif // CONFIG_VTEST_MOVING

		_targetPosePub.publish(_target_pose);

	}

	_targetEstimatorStatePub.publish(_vte_state);

	// TODO: decide what to do with Bias lim
	float bias_lim = _param_vte_bias_lim.get();

	if (((float)fabs(_vte_state.x_bias) > bias_lim
	     || (float)fabs(_vte_state.y_bias) > bias_lim || (float)fabs(_vte_state.z_bias) > bias_lim)) {

		PX4_DEBUG("Bias exceeds limit: %.2f bias x: %.2f bias y: %.2f bias z: %.2f", (double)bias_lim,
			  (double)_vte_state.x_bias, (double)_vte_state.y_bias, (double)_vte_state.z_bias);

		// resetFilter();
	}

}

void VTEPosition::checkMeasurementInputs()
{
	// Make sure range sensor, local position and local velocity are up to date.
	if (_range_sensor.valid) {
		_range_sensor.valid = isMeasUpdated(_range_sensor.timestamp);
	}

	if (_local_position.valid) {
		_local_position.valid = isMeasUpdated(_local_position.timestamp);
	}

	if (_local_velocity.valid) {
		_local_velocity.valid = isMeasUpdated(_local_velocity.timestamp);
	}

	if (_local_velocity.valid) {
		_local_velocity.valid = isMeasUpdated(_local_velocity.timestamp);
	}

#if defined(CONFIG_VTEST_MOVING)

	if (_target_gps_vel.valid) {
		_target_gps_vel.valid = isMeasUpdated(_target_gps_vel.timestamp);
	}

#endif // CONFIG_VTEST_MOVING

	if (_pos_rel_gnss.valid) {
		_pos_rel_gnss.valid = isMeasUpdated(_pos_rel_gnss.timestamp);
	}

	if (_velocity_offset_ned.valid) {
		_velocity_offset_ned.valid = isMeasUpdated(_velocity_offset_ned.timestamp);
	}

	if (_gps_pos_offset_ned.valid) {
		_gps_pos_offset_ned.valid = isMeasUpdated(_gps_pos_offset_ned.timestamp);
	}

	if (_uav_gps_vel.valid) {
		_uav_gps_vel.valid = isMeasUpdated(_uav_gps_vel.timestamp);
	}
}

void VTEPosition::set_gps_pos_offset(const matrix::Vector3f &xyz, const bool gps_is_offset)
{
	_gps_pos_is_offset = gps_is_offset;
	_gps_pos_offset_ned.xyz = xyz;
	_gps_pos_offset_ned.valid = PX4_ISFINITE(xyz(0))
				    && PX4_ISFINITE(xyz(1))
				    && PX4_ISFINITE(xyz(2));
	_gps_pos_offset_ned.timestamp = hrt_absolute_time();
}

void VTEPosition::set_vel_offset(const matrix::Vector3f &xyz)
{
	_velocity_offset_ned.xyz = xyz;
	_velocity_offset_ned.valid = PX4_ISFINITE(xyz(0))
				     && PX4_ISFINITE(xyz(1))
				     && PX4_ISFINITE(xyz(2));
	_velocity_offset_ned.timestamp = hrt_absolute_time();
}

void VTEPosition::set_range_sensor(const float dist, const bool valid, hrt_abstime timestamp)
{
	_range_sensor.valid = valid && isMeasUpdated(timestamp) && (PX4_ISFINITE(dist) && dist > 0.f);
	_range_sensor.dist_bottom = dist;
	_range_sensor.timestamp = timestamp;

}

void VTEPosition::set_local_velocity(const matrix::Vector3f &vel_xyz, const bool vel_valid, hrt_abstime timestamp)
{
	_local_velocity.xyz = vel_xyz;
	_local_velocity.valid = vel_valid && isMeasUpdated(timestamp);
	_local_velocity.timestamp = timestamp;
}

void VTEPosition::set_local_position(const matrix::Vector3f &xyz, const bool pos_valid, hrt_abstime timestamp)
{
	_local_position.xyz = xyz;
	_local_position.valid = pos_valid && isMeasUpdated(timestamp);
	_local_position.timestamp = timestamp;
}

void VTEPosition::set_mission_position(const double lat_deg, const double lon_deg, const float alt_m)
{
	if (_vte_aid_mask.flags.use_mission_pos) {
		_mission_land_position.lat_deg = lat_deg;
		_mission_land_position.lon_deg = lon_deg;
		_mission_land_position.alt_m = alt_m;
		_mission_land_position.valid = isLatLonAltValid(_mission_land_position.lat_deg, _mission_land_position.lon_deg,
					       _mission_land_position.alt_m, "Mission position ", &_mission_pos_warn_last);

		if (_mission_land_position.valid) {
			PX4_INFO("Mission position lat %.8f, lon %.8f [deg], alt %.1f [m]", lat_deg,
				 lon_deg, (double)(alt_m));

		} else {
			if (shouldEmitWarning(_mission_pos_status_warn_last)) {
				PX4_WARN("Mission position not used because not valid");
			}
		}

	} else {
		_mission_land_position.valid = false;
	}
}

void VTEPosition::updateParams()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	ModuleParams::updateParams();

	_target_acc_unc = _param_vte_acc_t_unc.get();
	_bias_unc = _param_vte_bias_unc.get();
	_uav_acc_unc = _param_vte_acc_d_unc.get();
	_ev_noise_md = _param_vte_ev_noise_md.get();
	const float new_gps_vel_noise = _param_vte_gps_vel_noise.get();
	const float new_gps_pos_noise = _param_vte_gps_pos_noise.get();
	const float new_ev_pos_noise = _param_vte_ev_pos_noise.get();
	const float new_nis_threshold = _param_vte_pos_nis_thre.get();

	if (PX4_ISFINITE(new_gps_pos_noise) && new_gps_pos_noise > kMinObservationNoise) {
		_min_gps_pos_var = new_gps_pos_noise * new_gps_pos_noise;

	} else {
		PX4_WARN("VTE: VTE_GPS_POS_NOISE %.1f <= %.1f, keeping previous value",
			 (double)new_gps_pos_noise, (double)kMinObservationNoise);
	}

	if (PX4_ISFINITE(new_gps_vel_noise) && new_gps_vel_noise > kMinObservationNoise) {
		_min_gps_vel_var = new_gps_vel_noise * new_gps_vel_noise;

	} else {
		PX4_WARN("VTE: VTE_GPS_VEL_NOISE %.1f <= %.1f, keeping previous value",
			 (double)new_gps_vel_noise, (double)kMinObservationNoise);
	}

	if (PX4_ISFINITE(new_ev_pos_noise) && new_ev_pos_noise > kMinObservationNoise) {
		_min_ev_pos_var = new_ev_pos_noise * new_ev_pos_noise;

	} else {
		PX4_WARN("VTE: VTE_EV_POS_NOISE %.1f <= %.1f, keeping previous value",
			 (double)new_ev_pos_noise, (double)kMinObservationNoise);
	}

	if (PX4_ISFINITE(new_nis_threshold) && new_nis_threshold > kMinNisThreshold) {
		_nis_threshold = new_nis_threshold;

	} else {
		PX4_WARN("VTE: VTE_POS_NIS_THRE %.1f <= %.1f, keeping previous value",
			 (double)new_nis_threshold, (double)kMinNisThreshold);
	}

#if defined(CONFIG_VTEST_MOVING)

	if (_param_mpc_z_v_auto_dn != PARAM_INVALID) {
		float new_mpc_z_v_auto_dn = _mpc_z_v_auto_dn;

		if (param_get(_param_mpc_z_v_auto_dn, &new_mpc_z_v_auto_dn) == PX4_OK) {
			_mpc_z_v_auto_dn = new_mpc_z_v_auto_dn;
		}
	}

#endif // CONFIG_VTEST_MOVING
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
