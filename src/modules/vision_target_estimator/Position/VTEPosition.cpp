/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
	_vte_aid_uwb_pub.advertise();

	updateParams();
}

VTEPosition::~VTEPosition()
{
	for (int i = 0; i < 3; i++) {
		delete _target_est_pos[i];
	}

	perf_free(_vte_predict_perf);
	perf_free(_vte_update_perf);
}

bool VTEPosition::init()
{
	// Check valid vtest_derivation/generated/state.h
#if defined(CONFIG_VTEST_MOVING)
	PX4_INFO("VTE for moving target init");
	PX4_WARN("VTE for moving targets has not been thoroughly tested");

	if (vtest::State::size != 5) {
		PX4_ERR("VTE: Invalid state size: %d, expected 5. Generate vtest_derivation/derivation.py",
			static_cast<int>(vtest::State::size));
		return false;
	}

#else
	PX4_INFO("VTE for static target init");

	if (vtest::State::size != 3) {
		PX4_ERR("VTE: Invalid state size: %d, expected 3. Generate vtest_derivation/derivation.py",
			static_cast<int>(vtest::State::size));
		return false;
	}

#endif // CONFIG_VTEST_MOVING

	if (!createEstimators()) {
		PX4_ERR("VTE position KF creation failed");
		return false;
	}

	return true;
}

void VTEPosition::reset_filter()
{
	_estimator_initialized = false;
	_last_vision_obs_fused_time = 0;
	_bias_set = false;
	_mission_land_position.valid = false;
	_has_timed_out = false;
}

void VTEPosition::update(const Vector3f &acc_ned)
{
	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	checkMeasurementInputs();

	// predict the target state using a constant relative acceleration model
	if (_estimator_initialized) {

		if (hrt_absolute_time() - _last_update > _vte_TIMEOUT_US) {
			PX4_WARN("VTE Position estimator has timed out");
			_has_timed_out = true;

		} else {
			perf_begin(_vte_predict_perf);
			predictionStep(acc_ned);
			perf_end(_vte_predict_perf);

			_last_predict = hrt_absolute_time();
		}
	}

	// Update and fuse the observations and publishes innovations
	if (updateStep(acc_ned)) {
		_last_update = _last_predict;
	}

	if (_estimator_initialized) {publishTarget();}
}

bool VTEPosition::initEstimator(const Matrix<float, Axis::Count, vtest::State::size>
				&state_init)
{
	// Get initial variance from params
	const float state_pos_var = _param_vte_pos_unc_in.get();
	const float state_vel_var = _param_vte_vel_unc_in.get();
	const float state_bias_var = _param_vte_bias_unc_in.get();

	const Vector3f state_pos_var_vect(state_pos_var, state_pos_var, state_pos_var);
	const Vector3f state_vel_var_vect(state_vel_var, state_vel_var, state_vel_var);
	const Vector3f state_bias_var_vect(state_bias_var, state_bias_var, state_bias_var);

	matrix::Matrix <float, Axis::Count, vtest::State::size> state_var_init;
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

	for (int i = 0; i < Axis::Count; i++) {

		_target_est_pos[i]->setState(state_init.row(i));
		_target_est_pos[i]->setStateVar(state_var_init.row(i));
	}

	// Debug INFO
	PX4_INFO("Rel pos init %.2f %.2f %.2f", (double)state_init(Axis::X, vtest::State::pos_rel),
		 (double)state_init(Axis::Y, vtest::State::pos_rel), (double)state_init(Axis::Z,
				 vtest::State::pos_rel));
	PX4_INFO("Vel uav init %.2f %.2f %.2f", (double)state_init(Axis::X, vtest::State::vel_uav),
		 (double)state_init(Axis::Y, vtest::State::vel_uav), (double)state_init(Axis::Z,
				 vtest::State::vel_uav));
	PX4_INFO("GNSS bias init %.2f %.2f %.2f", (double)state_init(Axis::X, vtest::State::bias),
		 (double)state_init(Axis::Y, vtest::State::bias), (double)state_init(Axis::Z,
				 vtest::State::bias));

#if defined(CONFIG_VTEST_MOVING)
	PX4_INFO("Target acc init %.2f %.2f %.2f", (double)state_init(Axis::X,
			vtest::State::acc_target),
		 (double)state_init(Axis::Y, vtest::State::acc_target), (double)state_init(Axis::Z,
				 vtest::State::acc_target));
	PX4_INFO("Target vel init %.2f %.2f %.2f", (double)state_init(Axis::X,
			vtest::State::vel_target),
		 (double)state_init(Axis::Y, vtest::State::vel_target), (double)state_init(Axis::Z,
				 vtest::State::vel_target));
#endif // CONFIG_VTEST_MOVING

	return true;
}

void VTEPosition::predictionStep(const Vector3f &vehicle_acc_ned)
{
	// Time from last prediciton
	const float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC_F;

	// The rotated input cov (from body to NED R*cov*R^T) is the same as the original input cov since input_cov = acc_unc * Identiy and R*R^T = Identity
	const SquareMatrix<float, Axis::Count> input_cov = diag(Vector3f(_uav_acc_unc, _uav_acc_unc, _uav_acc_unc));
	const SquareMatrix<float, Axis::Count> bias_cov = diag(Vector3f(_bias_unc, _bias_unc, _bias_unc));

#if defined(CONFIG_VTEST_MOVING)
	const SquareMatrix<float, Axis::Count> target_acc_cov = diag(Vector3f(_target_acc_unc, _target_acc_unc,
			_target_acc_unc));
#endif // CONFIG_VTEST_MOVING

	//Decoupled dynamics, we neglect the off diag elements.
	for (int i = 0; i < Axis::Count; i++) {

#if defined(CONFIG_VTEST_MOVING)
		_target_est_pos[i]->setTargetAccVar(target_acc_cov(i, i));
#endif // CONFIG_VTEST_MOVING

		_target_est_pos[i]->setBiasVar(bias_cov(i, i));
		_target_est_pos[i]->setInputAccVar(input_cov(i, i));

		_target_est_pos[i]->predictState(dt, vehicle_acc_ned(i));
		_target_est_pos[i]->predictCov(dt);
	}
}

bool VTEPosition::updateStep(const Vector3f &vehicle_acc_ned)
{

	// Update the observations and vte_fusion_aid_mask if any valid observation.
	targetObs observations[ObsType::Type_count];
	ObsValidMask vte_fusion_aid_mask = ObsValidMask::NO_VALID_DATA;
	processObservations(vte_fusion_aid_mask, observations);

	// No new observations --> no fusion.
	if (vte_fusion_aid_mask == ObsValidMask::NO_VALID_DATA) {
		return false;
	}

	// Check if we have new observations
	const bool new_pos_sensor = hasNewPositionSensorData(vte_fusion_aid_mask);

	// Only init estimator once we have a valid position observation
	if (!_estimator_initialized && !new_pos_sensor) {
		return false;
	}

	// Initialize estimator if not already initialized
	if (!_estimator_initialized &&
	    !initializeEstimator(vte_fusion_aid_mask, observations)) {
		return false;
	}

	// Update bias if needed.
	if (!_bias_set && shouldSetBias(vte_fusion_aid_mask)) {
		updateBias(vte_fusion_aid_mask, observations);
	}

	// Fuse new sensor data
	return fuseNewSensorData(vehicle_acc_ned, vte_fusion_aid_mask, observations);
}

void VTEPosition::processObservations(ObsValidMask &vte_fusion_aid_mask, targetObs obs[ObsType::Type_count])
{
	handleVisionData(vte_fusion_aid_mask, obs[ObsType::Fiducial_marker]);

	handleUwbData(vte_fusion_aid_mask, obs[ObsType::Uwb]);

	if (updateUavGpsData()) {
		handleUavGpsData(vte_fusion_aid_mask, obs[ObsType::Mission_gps_pos], obs[ObsType::Uav_gps_vel]);
	}

	handleTargetGpsData(vte_fusion_aid_mask, obs[ObsType::Target_gps_pos], obs[ObsType::Target_gps_vel]);
}

void VTEPosition::handleVisionData(ObsValidMask &vte_fusion_aid_mask, targetObs &obs_fiducial_marker)
{

	if (!(_vte_aid_mask & SensorFusionMask::USE_EXT_VIS_POS)) {
		return;
	}

	fiducial_marker_pos_report_s fiducial_marker_pose;

	if (!_fiducial_marker_report_sub.update(&fiducial_marker_pose)) {
		return;
	}

	if (!isVisionDataValid(fiducial_marker_pose)) {
		return;
	}

	if (processObsVision(fiducial_marker_pose, obs_fiducial_marker)) {
		vte_fusion_aid_mask = static_cast<ObsValidMask>(vte_fusion_aid_mask | ObsValidMask::FUSE_VISION);
	}
}

bool VTEPosition::isVisionDataValid(const fiducial_marker_pos_report_s &fiducial_marker_pose)
{
	// TODO: extend checks
	return isMeasValid(fiducial_marker_pose.timestamp);
}

void VTEPosition::handleUwbData(ObsValidMask &vte_fusion_aid_mask, targetObs &obs_uwb)
{
	sensor_uwb_s uwb_report;

	if (!(_vte_aid_mask & SensorFusionMask::USE_UWB)) {
		return;
	}

	if (!_sensor_uwb_sub.update(&uwb_report)) {
		return;
	}

	if (!isUwbDataValid(uwb_report)) {
		return;
	}

	if (processObsUwb(uwb_report, obs_uwb)) {
		vte_fusion_aid_mask = static_cast<ObsValidMask>(vte_fusion_aid_mask | ObsValidMask::FUSE_UWB);
	}
}

bool VTEPosition::isUwbDataValid(const sensor_uwb_s &uwb_report)
{
	if (!isMeasValid(uwb_report.timestamp)) {
		return false;
	}

	if (fabsf(uwb_report.aoa_azimuth_dev) > max_uwb_aoa_angle_degree ||
	    fabsf(uwb_report.aoa_elevation_dev) > max_uwb_aoa_angle_degree) {
		return false;
	}

	return true;

}

bool VTEPosition::processObsUwb(const sensor_uwb_s &uwb_report, targetObs &obs)
{

	// Convert degrees to radians
	const float theta_rad = math::radians(uwb_report.aoa_azimuth_dev);
	const float phi_rad = math::radians(uwb_report.aoa_elevation_dev);

	const float distance = uwb_report.distance;

	// Calculate the relative position components
	const float delta_z = -distance * cosf(phi_rad) * cosf(theta_rad); // Negative because Z is down in NED
	const float delta_y = distance * cosf(phi_rad) * sinf(theta_rad);
	const float delta_x = -distance * sinf(phi_rad);

	// Total position in NED frame
	const Vector3f pos_ned(uwb_report.offset_x + delta_x, uwb_report.offset_y + delta_y, uwb_report.offset_z + delta_z);

	obs.meas_xyz = pos_ned;

	obs.meas_h_xyz(Axis::X, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(Axis::Y, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(Axis::Z, vtest::State::pos_rel) = 1;

	// Variance of UWB Distance measurements is +/- 5 cm
	// Variance of UWB Angle of Arrival measurements is +/- 3° Degree
	const float unc = math::sq(distance * 0.02f) + 0.0004f;
	obs.meas_unc_xyz(Axis::X) = unc;
	obs.meas_unc_xyz(Axis::Y) = unc;
	obs.meas_unc_xyz(Axis::Z) = unc;

	obs.timestamp = uwb_report.timestamp;

	obs.type = ObsType::Uwb;
	obs.updated = true;

	return true;
}

void VTEPosition::handleUavGpsData(ObsValidMask &vte_fusion_aid_mask,
				   targetObs &obs_gps_pos_mission,
				   targetObs &obs_gps_vel_uav)
{

	/* Handle GPS mission pos*/
	if ((_vte_aid_mask & SensorFusionMask::USE_MISSION_POS) &&
	    _mission_land_position.valid &&
	    processObsGNSSPosMission(obs_gps_pos_mission)) {
		vte_fusion_aid_mask = static_cast<ObsValidMask>(vte_fusion_aid_mask | ObsValidMask::FUSE_MISSION_POS);
	}

	/* Handle GPS velocity */
	if ((_vte_aid_mask & SensorFusionMask::USE_UAV_GPS_VEL &&
	     _uav_gps_vel.valid &&
	     processObsGNSSVelUav(obs_gps_vel_uav)
	    )) {
		vte_fusion_aid_mask = static_cast<ObsValidMask>(vte_fusion_aid_mask | ObsValidMask::FUSE_UAV_GPS_VEL);
	}
}

bool VTEPosition::isUavGpsPositionValid()
{
	if (!isMeasValid(_uav_gps_position.timestamp)) {
		return false;
	}

	if (!isLatLonAltValid(_uav_gps_position.lat_deg, _uav_gps_position.lon_deg,
			      _uav_gps_position.alt_m, "UAV GPS ")) {
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
	if (!PX4_ISFINITE(_uav_gps_vel.xyz(Axis::X)) ||
	    !PX4_ISFINITE(_uav_gps_vel.xyz(Axis::Y)) ||
	    !PX4_ISFINITE(_uav_gps_vel.xyz(Axis::Z))) {
		PX4_WARN(" Uav Gps velocity not finite! vx: %.1f, vy: %.1f, vz: %.1f",
			 (double)_uav_gps_vel.xyz(Axis::X), (double)_uav_gps_vel.xyz(Axis::Y), (double)_uav_gps_vel.xyz(Axis::Z));
		return false;
	}

	if (!isMeasValid(_uav_gps_vel.timestamp)) {
		return false;
	}

	if (_gps_pos_is_offset && !_velocity_offset_ned.valid) {
		PX4_WARN("Uav Gps velocity invalid offset!"); // TODO: this message can spam
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
		_uav_gps_vel.xyz(Axis::X) = vehicle_gps_position.vel_n_m_s;
		_uav_gps_vel.xyz(Axis::Y) = vehicle_gps_position.vel_e_m_s;
		_uav_gps_vel.xyz(Axis::Z) = vehicle_gps_position.vel_d_m_s;
		_uav_gps_vel.uncertainty = vehicle_gps_position.s_variance_m_s;
		_uav_gps_vel.valid = vehicle_gps_position.vel_ned_valid && isUavGpsVelocityValid();

	} else {
		// Check if stored data is still valid
		_uav_gps_position.valid = _uav_gps_position.valid && isMeasValid(_uav_gps_position.timestamp);
		_uav_gps_vel.valid = _uav_gps_vel.valid && isMeasValid(_uav_gps_vel.timestamp);
	}

	return vehicle_gps_position_updated;
}

void VTEPosition::handleTargetGpsData(ObsValidMask &vte_fusion_aid_mask,
				      targetObs &obs_gps_pos_target,
				      targetObs &obs_gps_vel_target)
{
	target_gnss_s target_GNSS_report;

	if (!_target_gnss_sub.update(&target_GNSS_report)) {
		return;
	}

	if (!isTargetGpsPositionValid(target_GNSS_report)) {
		return;
	}

	if ((_vte_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) && _uav_gps_position.valid
	    && target_GNSS_report.abs_pos_updated) {

		if (processObsGNSSPosTarget(target_GNSS_report, obs_gps_pos_target)) {
			vte_fusion_aid_mask = static_cast<ObsValidMask>(vte_fusion_aid_mask |
					      ObsValidMask::FUSE_TARGET_GPS_POS);
		}
	}

#if defined(CONFIG_VTEST_MOVING)

	if (!isTargetGpsVelValid(target_GNSS_report)) {
		return;
	}

	updateTargetGpsVelocity(target_GNSS_report);

	if (_vte_aid_mask & SensorFusionMask::USE_TARGET_GPS_VEL && _target_gps_vel.valid) {

		if (processObsGNSSVelTarget(target_GNSS_report, obs_gps_vel_target)) {
			vte_fusion_aid_mask = static_cast<ObsValidMask>(vte_fusion_aid_mask |
					      ObsValidMask::FUSE_TARGET_GPS_VEL);
		}
	}

#endif // CONFIG_VTEST_MOVING
}

void VTEPosition::updateTargetGpsVelocity(const target_gnss_s &target_GNSS_report)
{
	_target_gps_vel.timestamp = target_GNSS_report.timestamp;
	_target_gps_vel.valid = isMeasValid(target_GNSS_report.timestamp);

	_target_gps_vel.xyz(Axis::X) = target_GNSS_report.vel_n_m_s;
	_target_gps_vel.xyz(Axis::Y) = target_GNSS_report.vel_e_m_s;
	_target_gps_vel.xyz(Axis::Z) = target_GNSS_report.vel_d_m_s;
}

bool VTEPosition::isTargetGpsPositionValid(const target_gnss_s &target_GNSS_report)
{
	if (!isMeasValid(target_GNSS_report.timestamp)) {
		return false;
	}

	if (!isLatLonAltValid(target_GNSS_report.latitude_deg, target_GNSS_report.longitude_deg,
			      target_GNSS_report.altitude_msl_m, "Target GPS ")) {
		return false;
	}

	return true;
}

bool VTEPosition::isTargetGpsVelocityValid(const target_gnss_s &target_GNSS_report)
{
	if (!isMeasValid(target_GNSS_report.timestamp)) {
		return false;
	}

	// Make sure measurement are valid
	if (!target_GNSS_report.vel_ned_updated ||
	    !PX4_ISFINITE(target_GNSS_report.vel_n_m_s) ||
	    !PX4_ISFINITE(target_GNSS_report.vel_e_m_s) ||
	    !PX4_ISFINITE(target_GNSS_report.vel_d_m_s)) {
		PX4_WARN("Target GPS velocity is corrupt!");
		return false;
	}

	return true;
}

bool VTEPosition::initializeEstimator(const ObsValidMask &vte_fusion_aid_mask,
				      const targetObs observations[ObsType::Type_count])
{
	if (!hasNewPositionSensorData(vte_fusion_aid_mask)) {
		return false;
	}

	// Check for initial velocity estimate
	const bool has_initial_velocity_estimate = (_local_velocity.valid && isMeasValid(_local_velocity.timestamp)) ||
			(_uav_gps_vel.valid && isMeasValid(_uav_gps_vel.timestamp));

	if (!has_initial_velocity_estimate) {
		PX4_WARN("No UAV velocity estimate. Estimator cannot be started.");
		return false;
	}

	// Initialize state variables
	Vector3f pos_init{};
	Vector3f uav_vel_init{};
	Vector3f bias_init{};
#if defined(CONFIG_VTEST_MOVING)
	Vector3f target_acc_init {};	// Assume null target absolute acceleration
	Vector3f target_vel_init{};
#endif // CONFIG_VTEST_MOVING

	// Get the initial position based on the current valid observations
	getPosInit(vte_fusion_aid_mask, observations, pos_init);

	// Compute initial bias if needed
	if (shouldSetBias(vte_fusion_aid_mask)) {
		PX4_INFO("VTE Position setting GNSS bias.");
		bias_init = _pos_rel_gnss.xyz - pos_init;
		_bias_set = true;
	}

#if defined(CONFIG_VTEST_MOVING)

	if (_target_gps_vel.valid && (isMeasValid(_target_gps_vel.timestamp))) {
		target_vel_init = _target_gps_vel.xyz;
	}

#endif // CONFIG_VTEST_MOVING

	// Define initial UAV velocity
	if (_uav_gps_vel.valid && isMeasValid(_uav_gps_vel.timestamp)) {
		uav_vel_init = _uav_gps_vel.xyz;

	} else if (_local_velocity.valid && isMeasValid(_local_velocity.timestamp)) {
		uav_vel_init = _local_velocity.xyz;
	}

	// Initialize estimator state
	matrix::Matrix <float, Axis::Count, vtest::State::size> state_init;
	state_init.col(vtest::State::pos_rel) = pos_init;
	state_init.col(vtest::State::vel_uav) = uav_vel_init;
	state_init.col(vtest::State::bias) = bias_init;

#if defined(CONFIG_VTEST_MOVING)
	state_init.col(vtest::State::acc_target) = target_acc_init;
	state_init.col(vtest::State::vel_target) = target_vel_init;
#endif // CONFIG_VTEST_MOVING

	if (!initEstimator(state_init)) {
		reset_filter();
		return false;
	}

	PX4_INFO("VTE Position Estimator properly initialized.");
	_estimator_initialized = true;
	_last_update = hrt_absolute_time();
	_last_predict = _last_update;
	return true;
}

void VTEPosition::getPosInit(const ObsValidMask &vte_fusion_aid_mask,
			     const targetObs observations[ObsType::Type_count], matrix::Vector3f &pos_init)
{

	// Non-GNSS observations must have the priority as this function is also used to get the initial GNSS bias
	if (vte_fusion_aid_mask & ObsValidMask::FUSE_VISION) {
		pos_init = observations[ObsType::Fiducial_marker].meas_xyz;

	} else if (vte_fusion_aid_mask & ObsValidMask::FUSE_UWB) {
		pos_init = observations[ObsType::Uwb].meas_xyz;

	} else if (vte_fusion_aid_mask & ObsValidMask::FUSE_TARGET_GPS_POS) {
		pos_init = observations[ObsType::Target_gps_pos].meas_xyz;

	} else if (vte_fusion_aid_mask & ObsValidMask::FUSE_MISSION_POS) {
		pos_init = observations[ObsType::Mission_gps_pos].meas_xyz;
	}

}

void VTEPosition::updateBias(const ObsValidMask &vte_fusion_aid_mask,
			     const targetObs observations[ObsType::Type_count])
{

	PX4_INFO("Second relative position measurement available, re-setting position and bias.");

	// Get the initial position based on the current valid observations
	Vector3f pos_init;
	getPosInit(vte_fusion_aid_mask, observations, pos_init);

	const float state_pos_var = _param_vte_pos_unc_in.get();
	const float state_bias_var = _param_vte_bias_unc_in.get();

	const Vector3f state_pos_var_vect(state_pos_var, state_pos_var, state_pos_var);
	const Vector3f state_bias_var_vect(state_bias_var, state_bias_var, state_bias_var);

	// Compute the initial bias
	const Vector3f bias_init = _pos_rel_gnss.xyz - pos_init;

	// Reset filter's state and variance
	for (int i = 0; i < Axis::Count; i++) {
		matrix::Vector<float, vtest::State::size> temp_state = _target_est_pos[i]->getState();
		temp_state(vtest::State::bias) = bias_init(i);
		temp_state(vtest::State::pos_rel) = pos_init(i);
		_target_est_pos[i]->setState(temp_state);

		matrix::Vector<float, vtest::State::size> temp_state_var = _target_est_pos[i]->getStateVar();
		temp_state_var(vtest::State::bias) = state_bias_var_vect(i);
		temp_state_var(vtest::State::pos_rel) = state_pos_var_vect(i);
		_target_est_pos[i]->setStateVar(temp_state_var);
	}

	_bias_set = true;

	PX4_INFO("Rel pos init %.2f %.2f %.2f", (double)pos_init(Axis::X),
		 (double)pos_init(Axis::Y), (double)pos_init(Axis::Z));

	PX4_INFO("GNSS bias init %.2f %.2f %.2f", (double)bias_init(Axis::X),
		 (double)bias_init(Axis::Y), (double)bias_init(Axis::Z));
}

bool VTEPosition::fuseNewSensorData(const matrix::Vector3f &vehicle_acc_ned, ObsValidMask &vte_fusion_aid_mask,
				    const targetObs observations[ObsType::Type_count])
{
	bool pos_fused = false;

	// Fuse target GPS position
	if (vte_fusion_aid_mask & ObsValidMask::FUSE_TARGET_GPS_POS) {
		pos_fused |= fuseMeas(vehicle_acc_ned, observations[ObsType::Target_gps_pos]);
	}

	// Fuse mission GPS position
	if (vte_fusion_aid_mask & ObsValidMask::FUSE_MISSION_POS) {
		pos_fused |= fuseMeas(vehicle_acc_ned, observations[ObsType::Mission_gps_pos]);
	}

	// Fuse vision position
	if (vte_fusion_aid_mask & ObsValidMask::FUSE_VISION) {
		if (fuseMeas(vehicle_acc_ned, observations[ObsType::Fiducial_marker])) {
			_last_vision_obs_fused_time = hrt_absolute_time();
			pos_fused = true;
		}
	}

	// Fuse uwb
	if (vte_fusion_aid_mask & ObsValidMask::FUSE_UWB) {
		pos_fused |= fuseMeas(vehicle_acc_ned, observations[ObsType::Uwb]);
	}

	// Fuse GPS relative velocity
	if (vte_fusion_aid_mask & ObsValidMask::FUSE_UAV_GPS_VEL) {
		fuseMeas(vehicle_acc_ned, observations[ObsType::Uav_gps_vel]);
	}

#if defined(CONFIG_VTEST_MOVING)

	// Fuse target GPS velocity
	if (vte_fusion_aid_mask & ObsValidMask::FUSE_TARGET_GPS_VEL) {
		fuseMeas(vehicle_acc_ned, observations[ObsType::Target_gps_vel]);
	}

#endif // CONFIG_VTEST_MOVING

	return pos_fused;
}

/*Vision observation: [rx, ry, rz]*/
bool VTEPosition::processObsVision(const fiducial_marker_pos_report_s &fiducial_marker_pose, targetObs &obs)
{
	/* Rotate vision observation from body FRD - to vc-NED */
	const matrix::Quaternionf quat_att(fiducial_marker_pose.q);
	const Vector3f vision_body(fiducial_marker_pose.x_rel_body, fiducial_marker_pose.y_rel_body,
				   fiducial_marker_pose.z_rel_body);
	const Vector3f vision_ned = quat_att.rotateVector(vision_body);

	// Rotate covariance matrix to vc-NED
	SquareMatrix<float, Axis::Count> Cov_rotated;

	// Use uncertainty from parameters or from vision messages
	if (_ev_noise_md) {
		// Uncertainty proportional to the vertical distance
		const float meas_uncertainty = _range_sensor.valid ? ((_ev_pos_noise * _ev_pos_noise) * fmaxf(_range_sensor.dist_bottom,
					       1.f)) : ((_ev_pos_noise * _ev_pos_noise) * 10);
		Cov_rotated = diag(Vector3f(meas_uncertainty, meas_uncertainty, meas_uncertainty));

	} else {
		const SquareMatrix<float, Axis::Count> covMat = diag(Vector3f(fmaxf(fiducial_marker_pose.var_x_rel_body,
				_ev_pos_noise * _ev_pos_noise),
				fmaxf(fiducial_marker_pose.var_y_rel_body, _ev_pos_noise * _ev_pos_noise),
				fmaxf(fiducial_marker_pose.var_z_rel_body, _ev_pos_noise * _ev_pos_noise)));
		const matrix::Dcmf R_att = matrix::Dcm<float>(quat_att);
		Cov_rotated = R_att * covMat * R_att.transpose();
	}

	/* RELATIVE POSITION*/
	if (!PX4_ISFINITE(vision_ned(0)) || !PX4_ISFINITE(vision_ned(1)) || !PX4_ISFINITE(vision_ned(2))) {
		PX4_WARN("VISION position is corrupt!");
		return false;
	}

	obs.timestamp = fiducial_marker_pose.timestamp;

	obs.type = ObsType::Fiducial_marker;
	obs.updated = true;

	// Assume noise correlation negligible:
	obs.meas_h_xyz(Axis::X, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(Axis::Y, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(Axis::Z, vtest::State::pos_rel) = 1;

	obs.meas_xyz = vision_ned;

	// Assume off diag elements ~ 0
	obs.meas_unc_xyz(Axis::X) = Cov_rotated(Axis::X, Axis::X);
	obs.meas_unc_xyz(Axis::Y) = Cov_rotated(Axis::Y, Axis::Y);
	obs.meas_unc_xyz(Axis::Z) = Cov_rotated(Axis::Z, Axis::Z);

	return true;
}

/*Drone GNSS velocity observation: [vel_uav_x, vel_uav_y ,vel_uav_z]*/
bool VTEPosition::processObsGNSSVelUav(targetObs &obs)
{
	Vector3f vel_uav_ned = _uav_gps_vel.xyz;

	if (_gps_pos_is_offset) {
		if (((_velocity_offset_ned.timestamp - _uav_gps_vel.timestamp) > meas_updated_TIMEOUT_US)) {

			return false;
		}

		vel_uav_ned -= _velocity_offset_ned.xyz;
	}

	obs.meas_xyz = vel_uav_ned;

	obs.meas_h_xyz(Axis::X, vtest::State::vel_uav) = 1;
	obs.meas_h_xyz(Axis::Y, vtest::State::vel_uav) = 1;
	obs.meas_h_xyz(Axis::Z, vtest::State::vel_uav) = 1;

	const float unc = fmaxf(_uav_gps_vel.uncertainty * _uav_gps_vel.uncertainty,
				_gps_vel_noise * _gps_vel_noise);
	obs.meas_unc_xyz(Axis::X) = unc;
	obs.meas_unc_xyz(Axis::Y) = unc;
	obs.meas_unc_xyz(Axis::Z) = unc;

	obs.timestamp = _uav_gps_vel.timestamp;

	obs.type = ObsType::Uav_gps_vel;
	obs.updated = true;

	return true;
}

#if defined(CONFIG_VTEST_MOVING)

/*Target GNSS velocity observation: [r_dotx, r_doty, r_dotz]*/
bool VTEPosition::processObsGNSSVelTarget(const target_gnss_s &target_GNSS_report, targetObs &obs)
{

	// If the target is moving, the relative velocity is expressed as the drone verlocity - the target velocity
	obs.meas_xyz(Axis::X) = target_GNSS_report.vel_n_m_s;
	obs.meas_xyz(Axis::Y) = target_GNSS_report.vel_e_m_s;
	obs.meas_xyz(Axis::Z) = target_GNSS_report.vel_d_m_s;

	const float unc = fmaxf(target_GNSS_report.s_variance_m_s * target_GNSS_report.s_variance_m_s,
				_gps_vel_noise * _gps_vel_noise);

	obs.meas_unc_xyz(Axis::X) = unc;
	obs.meas_unc_xyz(Axis::Y) = unc;
	obs.meas_unc_xyz(Axis::Z) = unc;

	obs.meas_h_xyz(Axis::X, vtest::State::vel_target) = 1;
	obs.meas_h_xyz(Axis::Y, vtest::State::vel_target) = 1;
	obs.meas_h_xyz(Axis::Z, vtest::State::vel_target) = 1;

	obs.timestamp = target_GNSS_report.timestamp;

	obs.type = ObsType::Target_gps_vel;
	obs.updated = true;

	return true;
}

#endif // CONFIG_VTEST_MOVING

/*Target GNSS mission observation: [rx + bx, ry + by, rz + bz]*/
bool VTEPosition::processObsGNSSPosMission(targetObs &obs)
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

		if ((_gps_pos_offset_ned.timestamp - _uav_gps_position.timestamp) > meas_updated_TIMEOUT_US) {
			return false;
		}

		gps_relative_pos += _gps_pos_offset_ned.xyz;
	}

	const float gps_unc_horizontal = fmaxf(_uav_gps_position.eph * _uav_gps_position.eph,
					       _gps_pos_noise * _gps_pos_noise);
	const float gps_unc_vertical = fmaxf(_uav_gps_position.epv * _uav_gps_position.epv,
					     _gps_pos_noise * _gps_pos_noise);

	// GPS already in NED, no rotation required.
	// Obs: [pos_rel + bias]
	obs.meas_h_xyz(Axis::X, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(Axis::Y, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(Axis::Z, vtest::State::pos_rel) = 1;

	if (_bias_set) {
		obs.meas_h_xyz(Axis::X, vtest::State::bias) = 1;
		obs.meas_h_xyz(Axis::Y, vtest::State::bias) = 1;
		obs.meas_h_xyz(Axis::Z, vtest::State::bias) = 1;
	}

	obs.timestamp = _uav_gps_position.timestamp;

	obs.meas_xyz = gps_relative_pos;

	obs.meas_unc_xyz(Axis::X) = gps_unc_horizontal;
	obs.meas_unc_xyz(Axis::Y) = gps_unc_horizontal;
	obs.meas_unc_xyz(Axis::Z) = gps_unc_vertical;

	obs.type = ObsType::Mission_gps_pos;
	obs.updated = true;

	// If the target GPS fusion is enabled give it priority over the GPS relative position measurement.
	if (!(_vte_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) || !(isMeasValid(_pos_rel_gnss.timestamp))) {
		_pos_rel_gnss.timestamp = obs.timestamp;
		_pos_rel_gnss.valid = (PX4_ISFINITE(gps_relative_pos(0)) && PX4_ISFINITE(gps_relative_pos(1))
				       && PX4_ISFINITE(gps_relative_pos(2)));
		_pos_rel_gnss.xyz = gps_relative_pos;
	}

	return true;
}

/*Target GNSS observation: [rx + bx, ry + by, rz + bz]*/
bool VTEPosition::processObsGNSSPosTarget(const target_gnss_s &target_GNSS_report, targetObs &obs)
{
	// TODO: the target's position can have latency, interpolate the position of the UAV based on the time diff.
	const float dt_sync_us = fabsf((float)(_uav_gps_position.timestamp - target_GNSS_report.timestamp));

	if (dt_sync_us > meas_valid_TIMEOUT_US) {
		PX4_INFO("Time diff between UAV GNSS and target GNNS too high: %.2f [ms] > timeout: %.2f [ms]",
			 (double)(dt_sync_us / 1000), (double)(meas_valid_TIMEOUT_US / 1000));
		return false;
	}

	// Obtain GPS relative measurements in NED as target_global - uav_gps_global followed by global2local transformation
	Vector3f gps_relative_pos;
	get_vector_to_next_waypoint(_uav_gps_position.lat_deg, _uav_gps_position.lon_deg,
				    target_GNSS_report.latitude_deg, target_GNSS_report.longitude_deg,
				    &gps_relative_pos(0), &gps_relative_pos(1));

	// Down direction (if the drone is above the target, the relative position is positive)
	gps_relative_pos(2) = _uav_gps_position.alt_m - target_GNSS_report.altitude_msl_m;

	// Offset gps relative position to the center of mass:
	if (_gps_pos_is_offset) {
		if ((_gps_pos_offset_ned.timestamp - _uav_gps_position.timestamp) > meas_updated_TIMEOUT_US) {

			return false;
		}

		gps_relative_pos += _gps_pos_offset_ned.xyz;
	}

	// Var(aX - bY) = a^2 Var(X) + b^2Var(Y) - 2ab Cov(X,Y)
	const float gps_unc_horizontal = fmaxf(_uav_gps_position.eph * _uav_gps_position.eph,
					       _gps_pos_noise * _gps_pos_noise) + fmaxf(target_GNSS_report.eph * target_GNSS_report.eph,
							       _gps_pos_noise * _gps_pos_noise);
	const float gps_unc_vertical = fmaxf(_uav_gps_position.epv * _uav_gps_position.epv,
					     _gps_pos_noise * _gps_pos_noise) + fmaxf(target_GNSS_report.epv * target_GNSS_report.epv,
							     _gps_pos_noise * _gps_pos_noise);

	// GPS already in NED, no rotation required.
	// Obs: [pos_rel + bias]
	obs.meas_h_xyz(Axis::X, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(Axis::Y, vtest::State::pos_rel) = 1;
	obs.meas_h_xyz(Axis::Z, vtest::State::pos_rel) = 1;

	if (_bias_set) {
		obs.meas_h_xyz(Axis::X, vtest::State::bias) = 1;
		obs.meas_h_xyz(Axis::Y, vtest::State::bias) = 1;
		obs.meas_h_xyz(Axis::Z, vtest::State::bias) = 1;
	}

	obs.timestamp = target_GNSS_report.timestamp;

	obs.meas_xyz = gps_relative_pos;

	obs.meas_unc_xyz(Axis::X) = gps_unc_horizontal;
	obs.meas_unc_xyz(Axis::Y) = gps_unc_horizontal;
	obs.meas_unc_xyz(Axis::Z) = gps_unc_vertical;

	obs.type = ObsType::Target_gps_pos;
	obs.updated = true;

	// Keep track of the gps relative position
	_pos_rel_gnss.timestamp = obs.timestamp;
	_pos_rel_gnss.valid = (PX4_ISFINITE(gps_relative_pos(0)) && PX4_ISFINITE(gps_relative_pos(1))
			       && PX4_ISFINITE(gps_relative_pos(2)));
	_pos_rel_gnss.xyz = gps_relative_pos;

	return true;
}

// TODO: decide if we want to split the estimation into vertical and horizontal component
bool VTEPosition::fuseMeas(const Vector3f &vehicle_acc_ned, const targetObs &target_obs)
{
	perf_begin(_vte_update_perf);

	estimator_aid_source3d_s target_innov;
	bool all_axis_fused = true;

	target_innov.time_last_fuse = _last_predict;
	target_innov.timestamp_sample = target_obs.timestamp;
	target_innov.timestamp = hrt_absolute_time();

	// measurement's time delay (difference between state and measurement time on validity)
	const float dt_sync_us = (_last_predict - target_obs.timestamp);

	// Reject old measurements or measurements in the "future" due to bad time sync
	if (dt_sync_us > meas_valid_TIMEOUT_US || dt_sync_us < 0.f) {

		PX4_DEBUG("Obs type: %d too old or in the future. Time sync: %.2f [ms] (timeout: %.2f [ms])",
			  target_obs.type,
			  (double)(dt_sync_us / 1000), (double)(meas_valid_TIMEOUT_US / 1000));

		target_innov.fused = false;
		perf_end(_vte_update_perf);
		publishInnov(target_innov, target_obs.type);
		return false;
	}

	if (!target_obs.updated) {
		all_axis_fused = false;
		PX4_DEBUG("Obs i = %d: non-valid", target_obs.type);
		target_innov.fused = false;
		perf_end(_vte_update_perf);
		publishInnov(target_innov, target_obs.type);
		return false;
	}

	const float dt_sync_s = dt_sync_us / SEC2USEC_F;

	for (int j = 0; j < Axis::Count; j++) {

		// Move state back to the measurement time of validity. The state synchronized with the measurement will be used to compute the innovation.
		_target_est_pos[j]->syncState(dt_sync_s, vehicle_acc_ned(j));

		//Get the corresponding row of the H matrix.
		Vector<float, vtest::State::size> meas_h_row = target_obs.meas_h_xyz.row(j);
		_target_est_pos[j]->setH(meas_h_row);

		target_innov.innovation_variance[j] = _target_est_pos[j]->computeInnovCov(target_obs.meas_unc_xyz(j));
		target_innov.innovation[j] = _target_est_pos[j]->computeInnov(target_obs.meas_xyz(j));

		// Set the Normalized Innovation Squared (NIS) check threshold. Used to reject outlier measurements
		_target_est_pos[j]->setNISthreshold(_nis_threshold);

		if (!_target_est_pos[j]->update()) {
			all_axis_fused = false;
			PX4_DEBUG("Obs i = %d : not fused in direction: %d", target_obs.type, j);
		}

		target_innov.observation[j] = target_obs.meas_xyz(j);
		target_innov.observation_variance[j] = target_obs.meas_unc_xyz(j);

		target_innov.test_ratio[j] = _target_est_pos[j]->getTestRatio();
	}

	target_innov.fused = all_axis_fused;
	target_innov.innovation_rejected = !all_axis_fused;

	perf_end(_vte_update_perf);
	publishInnov(target_innov, target_obs.type);

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

	case ObsType::Uwb:
		_vte_aid_uwb_pub.publish(target_innov);
		break;

	case ObsType::Type_count:
		break;
	}
}

void VTEPosition::publishTarget()
{

	vision_target_est_position_s vte_state{};
	landing_target_pose_s target_pose{};

	target_pose.timestamp = _last_predict;
	vte_state.timestamp = _last_predict;

	target_pose.rel_pos_valid =	isMeasValid(_last_update);

#if defined(CONFIG_VTEST_MOVING)
	target_pose.is_static = false;
#else
	target_pose.is_static = true;
#endif // CONFIG_VTEST_MOVING

	// Get state
	matrix::Vector<float, vtest::State::size> state_x = _target_est_pos[Axis::X]->getState();
	matrix::Vector<float, vtest::State::size> state_y = _target_est_pos[Axis::Y]->getState();
	matrix::Vector<float, vtest::State::size> state_z = _target_est_pos[Axis::Z]->getState();

	matrix::Vector<float, vtest::State::size> state_var_x = _target_est_pos[Axis::X]->getStateVar();
	matrix::Vector<float, vtest::State::size> state_var_y = _target_est_pos[Axis::Y]->getStateVar();
	matrix::Vector<float, vtest::State::size> state_var_z = _target_est_pos[Axis::Z]->getStateVar();

	// Fill target relative pose
	target_pose.x_rel = state_x(vtest::State::pos_rel);
	target_pose.y_rel = state_y(vtest::State::pos_rel);
	target_pose.z_rel = state_z(vtest::State::pos_rel);

	target_pose.cov_x_rel = state_var_x(vtest::State::pos_rel);
	target_pose.cov_y_rel = state_var_y(vtest::State::pos_rel);
	target_pose.cov_z_rel = state_var_z(vtest::State::pos_rel);

	// Fill target relative velocity
	target_pose.vx_rel = -state_x(vtest::State::vel_uav);
	target_pose.vy_rel = -state_y(vtest::State::vel_uav);
	target_pose.vz_rel = -state_z(vtest::State::vel_uav);

	target_pose.cov_vx_rel = state_var_x(vtest::State::vel_uav);
	target_pose.cov_vy_rel = state_var_y(vtest::State::vel_uav);
	target_pose.cov_vz_rel = state_var_z(vtest::State::vel_uav);

#if defined(CONFIG_VTEST_MOVING)
	// If target is moving, relative velocity = vel_target - vel_uav
	target_pose.vx_rel += state_x(vtest::State::vel_target);
	target_pose.vy_rel += state_y(vtest::State::vel_target);
	target_pose.vz_rel += state_z(vtest::State::vel_target);

	// Var(aX + bY) = a^2 Var(x) + b^2 Var(y) + 2abCov(X,Y)
	target_pose.cov_vx_rel += state_var_x(vtest::State::vel_target);
	target_pose.cov_vy_rel += state_var_y(vtest::State::vel_target);
	target_pose.cov_vz_rel += state_var_z(vtest::State::vel_target);

#endif // CONFIG_VTEST_MOVING

	// Fill vision target estimator state
	vte_state.x_rel = target_pose.x_rel;
	vte_state.y_rel = target_pose.y_rel;
	vte_state.z_rel = target_pose.z_rel;

	vte_state.cov_x_rel = target_pose.cov_x_rel;
	vte_state.cov_y_rel = target_pose.cov_y_rel;
	vte_state.cov_z_rel = target_pose.cov_z_rel;

	// Fill uav velocity
	vte_state.vx_uav = state_x(vtest::State::vel_uav);
	vte_state.vy_uav = state_y(vtest::State::vel_uav);
	vte_state.vz_uav = state_z(vtest::State::vel_uav);

	vte_state.cov_vx_uav = state_var_x(vtest::State::vel_uav);
	vte_state.cov_vy_uav = state_var_y(vtest::State::vel_uav);
	vte_state.cov_vz_uav = state_var_z(vtest::State::vel_uav);

	vte_state.x_bias = state_x(vtest::State::bias);
	vte_state.y_bias = state_y(vtest::State::bias);
	vte_state.z_bias = state_z(vtest::State::bias);

	vte_state.cov_x_bias = state_var_x(vtest::State::bias);
	vte_state.cov_y_bias = state_var_y(vtest::State::bias);
	vte_state.cov_z_bias = state_var_z(vtest::State::bias);

#if defined(CONFIG_VTEST_MOVING)

	vte_state.vx_target = state_x(vtest::State::vel_target);
	vte_state.vy_target = state_y(vtest::State::vel_target);
	vte_state.vz_target = state_z(vtest::State::vel_target);

	vte_state.cov_vx_target = state_var_x(vtest::State::vel_target);
	vte_state.cov_vy_target = state_var_y(vtest::State::vel_target);
	vte_state.cov_vz_target = state_var_z(vtest::State::vel_target);

	vte_state.ax_target = state_x(vtest::State::acc_target);
	vte_state.ay_target = state_y(vtest::State::acc_target);
	vte_state.az_target = state_z(vtest::State::acc_target);

	vte_state.cov_ax_target = state_var_x(vtest::State::acc_target);
	vte_state.cov_ay_target = state_var_y(vtest::State::acc_target);
	vte_state.cov_az_target = state_var_z(vtest::State::acc_target);

#endif // CONFIG_VTEST_MOVING

	// If the target is static, valid and vision obs was fused recently, use the relative to aid the EKF2 state estimation.
	// Check performed in EKF2 to use target vel: if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid)
	target_pose.rel_vel_valid = target_pose.is_static && _param_vte_ekf_aid.get() && target_pose.rel_pos_valid &&
				    (hrt_absolute_time() - _last_vision_obs_fused_time) < meas_valid_TIMEOUT_US;

	// Prec land does not check target_pose.abs_pos_valid. Only send the target if abs pose valid.
	if (_local_position.valid && target_pose.rel_pos_valid) {
		target_pose.x_abs = target_pose.x_rel + _local_position.xyz(0);
		target_pose.y_abs = target_pose.y_rel + _local_position.xyz(1);
		target_pose.z_abs = target_pose.z_rel + _local_position.xyz(2);
		target_pose.abs_pos_valid = true;

#if defined(CONFIG_VTEST_MOVING)

		// If the target is moving, move towards its expected location
		float mpc_z_v_auto_dn = 0.f;
		param_get(param_find("MPC_Z_V_AUTO_DN"), &mpc_z_v_auto_dn);
		float intersection_time_s = fabsf(target_pose.z_rel / mpc_z_v_auto_dn);
		intersection_time_s = math::constrain(intersection_time_s, _param_vte_moving_t_min.get(),
						      _param_vte_moving_t_max.get());

		// Anticipate where the target will be
		target_pose.x_abs += vte_state.vx_target * intersection_time_s;
		target_pose.y_abs += vte_state.vy_target * intersection_time_s;
		target_pose.z_abs += vte_state.vz_target * intersection_time_s;

#endif // CONFIG_VTEST_MOVING

		_targetPosePub.publish(target_pose);

	}

	_targetEstimatorStatePub.publish(vte_state);

	// TODO: decide what to do with Bias lim
	float bias_lim = _param_vte_bias_lim.get();

	if (((float)fabs(vte_state.x_bias) > bias_lim
	     || (float)fabs(vte_state.y_bias) > bias_lim || (float)fabs(vte_state.z_bias) > bias_lim)) {

		PX4_DEBUG("Bias exceeds limit: %.2f bias x: %.2f bias y: %.2f bias z: %.2f", (double)bias_lim,
			  (double)vte_state.x_bias, (double)vte_state.y_bias, (double)vte_state.z_bias);

		// reset_filter();
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

	// TODO: check other measurements?
}

void VTEPosition::set_gps_pos_offset(const matrix::Vector3f &xyz, const bool gps_is_offset)
{
	_gps_pos_is_offset = gps_is_offset;
	_gps_pos_offset_ned.xyz = xyz;
	_gps_pos_offset_ned.valid = (PX4_ISFINITE(_gps_pos_offset_ned.xyz(0)) && PX4_ISFINITE(_gps_pos_offset_ned.xyz(0))
				     && PX4_ISFINITE(_gps_pos_offset_ned.xyz(2)));
	_gps_pos_offset_ned.timestamp = hrt_absolute_time();
}

void VTEPosition::set_velocity_offset(const matrix::Vector3f &xyz)
{
	_velocity_offset_ned.xyz = xyz;
	_velocity_offset_ned.valid = (PX4_ISFINITE(_velocity_offset_ned.xyz(0)) && PX4_ISFINITE(_velocity_offset_ned.xyz(0))
				      && PX4_ISFINITE(_velocity_offset_ned.xyz(2)));
	_velocity_offset_ned.timestamp = hrt_absolute_time();
}

void VTEPosition::set_range_sensor(const float dist, const bool valid, hrt_abstime timestamp)
{
	_range_sensor.valid = valid && isMeasUpdated(timestamp);
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
	if (_vte_aid_mask & SensorFusionMask::USE_MISSION_POS) {
		_mission_land_position.lat_deg = lat_deg;
		_mission_land_position.lon_deg = lon_deg;
		_mission_land_position.alt_m = alt_m;
		_mission_land_position.valid = isLatLonAltValid(_mission_land_position.lat_deg, _mission_land_position.lon_deg,
					       _mission_land_position.alt_m, "Mission position ");

		if (_mission_land_position.valid) {
			PX4_INFO("Mission position lat %.8f, lon %.8f [deg], alt %.1f [m]", lat_deg,
				 lon_deg, (double)(alt_m));

		} else {
			PX4_WARN("Mission position not used because not valid");
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
	_gps_vel_noise = _param_vte_gps_vel_noise.get();
	_gps_pos_noise = _param_vte_gps_pos_noise.get();
	_ev_noise_md = _param_vte_ev_noise_md.get();
	_ev_pos_noise = _param_vte_ev_pos_noise.get();
	_nis_threshold = _param_vte_pos_nis_thre.get();
}

bool VTEPosition::createEstimators()
{

	// Array to hold temporary pointers
	KF_position *tmp[Axis::Count] = {nullptr, nullptr, nullptr};
	bool init_failed = false;

	// Try to allocate new estimators
	for (int axis = 0; axis < Axis::Count; ++axis) {
		tmp[axis] = new KF_position;

		if (tmp[axis] == nullptr) {
			init_failed = true;
			break;
		}
	}

	if (init_failed) {
		// Clean up any allocated estimators
		for (int axis = 0; axis < Axis::Count; ++axis) {
			delete tmp[axis];
			tmp[axis] = nullptr;
		}

		return false;

	} else {
		// Replace old estimators with new ones
		for (int axis = 0; axis < Axis::Count; ++axis) {
			delete _target_est_pos[axis];
			_target_est_pos[axis] = tmp[axis];
			tmp[axis] = nullptr; // Prevent deletion in the cleanup loop
		}

		return true;
	}
}

bool VTEPosition::isLatLonAltValid(double lat_deg, double lon_deg, float alt_m, const char *who) const
{
	// all finite
	if (!PX4_ISFINITE(lat_deg) || !PX4_ISFINITE(lon_deg) || !PX4_ISFINITE(alt_m)) {
		if (who) {
			PX4_WARN("%s position not finite! lat: %.8f, lon: %.8f, alt: %.1f",
				 who, lat_deg, lon_deg, (double)alt_m);
		}

		return false;
	}

	// latitude/longitude within symmetric geographic ranges
	if ((fabs(lat_deg) > lat_abs_max_deg) || (fabs(lon_deg) > lon_abs_max_deg)) {
		if (who) {
			PX4_WARN("%s lat/lon out of range! lat: %.8f, lon: %.8f", who, lat_deg, lon_deg);
		}

		return false;
	}

	// (0,0) sentinel is considered invalid
	if ((fabs(lat_deg) < DBL_EPSILON) && (fabs(lon_deg) < DBL_EPSILON)) {
		if (who) {
			PX4_WARN("%s position near (0,0) sentinel!", who);
		}

		return false;
	}

	// altitude window
	if ((alt_m < alt_min_m) || (alt_m > alt_max_m)) {
		if (who) {
			PX4_WARN("%s altitude out of range! alt: %.1f [m] (limits: %.1f;%.1f)",
				 who, (double)alt_m, (double)alt_min_m, (double)alt_max_m);
		}

		return false;
	}

	return true;
}

} // namespace vision_target_estimator
