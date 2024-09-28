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

	_check_params(true);
}


VTEPosition::~VTEPosition()
{
	for (int i = 0; i < 3; i++) {
		delete _target_estimator[i];
	}

	perf_free(_vte_predict_perf);
	perf_free(_vte_update_perf);
}


bool VTEPosition::init()
{

// Check valid python_derivation/generated/state.h
#if defined(CONFIG_VTEST_MOVING)
	PX4_WARN("VTE for moving targets has not been thoroughly tested.");

	if (vtest::State::size != 5) {
		PX4_ERR("VTE: Invalid state. Missing auto-generated files.");
		return false;
	}

#else

	if (vtest::State::size != 3) {
		PX4_ERR("VTE: Invalid state. Missing auto-generated files.");
		return false;
	}

#endif // CONFIG_VTEST_MOVING

	// Get params
	_vte_TIMEOUT_US = (uint32_t)(_param_vte_btout.get() * 1_s);
	_vte_aid_mask = _param_vte_aid_mask.get();

	if (_vte_aid_mask == SensorFusionMask::NO_SENSOR_FUSION) {
		PX4_ERR("VTE: no data fusion enabled. Modify VTE_AID_MASK and reboot");
		return false;
	}

	if (!initTargetEstimator()) {
		PX4_ERR("VTE position KF init failed");
		return false;
	}

	// TODO: move prints into a function.
#if defined(CONFIG_VTEST_MOVING)

	if (_vte_aid_mask & SensorFusionMask::USE_MISSION_POS) {
		PX4_WARN("VTE mission land position data fusion cannot be enabled for moving targets.");
		PX4_WARN("Disabling mission land position fusion.");
		_vte_aid_mask -= SensorFusionMask::USE_MISSION_POS;
	}

#endif // CONFIG_VTEST_MOVING

	if ((_vte_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) && (_vte_aid_mask & SensorFusionMask::USE_MISSION_POS)) {
		PX4_WARN("VTE both target GPS position and mission land position data fusion enabled.");
		PX4_WARN("Disabling mission land position fusion.");
		_vte_aid_mask -= SensorFusionMask::USE_MISSION_POS;
	}

	if (_vte_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) { PX4_INFO("VTE target GPS position data fusion enabled");}

	if (_vte_aid_mask & SensorFusionMask::USE_TARGET_GPS_VEL) { PX4_INFO("VTE target GPS velocity data fusion enabled");}

	if (_vte_aid_mask & SensorFusionMask::USE_MISSION_POS) { PX4_INFO("VTE PX4 mission position fusion enabled");}

	if (_vte_aid_mask & SensorFusionMask::USE_UAV_GPS_VEL) { PX4_INFO("VTE UAV GPS velocity data fusion enabled");}

	if (_vte_aid_mask & SensorFusionMask::USE_EXT_VIS_POS) { PX4_INFO("VTE target external vision-based relative position data fusion enabled");}

	return true;
}


void VTEPosition::resetFilter()
{
	_estimator_initialized = false;
	_last_vision_obs_fused_time = 0;
	_bias_set = false;
	_mission_position.valid = false;
	_has_timed_out = false;
}


void VTEPosition::update(const Vector3f &acc_ned)
{
	_check_params(false);

	// predict the target state using a constant relative acceleration model
	if (_estimator_initialized) {

		if (hrt_absolute_time() - _last_update > _vte_TIMEOUT_US) {
			PX4_WARN("VTE Position estimator timeout");
			_has_timed_out = true;

		} else {
			perf_begin(_vte_predict_perf);
			predictionStep(acc_ned);
			perf_end(_vte_predict_perf);

			_last_predict = hrt_absolute_time();
		}
	}

	// Update and fuse the observations and publishes innovations
	if (update_step(acc_ned)) {
		_last_update = _last_predict;
	}

	if (_estimator_initialized) {publishTarget();}
}


bool VTEPosition::initEstimator(const Matrix<float, Direction::nb_directions, vtest::State::size>
				&state_init)
{
	// Get initial variance from params
	const float state_pos_var = _param_vte_pos_unc_in.get();
	const float state_vel_var = _param_vte_vel_unc_in.get();
	const float state_bias_var = _param_vte_bias_unc_in.get();

	const Vector3f state_pos_var_vect(state_pos_var, state_pos_var, state_pos_var);
	const Vector3f state_vel_var_vect(state_vel_var, state_vel_var, state_vel_var);
	const Vector3f state_bias_var_vect(state_bias_var, state_bias_var, state_bias_var);

	matrix::Matrix <float, Direction::nb_directions, vtest::State::size> state_var_init;
	state_var_init.col(vtest::State::pos_rel.idx) = state_pos_var_vect;
	state_var_init.col(vtest::State::vel_uav.idx) = state_vel_var_vect;
	state_var_init.col(vtest::State::bias.idx) = state_bias_var_vect;

#if defined(CONFIG_VTEST_MOVING)
	const float state_acc_var = _param_vte_acc_unc_in.get();
	const float state_target_vel_var = _param_vte_vel_unc_in.get();
	const Vector3f state_acc_var_vect(state_acc_var, state_acc_var, state_acc_var);
	const Vector3f state_target_vel_var_vect(state_target_vel_var, state_target_vel_var, state_target_vel_var);
	state_var_init.col(vtest::State::acc_target.idx) = state_acc_var_vect;
	state_var_init.col(vtest::State::vel_target.idx) = state_target_vel_var_vect;

#endif // CONFIG_VTEST_MOVING

	for (int i = 0; i < Direction::nb_directions; i++) {

		_target_estimator[i]->setState(state_init.row(i));
		_target_estimator[i]->setStateVar(state_var_init.row(i));
	}

	// Debug INFO
	PX4_INFO("Pos init %.2f %.2f %.2f", (double)state_init(Direction::x, vtest::State::pos_rel.idx),
		 (double)state_init(Direction::y, vtest::State::pos_rel.idx), (double)state_init(Direction::z,
				 vtest::State::pos_rel.idx));
	PX4_INFO("Vel uav init %.2f %.2f %.2f", (double)state_init(Direction::x, vtest::State::vel_uav.idx),
		 (double)state_init(Direction::y, vtest::State::vel_uav.idx), (double)state_init(Direction::z,
				 vtest::State::vel_uav.idx));
	PX4_INFO("Bias init %.2f %.2f %.2f", (double)state_init(Direction::x, vtest::State::bias.idx),
		 (double)state_init(Direction::y, vtest::State::bias.idx), (double)state_init(Direction::z,
				 vtest::State::bias.idx));

#if defined(CONFIG_VTEST_MOVING)
	PX4_INFO("Target acc init %.2f %.2f %.2f", (double)state_init(Direction::x,
			vtest::State::acc_target.idx),
		 (double)state_init(Direction::y, vtest::State::acc_target.idx), (double)state_init(Direction::z,
				 vtest::State::acc_target.idx));
	PX4_INFO("Target vel init %.2f %.2f %.2f", (double)state_init(Direction::x,
			vtest::State::vel_target.idx),
		 (double)state_init(Direction::y, vtest::State::vel_target.idx), (double)state_init(Direction::z,
				 vtest::State::vel_target.idx));
#endif // CONFIG_VTEST_MOVING

	return true;
}


void VTEPosition::predictionStep(const Vector3f &vehicle_acc_ned)
{
	// predict target position with the help of accel data

	// Time from last prediciton
	const float dt = (hrt_absolute_time() - _last_predict) / 1_s;

	// The rotated input cov (from body to NED R*cov*R^T) is the same as the original input cov since input_cov = acc_unc * Identiy and R*R^T = Identity
	const SquareMatrix<float, Direction::nb_directions> input_cov = diag(Vector3f(_drone_acc_unc, _drone_acc_unc,
			_drone_acc_unc));
	const SquareMatrix<float, Direction::nb_directions> bias_cov = diag(Vector3f(_bias_unc, _bias_unc, _bias_unc));

#if defined(CONFIG_VTEST_MOVING)
	const SquareMatrix<float, Direction::nb_directions> target_acc_cov = diag(Vector3f(_target_acc_unc, _target_acc_unc,
			_target_acc_unc));
#endif // CONFIG_VTEST_MOVING

	//Decoupled dynamics, we neglect the off diag elements.
	for (int i = 0; i < Direction::nb_directions; i++) {

#if defined(CONFIG_VTEST_MOVING)
		_target_estimator[i]->setTargetAccVar(target_acc_cov(i, i));
#endif // CONFIG_VTEST_MOVING

		_target_estimator[i]->setBiasVar(bias_cov(i, i));
		_target_estimator[i]->setInputAccVar(input_cov(i, i));

		_target_estimator[i]->predictState(dt, vehicle_acc_ned(i));
		_target_estimator[i]->predictCov(dt);
	}
}

bool VTEPosition::update_step(const Vector3f &vehicle_acc_ned)
{

	// Update the observations and vte_fusion_aid_mask if any valid observation.
	targetObsPos observations[ObservationType::nb_observation_types];
	ObservationValidMask vte_fusion_aid_mask = ObservationValidMask::NO_VALID_DATA;
	processObservations(vte_fusion_aid_mask, observations);

	// No new observations --> no fusion.
	if (vte_fusion_aid_mask == ObservationValidMask::NO_VALID_DATA) {
		return false;
	}

	// Check if we have new observations
	const bool new_pos_sensor = _hasNewPositionSensorData(vte_fusion_aid_mask);

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
	if (!_bias_set && _should_set_bias(vte_fusion_aid_mask)) {
		// For now the only non-gnss measurement is vision.
		updateBias(observations[ObservationType::fiducial_marker].meas_xyz);
	}

	// Fuse new sensor data
	if (new_pos_sensor || _hasNewVelocitySensorData(vte_fusion_aid_mask)) {
		return fuseNewSensorData(vehicle_acc_ned, vte_fusion_aid_mask, observations);
	}

	return false;
}


void VTEPosition::processObservations(ObservationValidMask &vte_fusion_aid_mask,
				      targetObsPos observations[ObservationType::nb_observation_types])
{

	/* Handle Vision Data */
	handleVisionData(vte_fusion_aid_mask, observations[ObservationType::fiducial_marker]);

	/* Handle UAV GPS position */
	sensor_gps_s vehicle_gps_position;
	const bool vehicle_gps_position_updated = _vehicle_gps_position_sub.update(&vehicle_gps_position);
	const bool vehicle_gps_valid = isVehicleGpsDataValid(vehicle_gps_position);

	if (vehicle_gps_position_updated && vehicle_gps_valid) {
		handleUavGpsData(vehicle_gps_position, vte_fusion_aid_mask, observations[ObservationType::mission_gps_pos],
				 observations[ObservationType::uav_gps_vel]);
	}

	/* Handle Target GPS position */
	target_gnss_s target_GNSS_report;
	const bool target_GPS_updated = _target_gnss_sub.update(&target_GNSS_report);
	const bool target_gps_valid = isTargetGpsDataValid(target_GNSS_report);

	if (target_GPS_updated && target_gps_valid) {
		handleTargetGpsData(vehicle_gps_valid, vehicle_gps_position, target_GNSS_report, vte_fusion_aid_mask,
				    observations[ObservationType::target_gps_pos], observations[ObservationType::target_gps_vel]);
	}
}


void VTEPosition::handleVisionData(ObservationValidMask &vte_fusion_aid_mask, targetObsPos &obs_fiducial_marker)
{

	fiducial_marker_pos_report_s fiducial_marker_pose;

	if (!((_vte_aid_mask & SensorFusionMask::USE_EXT_VIS_POS)
	      && _fiducial_marker_report_sub.update(&fiducial_marker_pose))) {
		return;
	}

	if (!isVisionDataValid(fiducial_marker_pose)) {
		return;
	}

	if (processObsVision(fiducial_marker_pose, obs_fiducial_marker)) {
		vte_fusion_aid_mask = static_cast<ObservationValidMask>(vte_fusion_aid_mask | ObservationValidMask::FUSE_EXT_VIS_POS);
	}
}


bool VTEPosition::isVisionDataValid(const fiducial_marker_pos_report_s &fiducial_marker_pose)
{
	// TODO: extend checks
	return _is_meas_valid(fiducial_marker_pose.timestamp);
}


void VTEPosition::handleUavGpsData(const sensor_gps_s &vehicle_gps_position,
				   ObservationValidMask &vte_fusion_aid_mask,
				   targetObsPos &obs_gps_pos_mission,
				   targetObsPos &obs_gps_vel_uav)
{

	/* Handle GPS mission pos*/
	if ((_vte_aid_mask & SensorFusionMask::USE_MISSION_POS) && _mission_position.valid) {

		if (processObsGNSSPosMission(vehicle_gps_position, obs_gps_pos_mission)) {
			vte_fusion_aid_mask = static_cast<ObservationValidMask>(vte_fusion_aid_mask | ObservationValidMask::FUSE_MISSION_POS);
		}
	}

	// Update and process UAV GPS velocity
	updateUavGpsVelocity(vehicle_gps_position);

	/* Handle GPS velocity */
	if ((_vte_aid_mask & SensorFusionMask::USE_UAV_GPS_VEL && _uav_gps_vel.valid)) {

		if (processObsGNSSVelUav(vehicle_gps_position, obs_gps_vel_uav)) {
			vte_fusion_aid_mask = static_cast<ObservationValidMask>(vte_fusion_aid_mask | ObservationValidMask::FUSE_UAV_GPS_VEL);
		}
	}
}


void VTEPosition::updateUavGpsVelocity(const sensor_gps_s &vehicle_gps_position)
{
	_uav_gps_vel.timestamp = vehicle_gps_position.timestamp;
	_uav_gps_vel.valid = (vehicle_gps_position.vel_ned_valid &&
			      PX4_ISFINITE(vehicle_gps_position.vel_n_m_s) &&
			      PX4_ISFINITE(vehicle_gps_position.vel_e_m_s) &&
			      PX4_ISFINITE(vehicle_gps_position.vel_d_m_s));

	_uav_gps_vel.xyz(Direction::x) = vehicle_gps_position.vel_n_m_s;
	_uav_gps_vel.xyz(Direction::y) = vehicle_gps_position.vel_e_m_s;
	_uav_gps_vel.xyz(Direction::z) = vehicle_gps_position.vel_d_m_s;
}

bool VTEPosition::isVehicleGpsDataValid(const sensor_gps_s &vehicle_gps_position)
{
	// TODO: extend checks

	if (!_is_meas_valid(vehicle_gps_position.timestamp)) {
		return false;
	}

	if (((fabs(vehicle_gps_position.latitude_deg) < DBL_EPSILON)
	     && (fabs(vehicle_gps_position.longitude_deg) < DBL_EPSILON))
	    || !PX4_ISFINITE(vehicle_gps_position.altitude_msl_m)) {
		PX4_WARN("vehicle GPS position is corrupt!");
		return false;
	}

	return true;
}


void VTEPosition::handleTargetGpsData(const bool vehicle_gps_valid,
				      const sensor_gps_s &vehicle_gps_position,
				      const target_gnss_s &target_GNSS_report,
				      ObservationValidMask &vte_fusion_aid_mask,
				      targetObsPos &obs_gps_pos_target,
				      targetObsPos &obs_gps_vel_target)
{
	if ((_vte_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) && vehicle_gps_valid && target_GNSS_report.abs_pos_updated) {

		if (processObsGNSSPosTarget(target_GNSS_report, vehicle_gps_position, obs_gps_pos_target)) {
			vte_fusion_aid_mask = static_cast<ObservationValidMask>(vte_fusion_aid_mask |
					      ObservationValidMask::FUSE_TARGET_GPS_POS);
		}
	}

	// Update and process target GPS velocity
	updateTargetGpsVelocity(target_GNSS_report);

#if defined(CONFIG_VTEST_MOVING)

	if (_vte_aid_mask & SensorFusionMask::USE_TARGET_GPS_VEL && _target_gps_vel.valid) {

		if (processObsGNSSVelTarget(target_GNSS_report, obs_gps_vel_target)) {
			vte_fusion_aid_mask = static_cast<ObservationValidMask>(vte_fusion_aid_mask |
					      ObservationValidMask::FUSE_TARGET_GPS_VEL);
		}
	}

#endif // CONFIG_VTEST_MOVING

}


void VTEPosition::updateTargetGpsVelocity(const target_gnss_s &target_GNSS_report)
{
	_target_gps_vel.timestamp = target_GNSS_report.timestamp;
	_target_gps_vel.valid = (target_GNSS_report.vel_ned_updated &&
				 PX4_ISFINITE(target_GNSS_report.vel_n_m_s) &&
				 PX4_ISFINITE(target_GNSS_report.vel_e_m_s) &&
				 PX4_ISFINITE(target_GNSS_report.vel_d_m_s));

	_target_gps_vel.xyz(Direction::x) = target_GNSS_report.vel_n_m_s;
	_target_gps_vel.xyz(Direction::y) = target_GNSS_report.vel_e_m_s;
	_target_gps_vel.xyz(Direction::z) = target_GNSS_report.vel_d_m_s;
}

bool VTEPosition::isTargetGpsDataValid(const target_gnss_s &target_GNSS_report)
{
	// TODO: extend checks

	if (!_is_meas_valid(target_GNSS_report.timestamp)) {
		return false;
	}

	if (((fabs(target_GNSS_report.latitude_deg) < DBL_EPSILON)
	     && (fabs(target_GNSS_report.longitude_deg) < DBL_EPSILON))
	    || !PX4_ISFINITE(target_GNSS_report.altitude_msl_m)) {
		PX4_WARN("target GPS position is corrupt!");
		return false;
	}

	return true;
}


bool VTEPosition::initializeEstimator(const ObservationValidMask &vte_fusion_aid_mask,
				      const targetObsPos observations[ObservationType::nb_observation_types])
{
	if (!_hasNewPositionSensorData(vte_fusion_aid_mask)) {
		return false;
	}

	// Check for initial velocity estimate
	const bool has_initial_velocity_estimate = (_local_velocity.valid && _is_meas_valid(_local_velocity.timestamp)) ||
			(_uav_gps_vel.valid && _is_meas_valid(_uav_gps_vel.timestamp));

	if (!has_initial_velocity_estimate) {
		PX4_WARN("No UAV velocity estimate. Estimator cannot be started.");
		return false;
	}

	// Initialize state variables
	Vector3f pos_init;
	Vector3f uav_vel_init;
	Vector3f bias_init;
#if defined(CONFIG_VTEST_MOVING)
	Vector3f target_acc_init;	// Assume null target absolute acceleration
	Vector3f target_vel_init;
#endif // CONFIG_VTEST_MOVING


	// Define the initial relative position
	if (vte_fusion_aid_mask & ObservationValidMask::FUSE_EXT_VIS_POS) {
		pos_init = observations[ObservationType::fiducial_marker].meas_xyz;

	} else if (vte_fusion_aid_mask & ObservationValidMask::FUSE_TARGET_GPS_POS) {
		pos_init = observations[ObservationType::target_gps_pos].meas_xyz;

	} else if (vte_fusion_aid_mask & ObservationValidMask::FUSE_MISSION_POS) {
		pos_init = observations[ObservationType::mission_gps_pos].meas_xyz;
	}

	// Compute initial bias if needed
	if (_should_set_bias(vte_fusion_aid_mask)) {
		PX4_INFO("VTE Position setting GNSS bias.");
		bias_init = _pos_rel_gnss.xyz - pos_init;
		_bias_set = true;
	}

#if defined(CONFIG_VTEST_MOVING)

	if (_target_gps_vel.valid && (_is_meas_valid(_target_gps_vel.timestamp))) {
		target_vel_init = _target_gps_vel.xyz;
	}

#endif // CONFIG_VTEST_MOVING

	// Define initial UAV velocity
	if (_uav_gps_vel.valid && _is_meas_valid(_uav_gps_vel.timestamp)) {
		uav_vel_init = _uav_gps_vel.xyz;

	} else if (_local_velocity.valid && _is_meas_valid(_local_velocity.timestamp)) {
		uav_vel_init = _local_velocity.xyz;
	}

	// Initialize estimator state
	matrix::Matrix <float, Direction::nb_directions, vtest::State::size> state_init;
	state_init.col(vtest::State::pos_rel.idx) = pos_init;
	state_init.col(vtest::State::vel_uav.idx) = uav_vel_init;
	state_init.col(vtest::State::bias.idx) = bias_init;

#if defined(CONFIG_VTEST_MOVING)
	state_init.col(vtest::State::acc_target.idx) = target_acc_init;
	state_init.col(vtest::State::vel_target.idx) = target_vel_init;
#endif // CONFIG_VTEST_MOVING

	if (!initEstimator(state_init)) {
		resetFilter();
		return false;
	}

	PX4_INFO("VTE Position Estimator properly initialized.");
	_estimator_initialized = true;
	_uav_gps_vel.valid = false;
	_target_gps_vel.valid = false;
	_last_update = hrt_absolute_time();
	_last_predict = _last_update;
	return true;
}

void VTEPosition::updateBias(const Vector3f &pos_init)
{

	// TODO: decide if we print the bias and the pos_init
	PX4_INFO("Second relative position measurement available, re-setting position and bias.");

	const float state_pos_var = _param_vte_pos_unc_in.get();
	const float state_bias_var = _param_vte_bias_unc_in.get();

	const Vector3f state_pos_var_vect(state_pos_var, state_pos_var, state_pos_var);
	const Vector3f state_bias_var_vect(state_bias_var, state_bias_var, state_bias_var);

	// Compute the initial bias
	const Vector3f bias_init = _pos_rel_gnss.xyz - pos_init;

	// Reset filter's state and variance
	for (int i = 0; i < Direction::nb_directions; i++) {
		matrix::Vector<float, vtest::State::size> temp_state = _target_estimator[i]->getState();
		temp_state(vtest::State::bias.idx) = bias_init(i);
		temp_state(vtest::State::pos_rel.idx) = pos_init(i);
		_target_estimator[i]->setState(temp_state);

		matrix::Vector<float, vtest::State::size> temp_state_var = _target_estimator[i]->getStateVar();
		temp_state_var(vtest::State::bias.idx) = state_bias_var_vect(i);
		temp_state_var(vtest::State::pos_rel.idx) = state_pos_var_vect(i);
		_target_estimator[i]->setStateVar(temp_state_var);
	}

	_bias_set = true;
}


bool VTEPosition::fuseNewSensorData(const matrix::Vector3f &vehicle_acc_ned, ObservationValidMask &vte_fusion_aid_mask,
				    const targetObsPos observations[ObservationType::nb_observation_types])
{
	bool pos_fused = false;

	// Fuse target GPS position
	if (vte_fusion_aid_mask & ObservationValidMask::FUSE_TARGET_GPS_POS) {
		if (fuse_meas(vehicle_acc_ned, observations[ObservationType::target_gps_pos])) {
			pos_fused = true;
		}
	}

	// Fuse mission GPS position
	if (vte_fusion_aid_mask & ObservationValidMask::FUSE_MISSION_POS) {
		if (fuse_meas(vehicle_acc_ned, observations[ObservationType::mission_gps_pos])) {
			pos_fused = true;
		}
	}

	// Fuse vision position
	if (vte_fusion_aid_mask & ObservationValidMask::FUSE_EXT_VIS_POS) {
		if (fuse_meas(vehicle_acc_ned, observations[ObservationType::fiducial_marker])) {
			_last_vision_obs_fused_time = hrt_absolute_time();
			pos_fused = true;
		}
	}

	// Fuse GPS relative velocity
	if (vte_fusion_aid_mask & ObservationValidMask::FUSE_UAV_GPS_VEL) {
		fuse_meas(vehicle_acc_ned, observations[ObservationType::uav_gps_vel]);
	}

#if defined(CONFIG_VTEST_MOVING)

	// Fuse target GPS velocity
	if (vte_fusion_aid_mask & ObservationValidMask::FUSE_TARGET_GPS_VEL) {
		fuse_meas(vehicle_acc_ned, observations[ObservationType::target_gps_vel]);
	}

#endif // CONFIG_VTEST_MOVING

	return pos_fused;
}

/*Vision observation: [rx, ry, rz]*/
bool VTEPosition::processObsVision(const fiducial_marker_pos_report_s &fiducial_marker_pose, targetObsPos &obs)
{
	/* Rotate vision observation from body FRD - to vc-NED */
	const matrix::Quaternionf quat_att(fiducial_marker_pose.q);
	const Vector3f vision_body(fiducial_marker_pose.x_rel_body, fiducial_marker_pose.y_rel_body,
				   fiducial_marker_pose.z_rel_body);
	const Vector3f vision_ned = quat_att.rotateVector(vision_body);

	// Rotate covariance matrix to vc-NED
	SquareMatrix<float, Direction::nb_directions> Cov_rotated;

	// Use uncertainty from parameters or from vision messages
	if (_ev_noise_md) {
		// Uncertainty proportional to the vertical distance
		const float meas_uncertainty = _range_sensor.valid ? ((_ev_pos_noise * _ev_pos_noise) * fmaxf(_range_sensor.dist_bottom,
					       1.f)) : ((_ev_pos_noise * _ev_pos_noise) * 10);
		Cov_rotated = diag(Vector3f(meas_uncertainty, meas_uncertainty, meas_uncertainty));

	} else {
		const SquareMatrix<float, Direction::nb_directions> covMat = diag(Vector3f(fmaxf(fiducial_marker_pose.var_x_rel_body,
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

	obs.type = ObservationType::fiducial_marker;
	obs.updated = true;

	// Assume noise correlation negligible:
	obs.meas_h_xyz(Direction::x, vtest::State::pos_rel.idx) = 1;
	obs.meas_h_xyz(Direction::y, vtest::State::pos_rel.idx) = 1;
	obs.meas_h_xyz(Direction::z, vtest::State::pos_rel.idx) = 1;

	obs.meas_xyz = vision_ned;

	// Assume off diag elements ~ 0
	obs.meas_unc_xyz(Direction::x) = Cov_rotated(Direction::x, Direction::x);
	obs.meas_unc_xyz(Direction::y) = Cov_rotated(Direction::y, Direction::y);
	obs.meas_unc_xyz(Direction::z) = Cov_rotated(Direction::z, Direction::z);

	return true;
}

/*Drone GNSS velocity observation: [r_dotx, r_doty, r_dotz]*/
bool VTEPosition::processObsGNSSVelUav(const sensor_gps_s &vehicle_gps_position, targetObsPos &obs)
{

	// TODO: improve checks
	// Make sure measurement are valid
	if (!PX4_ISFINITE(vehicle_gps_position.vel_n_m_s) || !PX4_ISFINITE(vehicle_gps_position.vel_e_m_s)
	    || !PX4_ISFINITE(vehicle_gps_position.vel_d_m_s)) {
		PX4_WARN("UAV GPS velocity is corrupt!");
		return false;
	}

	Vector3f vel_uav_ned(vehicle_gps_position.vel_n_m_s, vehicle_gps_position.vel_e_m_s, vehicle_gps_position.vel_d_m_s);

	if (_gps_pos_is_offset) {
		if (!(_velocity_offset_ned.valid)
		    || ((_velocity_offset_ned.timestamp - vehicle_gps_position.timestamp) > measurement_updated_TIMEOUT_US)) {

			return false;
		}

		vel_uav_ned -= _velocity_offset_ned.xyz;
	}

	obs.meas_xyz = vel_uav_ned;

	obs.meas_h_xyz(Direction::x, vtest::State::vel_uav.idx) = 1;
	obs.meas_h_xyz(Direction::y, vtest::State::vel_uav.idx) = 1;
	obs.meas_h_xyz(Direction::z, vtest::State::vel_uav.idx) = 1;

	const float unc = fmaxf(vehicle_gps_position.s_variance_m_s * vehicle_gps_position.s_variance_m_s,
				_gps_vel_noise * _gps_vel_noise);
	obs.meas_unc_xyz(Direction::x) = unc;
	obs.meas_unc_xyz(Direction::y) = unc;
	obs.meas_unc_xyz(Direction::z) = unc;

	obs.timestamp = vehicle_gps_position.timestamp;

	obs.type = ObservationType::uav_gps_vel;
	obs.updated = true;

	return true;
}

#if defined(CONFIG_VTEST_MOVING)

/*Target GNSS velocity observation: [r_dotx, r_doty, r_dotz]*/
bool VTEPosition::processObsGNSSVelTarget(const target_gnss_s &target_GNSS_report, targetObsPos &obs)
{

	// Make sure measurement are valid
	if (!PX4_ISFINITE(target_GNSS_report.vel_n_m_s) || !PX4_ISFINITE(target_GNSS_report.vel_e_m_s)
	    || !PX4_ISFINITE(target_GNSS_report.vel_d_m_s)) {
		PX4_WARN("Target GPS velocity is corrupt!");
		return false;
	}

	// If the target is moving, the relative velocity is expressed as the drone verlocity - the target velocity
	obs.meas_xyz(Direction::x) = target_GNSS_report.vel_n_m_s;
	obs.meas_xyz(Direction::y) = target_GNSS_report.vel_e_m_s;
	obs.meas_xyz(Direction::z) = target_GNSS_report.vel_d_m_s;

	const float unc = fmaxf(target_GNSS_report.s_variance_m_s * target_GNSS_report.s_variance_m_s,
				_gps_vel_noise * _gps_vel_noise);

	obs.meas_unc_xyz(Direction::x) = unc;
	obs.meas_unc_xyz(Direction::y) = unc;
	obs.meas_unc_xyz(Direction::z) = unc;

	obs.meas_h_xyz(Direction::x, vtest::State::vel_target.idx) = 1;
	obs.meas_h_xyz(Direction::y, vtest::State::vel_target.idx) = 1;
	obs.meas_h_xyz(Direction::z, vtest::State::vel_target.idx) = 1;

	obs.timestamp = target_GNSS_report.timestamp;

	obs.type = ObservationType::target_gps_vel;
	obs.updated = true;

	return true;
}

#endif // CONFIG_VTEST_MOVING

/*Target GNSS mission observation: [rx + bx, ry + by, rz + bz]*/
bool VTEPosition::processObsGNSSPosMission(const sensor_gps_s &vehicle_gps_position, targetObsPos &obs)
{

	// TODO: improve checks
	if (((fabs(vehicle_gps_position.latitude_deg) < DBL_EPSILON)
	     && (fabs(vehicle_gps_position.longitude_deg) < DBL_EPSILON))
	    || !PX4_ISFINITE(vehicle_gps_position.altitude_msl_m)) {
		PX4_WARN("vehicle GPS position is corrupt!");
		return false;
	}

	if (_gps_pos_is_offset && !_gps_pos_offset_ned.valid) {
		return false;
	}


	// Obtain GPS relative measurements in NED as target_global - uav_gps_global followed by global2local transformation
	Vector3f gps_relative_pos;
	get_vector_to_next_waypoint(vehicle_gps_position.latitude_deg, vehicle_gps_position.longitude_deg,
				    _mission_position.lat_deg, _mission_position.lon_deg,
				    &gps_relative_pos(0), &gps_relative_pos(1));

	// Down direction (if the drone is above the target, the relative position is positive)
	gps_relative_pos(2) = (float)vehicle_gps_position.altitude_msl_m - _mission_position.alt_m;

	// Offset gps relative position to the center of mass:
	if (_gps_pos_is_offset && _gps_pos_offset_ned.valid
	    && ((_gps_pos_offset_ned.timestamp - vehicle_gps_position.timestamp) < measurement_updated_TIMEOUT_US)) {
		gps_relative_pos += _gps_pos_offset_ned.xyz;
	}

	const float gps_unc_horizontal = fmaxf(vehicle_gps_position.eph * vehicle_gps_position.eph,
					       _gps_pos_noise * _gps_pos_noise);
	const float gps_unc_vertical = fmaxf(vehicle_gps_position.epv * vehicle_gps_position.epv,
					     _gps_pos_noise * _gps_pos_noise);

	// GPS already in NED, no rotation required.
	// Obs: [pos_rel + bias]
	obs.meas_h_xyz(Direction::x, vtest::State::pos_rel.idx) = 1;
	obs.meas_h_xyz(Direction::y, vtest::State::pos_rel.idx) = 1;
	obs.meas_h_xyz(Direction::z, vtest::State::pos_rel.idx) = 1;

	if (_bias_set) {
		obs.meas_h_xyz(Direction::x, vtest::State::bias.idx) = 1;
		obs.meas_h_xyz(Direction::y, vtest::State::bias.idx) = 1;
		obs.meas_h_xyz(Direction::z, vtest::State::bias.idx) = 1;
	}

	obs.timestamp = vehicle_gps_position.timestamp;

	obs.meas_xyz = gps_relative_pos;

	obs.meas_unc_xyz(Direction::x) = gps_unc_horizontal;
	obs.meas_unc_xyz(Direction::y) = gps_unc_horizontal;
	obs.meas_unc_xyz(Direction::z) = gps_unc_vertical;

	obs.type = ObservationType::mission_gps_pos;
	obs.updated = true;

	// If the target GPS fusion is enabled give it priority over the GPS relative position measurement.
	if (!(_vte_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) || !(_is_meas_valid(_pos_rel_gnss.timestamp))) {
		_pos_rel_gnss.timestamp = obs.timestamp;
		_pos_rel_gnss.valid = (PX4_ISFINITE(gps_relative_pos(0)) && PX4_ISFINITE(gps_relative_pos(1))
				       && PX4_ISFINITE(gps_relative_pos(2)));
		_pos_rel_gnss.xyz = gps_relative_pos;
	}

	return true;
}

/*Target GNSS observation: [rx + bx, ry + by, rz + bz]*/
bool VTEPosition::processObsGNSSPosTarget(const target_gnss_s &target_GNSS_report,
		const sensor_gps_s &vehicle_gps_position, targetObsPos &obs)
{

	const float dt_sync_us = fabsf((float)(vehicle_gps_position.timestamp - target_GNSS_report.timestamp));

	if ((fabs(vehicle_gps_position.latitude_deg) < DBL_EPSILON
	     && fabs(vehicle_gps_position.longitude_deg) < DBL_EPSILON)
	    || !PX4_ISFINITE(vehicle_gps_position.altitude_msl_m)) {
		PX4_WARN("vehicle GPS position is corrupt!");
		return false;

	} else if ((fabs(target_GNSS_report.latitude_deg) < DBL_EPSILON
		    && fabs(target_GNSS_report.longitude_deg) < DBL_EPSILON) || !PX4_ISFINITE(target_GNSS_report.altitude_msl_m)) {
		PX4_WARN("Target GPS position is corrupt!");
		return false;

	} else if (dt_sync_us > measurement_valid_TIMEOUT_US) {
		PX4_INFO("Time diff. between UAV GNSS and target GNNS too high: %.2f [ms] > timeout: %.2f [ms]",
			 (double)(dt_sync_us / 1000), (double)(measurement_valid_TIMEOUT_US / 1000));
		return false;
	}

	// Obtain GPS relative measurements in NED as target_global - uav_gps_global followed by global2local transformation
	Vector3f gps_relative_pos;
	get_vector_to_next_waypoint(vehicle_gps_position.latitude_deg, vehicle_gps_position.longitude_deg,
				    target_GNSS_report.latitude_deg, target_GNSS_report.longitude_deg,
				    &gps_relative_pos(0), &gps_relative_pos(1));

	// Down direction (if the drone is above the target, the relative position is positive)
	gps_relative_pos(2) = (float)(vehicle_gps_position.altitude_msl_m) - target_GNSS_report.altitude_msl_m;

	// Offset gps relative position to the center of mass:
	if (_gps_pos_is_offset) {
		if ((_gps_pos_offset_ned.valid)
		    && ((_gps_pos_offset_ned.timestamp - vehicle_gps_position.timestamp) < measurement_updated_TIMEOUT_US)) {
			gps_relative_pos += _gps_pos_offset_ned.xyz;

		} else {
			return false;
		}
	}

	// Var(aX - bY) = a^2 Var(X) + b^2Var(Y) - 2ab Cov(X,Y)
	const float gps_unc_horizontal = fmaxf(vehicle_gps_position.eph * vehicle_gps_position.eph,
					       _gps_pos_noise * _gps_pos_noise) + fmaxf(target_GNSS_report.eph * target_GNSS_report.eph,
							       _gps_pos_noise * _gps_pos_noise);
	const float gps_unc_vertical = fmaxf(vehicle_gps_position.epv * vehicle_gps_position.epv,
					     _gps_pos_noise * _gps_pos_noise) + fmaxf(target_GNSS_report.epv * target_GNSS_report.epv,
							     _gps_pos_noise * _gps_pos_noise);

	// GPS already in NED, no rotation required.
	// Obs: [pos_rel + bias]
	obs.meas_h_xyz(Direction::x, vtest::State::pos_rel.idx) = 1;
	obs.meas_h_xyz(Direction::y, vtest::State::pos_rel.idx) = 1;
	obs.meas_h_xyz(Direction::z, vtest::State::pos_rel.idx) = 1;

	if (_bias_set) {
		obs.meas_h_xyz(Direction::x, vtest::State::bias.idx) = 1;
		obs.meas_h_xyz(Direction::y, vtest::State::bias.idx) = 1;
		obs.meas_h_xyz(Direction::z, vtest::State::bias.idx) = 1;
	}

	obs.timestamp = target_GNSS_report.timestamp;

	obs.meas_xyz = gps_relative_pos;

	obs.meas_unc_xyz(Direction::x) = gps_unc_horizontal;
	obs.meas_unc_xyz(Direction::y) = gps_unc_horizontal;
	obs.meas_unc_xyz(Direction::z) = gps_unc_vertical;

	obs.type = ObservationType::target_gps_pos;
	obs.updated = true;

	// Keep track of the gps relative position
	_pos_rel_gnss.timestamp = obs.timestamp;
	_pos_rel_gnss.valid = (PX4_ISFINITE(gps_relative_pos(0)) && PX4_ISFINITE(gps_relative_pos(1))
			       && PX4_ISFINITE(gps_relative_pos(2)));
	_pos_rel_gnss.xyz = gps_relative_pos;

	return true;
}

bool VTEPosition::fuse_meas(const Vector3f &vehicle_acc_ned, const targetObsPos &target_pos_obs)
{
	perf_begin(_vte_update_perf);

	estimator_aid_source3d_s target_innov;
	bool all_directions_fused = true;
	Vector<float, vtest::State::size> meas_h_row;

	// Compute the measurement's time delay (difference between state and measurement time on validity)
	const float dt_sync_us = (_last_predict - target_pos_obs.timestamp);
	target_innov.time_last_fuse = (int)(dt_sync_us / 1000); // For debug: log the time sync. TODO: remove
	target_innov.timestamp_sample = target_pos_obs.timestamp;
	target_innov.timestamp = hrt_absolute_time();

	if (dt_sync_us > measurement_valid_TIMEOUT_US) {

		PX4_DEBUG("Obs i = %d too old. Time sync: %.2f [ms] > timeout: %.2f [ms]",
			  target_pos_obs.type,
			  (double)(dt_sync_us / 1000), (double)(measurement_valid_TIMEOUT_US / 1000));

		target_innov.fused = false;
		perf_end(_vte_update_perf);
		publishInnov(target_innov, target_pos_obs.type);
		return false;
	}

	if (!target_pos_obs.updated) {
		all_directions_fused = false;
		PX4_DEBUG("Obs i = %d : non-valid", target_pos_obs.type);
		target_innov.fused = false;
		perf_end(_vte_update_perf);
		publishInnov(target_innov, target_pos_obs.type);
		return false;
	}

	const float dt_sync_s = dt_sync_us / 1_s;

	for (int j = 0; j < Direction::nb_directions; j++) {

		//Get the corresponding row of the H matrix.
		meas_h_row = target_pos_obs.meas_h_xyz.row(j);

		// Move state back to the measurement time of validity. The state synchronized with the measurement will be used to compute the innovation.
		_target_estimator[j]->syncState(dt_sync_s, vehicle_acc_ned(j));

		_target_estimator[j]->setH(meas_h_row);
		target_innov.innovation_variance[j] = _target_estimator[j]->computeInnovCov(
				target_pos_obs.meas_unc_xyz(j));
		target_innov.innovation[j] = _target_estimator[j]->computeInnov(target_pos_obs.meas_xyz(j));

		// Set the Normalized Innovation Squared (NIS) check threshold. Used to reject outlier measurements
		_target_estimator[j]->setNISthreshold(_nis_threshold);

		if (!_target_estimator[j]->update()) {
			all_directions_fused = false;
			PX4_DEBUG("Obs i = %d : not fused in direction: %d", target_pos_obs.type, j);
		}

		target_innov.observation[j] = target_pos_obs.meas_xyz(j);
		target_innov.observation_variance[j] = target_pos_obs.meas_unc_xyz(j);

		// log test ratio defined as _innov / _innov_cov * _innov. If test_ratio > 3.84, no fusion
		target_innov.test_ratio[j] = _target_estimator[j]->getTestRatio();
	}

	target_innov.fused = all_directions_fused;
	target_innov.innovation_rejected = !all_directions_fused;

	perf_end(_vte_update_perf);
	publishInnov(target_innov, target_pos_obs.type);

	return all_directions_fused;
}

void VTEPosition::publishInnov(const estimator_aid_source3d_s &target_innov, const ObservationType type)
{

	// Publish innovations
	switch (type) {
	case ObservationType::target_gps_pos:
		_vte_aid_gps_pos_target_pub.publish(target_innov);
		break;

	case ObservationType::mission_gps_pos:
		_vte_aid_gps_pos_mission_pub.publish(target_innov);
		break;

	case ObservationType::uav_gps_vel:
		_vte_aid_gps_vel_uav_pub.publish(target_innov);
		break;

	case ObservationType::target_gps_vel:
		_vte_aid_gps_vel_target_pub.publish(target_innov);
		break;

	case ObservationType::fiducial_marker:
		_vte_aid_fiducial_marker_pub.publish(target_innov);
		break;

	case ObservationType::nb_observation_types:
		break;
	}
}

void VTEPosition::publishTarget()
{

	vision_target_est_position_s vte_state{};
	landing_target_pose_s target_pose{};

	target_pose.timestamp = _last_predict;
	vte_state.timestamp = _last_predict;

	target_pose.rel_pos_valid =	_is_meas_valid(_last_update);

#if defined(CONFIG_VTEST_MOVING)
	target_pose.is_static = false;
#else
	target_pose.is_static = true;
#endif // CONFIG_VTEST_MOVING

	// Get state
	matrix::Vector<float, vtest::State::size> state_x = _target_estimator[Direction::x]->getState();
	matrix::Vector<float, vtest::State::size> state_y = _target_estimator[Direction::y]->getState();
	matrix::Vector<float, vtest::State::size> state_z = _target_estimator[Direction::z]->getState();

	matrix::Vector<float, vtest::State::size> state_var_x = _target_estimator[Direction::x]->getStateVar();
	matrix::Vector<float, vtest::State::size> state_var_y = _target_estimator[Direction::y]->getStateVar();
	matrix::Vector<float, vtest::State::size> state_var_z = _target_estimator[Direction::z]->getStateVar();

	// Fill target relative pose
	target_pose.x_rel = state_x(vtest::State::pos_rel.idx);
	target_pose.y_rel = state_y(vtest::State::pos_rel.idx);
	target_pose.z_rel = state_z(vtest::State::pos_rel.idx);

	target_pose.cov_x_rel = state_var_x(vtest::State::pos_rel.idx);
	target_pose.cov_y_rel = state_var_y(vtest::State::pos_rel.idx);
	target_pose.cov_z_rel = state_var_z(vtest::State::pos_rel.idx);

	// Fill target relative velocity
	target_pose.vx_rel = -state_x(vtest::State::vel_uav.idx);
	target_pose.vy_rel = -state_y(vtest::State::vel_uav.idx);
	target_pose.vz_rel = -state_z(vtest::State::vel_uav.idx);

	target_pose.cov_vx_rel = state_var_x(vtest::State::vel_uav.idx);
	target_pose.cov_vy_rel = state_var_y(vtest::State::vel_uav.idx);
	target_pose.cov_vz_rel = state_var_z(vtest::State::vel_uav.idx);

#if defined(CONFIG_VTEST_MOVING)
	// If target is moving, relative velocity = vel_target - vel_uav
	target_pose.vx_rel += state_x(vtest::State::vel_target.idx);
	target_pose.vy_rel += state_y(vtest::State::vel_target.idx);
	target_pose.vz_rel += state_z(vtest::State::vel_target.idx);

	// Var(aX + bY) = a^2 Var(x) + b^2 Var(y) + 2abCov(X,Y)
	target_pose.cov_vx_rel += state_var_x(vtest::State::vel_target.idx);
	target_pose.cov_vy_rel += state_var_y(vtest::State::vel_target.idx);
	target_pose.cov_vz_rel += state_var_z(vtest::State::vel_target.idx);

#endif // CONFIG_VTEST_MOVING

	// Fill vision target estimator state
	vte_state.x_rel = target_pose.x_rel;
	vte_state.y_rel = target_pose.y_rel;
	vte_state.z_rel = target_pose.z_rel;

	vte_state.cov_x_rel = target_pose.cov_x_rel;
	vte_state.cov_y_rel = target_pose.cov_y_rel;
	vte_state.cov_z_rel = target_pose.cov_z_rel;

	// Fill uav velocity
	vte_state.vx_uav = state_x(vtest::State::vel_uav.idx);
	vte_state.vy_uav = state_y(vtest::State::vel_uav.idx);
	vte_state.vz_uav = state_z(vtest::State::vel_uav.idx);

	vte_state.cov_vx_uav = state_var_x(vtest::State::vel_uav.idx);
	vte_state.cov_vy_uav = state_var_y(vtest::State::vel_uav.idx);
	vte_state.cov_vz_uav = state_var_z(vtest::State::vel_uav.idx);

	vte_state.x_bias = state_x(vtest::State::bias.idx);
	vte_state.y_bias = state_y(vtest::State::bias.idx);
	vte_state.z_bias = state_z(vtest::State::bias.idx);

	vte_state.cov_x_bias = state_var_x(vtest::State::bias.idx);
	vte_state.cov_y_bias = state_var_y(vtest::State::bias.idx);
	vte_state.cov_z_bias = state_var_z(vtest::State::bias.idx);

#if defined(CONFIG_VTEST_MOVING)

	vte_state.vx_target = state_x(vtest::State::vel_target.idx);
	vte_state.vy_target = state_y(vtest::State::vel_target.idx);
	vte_state.vz_target = state_z(vtest::State::vel_target.idx);

	vte_state.cov_vx_target = state_var_x(vtest::State::vel_target.idx);
	vte_state.cov_vy_target = state_var_y(vtest::State::vel_target.idx);
	vte_state.cov_vz_target = state_var_z(vtest::State::vel_target.idx);

	vte_state.ax_target = state_x(vtest::State::acc_target.idx);
	vte_state.ay_target = state_y(vtest::State::acc_target.idx);
	vte_state.az_target = state_z(vtest::State::acc_target.idx);

	vte_state.cov_ax_target = state_var_x(vtest::State::acc_target.idx);
	vte_state.cov_ay_target = state_var_y(vtest::State::acc_target.idx);
	vte_state.cov_az_target = state_var_z(vtest::State::acc_target.idx);

#endif // CONFIG_VTEST_MOVING



	// Prec land does not check target_pose.abs_pos_valid. Only send the target if abs pose valid.
	if (_local_position.valid && target_pose.rel_pos_valid) {
		target_pose.x_abs = target_pose.x_rel + _local_position.xyz(0);
		target_pose.y_abs = target_pose.y_rel + _local_position.xyz(1);
		target_pose.z_abs = target_pose.z_rel + _local_position.xyz(2);
		target_pose.abs_pos_valid = true;

#if defined(CONFIG_VTEST_MOVING)

		// If the target is moving, move towards its expected location
		float mpc_z_v_auto_dn = param_find("MPC_Z_V_AUTO_DN");
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

	// If the target is static, valid and vision obs was fused recently, use the relative to aid the EKF2 state estimation.
	// Check performed in EKF2 to use target vel: if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid)
	target_pose.rel_vel_valid = target_pose.is_static && _param_vte_ekf_aid.get() && target_pose.rel_pos_valid &&
				    (hrt_absolute_time() - _last_vision_obs_fused_time) < measurement_valid_TIMEOUT_US;

	// TODO: decide what to do with Bias lim
	float bias_lim = _param_vte_bias_lim.get();

	if (((float)fabs(vte_state.x_bias) > bias_lim
	     || (float)fabs(vte_state.y_bias) > bias_lim || (float)fabs(vte_state.z_bias) > bias_lim)) {

		PX4_DEBUG("Bias exceeds limit: %.2f bias x: %.2f bias y: %.2f bias z: %.2f", (double)bias_lim,
			  (double)vte_state.x_bias, (double)vte_state.y_bias, (double)vte_state.z_bias);

		// resetFilter();
	}

}

void VTEPosition::_check_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();
	}

	// Make sure range sensor, local position and local velocity are up to date.
	if (_range_sensor.valid) {
		_range_sensor.valid = _is_meas_updated(_range_sensor.timestamp);
	}

	if (_local_position.valid) {
		_local_position.valid = _is_meas_updated(_local_position.timestamp);
	}

	if (_local_velocity.valid) {
		_local_velocity.valid = _is_meas_updated(_local_velocity.timestamp);
	}
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
	_range_sensor.valid = valid && _is_meas_updated(timestamp);
	_range_sensor.dist_bottom = dist;
	_range_sensor.timestamp = timestamp;

}

void VTEPosition::set_local_velocity(const matrix::Vector3f &vel_xyz, const bool vel_valid, hrt_abstime timestamp)
{
	_local_velocity.xyz = vel_xyz;
	_local_velocity.valid = vel_valid && _is_meas_updated(timestamp);
	_local_velocity.timestamp = timestamp;
}

void VTEPosition::set_local_position(const matrix::Vector3f &xyz, const bool pos_valid, hrt_abstime timestamp)
{
	_local_position.xyz = xyz;
	_local_position.valid = pos_valid && _is_meas_updated(timestamp);
	_local_position.timestamp = timestamp;
}

void VTEPosition::set_mission_position(const double lat_deg, const double lon_deg, const float alt_m)
{
	if (_vte_aid_mask & SensorFusionMask::USE_MISSION_POS) {
		_mission_position.lat_deg = lat_deg;
		_mission_position.lon_deg = lon_deg;
		_mission_position.alt_m = alt_m;
		_mission_position.valid = (((fabs(_mission_position.lat_deg) > DBL_EPSILON)
					    || (fabs(_mission_position.lon_deg) > DBL_EPSILON))
					   && PX4_ISFINITE(_mission_position.alt_m));

		if (_mission_position.valid) {
			PX4_INFO("Mission position used lat: %.8f [deg] -- lon: %.8f [deg] -- alt %.2f [m]", lat_deg,
				 lon_deg, (double)(alt_m));

		} else {
			PX4_INFO("Mission position not used because not valid. lat: %.8f [deg] -- lon: %.8f [deg] -- alt %.2f [m]",
				 lat_deg,
				 lon_deg, (double)(alt_m));
		}

	} else {
		_mission_position.valid = false;
	}
}

void VTEPosition::updateParams()
{
	ModuleParams::updateParams();

	_target_acc_unc = _param_vte_acc_t_unc.get();
	_bias_unc = _param_vte_bias_unc.get();
	_drone_acc_unc = _param_vte_acc_d_unc.get();
	_gps_vel_noise = _param_vte_gps_vel_noise.get();
	_gps_pos_noise = _param_vte_gps_pos_noise.get();
	_ev_noise_md = _param_vte_ev_noise_md.get();
	_ev_pos_noise = _param_vte_ev_pos_noise.get();
	_vte_aid_mask = _param_vte_aid_mask.get();
	_nis_threshold = _param_vte_pos_nis_thre.get();
}

bool VTEPosition::initTargetEstimator()
{

	// Array to hold temporary pointers
	KF_position_unified *tmp[Direction::nb_directions] = {nullptr, nullptr, nullptr};
	bool init_failed = false;

	// Try to allocate new estimators
	for (int dir = 0; dir < Direction::nb_directions; ++dir) {
		tmp[dir] = new KF_position_unified;

		if (tmp[dir] == nullptr) {
			init_failed = true;
			break;
		}
	}

	if (init_failed) {
		// Clean up any allocated estimators
		for (int dir = 0; dir < Direction::nb_directions; ++dir) {
			delete tmp[dir];
			tmp[dir] = nullptr;
		}

		return false;

	} else {
		// Replace old estimators with new ones
		for (int dir = 0; dir < Direction::nb_directions; ++dir) {
			delete _target_estimator[dir];
			_target_estimator[dir] = tmp[dir];
			tmp[dir] = nullptr; // Prevent deletion in the cleanup loop
		}

		return true;
	}
}

} // namespace vision_target_estimator
