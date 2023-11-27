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
 * @brief Estimate the state of a target by processessing and fusing sensor data in a Kalman Filter.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

#include "VTEPosition.h"

#define SEC2USEC 1000000.0f

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
	_vte_aid_gps_vel_rel_pub.advertise();
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
	bool return_bool = false;

	_target_mode = (TargetMode)_param_vte_mode.get();
	_vte_aid_mask = _param_vte_aid_mask.get();
	_vte_TIMEOUT_US = (uint32_t)(_param_vte_btout.get() * SEC2USEC);

	if (selectTargetEstimator()) {

		if (_vte_aid_mask == 0) {
			PX4_ERR("VTE: no data fusion enabled. Modify VTE_AID_MASK and reboot");
			return false;
		}

		if (_vte_aid_mask & SensorFusionMask::USE_MISSION_POS && _target_mode == TargetMode::Moving) {
			PX4_WARN("VTE mission land position data fusion cannot be enabled for moving targets.");
			PX4_WARN("Disabling mission land position fusion.");
			_vte_aid_mask -= SensorFusionMask::USE_MISSION_POS;
		}

		if ((_vte_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) && (_vte_aid_mask & SensorFusionMask::USE_MISSION_POS)) {
			PX4_WARN("VTE both target GPS position and mission land position data fusion enabled.");
			PX4_WARN("Disabling mission land position fusion.");
			_vte_aid_mask -= SensorFusionMask::USE_MISSION_POS;
		}

		if (_vte_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) { PX4_INFO("VTE target GPS position data fusion enabled");}

		if (_vte_aid_mask & SensorFusionMask::USE_MISSION_POS) { PX4_INFO("VTE PX4 mission position fusion enabled");}

		if (_vte_aid_mask & SensorFusionMask::USE_GPS_REL_VEL) { PX4_INFO("VTE relative GPS velocity data fusion enabled");}

		if (_vte_aid_mask & SensorFusionMask::USE_EXT_VIS_POS) { PX4_INFO("VTE target external vision-based relative position data fusion enabled");}

		return_bool = true;
	}

	return return_bool;
}

void VTEPosition::resetFilter()
{
	_estimator_initialized = false;
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

bool VTEPosition::initEstimator(const Vector3f &pos_init, const Vector3f &vel_init, const Vector3f &target_acc_init,
				const Vector3f &bias_init, const Vector3f &target_vel_init)
{

	PX4_INFO("Pos init %.2f %.2f %.2f", (double)pos_init(0), (double)pos_init(1), (double)pos_init(2));
	PX4_INFO("Vel init %.2f %.2f %.2f", (double)vel_init(0), (double)vel_init(1), (double)vel_init(2));
	PX4_INFO("Target acc init %.2f %.2f %.2f", (double)target_acc_init(0), (double)target_acc_init(1),
		 (double)target_acc_init(2));
	PX4_INFO("Target vel init %.2f %.2f %.2f", (double)target_vel_init(0), (double)target_vel_init(1),
		 (double)target_vel_init(2));
	PX4_INFO("Bias init %.2f %.2f %.2f", (double)bias_init(0), (double)bias_init(1), (double)bias_init(2));

	const float state_pos_var = _param_vte_pos_unc_in.get();
	const float state_vel_var = _param_vte_vel_unc_in.get();
	const float state_bias_var = _param_vte_bias_unc_in.get();
	const float state_acc_var = _param_vte_acc_unc_in.get();
	const float state_target_vel_var = _param_vte_vel_unc_in.get();

	const Vector3f state_pos_var_vect(state_pos_var, state_pos_var, state_pos_var);
	const Vector3f state_vel_var_vect(state_vel_var, state_vel_var, state_vel_var);
	const Vector3f state_bias_var_vect(state_bias_var, state_bias_var, state_bias_var);
	const Vector3f state_acc_var_vect(state_acc_var, state_acc_var, state_acc_var);
	const Vector3f state_target_vel_var_vect(state_target_vel_var, state_target_vel_var, state_target_vel_var);

	Vector3f state_target_vel;

	for (int i = 0; i < 3; i++) {

		/* Set filter initial state */
		_target_estimator[i]->setPosition(pos_init(i));
		_target_estimator[i]->setVelocity(vel_init(i));
		_target_estimator[i]->setBias(bias_init(i));
		_target_estimator[i]->setTargetAcc(target_acc_init(i));
		_target_estimator[i]->setTargetVel(state_target_vel(i));

		/* Set initial state variance */
		_target_estimator[i]->setStatePosVar(state_pos_var_vect(i));
		_target_estimator[i]->setStateVelVar(state_vel_var_vect(i));
		_target_estimator[i]->setStateBiasVar(state_bias_var_vect(i));
		_target_estimator[i]->setStateAccVar(state_acc_var_vect(i));
		_target_estimator[i]->setStateTargetVelVar(state_target_vel_var_vect(i));
	}

	return true;
}


void VTEPosition::predictionStep(const Vector3f &vehicle_acc_ned)
{
	// predict target position with the help of accel data

	// Time from last prediciton
	const float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC;

	// The rotated input cov (from body to NED R*cov*R^T) is the same as the original input cov since input_cov = acc_unc * Identiy and R*R^T = Identity
	const SquareMatrix<float, 3> input_cov = diag(Vector3f(_drone_acc_unc, _drone_acc_unc, _drone_acc_unc));
	const SquareMatrix<float, 3> target_acc_cov = diag(Vector3f(_target_acc_unc, _target_acc_unc, _target_acc_unc));
	const SquareMatrix<float, 3> bias_cov = diag(Vector3f(_bias_unc, _bias_unc, _bias_unc));

	for (int i = 0; i < 3; i++) {
		//Decoupled dynamics, we neglect the off diag elements.
		if (_target_mode == TargetMode::Moving) {_target_estimator[i]->setTargetAccVar(target_acc_cov(i, i));}

		_target_estimator[i]->setBiasVar(bias_cov(i, i));
		_target_estimator[i]->setInputAccVar(input_cov(i, i));

		_target_estimator[i]->predictState(dt, vehicle_acc_ned(i));
		_target_estimator[i]->predictCov(dt);
	}
}



bool VTEPosition::update_step(const Vector3f &vehicle_acc_ned)
{

	sensor_gps_s vehicle_gps_position;
	target_gnss_s target_GNSS_report;
	fiducial_marker_pos_report_s fiducial_marker_pose;

	targetObsPos obs_gps_pos_target;
	targetObsPos obs_gps_pos_mission;
	targetObsPos obs_gps_vel_rel;
	targetObsPos obs_gps_vel_target;
	targetObsPos obs_fiducial_marker;

	int vte_fusion_aid_mask = 0;

	// Process data from all topics

	/* VISION */
	if ((_vte_aid_mask & SensorFusionMask::USE_EXT_VIS_POS)
	    && _fiducial_marker_report_sub.update(&fiducial_marker_pose)) {

		obs_fiducial_marker.type = fiducial_marker;

		if ((_is_meas_valid(fiducial_marker_pose.timestamp)) && processObsVision(fiducial_marker_pose, obs_fiducial_marker)) {
			vte_fusion_aid_mask += ObservationValidMask::FUSE_EXT_VIS_POS;
		}
	}

	/* GPS BASED OBSERVATIONS */
	bool vehicle_gps_position_updated = _vehicle_gps_position_sub.update(&vehicle_gps_position);

	if (_is_meas_updated(vehicle_gps_position.timestamp)) {

		bool target_GPS_updated = _target_gnss_sub.update(&target_GNSS_report);

		/* TARGET GPS */
		if ((_vte_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) && target_GPS_updated
		    && target_GNSS_report.abs_pos_updated) {

			obs_gps_pos_target.type = target_gps_pos;

			if ((_is_meas_valid(target_GNSS_report.timestamp))
			    && processObsGNSSPosTarget(target_GNSS_report, vehicle_gps_position, obs_gps_pos_target)) {
				vte_fusion_aid_mask += ObservationValidMask::FUSE_TARGET_GPS_POS;
			}
		}

		/* MISSION GPS POSE */
		if ((_vte_aid_mask & SensorFusionMask::USE_MISSION_POS) && vehicle_gps_position_updated && _mission_position.valid) {

			obs_gps_pos_mission.type = mission_gps_pos;

			if ((_is_meas_valid(vehicle_gps_position.timestamp))
			    && processObsGNSSPosMission(vehicle_gps_position, obs_gps_pos_mission)) {
				vte_fusion_aid_mask += ObservationValidMask::FUSE_MISSION_POS;
			}
		}

		// Keep track of the drone GPS velocity
		_uav_gps_vel.timestamp = vehicle_gps_position.timestamp;
		_uav_gps_vel.valid = (vehicle_gps_position.vel_ned_valid && (PX4_ISFINITE(vehicle_gps_position.vel_n_m_s)
				      && PX4_ISFINITE(vehicle_gps_position.vel_e_m_s) && PX4_ISFINITE(vehicle_gps_position.vel_d_m_s)));
		_uav_gps_vel.xyz(0) = vehicle_gps_position.vel_n_m_s;
		_uav_gps_vel.xyz(1) = vehicle_gps_position.vel_e_m_s;
		_uav_gps_vel.xyz(2) = vehicle_gps_position.vel_d_m_s;

		// Keep track of the target GPS velocity
		_target_gps_vel.timestamp = target_GNSS_report.timestamp;
		_target_gps_vel.valid = (target_GNSS_report.vel_ned_updated && PX4_ISFINITE(target_GNSS_report.vel_n_m_s)
					 && PX4_ISFINITE(target_GNSS_report.vel_e_m_s)
					 && PX4_ISFINITE(target_GNSS_report.vel_d_m_s));
		_target_gps_vel.xyz(0) = target_GNSS_report.vel_n_m_s;
		_target_gps_vel.xyz(1) = target_GNSS_report.vel_e_m_s;
		_target_gps_vel.xyz(2) = target_GNSS_report.vel_d_m_s;

		if ((_vte_aid_mask & SensorFusionMask::USE_GPS_REL_VEL)) {

			/* TARGET GPS VELOCITY */
			if (_target_mode == TargetMode::Moving && target_GPS_updated && _target_gps_vel.valid) {

				obs_gps_vel_target.type = vel_target_gps;

				if ((_is_meas_valid(target_GNSS_report.timestamp)) && processObsGNSSVelTarget(target_GNSS_report, obs_gps_vel_target)) {
					vte_fusion_aid_mask += ObservationValidMask::FUSE_TARGET_GPS_VEL;
				}
			}

			/* RELATIVE GPS velocity */
			if (_uav_gps_vel.valid && vehicle_gps_position_updated) {

				obs_gps_vel_rel.type = vel_rel_gps;

				if ((_is_meas_valid(vehicle_gps_position.timestamp)) && processObsGNSSVelRel(vehicle_gps_position, obs_gps_vel_rel)) {
					vte_fusion_aid_mask += ObservationValidMask::FUSE_GPS_REL_VEL;
				}
			}
		}
	}

	const bool new_gnss_pos_sensor = (vte_fusion_aid_mask & ObservationValidMask::FUSE_MISSION_POS) ||
					 (vte_fusion_aid_mask & ObservationValidMask::FUSE_TARGET_GPS_POS);
	const bool new_non_gnss_pos_sensor = (vte_fusion_aid_mask & ObservationValidMask::FUSE_EXT_VIS_POS);
	const bool new_pos_sensor = new_non_gnss_pos_sensor || new_gnss_pos_sensor;
	const bool new_vel_sensor = (vte_fusion_aid_mask & ObservationValidMask::FUSE_TARGET_GPS_VEL) ||
				    (vte_fusion_aid_mask & ObservationValidMask::FUSE_GPS_REL_VEL);

	// Only estimate the GNSS bias if we have a GNSS estimation and a secondary source of position
	const bool should_set_bias = new_non_gnss_pos_sensor && (_is_meas_valid(_pos_rel_gnss.timestamp));

	if (!_estimator_initialized) {

		if (!new_pos_sensor) {
			return false;
		}

		const bool has_initial_velocity_estimate = (_local_velocity.valid && (_is_meas_valid(_local_velocity.timestamp))) ||
				(_uav_gps_vel.valid && (_is_meas_valid(_uav_gps_vel.timestamp)));

		if (!has_initial_velocity_estimate) {
			PX4_WARN("No UAV velocity estimate. Estimator cannot be started.");
			return false;
		}

		Vector3f pos_init;
		Vector3f vel_init;
		Vector3f target_acc_init;	// Assume null target absolute acceleration
		Vector3f bias_init;
		Vector3f target_vel_init;

		// Define the initial relative position of target w.r.t. the drone in NED frame using the available measurement
		if (vte_fusion_aid_mask & ObservationValidMask::FUSE_EXT_VIS_POS) {
			pos_init = obs_fiducial_marker.meas_xyz;

		} else if (vte_fusion_aid_mask & ObservationValidMask::FUSE_TARGET_GPS_POS) {
			pos_init = obs_gps_pos_target.meas_xyz;

		} else if (vte_fusion_aid_mask & ObservationValidMask::FUSE_MISSION_POS) {
			pos_init = obs_gps_pos_mission.meas_xyz;
		}

		// Compute the initial bias as the difference between the GPS and external position estimate.
		if (should_set_bias) {
			// We assume that gnss observations have a bias but other position obs don't. It follows: gnss_obs = state + bias <--> bias = gnss_obs - state
			PX4_INFO("VTE Position setting GNSS bias.");
			bias_init =  _pos_rel_gnss.xyz - pos_init;
			_bias_set = true;
		}

		if (_target_gps_vel.valid && (_is_meas_valid(_target_gps_vel.timestamp))) {
			target_vel_init = _target_gps_vel.xyz;
		}

		// Define initial relative velocity of the target w.r.t. to the drone in NED frame
		if (_uav_gps_vel.valid && (_is_meas_valid(_uav_gps_vel.timestamp))) {

			if (_target_mode == TargetMode::Stationary) {
				vel_init = -_uav_gps_vel.xyz;

			} else if (_target_mode == TargetMode::Moving) {
				vel_init = _uav_gps_vel.xyz;
			}

		} else if (_local_velocity.valid && (_is_meas_valid(_local_velocity.timestamp))) {

			if (_target_mode == TargetMode::Stationary) {
				vel_init = -_local_velocity.xyz;

			} else if (_target_mode == TargetMode::Moving) {
				vel_init = _local_velocity.xyz;
			}
		}

		if (initEstimator(pos_init, vel_init, target_acc_init, bias_init, target_vel_init)) {
			PX4_INFO("VTE Position Estimator properly initialized.");
			_estimator_initialized = true;
			_uav_gps_vel.valid = false;
			_target_gps_vel.valid = false;
			_last_update = hrt_absolute_time();
			_last_predict = _last_update;

		} else {
			resetFilter();
		}
	}

	if (!_estimator_initialized) {
		return false;
	}

	// Once a position measurement other than the target GPS is available, reset the position and bias but keep the velocity estimate.
	if (!_bias_set && should_set_bias) {

		PX4_INFO("Second relative position measurement available, re-setting position and bias.");

		const float state_pos_var = _param_vte_pos_unc_in.get();
		const float state_bias_var = _param_vte_bias_unc_in.get();

		const Vector3f state_pos_var_vect(state_pos_var, state_pos_var, state_pos_var);
		const Vector3f state_bias_var_vect(state_bias_var, state_bias_var, state_bias_var);
		const Vector3f pos_init = obs_fiducial_marker.meas_xyz;

		// Compute the initial bias as the difference between the GPS and external position estimate.
		// We assume that gnss observations have a bias but other position obs don't. It follows: gnss_obs = state + bias <--> bias = gnss_obs - state
		const Vector3f bias_init =  _pos_rel_gnss.xyz - pos_init;

		for (int i = 0; i < 3; i++) {
			/* Reset filter initial state */
			_target_estimator[i]->setPosition(pos_init(i));
			_target_estimator[i]->setBias(bias_init(i));

			/* Reset initial state variance */
			_target_estimator[i]->setStatePosVar(state_pos_var_vect(i));
			_target_estimator[i]->setStateBiasVar(state_bias_var_vect(i));
		}

		_bias_set = true;
	}

	// If we have a new sensor: fuse available measurements and publish innov.
	if (new_pos_sensor || new_vel_sensor) {

		bool pos_fused = false;

		/*TARGET GPS*/
		if (vte_fusion_aid_mask & ObservationValidMask::FUSE_TARGET_GPS_POS) {

			if (fuse_meas(vehicle_acc_ned, obs_gps_pos_target)) {
				pos_fused = true;
			}
		}

		/*MISSION POS GPS*/
		if (vte_fusion_aid_mask & ObservationValidMask::FUSE_MISSION_POS) {

			if (fuse_meas(vehicle_acc_ned, obs_gps_pos_mission)) {
				pos_fused = true;
			}
		}

		/*GPS RELATIVE VELOCITY*/
		if (vte_fusion_aid_mask & ObservationValidMask::FUSE_GPS_REL_VEL) {
			fuse_meas(vehicle_acc_ned, obs_gps_vel_rel);
		}

		/*TARGET GPS VELOCITY*/
		if (vte_fusion_aid_mask & ObservationValidMask::FUSE_TARGET_GPS_VEL) {
			fuse_meas(vehicle_acc_ned, obs_gps_vel_target);
		}

		/*VISION*/
		if (vte_fusion_aid_mask & ObservationValidMask::FUSE_EXT_VIS_POS) {

			if (fuse_meas(vehicle_acc_ned, obs_fiducial_marker)) {
				pos_fused = true;
			}
		}

		// If at least one pos measurement was fused, consider the filter updated
		return pos_fused;

	}

	return false;
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
	SquareMatrix<float, 3> Cov_rotated;

	// Use uncertainty from parameters or from vision messages
	if (_ev_noise_md) {
		// Uncertainty proportional to the vertical distance
		const float meas_uncertainty = _range_sensor.valid ? ((_ev_pos_noise * _ev_pos_noise) * fmaxf(_range_sensor.dist_bottom,
					       1.f)) : ((_ev_pos_noise * _ev_pos_noise) * 10);
		Cov_rotated = diag(Vector3f(meas_uncertainty, meas_uncertainty, meas_uncertainty));

	} else {
		const SquareMatrix<float, 3> covMat = diag(Vector3f(fmaxf(fiducial_marker_pose.var_x_rel_body,
						      _ev_pos_noise * _ev_pos_noise),
						      fmaxf(fiducial_marker_pose.var_y_rel_body, _ev_pos_noise * _ev_pos_noise),
						      fmaxf(fiducial_marker_pose.var_z_rel_body, _ev_pos_noise * _ev_pos_noise)));
		const matrix::Dcmf R_att = matrix::Dcm<float>(quat_att);
		Cov_rotated = R_att * covMat * R_att.transpose();
	}

	/* RELATIVE POSITION*/
	if (!PX4_ISFINITE(vision_ned(0)) || !PX4_ISFINITE(vision_ned(1)) || !PX4_ISFINITE(vision_ned(2))) {
		PX4_WARN("VISION position is corrupt!");

	} else {

		obs.timestamp = fiducial_marker_pose.timestamp;

		obs.updated_xyz.setAll(true);

		// Assume noise correlation negligible:

		// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
		obs.meas_h_xyz(0, 0) = 1;
		obs.meas_h_xyz(1, 1) = 1;
		obs.meas_h_xyz(2, 2) = 1;

		obs.meas_xyz = vision_ned;

		// Assume off diag elements ~ 0
		obs.meas_unc_xyz(0) = Cov_rotated(0, 0);
		obs.meas_unc_xyz(1) = Cov_rotated(1, 1);
		obs.meas_unc_xyz(2) = Cov_rotated(2, 2);

		return true;
	}

	return false;
}

/*Drone GNSS velocity observation: [r_dotx, r_doty, r_dotz]*/
bool VTEPosition::processObsGNSSVelRel(const sensor_gps_s &vehicle_gps_position, targetObsPos &obs)
{

	// Make sure measurement are valid
	if (!PX4_ISFINITE(vehicle_gps_position.vel_n_m_s) || !PX4_ISFINITE(vehicle_gps_position.vel_e_m_s)
	    || !PX4_ISFINITE(vehicle_gps_position.vel_d_m_s)) {
		PX4_WARN("UAV GPS velocity is corrupt!");
		return false;

	} else {

		Vector3f vel_uav_ned(vehicle_gps_position.vel_n_m_s, vehicle_gps_position.vel_e_m_s, vehicle_gps_position.vel_d_m_s);

		if (_gps_pos_is_offset) {
			if ((_velocity_offset_ned.valid)
			    && ((_velocity_offset_ned.timestamp - vehicle_gps_position.timestamp) < measurement_updated_TIMEOUT_US)) {

				vel_uav_ned -= _velocity_offset_ned.xyz;

			} else {
				return false;
			}
		}

		if (_target_mode == TargetMode::Stationary) {
			obs.meas_xyz = vel_uav_ned * (-1.f);

		} else if (_target_mode == TargetMode::Moving) {
			obs.meas_xyz = vel_uav_ned;
		}


		// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
		// Obs: [r_dotx, r_doty, r_dotz]

		obs.meas_h_xyz(0, 3) = 1; // x direction
		obs.meas_h_xyz(1, 4) = 1; // y direction
		obs.meas_h_xyz(2, 5) = 1; // z direction

		const float unc = fmaxf(vehicle_gps_position.s_variance_m_s * vehicle_gps_position.s_variance_m_s,
					_gps_vel_noise * _gps_vel_noise);
		obs.meas_unc_xyz(0) = unc;
		obs.meas_unc_xyz(1) = unc;
		obs.meas_unc_xyz(2) = unc;

		obs.timestamp = vehicle_gps_position.timestamp;

		obs.updated_xyz.setAll(true);

		return true;
	}

	return false;

}

/*Target GNSS velocity observation: [r_dotx, r_doty, r_dotz]*/
bool VTEPosition::processObsGNSSVelTarget(const target_gnss_s &target_GNSS_report, targetObsPos &obs)
{

	// Make sure measurement are valid
	if (!PX4_ISFINITE(target_GNSS_report.vel_n_m_s) || !PX4_ISFINITE(target_GNSS_report.vel_e_m_s)
	    || !PX4_ISFINITE(target_GNSS_report.vel_d_m_s)) {
		PX4_WARN("Target GPS velocity is corrupt!");
		return false;

	} else {

		// If the target is moving, the relative velocity is expressed as the drone verlocity - the target velocity
		obs.meas_xyz(0) = target_GNSS_report.vel_n_m_s;
		obs.meas_xyz(1) = target_GNSS_report.vel_e_m_s;
		obs.meas_xyz(2) = target_GNSS_report.vel_d_m_s;

		const float unc = fmaxf(target_GNSS_report.s_variance_m_s * target_GNSS_report.s_variance_m_s,
					_gps_vel_noise * _gps_vel_noise);

		obs.meas_unc_xyz(0) = unc;
		obs.meas_unc_xyz(1) = unc;
		obs.meas_unc_xyz(2) = unc;

		// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz, vty, vty, vtz]
		// Obs: [vtx, vty, vtz]

		obs.meas_h_xyz(0, 12) = 1; // x direction
		obs.meas_h_xyz(1, 13) = 1; // y direction
		obs.meas_h_xyz(2, 14) = 1; // z direction

		obs.timestamp = target_GNSS_report.timestamp;

		obs.updated_xyz.setAll(true);

		return true;
	}

	return false;
}

/*Target GNSS mission observation: [rx + bx, ry + by, rz + bz]*/
bool VTEPosition::processObsGNSSPosMission(const sensor_gps_s &vehicle_gps_position, targetObsPos &obs)
{


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
	// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
	// Obs: [rx + bx, ry + by, rz + bz]

	// x direction H = [1, 0, 0, 0, 0, 0, 1, 0, 0, ...]
	obs.meas_h_xyz(0, 0) = 1;

	// y direction H = [0, 1, 0, 0, 0, 0, 0, 1, 0, ...]
	obs.meas_h_xyz(1, 1) = 1;

	// z direction H = [0, 0, 1, 0, 0, 0, 0, 0, 1, ...]
	obs.meas_h_xyz(2, 2) = 1;

	if (_bias_set) {
		obs.meas_h_xyz(0, 6) = 1;
		obs.meas_h_xyz(1, 7) = 1;
		obs.meas_h_xyz(2, 8) = 1;
	}

	obs.timestamp = vehicle_gps_position.timestamp;

	obs.meas_xyz = gps_relative_pos;

	obs.meas_unc_xyz(0) = gps_unc_horizontal;
	obs.meas_unc_xyz(1) = gps_unc_horizontal;
	obs.meas_unc_xyz(2) = gps_unc_vertical;

	obs.updated_xyz.setAll(true);

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
	// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
	// Obs: [rx + bx, ry + by, rz + bz]

	// x direction H = [1, 0, 0, 0, 0, 0, 1, 0, 0, ...]
	obs.meas_h_xyz(0, 0) = 1;

	// y direction H = [0, 1, 0, 0, 0, 0, 0, 1, 0, ...]
	obs.meas_h_xyz(1, 1) = 1;

	// z direction H = [0, 0, 1, 0, 0, 0, 0, 0, 1, ...]
	obs.meas_h_xyz(2, 2) = 1;

	if (_bias_set) {
		obs.meas_h_xyz(0, 6) = 1;
		obs.meas_h_xyz(1, 7) = 1;
		obs.meas_h_xyz(2, 8) = 1;
	}

	obs.timestamp = target_GNSS_report.timestamp;

	obs.meas_xyz = gps_relative_pos;

	obs.meas_unc_xyz(0) = gps_unc_horizontal;
	obs.meas_unc_xyz(1) = gps_unc_horizontal;
	obs.meas_unc_xyz(2) = gps_unc_vertical;

	obs.updated_xyz.setAll(true);

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
	Vector<bool, 3> meas_xyz_fused{};
	bool all_directions_fused = false;
	Vector<float, 15> meas_h_row;

	// Compute the measurement's time delay (difference between state and measurement time on validity)
	const float dt_sync_us = (_last_predict - target_pos_obs.timestamp);

	if (dt_sync_us > measurement_valid_TIMEOUT_US) {

		PX4_INFO("Obs i = %d rejected because too old. Time sync: %.2f [ms] > timeout: %.2f [ms]",
			 target_pos_obs.type,
			 (double)(dt_sync_us / 1000), (double)(measurement_valid_TIMEOUT_US / 1000));

		// No measurement update, set to false
		target_innov.fused = false;

	} else {

		// For debug: log the time sync
		target_innov.time_last_fuse = (int)(dt_sync_us / 1000);

		// Convert time sync to seconds
		const float dt_sync_s = dt_sync_us / SEC2USEC;

		// Fill the timestamp field of innovation
		target_innov.timestamp_sample = target_pos_obs.timestamp;
		target_innov.timestamp = hrt_absolute_time();

		//If the measurement of this filter (x,y or z) has not been updated:
		if (!target_pos_obs.updated_xyz(0) || !target_pos_obs.updated_xyz(1) || !target_pos_obs.updated_xyz(2)) {

			// No measurement
			PX4_DEBUG("Obs i = %d : at least one non-valid observation. x: %d, y: %d, z: %d", target_pos_obs.type,
				  target_pos_obs.updated_xyz(0),
				  target_pos_obs.updated_xyz(1), target_pos_obs.updated_xyz(2));

			// Set innovations to zero
			target_innov.fused = false;

		} else {

			// Loop over x,y,z directions.
			for (int j = 0; j < 3; j++) {

				//Get the corresponding row of the H matrix.
				meas_h_row = target_pos_obs.meas_h_xyz.row(j);

				// Move state back to the measurement time of validity. The state synchronized with the measurement will be used to compute the innovation.
				_target_estimator[j]->syncState(dt_sync_s, vehicle_acc_ned(j));
				_target_estimator[j]->setH(meas_h_row, j);
				// Compute innovations and fill thet target innovation message
				target_innov.innovation_variance[j] = _target_estimator[j]->computeInnovCov(
						target_pos_obs.meas_unc_xyz(j));
				target_innov.innovation[j] = _target_estimator[j]->computeInnov(target_pos_obs.meas_xyz(j));
				// Set the Normalized Innovation Squared (NIS) check threshold. Used to reject outlier measurements
				_target_estimator[j]->setNISthreshold(_nis_threshold);
				// Update step
				meas_xyz_fused(j) = _target_estimator[j]->update();

				target_innov.observation[j] = target_pos_obs.meas_xyz(j);
				target_innov.observation_variance[j] = target_pos_obs.meas_unc_xyz(j);

				// log test ratio defined as _innov / _innov_cov * _innov. If test_ratio > 3.84, no fusion
				target_innov.test_ratio[j] = _target_estimator[j]->getTestRatio();
			}
		}

		// If we have updated all three directions (x,y,z) for one relative position measurement, consider the state updated.
		if (meas_xyz_fused(0) && meas_xyz_fused(1) && meas_xyz_fused(2)) {

			all_directions_fused = true;

		} else {
			PX4_DEBUG("Obs i = %d : at least one direction not fused: . x: %d, y: %d, z: %d", target_pos_obs.type,
				  meas_xyz_fused(0),
				  meas_xyz_fused(1),
				  meas_xyz_fused(2));
		}

		target_innov.fused = all_directions_fused;
		target_innov.innovation_rejected = !all_directions_fused;
	}

	perf_end(_vte_update_perf);

	// Publish innovations
	switch (target_pos_obs.type) {
	case target_gps_pos:
		_vte_aid_gps_pos_target_pub.publish(target_innov);
		break;

	case mission_gps_pos:
		_vte_aid_gps_pos_mission_pub.publish(target_innov);
		break;

	case vel_rel_gps:
		_vte_aid_gps_vel_rel_pub.publish(target_innov);
		break;

	case vel_target_gps:
		_vte_aid_gps_vel_target_pub.publish(target_innov);
		break;

	case fiducial_marker:
		_vte_aid_fiducial_marker_pub.publish(target_innov);
		break;
	}

	return all_directions_fused;
}

void VTEPosition::publishTarget()
{

	vision_target_est_position_s vte_state{};

	landing_target_pose_s target_pose{};

	target_pose.timestamp = _last_predict;
	vte_state.timestamp = _last_predict;
	target_pose.is_static = (_target_mode == TargetMode::Stationary);

	target_pose.rel_pos_valid =	_is_meas_valid(_last_update);

	// Fill target pose
	target_pose.x_rel = _target_estimator[x]->getPosition();
	target_pose.y_rel = _target_estimator[y]->getPosition();
	target_pose.z_rel = _target_estimator[z]->getPosition();

	target_pose.cov_x_rel = _target_estimator[x]->getPosVar();
	target_pose.cov_y_rel = _target_estimator[y]->getPosVar();
	target_pose.cov_z_rel = _target_estimator[z]->getPosVar();

	target_pose.vx_rel = _target_estimator[x]->getVelocity();
	target_pose.vy_rel = _target_estimator[y]->getVelocity();
	target_pose.vz_rel = _target_estimator[z]->getVelocity();

	target_pose.cov_vx_rel = _target_estimator[x]->getVelVar();
	target_pose.cov_vy_rel = _target_estimator[y]->getVelVar();
	target_pose.cov_vz_rel = _target_estimator[z]->getVelVar();

	if (_target_mode == TargetMode::Moving) {

		vte_state.vx_target = _target_estimator[x]->getTargetVel();
		vte_state.vy_target = _target_estimator[y]->getTargetVel();
		vte_state.vz_target = _target_estimator[z]->getTargetVel();

		vte_state.cov_vx_target = _target_estimator[x]->getTargetVelVar();
		vte_state.cov_vy_target = _target_estimator[y]->getTargetVelVar();
		vte_state.cov_vz_target = _target_estimator[z]->getTargetVelVar();

		target_pose.vx_rel = vte_state.vx_target - target_pose.vx_rel;
		target_pose.vy_rel = vte_state.vy_target - target_pose.vy_rel;
		target_pose.vz_rel = vte_state.vz_target - target_pose.vz_rel;

		/* Var(aX + bY) = a^2 Var(x) + b^2 Var(y) + 2abCov(X,Y) */
		target_pose.cov_vx_rel += vte_state.cov_vx_target;
		target_pose.cov_vy_rel += vte_state.cov_vy_target;
		target_pose.cov_vz_rel += vte_state.cov_vz_target;
	}

	// Fill target estimator state
	vte_state.x_rel = target_pose.x_rel;
	vte_state.y_rel = target_pose.y_rel;
	vte_state.z_rel = target_pose.z_rel;

	vte_state.cov_x_rel = target_pose.cov_x_rel;
	vte_state.cov_y_rel = target_pose.cov_y_rel;
	vte_state.cov_z_rel = target_pose.cov_z_rel;

	vte_state.vx_rel = target_pose.vx_rel;
	vte_state.vy_rel = target_pose.vy_rel;
	vte_state.vz_rel = target_pose.vz_rel;

	vte_state.cov_vx_rel = target_pose.cov_vx_rel;
	vte_state.cov_vy_rel = target_pose.cov_vy_rel;
	vte_state.cov_vz_rel = target_pose.cov_vz_rel;

	vte_state.x_bias = _target_estimator[x]->getBias();
	vte_state.y_bias = _target_estimator[y]->getBias();
	vte_state.z_bias = _target_estimator[z]->getBias();

	vte_state.cov_x_bias = _target_estimator[x]->getBiasVar();
	vte_state.cov_y_bias = _target_estimator[y]->getBiasVar();
	vte_state.cov_z_bias = _target_estimator[z]->getBiasVar();

	if (_target_mode == TargetMode::Moving) {
		vte_state.ax_target = _target_estimator[x]->getAcceleration();
		vte_state.ay_target = _target_estimator[y]->getAcceleration();
		vte_state.az_target = _target_estimator[z]->getAcceleration();

		vte_state.cov_ax_target = _target_estimator[x]->getAccVar();
		vte_state.cov_ay_target = _target_estimator[y]->getAccVar();
		vte_state.cov_az_target = _target_estimator[z]->getAccVar();
	}

	// Prec land does not check target_pose.abs_pos_valid. Only send the target if abs pose valid.
	if (_local_position.valid && target_pose.rel_pos_valid) {
		target_pose.x_abs = target_pose.x_rel + _local_position.xyz(0);
		target_pose.y_abs = target_pose.y_rel + _local_position.xyz(1);
		target_pose.z_abs = target_pose.z_rel + _local_position.xyz(2);
		target_pose.abs_pos_valid = true;

		if (_target_mode == TargetMode::Moving) {
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
		}

		_targetPosePub.publish(target_pose);

	}

	_targetEstimatorStatePub.publish(vte_state);

	// If the target is static, use the relative to aid the EKF2 state estimation.
	// Check performed in EKF2 to use target vel: if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid)
	target_pose.rel_vel_valid = target_pose.is_static && _param_vte_ekf_aid.get() && target_pose.rel_pos_valid;

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

bool VTEPosition::selectTargetEstimator()
{
	Base_KF_decoupled *tmp_x = nullptr;
	Base_KF_decoupled *tmp_y = nullptr;
	Base_KF_decoupled *tmp_z = nullptr;

	switch (_target_mode) {
	case TargetMode::Stationary:

		tmp_x = new KF_xyzb_decoupled_static;
		tmp_y = new KF_xyzb_decoupled_static;
		tmp_z = new KF_xyzb_decoupled_static;
		PX4_INFO("VTE postion init for static target, [x,y,z,b] decoupled in x,y,z filters.");

		break;

	case TargetMode::Moving:

		tmp_x = new KF_xyzb_v_decoupled_moving;
		tmp_y = new KF_xyzb_v_decoupled_moving;
		tmp_z = new KF_xyzb_v_decoupled_moving;
		PX4_INFO("VTE position init for moving target, [x,y,z,b,v] decoupled in x,y,z filters.");

		break;

	case TargetMode::NotInit:
		break;
	}

	const bool init_failed = ((tmp_x == nullptr) || (tmp_y == nullptr) || (tmp_z == nullptr));

	if (init_failed) {
		PX4_ERR("VTE position init failed");
		return false;

	} else {

		delete _target_estimator[x];
		delete _target_estimator[y];
		delete _target_estimator[z];

		_target_estimator[x] = tmp_x;
		_target_estimator[y] = tmp_y;
		_target_estimator[z] = tmp_z;

		return true;
	}
}

} // namespace vision_target_estimator
