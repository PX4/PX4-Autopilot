/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

/*
 * @file LandingTargetEstimator.cpp
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 * @author Mohammed Kabir <kabir@uasys.io>
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

#include "LandingTargetEstimator.h"

#define SEC2USEC 1000000.0f

namespace landing_target_estimator
{

using namespace matrix;

LandingTargetEstimator::LandingTargetEstimator() :
	ModuleParams(nullptr)
{
	_targetPosePub.advertise();
	_targetEstimatorStatePub.advertise();
	_target_estimator_aid_gps_pos_pub.advertise();
	_target_estimator_aid_gps_vel_pub.advertise();
	_target_estimator_aid_vision_pub.advertise();
	_target_estimator_aid_irlock_pub.advertise();
	_target_estimator_aid_uwb_pub.advertise();

	_check_params(true);
}

LandingTargetEstimator::~LandingTargetEstimator()
{
	for (int i = 0; i < 3; i++) {
		delete _target_estimator[i];
	}

	delete _target_estimator_coupled;

	perf_free(_ltest_predict_perf);
	perf_free(_ltest_update_perf);
	perf_free(_ltest_update_full_perf);
}

void LandingTargetEstimator::resetFilter()
{
	_estimator_initialized = false;
	_new_pos_sensor_acquired_time = 0;
	_bias_set = false;
}

void LandingTargetEstimator::update()
{
	_check_params(false);

	// Get the drone's acceleration and check if the next waypoint is a land waypoint.
	accInput input;
	get_input(&input);

	// No attitude or acceleration or next waypoint is not land: early return;
	if (!input.acc_ned_valid || !_start_filter) {
		return;
	}

	// predict the target state using a constant relative acceleration model
	if (_estimator_initialized) {

		if (hrt_absolute_time() - _last_update > _ltest_TIMEOUT_US) {
			PX4_WARN("LTE estimator timeout");
			resetFilter();

		} else {
			const hrt_abstime ltest_predict_start = hrt_absolute_time();
			predictionStep(input.vehicle_acc_ned);
			perf_set_elapsed(_ltest_predict_perf, hrt_elapsed_time(&ltest_predict_start));

			_last_predict = hrt_absolute_time();
		}
	}

	// Update and fuse the observations and pulishes innovations
	if (update_step(input.vehicle_acc_ned)) {
		_last_update = _last_predict;
	}

	if (_estimator_initialized) {publishTarget();}
}

bool LandingTargetEstimator::initEstimator(Vector3f pos_init, Vector3f vel_rel_init, Vector3f a_init,
		Vector3f bias_init)
{

	PX4_INFO("Pos init %.2f %.2f %.2f", (double)pos_init(0), (double)pos_init(1), (double)pos_init(2));
	PX4_INFO("Vel init %.2f %.2f %.2f", (double)vel_rel_init(0), (double)vel_rel_init(1), (double)vel_rel_init(2));
	PX4_INFO("Acc init %.2f %.2f %.2f", (double)a_init(0), (double)a_init(1), (double)a_init(2));
	PX4_INFO("Bias init %.2f %.2f %.2f", (double)bias_init(0), (double)bias_init(1), (double)bias_init(2));

	float state_pos_var = _param_ltest_pos_unc_in.get();
	float state_vel_var = _param_ltest_vel_unc_in.get();
	float state_bias_var = _param_ltest_bias_unc_in.get();
	float state_acc_var = _param_ltest_acc_unc_in.get();

	Vector3f state_pos_var_vect(state_pos_var, state_pos_var, state_pos_var);
	Vector3f state_vel_var_vect(state_vel_var, state_vel_var, state_vel_var);
	Vector3f state_bias_var_vect(state_bias_var, state_bias_var, state_bias_var);
	Vector3f state_acc_var_vect(state_acc_var, state_acc_var, state_acc_var);

	if (_target_model == TargetModel::FullPoseCoupled) {

		_target_estimator_coupled->setPosition(pos_init);
		_target_estimator_coupled->setVelocity(vel_rel_init);
		_target_estimator_coupled->setTargetAcc(a_init);

		_target_estimator_coupled->setStatePosVar(state_pos_var_vect);
		_target_estimator_coupled->setStateVelVar(state_vel_var_vect);
		_target_estimator_coupled->setStateAccVar(state_acc_var_vect);

		_target_estimator_coupled->setBias(bias_init);
		_target_estimator_coupled->setStateBiasVar(state_bias_var_vect);

	} else {

		for (int i = 0; i < _nb_position_kf; i++) {
			_target_estimator[i]->setPosition(pos_init(i));
			_target_estimator[i]->setVelocity(vel_rel_init(i));
			_target_estimator[i]->setBias(bias_init(i));
			_target_estimator[i]->setTargetAcc(a_init(i));

			_target_estimator[i]->setStatePosVar(state_pos_var_vect(i));
			_target_estimator[i]->setStateVelVar(state_vel_var_vect(i));
			_target_estimator[i]->setStateBiasVar(state_bias_var_vect(i));
			_target_estimator[i]->setStateAccVar(state_acc_var_vect(i));
		}
	}

	return true;
}


void LandingTargetEstimator::predictionStep(Vector3f vehicle_acc_ned)
{
	// predict target position with the help of accel data

	// Time from last prediciton
	float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC;

	//TODO: eventually get the acc variance from the EKF:
	float drone_acc_unc = _param_ltest_acc_d_unc.get();
	SquareMatrix<float, 3> input_cov = diag(Vector3f(drone_acc_unc, drone_acc_unc, drone_acc_unc));

	// Rotate input covariance from body to NED (note _q_att was updated by get_input())
	Dcmf R_att = Dcm<float>(_q_att);
	input_cov = R_att * input_cov * R_att.transpose();

	SquareMatrix<float, 3> target_acc_cov = diag(Vector3f(_target_acc_unc, _target_acc_unc, _target_acc_unc));
	SquareMatrix<float, 3> bias_cov = diag(Vector3f(_bias_unc, _bias_unc, _bias_unc));

	if (_target_model == TargetModel::FullPoseCoupled) {

		if (_target_mode == TargetMode::Moving) {_target_estimator_coupled->setTargetAccVar(target_acc_cov);}

		_target_estimator_coupled->setBiasVar(bias_cov);
		_target_estimator_coupled->setInputAccVar(input_cov);

		_target_estimator_coupled->predictState(dt, vehicle_acc_ned);
		_target_estimator_coupled->predictCov(dt);

	} else {
		for (int i = 0; i < _nb_position_kf; i++) {
			//For decoupled dynamics, we neglect the off diag elements.
			if (_target_mode == TargetMode::Moving) {_target_estimator[i]->setTargetAccVar(target_acc_cov(i, i));}

			_target_estimator[i]->setBiasVar(bias_cov(i, i));
			_target_estimator[i]->setInputAccVar(input_cov(i, i));

			_target_estimator[i]->predictState(dt, vehicle_acc_ned(i));
			_target_estimator[i]->predictCov(dt);
		}
	}
}



bool LandingTargetEstimator::update_step(Vector3f vehicle_acc_ned)
{

	sensor_gps_s vehicle_gps_position;
	landing_target_gnss_s target_GNSS_report;
	landing_target_pose_s fiducial_marker_pose;
	irlock_report_s irlock_report;
	uwb_distance_s	uwb_distance;

	targetObsPos obs_target_gps_pos;
	targetObsPos obs_uav_gps_vel;
	targetObsPos obs_fiducial_marker;
	targetObsPos obs_irlock;
	targetObsPos obs_uwb;

	bool pos_GNSS_valid = false;
	bool target_GNSS_valid = false;
	bool uav_gps_vel_valid = false;
	bool fiducial_marker_valid = false;
	bool irlock_valid = false;
	bool uwb_valid = false;

	// Process data from all topics

	/*IRLOCK*/
	if ((_ltest_aid_mask & SensorFusionMask::USE_IRLOCK_POS) && _dist_bottom_valid
	    && _irlockReportSub.update(&irlock_report)) {

		obs_irlock.type = irlock;

		if (processObsIRlock(irlock_report, obs_irlock)) {
			irlock_valid = ((hrt_absolute_time() - obs_irlock.timestamp) < measurement_valid_TIMEOUT_US);
		}
	}

	/*UWB*/
	if ((_ltest_aid_mask & SensorFusionMask::USE_UWB_POS) && _dist_bottom_valid && _uwbDistanceSub.update(&uwb_distance)) {

		obs_uwb.type = uwb;

		if (processObsUWB(uwb_distance, obs_uwb)) {
			uwb_valid = ((hrt_absolute_time() - obs_uwb.timestamp) < measurement_valid_TIMEOUT_US);
		}
	}

	/*VISION*/
	if ((_ltest_aid_mask & SensorFusionMask::USE_EXT_VIS_POS)
	    && _fiducial_marker_report_sub.update(&fiducial_marker_pose)) {

		obs_fiducial_marker.type = fiducial_marker;

		if (processObsVision(fiducial_marker_pose, obs_fiducial_marker)) {
			fiducial_marker_valid = ((hrt_absolute_time() - obs_fiducial_marker.timestamp) < measurement_valid_TIMEOUT_US);
		}
	}

	// Observations requiering a drone GPS update
	if ((_target_model == TargetModel::FullPoseDecoupled || _target_model == TargetModel::FullPoseCoupled)
	    && ((_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) || (_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS)
		|| (_ltest_aid_mask & SensorFusionMask::USE_UAV_GPS_VEL))) {

		_vehicle_gps_position_sub.update(&vehicle_gps_position);

		if ((hrt_absolute_time() - vehicle_gps_position.timestamp < measurement_updated_TIMEOUT_US)) {

			target_GNSS_valid = _landing_target_gnss_sub.update(&target_GNSS_report);

			/*TARGET GPS*/
			if ((((_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) && target_GNSS_valid)
			     || ((_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS) && _nave_state_mission && _landing_pos.valid))) {

				obs_target_gps_pos.type = target_gps_pos;

				if (processObsTargetGNSS(target_GNSS_report, target_GNSS_valid, vehicle_gps_position, obs_target_gps_pos)) {
					pos_GNSS_valid	= ((hrt_absolute_time() - obs_target_gps_pos.timestamp) < measurement_valid_TIMEOUT_US);
				}
			}

			/*UAV GPS velocity*/
			if ((vehicle_gps_position.vel_ned_valid && (_ltest_aid_mask & SensorFusionMask::USE_UAV_GPS_VEL)) &&
			    (_target_mode == TargetMode::Stationary || (_target_mode == TargetMode::Moving && target_GNSS_valid))) {

				obs_uav_gps_vel.type = uav_gps_vel;

				if (processObsUavGNSSVel(target_GNSS_report, vehicle_gps_position, obs_uav_gps_vel)) {
					uav_gps_vel_valid = ((hrt_absolute_time() - obs_uav_gps_vel.timestamp) < measurement_valid_TIMEOUT_US);
				}
			}
		}
	}


	// If one pos measurement was updated, return true
	bool new_pos_sensor = pos_GNSS_valid || fiducial_marker_valid || irlock_valid || uwb_valid;
	bool new_non_gnss_pos_sensor = fiducial_marker_valid || irlock_valid || uwb_valid;
	bool new_vel_sensor = uav_gps_vel_valid;

	// Once a position measurement other than the target GPS is available, restart the filter and set the bias.
	if (!_bias_set && ((_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS)
			   || (_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS))
	    && (((hrt_absolute_time() - _pos_rel_gnss.timestamp) < measurement_valid_TIMEOUT_US) && new_non_gnss_pos_sensor)) {

		if (_estimator_initialized) {
			PX4_INFO("Second relative position measurement available, restarting filter.");
		}

		_estimator_initialized = false;
	}

	bool pos_fused = false;

	// If we have a new sensor and the estimator is initialized: fuse available measurements and publish innov.
	if ((new_pos_sensor || new_vel_sensor) && _estimator_initialized) {

		const hrt_abstime ltest_update_full_start = hrt_absolute_time();

		/*TARGET GPS*/
		if (pos_GNSS_valid) {

			if (fuse_meas(vehicle_acc_ned, obs_target_gps_pos)) {
				pos_fused = true;
			}
		}

		/*UAV GPS VELOCITY*/
		if (uav_gps_vel_valid) {

			fuse_meas(vehicle_acc_ned, obs_uav_gps_vel);
		}

		/*VISION*/
		if (fiducial_marker_valid) {

			if (fuse_meas(vehicle_acc_ned, obs_fiducial_marker)) {
				pos_fused = true;
			}
		}

		/*IRLOCK*/
		if (irlock_valid) {

			if (fuse_meas(vehicle_acc_ned, obs_irlock)) {
				pos_fused = true;
			}
		}

		/*UWB*/
		if (uwb_valid) {

			if (fuse_meas(vehicle_acc_ned, obs_uwb)) {
				pos_fused = true;
			}
		}

		perf_set_elapsed(_ltest_update_full_perf, hrt_elapsed_time(&ltest_update_full_start));

		// If at least one pos measurement was fused, consider the filter updated
		return pos_fused;

	} else if (new_pos_sensor && !_estimator_initialized) {

		// Wait 1 second before initilazing the estimator to have a velocity initial estimate.
		if (!_new_pos_sensor_acquired_time) {
			_new_pos_sensor_acquired_time = hrt_absolute_time();

		} else if ((hrt_absolute_time() - _new_pos_sensor_acquired_time) > 1000000) {

			Vector3f pos_init;
			Vector3f vel_rel_init;
			Vector3f acc_init;	// Assume null relative acceleration
			Vector3f bias_init; // TODO: init the GPS bias to vision - GNSS?

			// Define the initial relative position of target w.r.t. the drone in NED frame using the available measurement
			if (fiducial_marker_valid) {
				pos_init = obs_fiducial_marker.meas_xyz;

			} else if (irlock_valid) {
				pos_init = obs_irlock.meas_xyz;

			} else if (uwb_valid) {
				pos_init = obs_uwb.meas_xyz;

			} else if (pos_GNSS_valid) {
				pos_init = obs_target_gps_pos.meas_xyz;
			}

			// Compute the initial bias as the difference between the GPS and external position estimate.
			if (((hrt_absolute_time() - _pos_rel_gnss.timestamp) < measurement_valid_TIMEOUT_US) && new_non_gnss_pos_sensor) {
				// We assume that gnss observations have a bias but other position obs don't. It follows: gnss_obs = state + bias <--> bias = gnss_obs - state
				bias_init =  _pos_rel_gnss.xyz - pos_init;
				_bias_set = true;
			}

			// Define initial relative velocity of the target w.r.t. to the drone in NED frame
			if (_vel_rel_init.valid && ((hrt_absolute_time() - _vel_rel_init.timestamp) < measurement_valid_TIMEOUT_US)) {
				vel_rel_init = _vel_rel_init.xyz;
			}

			if (initEstimator(pos_init, vel_rel_init, acc_init, bias_init)) {
				PX4_INFO("LTE Position Estimator properly initialized.");
				_estimator_initialized = true;
				_vel_rel_init.valid = false;
				_last_update = hrt_absolute_time();
				_last_predict = _last_update;

			} else {
				resetFilter();
			}
		}
	}

	return false;
}

/*Vision observation: [rx, ry, rz]*/
bool LandingTargetEstimator::processObsVision(const landing_target_pose_s &fiducial_marker_pose, targetObsPos &obs)
{

	// Assume vision measurement is in NED frame.
	// Note: (The vision estimate is rotated into the NED navigation frame offboard to avoid sync issues (target detection needs time))

	// Fiducial marker measurements :
	float vision_r_x = fiducial_marker_pose.x_rel;
	float vision_r_y = fiducial_marker_pose.y_rel;
	float vision_r_z = fiducial_marker_pose.z_rel;

	// TODO: complete mavlink message to include covariance matrix rotated in NED.
	/*
		SquareMatrix<float, 3> R_rotated = diag(Vector3f(fiducial_marker_pose.cov_x_rel, fiducial_marker_pose.cov_y_rel, fiducial_marker_pose.cov_z_rel));
		R_rotated(0,1) = fiducial_marker_pose.cov_x_y_rel;
		R_rotated(0,2) = fiducial_marker_pose.cov_x_z_rel;
		R_rotated(1,2) = fiducial_marker_pose.cov_y_z_rel;
		R_rotated(1,0) = R_rotated(0,1);
		R_rotated(2,0) = R_rotated(0,2);
		R_rotated(2,1) = R_rotated(1,2);
	*/

	// For now assume that we are at 10m if no distance bottom is valid
	float meas_uncertainty = _dist_bottom_valid ? (_meas_unc * _dist_bottom) : (_meas_unc * 10);
	SquareMatrix<float, 3> R_rotated = diag(Vector3f(meas_uncertainty, meas_uncertainty, meas_uncertainty));

	/* RELATIVE POSITION*/
	if (!PX4_ISFINITE(vision_r_x) || !PX4_ISFINITE(vision_r_y) || !PX4_ISFINITE(vision_r_y)) {
		PX4_WARN("VISION position is corrupt!");

	} else {

		obs.timestamp = fiducial_marker_pose.timestamp;

		obs.updated_xyz.setAll(true);

		Vector3f Z(vision_r_x, vision_r_y, vision_r_z);

		if (_target_model == TargetModel::FullPoseCoupled) {

			// Process measurements for sequential update by diagonalizing the measurement covariance matrix

			// Sliced state: [r_x,r_y,r_z]; (real state: [pose,vel,bias,acc] or [pose,vel,bias] but assume vision measurements independent of other measurements)
			Matrix<float, 3, 3> H_position;
			H_position(0, 0) = 1;
			H_position(1, 1) = 1;
			H_position(2, 2) = 1;

			// Cholesky decomposition R = L*D*L.T() to find L_inv = inv(L) such that R_diag = L_inv*R*L_inv.T() = D is diagonal:
			Matrix<float, 3, 3> L_inv = matrix::inv(matrix::choleskyLDLT(R_rotated));

			// Diagonalize R_rotated:
			Matrix<float, 3, 3> R_diag =  L_inv * R_rotated * L_inv.T();

			bool cholesky_failed = false;

			for (int i = 0; i < 3 ; i ++) {
				// Check if the diagonal matrix has zero or negative entries
				if (R_diag(i, i) < (float)1e-6) {
					cholesky_failed = true;
				}
			}

			if (cholesky_failed) {
				PX4_DEBUG("Cholesky decomposition failed, setting Linv to identity matrix");
				L_inv.identity();
				R_diag = R_rotated;
			}

			obs.meas_unc_xyz(0) = R_diag(0, 0);
			obs.meas_unc_xyz(1) = R_diag(1, 1);
			obs.meas_unc_xyz(2) = R_diag(2, 2);
			//TODO: replace by obs->meas_unc_xyz = R_diag.diag();

			//Transform measurements Z:
			Vector3f Z_transformed = L_inv * Z;

			obs.meas_xyz = Z_transformed;

			//Transform H
			Matrix<float, 3, 3> H_transformed = L_inv * H_position;

			//Bring H_position back to the full H: [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
			obs.meas_h_xyz(0, 0) = H_transformed(0, 0);
			obs.meas_h_xyz(0, 1) = H_transformed(0, 1);
			obs.meas_h_xyz(0, 2) = H_transformed(0, 2);

			obs.meas_h_xyz(1, 0) = H_transformed(1, 0);
			obs.meas_h_xyz(1, 1) = H_transformed(1, 1);
			obs.meas_h_xyz(1, 2) = H_transformed(1, 2);

			obs.meas_h_xyz(2, 0) = H_transformed(2, 0);
			obs.meas_h_xyz(2, 1) = H_transformed(2, 1);
			obs.meas_h_xyz(2, 2) = H_transformed(2, 2);

		} else {
			// Assume noise correlation negligible:

			// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
			obs.meas_h_xyz(0, 0) = 1;
			obs.meas_h_xyz(1, 1) = 1;
			obs.meas_h_xyz(2, 2) = 1;

			obs.meas_xyz = Z;

			// Assume off diag elements ~ 0
			obs.meas_unc_xyz(0) = R_rotated(0, 0);
			obs.meas_unc_xyz(1) = R_rotated(1, 1);
			obs.meas_unc_xyz(2) = R_rotated(2, 2);
		}

		return true;
	}

	return false;
}

/*Drone GNSS velocity observation: [r_dotx, r_doty, r_dotz]*/
bool LandingTargetEstimator::processObsUavGNSSVel(const landing_target_gnss_s &target_GNSS_report,
		const sensor_gps_s &vehicle_gps_position, targetObsPos &obs)
{

	// TODO: convert .s_variance_m_s from accuracy to variance

	if (_target_mode == TargetMode::Stationary) {

		obs.meas_xyz(0) = -vehicle_gps_position.vel_n_m_s;
		obs.meas_xyz(1) = -vehicle_gps_position.vel_e_m_s;
		obs.meas_xyz(2) = -vehicle_gps_position.vel_d_m_s;

		obs.meas_unc_xyz(0) = vehicle_gps_position.s_variance_m_s;
		obs.meas_unc_xyz(1) = vehicle_gps_position.s_variance_m_s;
		obs.meas_unc_xyz(2) = vehicle_gps_position.s_variance_m_s;

	} else {

		// If the target is moving, the relative velocity is expressed as the drone verlocity - the target velocity
		obs.meas_xyz(0) = vehicle_gps_position.vel_n_m_s - target_GNSS_report.vel_n_m_s;
		obs.meas_xyz(1) = vehicle_gps_position.vel_e_m_s - target_GNSS_report.vel_e_m_s;
		obs.meas_xyz(2) = vehicle_gps_position.vel_d_m_s - target_GNSS_report.vel_d_m_s;

		// TODO: uncomment once the mavlink message is updated with covariances
		// float unc = vehicle_gps_position.s_variance_m_s + target_GNSS_report.s_variance_m_s;
		float unc = vehicle_gps_position.s_variance_m_s + _gps_target_unc;
		obs.meas_unc_xyz(0) = unc;
		obs.meas_unc_xyz(1) = unc;
		obs.meas_unc_xyz(2) = unc;
	}

	// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
	// Obs: [r_dotx, r_doty, r_dotz]

	obs.meas_h_xyz(0, 3) = 1; // x direction
	obs.meas_h_xyz(1, 4) = 1; // y direction
	obs.meas_h_xyz(2, 5) = 1; // z direction

	obs.timestamp = vehicle_gps_position.timestamp;

	obs.updated_xyz.setAll(true);

	// Keep track of the initial relative velocity
	_vel_rel_init.timestamp = vehicle_gps_position.timestamp;
	_vel_rel_init.valid = vehicle_gps_position.vel_ned_valid;
	_vel_rel_init.xyz = obs.meas_xyz;

	return true;

}

/*Target GNSS observation: [rx + bx, ry + by, rz + bz]*/
bool LandingTargetEstimator::processObsTargetGNSS(const landing_target_gnss_s &target_GNSS_report,
		bool target_GNSS_report_valid,  const sensor_gps_s &vehicle_gps_position, targetObsPos &obs)
{

	bool use_gps_measurements = false;

	int target_gps_lat;		// 1e-7 deg
	int target_gps_lon;		// 1e-7 deg
	float target_gps_alt;	// AMSL [mm]

	// TODO: convert .epv and .eph from accuracy to variance
	float gps_target_eph;
	float gps_target_epv;

	hrt_abstime gps_timestamp = vehicle_gps_position.timestamp;

	// Measurement comes from an actual GPS on the target
	if ((_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) && target_GNSS_report_valid) {

		use_gps_measurements = true;
		gps_timestamp = target_GNSS_report.timestamp;

		// If mission mode && landing position valid && fusion mission position
		if ((_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS) && _nave_state_mission && _landing_pos.valid) {

			// Average between the landing point and the target GPS position is performed
			// TODO once target_GNSS_report.epv is available: weighted average
			target_gps_lat = (int)((target_GNSS_report.lat + _landing_pos.lat) / 2);
			target_gps_lon = (int)((target_GNSS_report.lon + _landing_pos.lon) / 2);
			target_gps_alt = (target_GNSS_report.alt + _landing_pos.alt) / 2.f;

			gps_target_eph = _gps_target_unc;
			gps_target_epv = _gps_target_unc;

		} else {

			target_gps_lat = target_GNSS_report.lat;
			target_gps_lon = target_GNSS_report.lon;
			target_gps_alt = target_GNSS_report.alt;

			// TODO: complete mavlink message to include uncertainties.
			// gps_target_eph = target_GNSS_report.eph;
			// gps_target_epv = target_GNSS_report.epv;

			gps_target_eph = _gps_target_unc;
			gps_target_epv = _gps_target_unc;
		}

	} else if ((_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS) && _nave_state_mission && _landing_pos.valid) {

		use_gps_measurements = true;

		// Measurement of the landing target position comes from the mission item only
		target_gps_lat = _landing_pos.lat;
		target_gps_lon = _landing_pos.lon;
		target_gps_alt = _landing_pos.alt;

		gps_target_eph = _gps_target_unc;
		gps_target_epv = _gps_target_unc;
	}

	if (use_gps_measurements) {

		// Obtain GPS relative measurements in NED as target_global - uav_gps_global followed by global2local transformation
		Vector3f gps_relative_pos;
		get_vector_to_next_waypoint((vehicle_gps_position.lat / 1.0e7), (vehicle_gps_position.lon / 1.0e7),
					    (target_gps_lat / 1.0e7), (target_gps_lon / 1.0e7),
					    &gps_relative_pos(0), &gps_relative_pos(1));

		// Down direction (if the drone is above the target, the relative position is positive)
		gps_relative_pos(2) = (vehicle_gps_position.alt - target_gps_alt) / 1000.f; // transform mm to m

		// Var(aX - bY) = a^2 Var(X) + b^2Var(Y) - 2ab Cov(X,Y)
		float gps_unc_horizontal = vehicle_gps_position.eph + gps_target_eph;
		float gps_unc_vertical = vehicle_gps_position.epv + gps_target_epv;

		// GPS already in NED, no rotation required. STATE: [pose,vel,bias,acc]

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

		obs.timestamp = gps_timestamp;

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

	return false;

}

/*UWB observation: [rx, ry, rz]*/
bool LandingTargetEstimator::processObsUWB(const uwb_distance_s &uwb_distance, targetObsPos &obs)
{

	if (!PX4_ISFINITE((float)uwb_distance.position[0]) || !PX4_ISFINITE((float)uwb_distance.position[1]) ||
	    !PX4_ISFINITE((float)uwb_distance.position[2])) {
		PX4_WARN("UWB position is corrupt!");

	} else {

		obs.timestamp = uwb_distance.timestamp;

		// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
		obs.meas_h_xyz(0, 0) = 1; // x direction
		obs.meas_h_xyz(1, 1) = 1; // y direction
		obs.meas_h_xyz(2, 2) = 1; // z direction

		// The coordinate system is NED (north-east-down) with the position of the landing point relative to the vehicle.
		// the uwb_distance msg contains the Position in NED, Vehicle relative to LP. To change POV we negate every Axis:
		obs.meas_xyz(0) = -uwb_distance.position[0];
		obs.meas_xyz(1) = -uwb_distance.position[1];
		obs.meas_xyz(2) = -uwb_distance.position[2];

		obs.updated_xyz.setAll(true);

		float dist_z = _dist_bottom - _param_ltest_sens_pos_z.get();

		float measurement_uncertainty = _meas_unc * dist_z * dist_z;

		obs.meas_unc_xyz(0) = measurement_uncertainty;
		obs.meas_unc_xyz(1) = measurement_uncertainty;

		return true;
	}

	return false;
}


bool LandingTargetEstimator::processObsIRlock(const irlock_report_s &irlock_report, targetObsPos &obs)
{
	if (!PX4_ISFINITE(irlock_report.pos_y) || !PX4_ISFINITE(irlock_report.pos_x)) {
		PX4_WARN("IRLOCK position is corrupt!");

	} else {

		Vector3f sensor_ray; // ray pointing towards target in body frame
		sensor_ray(0) = irlock_report.pos_x * _param_ltest_scale_x.get(); // forward
		sensor_ray(1) = irlock_report.pos_y * _param_ltest_scale_y.get(); // right
		sensor_ray(2) = 1.0f;

		// rotate unit ray according to sensor orientation
		Dcmf S_att; //Orientation of the sensor relative to body frame
		S_att = get_rot_matrix(static_cast<enum Rotation>(_param_ltest_sens_rot.get()));
		sensor_ray = S_att * sensor_ray;

		// rotate the unit ray into the navigation frame
		Dcmf R_att = Dcm<float>(_q_att);
		sensor_ray = R_att * sensor_ray;

		//TODO: rotate measurement uncertainty S_att & R_att

		// z component of measurement safe, use this measurement
		if (fabsf(sensor_ray(2)) > 1e-6f) {

			float dist_z = _dist_bottom - _param_ltest_sens_pos_z.get();

			// scale the ray s.t. the z component has length of _uncertainty_scale
			float rel_pos_x = sensor_ray(0) / sensor_ray(2) * dist_z;
			float rel_pos_y = sensor_ray(1) / sensor_ray(2) * dist_z;
			float rel_pos_z = dist_z;

			// Adjust relative position according to sensor offset
			rel_pos_x += _param_ltest_sens_pos_x.get();
			rel_pos_y += _param_ltest_sens_pos_y.get();

			//TODO: For coupled dynamics: Cholesky decomposition as for vision obs to uncorrelate noise (find matrix T that diagonalizes R) z' = T*z; H' = T*H

			// Fill the observations for the irlock sensor
			obs.timestamp = irlock_report.timestamp;

			// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
			// Obs: [rx, ry, rz]
			obs.meas_h_xyz(0, 0) = 1; // x direction
			obs.meas_h_xyz(1, 1) = 1; // y direction
			obs.meas_h_xyz(2, 2) = 1; // z direction

			obs.meas_xyz(0) = rel_pos_x;
			obs.meas_xyz(1) = rel_pos_y;
			obs.meas_xyz(2) = rel_pos_z;

			obs.updated_xyz.setAll(true);

			float measurement_uncertainty = _meas_unc * dist_z * dist_z;

			obs.meas_unc_xyz(0) = measurement_uncertainty;
			obs.meas_unc_xyz(1) = measurement_uncertainty;
			obs.meas_unc_xyz(2) = measurement_uncertainty;

			return true;
		}
	}

	return false;
}

bool LandingTargetEstimator::fuse_meas(const Vector3f vehicle_acc_ned, const targetObsPos &target_pos_obs)
{
	const hrt_abstime ltest_update_start = hrt_absolute_time();

	estimator_aid_source_3d_s target_innov;
	Vector<bool, 3> meas_xyz_fused{};
	bool all_directions_fused = false;
	Vector<float, 12> meas_h_row;

	// Number of direction:  x,y,z for all filters excep for the horizontal filter: x,y
	int nb_update_directions = (_target_model == TargetModel::Horizontal) ? 2 : 3;

	// Compute the measurement's time delay (difference between state and measurement time on validity)
	const float dt_sync_us = (_last_predict - target_pos_obs.timestamp);

	if (dt_sync_us > measurement_valid_TIMEOUT_US) {

		PX4_INFO("Obs i = %d rejected because too old. Time sync: %.2f [seconds] > timeout: %.2f [seconds]",
			 target_pos_obs.type,
			 (double)(dt_sync_us / SEC2USEC), (double)(measurement_valid_TIMEOUT_US / SEC2USEC));

		// No measurement update, set to false
		for (int k = 0; k < 3; k++) {
			target_innov.fusion_enabled[k] = false;
			target_innov.fused[k] = false;
		}

	} else {

		// For debug: log the time sync
		target_innov.test_ratio[0] = dt_sync_us / SEC2USEC;

		// TODO: Eventually remove, for now to debug, assume prediction time = measurement time
		const float dt_sync_s = 0.f;

		// Convert time sync to seconds
		// const float dt_sync_s = dt_sync_us / SEC2USEC;

		// Fill the timestamp field of innovation
		target_innov.timestamp_sample = target_pos_obs.timestamp;
		target_innov.timestamp = hrt_absolute_time();

		// Loop over x,y,z directions. Note: even with coupled dynamics we have a sequential update of measurements in x,y,z directions separately
		for (int j = 0; j < nb_update_directions; j++) {

			//If the measurement of this filter (x,y or z) has not been updated:
			if (!target_pos_obs.updated_xyz(j)) {

				// No measurement
				PX4_DEBUG("Obs i = %d : at least one non-valid observation. x: %d, y: %d, z: %d", target_pos_obs.type,
					  target_pos_obs.updated_xyz(0),
					  target_pos_obs.updated_xyz(1), target_pos_obs.updated_xyz(2));

				// Set innovations to zero
				target_innov.fusion_enabled[j] = false;
				target_innov.fused[j] = false;

			} else {

				//Get the corresponding row of the H matrix.
				meas_h_row = target_pos_obs.meas_h_xyz.row(j);

				// Sync measurement using the prediction model
				if (_target_model == TargetModel::FullPoseCoupled) {

					// Move state back to the measurement time of validity. The state synchronized with the measurement will be used to compute the innovation.
					_target_estimator_coupled->syncState(dt_sync_s, vehicle_acc_ned);
					_target_estimator_coupled->setH(meas_h_row);
					// Compute innovations and fill thet target innovation message
					target_innov.innovation_variance[j] = _target_estimator_coupled->computeInnovCov(
							target_pos_obs.meas_unc_xyz(j));
					target_innov.innovation[j] = _target_estimator_coupled->computeInnov(target_pos_obs.meas_xyz(j));
					// Update step
					meas_xyz_fused(j) = _target_estimator_coupled->update();

				} else {
					// Move state back to the measurement time of validity. The state synchronized with the measurement will be used to compute the innovation.
					_target_estimator[j]->syncState(dt_sync_s, vehicle_acc_ned(j));
					_target_estimator[j]->setH(meas_h_row);
					// Compute innovations and fill thet target innovation message
					target_innov.innovation_variance[j] = _target_estimator[j]->computeInnovCov(
							target_pos_obs.meas_unc_xyz(j));
					target_innov.innovation[j] = _target_estimator[j]->computeInnov(target_pos_obs.meas_xyz(j));
					// Update step
					meas_xyz_fused(j) = _target_estimator[j]->update();
				}

				// Fill the target innovation message
				target_innov.fusion_enabled[j] = true;
				target_innov.innovation_rejected[j] = !meas_xyz_fused(j);
				target_innov.fused[j] = meas_xyz_fused(j);

				target_innov.observation[j] = target_pos_obs.meas_xyz(j);
				target_innov.observation_variance[j] = target_pos_obs.meas_unc_xyz(j);

				/*	TODO: fill those fields of estimator_aid_source_3d_s
					uint8 estimator_instance
					uint32 device_id
					uint64[3] time_last_fuse
					float32[3] test_ratio
				*/
			}
		}

		// If we have updated all three directions (x,y,z) for one relative position measurement, consider the state updated.
		if (meas_xyz_fused(0) && meas_xyz_fused(1) && (meas_xyz_fused(2) || _target_model == TargetModel::Horizontal)) {

			all_directions_fused = true;

		} else {
			PX4_DEBUG("Obs i = %d : at least one direction not fused: . x: %d, y: %d, z: %d", target_pos_obs.type,
				  meas_xyz_fused(0),
				  meas_xyz_fused(1),
				  meas_xyz_fused(2));
		}
	}

	perf_set_elapsed(_ltest_update_perf, hrt_elapsed_time(&ltest_update_start));

	// Publish innovations
	switch (target_pos_obs.type) {
	case target_gps_pos:
		_target_estimator_aid_gps_pos_pub.publish(target_innov);
		break;

	case uav_gps_vel:
		_target_estimator_aid_gps_vel_pub.publish(target_innov);
		break;

	case fiducial_marker:
		_target_estimator_aid_vision_pub.publish(target_innov);
		break;

	case irlock:
		_target_estimator_aid_irlock_pub.publish(target_innov);
		break;

	case uwb:
		_target_estimator_aid_uwb_pub.publish(target_innov);
		break;
	}

	return all_directions_fused;
}

void LandingTargetEstimator::publishTarget()
{
	target_estimator_state_s target_estimator_state{};

	landing_target_pose_s target_pose{};

	target_pose.timestamp = _last_predict;
	target_estimator_state.timestamp = _last_predict;
	target_pose.is_static = (_target_mode == TargetMode::Stationary);

	target_pose.rel_pos_valid = (hrt_absolute_time() - _last_update < landing_target_valid_TIMEOUT_US);

	// TODO: eventually set to true, but for testing we don't want to use the target as an external source of velocity in the EKF
	target_pose.rel_vel_valid = false;

	// target_pose.rel_orientation_valid = ;

	if (_target_model == TargetModel::FullPoseCoupled) {

		// Fill target pose
		Vector3f pos_vect = _target_estimator_coupled->getPositionVect();
		target_pose.x_rel = pos_vect(0);
		target_pose.y_rel = pos_vect(1);
		target_pose.z_rel = pos_vect(2);

		Vector3f vel_vect = _target_estimator_coupled->getVelocityVect();
		target_pose.vx_rel = vel_vect(0);
		target_pose.vy_rel = vel_vect(1);
		target_pose.vz_rel = vel_vect(2);

		// Fill target estimator state
		target_estimator_state.x_rel = pos_vect(0);
		target_estimator_state.y_rel = pos_vect(1);
		target_estimator_state.z_rel = pos_vect(2);

		Vector3f cov_pos_vect = _target_estimator_coupled->getPosVarVect();
		target_estimator_state.cov_x_rel = cov_pos_vect(0);
		target_estimator_state.cov_y_rel = cov_pos_vect(1);
		target_estimator_state.cov_z_rel = cov_pos_vect(2);

		target_estimator_state.vx_rel = vel_vect(0);
		target_estimator_state.vy_rel = vel_vect(1);
		target_estimator_state.vz_rel = vel_vect(2);

		Vector3f cov_vel_vect = _target_estimator_coupled->getVelVarVect();
		target_estimator_state.cov_vx_rel = cov_vel_vect(0);
		target_estimator_state.cov_vy_rel = cov_vel_vect(1);
		target_estimator_state.cov_vz_rel = cov_vel_vect(2);

		Vector3f bias_vect = _target_estimator_coupled->getBiasVect();
		target_estimator_state.x_bias = bias_vect(0);
		target_estimator_state.y_bias = bias_vect(1);
		target_estimator_state.z_bias = bias_vect(2);

		Vector3f cov_bias_vect = _target_estimator_coupled->getBiasVarVect();
		target_estimator_state.cov_x_bias = cov_bias_vect(0);
		target_estimator_state.cov_y_bias = cov_bias_vect(1);
		target_estimator_state.cov_z_bias = cov_bias_vect(2);

		if (_target_mode == TargetMode::Moving) {
			Vector3f acc_target_vect = _target_estimator_coupled->getAccelerationVect();
			target_estimator_state.ax_target = acc_target_vect(0);
			target_estimator_state.ay_target = acc_target_vect(1);
			target_estimator_state.az_target = acc_target_vect(2);

			Vector3f cov_acc_target_vect = _target_estimator_coupled->getAccVarVect();
			target_estimator_state.cov_ax_target = cov_acc_target_vect(0);
			target_estimator_state.cov_ay_target = cov_acc_target_vect(1);
			target_estimator_state.cov_az_target = cov_acc_target_vect(2);
		}

	} else {

		float rel_z = 0.f;

		if (_dist_bottom_valid) {
			rel_z = _dist_bottom - _param_ltest_sens_pos_z.get();
		}

		// Fill target pose
		target_pose.x_rel = _target_estimator[x]->getPosition();
		target_pose.y_rel = _target_estimator[y]->getPosition();
		target_pose.z_rel = _nb_position_kf > 2 ? _target_estimator[z]->getPosition() : rel_z;

		target_pose.vx_rel = _target_estimator[x]->getVelocity();
		target_pose.vy_rel = _target_estimator[y]->getVelocity();
		target_pose.vz_rel = _nb_position_kf > 2 ? _target_estimator[z]->getVelocity() : 0.f;

		// Fill target estimator state
		target_estimator_state.x_rel = _target_estimator[x]->getPosition();
		target_estimator_state.y_rel = _target_estimator[y]->getPosition();
		target_estimator_state.z_rel = _nb_position_kf > 2 ? _target_estimator[z]->getPosition() : rel_z;

		target_estimator_state.cov_x_rel = _target_estimator[x]->getPosVar();
		target_estimator_state.cov_y_rel = _target_estimator[y]->getPosVar();
		target_estimator_state.cov_z_rel = _nb_position_kf > 2 ? _target_estimator[z]->getPosVar() : 0.f;

		target_estimator_state.vx_rel = _target_estimator[x]->getVelocity();
		target_estimator_state.vy_rel = _target_estimator[y]->getVelocity();
		target_estimator_state.vz_rel = _nb_position_kf > 2 ? _target_estimator[z]->getVelocity() : 0.f;

		target_estimator_state.cov_vx_rel = _target_estimator[x]->getVelVar();
		target_estimator_state.cov_vy_rel = _target_estimator[y]->getVelVar();
		target_estimator_state.cov_vz_rel = _nb_position_kf > 2 ? _target_estimator[z]->getVelVar() : 0.f;

		target_estimator_state.x_bias = _target_estimator[x]->getBias();
		target_estimator_state.y_bias = _target_estimator[y]->getBias();
		target_estimator_state.z_bias = _nb_position_kf > 2 ? _target_estimator[z]->getBias() : 0.f;

		target_estimator_state.cov_x_bias = _target_estimator[x]->getBiasVar();
		target_estimator_state.cov_y_bias = _target_estimator[y]->getBiasVar();
		target_estimator_state.cov_z_bias = _nb_position_kf > 2 ? _target_estimator[z]->getBiasVar() : 0.f;

		if (_target_mode == TargetMode::Moving) {
			target_estimator_state.ax_target = _target_estimator[x]->getAcceleration();
			target_estimator_state.ay_target = _target_estimator[y]->getAcceleration();
			target_estimator_state.az_target = _nb_position_kf > 2 ? _target_estimator[z]->getAcceleration() : 0.f;

			target_estimator_state.cov_ax_target = _target_estimator[x]->getAccVar();
			target_estimator_state.cov_ay_target = _target_estimator[y]->getAccVar();
			target_estimator_state.cov_az_target = _nb_position_kf > 2 ? _target_estimator[z]->getAccVar() : 0.f;
		}
	}

	if (_local_pos.valid) {
		target_pose.x_abs = target_pose.x_rel + _local_pos.x;
		target_pose.y_abs = target_pose.y_rel + _local_pos.y;
		target_pose.z_abs = target_pose.z_rel + _local_pos.z;
		target_pose.abs_pos_valid = true;

	} else {
		target_pose.abs_pos_valid = false;
	}

	_targetPosePub.publish(target_pose);
	_targetEstimatorStatePub.publish(target_estimator_state);


	// TODO: decide what to do with Bias lim
	float bias_lim = _param_ltest_bias_lim.get();

	if (_target_model != TargetModel::Horizontal && ((float)fabs(target_estimator_state.x_bias) > bias_lim
			|| (float)fabs(target_estimator_state.y_bias) > bias_lim || (float)fabs(target_estimator_state.z_bias) > bias_lim)) {

		PX4_DEBUG("Bias exceeds limit: %.2f bias x: %.2f bias y: %.2f bias z: %.2f", (double)bias_lim,
			  (double)target_estimator_state.x_bias, (double)target_estimator_state.y_bias, (double)target_estimator_state.z_bias);

		// resetFilter();
	}

}

void LandingTargetEstimator::_check_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();
	}
}

void LandingTargetEstimator::get_input(accInput *input)
{
	vehicle_attitude_s	vehicle_attitude;
	vehicle_local_position_s	vehicle_local_position;
	vehicle_status_s vehicle_status;
	position_setpoint_triplet_s pos_sp_triplet;
	vehicle_acceleration_s		vehicle_acceleration;
	vehicle_land_detected_s vehicle_land_detected;

	//Update topics
	bool vehicle_local_position_valid = _vehicleLocalPositionSub.update(&vehicle_local_position);
	bool vehicle_attitude_valid = _attitudeSub.update(&vehicle_attitude);
	bool vehicle_acceleration_valid = _vehicle_acceleration_sub.update(&vehicle_acceleration);
	bool pos_sp_triplet_valid = _pos_sp_triplet_sub.update(&pos_sp_triplet);

	// Update nav state
	if (_vehicle_status_sub.update(&vehicle_status)) {
		_nave_state_mission = (vehicle_status.nav_state  == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	}

	// Stop computations once the drone has landed
	if (_start_filter && _vehicle_land_detected_sub.update(&vehicle_land_detected) && vehicle_land_detected.landed) {
		PX4_INFO("Land detected, target estimator stoped.");

		resetFilter();

		_start_filter = false;
		_landing_pos.valid = false;
		_land_time = hrt_absolute_time();

		return;
	}

	if (!_start_filter && ((hrt_absolute_time() - _land_time) > 5000000) && pos_sp_triplet_valid) {
		_start_filter = (pos_sp_triplet.next.type == position_setpoint_s::SETPOINT_TYPE_LAND);

		if (_start_filter) {

			if (_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS) {
				_landing_pos.lat = (int)(pos_sp_triplet.next.lat * 1e7);
				_landing_pos.lon = (int)(pos_sp_triplet.next.lon * 1e7);
				_landing_pos.alt = pos_sp_triplet.next.alt * 1000.f;
				_landing_pos.valid = (_landing_pos.lat != 0 && _landing_pos.lon != 0);

				if (_landing_pos.valid) {
					PX4_INFO("Landing pos used:  %.2f %.2f %.2f", (double)(pos_sp_triplet.next.lat),
						 (double)(pos_sp_triplet.next.lon), (double)(pos_sp_triplet.next.alt));

				} else {
					PX4_INFO("Landing target estimator detection enabled but landing pos not valid:  %.2f %.2f %.2f",
						 (double)(pos_sp_triplet.next.lat),
						 (double)(pos_sp_triplet.next.lon), (double)(pos_sp_triplet.next.alt));
				}
			}
		}
	}

	// To save stack space, only use x,y,z,valid as global variables (_local_pos is used when the target is published)
	_local_pos.x = vehicle_local_position.x;
	_local_pos.y = vehicle_local_position.y;
	_local_pos.z = vehicle_local_position.z;
	_local_pos.valid = (vehicle_local_position_valid && vehicle_local_position.xy_valid);

	_dist_bottom = vehicle_local_position.dist_bottom;
	_dist_bottom_valid = vehicle_local_position.dist_bottom_valid;

	// Minimal requirement: acceleraion (for input) and attitude (to rotate acc in vehicle-carried NED frame)
	if (!vehicle_attitude_valid || !vehicle_acceleration_valid) {
		// Only print if the estimator has been initialized
		if (_estimator_initialized) {
			PX4_INFO("Kalman input not available: Attitude: %d, Acc: %d", vehicle_attitude_valid, vehicle_acceleration_valid);
		}

		input->acc_ned_valid = false;
		return;

	} else {
		Quaternion<float> q_att(&vehicle_attitude.q[0]);
		_q_att = q_att;
		Dcmf R_att = Dcm<float>(_q_att);

		// Transform body acc to NED
		Vector3f vehicle_acc{vehicle_acceleration.xyz};

		// Compensate for gravity:
		Dcmf R_att_inv = inv(R_att);
		Vector3f gravity_ned(0, 0, 9.807);
		Vector3f gravity_body = R_att_inv * gravity_ned;

		input->vehicle_acc_ned = R_att * (vehicle_acc + gravity_body);
		input->acc_ned_valid = true;
	}
}

void LandingTargetEstimator::updateParams()
{

	ModuleParams::updateParams();

	const TargetMode param_target_mode = (TargetMode)_param_ltest_mode.get();
	const TargetModel param_target_model = (TargetModel)_param_ltest_model.get();
	_ltest_aid_mask = _param_ltest_aid_mask.get();

	_target_acc_unc = _param_ltest_acc_t_unc.get();
	_bias_unc = _param_ltest_bias_unc.get();
	_meas_unc = _param_ltest_meas_unc.get();
	_gps_target_unc = _param_ltest_gps_t_unc.get();

	PX4_INFO("LTE position estimator enabled.");

	if (_ltest_aid_mask == 0) { PX4_ERR("LTE no data fusion enabled. Modify LTEST_AID_MASK and reboot");}

	if (_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) { PX4_INFO("LTE target GPS position data fusion enabled");}

	if (_ltest_aid_mask & SensorFusionMask::USE_UAV_GPS_VEL) { PX4_INFO("LTE drone GPS velocity data fusion enabled");}

	if (_ltest_aid_mask & SensorFusionMask::USE_EXT_VIS_POS) { PX4_INFO("LTE target external vision-based relative position data fusion enabled");}

	if (_ltest_aid_mask & SensorFusionMask::USE_IRLOCK_POS) { PX4_INFO("LTE target relative position from irlock data fusion enabled");}

	if (_ltest_aid_mask & SensorFusionMask::USE_UWB_POS) { PX4_INFO("LTE target relative position from uwb data fusion enabled");}

	if (_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS) { PX4_INFO("LTE PX4 mission landing position fusion enabled");}

	if (_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS && _ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) { PX4_INFO("LTE a weighted average between the landing point and the target GPS position will be performed");}

	if (_target_mode != param_target_mode || _target_model != param_target_model) {

		// Define the target mode and model
		_target_mode = param_target_mode;
		_target_model = param_target_model;

		if (!selectTargetEstimator()) {
			// TODO: decide on behaviour
		}

		// Define LTEST timeout
		_ltest_TIMEOUT_US = (uint32_t)(_param_ltest_btout.get() * SEC2USEC);
	}

	switch (_target_model) {
	case TargetModel::FullPoseDecoupled:
		_nb_position_kf = 3;

		if ((_target_estimator[x] == nullptr) || (_target_estimator[y] == nullptr) || (_target_estimator[z] == nullptr)) {
			// TODO: should return false
			return;
		}

		break;

	case TargetModel::Horizontal:
		_nb_position_kf = 2;

		if ((_target_estimator[x] == nullptr) || (_target_estimator[y] == nullptr)) {
			return;
		}

		break;

	case TargetModel::FullPoseCoupled:
		_nb_position_kf = 1;

		if (_target_estimator_coupled == nullptr) {
			return;
		}

		break;

	case TargetModel::NotInit:
		return;
	}
}

bool LandingTargetEstimator::selectTargetEstimator()
{
	TargetEstimator *tmp_x = nullptr;
	TargetEstimator *tmp_y = nullptr;
	TargetEstimator *tmp_z = nullptr;
	TargetEstimatorCoupled *tmp_xyz = nullptr;

	bool init_failed = true;

	switch (_target_model) {
	case TargetModel::FullPoseDecoupled:

		if (_target_mode == TargetMode::Moving) {
			tmp_x = new KF_xyzb_decoupled_moving;
			tmp_y = new KF_xyzb_decoupled_moving;
			tmp_z = new KF_xyzb_decoupled_moving;
			PX4_INFO("Init LTEST: moving target, [x,y,z,b] decoupled in three filters.");

		} else {
			tmp_x = new KF_xyzb_decoupled_static;
			tmp_y = new KF_xyzb_decoupled_static;
			tmp_z = new KF_xyzb_decoupled_static;
			PX4_INFO("Init LTEST: static target, [x,y,z,b] decoupled in three filters.");
		}

		init_failed = (tmp_x == nullptr) || (tmp_y == nullptr) || (tmp_z == nullptr);

		break;

	case TargetModel::FullPoseCoupled:

		if (_target_mode == TargetMode::Moving) {

			tmp_xyz = new KF_xyzb_coupled_moving;
			PX4_INFO("Init LTEST: moving target, [x,y,z,b] coupled in one filter.");

		} else {

			tmp_xyz = new KF_xyzb_coupled_static;
			PX4_INFO("Init LTEST: static target, [x,y,z,b] coupled in one filter.");

		}

		init_failed = (tmp_xyz == nullptr);
		break;

	case TargetModel::Horizontal:
		tmp_x = new KalmanFilter();
		tmp_y = new KalmanFilter();
		PX4_INFO("LTE estimator: Horizontal position only.");

		init_failed = (tmp_x == nullptr) || (tmp_y == nullptr);
		break;

	case TargetModel::NotInit:
		init_failed = true;
		break;
	}

	if (init_failed) {
		PX4_ERR("LTE init failed");
		return false;
		// TODO: decide on a behaviour

	} else {

		switch (_target_model) {
		case TargetModel::FullPoseDecoupled:

			delete _target_estimator[x];
			delete _target_estimator[y];
			delete _target_estimator[z];

			_target_estimator[x] = tmp_x;
			_target_estimator[y] = tmp_y;
			_target_estimator[z] = tmp_z;

			break;

		case TargetModel::FullPoseCoupled:
			delete _target_estimator_coupled;
			_target_estimator_coupled = tmp_xyz;

			break;

		case TargetModel::Horizontal:
			delete _target_estimator[x];
			delete _target_estimator[y];

			_target_estimator[x] = tmp_x;
			_target_estimator[y] = tmp_y;

			break;

		case TargetModel::NotInit:
			break;
		}

		return true;
	}
}

} // namespace landing_target_estimator
