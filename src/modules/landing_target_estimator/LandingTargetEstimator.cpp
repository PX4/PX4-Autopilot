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

/*
	TODO:
		- Stop the target estimator module once disarmed (we have the nav state)
		- Complete the landing target MAVLINK message to include measurement covariance matrix
		- Get drone's acceleration uncertainty
		- Finish measurement processing: Get mission land point lat/lon
		- Orientation (the precision landing algorithm doesn't take the orientation into consideration yet):
			- Define initial values
			- Complete MAVLINK landing target message to include theta
		- Fill estimator_aid_source_3d_s missing fields (not too important for now)
*/

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

	delete _target_estimator_orientation;
	delete _target_estimator_coupled;
}

void LandingTargetEstimator::update()
{
	_check_params(false);

	// Get curent Body-NED rotation matrix, and sensor observations

	accInput input;

	get_input(&input);

	// No attitude or acceleration: early return;
	// Init the estimator when landing is the next setpoint
	if (!input.acc_ned_valid || !_start_detection) {
		return;
	}

	/* predict */
	if (_estimator_initialized) {

		if (hrt_absolute_time() - _last_update > _ltest_TIMEOUT_US) {
			PX4_WARN("Timeout");
			_estimator_initialized = false;
			_new_pos_sensor_acquired_time = 0;

		} else {
			predictionStep(input.vehicle_acc_ned);
			_last_predict = hrt_absolute_time();
		}
	}

	// This function updates the topics, updates the filter and pulishes innovations
	if (update_step(input.vehicle_acc_ned)) {
		_last_update = _last_predict;
	}

	// 		// Update orientation (Eventually use orientation_updated)
	// 		if (_estimate_orientation) {
	// 			updateOrientation();
	// 		}

	if (_estimator_initialized) {publishTarget();}
}

void LandingTargetEstimator::initEstimator(Vector3f pos_init, Vector3f vel_rel_init, Vector3f a_init,
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
		_target_estimator_coupled->setBias(bias_init);
		_target_estimator_coupled->setTargetAcc(a_init);

		_target_estimator_coupled->setStatePosVar(state_pos_var_vect);
		_target_estimator_coupled->setStateVelVar(state_vel_var_vect);
		_target_estimator_coupled->setStateBiasVar(state_bias_var_vect);
		_target_estimator_coupled->setStateAccVar(state_acc_var_vect);

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

	if (_estimate_orientation) {
		//TODO: define thse values
		_target_estimator_orientation->setPosition(0.f);
		_target_estimator_orientation->setVelocity(0.f);
		_target_estimator_orientation->setStatePosVar(state_pos_var);
		_target_estimator_orientation->setStateVelVar(state_vel_var);
	}
}


void LandingTargetEstimator::predictionStep(Vector3f vehicle_acc_ned)
{
	// predict target position with the help of accel data

	// Time from last prediciton
	float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC;

	//TODO: eventually get the acc variance from the PX4 EKF:
	float drone_acc_unc = _param_ltest_acc_d_unc.get();
	SquareMatrix<float, 3> input_cov = diag(Vector3f(drone_acc_unc, drone_acc_unc, drone_acc_unc));

	// Rotate input covariance from body to NED (note _q_att was updated by get_input())
	Dcmf R_att = Dcm<float>(_q_att);
	input_cov = R_att * input_cov * R_att.transpose();

	// TODO: This can eventually be moved to the updateParams function and set as a global variable because no rotation is required
	float target_acc_unc = _param_ltest_acc_t_unc.get();
	SquareMatrix<float, 3> target_acc_cov = diag(Vector3f(target_acc_unc, target_acc_unc, target_acc_unc));

	float bias_unc = _param_ltest_bias_unc.get();
	SquareMatrix<float, 3> bias_cov = diag(Vector3f(bias_unc, bias_unc, bias_unc));

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

	if (_estimate_orientation) {
		//Orientation (theta) prediction (no input)
		_target_estimator_orientation->predictState(dt, 0.0);
		_target_estimator_orientation->predictCov(dt);
	}
}



bool LandingTargetEstimator::update_step(Vector3f vehicle_acc_ned)
{

	sensor_gps_s vehicle_gps_position;
	landing_target_pose_s target_GNSS_report;
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

		if (processObsIRlock(irlock_report, &obs_irlock)) {
			irlock_valid = true;
		}
	}

	/*UWB*/
	if ((_ltest_aid_mask & SensorFusionMask::USE_UWB_POS) && _dist_bottom_valid && _uwbDistanceSub.update(&uwb_distance)) {

		obs_uwb.type = uwb;

		if (processObsUWB(uwb_distance, &obs_uwb)) {
			uwb_valid = true;
		}
	}

	/*VISION*/
	if ((_ltest_aid_mask & SensorFusionMask::USE_EXT_VIS_POS)
	    && _fiducial_marker_report_sub.update(&fiducial_marker_pose)) {

		obs_fiducial_marker.type = fiducial_marker;

		if (processObsVision(fiducial_marker_pose, &obs_fiducial_marker)) {
			fiducial_marker_valid = true;
		}
	}

	// Observations requiering a drone GPS update
	if ((_target_model == TargetModel::FullPoseDecoupled || _target_model == TargetModel::FullPoseCoupled)
	    && ((_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) || (_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS)
		|| (_ltest_aid_mask & SensorFusionMask::USE_UAV_GPS_VEL))) {

		_vehicle_gps_position_sub.update(&vehicle_gps_position);

		if ((hrt_absolute_time() - vehicle_gps_position.timestamp < measurement_updated_TIMEOUT_US)) {

			target_GNSS_valid = _target_GNSS_report_sub.update(&target_GNSS_report);

			/*TARGET GPS*/
			if ((((_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) && target_GNSS_valid)
			     || ((_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS)
				 && _nave_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION && _landing_pos.valid))) {

				obs_target_gps_pos.type = target_gps_pos;

				if (processObsTargetGNSS(target_GNSS_report, target_GNSS_valid, vehicle_gps_position, &obs_target_gps_pos)) {
					pos_GNSS_valid	= true;
				}
			}

			/*UAV GPS velocity*/
			if ((vehicle_gps_position.vel_ned_valid && (_ltest_aid_mask & SensorFusionMask::USE_UAV_GPS_VEL)) &&
			    (_target_mode == TargetMode::Stationary || (_target_mode == TargetMode::Moving && target_GNSS_valid))) {

				obs_uav_gps_vel.type = uav_gps_vel;

				if (processObsUavGNSSVel(target_GNSS_report, vehicle_gps_position, &obs_uav_gps_vel)) {
					uav_gps_vel_valid = true;
				}
			}
		}
	}


	// If one pos measurement was updated, return true
	bool new_pos_sensor = pos_GNSS_valid || fiducial_marker_valid || irlock_valid || uwb_valid;
	bool new_vel_sensor = uav_gps_vel_valid;

	bool pos_fused = false;

	// If we have a new sensor and the estimator is initialized: fuse available measurements and publish innov.
	if ((new_pos_sensor || new_vel_sensor) && _estimator_initialized) {

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

		// If at least one position measurement was fused, consider the filter updated
		return pos_fused;

	} else if (new_pos_sensor && !_estimator_initialized) {

		// Wait 1 second before initilazing the estimator to allow to have a velocity initial estimate.
		if (!_new_pos_sensor_acquired_time) {
			// target just became visible. Stop climbing, but give it some margin so we don't stop too apruptly
			_new_pos_sensor_acquired_time = hrt_absolute_time();

		} else if ((hrt_absolute_time() - _new_pos_sensor_acquired_time) > 1000000) {

			// TODO: do we want to init the target/UAV GPS bias?

			// Assume null relative acceleration
			Vector3f pos_init;
			Vector3f vel_rel_init;
			Vector3f acc_init;
			Vector3f bias_init;

			// Define the initial relative position of target w.r.t. the drone in NED frame using the available measurement
			if (pos_GNSS_valid) {
				pos_init = obs_target_gps_pos.meas_xyz;

			} else if (fiducial_marker_valid) {
				pos_init = obs_fiducial_marker.meas_xyz;

			} else if (irlock_valid) {
				pos_init = obs_irlock.meas_xyz;

			} else if (uwb_valid) {
				pos_init = obs_uwb.meas_xyz;

			} else {
				pos_init.zero();
			}

			// Define initial relative velocity of the target w.r.t. to the drone in NED frame
			if (_vel_rel_init.valid && (hrt_absolute_time() - _vel_rel_init.timestamp < measurement_valid_TIMEOUT_US)) {
				vel_rel_init = _vel_rel_init.vel;

			} else {
				vel_rel_init.zero();
			}

			initEstimator(pos_init, vel_rel_init, acc_init, bias_init);

			PX4_INFO("LTE Estimator properly initialized.");
			_estimator_initialized = true;
			_last_update = hrt_absolute_time();
			_last_predict = _last_update;
		}
	}

	return false;
}


bool LandingTargetEstimator::processObsVisionOrientation(const landing_target_pose_s fiducial_marker_pose,
		targetObsOrientation *obs)
{

	// float vision_r_theta_unc = 0.f;
	// hrt_abstime vision_timestamp = fiducial_marker_pose.timestamp;

	// /* ORIENTATION */
	// if (!vehicle_local_position_valid) {
	// 	// don't have the data needed for an update
	// 	PX4_INFO("Attitude: %d, Local pos: %d", vehicle_attitude_valid, vehicle_local_position_valid);

	// } else if (!PX4_ISFINITE(vision_r_theta)) {
	// 	PX4_WARN("VISION orientation is corrupt!");

	// } else {

	// 	// TODO: obtain relative orientation using the orientation of the drone.
	// 	// (vision gives orientation between body and target frame. We can thus use the orientation between the body frame and NED to obtain the orientation between NED and target)

	// 	_target_orientation_obs.timestamp = vision_timestamp;
	// 	_target_orientation_obs.updated_theta = true;
	// 	_target_orientation_obs.meas_unc_theta = vision_r_theta_unc;
	// 	_target_orientation_obs.meas_theta = vision_r_theta;
	// 	_target_orientation_obs.meas_h_theta = 1;

	// }

	return false;

}

bool LandingTargetEstimator::processObsVision(const landing_target_pose_s fiducial_marker_pose, targetObsPos *obs)
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
	float meas_uncertainty = _param_ltest_meas_unc.get();

	// For now assume that we are at 10m if no distance bottom is valid
	meas_uncertainty = _dist_bottom_valid ? (meas_uncertainty * _dist_bottom) : (meas_uncertainty * 10);

	SquareMatrix<float, 3> R_rotated = diag(Vector3f(meas_uncertainty, meas_uncertainty, meas_uncertainty));


	/* RELATIVE POSITION*/
	if (!PX4_ISFINITE(vision_r_x) || !PX4_ISFINITE(vision_r_y) || !PX4_ISFINITE(vision_r_y)) {
		PX4_WARN("VISION position is corrupt!");

	} else {

		obs->timestamp = fiducial_marker_pose.timestamp;

		obs->any_xyz_updated = true;
		obs->updated_xyz.setAll(true);

		if (_target_model == TargetModel::FullPoseCoupled) {
			// Process measurements for sequential update by diagonalizing the measurement covariance matrix

			// Sliced state: [r_x,r_y,r_z]; (real state: [pose,vel,bias,acc] or [pose,vel,bias] but assume vision measurements independent of other measurements)
			Matrix<float, 3, 3> H_position =  zeros<float, 3, 3>();
			H_position(0, 0) = 1;
			H_position(1, 1) = 1;
			H_position(2, 2) = 1;

			// Cholesky decomposition using PX4 matrix:
			// TODO: correct
			// Matrix<float, 3, 3> T =  matrix::cholesky(R_rotated);

			Matrix3f T;
			T.identity();


			// Diagonalize R_rotated:
			Matrix<float, 3, 3> R_diag =  T * R_rotated;

			obs->meas_unc_xyz(0) = R_diag(0, 0);
			obs->meas_unc_xyz(1) = R_diag(1, 1);
			obs->meas_unc_xyz(2) = R_diag(2, 2);
			//TODO: replace by obs->meas_unc_xyz = R_diag.diag()

			//Transform measurements Z:
			Vector3f Z(vision_r_x, vision_r_y, vision_r_z);
			Vector3f Z_transformed = T * Z;

			obs->meas_xyz = Z_transformed;

			//Transform H
			Matrix<float, 3, 3> H_transformed = T * H_position;

			//Bring H_position back to the full H:
			//TODO: use slicing functions to insert 3x3 matrix H_transformed into 3x12 matrix meas_h_xyz
			obs->meas_h_xyz(0, 0) = H_transformed(0, 0);
			obs->meas_h_xyz(0, 1) = H_transformed(0, 1);
			obs->meas_h_xyz(0, 2) = H_transformed(0, 2);

			obs->meas_h_xyz(1, 0) = H_transformed(1, 0);
			obs->meas_h_xyz(1, 1) = H_transformed(1, 1);
			obs->meas_h_xyz(1, 2) = H_transformed(1, 2);

			obs->meas_h_xyz(2, 0) = H_transformed(2, 0);
			obs->meas_h_xyz(2, 1) = H_transformed(2, 1);
			obs->meas_h_xyz(2, 2) = H_transformed(2, 2);

		} else {
			// Assume noise correlation negligible:

			// State: [r, r_dot, ...]
			obs->meas_h_xyz(0, 0) = 1;
			obs->meas_h_xyz(1, 0) = 1;
			obs->meas_h_xyz(2, 0) = 1;

			obs->meas_xyz(0) = vision_r_x;
			obs->meas_xyz(1) = vision_r_y;
			obs->meas_xyz(2) = vision_r_z;

			// Assume off diag elements ~ 0
			obs->meas_unc_xyz(0) = R_rotated(0, 0);
			obs->meas_unc_xyz(1) = R_rotated(1, 1);
			obs->meas_unc_xyz(2) = R_rotated(2, 2);
		}

		return true;
	}

	return false;
}


bool LandingTargetEstimator::processObsUavGNSSVel(const landing_target_pose_s target_GNSS_report,
		const sensor_gps_s vehicle_gps_position, targetObsPos *obs)
{

	// TODO: convert .s_variance_m_s from accuracy to variance

	if (_target_mode == TargetMode::Stationary) {

		obs->meas_xyz(0) = -vehicle_gps_position.vel_n_m_s;
		obs->meas_xyz(1) = -vehicle_gps_position.vel_e_m_s;
		obs->meas_xyz(2) = -vehicle_gps_position.vel_d_m_s;

		obs->meas_unc_xyz(0) = vehicle_gps_position.s_variance_m_s;
		obs->meas_unc_xyz(1) = vehicle_gps_position.s_variance_m_s;
		obs->meas_unc_xyz(2) = vehicle_gps_position.s_variance_m_s;

	} else {

		float gps_target_unc = _param_ltest_gps_t_unc.get();

		// If the target is moving, the relative velocity is expressed as the drone verlocity - the target velocity
		obs->meas_xyz(0) = vehicle_gps_position.vel_n_m_s - target_GNSS_report.vx_rel;
		obs->meas_xyz(1) = vehicle_gps_position.vel_e_m_s - target_GNSS_report.vy_rel;
		obs->meas_xyz(2) = vehicle_gps_position.vel_d_m_s - target_GNSS_report.vz_rel;

		float unc = vehicle_gps_position.s_variance_m_s + gps_target_unc;
		obs->meas_unc_xyz(0) = unc;
		obs->meas_unc_xyz(1) = unc;
		obs->meas_unc_xyz(2) = unc;

		// TODO: uncomment once the mavlink message is updated with covariances
		// obs->meas_unc_xyz(0) = vehicle_gps_position.s_variance_m_s + target_GNSS_report.cov_vx_rel;
		// obs->meas_unc_xyz(1) = vehicle_gps_position.s_variance_m_s + target_GNSS_report.cov_vy_rel;
		// obs->meas_unc_xyz(2) = vehicle_gps_position.s_variance_m_s + target_GNSS_report.cov_vx_rel;
	}

	if (_target_model == TargetModel::FullPoseCoupled) {
		// State: [rx, ry, rz, rx_dot, ry_dot, rz_dot, ... ]
		obs->meas_h_xyz(0, 3) = 1; // x direction
		obs->meas_h_xyz(1, 4) = 1; // y direction
		obs->meas_h_xyz(2, 5) = 1; // z direction

	} else {
		// State: [r, r_dot, ...] --> x, y, z directions are the same (decoupled)
		obs->meas_h_xyz(0, 1) = 1;
		obs->meas_h_xyz(1, 1) = 1;
		obs->meas_h_xyz(2, 1) = 1;
	}

	obs->timestamp = vehicle_gps_position.timestamp;

	obs->any_xyz_updated = true;
	obs->updated_xyz.setAll(true);

	// Keep track of the initial relative velocity
	_vel_rel_init.timestamp = vehicle_gps_position.timestamp;
	_vel_rel_init.valid = vehicle_gps_position.vel_ned_valid;
	_vel_rel_init.vel(0) = -vehicle_gps_position.vel_n_m_s;
	_vel_rel_init.vel(1) = -vehicle_gps_position.vel_e_m_s;
	_vel_rel_init.vel(2) = -vehicle_gps_position.vel_d_m_s;

	return true;

}

bool LandingTargetEstimator::processObsTargetGNSS(const landing_target_pose_s target_GNSS_report,
		bool target_GNSS_report_valid,  const sensor_gps_s vehicle_gps_position, targetObsPos *obs)
{

	bool use_gps_measurements = false;

	int target_gps_lat;		// 1e-7 deg
	int target_gps_lon;		// 1e-7 deg
	float target_gps_alt;	// AMSL [mm]

	// TODO: convert .epv and .eph from accuracy to variance
	float gps_target_unc = _param_ltest_gps_t_unc.get();
	float gps_target_eph;
	float gps_target_epv;

	hrt_abstime gps_timestamp = vehicle_gps_position.timestamp;

	// Measurement comes from an actual GPS on the target
	if ((_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) && target_GNSS_report_valid) {

		use_gps_measurements = true;
		gps_timestamp = target_GNSS_report.timestamp;

		// If mission mode && landing position valid && fusion mission position
		if ((_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS)
		    && _nave_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION && _landing_pos.valid) {

			// Weighted average between the landing point and the target GPS position is performed
			// TODO once target_GNSS_report.cov_x_rel is available: weighted average
			target_gps_lat = (int)((target_GNSS_report.lat + _landing_pos.lat) / 2);
			target_gps_lon = (int)((target_GNSS_report.lon + _landing_pos.lon) / 2);
			target_gps_alt = (target_GNSS_report.alt + _landing_pos.alt) / 2.f;

			gps_target_eph = gps_target_unc;
			gps_target_epv = gps_target_unc;

		} else {

			target_gps_lat = target_GNSS_report.lat;
			target_gps_lon = target_GNSS_report.lon;
			target_gps_alt = target_GNSS_report.alt;

			// TODO: complete mavlink message to include uncertainties.
			// gps_target_eph = target_GNSS_report.cov_x_rel;
			// gps_target_epv = target_GNSS_report.cov_z_rel;

			gps_target_eph = gps_target_unc;
			gps_target_epv = gps_target_unc;
		}

	} else if ((_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS)
		   && _nave_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION && _landing_pos.valid) {

		use_gps_measurements = true;

		// Measurement of the landing target position comes from the mission item only
		target_gps_lat = _landing_pos.lat;
		target_gps_lon = _landing_pos.lon;
		target_gps_alt = _landing_pos.alt;

		gps_target_eph = gps_target_unc;
		gps_target_epv = gps_target_unc;
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
		if (_target_model == TargetModel::FullPoseCoupled) {
			// State: [rx, ry, rz, r_x_dot, r_y_dot, r_z_dot, bx, by, bz, ... ]

			// TODO: uncomment bias

			// x direction H = [1, 0, 0, 0, 0, 0, 1, 0, 0, ...]
			obs->meas_h_xyz(0, 0) = 1;
			obs->meas_h_xyz(0, 6) = 1;

			// y direction H = [0, 1, 0, 0, 0, 0, 0, 1, 0, ...]
			obs->meas_h_xyz(1, 1) = 1;
			obs->meas_h_xyz(1, 7) = 1;

			// z direction H = [0, 0, 1, 0, 0, 0, 0, 0, 1, ...]
			obs->meas_h_xyz(2, 2) = 1;
			obs->meas_h_xyz(2, 8) = 1;

		} else {
			// State: [r, r_dot, b, ...] --> same for x,y,z directions (decoupled)
			obs->meas_h_xyz(0, 0) = 1;
			obs->meas_h_xyz(0, 2) = 1;

			obs->meas_h_xyz(1, 0) = 1;
			obs->meas_h_xyz(1, 2) = 1;

			obs->meas_h_xyz(2, 0) = 1;
			obs->meas_h_xyz(2, 2) = 1;
		}

		obs->timestamp = gps_timestamp;

		obs->meas_xyz = gps_relative_pos;

		obs->meas_unc_xyz(0) = gps_unc_horizontal;
		obs->meas_unc_xyz(1) = gps_unc_horizontal;
		obs->meas_unc_xyz(2) = gps_unc_vertical;

		obs->any_xyz_updated = true;
		obs->updated_xyz.setAll(true);

		return true;
	}

	return false;

}

bool LandingTargetEstimator::processObsUWB(const uwb_distance_s uwb_distance, targetObsPos *obs)
{

	if (!PX4_ISFINITE((float)uwb_distance.position[0]) || !PX4_ISFINITE((float)uwb_distance.position[1]) ||
	    !PX4_ISFINITE((float)uwb_distance.position[2])) {
		PX4_WARN("UWB position is corrupt!");

	} else {

		obs->timestamp = uwb_distance.timestamp;

		if (_target_model == TargetModel::FullPoseCoupled) {
			// State: [rx, ry, rz, ... ]
			obs->meas_h_xyz(0, 0) = 1; // x direction
			obs->meas_h_xyz(1, 1) = 1; // y direction
			obs->meas_h_xyz(2, 2) = 1; // z direction

		} else {
			// State: [r, r_dot, ...] --> x, y, z directions are the same (decoupled)
			obs->meas_h_xyz(0, 0) = 1;
			obs->meas_h_xyz(1, 0) = 1;
			obs->meas_h_xyz(2, 0) = 1;
		}

		// The coordinate system is NED (north-east-down) with the position of the landing point relative to the vehicle.
		// the uwb_distance msg contains the Position in NED, Vehicle relative to LP. To change POV we negate every Axis:
		obs->meas_xyz(0) = -uwb_distance.position[0];
		obs->meas_xyz(1) = -uwb_distance.position[1];
		obs->meas_xyz(2) = -uwb_distance.position[2];

		obs->any_xyz_updated = true;
		obs->updated_xyz.setAll(true);

		float dist_z = _dist_bottom - _param_ltest_sens_pos_z.get();

		float measurement_uncertainty = _param_ltest_meas_unc.get() * dist_z * dist_z;

		obs->meas_unc_xyz(0) = measurement_uncertainty;
		obs->meas_unc_xyz(1) = measurement_uncertainty;

		return true;
	}

	return false;
}


bool LandingTargetEstimator::processObsIRlock(const irlock_report_s irlock_report, targetObsPos *obs)
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

			//TODO: For coupled dynamics: Cholesky decomposition to uncorrelate noise (find matrix T that diagonalizes R) z' = T*z; H' = T*H

			// Fill the observations for the irlock sensor
			obs->timestamp = irlock_report.timestamp;

			if (_target_model == TargetModel::FullPoseCoupled) {
				// State: [rx, ry, rz, ... ]
				obs->meas_h_xyz(0, 0) = 1; // x direction
				obs->meas_h_xyz(1, 1) = 1; // y direction
				obs->meas_h_xyz(2, 2) = 1; // z direction

			} else {
				// State: [r, r_dot, ...] --> x, y, z directions are the same (decoupled)
				obs->meas_h_xyz(0, 0) = 1;
				obs->meas_h_xyz(1, 0) = 1;
				obs->meas_h_xyz(2, 0) = 1;
			}

			obs->meas_xyz(0) = rel_pos_x;
			obs->meas_xyz(1) = rel_pos_y;
			obs->meas_xyz(2) = rel_pos_z;

			obs->any_xyz_updated = true;
			obs->updated_xyz.setAll(true);

			float measurement_uncertainty = _param_ltest_meas_unc.get() * dist_z * dist_z;

			obs->meas_unc_xyz(0) = measurement_uncertainty;
			obs->meas_unc_xyz(1) = measurement_uncertainty;
			obs->meas_unc_xyz(2) = measurement_uncertainty;

			return true;
		}
	}

	return false;
}

bool LandingTargetEstimator::fuse_meas(const Vector3f vehicle_acc_ned, const targetObsPos target_pos_obs)
{
	estimator_aid_source_3d_s target_innov;
	Vector<bool, 3> meas_xyz_fused{};
	bool all_directions_fused = false;
	Vector<float, 12> meas_h_row;

	// Number of direction:  x,y,z for all filters excep for the horizontal filter: x,y
	int nb_update_directions = (_target_model == TargetModel::Horizontal) ? 2 : 3;

	// Compute the measurement's time delay (difference between state and measurement time on validity)
	float dt_sync = (_last_predict - target_pos_obs.timestamp);

	// TODO: when using the mission target, the GPS field has no timestamp (maybe only in simulation) once solved: remove && 0
	if (dt_sync > measurement_valid_TIMEOUT_US) {

		PX4_INFO("Obs i = %d rejected because too old. Time sync: %.2f [seconds] > timeout: %.2f [seconds]",
			 target_pos_obs.type,
			 (double)(dt_sync / SEC2USEC), (double)(measurement_valid_TIMEOUT_US / SEC2USEC));

		// No measurement update, set to false
		for (int k = 0; k < 3; k++) {
			target_innov.fusion_enabled[k] = false;
			target_innov.fused[k] = false;
		}

	} else {

		// Convert time sync to seconds
		dt_sync = dt_sync / SEC2USEC;

		// For debug: save the time sync
		target_innov.test_ratio[0] = dt_sync;

		// TODO: Eventually remove, for now to debug, assume prediction time = measurement time
		dt_sync = 0.f;
		// Fill the timestamp field of innovation
		target_innov.timestamp_sample = target_pos_obs.timestamp;
		target_innov.timestamp = hrt_absolute_time(); // TODO: check if correct hrt_absolute_time() or _last_predict

		// Loop over x,y,z directions. Note: even with coupled dynamics we have a sequential update of x,y,z directions separately
		for (int j = 0; j < nb_update_directions; j++) {

			//If the measurement of this filter (x,y or z) has not been updated:
			if (!target_pos_obs.updated_xyz(j)) {

				// No measurement
				// PX4_INFO("Obs i = %d : at least one non-valid observation. x: %d, y: %d, z: %d", target_pos_obs.type,
				// 	 target_pos_obs.updated_xyz(0),
				// 	 target_pos_obs.updated_xyz(1), target_pos_obs.updated_xyz(2));

				// Set innovations to zero
				target_innov.fusion_enabled[j] = false;
				target_innov.fused[j] = false;

			} else {

				//Get the corresponding row of the H matrix.
				meas_h_row = target_pos_obs.meas_h_xyz.row(j);

				// Sync measurement using the prediction model
				if (_target_model == TargetModel::FullPoseCoupled) {

					// Move state back to the measurement time of validity. The state synchronized will be used to compute the innovation.
					_target_estimator_coupled->syncState(dt_sync, vehicle_acc_ned);
					_target_estimator_coupled->setH(meas_h_row);
					// Compute innovations and fill thet target innovation message
					target_innov.innovation_variance[j] = _target_estimator_coupled->computeInnovCov(
							target_pos_obs.meas_unc_xyz(j));
					target_innov.innovation[j] = _target_estimator_coupled->computeInnov(target_pos_obs.meas_xyz(j));
					// Update step
					meas_xyz_fused(j) = _target_estimator_coupled->update();

				} else {
					// Move state back to the measurement time of validity. The state synchronized will be used to compute the innovation.
					_target_estimator[j]->syncState(dt_sync, vehicle_acc_ned(j));
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
			// PX4_INFO("Obs i = %d : at least one direction not fused: . x: %d, y: %d, z: %d", target_pos_obs.type, meas_xyz_fused(0),
			// 	 meas_xyz_fused(1),
			// 	 meas_xyz_fused(2));
		}
	}

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

bool LandingTargetEstimator::updateOrientation()
{
	// Update step for orientation
	bool meas_fused = false;

	if (_target_orientation_obs.updated_theta) {
		_target_estimator_orientation->setH(_target_orientation_obs.meas_h_theta);
		_target_estimator_aid_ev_yaw.innovation_variance = _target_estimator_orientation->computeInnovCov(
					_target_orientation_obs.meas_unc_theta);
		_target_estimator_aid_ev_yaw.innovation = _target_estimator_orientation->computeInnov(
					_target_orientation_obs.meas_theta);
		meas_fused = _target_estimator_orientation->update();

		// Fill the target innovation field
		_target_estimator_aid_ev_yaw.fusion_enabled = true;
		_target_estimator_aid_ev_yaw.innovation_rejected = !meas_fused;
		_target_estimator_aid_ev_yaw.fused = meas_fused;

		_target_estimator_aid_ev_yaw.observation = _target_orientation_obs.meas_theta;
		_target_estimator_aid_ev_yaw.observation_variance = _target_orientation_obs.meas_unc_theta;
		_target_estimator_aid_ev_yaw.timestamp_sample = _target_orientation_obs.timestamp;
		_target_estimator_aid_ev_yaw.timestamp =
			hrt_absolute_time(); // TODO: check if correct hrt_absolute_time() or _last_predict

		// Mark measurement as consumed
		_target_orientation_obs.updated_theta = false;

	} else {
		// No yaw measurement
		_target_estimator_aid_ev_yaw.fusion_enabled = false;
		_target_estimator_aid_ev_yaw.fused = false;
	}

	// 	_target_estimator_aid_ev_yaw_pub.publish(_target_estimator_aid_ev_yaw);

	return meas_fused;
}


void LandingTargetEstimator::publishTarget()
{
	target_estimator_state_s target_estimator_state{};

	//TODO: Update target_pose msg to add orientation: target_pose.rel_orientation_valid -- target_pose.theta_rel -- target_pose.cov_theta_rel
	landing_target_pose_s target_pose{};

	target_pose.timestamp = _last_predict;
	target_estimator_state.timestamp = _last_predict;
	target_pose.is_static = (_target_mode == TargetMode::Stationary);

	bool target_valid = (hrt_absolute_time() - _last_update < landing_target_valid_TIMEOUT_US);

	// TODO: have _last_update_pos ; _last_update_vel ; _last_update_orientation
	target_pose.rel_pos_valid = target_valid;

	// TODO: eventually set to true, but for testing we don't want to use the target as an external source of velocity in the EKF
	target_pose.rel_vel_valid = false;
	// target_pose.rel_orientation_valid = target_valid;

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

		Vector3f bias_vect = _target_estimator_coupled->getVelocityVect();
		target_estimator_state.x_bias = bias_vect(0);
		target_estimator_state.y_bias = bias_vect(1);
		target_estimator_state.z_bias = bias_vect(2);

		Vector3f cov_bias_vect = _target_estimator_coupled->getVelVarVect();
		target_estimator_state.cov_x_bias = cov_bias_vect(0);
		target_estimator_state.cov_y_bias = cov_bias_vect(1);
		target_estimator_state.cov_z_bias = cov_bias_vect(2);

		Vector3f acc_target_vect = _target_estimator_coupled->getAccelerationVect();
		target_estimator_state.ax_target = acc_target_vect(0);
		target_estimator_state.ay_target = acc_target_vect(1);
		target_estimator_state.az_target = acc_target_vect(2);

		Vector3f cov_acc_target_vect = _target_estimator_coupled->getAccVarVect();
		target_estimator_state.cov_ax_target = cov_acc_target_vect(0);
		target_estimator_state.cov_ay_target = cov_acc_target_vect(1);
		target_estimator_state.cov_az_target = cov_acc_target_vect(2);

	} else {
		// This should eventually be removed

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

		target_estimator_state.ax_target = _target_estimator[x]->getAcceleration();
		target_estimator_state.ay_target = _target_estimator[y]->getAcceleration();
		target_estimator_state.az_target = _nb_position_kf > 2 ? _target_estimator[z]->getAcceleration() : 0.f;

		target_estimator_state.cov_ax_target = _target_estimator[x]->getAccVar();
		target_estimator_state.cov_ay_target = _target_estimator[y]->getAccVar();
		target_estimator_state.cov_az_target = _nb_position_kf > 2 ? _target_estimator[z]->getAccVar() : 0.f;
	}

	// TODO: eventually uncomment
	// target_pose.theta_rel = _estimate_orientation ? _target_estimator_orientation->getPosition() : 0.f; ;
	// target_pose.vtheta_abs = _target_estimator_orientation->getVelocity();
	// target_pose.cov_theta_rel =  _target_estimator_orientation->getPosVar();

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
	// float bias_lim = _param_ltest_bias_lim.get();

	// if (_target_model != TargetModel::Horizontal && (fabs(target_estimator_state.x_bias) > bias_lim
	// 		|| fabs(target_estimator_state.y_bias) > bias_lim || fabs(target_estimator_state.z_bias) > bias_lim)) {

	// 	PX4_WARN("Bias exceeds limit: %.2f bias x: %.2f bias y: %.2f bias z: %.2f", (double)bias_lim,
	// 		 (double)target_estimator_state.x_bias, (double)target_estimator_state.y_bias, (double)target_estimator_state.z_bias);
	// 	// _estimator_initialized = false;
	//  // _new_pos_sensor_acquired_time = 0;
	// }

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

	//Update topics
	bool vehicle_local_position_valid = _vehicleLocalPositionSub.update(&vehicle_local_position);
	bool vehicle_attitude_valid = _attitudeSub.update(&vehicle_attitude);
	bool vehicle_acceleration_valid = _vehicle_acceleration_sub.update(&vehicle_acceleration);
	bool pos_sp_triplet_valid = _pos_sp_triplet_sub.update(&pos_sp_triplet);

	// Update nav state
	if (_vehicle_status_sub.update(&vehicle_status)) {
		_nave_state = vehicle_status.nav_state;
	}

	if (!_start_detection && pos_sp_triplet_valid) {
		_start_detection = (pos_sp_triplet.next.type == position_setpoint_s::SETPOINT_TYPE_LAND);

		if (_start_detection) {

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

	Dcmf R_att;

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
		R_att = Dcm<float>(_q_att);

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
	const TargetMode param_target_mode = (TargetMode)_param_ltest_mode.get();
	const TargetModel param_target_model = (TargetModel)_param_ltest_model.get();
	_estimate_orientation = false; // TODO: change to a parameter
	_ltest_aid_mask = _param_ltest_aid_mask.get();

	// TODO: if we don't have observations, do not init the estimator
	if (_ltest_aid_mask == 0) { PX4_ERR("LTE no data fusion enabled. Modify LTEST_AID_MASK and reboot");}

	if (_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) { PX4_INFO("LTE target GPS position data fusion enabled");}

	if (_ltest_aid_mask & SensorFusionMask::USE_UAV_GPS_VEL) { PX4_INFO("LTE drone GPS velocity data fusion enabled");}

	if (_ltest_aid_mask & SensorFusionMask::USE_EXT_VIS_POS) { PX4_INFO("LTE target external vision-based relative position data fusion enabled");}

	if (_ltest_aid_mask & SensorFusionMask::USE_IRLOCK_POS) { PX4_INFO("LTE target relative position from irlock data fusion enabled");}

	if (_ltest_aid_mask & SensorFusionMask::USE_UWB_POS) { PX4_INFO("LTE target relative position from uwb data fusion enabled");}

	if (_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS) { PX4_INFO("LTE PX4 mission landing position fusion enabled");}

	if (_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS && _ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) { PX4_INFO("LTE a weighted average between the landing point and the target GPS position will be performed");}

	// TODO: add orientation
	if (_target_mode != param_target_mode || _target_model != param_target_model) {

		// Define the target mode and model
		_target_mode = param_target_mode;
		_target_model = param_target_model;
		selectTargetEstimator();

		// Define LTEST timeout
		_ltest_TIMEOUT_US = (uint32_t)(_param_ltest_btout.get() * 1e6f);
	}

	switch (_target_model) {
	case TargetModel::FullPoseDecoupled:
		_nb_position_kf = 3;

		if ((_target_estimator[x] == nullptr) || (_target_estimator[y] == nullptr) || (_target_estimator[z] == nullptr)
		    || (_target_estimator_orientation == nullptr)) {
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

	// TODO: check if this should be above in the function.
	ModuleParams::updateParams();
}

void LandingTargetEstimator::selectTargetEstimator()
{
	TargetEstimator *tmp_x = nullptr;
	TargetEstimator *tmp_y = nullptr;
	TargetEstimator *tmp_z = nullptr;
	TargetEstimator *tmp_theta = nullptr;
	TargetEstimatorCoupled *tmp_xyz = nullptr;

	bool init_failed = true;

	switch (_target_model) {
	case TargetModel::FullPoseDecoupled:

		if (_target_mode == TargetMode::Moving) {
			tmp_x = new KFxyzDecoupledMoving;
			tmp_y = new KFxyzDecoupledMoving;
			tmp_z = new KFxyzDecoupledMoving;
			PX4_INFO("LTE estimator: Moving target, full pose with x,y,z decoupled.");

		} else {
			tmp_x = new KFxyzDecoupledStatic;
			tmp_y = new KFxyzDecoupledStatic;
			tmp_z = new KFxyzDecoupledStatic;
			PX4_INFO("LTE estimator: Static target, full pose with x,y,z decoupled.");
		}

		init_failed = (tmp_x == nullptr) || (tmp_y == nullptr) || (tmp_z == nullptr);

		break;

	case TargetModel::FullPoseCoupled:

		if (_target_mode == TargetMode::Moving) {
			tmp_xyz = new KFxyzCoupledMoving;
			PX4_INFO("LTE estimator: Moving target, full pose with x,y,z coupled in one filter.");

		} else {
			tmp_xyz = new KFxyzCoupledStatic;
			PX4_INFO("LTE estimator: Static target, full pose with x,y,z coupled in one filter.");
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

	if (_estimate_orientation) {
		// TODO: complete

		if (_target_mode == TargetMode::Moving) {
			// tmp_theta = new KFOrientationMoving;
		} else {
			// tmp_theta = new KFOrientationStationary;
		}

		// init_failed = (tmp_theta == nullptr);

	}


	if (init_failed) {
		PX4_ERR("LTE init failed");
		_param_ltest_mode.set(0); // TODO: why?

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

		if (_estimate_orientation) {
			delete _target_estimator_orientation;
			_target_estimator_orientation = tmp_theta;
		}
	}
}

} // namespace landing_target_estimator
