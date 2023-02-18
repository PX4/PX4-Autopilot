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
 * @file LTEstPosition.cpp
 *
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

#include "LTEstPosition.h"

#define SEC2USEC 1000000.0f

namespace landing_target_estimator
{

using namespace matrix;

LTEstPosition::LTEstPosition() :
	ModuleParams(nullptr)
{
	_targetPosePub.advertise();
	_targetEstimatorStatePub.advertise();
	_ltest_aid_gps_pos_target_pub.advertise();
	_ltest_aid_gps_pos_mission_pub.advertise();
	_ltest_aid_gps_vel_rel_pub.advertise();
	_ltest_aid_gps_vel_target_pub.advertise();
	_ltest_aid_fiducial_marker_pub.advertise();
	_ltest_aid_irlock_pub.advertise();
	_ltest_aid_uwb_pub.advertise();

	_check_params(true);
}

LTEstPosition::~LTEstPosition()
{
	for (int i = 0; i < 3; i++) {
		delete _target_estimator[i];
	}

	delete _target_estimator_coupled;

	perf_free(_ltest_predict_perf);
	perf_free(_ltest_update_perf);
}

bool LTEstPosition::init()
{
	bool return_bool = false;

	_target_mode = (TargetMode)_param_ltest_mode.get();
	_target_model = (TargetModel)_param_ltest_model.get();
	_ltest_aid_mask = _param_ltest_aid_mask.get();
	_ltest_TIMEOUT_US = (uint32_t)(_param_ltest_btout.get() * SEC2USEC);

	if (selectTargetEstimator()) {

		if (_ltest_aid_mask == 0) {
			PX4_ERR("LTEst: no data fusion enabled. Modify LTEST_AID_MASK and reboot");
			return false;
		}

		if (_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) { PX4_INFO("LTEst target GPS position data fusion enabled");}

		if (_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS) { PX4_INFO("LTEst PX4 mission landing position fusion enabled");}

		if (_ltest_aid_mask & SensorFusionMask::USE_GPS_REL_VEL) { PX4_INFO("LTEst relative GPS velocity data fusion enabled");}

		if (_ltest_aid_mask & SensorFusionMask::USE_EXT_VIS_POS) { PX4_INFO("LTEst target external vision-based relative position data fusion enabled");}

		if (_ltest_aid_mask & SensorFusionMask::USE_IRLOCK_POS) { PX4_INFO("LTEst target relative position from irlock data fusion enabled");}

		if (_ltest_aid_mask & SensorFusionMask::USE_UWB_POS) { PX4_INFO("LTEst target relative position from uwb data fusion enabled");}

		return_bool = true;
	}

	return return_bool;
}

void LTEstPosition::resetFilter()
{
	_estimator_initialized = false;
	_new_pos_sensor_acquired_time = 0;
	_bias_set = false;
	_landing_pos.valid = false;
}

void LTEstPosition::update(const Vector3f &acc_ned)
{
	_check_params(false);

	// predict the target state using a constant relative acceleration model
	if (_estimator_initialized) {

		if (hrt_absolute_time() - _last_update > _ltest_TIMEOUT_US) {
			PX4_WARN("LTE estimator timeout");
			resetFilter();

		} else {
			perf_begin(_ltest_predict_perf);
			predictionStep(acc_ned);
			perf_end(_ltest_predict_perf);

			_last_predict = hrt_absolute_time();
		}
	}

	// Update and fuse the observations and pulishes innovations
	if (update_step(acc_ned)) {
		_last_update = _last_predict;
	}

	if (_estimator_initialized) {publishTarget();}
}

bool LTEstPosition::initEstimator(Vector3f pos_init, Vector3f vel_init, Vector3f target_acc_init,
				  Vector3f bias_init, Vector3f target_vel_init)
{

	PX4_INFO("Pos init %.2f %.2f %.2f", (double)pos_init(0), (double)pos_init(1), (double)pos_init(2));
	PX4_INFO("Vel init %.2f %.2f %.2f", (double)vel_init(0), (double)vel_init(1), (double)vel_init(2));
	PX4_INFO("Target acc init %.2f %.2f %.2f", (double)target_acc_init(0), (double)target_acc_init(1),
		 (double)target_acc_init(2));
	PX4_INFO("Target vel init %.2f %.2f %.2f", (double)target_vel_init(0), (double)target_vel_init(1),
		 (double)target_vel_init(2));
	PX4_INFO("Bias init %.2f %.2f %.2f", (double)bias_init(0), (double)bias_init(1), (double)bias_init(2));

	const float state_pos_var = _param_ltest_pos_unc_in.get();
	const float state_vel_var = _param_ltest_vel_unc_in.get();
	const float state_bias_var = _param_ltest_bias_unc_in.get();
	const float state_acc_var = _param_ltest_acc_unc_in.get();
	const float state_target_vel_var = _param_ltest_vel_unc_in.get();

	const Vector3f state_pos_var_vect(state_pos_var, state_pos_var, state_pos_var);
	const Vector3f state_vel_var_vect(state_vel_var, state_vel_var, state_vel_var);
	const Vector3f state_bias_var_vect(state_bias_var, state_bias_var, state_bias_var);
	const Vector3f state_acc_var_vect(state_acc_var, state_acc_var, state_acc_var);
	const Vector3f state_target_vel_var_vect(state_target_vel_var, state_target_vel_var, state_target_vel_var);

	Vector3f state_target_vel;

	if (_target_model == TargetModel::Coupled) {

		/* Set filter initiaé state */
		_target_estimator_coupled->setPosition(pos_init);
		_target_estimator_coupled->setVelocity(vel_init);
		_target_estimator_coupled->setTargetAcc(target_acc_init);
		_target_estimator_coupled->setBias(bias_init);
		_target_estimator_coupled->setTargetVel(state_target_vel);

		/* Set initial state variance */
		_target_estimator_coupled->setStatePosVar(state_pos_var_vect);
		_target_estimator_coupled->setStateVelVar(state_vel_var_vect);
		_target_estimator_coupled->setStateAccVar(state_acc_var_vect);
		_target_estimator_coupled->setStateBiasVar(state_bias_var_vect);
		_target_estimator_coupled->setStateTargetVelVar(state_target_vel_var_vect);

	} else {

		for (int i = 0; i < 3; i++) {

			/* Set filter initiaé state */
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
	}

	return true;
}


void LTEstPosition::predictionStep(Vector3f vehicle_acc_ned)
{
	// predict target position with the help of accel data

	// Time from last prediciton
	const float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC;

	// The rotated input cov (from body to NED R*cov*R^T) is the same as the original input cov since input_cov = acc_unc * Identiy and R*R^T = Identity
	const SquareMatrix<float, 3> input_cov = diag(Vector3f(_drone_acc_unc, _drone_acc_unc, _drone_acc_unc));
	const SquareMatrix<float, 3> target_acc_cov = diag(Vector3f(_target_acc_unc, _target_acc_unc, _target_acc_unc));
	const SquareMatrix<float, 3> bias_cov = diag(Vector3f(_bias_unc, _bias_unc, _bias_unc));

	if (_target_model == TargetModel::Coupled) {

		if (_target_mode == TargetMode::Moving || _target_mode == TargetMode::MovingAugmented) {_target_estimator_coupled->setTargetAccVar(target_acc_cov);}

		_target_estimator_coupled->setBiasVar(bias_cov);
		_target_estimator_coupled->setInputAccVar(input_cov);

		_target_estimator_coupled->predictState(dt, vehicle_acc_ned);
		_target_estimator_coupled->predictCov(dt);

	} else {
		for (int i = 0; i < 3; i++) {
			//For decoupled dynamics, we neglect the off diag elements.
			if (_target_mode == TargetMode::Moving || _target_mode == TargetMode::MovingAugmented) {_target_estimator[i]->setTargetAccVar(target_acc_cov(i, i));}

			_target_estimator[i]->setBiasVar(bias_cov(i, i));
			_target_estimator[i]->setInputAccVar(input_cov(i, i));

			_target_estimator[i]->predictState(dt, vehicle_acc_ned(i));
			_target_estimator[i]->predictCov(dt);
		}
	}
}



bool LTEstPosition::update_step(Vector3f vehicle_acc_ned)
{

	sensor_gps_s vehicle_gps_position;
	landing_target_gnss_s target_GNSS_report;
	fiducial_marker_pos_report_s fiducial_marker_pose;
	irlock_report_s irlock_report;
	uwb_distance_s	uwb_distance;

	targetObsPos obs_gps_pos_target;
	targetObsPos obs_gps_pos_mission;
	targetObsPos obs_gps_vel_rel;
	targetObsPos obs_gps_vel_target;
	targetObsPos obs_fiducial_marker;
	targetObsPos obs_irlock;
	targetObsPos obs_uwb;

	bool pos_target_GPS_valid = false;
	bool pos_mission_GPS_valid = false;
	bool vel_target_GPS_valid = false;
	bool vel_rel_GPS_valid = false;
	bool fiducial_marker_valid = false;
	bool irlock_valid = false;
	bool uwb_valid = false;

	// Process data from all topics

	/* IRLOCK */
	if ((_ltest_aid_mask & SensorFusionMask::USE_IRLOCK_POS) && _range_sensor.valid
	    && _irlockReportSub.update(&irlock_report)) {

		obs_irlock.type = irlock;

		if (processObsIRlock(irlock_report, obs_irlock)) {
			irlock_valid = ((hrt_absolute_time() - obs_irlock.timestamp) < measurement_valid_TIMEOUT_US);
		}
	}

	/* UWB */
	if ((_ltest_aid_mask & SensorFusionMask::USE_UWB_POS) && _range_sensor.valid && _uwbDistanceSub.update(&uwb_distance)) {

		obs_uwb.type = uwb;

		if (processObsUWB(uwb_distance, obs_uwb)) {
			uwb_valid = ((hrt_absolute_time() - obs_uwb.timestamp) < measurement_valid_TIMEOUT_US);
		}
	}

	/* VISION */
	if ((_ltest_aid_mask & SensorFusionMask::USE_EXT_VIS_POS)
	    && _fiducial_marker_report_sub.update(&fiducial_marker_pose)) {

		obs_fiducial_marker.type = fiducial_marker;

		if (processObsVision(fiducial_marker_pose, obs_fiducial_marker)) {
			fiducial_marker_valid = ((hrt_absolute_time() - obs_fiducial_marker.timestamp) < measurement_valid_TIMEOUT_US);
		}
	}

	/* GPS BASED OBSERVATIONS */
	bool vehicle_gps_position_updated = _vehicle_gps_position_sub.update(&vehicle_gps_position);

	if ((hrt_absolute_time() - vehicle_gps_position.timestamp < measurement_updated_TIMEOUT_US)) {

		bool target_GPS_updated = _landing_target_gnss_sub.update(&target_GNSS_report);

		/* TARGET GPS */
		if ((_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) && target_GPS_updated
		    && target_GNSS_report.abs_pos_updated) {

			obs_gps_pos_target.type = target_gps_pos;

			if (processObsGNSSPosTarget(target_GNSS_report, vehicle_gps_position, obs_gps_pos_target)) {
				pos_target_GPS_valid	= ((hrt_absolute_time() - obs_gps_pos_target.timestamp) < measurement_valid_TIMEOUT_US);
			}
		}

		/* MISSION GPS POSE */
		if ((_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS) && vehicle_gps_position_updated && _landing_pos.valid) {

			obs_gps_pos_mission.type = mission_gps_pos;

			if (processObsGNSSPosMission(vehicle_gps_position, obs_gps_pos_mission)) {
				pos_mission_GPS_valid	= ((hrt_absolute_time() - obs_gps_pos_mission.timestamp) < measurement_valid_TIMEOUT_US);
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

		if ((_ltest_aid_mask & SensorFusionMask::USE_GPS_REL_VEL)) {

			/* TARGET GPS VELOCITY */
			if (_target_mode == TargetMode::MovingAugmented && target_GPS_updated && _target_gps_vel.valid) {

				obs_gps_vel_target.type = vel_target_gps;

				if (processObsGNSSVelTarget(target_GNSS_report, obs_gps_vel_target)) {
					vel_target_GPS_valid	= ((hrt_absolute_time() - obs_gps_vel_target.timestamp) < measurement_valid_TIMEOUT_US);
				}
			}

			/* RELATIVE GPS velocity */
			if (_uav_gps_vel.valid && ((_target_gps_vel.valid && target_GPS_updated && _target_mode == TargetMode::Moving)
						   || (vehicle_gps_position_updated
						       && _target_mode != TargetMode::Moving))) {

				obs_gps_vel_rel.type = vel_rel_gps;

				if (processObsGNSSVelRel(target_GNSS_report, target_GPS_updated, vehicle_gps_position, vehicle_gps_position_updated,
							 obs_gps_vel_rel)) {
					vel_rel_GPS_valid = ((hrt_absolute_time() - obs_gps_vel_rel.timestamp) < measurement_valid_TIMEOUT_US);
				}
			}
		}
	}

	// If one pos measurement was updated, return true
	const bool new_pos_sensor = pos_mission_GPS_valid || pos_target_GPS_valid || fiducial_marker_valid || irlock_valid
				    || uwb_valid;
	const bool new_non_gnss_pos_sensor = fiducial_marker_valid || irlock_valid || uwb_valid;
	const bool new_vel_sensor = vel_rel_GPS_valid || vel_target_GPS_valid;

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

		/*TARGET GPS*/
		if (pos_target_GPS_valid) {

			if (fuse_meas(vehicle_acc_ned, obs_gps_pos_target)) {
				pos_fused = true;
			}
		}

		/*MISSION POS GPS*/
		if (pos_mission_GPS_valid) {

			if (fuse_meas(vehicle_acc_ned, obs_gps_pos_mission)) {
				pos_fused = true;
			}
		}

		/*GPS RELATIVE VELOCITY*/
		if (vel_rel_GPS_valid) {
			fuse_meas(vehicle_acc_ned, obs_gps_vel_rel);
		}

		/*TARGET GPS VELOCITY*/
		if (vel_target_GPS_valid) {
			fuse_meas(vehicle_acc_ned, obs_gps_vel_target);
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

		// If at least one pos measurement was fused, consider the filter updated
		return pos_fused;

	} else if (new_pos_sensor && !_estimator_initialized) {

		// Wait 1 second before initilazing the estimator to have an initial velocity estimate.
		if (!_new_pos_sensor_acquired_time) {
			_new_pos_sensor_acquired_time = hrt_absolute_time();

		} else if ((hrt_absolute_time() - _new_pos_sensor_acquired_time) > 1000000) {

			Vector3f pos_init;
			Vector3f vel_init;
			Vector3f target_acc_init;	// Assume null target absolute acceleration
			Vector3f bias_init;
			Vector3f target_vel_init;

			// Define the initial relative position of target w.r.t. the drone in NED frame using the available measurement
			if (fiducial_marker_valid) {
				pos_init = obs_fiducial_marker.meas_xyz;

			} else if (irlock_valid) {
				pos_init = obs_irlock.meas_xyz;

			} else if (uwb_valid) {
				pos_init = obs_uwb.meas_xyz;

			} else if (pos_target_GPS_valid) {
				pos_init = obs_gps_pos_target.meas_xyz;

			} else if (pos_mission_GPS_valid) {
				pos_init = obs_gps_pos_mission.meas_xyz;
			}

			// Compute the initial bias as the difference between the GPS and external position estimate.
			if (((hrt_absolute_time() - _pos_rel_gnss.timestamp) < measurement_valid_TIMEOUT_US) && new_non_gnss_pos_sensor) {
				// We assume that gnss observations have a bias but other position obs don't. It follows: gnss_obs = state + bias <--> bias = gnss_obs - state
				bias_init =  _pos_rel_gnss.xyz - pos_init;
				_bias_set = true;
			}

			if (_target_gps_vel.valid && ((hrt_absolute_time() - _target_gps_vel.timestamp) < measurement_valid_TIMEOUT_US)) {
				target_vel_init = _target_gps_vel.xyz;
			}

			// Define initial relative velocity of the target w.r.t. to the drone in NED frame
			if (_uav_gps_vel.valid && ((hrt_absolute_time() - _uav_gps_vel.timestamp) < measurement_valid_TIMEOUT_US)) {

				if (_target_mode == TargetMode::Stationary) {
					vel_init = -_uav_gps_vel.xyz;

				} else if ((_target_mode == TargetMode::Moving)) {
					vel_init = target_vel_init - _uav_gps_vel.xyz;

				} else if (_target_mode == TargetMode::MovingAugmented) {
					vel_init = _uav_gps_vel.xyz;
				}
			}

			if (initEstimator(pos_init, vel_init, target_acc_init, bias_init, target_vel_init)) {
				PX4_INFO("LTE Position Estimator properly initialized.");
				_estimator_initialized = true;
				_uav_gps_vel.valid = false;
				_target_gps_vel.valid = false;
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
bool LTEstPosition::processObsVision(const fiducial_marker_pos_report_s &fiducial_marker_pose, targetObsPos &obs)
{
	/* Rotate vision observation from body FRD - to vc-NED */
	const matrix::Quaternionf quat_att(fiducial_marker_pose.q);
	const Vector3f vision_body(fiducial_marker_pose.x_rel_body, fiducial_marker_pose.y_rel_body,
				   fiducial_marker_pose.z_rel_body);
	const Vector3f vision_ned = quat_att.rotateVector(vision_body);

	const SquareMatrix<float, 3> covMat = diag(Vector3f(fiducial_marker_pose.cov_x_rel_body,
					      fiducial_marker_pose.cov_y_rel_body,
					      fiducial_marker_pose.cov_z_rel_body));
	const matrix::Dcmf R_att = matrix::Dcm<float>(quat_att);

	// Rotate covariance matrix to vc-NED
	SquareMatrix<float, 3> Cov_rotated = R_att * covMat * R_att.transpose();

	// If the variance was not set, use default
	if (fiducial_marker_pose.cov_x_rel_body < (float)1e-6 && fiducial_marker_pose.cov_y_rel_body < (float)1e-6
	    && fiducial_marker_pose.cov_z_rel_body < (float)1e-6) {
		// Uncertainty proportional to the vertical distance
		const float meas_uncertainty = _range_sensor.valid ? (_meas_unc * _range_sensor.dist_bottom) : (_meas_unc * 10);
		Cov_rotated = diag(Vector3f(meas_uncertainty, meas_uncertainty, meas_uncertainty));
	}

	/* RELATIVE POSITION*/
	if (!PX4_ISFINITE(vision_ned(0)) || !PX4_ISFINITE(vision_ned(1)) || !PX4_ISFINITE(vision_ned(2))) {
		PX4_WARN("VISION position is corrupt!");

	} else {

		obs.timestamp = fiducial_marker_pose.timestamp;

		obs.updated_xyz.setAll(true);

		if (_target_model == TargetModel::Coupled) {

			// Process measurements for sequential update by diagonalizing the measurement covariance matrix

			// Sliced state: [r_x,r_y,r_z]; (real state: [pose,vel,bias,acc] or [pose,vel,bias] but assume vision measurements independent of other measurements)
			Matrix<float, 3, 3> H_position;
			H_position(0, 0) = 1;
			H_position(1, 1) = 1;
			H_position(2, 2) = 1;

			// Cholesky decomposition R = L*D*L.T() to find L_inv = inv(L) such that R_diag = L_inv*R*L_inv.T() = D is diagonal:
			Matrix<float, 3, 3> L_inv = matrix::inv(matrix::choleskyLDLT(Cov_rotated));

			// Diagonalize Cov_rotated:
			Matrix<float, 3, 3> R_diag =  L_inv * Cov_rotated * L_inv.T();

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
				R_diag = Cov_rotated;
			}

			obs.meas_unc_xyz(0) = R_diag(0, 0);
			obs.meas_unc_xyz(1) = R_diag(1, 1);
			obs.meas_unc_xyz(2) = R_diag(2, 2);

			//Transform measurements:
			const Vector3f Z_transformed = L_inv * vision_ned;

			obs.meas_xyz = Z_transformed;

			//Transform H
			const Matrix<float, 3, 3> H_transformed = L_inv * H_position;

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

			obs.meas_xyz = vision_ned;

			// Assume off diag elements ~ 0
			obs.meas_unc_xyz(0) = Cov_rotated(0, 0);
			obs.meas_unc_xyz(1) = Cov_rotated(1, 1);
			obs.meas_unc_xyz(2) = Cov_rotated(2, 2);
		}

		return true;
	}

	return false;
}

/*Drone GNSS velocity observation: [r_dotx, r_doty, r_dotz]*/
bool LTEstPosition::processObsGNSSVelRel(const landing_target_gnss_s &target_GNSS_report, bool target_GPS_updated,
		const sensor_gps_s &vehicle_gps_position, bool vehicle_gps_vel_updated, targetObsPos &obs)
{

	bool obs_updated = false;

	switch (_target_mode) {
	case TargetMode::Stationary:

		if (vehicle_gps_vel_updated) {
			obs.meas_xyz(0) = -vehicle_gps_position.vel_n_m_s;
			obs.meas_xyz(1) = -vehicle_gps_position.vel_e_m_s;
			obs.meas_xyz(2) = -vehicle_gps_position.vel_d_m_s;

			obs.meas_unc_xyz(0) = vehicle_gps_position.s_variance_m_s;
			obs.meas_unc_xyz(1) = vehicle_gps_position.s_variance_m_s;
			obs.meas_unc_xyz(2) = vehicle_gps_position.s_variance_m_s;

			obs_updated = true;
		}

		break;

	case TargetMode::Moving:

		if (target_GPS_updated) {
			const float dt_sync_us = fabs(vehicle_gps_position.timestamp - target_GNSS_report.timestamp);

			// Make sure measurement makes sense
			if (!PX4_ISFINITE(target_GNSS_report.vel_n_m_s) || !PX4_ISFINITE(target_GNSS_report.vel_e_m_s)
			    || !PX4_ISFINITE(target_GNSS_report.vel_d_m_s)) {
				PX4_WARN("Target GPS velocity is corrupt!");

			} else if (dt_sync_us > measurement_valid_TIMEOUT_US) {
				PX4_INFO("Target GPS velocity rejected because too old. Time sync: %.2f [ms] > timeout: %.2f [ms]",
					 (double)(dt_sync_us / 1000), (double)(measurement_valid_TIMEOUT_US / 1000));

			} else {

				// If the target is moving, the relative velocity is expressed as the drone verlocity - the target velocity
				obs.meas_xyz(0) = vehicle_gps_position.vel_n_m_s - target_GNSS_report.vel_n_m_s;
				obs.meas_xyz(1) = vehicle_gps_position.vel_e_m_s - target_GNSS_report.vel_e_m_s;
				obs.meas_xyz(2) = vehicle_gps_position.vel_d_m_s - target_GNSS_report.vel_d_m_s;

				const float unc = vehicle_gps_position.s_variance_m_s * vehicle_gps_position.s_variance_m_s +
						  target_GNSS_report.s_variance_m_s * target_GNSS_report.s_variance_m_s;

				obs.meas_unc_xyz(0) = unc;
				obs.meas_unc_xyz(1) = unc;
				obs.meas_unc_xyz(2) = unc;

				obs_updated = true;
			}
		}

		break;

	case TargetMode::MovingAugmented:

		if (vehicle_gps_vel_updated) {
			obs.meas_xyz(0) = vehicle_gps_position.vel_n_m_s;
			obs.meas_xyz(1) = vehicle_gps_position.vel_e_m_s;
			obs.meas_xyz(2) = vehicle_gps_position.vel_d_m_s;

			obs.meas_unc_xyz(0) = vehicle_gps_position.s_variance_m_s;
			obs.meas_unc_xyz(1) = vehicle_gps_position.s_variance_m_s;
			obs.meas_unc_xyz(2) = vehicle_gps_position.s_variance_m_s;

			obs_updated = true;
		}

		break;

	case TargetMode::NotInit:
		break;
	}

	if (obs_updated) {

		// h_meas = [rx, ry, rz, r_dotx, r_doty, r_dotz, bx, by, bz, atx, aty, atz]
		// Obs: [r_dotx, r_doty, r_dotz]

		obs.meas_h_xyz(0, 3) = 1; // x direction
		obs.meas_h_xyz(1, 4) = 1; // y direction
		obs.meas_h_xyz(2, 5) = 1; // z direction

		obs.timestamp = vehicle_gps_position.timestamp;

		obs.updated_xyz.setAll(true);

		return true;
	}

	return false;

}

/*Target GNSS velocity observation: [r_dotx, r_doty, r_dotz]*/
bool LTEstPosition::processObsGNSSVelTarget(const landing_target_gnss_s &target_GNSS_report, targetObsPos &obs)
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

		const float unc = target_GNSS_report.s_variance_m_s * target_GNSS_report.s_variance_m_s;

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
bool LTEstPosition::processObsGNSSPosMission(const sensor_gps_s &vehicle_gps_position, targetObsPos &obs)
{

	if (vehicle_gps_position.lat == 0 || vehicle_gps_position.lon == 0 || !PX4_ISFINITE((float)vehicle_gps_position.alt)) {
		PX4_WARN("vehicle GPS position is corrupt!");

	} else {
		// Obtain GPS relative measurements in NED as target_global - uav_gps_global followed by global2local transformation
		Vector3f gps_relative_pos;
		get_vector_to_next_waypoint((vehicle_gps_position.lat / 1.0e7), (vehicle_gps_position.lon / 1.0e7),
					    (_landing_pos.lat / 1.0e7), (_landing_pos.lon / 1.0e7),
					    &gps_relative_pos(0), &gps_relative_pos(1));

		// Down direction (if the drone is above the target, the relative position is positive)
		gps_relative_pos(2) = (vehicle_gps_position.alt - _landing_pos.alt) / 1000.f; // transform mm to m

		const float gps_unc_horizontal = vehicle_gps_position.eph * vehicle_gps_position.eph;
		const float gps_unc_vertical = vehicle_gps_position.epv * vehicle_gps_position.epv;

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

		// Keep track of the gps relative position if not already done using the target GPS
		if ((hrt_absolute_time() - _pos_rel_gnss.timestamp) > measurement_valid_TIMEOUT_US) {
			_pos_rel_gnss.timestamp = obs.timestamp;
			_pos_rel_gnss.valid = (PX4_ISFINITE(gps_relative_pos(0)) && PX4_ISFINITE(gps_relative_pos(1))
					       && PX4_ISFINITE(gps_relative_pos(2)));
			_pos_rel_gnss.xyz = gps_relative_pos;
		}

		return true;
	}

	return false;
}

/*Target GNSS observation: [rx + bx, ry + by, rz + bz]*/
bool LTEstPosition::processObsGNSSPosTarget(const landing_target_gnss_s &target_GNSS_report,
		const sensor_gps_s &vehicle_gps_position, targetObsPos &obs)
{

	const float dt_sync_us = fabs(vehicle_gps_position.timestamp - target_GNSS_report.timestamp);

	const int target_gps_lat = target_GNSS_report.lat; // 1e-7 [deg]
	const int target_gps_lon = target_GNSS_report.lon; // 1e-7 [deg]
	const float target_gps_alt = target_GNSS_report.alt; // AMSL [mm]

	if (vehicle_gps_position.lat == 0 || vehicle_gps_position.lon == 0 || !PX4_ISFINITE((float)vehicle_gps_position.alt)) {
		PX4_WARN("vehicle GPS position is corrupt!");

	} else if (target_gps_lat == 0 || target_gps_lon == 0 || !PX4_ISFINITE(target_gps_alt)) {
		PX4_WARN("Target GPS position is corrupt!");
		return false;

	} else if (dt_sync_us > measurement_valid_TIMEOUT_US) {
		PX4_INFO("Target GPS position rejected because too old. Time sync: %.2f [ms] > timeout: %.2f [ms]",
			 (double)(dt_sync_us / 1000), (double)(measurement_valid_TIMEOUT_US / 1000));
		return false;

	} else {

		const float gps_target_eph = target_GNSS_report.eph;
		const float gps_target_epv = target_GNSS_report.epv;

		// Obtain GPS relative measurements in NED as target_global - uav_gps_global followed by global2local transformation
		Vector3f gps_relative_pos;
		get_vector_to_next_waypoint((vehicle_gps_position.lat / 1.0e7), (vehicle_gps_position.lon / 1.0e7),
					    (target_gps_lat / 1.0e7), (target_gps_lon / 1.0e7),
					    &gps_relative_pos(0), &gps_relative_pos(1));

		// Down direction (if the drone is above the target, the relative position is positive)
		gps_relative_pos(2) = (vehicle_gps_position.alt - target_gps_alt) / 1000.f; // transform mm to m

		// Var(aX - bY) = a^2 Var(X) + b^2Var(Y) - 2ab Cov(X,Y)
		const float gps_unc_horizontal = vehicle_gps_position.eph * vehicle_gps_position.eph + gps_target_eph * gps_target_eph;
		const float gps_unc_vertical = vehicle_gps_position.epv * vehicle_gps_position.epv + gps_target_epv * gps_target_epv;

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

	return false;
}

/*UWB observation: [rx, ry, rz]*/
bool LTEstPosition::processObsUWB(const uwb_distance_s &uwb_distance, targetObsPos &obs)
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

		const float dist_z = _range_sensor.dist_bottom - _param_ltest_sens_pos_z.get();

		const float measurement_uncertainty = _meas_unc * dist_z * dist_z;

		obs.meas_unc_xyz(0) = measurement_uncertainty;
		obs.meas_unc_xyz(1) = measurement_uncertainty;
		obs.meas_unc_xyz(2) = measurement_uncertainty;

		return true;
	}

	return false;
}


bool LTEstPosition::processObsIRlock(const irlock_report_s &irlock_report, targetObsPos &obs)
{
	if (!PX4_ISFINITE(irlock_report.pos_y) || !PX4_ISFINITE(irlock_report.pos_x)) {
		PX4_WARN("IRLOCK position is corrupt!");

	} else {

		matrix::Vector3f sensor_ray; // ray pointing towards target in body frame
		sensor_ray(0) = irlock_report.pos_x * _param_ltest_scale_x.get(); // forward
		sensor_ray(1) = irlock_report.pos_y * _param_ltest_scale_y.get(); // right
		sensor_ray(2) = 1.0f;

		// rotate unit ray according to sensor orientation
		matrix::Dcmf S_att; //Orientation of the sensor relative to body frame
		S_att = get_rot_matrix(static_cast<enum Rotation>(_param_ltest_sens_rot.get()));
		sensor_ray = S_att * sensor_ray;

		// Adjust relative position according to sensor offset
		sensor_ray(0) += _param_ltest_sens_pos_x.get();
		sensor_ray(1) += _param_ltest_sens_pos_y.get();

		// Rotate the unit ray into the navigation frame.
		const matrix::Quaternionf quat_att(irlock_report.q);
		sensor_ray = quat_att.rotateVector(sensor_ray);

		// z component of measurement safe, use this measurement
		if (fabsf(sensor_ray(2)) > 1e-6f) {

			const float dist_z = _range_sensor.dist_bottom - _param_ltest_sens_pos_z.get();

			// scale the ray s.t. the z component has length of _uncertainty_scale
			const float rel_pos_x = sensor_ray(0) / sensor_ray(2) * dist_z;
			const float rel_pos_y = sensor_ray(1) / sensor_ray(2) * dist_z;
			const float rel_pos_z = dist_z;

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

			const float measurement_uncertainty = _meas_unc * dist_z * dist_z;

			obs.meas_unc_xyz(0) = measurement_uncertainty;
			obs.meas_unc_xyz(1) = measurement_uncertainty;
			obs.meas_unc_xyz(2) = measurement_uncertainty;

			return true;
		}
	}

	return false;
}

bool LTEstPosition::fuse_meas(const Vector3f vehicle_acc_ned, const targetObsPos &target_pos_obs)
{
	perf_begin(_ltest_update_perf);

	estimator_aid_source_3d_s target_innov;
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
		for (int k = 0; k < 3; k++) {
			target_innov.fusion_enabled[k] = false;
			target_innov.fused[k] = false;
		}

	} else {

		// For debug: log the time sync
		target_innov.time_last_fuse[0] = (int)(dt_sync_us / 1000);

		// Convert time sync to seconds
		const float dt_sync_s = dt_sync_us / SEC2USEC;

		// Fill the timestamp field of innovation
		target_innov.timestamp_sample = target_pos_obs.timestamp;
		target_innov.timestamp = hrt_absolute_time();

		// Loop over x,y,z directions. Note: even with coupled dynamics we have a sequential update of measurements in x,y,z directions separately
		for (int j = 0; j < 3; j++) {

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
				if (_target_model == TargetModel::Coupled) {

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
					_target_estimator[j]->setH(meas_h_row, j);
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
	}

	perf_end(_ltest_update_perf);

	// Publish innovations
	switch (target_pos_obs.type) {
	case target_gps_pos:
		_ltest_aid_gps_pos_target_pub.publish(target_innov);
		break;

	case mission_gps_pos:
		_ltest_aid_gps_pos_mission_pub.publish(target_innov);
		break;

	case vel_rel_gps:
		_ltest_aid_gps_vel_rel_pub.publish(target_innov);
		break;

	case vel_target_gps:
		_ltest_aid_gps_vel_target_pub.publish(target_innov);
		break;

	case fiducial_marker:
		_ltest_aid_fiducial_marker_pub.publish(target_innov);
		break;

	case irlock:
		_ltest_aid_irlock_pub.publish(target_innov);
		break;

	case uwb:
		_ltest_aid_uwb_pub.publish(target_innov);
		break;
	}

	return all_directions_fused;
}

void LTEstPosition::publishTarget()
{
	target_estimator_state_s target_estimator_state{};

	landing_target_pose_s target_pose{};

	target_pose.timestamp = _last_predict;
	target_estimator_state.timestamp = _last_predict;
	target_pose.is_static = (_target_mode == TargetMode::Stationary);

	target_pose.rel_pos_valid = (hrt_absolute_time() - _last_update < landing_target_valid_TIMEOUT_US);

	// TODO: eventually set to true, but for testing we don't want to use the target as an external source of velocity in the EKF
	target_pose.rel_vel_valid = false;

	if (_target_model == TargetModel::Coupled) {

		// Fill target pose
		const Vector3f pos_vect = _target_estimator_coupled->getPositionVect();
		target_pose.x_rel = pos_vect(0);
		target_pose.y_rel = pos_vect(1);
		target_pose.z_rel = pos_vect(2);

		const Vector3f cov_pos_vect = _target_estimator_coupled->getPosVarVect();
		target_pose.cov_x_rel = cov_pos_vect(0);
		target_pose.cov_y_rel = cov_pos_vect(1);
		target_pose.cov_z_rel = cov_pos_vect(2);

		const Vector3f vel_vect = _target_estimator_coupled->getVelocityVect();
		target_pose.vx_rel = vel_vect(0);
		target_pose.vy_rel = vel_vect(1);
		target_pose.vz_rel = vel_vect(2);

		const Vector3f cov_vel_vect = _target_estimator_coupled->getVelVarVect();
		target_pose.cov_vx_rel = cov_vel_vect(0);
		target_pose.cov_vy_rel = cov_vel_vect(1);
		target_pose.cov_vz_rel = cov_vel_vect(2);

		if (_target_mode == TargetMode::MovingAugmented) {

			// Fill target state msg
			const Vector3f vel_target_vect = _target_estimator_coupled->getTargetVel();
			target_estimator_state.vx_target = vel_target_vect(0);
			target_estimator_state.vy_target = vel_target_vect(1);
			target_estimator_state.vz_target = vel_target_vect(2);

			const Vector3f cov_vel_target_vect = _target_estimator_coupled->getTargetVelVar();
			target_estimator_state.cov_vx_target = cov_vel_target_vect(0);
			target_estimator_state.cov_vy_target = cov_vel_target_vect(1);
			target_estimator_state.cov_vz_target = cov_vel_target_vect(2);

			/* Var(aX + bY) = a^2 Var(x) + b^2 Var(y) + 2abCov(X,Y) */
			target_pose.cov_vx_rel += cov_vel_target_vect(0);
			target_pose.cov_vy_rel += cov_vel_target_vect(1);
			target_pose.cov_vz_rel += cov_vel_target_vect(2);

			// Overwrite Target pose msg
			target_pose.vx_rel = vel_target_vect(0) - vel_vect(0);
			target_pose.vy_rel = vel_target_vect(1) - vel_vect(1);
			target_pose.vz_rel = vel_target_vect(2) - vel_vect(2);
		}

		// Fill target estimator state
		target_estimator_state.x_rel = target_pose.x_rel;
		target_estimator_state.y_rel = target_pose.y_rel;
		target_estimator_state.z_rel = target_pose.z_rel;

		target_estimator_state.cov_x_rel = target_pose.cov_x_rel;
		target_estimator_state.cov_y_rel = target_pose.cov_y_rel;
		target_estimator_state.cov_z_rel = target_pose.cov_z_rel;

		target_estimator_state.vx_rel = target_pose.vx_rel;
		target_estimator_state.vy_rel = target_pose.vy_rel;
		target_estimator_state.vz_rel = target_pose.vz_rel;

		target_estimator_state.cov_vx_rel = target_pose.cov_vx_rel;
		target_estimator_state.cov_vy_rel = target_pose.cov_vy_rel;
		target_estimator_state.cov_vz_rel = target_pose.cov_vz_rel;

		const Vector3f bias_vect = _target_estimator_coupled->getBiasVect();
		target_estimator_state.x_bias = bias_vect(0);
		target_estimator_state.y_bias = bias_vect(1);
		target_estimator_state.z_bias = bias_vect(2);

		const Vector3f cov_bias_vect = _target_estimator_coupled->getBiasVarVect();
		target_estimator_state.cov_x_bias = cov_bias_vect(0);
		target_estimator_state.cov_y_bias = cov_bias_vect(1);
		target_estimator_state.cov_z_bias = cov_bias_vect(2);

		if (_target_mode == TargetMode::Moving || _target_mode == TargetMode::MovingAugmented) {

			const Vector3f acc_target_vect = _target_estimator_coupled->getAccelerationVect();
			target_estimator_state.ax_target = acc_target_vect(0);
			target_estimator_state.ay_target = acc_target_vect(1);
			target_estimator_state.az_target = acc_target_vect(2);

			const Vector3f cov_acc_target_vect = _target_estimator_coupled->getAccVarVect();
			target_estimator_state.cov_ax_target = cov_acc_target_vect(0);
			target_estimator_state.cov_ay_target = cov_acc_target_vect(1);
			target_estimator_state.cov_az_target = cov_acc_target_vect(2);
		}

	} else {

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

		if (_target_mode == TargetMode::MovingAugmented) {

			target_estimator_state.vx_target = _target_estimator[x]->getTargetVel();
			target_estimator_state.vy_target = _target_estimator[y]->getTargetVel();
			target_estimator_state.vz_target = _target_estimator[z]->getTargetVel();

			target_estimator_state.cov_vx_target = _target_estimator[x]->getTargetVelVar();
			target_estimator_state.cov_vy_target = _target_estimator[y]->getTargetVelVar();
			target_estimator_state.cov_vz_target = _target_estimator[z]->getTargetVelVar();

			target_pose.vx_rel = target_estimator_state.vx_target - target_pose.vx_rel;
			target_pose.vy_rel = target_estimator_state.vy_target - target_pose.vy_rel;
			target_pose.vz_rel = target_estimator_state.vz_target - target_pose.vz_rel;

			/* Var(aX + bY) = a^2 Var(x) + b^2 Var(y) + 2abCov(X,Y) */
			target_pose.cov_vx_rel += target_estimator_state.cov_vx_target;
			target_pose.cov_vy_rel += target_estimator_state.cov_vy_target;
			target_pose.cov_vz_rel += target_estimator_state.cov_vz_target;
		}

		// Fill target estimator state
		target_estimator_state.x_rel = target_pose.x_rel;
		target_estimator_state.y_rel = target_pose.y_rel;
		target_estimator_state.z_rel = target_pose.z_rel;

		target_estimator_state.cov_x_rel = target_pose.cov_x_rel;
		target_estimator_state.cov_y_rel = target_pose.cov_y_rel;
		target_estimator_state.cov_z_rel = target_pose.cov_z_rel;

		target_estimator_state.vx_rel = target_pose.vx_rel;
		target_estimator_state.vy_rel = target_pose.vy_rel;
		target_estimator_state.vz_rel = target_pose.vz_rel;

		target_estimator_state.cov_vx_rel = target_pose.cov_vx_rel;
		target_estimator_state.cov_vy_rel = target_pose.cov_vy_rel;
		target_estimator_state.cov_vz_rel = target_pose.cov_vz_rel;

		target_estimator_state.x_bias = _target_estimator[x]->getBias();
		target_estimator_state.y_bias = _target_estimator[y]->getBias();
		target_estimator_state.z_bias = _target_estimator[z]->getBias();

		target_estimator_state.cov_x_bias = _target_estimator[x]->getBiasVar();
		target_estimator_state.cov_y_bias = _target_estimator[y]->getBiasVar();
		target_estimator_state.cov_z_bias = _target_estimator[z]->getBiasVar();

		if (_target_mode == TargetMode::Moving || _target_mode == TargetMode::MovingAugmented) {
			target_estimator_state.ax_target = _target_estimator[x]->getAcceleration();
			target_estimator_state.ay_target = _target_estimator[y]->getAcceleration();
			target_estimator_state.az_target = _target_estimator[z]->getAcceleration();

			target_estimator_state.cov_ax_target = _target_estimator[x]->getAccVar();
			target_estimator_state.cov_ay_target = _target_estimator[y]->getAccVar();
			target_estimator_state.cov_az_target = _target_estimator[z]->getAccVar();
		}
	}

	// prec land does not check target_pose.abs_pos_valid. Only send the target if bas pose valid.
	if (_local_position.valid) {
		target_pose.x_abs = target_pose.x_rel + _local_position.xyz(0);
		target_pose.y_abs = target_pose.y_rel + _local_position.xyz(1);
		target_pose.z_abs = target_pose.z_rel + _local_position.xyz(2);
		target_pose.abs_pos_valid = true;
		_targetPosePub.publish(target_pose);

	}

	_targetEstimatorStatePub.publish(target_estimator_state);


	// TODO: decide what to do with Bias lim
	float bias_lim = _param_ltest_bias_lim.get();

	if (((float)fabs(target_estimator_state.x_bias) > bias_lim
	     || (float)fabs(target_estimator_state.y_bias) > bias_lim || (float)fabs(target_estimator_state.z_bias) > bias_lim)) {

		PX4_DEBUG("Bias exceeds limit: %.2f bias x: %.2f bias y: %.2f bias z: %.2f", (double)bias_lim,
			  (double)target_estimator_state.x_bias, (double)target_estimator_state.y_bias, (double)target_estimator_state.z_bias);

		// resetFilter();
	}

}

void LTEstPosition::_check_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();
	}

	// Make sure range sensor and local position are up to date.
	if (_range_sensor.valid) {
		_range_sensor.valid = (hrt_absolute_time() - _range_sensor.last_update) < measurement_updated_TIMEOUT_US;
	}

	if (_local_position.valid) {
		_local_position.valid = (hrt_absolute_time() - _local_position.last_update) < measurement_updated_TIMEOUT_US;
	}
}

void LTEstPosition::set_range_sensor(const float dist, const bool valid)
{
	_range_sensor.valid = valid;
	_range_sensor.dist_bottom = dist;
	_range_sensor.last_update = hrt_absolute_time();

}

void LTEstPosition::set_local_position(const matrix::Vector3f &xyz, const bool valid)
{
	_local_position.xyz = xyz;
	_local_position.valid = valid;
	_local_position.last_update = hrt_absolute_time();
}

void LTEstPosition::set_landpoint(const int lat, const int lon, const float alt)
{
	if (_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS) {
		_landing_pos.lat = lat;
		_landing_pos.lon = lon;
		_landing_pos.alt = alt;
		_landing_pos.valid = (_landing_pos.lat != 0 && _landing_pos.lon != 0 && PX4_ISFINITE(_landing_pos.alt));

		if (_landing_pos.valid) {
			PX4_INFO("Landing pos used lat: %.0f [1e-7 deg] -- lon: %.0f [1e-7 deg] -- alt %.2f [m]", (double)(lat),
				 (double)(lon), (double)(alt / 1000));

		} else {
			PX4_INFO("Landing target estimator detection enabled but landing pos not valid. lat: %.2f [1e-7 deg] -- lon: %.2f [1e-7 deg] -- alt %.2f [m]",
				 (double)(lat),
				 (double)(lon), (double)(alt));
		}

	} else {
		_landing_pos.valid = false;
	}
}

void LTEstPosition::updateParams()
{
	ModuleParams::updateParams();

	_target_acc_unc = _param_ltest_acc_t_unc.get();
	_bias_unc = _param_ltest_bias_unc.get();
	_meas_unc = _param_ltest_meas_unc.get();
	_drone_acc_unc = _param_ltest_acc_d_unc.get();
}

bool LTEstPosition::selectTargetEstimator()
{
	Base_KF_decoupled *tmp_x = nullptr;
	Base_KF_decoupled *tmp_y = nullptr;
	Base_KF_decoupled *tmp_z = nullptr;
	Base_KF_coupled *tmp_xyz = nullptr;

	switch (_target_mode) {
	case TargetMode::Stationary:

		if (_target_model == TargetModel::Decoupled) {

			tmp_x = new KF_xyzb_decoupled_static;
			tmp_y = new KF_xyzb_decoupled_static;
			tmp_z = new KF_xyzb_decoupled_static;
			PX4_INFO("Init LTEst: static target, [x,y,z,b] decoupled in three filters.");

		} else {

			tmp_xyz = new KF_xyzb_coupled_static;
			PX4_INFO("Init LTEst: static target, [x,y,z,b] coupled in one filter.");
		}

		break;

	case TargetMode::Moving:

		if (_target_model == TargetModel::Decoupled) {

			tmp_x = new KF_xyzb_decoupled_moving;
			tmp_y = new KF_xyzb_decoupled_moving;
			tmp_z = new KF_xyzb_decoupled_moving;
			PX4_INFO("Init LTEst: moving target, [x,y,z,b] decoupled in three filters.");

		} else {

			tmp_xyz = new KF_xyzb_coupled_moving;
			PX4_INFO("Init LTEST: moving target, [x,y,z,b] coupled in one filter.");
		}

		break;

	case TargetMode::MovingAugmented:

		if (_target_model == TargetModel::Decoupled) {

			tmp_x = new KF_xyzb_v_decoupled_moving;
			tmp_y = new KF_xyzb_v_decoupled_moving;
			tmp_z = new KF_xyzb_v_decoupled_moving;
			PX4_INFO("Init LTEst: moving target, [x,y,z,b,v] decoupled in three filters.");

		} else {
			tmp_xyz = new KF_xyzb_v_coupled_moving;
			PX4_INFO("Init LTEST: moving target, [x,y,z,b,v] coupled in one filter.");
		}

	case TargetMode::NotInit:
		break;
	}

	bool init_failed = true;

	if (_target_model == TargetModel::Decoupled) {
		init_failed = ((tmp_x == nullptr) || (tmp_y == nullptr) || (tmp_z == nullptr));

	} else {

		init_failed = (tmp_xyz == nullptr);
	}

	if (init_failed) {
		PX4_ERR("LTE init failed");
		return false;

	} else {

		switch (_target_model) {
		case TargetModel::Decoupled:

			delete _target_estimator[x];
			delete _target_estimator[y];
			delete _target_estimator[z];

			_target_estimator[x] = tmp_x;
			_target_estimator[y] = tmp_y;
			_target_estimator[z] = tmp_z;

			break;

		case TargetModel::Coupled:
			delete _target_estimator_coupled;
			_target_estimator_coupled = tmp_xyz;

			break;

		case TargetModel::NotInit:
			break;
		}

		return true;
	}
}

} // namespace landing_target_estimator
