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
		- Implement new filters (stationary and moving target) using symforce
		- Complete the landing target MAVLINK message to include measurement covariance matrix
		- Get drone's acceleration uncertainty
		-
		- Finish measurement processing: Get mission land point lat/lon
		- Complete landing_target_pose.msg to include theta
		- Orientation (the precision landing algorithm doesn't take the orientation into consideration yet):
			- Define initial values
			- Complete landing_target_pose.msg to include theta
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
	_check_params(true);
}

LandingTargetEstimator::~LandingTargetEstimator()
{
	for (int i = 0; i < 4; i++) {
		delete _target_estimator[i];
	}
}

void LandingTargetEstimator::update()
{
	_check_params(false);

	// Get curent Body-NED rotation matrix, and sensor observations

	accInput input;

	_update_topics(&input);

	// No attitude or acceleration: early return;
	if (!input.acc_ned_valid) {
		return;
	}

	/* predict */
	if (_estimator_initialized) {

		if (hrt_absolute_time() - _last_update > landing_target_estimator_TIMEOUT_US) {
			PX4_INFO("Lost sight of Marker");
			_estimator_initialized = false;

		} else {
			predictionStep(input.vehicle_acc_ned);
			_last_predict = hrt_absolute_time();
		}
	}

	/* init estimator */
	if (!_estimator_initialized) {

		// Wait for a sensor update to get the initial position of the target
		if (_new_sensorReport) {
			initEstimator();
			_estimator_initialized = true;
			_last_update = hrt_absolute_time();
			_last_predict = _last_update;
		}

	} else {

		// If estimator initialized, and we have at least one measurement: update step
		if (_new_sensorReport) {

			// If at least one sensor observation was fused in all directions (x,y,z) mark state as updated
			if (updateNED(input.vehicle_acc_ned)) {
				_last_update = _last_predict;
			}

			// Update orientation (Eventually use orientation_updated)
			if (_estimate_orientation) {
				updateOrientation();
			}
		}

		// mark sensor measurements as consumed
		_new_sensorReport = false;

		// Always publish the target (using the prediction step). The target is set as invalid if: time from the previous update > landing_target_valid_TIMEOUT_US);
		publishTarget();
		publishInnovations();
	}
}

void LandingTargetEstimator::initEstimator()
{
	// TODO: do we want to init the target/UAV GPS bias?

	// This function is only called once, it's better to re-update the vehicle local position once than having it in the stack memory
	vehicle_local_position_s	vehicle_local_position;
	_vehicleLocalPositionSub.update(&vehicle_local_position);

	// Define the initial relative position of target w.r.t. the drone in NED frame using the available measurement
	Vector3f p_init;

	if (_target_pos_obs[target_gps_pos].updated_xyz(0) && _target_pos_obs[target_gps_pos].updated_xyz(1)
	    && _target_pos_obs[target_gps_pos].updated_xyz(2)) {
		p_init = _target_pos_obs[target_gps_pos].meas_xyz;

	} else if (_target_pos_obs[fiducial_marker].updated_xyz(0) && _target_pos_obs[fiducial_marker].updated_xyz(1)
		   && _target_pos_obs[fiducial_marker].updated_xyz(2)) {
		p_init = _target_pos_obs[fiducial_marker].meas_xyz;

	} else if (_target_pos_obs[irlock].updated_xyz(0) && _target_pos_obs[irlock].updated_xyz(1)
		   && _target_pos_obs[irlock].updated_xyz(2)) {
		p_init = _target_pos_obs[irlock].meas_xyz;

	} else if (_target_pos_obs[uwb].updated_xyz(0) && _target_pos_obs[uwb].updated_xyz(1)
		   && _target_pos_obs[uwb].updated_xyz(2)) {
		p_init = _target_pos_obs[uwb].meas_xyz;

	} else {
		p_init.zero();
	}

	// Define initial relative velocity of the target w.r.t. to the drone in NED frame
	Vector3f v_rel_init;
	v_rel_init(0) = vehicle_local_position.v_xy_valid ? -vehicle_local_position.vx : 0.f;
	v_rel_init(1) = vehicle_local_position.v_xy_valid ? -vehicle_local_position.vy : 0.f;
	v_rel_init(2) = vehicle_local_position.v_z_valid  ? -vehicle_local_position.vz : 0.f;

	// Define initial acceleration of the target in NED frame. Since we have no info on the target, assume no initial acceleration
	Vector3f a_init{};

	PX4_INFO("Pos init %.2f %.2f %.2f", (double)p_init(0), (double)p_init(1), (double)p_init(2));
	PX4_INFO("Vel init %.2f %.2f %.2f", (double)v_rel_init(0), (double)v_rel_init(1), (double)v_rel_init(2));
	PX4_INFO("Acc init %.2f %.2f %.2f", (double)a_init(0), (double)a_init(1), (double)a_init(2));

	float statePosVar = _param_ltest_pos_unc_in.get();
	float stateVelVar = _param_ltest_vel_unc_in.get();
	float stateAccVar = _param_ltest_acc_unc_in.get();

	Vector3f statePosVarVect(statePosVar, statePosVar, statePosVar);
	Vector3f stateVelVarVect(stateVelVar, stateVelVar, stateVelVar);
	Vector3f stateAccVarVect(stateAccVar, stateAccVar, stateAccVar);

	if (_target_model == TargetModel::FullPoseCoupled) {

		_target_estimator[xyz]->setPosition(p_init);
		_target_estimator[xyz]->setVelocity(v_rel_init);
		_target_estimator[xyz]->setTargetAcc(a_init);

		_target_estimator[xyz]->setStatePosVar(statePosVarVect);
		_target_estimator[xyz]->setStateVelVar(stateVelVarVect);
		_target_estimator[xyz]->setStateAccVar(stateAccVarVect);

	} else {

		for (int i = 0; i < _nb_position_kf; i++) {
			_target_estimator[i]->setPosition(p_init(i));
			_target_estimator[i]->setVelocity(v_rel_init(i));
			_target_estimator[i]->setTargetAcc(a_init(i));

			_target_estimator[i]->setStatePosVar(statePosVarVect(i));
			_target_estimator[i]->setStateVelVar(stateVelVarVect(i));
			_target_estimator[i]->setStateAccVar(stateAccVarVect(i));
		}
	}

	if (_estimate_orientation) {
		//TODO: define thse values
		_target_estimator[theta]->setPosition(0.f);
		_target_estimator[theta]->setVelocity(0.f);
		_target_estimator[theta]->setStatePosVar(statePosVar);
		_target_estimator[theta]->setStateVelVar(stateVelVar);
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

	// Rotate input covariance from body to NED (note _q_att was updated by _update_topics())
	Dcmf R_att = Dcm<float>(_q_att);
	input_cov = R_att * input_cov * R_att.transpose();

	// TODO: This can eventually be moved to the updateParams function and set as a global variable because no rotation is required
	float target_acc_unc = _param_ltest_acc_t_unc.get();
	SquareMatrix<float, 3> target_acc_cov = diag(Vector3f(target_acc_unc, target_acc_unc, target_acc_unc));

	float bias_unc = _param_ltest_bias_unc.get();
	SquareMatrix<float, 3> bias_cov = diag(Vector3f(bias_unc, bias_unc, bias_unc));

	if (_target_model == TargetModel::FullPoseCoupled) {

		if (_target_mode == TargetMode::Moving) {_target_estimator[xyz]->setTargetAccVar(target_acc_cov);}

		_target_estimator[xyz]->setBiasVar(bias_cov);
		_target_estimator[xyz]->setInputAccVar(input_cov);
		_target_estimator[xyz]->predictState(dt, vehicle_acc_ned);
		_target_estimator[xyz]->predictCov(dt);

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
		_target_estimator[theta]->predictState(dt, 0.0);
		_target_estimator[theta]->predictCov(dt);
	}
}

bool LandingTargetEstimator::updateNED(Vector3f vehicle_acc_ned)
{

	Vector<bool, 3> meas_xyz_fused{};

	bool all_directions_fused = false;
	float dt_sync;
	Vector<float, 12> meas_h_row;

	// Number of direction:  x,y,z for all filters excep for the horizontal filter: x,y
	int nb_update_directions = (_target_model == TargetModel::Horizontal) ? 3 : 2;

	// Loop over senors (gps_pos, gps_vel, vision, range_z, irlock, uwb)
	for (int i = 0; i < nb_observations; i++) {

		// Check if at least one measurement from this sensor was updated
		if (!_target_pos_obs[i].any_xyz_updated) {

			// No measurement update, set to false
			for (int k = 0; k < 3; k++) {
				_target_innovations_array[i].fusion_enabled[k] = false;
				_target_innovations_array[i].fused[k] = false;
			}

		} else {

			// Compute the measurement's time delay (difference between state and measurement time on validity)
			dt_sync = (_last_predict - _target_pos_obs[i].timestamp);

			if (dt_sync > measurement_valid_TIMEOUT_US) {

				PX4_INFO("Measurement rejected because too old. Time sync: %.2f [seconds] > timeout: %.2f [seconds]",
					 (double)(dt_sync / SEC2USEC), (double)(measurement_valid_TIMEOUT_US / SEC2USEC));

			} else {

				// Convert time sync to seconds
				dt_sync = dt_sync / SEC2USEC;
				// TODO: Eventually remove, for now to debug, assume prediction time = measurement time
				dt_sync = 0.f;
				// Fill the timestamp field of innovation
				_target_innovations_array[i].timestamp_sample = _target_pos_obs[i].timestamp;
				_target_innovations_array[i].timestamp =
					hrt_absolute_time(); // TODO: check if correct hrt_absolute_time() or _last_predict

				// Mark measurements as consumed for the next step.
				_target_pos_obs[i].any_xyz_updated = false;

				// Loop over x,y,z directions. Note: even with coupled dynamics we have a sequential update of x,y,z directions separately
				for (int j = 0; j < nb_update_directions; j++) {

					//If the measurement of this filter (x,y or z) has not been updated:
					if (!_target_pos_obs[i].updated_xyz(j)) {

						// No measurement Note: .fusion_enabled[j] and .fused[j] = false are already false by default. (set in publishInnovations())
						PX4_INFO("At least one non-valid observation. x: %d, y: %d, z: %d", _target_pos_obs[i].updated_xyz(0),
							 _target_pos_obs[i].updated_xyz(1), _target_pos_obs[i].updated_xyz(2));

					} else {

						int filter_idx = j;

						// Sync measurement using the prediction model
						if (_target_model == TargetModel::FullPoseCoupled) {
							// For the single filter, update the same estimator for each measurement (since there is only one)
							filter_idx = xyz;
							// Move state back to the measurement time of validity. The state synchronized will be used to compute the innovation.
							_target_estimator[filter_idx]->syncState(dt_sync, vehicle_acc_ned);

						} else {
							_target_estimator[filter_idx]->syncState(dt_sync, vehicle_acc_ned(j));
						}

						//Get the corresponding row of the H matrix.
						meas_h_row = _target_pos_obs[i].meas_h_xyz.row(j);
						_target_estimator[filter_idx]->setH(meas_h_row);

						// Compute innovations and fill thet target innovation message
						_target_innovations_array[i].innovation_variance[j] = _target_estimator[filter_idx]->computeInnovCov(
									_target_pos_obs[i].meas_unc_xyz(j));
						_target_innovations_array[i].innovation[j] = _target_estimator[filter_idx]->computeInnov(_target_pos_obs[i].meas_xyz(
									j));

						// Update step
						meas_xyz_fused(j) = _target_estimator[filter_idx]->update();

						// Fill the target innovation message
						_target_innovations_array[i].fusion_enabled[j] = true;
						_target_innovations_array[i].innovation_rejected[j] = !meas_xyz_fused(j);
						_target_innovations_array[i].fused[j] = meas_xyz_fused(j);

						_target_innovations_array[i].observation[j] = _target_pos_obs[i].meas_xyz(j);
						_target_innovations_array[i].observation_variance[j] = _target_pos_obs[i].meas_unc_xyz(j);

						/*	TODO: fill those fields of estimator_aid_source_3d_s
							uint8 estimator_instance
							uint32 device_id
							uint64[3] time_last_fuse
							float32[3] test_ratio
						*/

						// Mark measurement as consumed for next iteration
						_target_pos_obs[i].updated_xyz(j) = false;

					}
				}

				// If we have updated all three directions (x,y,z) for one measurement, consider the state updated.
				if (meas_xyz_fused(0) && meas_xyz_fused(1) && (meas_xyz_fused(2) || _target_model == TargetModel::Horizontal)) {
					all_directions_fused = true;
				}

				// Reset meas_xyz_fused to false
				meas_xyz_fused.setAll(false);
			}
		}
	}

	return all_directions_fused;
}

bool LandingTargetEstimator::updateOrientation()
{
	// Update step for orientation
	bool meas_fused = false;

	if (_target_orientation_obs.updated_theta) {
		_target_estimator[theta]->setH(_target_orientation_obs.meas_h_theta);
		_target_estimator_aid_ev_yaw.innovation_variance = _target_estimator[theta]->computeInnovCov(
					_target_orientation_obs.meas_unc_theta);
		_target_estimator_aid_ev_yaw.innovation = _target_estimator[theta]->computeInnov(_target_orientation_obs.meas_theta);
		meas_fused = _target_estimator[theta]->update();

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

	return meas_fused;
}

void LandingTargetEstimator::publishInnovations()
{
	// One innov message for each sensor.
	_target_estimator_aid_gps_pos_pub.publish(_target_innovations_array[target_gps_pos]);
	_target_estimator_aid_gps_vel_pub.publish(_target_innovations_array[uav_gps_vel]);
	_target_estimator_aid_vision_pub.publish(_target_innovations_array[fiducial_marker]);
	_target_estimator_aid_irlock_pub.publish(_target_innovations_array[irlock]);
	_target_estimator_aid_uwb_pub.publish(_target_innovations_array[uwb]);

	// Orientation innovation
	_target_estimator_aid_ev_yaw_pub.publish(_target_estimator_aid_ev_yaw);

	// Reset innovations to zero
	for (int i = 0; i < nb_observations; i++) {
		for (int k = 0; k < 3; k++) {
			_target_innovations_array[i].fusion_enabled[k] = false;
			_target_innovations_array[i].fused[k] = false;
		}
	}
}

void LandingTargetEstimator::publishTarget()
{
	//TODO: Update target_pose msg to add orientation: target_pose.rel_orientation_valid -- target_pose.theta_rel -- target_pose.cov_theta_rel
	landing_target_pose_s target_pose{};

	target_pose.timestamp = _last_predict;
	target_pose.is_static = (_target_mode == TargetMode::Stationary);

	bool target_valid = (hrt_absolute_time() - _last_update > landing_target_valid_TIMEOUT_US);

	// TODO: have _last_update_pos ; _last_update_vel ; _last_update_orientation
	target_pose.rel_pos_valid = target_valid;
	target_pose.rel_vel_valid = target_valid;
	// target_pose.rel_orientation_valid = target_valid;

	if (_target_model == TargetModel::FullPoseCoupled) {

		Vector3f posVect = _target_estimator[xyz]->getPositionVect();
		target_pose.x_rel = posVect(0);
		target_pose.y_rel = posVect(1);
		target_pose.z_rel = posVect(2);

		Vector3f covPosVect = _target_estimator[xyz]->getPosVarVect();
		target_pose.cov_x_rel = covPosVect(0);
		target_pose.cov_y_rel = covPosVect(1);
		target_pose.cov_z_rel = covPosVect(2);

		Vector3f velVect = _target_estimator[xyz]->getVelocityVect();
		target_pose.vx_rel = velVect(0);
		target_pose.vy_rel = velVect(1);
		target_pose.vz_rel = velVect(2);

		Vector3f covVelVect = _target_estimator[xyz]->getVelVarVect();
		target_pose.cov_vx_rel = covVelVect(0);
		target_pose.cov_vy_rel = covVelVect(1);
		target_pose.cov_vz_rel = covVelVect(2);

	} else {
		// This should eventually be removed
		float rel_z = 0.f;

		if (_target_model == TargetModel::Horizontal) {
			if (_target_pos_obs[irlock].updated_xyz(2)) {
				rel_z = _target_pos_obs[irlock].meas_xyz(2);

			} else if (_target_pos_obs[uwb].updated_xyz(2)) {
				rel_z = _target_pos_obs[uwb].meas_xyz(2);
			}
		}

		target_pose.x_rel = _target_estimator[x]->getPosition();
		target_pose.y_rel = _target_estimator[y]->getPosition();
		target_pose.z_rel = _nb_position_kf > 2 ? _target_estimator[z]->getPosition() : rel_z;

		target_pose.cov_x_rel = _target_estimator[x]->getPosVar();
		target_pose.cov_y_rel = _target_estimator[y]->getPosVar();
		target_pose.cov_z_rel = _nb_position_kf > 2 ? _target_estimator[z]->getPosVar() : 0.f;


		target_pose.vx_rel = _target_estimator[x]->getVelocity();
		target_pose.vy_rel = _target_estimator[y]->getVelocity();
		target_pose.vz_rel = _nb_position_kf > 2 ? _target_estimator[z]->getVelocity() : 0.f;

		target_pose.cov_vx_rel = _target_estimator[x]->getVelVar();
		target_pose.cov_vy_rel = _target_estimator[y]->getVelVar();
		target_pose.cov_vz_rel = _nb_position_kf > 2 ? _target_estimator[z]->getVelVar() : 0.f;
	}

	// TODO: eventually uncomment
	// target_pose.theta_rel = _estimate_orientation ? _target_estimator[theta]->getPosition() : 0.f; ;
	// target_pose.vtheta_abs = _target_estimator[theta]->getVelocity();
	// target_pose.cov_theta_rel =  _target_estimator[theta]->getPosVar();

	if (_local_pos.valid) {
		target_pose.x_abs = target_pose.x_rel + _local_pos.x;
		target_pose.y_abs = target_pose.y_rel + _local_pos.y;
		target_pose.z_abs = target_pose.z_rel + _local_pos.z;
		target_pose.abs_pos_valid = true;

	} else {
		target_pose.abs_pos_valid = false;
	}

	_targetPosePub.publish(target_pose);
}

void LandingTargetEstimator::_check_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();
	}
}

void LandingTargetEstimator::_update_topics(accInput *input)
{
	sensor_gps_s vehicle_gps_position;
	landing_target_pose_s target_GNSS_report;
	landing_target_pose_s fiducial_marker_pose;
	irlock_report_s irlock_report;
	uwb_distance_s	uwb_distance;
	vehicle_attitude_s	vehicle_attitude;
	vehicle_local_position_s	vehicle_local_position;
	vehicle_status_s vehicle_status;
	position_setpoint_triplet_s pos_sp_triplet;
	vehicle_acceleration_s		vehicle_acceleration;

	//Update topics
	bool vehicle_local_position_valid = _vehicleLocalPositionSub.update(&vehicle_local_position);
	bool vehicle_attitude_valid = _attitudeSub.update(&vehicle_attitude);
	bool vehicle_acceleration_valid = _vehicle_acceleration_sub.update(&vehicle_acceleration);

	if (_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS) {

		// Update nav state
		if (_vehicle_status_sub.update(&vehicle_status)) {
			_nave_state = vehicle_status.nav_state;
		}

		// Get landing target from mission:
		if (!_landing_pos.valid && _pos_sp_triplet_sub.update(&pos_sp_triplet)) {
			if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
				_landing_pos.lat = (int)(pos_sp_triplet.current.lat * 1e7);
				_landing_pos.lon = (int)(pos_sp_triplet.current.lon * 1e7);
				_landing_pos.alt = pos_sp_triplet.current.alt * 1000.f;
				_landing_pos.valid = (_landing_pos.lat != 0 && _landing_pos.lon != 0);
			}
		}
	}

	// To save stack space, only use x,y,z,valid as global variables (_local_pos is used when the target is published)
	_local_pos.x = vehicle_local_position.x;
	_local_pos.y = vehicle_local_position.y;
	_local_pos.z = vehicle_local_position.z;
	_local_pos.valid = (vehicle_local_position_valid && vehicle_local_position.xy_valid);

	Dcmf R_att;

	// Minimal requirement: acceleraion (for input) and attitude (to rotate acc in vehicle-carried NED frame)
	if (!vehicle_attitude_valid || !vehicle_acceleration_valid) {
		PX4_INFO("Kalman input not available: Attitude: %d, Acc: %d", vehicle_attitude_valid, vehicle_acceleration_valid);
		input->acc_ned_valid = false;
		return;

	} else {
		Quaternion<float> q_att(&vehicle_attitude.q[0]);
		_q_att = q_att;
		R_att = Dcm<float>(_q_att);

		// Transform body acc to NED
		Vector3f vehicle_acc{vehicle_acceleration.xyz};
		input->vehicle_acc_ned = R_att * vehicle_acc;
		input->acc_ned_valid = true;
	}

	/* IRLOCK SENSOR measures [r_x,r_y,r_z]*/
	if (((_ltest_aid_mask & SensorFusionMask::USE_IRLOCK_POS)) && _irlockReportSub.update(&irlock_report)) {

		if (!vehicle_local_position_valid || !vehicle_local_position.dist_bottom_valid) {
			// don't have the data needed for an update
			PX4_INFO("Local pos: %d, Dist bottom: %d", vehicle_local_position_valid, vehicle_local_position.dist_bottom_valid);

		} else if (!PX4_ISFINITE(irlock_report.pos_y) || !PX4_ISFINITE(irlock_report.pos_x)) {
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
			sensor_ray = R_att * sensor_ray;

			//TODO: rotate measurement uncertainty S_att & R_att

			// z component of measurement safe, use this measurement
			if (fabsf(sensor_ray(2)) > 1e-6f) {

				float dist_z = vehicle_local_position.dist_bottom - _param_ltest_sens_pos_z.get();

				// scale the ray s.t. the z component has length of _uncertainty_scale
				float rel_pos_x = sensor_ray(0) / sensor_ray(2) * dist_z;
				float rel_pos_y = sensor_ray(1) / sensor_ray(2) * dist_z;
				float rel_pos_z = dist_z;

				// Adjust relative position according to sensor offset
				rel_pos_x += _param_ltest_sens_pos_x.get();
				rel_pos_y += _param_ltest_sens_pos_y.get();

				//TODO: For coupled dynamics: Cholesky decomposition to uncorrelate noise (find matrix T that diagonalizes R) z' = T*z; H' = T*H

				// Fill the observations for the irlock sensor
				targetObsPos temp_obs = {};

				temp_obs.timestamp = irlock_report.timestamp;

				if (_target_model == TargetModel::FullPoseCoupled) {
					// State: [rx, ry, rz, ... ]
					temp_obs.meas_h_xyz(0, 0) = 1; // x direction
					temp_obs.meas_h_xyz(1, 1) = 1; // y direction
					temp_obs.meas_h_xyz(2, 2) = 1; // z direction

				} else {
					// State: [r, r_dot, ...] --> x, y, z directions are the same (decoupled)
					temp_obs.meas_h_xyz(0, 0) = 1;
					temp_obs.meas_h_xyz(1, 0) = 1;
					temp_obs.meas_h_xyz(2, 0) = 1;
				}

				temp_obs.meas_xyz(0) = rel_pos_x;
				temp_obs.meas_xyz(1) = rel_pos_y;
				temp_obs.meas_xyz(2) = rel_pos_z;

				temp_obs.any_xyz_updated = true;
				temp_obs.updated_xyz.setAll(true);

				float measurement_uncertainty = _param_ltest_meas_unc.get() * dist_z * dist_z;

				temp_obs.meas_unc_xyz(0) = measurement_uncertainty;
				temp_obs.meas_unc_xyz(1) = measurement_uncertainty;
				temp_obs.meas_unc_xyz(2) = measurement_uncertainty;

				_target_pos_obs[irlock] = temp_obs;
				_new_sensorReport = true;
			}
		}
	}

	/* UWB SENSOR measures [r_x,r_y,r_z] */
	if (((_ltest_aid_mask & SensorFusionMask::USE_UWB_POS)) && _uwbDistanceSub.update(&uwb_distance)) {
		if (!vehicle_local_position_valid) {
			// don't have the data needed for an update
			PX4_INFO("Attitude: %d, Local pos: %d", vehicle_attitude_valid, vehicle_local_position_valid);

		} else if (!PX4_ISFINITE((float)uwb_distance.position[0]) || !PX4_ISFINITE((float)uwb_distance.position[1]) ||
			   !PX4_ISFINITE((float)uwb_distance.position[2])) {
			PX4_WARN("UWB position is corrupt!");

		} else {

			targetObsPos temp_obs = {};

			temp_obs.timestamp = uwb_distance.timestamp;

			if (_target_model == TargetModel::FullPoseCoupled) {
				// State: [rx, ry, rz, ... ]
				temp_obs.meas_h_xyz(0, 0) = 1; // x direction
				temp_obs.meas_h_xyz(1, 1) = 1; // y direction
				temp_obs.meas_h_xyz(2, 2) = 1; // z direction

			} else {
				// State: [r, r_dot, ...] --> x, y, z directions are the same (decoupled)
				temp_obs.meas_h_xyz(0, 0) = 1;
				temp_obs.meas_h_xyz(1, 0) = 1;
				temp_obs.meas_h_xyz(2, 0) = 1;
			}

			// The coordinate system is NED (north-east-down)
			// the uwb_distance msg contains the Position in NED, Vehicle relative to LP
			// The coordinates "rel_pos_*" are the position of the landing point relative to the vehicle.
			// To change POV we negate every Axis:
			temp_obs.meas_xyz(0) = -uwb_distance.position[0];
			temp_obs.meas_xyz(1) = -uwb_distance.position[1];
			temp_obs.meas_xyz(2) = -uwb_distance.position[2];

			temp_obs.any_xyz_updated = true;
			temp_obs.updated_xyz.setAll(true);

			float dist_z = vehicle_local_position.dist_bottom - _param_ltest_sens_pos_z.get();

			float measurement_uncertainty = _param_ltest_meas_unc.get() * dist_z * dist_z;

			temp_obs.meas_unc_xyz(0) = measurement_uncertainty;
			temp_obs.meas_unc_xyz(1) = measurement_uncertainty;

			_target_pos_obs[uwb] = temp_obs;
			_new_sensorReport = true;
		}
	}

	/* VISION SENSOR measures [r_x,r_y,r_z, r_theta]*/
	if ((_ltest_aid_mask & SensorFusionMask::USE_EXT_VIS_POS)
	    && _fiducial_marker_report_sub.update(&fiducial_marker_pose)) {

		// Assume vision measurement is in NED frame.
		// Note: (The vision estimate is rotated into the NED navigation frame offboard to avoid sync issues (target detection needs time))

		// Fiducial marker measurements :
		float vision_r_x = fiducial_marker_pose.x_rel;
		float vision_r_y = fiducial_marker_pose.y_rel;
		float vision_r_z = fiducial_marker_pose.z_rel;
		float vision_r_theta = 0.f; //TODO: complete MAVLINK message to obtain theta

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
		SquareMatrix<float, 3> R_rotated = diag(Vector3f(meas_uncertainty, meas_uncertainty, meas_uncertainty));

		float vision_r_theta_unc = 0.f;
		hrt_abstime vision_timestamp = fiducial_marker_pose.timestamp;

		/* ORIENTATION */
		if (!vehicle_local_position_valid) {
			// don't have the data needed for an update
			PX4_INFO("Attitude: %d, Local pos: %d", vehicle_attitude_valid, vehicle_local_position_valid);

		} else if (!PX4_ISFINITE(vision_r_theta)) {
			PX4_WARN("VISION orientation is corrupt!");

		} else {

			// TODO: obtain relative orientation using the orientation of the drone.
			// (vision gives orientation between body and target frame. We can thus use the orientation between the body frame and NED to obtain the orientation between NED and target)

			_target_orientation_obs.timestamp = vision_timestamp;
			_target_orientation_obs.updated_theta = true;
			_target_orientation_obs.meas_unc_theta = vision_r_theta_unc;
			_target_orientation_obs.meas_theta = vision_r_theta;
			_target_orientation_obs.meas_h_theta = 1;

			_new_sensorReport = true;
		}

		/* RELATIVE POSITION*/
		if (!PX4_ISFINITE(vision_r_x) || !PX4_ISFINITE(vision_r_y) || !PX4_ISFINITE(vision_r_y)) {
			PX4_WARN("VISION position is corrupt!");

		} else {

			targetObsPos temp_obs = {};

			temp_obs.timestamp = vision_timestamp;

			temp_obs.any_xyz_updated = true;
			temp_obs.updated_xyz.setAll(true);

			if (_target_model == TargetModel::FullPoseCoupled) {
				// Process measurements for sequential update by diagonalizing the measurement covariance matrix

				// Sliced state: [r_x,r_y,r_z]; (real state: [pose,vel,bias,acc] or [pose,vel,bias] but assume vision measurements independent of other measurements)
				Matrix<float, 3, 3> H_position =  zeros<float, 3, 3>();
				H_position(0, 0) = 1;
				H_position(1, 1) = 1;
				H_position(2, 2) = 1;

				// Cholesky decomposition using PX4 matrix:
				// TODO: make sure this is correct
				Matrix<float, 3, 3> T =  matrix::cholesky(R_rotated);

				// Diagonalize R_rotated:
				Matrix<float, 3, 3> R_diag =  T * R_rotated;

				temp_obs.meas_unc_xyz(0) = R_diag(0, 0);
				temp_obs.meas_unc_xyz(1) = R_diag(1, 1);
				temp_obs.meas_unc_xyz(2) = R_diag(2, 2);
				//TODO: replace by temp_obs.meas_unc_xyz = R_diag.diag()

				//Transform measurements Z:
				Vector3f Z(vision_r_x, vision_r_y, vision_r_z);
				Vector3f Z_transformed = T * Z;

				temp_obs.meas_xyz = Z_transformed;

				//Transform H
				Matrix<float, 3, 3> H_transformed = T * H_position;

				//Bring H_position back to the full H:
				//TODO: use slicing functions to insert 3x3 matrix H_transformed into 3x12 matrix meas_h_xyz
				temp_obs.meas_h_xyz(0, 0) = H_transformed(0, 0);
				temp_obs.meas_h_xyz(0, 1) = H_transformed(0, 1);
				temp_obs.meas_h_xyz(0, 2) = H_transformed(0, 2);

				temp_obs.meas_h_xyz(1, 0) = H_transformed(1, 0);
				temp_obs.meas_h_xyz(1, 1) = H_transformed(1, 1);
				temp_obs.meas_h_xyz(1, 2) = H_transformed(1, 2);

				temp_obs.meas_h_xyz(2, 0) = H_transformed(2, 0);
				temp_obs.meas_h_xyz(2, 1) = H_transformed(2, 1);
				temp_obs.meas_h_xyz(2, 2) = H_transformed(2, 2);

			} else {
				// Assume noise correlation negligible:

				// State: [r, r_dot, ...]
				temp_obs.meas_h_xyz(0, 0) = 1;
				temp_obs.meas_h_xyz(1, 0) = 1;
				temp_obs.meas_h_xyz(2, 0) = 1;

				temp_obs.meas_xyz(0) = vision_r_x;
				temp_obs.meas_xyz(1) = vision_r_y;
				temp_obs.meas_xyz(2) = vision_r_z;

				// Assume off diag elements ~ 0
				temp_obs.meas_unc_xyz(0) = R_rotated(0, 0);
				temp_obs.meas_unc_xyz(1) = R_rotated(1, 1);
				temp_obs.meas_unc_xyz(2) = R_rotated(2, 2);
			}

			_new_sensorReport = true;
			_target_pos_obs[fiducial_marker] = temp_obs;
		}
	}

	// TODO: might need to create a class variable with (vehicle_gps_position.timestamp_sample, pos_valid, lat, lon, alt, eph, epv, vel_ned_valid, vel_n, vel_e, vel_d, vel_var).
	/* Measurement requiering GPS observation*/
	if (_vehicle_gps_position_sub.update(&vehicle_gps_position)) {

		/* TARGET GPS SENSOR measures [rx + bx, ry + by, rz + bz] */
		if ((_target_model == TargetModel::FullPoseDecoupled || _target_model == TargetModel::FullPoseCoupled)
		    && (((_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS) && _target_GNSS_report_sub.update(&target_GNSS_report))
			|| (_ltest_aid_mask & SensorFusionMask::USE_MISSION_POS))) {

			bool use_gps_measurements = false;

			int target_gps_lat;		// 1e-7 deg
			int target_gps_lon;		// 1e-7 deg
			float target_gps_alt;	// AMSL [mm]

			// TODO: convert .epv and .eph from accuracy to variance
			float gps_target_unc = _param_ltest_gps_t_unc.get();
			float gps_target_eph;
			float gps_target_epv;

			hrt_abstime gps_timestamp = vehicle_gps_position.timestamp_sample;

			// Measurement comes from an actual GPS on the target
			if ((_ltest_aid_mask & SensorFusionMask::USE_TARGET_GPS_POS)) {

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

			} else if (_nave_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION && _landing_pos.valid) {

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
				targetObsPos temp_obs = {};

				if (_target_model == TargetModel::FullPoseCoupled) {
					// State: [rx, ry, rz, r_x_dot, r_y_dot, r_z_dot, bx, by, bz, ... ]

					// x direction H = [1, 0, 0, 0, 0, 0, 1, 0, 0, ...]
					temp_obs.meas_h_xyz(0, 0) = 1;
					temp_obs.meas_h_xyz(0, 6) = 1;

					// y direction H = [0, 1, 0, 0, 0, 0, 0, 1, 0, ...]
					temp_obs.meas_h_xyz(1, 1) = 1;
					temp_obs.meas_h_xyz(1, 7) = 1;

					// z direction H = [0, 0, 1, 0, 0, 0, 0, 0, 1, ...]
					temp_obs.meas_h_xyz(2, 2) = 1;
					temp_obs.meas_h_xyz(2, 8) = 1;

				} else {
					// State: [r, r_dot, b, ...] --> same for x,y,z directions (decoupled)
					temp_obs.meas_h_xyz(0, 0) = 1;
					temp_obs.meas_h_xyz(0, 2) = 1;

					temp_obs.meas_h_xyz(1, 0) = 1;
					temp_obs.meas_h_xyz(1, 2) = 1;

					temp_obs.meas_h_xyz(2, 0) = 1;
					temp_obs.meas_h_xyz(2, 2) = 1;
				}

				temp_obs.timestamp = gps_timestamp;

				temp_obs.meas_xyz = gps_relative_pos;

				temp_obs.meas_unc_xyz(0) = gps_unc_horizontal;
				temp_obs.meas_unc_xyz(1) = gps_unc_horizontal;
				temp_obs.meas_unc_xyz(2) = gps_unc_vertical;

				temp_obs.any_xyz_updated = true;
				temp_obs.updated_xyz.setAll(true);

				_new_sensorReport = true;
				_target_pos_obs[target_gps_pos] = temp_obs;
			}
		}

		/* UAV GPS SENSOR [r_dotx, r_doty, r_dotz] */
		if (vehicle_gps_position.vel_ned_valid && (_target_model == TargetModel::FullPoseDecoupled
				|| _target_model == TargetModel::FullPoseCoupled)
		    && (_ltest_aid_mask & SensorFusionMask::USE_UAV_GPS_VEL)) {

			if (_target_mode == TargetMode::Stationary || (_target_mode == TargetMode::Moving
					&& _target_GNSS_report_sub.update(&target_GNSS_report))) {

				// GPS already in NED, no rotation required
				targetObsPos temp_obs = {};

				// TODO: convert .s_variance_m_s from accuracy to variance

				if (_target_mode == TargetMode::Stationary) {

					temp_obs.meas_xyz(0) = vehicle_gps_position.vel_n_m_s;
					temp_obs.meas_xyz(1) = vehicle_gps_position.vel_e_m_s;
					temp_obs.meas_xyz(2) = vehicle_gps_position.vel_d_m_s;

					temp_obs.meas_unc_xyz(0) = vehicle_gps_position.s_variance_m_s;
					temp_obs.meas_unc_xyz(1) = vehicle_gps_position.s_variance_m_s;
					temp_obs.meas_unc_xyz(2) = vehicle_gps_position.s_variance_m_s;

				} else {

					float gps_target_unc = _param_ltest_gps_t_unc.get();

					// If the target is moving, the relative velocity is expressed as the drone verlocity - the target velocity
					temp_obs.meas_xyz(0) = vehicle_gps_position.vel_n_m_s - target_GNSS_report.vx_rel;
					temp_obs.meas_xyz(1) = vehicle_gps_position.vel_e_m_s - target_GNSS_report.vy_rel;
					temp_obs.meas_xyz(2) = vehicle_gps_position.vel_d_m_s - target_GNSS_report.vz_rel;

					float unc = vehicle_gps_position.s_variance_m_s + gps_target_unc;
					temp_obs.meas_unc_xyz(0) = unc;
					temp_obs.meas_unc_xyz(1) = unc;
					temp_obs.meas_unc_xyz(2) = unc;

					// TODO: uncomment once the mavlink message is updated with covariances
					// temp_obs.meas_unc_xyz(0) = vehicle_gps_position.s_variance_m_s + target_GNSS_report.cov_vx_rel;
					// temp_obs.meas_unc_xyz(1) = vehicle_gps_position.s_variance_m_s + target_GNSS_report.cov_vy_rel;
					// temp_obs.meas_unc_xyz(2) = vehicle_gps_position.s_variance_m_s + target_GNSS_report.cov_vx_rel;
				}

				if (_target_model == TargetModel::FullPoseCoupled) {
					// State: [rx, ry, rz, rx_dot, ry_dot, rz_dot, ... ]
					temp_obs.meas_h_xyz(0, 3) = 1; // x direction
					temp_obs.meas_h_xyz(1, 4) = 1; // y direction
					temp_obs.meas_h_xyz(2, 5) = 1; // z direction

				} else {
					// State: [r, r_dot, ...] --> x, y, z directions are the same (decoupled)
					temp_obs.meas_h_xyz(0, 1) = 1;
					temp_obs.meas_h_xyz(1, 1) = 1;
					temp_obs.meas_h_xyz(2, 1) = 1;
				}

				temp_obs.timestamp = vehicle_gps_position.timestamp_sample;

				temp_obs.any_xyz_updated = true;
				temp_obs.updated_xyz.setAll(true);

				_new_sensorReport = true;
				_target_pos_obs[uav_gps_vel] = temp_obs;
			}
		}
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
	}

	switch (_target_model) {
	case TargetModel::FullPoseDecoupled:
		_nb_position_kf = 3;

		if ((_target_estimator[x] == nullptr) || (_target_estimator[y] == nullptr) || (_target_estimator[z] == nullptr)
		    || (_target_estimator[theta] == nullptr)) {
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

		if (_target_estimator[xyz] == nullptr) {
			return;
		}

		break;

	case TargetModel::NotInit:
		return;
	}

	ModuleParams::updateParams();
}

void LandingTargetEstimator::selectTargetEstimator()
{
	TargetEstimator *tmp_x = nullptr;
	TargetEstimator *tmp_y = nullptr;
	TargetEstimator *tmp_z = nullptr;
	TargetEstimator *tmp_theta = nullptr;

	bool init_failed = true;

	switch (_target_model) {
	case TargetModel::FullPoseDecoupled:

		if (_target_mode == TargetMode::Moving) {
			tmp_x = new KFxyzDecoupledMoving;
			tmp_y = new KFxyzDecoupledMoving;
			tmp_z = new KFxyzDecoupledMoving;

		} else {
			tmp_x = new KFxyzDecoupledStatic;
			tmp_y = new KFxyzDecoupledStatic;
			tmp_z = new KFxyzDecoupledStatic;
		}

		init_failed = (tmp_x == nullptr) || (tmp_y == nullptr) || (tmp_z == nullptr);

		break;

	case TargetModel::FullPoseCoupled:

		// TODO: single filter (12 states) + theta filter

		if (_target_mode == TargetMode::Moving) {
			tmp_x = new KFxyzCoupledMoving;

		} else {
			tmp_x = new KFxyzCoupledStatic;
		}

		init_failed = (tmp_x == nullptr);
		break;

	case TargetModel::Horizontal:
		// TODO: new Kalman filter
		tmp_x = new KalmanFilter();
		tmp_y = new KalmanFilter();

		init_failed = (tmp_x == nullptr) || (tmp_y == nullptr);
		break;

	case TargetModel::NotInit:
		init_failed = true;
		break;
	}

	if (_estimate_orientation) {

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
			delete _target_estimator[xyz];
			_target_estimator[xyz] = tmp_x;

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
			delete _target_estimator[theta];
			_target_estimator[theta] = tmp_theta;
		}
	}
}

} // namespace landing_target_estimator
