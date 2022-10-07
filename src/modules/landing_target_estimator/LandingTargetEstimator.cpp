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
	_check_params(true);
}

LandingTargetEstimator::~LandingTargetEstimator()
{
	delete _target_estimator[0];
	delete _target_estimator[1];
}

void LandingTargetEstimator::update()
{
	_check_params(false);

	if ((_target_estimator[0] == nullptr) || (_target_estimator[1] == nullptr)) {
		return;
	}

	_update_topics();

	/* predict */
	if (_estimator_initialized) {
		if (hrt_absolute_time() - _last_update > landing_target_estimator_TIMEOUT_US) {
			PX4_WARN("Timeout");
			_estimator_initialized = false;

		} else {
			float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC;

			// predict target position with the help of accel data
			matrix::Vector3f a{_vehicle_acceleration.xyz};

			if (_vehicleAttitude_valid && _vehicle_acceleration_valid) {
				matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
				_R_att = matrix::Dcm<float>(q_att);
				a = _R_att * a;

			} else {
				a.zero();
			}

			_target_estimator[0]->predict(dt, -a(0));
			_target_estimator[1]->predict(dt, -a(1));

			_last_predict = hrt_absolute_time();
		}
	}

	if (!_new_sensorReport) {
		// nothing to do
		return;
	}

	// mark this sensor measurement as consumed
	_new_sensorReport = false;


	if (!_estimator_initialized) {
		Vector2f v_init;
		v_init(0) = _vehicleLocalPosition.v_xy_valid ? -_vehicleLocalPosition.vx : 0.f;
		v_init(1) = _vehicleLocalPosition.v_xy_valid ? -_vehicleLocalPosition.vy : 0.f;

		Vector2f p_init(_target_position_report.rel_pos_x, _target_position_report.rel_pos_y);

		PX4_INFO("Init %.2f %.2f", (double)v_init(0), (double)v_init(1));

		for (int i = 0; i < 2; i++) {
			_target_estimator[i]->setPosition(p_init(i));
			_target_estimator[i]->setVelocity(v_init(i));
			_target_estimator[i]->setStatePosVar(_param_ltest_pos_unc_in.get());
			_target_estimator[i]->setStateVelVar(_param_ltest_vel_unc_in.get());
		}

		_estimator_initialized = true;
		_last_update = hrt_absolute_time();
		_last_predict = _last_update;

	} else {
		// update
		const float measurement_uncertainty = _param_ltest_meas_unc.get() * _dist_z * _dist_z;
		bool update_x = _target_estimator[0]->fusePosition(_target_position_report.rel_pos_x, measurement_uncertainty);
		bool update_y = _target_estimator[1]->fusePosition(_target_position_report.rel_pos_y, measurement_uncertainty);

		if (!update_x || !update_y) {
			if (!_faulty) {
				_faulty = true;
				PX4_WARN("Landing target measurement rejected:%s%s", update_x ? "" : " x", update_y ? "" : " y");
			}

		} else {
			_faulty = false;
		}

		if (!_faulty) {
			// only publish if both measurements were good

			const float x = _target_estimator[0]->getPosition();
			const float y = _target_estimator[1]->getPosition();

			_target_pose.timestamp = _target_position_report.timestamp;

			_target_pose.is_static = ((TargetMode)_param_ltest_mode.get() == TargetMode::Stationary);

			_target_pose.rel_pos_valid = true;
			_target_pose.rel_vel_valid = true;
			_target_pose.x_rel = x;
			_target_pose.y_rel = y;
			_target_pose.z_rel = _target_position_report.rel_pos_z ;
			_target_pose.vx_rel = _target_estimator[0]->getVelocity();
			_target_pose.vy_rel = _target_estimator[1]->getVelocity();

			_target_pose.cov_x_rel = _target_estimator[0]->getPosVar();
			_target_pose.cov_y_rel = _target_estimator[1]->getPosVar();

			_target_pose.cov_vx_rel = _target_estimator[0]->getVelVar();
			_target_pose.cov_vy_rel = _target_estimator[0]->getVelVar();

			if (_vehicleLocalPosition_valid && _vehicleLocalPosition.xy_valid) {
				_target_pose.x_abs = x + _vehicleLocalPosition.x;
				_target_pose.y_abs = y + _vehicleLocalPosition.y;
				_target_pose.z_abs = _target_position_report.rel_pos_z  + _vehicleLocalPosition.z;
				_target_pose.abs_pos_valid = true;

			} else {
				_target_pose.abs_pos_valid = false;
			}

			_targetPosePub.publish(_target_pose);

			_last_update = hrt_absolute_time();
			_last_predict = _last_update;
		}

		//TODO:fix
		/* float innov_x, innov_cov_x, innov_y, innov_cov_y; */
		/* _target_estimator[0]->getInnovations(innov_x, innov_cov_x); */
		/* _target_estimator[1]->getInnovations(innov_y, innov_cov_y); */

		/* _target_innovations.timestamp = _target_position_report.timestamp; */
		/* _target_innovations.innov_x = innov_x; */
		/* _target_innovations.innov_cov_x = innov_cov_x; */
		/* _target_innovations.innov_y = innov_y; */
		/* _target_innovations.innov_cov_y = innov_cov_y; */

		_targetInnovationsPub.publish(_target_innovations);
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

void LandingTargetEstimator::_update_topics()
{
	_vehicleLocalPosition_valid = _vehicleLocalPositionSub.update(&_vehicleLocalPosition);
	_vehicleAttitude_valid = _attitudeSub.update(&_vehicleAttitude);
	_vehicle_acceleration_valid = _vehicle_acceleration_sub.update(&_vehicle_acceleration);


	if (_irlockReportSub.update(&_irlockReport)) { //
		_new_irlockReport = true;

		if (!_vehicleAttitude_valid || !_vehicleLocalPosition_valid || !_vehicleLocalPosition.dist_bottom_valid) {
			// don't have the data needed for an update
			return;
		}

		if (!PX4_ISFINITE(_irlockReport.pos_y) || !PX4_ISFINITE(_irlockReport.pos_x)) {
			return;
		}

		matrix::Vector<float, 3> sensor_ray; // ray pointing towards target in body frame
		sensor_ray(0) = _irlockReport.pos_x * _param_ltest_scale_x.get(); // forward
		sensor_ray(1) = _irlockReport.pos_y * _param_ltest_scale_y.get(); // right
		sensor_ray(2) = 1.0f;

		// rotate unit ray according to sensor orientation
		_S_att = get_rot_matrix(static_cast<enum Rotation>(_param_ltest_sens_rot.get()));
		sensor_ray = _S_att * sensor_ray;

		// rotate the unit ray into the navigation frame
		matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
		_R_att = matrix::Dcm<float>(q_att);
		sensor_ray = _R_att * sensor_ray;

		if (fabsf(sensor_ray(2)) < 1e-6f) {
			// z component of measurement unsafe, don't use this measurement
			return;
		}

		_dist_z = _vehicleLocalPosition.dist_bottom - _param_ltest_sens_pos_z.get();

		// scale the ray s.t. the z component has length of _uncertainty_scale
		_target_position_report.timestamp = _irlockReport.timestamp;
		_target_position_report.rel_pos_x = sensor_ray(0) / sensor_ray(2) * _dist_z;
		_target_position_report.rel_pos_y = sensor_ray(1) / sensor_ray(2) * _dist_z;
		_target_position_report.rel_pos_z = _dist_z;

		// Adjust relative position according to sensor offset
		_target_position_report.rel_pos_x += _param_ltest_sens_pos_x.get();
		_target_position_report.rel_pos_y += _param_ltest_sens_pos_y.get();

		_new_sensorReport = true;

	} else if (_uwbDistanceSub.update(&_uwbDistance)) {
		if (!_vehicleAttitude_valid || !_vehicleLocalPosition_valid) {
			// don't have the data needed for an update
			PX4_INFO("Attitude: %d, Local pos: %d", _vehicleAttitude_valid, _vehicleLocalPosition_valid);
			return;
		}

		if (!PX4_ISFINITE((float)_uwbDistance.position[0]) || !PX4_ISFINITE((float)_uwbDistance.position[1]) ||
		    !PX4_ISFINITE((float)_uwbDistance.position[2])) {
			PX4_WARN("Position is corrupt!");
			return;
		}

		_new_sensorReport = true;

		// The coordinate system is NED (north-east-down)
		// the uwb_distance msg contains the Position in NED, Vehicle relative to LP
		// The coordinates "rel_pos_*" are the position of the landing point relative to the vehicle.
		// To change POV we negate every Axis:
		_target_position_report.timestamp = _uwbDistance.timestamp;
		_target_position_report.rel_pos_x = -_uwbDistance.position[0];
		_target_position_report.rel_pos_y = -_uwbDistance.position[1];
		_target_position_report.rel_pos_z = -_uwbDistance.position[2];
	}
}

void LandingTargetEstimator::updateParams()
{
	int32_t current_target_estimator_mode = _param_ltest_mode.get();
	ModuleParams::updateParams();

	if ((current_target_estimator_mode != _param_ltest_mode.get()) || (_target_estimator[0] == nullptr)
	    || (_target_estimator[1] == nullptr)) {
		selectTargetEstimator();
	}

	_target_estimator[0]->setInputAccVar(_param_ltest_acc_unc.get());
	_target_estimator[1]->setInputAccVar(_param_ltest_acc_unc.get());
}

void LandingTargetEstimator::selectTargetEstimator()
{
	const TargetMode target_mode = (TargetMode)_param_ltest_mode.get();

	TargetEstimator *tmp_x = nullptr;
	TargetEstimator *tmp_y = nullptr;

	switch (target_mode) {
	case TargetMode::Moving:
		/* tmp = new xxx */
		break;

	case TargetMode::Stationary:
		tmp_x = new KalmanFilter();
		tmp_y = new KalmanFilter();
		break;
	}

	if ((tmp_x == nullptr) || (tmp_y == nullptr)) {
		PX4_ERR("LTE init failed");
		_param_ltest_mode.set(0);

	} else {
		delete _target_estimator[0];
		delete _target_estimator[1];
		_target_estimator[0] = tmp_x;
		_target_estimator[1] = tmp_y;
	}
}

} // namespace landing_target_estimator
