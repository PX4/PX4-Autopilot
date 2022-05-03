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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

#include "LandingTargetEstimator.h"

#define SEC2USEC 1000000.0f

LandingTargetEstimator::LandingTargetEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_check_params(true);
}

void LandingTargetEstimator::Run()
{
	_check_params(false);

	_update_topics();

	/* predict */
	if (_estimator_initialized) {
		if (hrt_absolute_time() - _last_update > TARGET_UPDATE_TIMEOUT_US) {
			PX4_INFO("Target lost");
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

			_kalman_filter_x.predict(dt, -a(0), _param_acc_unc.get());
			_kalman_filter_y.predict(dt, -a(1), _param_acc_unc.get());

			_last_predict = hrt_absolute_time();
		}
	}

	if (!_new_sensorReport) {
		// nothing to do
		return;
	}

	// mark this sensor measurement as consumed
	_new_sensorReport = false;

	if (!PX4_ISFINITE(_irlockReport.pos_y) || !PX4_ISFINITE(_irlockReport.pos_x)) {
		return;
	}

	if (!_estimator_initialized) {
		float vx_init = _vehicleLocalPosition.v_xy_valid ? -_vehicleLocalPosition.vx : 0.f;
		float vy_init = _vehicleLocalPosition.v_xy_valid ? -_vehicleLocalPosition.vy : 0.f;
		PX4_INFO("Target acquired %.2f %.2f", (double)vx_init, (double)vy_init);
		_kalman_filter_x.init(_target_position_report.rel_pos_x, vx_init, _param_pos_unc_in.get(), _param_vel_unc_in.get());
		_kalman_filter_y.init(_target_position_report.rel_pos_y, vy_init, _param_pos_unc_in.get(), _param_vel_unc_in.get());

		_estimator_initialized = true;
		_last_update = hrt_absolute_time();
		_last_predict = _last_update;

	} else {
		const float measurement_uncertainty = _param_meas_unc.get() * _dist_z * _dist_z;
		bool update_x = _kalman_filter_x.update(_target_position_report.rel_pos_x, measurement_uncertainty);
		bool update_y = _kalman_filter_y.update(_target_position_report.rel_pos_y, measurement_uncertainty);

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

			_target_pose.timestamp = _target_position_report.timestamp;

			float x, xvel, y, yvel, covx, covx_v, covy, covy_v;
			_kalman_filter_x.getState(x, xvel);
			_kalman_filter_x.getCovariance(covx, covx_v);

			_kalman_filter_y.getState(y, yvel);
			_kalman_filter_y.getCovariance(covy, covy_v);

			_target_pose.is_static = ((TargetMode)_param_mode.get() == TargetMode::Stationary);

			_target_pose.rel_pos_valid = true;
			_target_pose.rel_vel_valid = true;
			_target_pose.x_rel = x;
			_target_pose.y_rel = y;
			_target_pose.z_rel = _target_position_report.rel_pos_z ;
			_target_pose.vx_rel = xvel;
			_target_pose.vy_rel = yvel;

			_target_pose.cov_x_rel = covx;
			_target_pose.cov_y_rel = covy;

			_target_pose.cov_vx_rel = covx_v;
			_target_pose.cov_vy_rel = covy_v;

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

		float innov_x, innov_cov_x, innov_y, innov_cov_y;
		_kalman_filter_x.getInnovations(innov_x, innov_cov_x);
		_kalman_filter_y.getInnovations(innov_y, innov_cov_y);

		_target_innovations.timestamp = _target_position_report.timestamp;
		_target_innovations.innov_x = innov_x;
		_target_innovations.innov_cov_x = innov_cov_x;
		_target_innovations.innov_y = innov_y;
		_target_innovations.innov_cov_y = innov_cov_y;

		_targetInnovationsPub.publish(_target_innovations);
	}
}

void LandingTargetEstimator::_check_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		ModuleParams::updateParams();
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
		sensor_ray(0) = _irlockReport.pos_x * _param_scale_x.get(); // forward
		sensor_ray(1) = _irlockReport.pos_y * _param_scale_y.get(); // right
		sensor_ray(2) = 1.0f;

		// rotate unit ray according to sensor orientation
		_S_att = get_rot_matrix((Rotation)_param_sens_rot.get());
		sensor_ray = _S_att * sensor_ray;

		// rotate the unit ray into the navigation frame
		matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
		_R_att = matrix::Dcm<float>(q_att);
		sensor_ray = _R_att * sensor_ray;

		if (fabsf(sensor_ray(2)) < 1e-6f) {
			// z component of measurement unsafe, don't use this measurement
			return;
		}

		_dist_z = _vehicleLocalPosition.dist_bottom - _param_sens_pos_z.get();

		// scale the ray s.t. the z component has length of _uncertainty_scale
		_target_position_report.timestamp = _irlockReport.timestamp;
		_target_position_report.rel_pos_x = sensor_ray(0) / sensor_ray(2) * _dist_z;
		_target_position_report.rel_pos_y = sensor_ray(1) / sensor_ray(2) * _dist_z;
		_target_position_report.rel_pos_z = _dist_z;

		// Adjust relative position according to sensor offset
		_target_position_report.rel_pos_x += _param_sens_pos_x.get();
		_target_position_report.rel_pos_y += _param_sens_pos_y.get();

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

int LandingTargetEstimator::start()
{
	ScheduleOnInterval(1000000 / SAMPLE_RATE);
	return PX4_OK;
}

int LandingTargetEstimator::task_spawn(int argc, char *argv[])
{
	LandingTargetEstimator *instance = new LandingTargetEstimator();

	if (!instance) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->start();
	return 0;
}

int LandingTargetEstimator::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically on the LP work queue which filters and estimates a targets position.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("landing_target_estimator", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int LandingTargetEstimator::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}

extern "C" __EXPORT int landing_target_estimator_main(int argc, char *argv[])
{
	return LandingTargetEstimator::main(argc, argv);
}
