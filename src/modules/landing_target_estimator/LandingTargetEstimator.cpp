/****************************************************************************
 *
 *   Copyright (c) 2018-2022 PX4 Development Team. All rights reserved.
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

#include "LandingTargetEstimator.hpp"

LandingTargetEstimator::LandingTargetEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

LandingTargetEstimator::~LandingTargetEstimator()
{
	perf_free(_cycle_perf);
}

bool LandingTargetEstimator::init()
{
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void LandingTargetEstimator::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// backup schedule
	ScheduleDelayed(100_ms);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		ModuleParams::updateParams();
	}

	perf_begin(_cycle_perf);

	// predict
	vehicle_local_position_s vehicle_local_position;

	if (_vehicle_local_position_sub.update(&vehicle_local_position)) {
		if (_estimator_initialized) {
			const float dt = math::constrain((vehicle_local_position.timestamp_sample - _last_predict) * 1e-6f, 0.001f, 0.1f);

			// predict target position with the help of accel data
			_kalman_filter_x.predict(dt, -vehicle_local_position.ax, _param_ltest_acc_unc.get());
			_kalman_filter_y.predict(dt, -vehicle_local_position.ay, _param_ltest_acc_unc.get());

			_last_predict = vehicle_local_position.timestamp_sample;
		}
	}

	landing_target_pose_s landing_target_pose{};

	bool update_x = false;
	bool update_y = false;

	if (_irlock_report_sub.advertised()) {
		// using irlock_report
		irlock_report_s irlock_report;

		if (_irlock_report_sub.update(&irlock_report)) {
			vehicle_attitude_s vehicle_attitude;

			if (!vehicle_local_position.v_xy_valid || !vehicle_local_position.dist_bottom_valid
			    || !_vehicle_attitude_sub.update(&vehicle_attitude)
			    || !PX4_ISFINITE(irlock_report.pos_y) || !PX4_ISFINITE(irlock_report.pos_x)) {
				// don't have the data needed for an update
				return;
			}

			matrix::Vector3f sensor_ray; // ray pointing towards target in body frame
			sensor_ray(0) = irlock_report.pos_x * _param_ltest_scale_x.get(); // forward
			sensor_ray(1) = irlock_report.pos_y * _param_ltest_scale_y.get(); // right
			sensor_ray(2) = 1.f;

			// Orientation of the sensor relative to body frame
			const matrix::Dcmf S_att = get_rot_matrix((Rotation)_param_ltest_sens_rot.get());
			// rotate unit ray according to sensor orientation
			sensor_ray = S_att * sensor_ray;

			// rotate the unit ray into the navigation frame
			const matrix::Quatf q_att(vehicle_attitude.q);
			const matrix::Dcmf R_att(q_att); // Orientation of the body frame
			sensor_ray = R_att * sensor_ray;

			if (fabsf(sensor_ray(2)) < 1e-6f) {
				// z component of measurement unsafe, don't use this measurement
				return;
			}

			matrix::Vector3f rel_pos{};

			const float dist_z = vehicle_local_position.dist_bottom - _param_ltest_sens_pos_z.get();

			// scale the ray s.t. the z component has length of _uncertainty_scale
			//  - adjust relative position according to sensor offset
			rel_pos(0) = (sensor_ray(0) / sensor_ray(2) * dist_z) + _param_ltest_sens_pos_x.get();
			rel_pos(1) = (sensor_ray(1) / sensor_ray(2) * dist_z) + _param_ltest_sens_pos_x.get();
			rel_pos(2) = dist_z;

			if (!_estimator_initialized) {
				float vx_init = -vehicle_local_position.vx;
				float vy_init = -vehicle_local_position.vy;

				PX4_INFO("IRLock target acquired %.2f %.2f", (double)vx_init, (double)vy_init);

				_kalman_filter_x.init(rel_pos(0), vx_init, _param_ltest_pos_unc_in.get(), _param_ltest_vel_unc_in.get());
				_kalman_filter_y.init(rel_pos(1), vy_init,  _param_ltest_pos_unc_in.get(), _param_ltest_vel_unc_in.get());

				_estimator_initialized = true;
				_last_predict = vehicle_local_position.timestamp_sample;
				_last_update = vehicle_local_position.timestamp_sample;

			} else {
				const float measurement_uncertainty = _param_ltest_meas_unc.get() * dist_z * dist_z;

				update_x = _kalman_filter_x.update(rel_pos(0), measurement_uncertainty);
				update_y = _kalman_filter_y.update(rel_pos(1), measurement_uncertainty);

				if (!update_x || !update_y) {
					PX4_DEBUG("IRLock distance measurement rejected (%.3f, %.3f)", (double)rel_pos(0), (double)rel_pos(1));
				}

				landing_target_pose.timestamp_sample = irlock_report.timestamp;
				landing_target_pose.z_rel = dist_z;
			}
		}

	} else if (_uwb_distance_sub.advertised()) {
		uwb_distance_s uwb_distance;

		if (_uwb_distance_sub.update(&uwb_distance)) {

			if (!vehicle_local_position.v_xy_valid) {
				// don't have the data needed for an update
				return;
			}

			if (!PX4_ISFINITE((float)uwb_distance.position[0])
			    || !PX4_ISFINITE((float)uwb_distance.position[1])
			    || !PX4_ISFINITE((float)uwb_distance.position[2])) {

				PX4_WARN("uwb_distance position is corrupt!");
				return;
			}

			const matrix::Vector3f position(-uwb_distance.position[0], -uwb_distance.position[1], -uwb_distance.position[2]);

			// The coordinate system is NED (north-east-down)
			// the uwb_distance msg contains the Position in NED, Vehicle relative to LP
			// The coordinates "rel_pos_*" are the position of the landing point relative to the vehicle.
			// To change POV we negate every Axis:
			if (!_estimator_initialized) {
				float vx_init = -vehicle_local_position.vx;
				float vy_init = -vehicle_local_position.vy;

				PX4_INFO("UWB distance target acquired %.2f %.2f", (double)vx_init, (double)vy_init);

				_kalman_filter_x.init(position(0), vx_init,
						      _param_ltest_pos_unc_in.get(), _param_ltest_vel_unc_in.get());

				_kalman_filter_y.init(position(1), vy_init,
						      _param_ltest_pos_unc_in.get(), _param_ltest_vel_unc_in.get());

				_estimator_initialized = true;
				_last_predict = vehicle_local_position.timestamp_sample;
				_last_update = vehicle_local_position.timestamp_sample;

			} else {

				float measurement_uncertainty = _param_ltest_meas_unc.get();

				update_x = _kalman_filter_x.update(position(0), measurement_uncertainty);
				update_y = _kalman_filter_y.update(position(1), measurement_uncertainty);

				if (!update_x || !update_y) {
					PX4_DEBUG("UWB distance measurement rejected (%.3f, %.3f)", (double)position(1), (double)position(1));
				}

				landing_target_pose.timestamp_sample = uwb_distance.timestamp;
				landing_target_pose.z_rel = uwb_distance.position[2];
			}
		}
	}

	if (vehicle_local_position.timestamp_sample > _last_update + TARGET_UPDATE_TIMEOUT_US) {
		if (_estimator_initialized) {
			PX4_INFO("Target lost");
			_estimator_initialized = false;
		}
	}

	if (_estimator_initialized) {
		// publish landing_target_pose
		if (update_x && update_y) {
			_last_update = vehicle_local_position.timestamp_sample;

			// only publish if both measurements were good
			landing_target_pose.is_static = ((TargetMode)_param_ltest_mode.get() == TargetMode::Stationary);

			landing_target_pose.rel_pos_valid = true;
			landing_target_pose.rel_vel_valid = true;

			landing_target_pose.x_rel      = _kalman_filter_x.getState()(0);
			landing_target_pose.vx_rel     = _kalman_filter_x.getState()(1);
			landing_target_pose.cov_x_rel  = _kalman_filter_x.getCovariance()(0, 0);
			landing_target_pose.cov_vx_rel = _kalman_filter_x.getCovariance()(1, 1);

			landing_target_pose.y_rel      = _kalman_filter_y.getState()(0);
			landing_target_pose.vy_rel     = _kalman_filter_y.getState()(1);
			landing_target_pose.cov_y_rel  = _kalman_filter_y.getCovariance()(0, 0);
			landing_target_pose.cov_vy_rel = _kalman_filter_y.getCovariance()(1, 1);

			if (vehicle_local_position.xy_valid) {
				landing_target_pose.x_abs = landing_target_pose.x_rel + vehicle_local_position.x;
				landing_target_pose.y_abs = landing_target_pose.y_rel + vehicle_local_position.y;
				landing_target_pose.z_abs = landing_target_pose.z_rel + vehicle_local_position.z;
				landing_target_pose.abs_pos_valid = true;

			} else {
				landing_target_pose.x_abs = NAN;
				landing_target_pose.y_abs = NAN;
				landing_target_pose.z_abs = NAN;
				landing_target_pose.abs_pos_valid = false;
			}

			landing_target_pose.timestamp = hrt_absolute_time();
			_landing_target_pose_pub.publish(landing_target_pose);
		}

		// publish landing_target_innovations
		if (landing_target_pose.timestamp_sample != 0) {

			landing_target_innovations_s landing_target_innovations{};
			_kalman_filter_x.getInnovations(landing_target_innovations.innov_x, landing_target_innovations.innov_cov_x);
			_kalman_filter_y.getInnovations(landing_target_innovations.innov_y, landing_target_innovations.innov_cov_y);

			landing_target_innovations.timestamp_sample = landing_target_pose.timestamp_sample;
			landing_target_innovations.timestamp = hrt_absolute_time();
			_landing_target_innovations_pub.publish(landing_target_innovations);
		}
	}

	perf_end(_cycle_perf);
}

int LandingTargetEstimator::task_spawn(int argc, char *argv[])
{
	LandingTargetEstimator *instance = new LandingTargetEstimator();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int LandingTargetEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LandingTargetEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Target position estimator using either IRLock or UWB (ultra wide band) data.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("landing_target_estimator", "estimator");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int landing_target_estimator_main(int argc, char *argv[])
{
	return LandingTargetEstimator::main(argc, argv);
}
