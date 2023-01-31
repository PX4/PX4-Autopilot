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
#include <uORB/topics/parameter_update.h>

#include "LandingTargetEstimator.h"

#define SEC2USEC 1000000.0f

namespace landing_target_estimator
{

LandingTargetEstimator::LandingTargetEstimator() :
	ModuleParams(nullptr)
{
	updateParams();
}

void LandingTargetEstimator::update()
{
	_check_params(false);

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

			_kalman_filter_x.predict(dt, -a(0), _params.acc_unc);
			_kalman_filter_y.predict(dt, -a(1), _params.acc_unc);

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
		float vx_init = _vehicleLocalPosition.v_xy_valid ? -_vehicleLocalPosition.vx : 0.f;
		float vy_init = _vehicleLocalPosition.v_xy_valid ? -_vehicleLocalPosition.vy : 0.f;
		PX4_INFO("Init %.2f %.2f", (double)vx_init, (double)vy_init);
		_kalman_filter_x.init(_target_position_report.rel_pos_x, vx_init, _params.pos_unc_init, _params.vel_unc_init);
		_kalman_filter_y.init(_target_position_report.rel_pos_y, vy_init, _params.pos_unc_init, _params.vel_unc_init);

		_estimator_initialized = true;
		_last_update = hrt_absolute_time();
		_last_predict = _last_update;

	} else {
		// update
		const float measurement_uncertainty = _params.meas_unc * _dist_z * _dist_z;
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

			_target_pose.is_static = (_params.mode == TargetMode::Stationary);

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

		parameters_update();
		// _update_params();
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
		sensor_ray(0) = _irlockReport.pos_x * _params.scale_x; // forward
		sensor_ray(1) = _irlockReport.pos_y * _params.scale_y; // right
		sensor_ray(2) = 1.0f;

		// rotate unit ray according to sensor orientation
		_S_att = get_rot_matrix(_params.sensor_yaw);
		sensor_ray = _S_att * sensor_ray;

		// rotate the unit ray into the navigation frame
		matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
		_R_att = matrix::Dcm<float>(q_att);
		sensor_ray = _R_att * sensor_ray;

		if (fabsf(sensor_ray(2)) < 1e-6f) {
			// z component of measurement unsafe, don't use this measurement
			return;
		}

		_dist_z = _vehicleLocalPosition.dist_bottom - _params.offset_z;

		// scale the ray s.t. the z component has length of _uncertainty_scale
		_target_position_report.timestamp = _irlockReport.timestamp;
		_target_position_report.rel_pos_x = sensor_ray(0) / sensor_ray(2) * _dist_z;
		_target_position_report.rel_pos_y = sensor_ray(1) / sensor_ray(2) * _dist_z;
		_target_position_report.rel_pos_z = _dist_z;

		// Adjust relative position according to sensor offset
		_target_position_report.rel_pos_x += _params.offset_x;
		_target_position_report.rel_pos_y += _params.offset_y;

		_new_sensorReport = true;

	} else if (_sensorUwbSub.update(&_sensorUwb)) {
		if (!_vehicleAttitude_valid || !_vehicleLocalPosition_valid) {
			// don't have the data needed for an update
			PX4_INFO("Attitude: %d, Local pos: %d", _vehicleAttitude_valid, _vehicleLocalPosition_valid);
			return;
		}

		if (!matrix::Vector3f(_sensorUwb.position).isAllFinite()) {
			PX4_WARN("Position is corrupt!");
			return;
		}

		_new_sensorReport = true;

		// The coordinate system is NED (north-east-down)
		// the sensor_uwb msg contains the Position in NED, Vehicle relative to LP
		// The coordinates "rel_pos_*" are the position of the landing point relative to the vehicle.
		// To change POV we negate every Axis:
		_target_position_report.timestamp = _sensorUwb.timestamp;
		_target_position_report.rel_pos_x = -_sensorUwb.position[0];
		_target_position_report.rel_pos_y = -_sensorUwb.position[1];
		_target_position_report.rel_pos_z = -_sensorUwb.position[2];
	}
}

void LandingTargetEstimator::_update_params()
{
	param_get(_paramHandle.acc_unc, &_params.acc_unc);
	param_get(_paramHandle.meas_unc, &_params.meas_unc);
	param_get(_paramHandle.pos_unc_init, &_params.pos_unc_init);
	param_get(_paramHandle.vel_unc_init, &_params.vel_unc_init);

	int32_t mode = 0;
	param_get(_paramHandle.mode, &mode);
	_params.mode = (TargetMode)mode;

	param_get(_paramHandle.scale_x, &_params.scale_x);
	param_get(_paramHandle.scale_y, &_params.scale_y);

	int32_t sensor_yaw = 0;
	param_get(_paramHandle.sensor_yaw, &sensor_yaw);
	_params.sensor_yaw = static_cast<enum Rotation>(sensor_yaw);

	param_get(_paramHandle.offset_x, &_params.offset_x);
	param_get(_paramHandle.offset_y, &_params.offset_y);
	param_get(_paramHandle.offset_z, &_params.offset_z);
}

// math needs to be fixed
matrix::Vector3d LandingTargetEstimator::UWB_SR150_localization(double distance, double azimuth_dev, double elevation_dev)
{
	/* UWB_SR150::localization takes distance and angle measurements and publishes position data.
	can be used to make a rover follow an UWB receiver
	*/
	double deg2rad = M_PI / 180.0;
	// Catch angle measurements at the end of the range and discard them
	/*
	if(60.0 > azimuth_dev  || -60.0 < azimuth_dev){
		return;
	}
	if(60.0 > elevation_dev  || -60.0 < elevation_dev){
		return;
	}*/


	double azimuth 	 = azimuth_dev * deg2rad; 	//subtract yaw offset and convert to rad
	double elevation = elevation_dev  * deg2rad; 	//subtract pitch offset and convert to rad

	matrix::Vector3d position(	sin(azimuth) * sin(elevation),
					cos(azimuth) * sin(elevation),
					-cos(elevation));

	position *= distance; //scale the vector to the distance
	//Output is the Coordinates of the Initiator in relation to the UWB Receiver in NED (North-East-Down) Framing

	// Now the position is the landing point relative to the vehicle.
	// so the only thing left is to add the Initiator offset
	position +=  matrix::Vector3d(_uwb_init_offset);

	return position;
}

//TODO: Find out if this works
void LandingTargetEstimator::uwb_sr150_prec_nav()
{ //Precision landing mode
				// _sensor_uwb.status = 10;
				_rel_pos = LandingTargetEstimator::UWB_SR150_localization(_sensor_uwb.distance, _sensor_uwb.aoa_azimuth_dev,
								   _sensor_uwb.aoa_elevation_dev);
				_sensor_uwb.position[0] = _rel_pos(0);
				_sensor_uwb.position[1] = _rel_pos(1);
				_sensor_uwb.position[2] = _rel_pos(2);
}

/* void LandingTargetEstimator::uwb_sr150_followme()
{ // Follow me mode
			// _sensor_uwb.status = 11;
			actuator_control(_sensor_uwb.distance, _sensor_uwb.aoa_azimuth_dev,
						_sensor_uwb.aoa_elevation_dev);
}

void LandingTargetEstimator::actuator_control(double distance, double azimuth, double elevation)
{
	/ * UWB_SR150::actuator_control takes distance and angle measurements and publishes proportional thrust commands.
	can be used to make a rover follow an UWB receiver
	* /

	//Lots of Params to finetune the Follow me behavior
	double follow_distance_max =	_uwb_follow_distance_max.get();
	double follow_distance = 	_uwb_follow_distance.get();
	double follow_distance_min =	_uwb_follow_distance_min.get();
	double throttle_max =		_uwb_throttle.get();
	double thrust_heading =		_uwb_thrust_head.get(); // heading thrust multiplier
	double thrust_heading_min =	_uwb_thrust_head_min.get();
	double thrust_heading_max =	_uwb_thrust_head_max.get();
	double throttle_reverse =	_uwb_throttle_reverse.get();

	double heading = azimuth / 70; //normalize the AoA to -1..+1
	double throttle = 0;

	//simple heading control loop
	if (azimuth >= 0) { // Heading deadzone
		heading = heading * thrust_heading;
		heading = math::constrain(heading, thrust_heading_min, thrust_heading_max) ; //Limit heading to -1..+1

	} else if (azimuth <= 0) { // Heading deadzone
		heading = heading * thrust_heading;
		heading = math::constrain(heading, -thrust_heading_max, -thrust_heading_min) ;

	} else {
		heading = 0;
	}

	//simple Throttle control loop
	if (distance > follow_distance_max) {
		throttle = 0;

	} else if (distance <= follow_distance_min) {
		throttle =	throttle_reverse;

	} else if ((distance >= follow_distance_min) && (distance < follow_distance)) {
		throttle = 0;

	} else if ((distance >= follow_distance) && (distance < follow_distance_max)) {
		throttle = (distance - follow_distance) / 2; //Throttle will reach 100% over 2 meters

	}

	throttle = math::constrain(throttle, -throttle_max, throttle_max); //limit throttle to -1..1

	offboard_control_mode_s offboard_control_mode{}; //publish offboard control mode to enable it
	offboard_control_mode.timestamp = hrt_absolute_time();
	offboard_control_mode.actuator = true;
	_offboard_control_mode_pub.publish(offboard_control_mode);

	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);

	// Publish actuator controls only once in OFFBOARD
	if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		//TODO query frame type and disable if not rover

		/ *
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};
		vehicle_rates_setpoint.timestamp = hrt_absolute_time();
		vehicle_rates_setpoint.roll = heading;
		vehicle_rates_setpoint.pitch = heading;
		vehicle_rates_setpoint.yaw = heading;
		vehicle_rates_setpoint.thrust_body[0] = throttle * max_throttle * timeout;
		vehicle_rates_setpoint.thrust_body[1] = throttle * max_throttle * timeout;
		vehicle_rates_setpoint.thrust_body[2] = throttle * max_throttle * timeout;
		_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
		* /

		actuator_controls_s actuator_controls{};
		actuator_controls.timestamp = hrt_absolute_time();
		actuator_controls.control[2] = heading; //yaw
		actuator_controls.control[3] = throttle; //Thrust
		_actuator_controls_pubs[0].publish(actuator_controls); //flight controls
	}

} */

void LandingTargetEstimator::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}

} // namespace landing_target_estimator
