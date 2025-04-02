/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include "MovingPlatformController.hpp"

using namespace custom;

// Register the plugin
GZ_ADD_PLUGIN(
	MovingPlatformController,
	gz::sim::System,
	gz::sim::ISystemPreUpdate,
	gz::sim::ISystemConfigure
)

void MovingPlatformController::Configure(const gz::sim::Entity &entity,
		const std::shared_ptr<const sdf::Element> &sdf,
		gz::sim::EntityComponentManager &ecm,
		gz::sim::EventManager &eventMgr)
{
	_entity = entity;
	_model = gz::sim::Model(entity);

	const std::string link_name = sdf->Get<std::string>("link_name");
	_link_entity = _model.LinkByName(ecm, link_name);

	if (!_link_entity) {
		throw std::runtime_error("MovingPlatformController::Configure: Link \"" + link_name + "\" was not found. "
			"Please ensure that your model contains the corresponding link.");
	}

	_link = gz::sim::Link(_link_entity);

	// Needed to report linear & angular velocity
	_link.EnableVelocityChecks(ecm, true);

	// Get velocity and orientation setpoints from env vars
	{
		const double vel_forward = ReadEnvVar("PX4_GZ_PLATFORM_VEL", 1.0);
		const double heading_deg = ReadEnvVar("PX4_GZ_PLATFORM_HEADING_DEG", 0.0);
		const double heading = GZ_DTOR(heading_deg);

		_orientation_sp = gz::math::Quaterniond(0., 0., heading);

		// Set initial orientation to match the heading
		const auto optional_original_pose = _link.WorldPose(ecm);
		if (optional_original_pose.has_value()) {
			const gz::math::Pose3d original_pose = optional_original_pose.value();
			const gz::math::Pose3d new_pose(original_pose.Pos(), _orientation_sp);
			_model.SetWorldPoseCmd(ecm, new_pose);

		} else {
			gzwarn << "Unable to get initial platform pose. Heading will not be set" << std::endl;
		}

		// Velocity setpoint in world frame.
		const gz::math::Vector3d _body_velocity_sp(vel_forward, 0., 0.);
		_velocity_sp = _orientation_sp * _body_velocity_sp;
	}

	// Get gravity, model mass, platform height.
	{
		const auto world_entity = gz::sim::worldEntity(ecm);
		const auto world = gz::sim::World(world_entity);
		const auto gravity = world.Gravity(ecm);

		if (gravity.has_value()) {
			_gravity = world.Gravity(ecm).value().Z();
		} else {
			gzwarn << "Unable to get gazebo world gravity. Keeping default of " << _gravity << std::endl;
		}

		auto inertial_component = ecm.Component<gz::sim::components::Inertial>(_link_entity);

		if (inertial_component) {
			_platform_mass = inertial_component->Data().MassMatrix().Mass();
		} else {
			gzwarn << "Unable to get inertial component for link " << link_name << ". Keeping default mass of " << _platform_mass << std::endl;
		}

		auto pose_component = ecm.Component<gz::sim::components::Pose>(_link_entity);

		if (pose_component) {
			_platform_height_setpoint = pose_component->Data().Z();
		} else {
			gzwarn << "Unable to get pose component for link " << link_name << ". Keeping default platform height setpoint of " << _platform_height_setpoint << std::endl;
		}
	}

}

void MovingPlatformController::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &ecm)
{
	updatePlatformState(ecm);
	updateWrenchCommand(_velocity_sp, _orientation_sp);
	sendWrenchCommand(ecm);
}

void MovingPlatformController::updateWrenchCommand(
	const gz::math::Vector3d &velocity_setpoint,
	const gz::math::Quaterniond &orientation_setpoint)
{
	// (force, torque) =
	//    constant normal force / buoyancy term offsetting gravity
	//  + low-pass filtered white noise representing waves, road noise, etc.
	//  + PD feedback term representing stable vehicle design

	gz::math::Vector3d noise_force = gz::math::Vector3d(
			gz::math::Rand::DblNormal(),
			gz::math::Rand::DblNormal(),
			gz::math::Rand::DblNormal()
					  );

	gz::math::Vector3d noise_torque = gz::math::Vector3d(
			gz::math::Rand::DblNormal(),
			gz::math::Rand::DblNormal(),
			gz::math::Rand::DblNormal()
					  );

	// Filter coefficients for low-pass filtering the white noise.
	// larger number = faster movement
	// should be between 0 and 1
	const double filter_coef_force = 0.1;
	const double filter_coef_torque = 0.1;

	// Noise amplitude.
	// larger number = bigger movement
	// should be >= 0
	const double noise_ampl_force = 1. * _platform_mass;
	const double noise_ampl_torque = 1. * _platform_mass;

	_noise_lowpass_force = (1 - filter_coef_force) * _noise_lowpass_force + filter_coef_force * noise_force;
	_noise_lowpass_torque = (1 - filter_coef_torque) * _noise_lowpass_torque + filter_coef_torque * noise_torque;

	const gz::math::Vector3d normal_force(0., 0., -_gravity * _platform_mass);

	_force = noise_ampl_force * _noise_lowpass_force + normal_force;
	_torque = noise_ampl_torque * _noise_lowpass_torque;

	// Feedback terms to ensure stability of the platform.
	{

		const gz::math::Vector3d pos_gains = _platform_mass * gz::math::Vector3d(0., 0., 1.);
		const gz::math::Vector3d vel_gains = _platform_mass * gz::math::Vector3d(1., 1., 1.);

		const gz::math::Vector3d platform_pos_error = (_platform_position - gz::math::Vector3d(0., 0., _platform_height_setpoint));
		const gz::math::Vector3d platform_vel_error = (_platform_velocity - velocity_setpoint);

		// * is element-wise
		const gz::math::Vector3d feedback_force = -pos_gains * platform_pos_error - vel_gains * platform_vel_error;

		// Clip horizontal force to avoid large accelerations, which might cause the drone to slip off the platform.
		const float max_accel = 2.; // [m/s^2]
		const gz::math::Vector2d _force_xy = gz::math::Vector2d(feedback_force.X(), feedback_force.Y());
		const float accel_xy = _force_xy.Length() / _platform_mass;

		if (accel_xy > max_accel) {
			const float scaling = max_accel / accel_xy;
			_force += feedback_force * gz::math::Vector3d(scaling, scaling, 1.);
		} else {
			_force += feedback_force;
		}

		// Attitude - similar but accounting for quaternion weirdness.
		// Combining ideas from:
		//  - Eq. 23 in Nonlinear Quadrocopter Attitude Control (Brescianini, Hehn, D'Andrea)
		//    https://www.research-collection.ethz.ch/handle/20.500.11850/154099
		//  - Eq. 20 in Full Quaternion Based Attitude Control for a Quadrotor (Fresk, Nikolakopoulos)
		//    https://www.diva-portal.org/smash/get/diva2:1010947/FULLTEXT01.pdf

		const gz::math::Quaterniond attitude_err = _platform_orientation * orientation_setpoint.Inverse();

		const double attitude_p_gain = 1. * _platform_mass;
		const double attitude_d_gain = 1. * _platform_mass;

		const double sgn = attitude_err.W() > 0. ? 1. : -1.;
		gz::math::Vector3d attitude_err_imag = sgn * gz::math::Vector3d(attitude_err.X(), attitude_err.Y(), attitude_err.Z());

		_torque += -attitude_p_gain * attitude_err_imag - attitude_d_gain * _platform_angular_velocity;
	}
}


void MovingPlatformController::updatePlatformState(const gz::sim::EntityComponentManager &ecm)
{

	const auto optional_pose = _link.WorldPose(ecm);

	if (optional_pose.has_value()) {
		_platform_position = optional_pose.value().Pos();
		_platform_orientation = optional_pose.value().Rot();

	} else {
		gzerr << "Unable to get pose" << std::endl;
	}


	const auto optional_vel = _link.WorldLinearVelocity(ecm);

	if (optional_vel.has_value()) {
		_platform_velocity = optional_vel.value();

	} else {
		gzerr << "Unable to get linear velocity" << std::endl;
	}


	const auto optional_angular_vel = _link.WorldAngularVelocity(ecm);

	if (optional_angular_vel.has_value()) {
		_platform_angular_velocity = optional_angular_vel.value();

	} else {
		gzerr << "Unable to get angular velocity" << std::endl;
	}

}

void MovingPlatformController::sendWrenchCommand(gz::sim::EntityComponentManager &ecm)
{
	_link.AddWorldWrench(ecm, _force, _torque);
}


double MovingPlatformController::ReadEnvVar(const char* env_var_name, double default_value) {

	const char *env_var_value = std::getenv(env_var_name);

	if (env_var_value) {
		try {
			double value = std::stod(env_var_value);
			return value;

		} catch (const std::invalid_argument &e) {
			// These warnings will only be visible with sufficient verbosity level...
			gzwarn << "Invalid argument: " << env_var_value << " is not a valid double for " << env_var_name << std::endl;
			gzwarn << "Keeping default value of " << default_value << " m/s." << std::endl;

		} catch (const std::out_of_range &e) {
			gzwarn << "Out of range: " << env_var_value << " is out of range for " << env_var_name << std::endl;
			gzwarn << "Keeping default value of " << default_value << " m/s." << std::endl;
		}
	}

	return default_value;
}
