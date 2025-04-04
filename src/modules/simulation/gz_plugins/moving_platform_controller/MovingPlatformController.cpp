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

	_world_entity = gz::sim::worldEntity(ecm);
	_world = gz::sim::World(_world_entity);

	getVehicleModelName();

	_startup_timer = gz::common::Timer();
	_startup_timer.Start();

	// Get velocity and orientation setpoints from env vars
	{
		const double vel_forward = readEnvVar("PX4_GZ_PLATFORM_VEL", 1.0);
		const double heading_deg = readEnvVar("PX4_GZ_PLATFORM_HEADING_DEG", 0.0);
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
		const auto gravity = _world.Gravity(ecm);

		if (gravity.has_value()) {
			_gravity = gravity.value().Z();

		} else {
			gzwarn << "Unable to get gazebo world gravity. Keeping default of " << _gravity << std::endl;
		}

		auto inertial_component = ecm.Component<gz::sim::components::Inertial>(_link_entity);

		if (inertial_component) {
			_platform_mass = inertial_component->Data().MassMatrix().Mass();
			_platform_diag_moments = inertial_component->Data().MassMatrix().DiagonalMoments();

		} else {
			gzwarn << "Unable to get inertial component for link " << link_name << ". Keeping default mass of " << _platform_mass <<
			       std::endl;
		}

		auto pose_component = ecm.Component<gz::sim::components::Pose>(_link_entity);

		if (pose_component) {
			_platform_height_setpoint = pose_component->Data().Z();

		} else {
			gzwarn << "Unable to get pose component for link " << link_name << ". Keeping default platform height setpoint of " <<
			       _platform_height_setpoint << std::endl;
		}
	}

}

void MovingPlatformController::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &ecm)
{
	getPlatformState(ecm);

	const double dt_sec = std::chrono::duration<double>(_info.dt).count();
	updateNoise(dt_sec);

	// Keep stationary if model is not yet spawned (and model name valid).
	// If model name invalid, then we don't know what to wait for and move it immediately.
	const bool vehicle_has_spawned = 0 != _world.ModelByName(ecm, _vehicle_model_name);
	const bool keep_stationary = _wait_for_vehicle_spawned && !vehicle_has_spawned;

	updateWrenchCommand(_velocity_sp, _orientation_sp, keep_stationary);

	sendWrenchCommand(ecm);
}

void MovingPlatformController::updateNoise(const double dt)
{
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

	// Cutoff frequencies for low-pass filtering the white noise.
	// larger number = faster movement
	// Should be between 0 and sample_freq / 2, with sample_freq = 1/dt
	const double cutoff_freq_force  = 5.;  // 1/s
	const double cutoff_freq_torque = 5.;  // 1/s

	const double max_cutoff_freq = 0.5 / dt;

	if (cutoff_freq_force < 0 || cutoff_freq_force > max_cutoff_freq ||
	    cutoff_freq_torque < 0 || cutoff_freq_torque > max_cutoff_freq) {
		throw std::runtime_error("MovingPlatformController::updateWrenchCommand: invalid cutoff frequency");
	}

	// Cutoff frequency -> filter coefficient calculation from PX4's AlphaFilter.hpp
	// from AlphaFilter::setCutoffFreq
	const double time_constant_force = 1. / (2 * GZ_PI * cutoff_freq_force);
	const double time_constant_torque = 1. / (2 * GZ_PI * cutoff_freq_torque);

	// from AlphaFilter::setParameters
	const double filter_coef_force = dt / (dt + time_constant_force);
	const double filter_coef_torque = dt / (dt + time_constant_torque);

	// This noise is dimensionless, units only come into play in updateWrenchCommand
	_noise_lowpass_force = (1. - filter_coef_force) * _noise_lowpass_force + filter_coef_force * noise_force;
	_noise_lowpass_torque = (1. - filter_coef_torque) * _noise_lowpass_torque + filter_coef_torque * noise_torque;
}

void MovingPlatformController::updateWrenchCommand(
	const gz::math::Vector3d &velocity_setpoint,
	const gz::math::Quaterniond &orientation_setpoint,
	const bool keep_stationary)
{
	// (force, torque) =
	//    constant normal force / buoyancy term offsetting gravity
	//  + low-pass filtered white noise representing waves, road noise, etc.
	//  + PD feedback term representing stable vehicle design

	// If keep_stationary, override noise amplitude and velocity setpoint to zero.
	// This is to wait for the model to spawn initially.

	// Noise amplitude >= 0
	// larger number = bigger movement
	const double noise_ampl_force_scaling = keep_stationary ? 0. : 1.;          // [N / kg]
	const double noise_ampl_force = noise_ampl_force_scaling * _platform_mass;  // [N]

	const double noise_ampl_torque_scaling = keep_stationary ? 0. : 1.;         // [N m / (kg m^2) = 1/s^2]
	const gz::math::Vector3d noise_ampl_torque = noise_ampl_torque_scaling * _platform_diag_moments; // [N m]

	const gz::math::Vector3d normal_force(0., 0., -_gravity * _platform_mass);  // [N]

	_force = noise_ampl_force * _noise_lowpass_force + normal_force;            // [N]
	_torque = noise_ampl_torque * _noise_lowpass_torque;                        // [N m]

	// Feedback terms to ensure stability of the platform.
	{

		// Position - simple PD controller, but with zero position gains in xy direction.
		// With the Vector3d on the RHS having units of 1/s^2 and 1/s, respectively
		const gz::math::Vector3d pos_gains = _platform_mass * gz::math::Vector3d(0., 0., 1.); // [N / m]
		const gz::math::Vector3d vel_gains = _platform_mass * gz::math::Vector3d(1., 1., 1.); // [N / (m/s)]

		const gz::math::Vector3d platform_position_setpoint(0., 0., _platform_height_setpoint);
		const gz::math::Vector3d current_velocity_setpoint = keep_stationary ? gz::math::Vector3d::Zero : velocity_setpoint;

		const gz::math::Vector3d platform_pos_error = (_platform_position - platform_position_setpoint);
		const gz::math::Vector3d platform_vel_error = (_platform_velocity - current_velocity_setpoint);

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

		// With the factors of 1. having units of 1 / (m rad) and s / (m rad), respectively
		const gz::math::Vector3d attitude_p_gain = 1. * _platform_diag_moments; // [N m / rad]
		const gz::math::Vector3d attitude_d_gain = 1. * _platform_diag_moments; // [N m / (rad/s)]

		const double sgn = attitude_err.W() > 0. ? 1. : -1.;
		gz::math::Vector3d attitude_err_imag = sgn * gz::math::Vector3d(attitude_err.X(), attitude_err.Y(), attitude_err.Z());

		// Factor of 2 to convert quaternion error to rad
		_torque += -2. * attitude_p_gain * attitude_err_imag - attitude_d_gain * _platform_angular_velocity;
	}
}


void MovingPlatformController::getPlatformState(const gz::sim::EntityComponentManager &ecm)
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


double MovingPlatformController::readEnvVar(const char *env_var_name, double default_value)
{

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

void MovingPlatformController::getVehicleModelName()
{
	// Find the name of the gazebo vehicle model.
	// The name is constructed in px4-rc.gzsim as:
	//     MODEL_NAME="${PX4_SIM_MODEL#*gz_}"
	//     MODEL_NAME_INSTANCE="${MODEL_NAME}_${px4_instance}"
	// So here we replicate that.

	const char *px4_sim_model_cstr = std::getenv("PX4_SIM_MODEL");
	std::string px4_sim_model = "";

	const char *px4_gz_model_name_cstr = std::getenv("PX4_GZ_MODEL_NAME");

	if (px4_sim_model_cstr != nullptr) {

		px4_sim_model = px4_sim_model_cstr;

	} else if (px4_gz_model_name_cstr != nullptr) {

		// This happens if we attach to an existing model. In this case,
		// do not wait for any vehicle to spawn.

		gzwarn << "PX4_SIM_MODEL not set. Proceeding without vehicle name and moving platform immediately." << std::endl;
		_wait_for_vehicle_spawned = false;

	} else {

		// If neither are set, the px4-rc.gzsim script should have
		// exited 1. We could land here if these environment variables
		// change -- if so, update this function accordingly.

		gzerr << "Neither PX4_MODEL nor PX4_GZ_MODEL_NAME are set. One needed to proceed." << std::endl;
		_wait_for_vehicle_spawned = false;
	}

	// Remove leading "gz_"
	const std::string prefix_to_remove = "gz_";
	size_t pos = px4_sim_model.find(prefix_to_remove);

	if (pos != std::string::npos) {
		px4_sim_model = px4_sim_model.substr(pos + prefix_to_remove.length());

	} else {
		gzwarn << "Error: \"gz_\" not found in PX4_SIM_MODEL. Using the entire string as MODEL_NAME." << std::endl;
	}

	// Get the px4_instance environment variable
	const char *px4_instance_cstr = std::getenv("px4_instance");
	std::string px4_instance = "";

	if (px4_instance_cstr != nullptr) {
		px4_instance = px4_instance_cstr;

	} else {
		px4_instance = "0";
	}

	_vehicle_model_name = px4_sim_model + "_" + px4_instance;
	_wait_for_vehicle_spawned = true;
}
