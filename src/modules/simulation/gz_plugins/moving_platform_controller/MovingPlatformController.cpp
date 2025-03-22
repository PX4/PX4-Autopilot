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

	double v_x = 1.0f;

	// Read v_x from environment variable.
	{
		const char *env_var_name = "PX4_GZ_PLATFORM_VEL";
		const char *env_var_value = std::getenv(env_var_name);

		if (env_var_value) {
			try {
				v_x = std::stof(env_var_value);

			} catch (const std::invalid_argument &e) {
				// These warnings will only be visible with sufficient verbosity level...
				gzwarn << "Invalid argument: " << env_var_value << " is not a valid double for " << env_var_name << std::endl;
				gzwarn << "Keeping default value of " << v_x << " m/s." << std::endl;

			} catch (const std::out_of_range &e) {
				gzwarn << "Out of range: " << env_var_value << " is out of range for " << env_var_name << std::endl;
				gzwarn << "Keeping default value of " << v_x << " m/s." << std::endl;
			}
		}
	}

	_target_vel_sp = gz::math::Vector3d(v_x, 0., 0.);

	// Advertise topic to communicate with VelocityControl plugin
	std::string model_name = _model.Name(ecm);
	std::string cmd_vel_topic = "/model/" + model_name + "/cmd_vel";
	_platform_twist_pub = _node.Advertise<gz::msgs::Twist>(cmd_vel_topic);
}

void MovingPlatformController::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &ecm)
{
	updatePose(ecm);

	// Initially, ramp up the velocity with constant acceleration. Otherwise
	// we can't really simulate fast velocities as the drone just slips off
	// the platform.

	const double max_accel_mag = 2.; // m/s^2
	const double dt = std::chrono::duration<double>(_info.dt).count(); // s

	const gz::math::Vector3d vel_sp_err = _target_vel_sp - _current_vel_sp;

	if (vel_sp_err.Length() < max_accel_mag * dt) {
		// we are already almost there. set directly to avoid division by 0.
		_current_vel_sp = _target_vel_sp;

	} else {
		// acclerate towards target velocity.
		const gz::math::Vector3d accel = vel_sp_err.Normalized() * max_accel_mag;
		_current_vel_sp = _current_vel_sp + accel * dt;
	}

	updateVelocityCommands(_current_vel_sp);

	sendVelocityCommands();
}

void MovingPlatformController::updateVelocityCommands(const gz::math::Vector3d &velocity_setpoint)
{
	// Velocity and angular velocity = low pass filtered white noise + feedback term.

	gz::math::Vector3d noise_v = gz::math::Vector3d(
					     gz::math::Rand::DblNormal(),
					     gz::math::Rand::DblNormal(),
					     gz::math::Rand::DblNormal()
				     );

	gz::math::Vector3d noise_w = gz::math::Vector3d(
					     gz::math::Rand::DblNormal(),
					     gz::math::Rand::DblNormal(),
					     gz::math::Rand::DblNormal()
				     );

	// Filter coefficients for low-pass filtering the white noise.
	// larger number here = faster movement
	// write these in terms of time constants?
	// https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/Signals-and-Systems/Lectures/Lecture%20Notes%209.pdf
	const double filter_coef_v = 0.01;
	const double filter_coef_w = 0.01;

	// Noise amplitude.
	// larger number here = bigger movement
	const double ampl_v = 1.;
	const double ampl_w = 1.;

	// For ultra realism we might have axis specific versions of these
	// constants -- Real ships roll more quickly than they pitch and yaw.
	// But probably overkill for now.

	_noise_v_lowpass = (1 - filter_coef_v) * _noise_v_lowpass + filter_coef_v * noise_v;
	_noise_w_lowpass = (1 - filter_coef_w) * _noise_w_lowpass + filter_coef_w * noise_w;

	_platform_v = ampl_v * _noise_v_lowpass + velocity_setpoint;
	_platform_w = ampl_w * _noise_w_lowpass;

	// feedback terms to ensure the random walk (= integral of noise)
	// stays within a realistic region.
	{
		// small feedback to maintain height. no attempt to stabilise x and y.
		const double platform_height = 2.;
		const gz::math::Vector3d pos_gains(0., 0., 1.);  // [m/s / m]
		_platform_v += -pos_gains * (_platform_position - gz::math::Vector3d(0., platform_height, 0.));

		// eq. 23 from Nonlinear Quadrocopter Attitude Control (Brescianini, Hehn, D'Andrea)
		// https://www.research-collection.ethz.ch/handle/20.500.11850/154099
		const double sgn = _platform_orientation.W() > 0 ? 1. : -1.;

		const double attitude_gain = 1.;

		gz::math::Vector3d q_imag = gz::math::Vector3d(
						    _platform_orientation.X(),
						    _platform_orientation.Y(),
						    _platform_orientation.Z()
					    );

		_platform_w += -attitude_gain * sgn * q_imag;
	}
}


void MovingPlatformController::updatePose(const gz::sim::EntityComponentManager &ecm)
{
	auto pose = ecm.Component<gz::sim::components::Pose>(_entity);

	if (pose != nullptr) {
		_platform_position = pose->Data().Pos();
		_platform_orientation = pose->Data().Rot();

	} else {
		std::cerr << "MovingPlatformController: got nullptr pose" << std::endl;
	}
}

void MovingPlatformController::sendVelocityCommands()
{
	// We use the velocity control plugin to directly set velocities.
	gz::msgs::Twist twist_msg;
	gz::msgs::Set(twist_msg.mutable_linear(), _platform_v);
	gz::msgs::Set(twist_msg.mutable_angular(), _platform_w);
	_platform_twist_pub.Publish(twist_msg);
}
