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

#include <gz/plugin/Register.hh>

#include "gz/sim/components/Pose.hh"
#include <gz/sim/components/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include <gz/math.hh>
#include <gz/math/Rand.hh>
#include <gz/math/Pose3.hh>

void MovingPlatformController::Configure(const gz::sim::Entity &entity,
		const std::shared_ptr<const sdf::Element> &sdf,
		gz::sim::EntityComponentManager &ecm,
		gz::sim::EventManager &eventMgr)
{
	_entity = entity;
}

void MovingPlatformController::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &ecm)
{
	updatePose(ecm);

	const gz::math::Vector3d mean_vel(1.0, 0.0, 0.0);
	updateVelocityCommands(mean_vel);

	sendVelocityCommands();
}

void MovingPlatformController::updateVelocityCommands(const gz::math::Vector3d &mean_velocity)
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

	// Update rates for the filtered white noise.
	// larger number here = faster movement
	// write these in terms of time constants?
	// https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/Signals-and-Systems/Lectures/Lecture%20Notes%209.pdf
	const double alpha_v = 0.01;
	const double alpha_w = 0.01;

	// Noise amplitude.
	// larger number here = bigger movement
	// we scale it by the update rates to keep total energy constant
	const double ampl_v = 0.01 / alpha_v;
	const double ampl_w = 0.01 / alpha_w;

	// For ultra realism we might have axis specific versions of these
	// constants -- Real ships roll more quickly than they pitch and yaw.
	// But probably overkill for now.

	_noise_v_lowpass = (1 - alpha_v) * _noise_v_lowpass + alpha_v * noise_v;
	_noise_w_lowpass = (1 - alpha_w) * _noise_w_lowpass + alpha_w * noise_w;

	_platform_v = ampl_v * _noise_v_lowpass + mean_velocity;
	_platform_w = ampl_w * _noise_w_lowpass;

	const bool feedback = true;

	// feedback terms to ensure the random walk (= integral of noise)
	// stays within a realistic region.
	if (feedback) {

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
	_platform_position = pose->Data().Pos();
	_platform_orientation = pose->Data().Rot();
}

void MovingPlatformController::sendVelocityCommands()
{

}

GZ_ADD_PLUGIN(MovingPlatformController, gz::sim::System, MovingPlatformController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(MovingPlatformController, "custom::MovingPlatformController")
