/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "ElasticCatapultSystem.hpp"

#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Helpers.hh>   // GZ_DTOR

#include <chrono>
#include <cmath>

using namespace custom;

GZ_ADD_PLUGIN(
	ElasticCatapultSystem,
	gz::sim::System,
	ElasticCatapultSystem::ISystemConfigure,
	ElasticCatapultSystem::ISystemPreUpdate
)

namespace
{
/* Read an optional scalar/string SDF element, falling back to a default. */
template <typename T>
T sdfGet(const std::shared_ptr<const sdf::Element> &sdf, const std::string &key, const T &fallback)
{
	if (sdf->HasElement(key)) {
		return sdf->Get<T>(key);
	}

	return fallback;
}
} // namespace

void ElasticCatapultSystem::Configure(const gz::sim::Entity &entity,
				      const std::shared_ptr<const sdf::Element> &sdf,
				      gz::sim::EntityComponentManager &ecm,
				      gz::sim::EventManager & /*eventMgr*/)
{
	_model = gz::sim::Model(entity);

	/* ── Parameters ── */
	_link_name                    = sdfGet<std::string>(sdf, "link_name", _link_name);
	_t_start                      = sdfGet<double>(sdf, "t_start", _t_start);
	_t_catapult                   = sdfGet<double>(sdf, "t_catapult", _t_catapult);
	_target_speed                 = sdfGet<double>(sdf, "target_speed", _target_speed);
	_initial_roll_deg             = sdfGet<double>(sdf, "initial_roll_deg", _initial_roll_deg);
	_initial_pitch_deg            = sdfGet<double>(sdf, "initial_pitch_deg", _initial_pitch_deg);
	_initial_yaw_deg              = sdfGet<double>(sdf, "initial_yaw_deg", _initial_yaw_deg);
	_hold_attitude_until_release  = sdfGet<bool>(sdf, "hold_attitude_until_release", _hold_attitude_until_release);
	_force_model                  = sdfGet<std::string>(sdf, "force_model", _force_model);
	_spring_k                     = sdfGet<double>(sdf, "spring_k", _spring_k);
	_spring_x0                    = sdfGet<double>(sdf, "spring_x0", _spring_x0);
	_spring_c                     = sdfGet<double>(sdf, "spring_c", _spring_c);
	_catapult_direction_mode      = sdfGet<std::string>(sdf, "catapult_direction_mode", _catapult_direction_mode);

	if (_t_catapult <= 0.0) {
		gzerr << "[ElasticCatapult] t_catapult must be > 0; got " << _t_catapult << ". Disabling plugin." << std::endl;
		return;
	}

	/* ── Resolve the target link ── */
	_link_entity = _model.LinkByName(ecm, _link_name);

	if (_link_entity == gz::sim::kNullEntity) {
		gzerr << "[ElasticCatapult] Link \"" << _link_name << "\" not found on model. Disabling plugin." << std::endl;
		return;
	}

	_link = gz::sim::Link(_link_entity);
	_link.EnableVelocityChecks(ecm, true);

	_world = gz::sim::World(gz::sim::worldEntity(ecm));

	/* ── Fixed launch attitude and forward direction ── */
	_q_hold = gz::math::Quaterniond(GZ_DTOR(_initial_roll_deg),
					GZ_DTOR(_initial_pitch_deg),
					GZ_DTOR(_initial_yaw_deg));
	_dir_world = _q_hold.RotateVector(gz::math::Vector3d(1.0, 0.0, 0.0));

	if (_dir_world.Length() > 1e-9) {
		_dir_world.Normalize();
	}

	/* ── Mass (for F = m a) ── */
	auto inertial = ecm.Component<gz::sim::components::Inertial>(_link_entity);

	if (inertial) {
		_mass = inertial->Data().MassMatrix().Mass();

	} else {
		gzwarn << "[ElasticCatapult] No inertial component on link \"" << _link_name
		       << "\"; using fallback mass " << _mass << " kg." << std::endl;
	}

	/* ── Constant-model force magnitude: reach target_speed from rest in t_catapult ── */
	_force_const = _mass * _target_speed / _t_catapult;

	_state = State::WAITING;
	_configured = true;

	gzmsg << "[ElasticCatapult] configured: link=" << _link_name
	      << " mass=" << _mass << "kg"
	      << " t_start=" << _t_start << "s t_catapult=" << _t_catapult << "s"
	      << " target=" << _target_speed << "m/s"
	      << " rpy=[" << _initial_roll_deg << "," << _initial_pitch_deg << "," << _initial_yaw_deg << "]deg"
	      << " force_model=" << _force_model
	      << " dir_world=[" << _dir_world.X() << "," << _dir_world.Y() << "," << _dir_world.Z() << "]"
	      << " F_const=" << _force_const << "N" << std::endl;
}

void ElasticCatapultSystem::holdPoseStationary(gz::sim::EntityComponentManager &ecm)
{
	if (!_hold_attitude_until_release) {
		return;
	}

	const auto pose = _link.WorldPose(ecm);

	if (pose.has_value()) {
		// Pin position + orientation and zero out all motion.
		_model.SetWorldPoseCmd(ecm, gz::math::Pose3d(pose.value().Pos(), _q_hold));
		_link.SetLinearVelocity(ecm, gz::math::Vector3d::Zero);
		_link.SetAngularVelocity(ecm, gz::math::Vector3d::Zero);
	}
}

void ElasticCatapultSystem::holdAttitudeOnly(gz::sim::EntityComponentManager &ecm)
{
	if (!_hold_attitude_until_release) {
		return;
	}

	// Keep the launch attitude but never zero linear velocity here, so the
	// catapult can keep accelerating the airframe. Zeroing angular velocity
	// prevents tumbling while the attitude is held.
	_link.SetAngularVelocity(ecm, gz::math::Vector3d::Zero);
}

gz::math::Vector3d ElasticCatapultSystem::computeForce(double sim_time, gz::sim::EntityComponentManager &ecm) const
{
	double magnitude = _force_const;

	if (_force_model == "spring") {
		const double tau = sim_time - _t_start;
		const double x = _spring_x0 * std::max(0.0, 1.0 - tau / _t_catapult);
		magnitude = _spring_k * x;

		if (_spring_c > 0.0) {
			const auto vel = _link.WorldLinearVelocity(ecm);

			if (vel.has_value()) {
				const double v_along = vel.value().Dot(_dir_world);
				magnitude -= _spring_c * v_along;
			}
		}

		magnitude = std::max(0.0, magnitude);
	}

	return _dir_world * magnitude;
}

void ElasticCatapultSystem::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
	if (!_configured || _info.paused) {
		return;
	}

	const double sim_time = std::chrono::duration<double>(_info.simTime).count();
	const double t_release = _t_start + _t_catapult;

	switch (_state) {
	case State::WAITING:
		holdPoseStationary(_ecm);

		if (sim_time >= _t_start) {
			_state = State::CATAPULTING;
			gzmsg << "[ElasticCatapult] t=" << sim_time << "s  WAITING -> CATAPULTING" << std::endl;
		}

		break;

	case State::CATAPULTING: {
			holdAttitudeOnly(_ecm);

			if (_force_model == "velocity") {
				// Deterministic release speed: ramp the launch-direction speed
				// linearly from 0 to target over t_catapult. Avoids fighting the
				// physics solver with a pose teleport (see spec note 14).
				const double tau = sim_time - _t_start;
				const double ramp = std::min(1.0, std::max(0.0, tau / _t_catapult));
				_link.SetLinearVelocity(_ecm, _dir_world * (_target_speed * ramp));

			} else {
				// Pure force models ("constant" | "spring").
				const gz::math::Vector3d force = computeForce(sim_time, _ecm);
				_link.AddWorldWrench(_ecm, force, gz::math::Vector3d::Zero);
			}

			if (sim_time >= t_release) {
				double speed = 0.0;
				const auto vel = _link.WorldLinearVelocity(_ecm);

				if (vel.has_value()) {
					speed = vel.value().Length();
				}

				_state = State::RELEASED;
				gzmsg << "[ElasticCatapult] t=" << sim_time
				      << "s  CATAPULTING -> RELEASED  release_speed=" << speed
				      << "m/s (target=" << _target_speed << "m/s)" << std::endl;
			}

			break;
		}

	case State::RELEASED:
		// Constraint and force removed; hand over to physics + PX4 control.
		break;
	}
}
