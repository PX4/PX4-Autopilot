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

/**
 * @file ElasticCatapultSystem.hpp
 *
 * Gazebo Harmonic (gz-sim) System plugin that reproduces a catapult launch of
 * a fixed-wing model: it holds the airframe at a fixed launch attitude, then
 * applies an external force along the initial-attitude forward direction for a
 * configured duration so that the model reaches a target release speed, and
 * finally releases it to free physics + PX4 control.
 *
 * State machine:  WAITING -> CATAPULTING -> RELEASED
 */

#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <string>

namespace custom
{

class ElasticCatapultSystem:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity,
		       const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm,
		       gz::sim::EventManager &eventMgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) final;

private:
	enum class State : uint8_t {
		WAITING     = 0,
		CATAPULTING = 1,
		RELEASED    = 2,
	};

	/* Pin the model at the fixed launch attitude and current position, and
	 * zero its velocities (used while WAITING). */
	void holdPoseStationary(gz::sim::EntityComponentManager &ecm);

	/* Re-assert the fixed launch orientation without zeroing linear velocity
	 * (used while CATAPULTING so the model can keep accelerating). */
	void holdAttitudeOnly(gz::sim::EntityComponentManager &ecm);

	/* World-frame catapult force for the current sim time. */
	gz::math::Vector3d computeForce(double sim_time, gz::sim::EntityComponentManager &ecm) const;

	/* ── Entities ── */
	gz::sim::Entity _link_entity{gz::sim::kNullEntity};
	gz::sim::Model  _model{gz::sim::kNullEntity};
	gz::sim::Link   _link;
	gz::sim::World  _world;

	/* ── Parameters (from SDF) ── */
	std::string _link_name{"base_link"};
	double _t_start{3.0};            // [s] sim start -> launch start
	double _t_catapult{1.0};         // [s] launch start -> release
	double _target_speed{20.0};      // [m/s] release speed
	double _initial_roll_deg{0.0};
	double _initial_pitch_deg{10.0};
	double _initial_yaw_deg{0.0};
	bool   _hold_attitude_until_release{true};
	std::string _force_model{"velocity"};    // "velocity" | "constant" | "spring"
	double _spring_k{80.0};
	double _spring_x0{1.5};
	double _spring_c{0.0};
	std::string _catapult_direction_mode{"initial_attitude_forward"};

	/* ── Derived ── */
	gz::math::Quaterniond _q_hold{1., 0., 0., 0.};   // fixed launch attitude
	gz::math::Vector3d _dir_world{1., 0., 0.};       // unit launch direction (world)
	double _mass{2.0};                               // [kg] from inertial component
	double _force_const{0.0};                        // [N] constant-model magnitude

	/* ── Runtime ── */
	State _state{State::WAITING};
	bool _configured{false};
};

} // namespace custom
