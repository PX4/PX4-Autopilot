/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include <gz/msgs/boolean.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <atomic>
#include <string>

namespace gz
{
namespace sim
{
namespace systems
{

/**
 * @brief Parachute system plugin
 *
 * Simulates a parachute attached to a model. The parachute deploys on the release message
 * that the PX4 gz bridge forwards on /model/<name>/parachute (independent of which output
 * the Parachute function is mapped to), and from then on applies a velocity-dependent drag
 * force plus an angular damping torque to the configured link. Deployment latches until
 * the simulation is reset.
 *
 * SDF configuration (all optional):
 * - <link_name>: link the drag is applied to (default: base_link)
 * - <trigger_topic>: topic override for the trigger subscription (gz.msgs.Boolean)
 * - <cd_a>: drag coefficient times canopy area [m^2] (default: 1.3)
 * - <angular_damping>: angular damping coefficient [N*m*s] (default: 1.0)
 */
class ParachuteSystem:
	public System,
	public ISystemConfigure,
	public ISystemPreUpdate
{
public:
	void Configure(const Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       EntityComponentManager &_ecm,
		       EventManager &_eventMgr) final;

	void PreUpdate(const UpdateInfo &_info,
		       EntityComponentManager &_ecm) final;

private:
	void TriggerCallback(const gz::msgs::Boolean &_msg);

	Model _model{kNullEntity};
	Link _link{kNullEntity};
	std::string _link_name{"base_link"};

	transport::Node _node;

	std::atomic<bool> _deployed{false};
	bool _deployed_notified{false};
	bool _link_initialized{false};

	double _cd_a{1.3};             // [m^2] drag coefficient * canopy area
	double _angular_damping{1.0};  // [N*m*s]

	static constexpr double kAirDensity = 1.225; // [kg/m^3]
};

} // namespace systems
} // namespace sim
} // namespace gz
