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

#include "ParachuteSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/Util.hh>

using namespace gz;
using namespace sim;
using namespace systems;

//////////////////////////////////////////////////
void ParachuteSystem::Configure(const Entity &_entity,
				const std::shared_ptr<const sdf::Element> &_sdf,
				EntityComponentManager &_ecm,
				EventManager &/*_eventMgr*/)
{
	_model = Model(_entity);

	if (!_model.Valid(_ecm)) {
		gzerr << "[ParachuteSystem] plugin should be attached to a model entity. "
		      << "Failed to initialize." << std::endl;
		return;
	}

	const std::string model_name = _model.Name(_ecm);

	if (_sdf->HasElement("link_name")) {
		_link_name = _sdf->Get<std::string>("link_name");
	}

	if (_sdf->HasElement("cd_a")) {
		_cd_a = _sdf->Get<double>("cd_a");
	}

	if (_sdf->HasElement("angular_damping")) {
		_angular_damping = _sdf->Get<double>("angular_damping");
	}

	// the PX4 gz bridge forwards the vehicle's parachute state on this topic
	std::string trigger_topic = "/model/" + model_name + "/parachute";

	if (_sdf->HasElement("trigger_topic")) {
		trigger_topic = _sdf->Get<std::string>("trigger_topic");
	}

	if (!_node.Subscribe(trigger_topic, &ParachuteSystem::TriggerCallback, this)) {
		gzerr << "[ParachuteSystem] Error subscribing to topic [" << trigger_topic << "]" << std::endl;
		return;
	}

	gzmsg << "[ParachuteSystem] Initialized for model " << model_name
	      << ", trigger topic " << trigger_topic
	      << ", CdA " << _cd_a << " m^2" << std::endl;
}

//////////////////////////////////////////////////
void ParachuteSystem::TriggerCallback(const gz::msgs::Boolean &_msg)
{
	if (_msg.data()) {
		// a released parachute cannot be restowed
		_deployed = true;
	}
}

//////////////////////////////////////////////////
void ParachuteSystem::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
	if (_info.paused || !_deployed) {
		return;
	}

	if (!_link_initialized) {
		const Entity link_entity = _model.LinkByName(_ecm, _link_name);

		if (link_entity == kNullEntity) {
			gzerr << "[ParachuteSystem] Link [" << _link_name << "] not found" << std::endl;
			_deployed = false;
			return;
		}

		_link = Link(link_entity);
		_link.EnableVelocityChecks(_ecm, true);
		_link_initialized = true;
		return; // velocity components become available next step
	}

	if (!_deployed_notified) {
		gzmsg << "[ParachuteSystem] Parachute deployed!" << std::endl;
		_deployed_notified = true;
	}

	const auto velocity = _link.WorldLinearVelocity(_ecm);

	if (velocity.has_value()) {
		// the canopy drifts with the wind: drag acts on the air-relative velocity
		math::Vector3d wind_velocity{0.0, 0.0, 0.0};
		const Entity wind_entity = _ecm.EntityByComponents(components::Wind());

		if (wind_entity != kNullEntity) {
			const auto wind_linear_velocity = _ecm.Component<components::WorldLinearVelocity>(wind_entity);

			if (wind_linear_velocity) {
				wind_velocity = wind_linear_velocity->Data();
			}
		}

		const math::Vector3d air_relative_velocity = velocity.value() - wind_velocity;

		// canopy drag: F = -1/2 * rho * CdA * |v| * v
		const math::Vector3d force = -0.5 * kAirDensity * _cd_a * air_relative_velocity.Length() * air_relative_velocity;
		math::Vector3d torque{0.0, 0.0, 0.0};

		const auto angular_velocity = _link.WorldAngularVelocity(_ecm);

		if (angular_velocity.has_value()) {
			// the canopy suppresses tumbling of the suspended vehicle
			torque = -_angular_damping * angular_velocity.value();
		}

		_link.AddWorldWrench(_ecm, force, torque);
	}
}

// Register the plugin
GZ_ADD_PLUGIN(
	ParachuteSystem,
	gz::sim::System,
	ParachuteSystem::ISystemConfigure,
	ParachuteSystem::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(ParachuteSystem,
		    "gz::sim::systems::ParachuteSystem")
