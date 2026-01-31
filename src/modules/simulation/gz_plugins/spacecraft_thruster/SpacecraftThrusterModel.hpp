/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef GZ_SIM_SYSTEMS_SPACECRAFTTHRUSTERMODEL_HH_
#define GZ_SIM_SYSTEMS_SPACECRAFTTHRUSTERMODEL_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
// Forward declaration
class SpacecraftThrusterModelPrivate;

/// \brief This system applies a thrust force to models with RCS-like
/// thrusters. See tutorials/spacecraft_thrusters.md for a tutorial usage.
/// Below follow the minimum necessary parameters needed by the plugin:
/// \param link_name Name of the link that the thruster is attached to.
/// \param actuator_number Index of the element to be used from the
///        actuators message for a joint.
/// \param duty_cycle_frequency Frequency of the duty cycle signal in Hz.
/// \param max_thrust Maximum thrust force in Newtons, applied on the
///        «on» phase of the duty cycle.
/// \param topic Name of the topic where the commanded normalized
///        thrust is published. Unit is <0, 1>, corresponding to the
///        percentage of the duty cycle that the thruster is on.
///        Default uses the models name.
/// \param sub_topic [optional] Name of the sub_topic to listen to actuator
///        message on.
///
/// This plugin replicates the PWM thruster behavior in:
/// Nakka, Yashwanth Kumar, et al. "A six degree-of-freedom spacecraft
/// dynamics simulator for formation control research." 2018 AAS/AIAA
/// Astrodynamics Specialist Conference. 2018. -> 'Thruster Firing Time'
/// Phodapol, S. (2023). Predictive Controllers for Load Transportation in
/// Microgravity Environments (Dissertation). -> '5.3.4 PWM Controller Node'
/// Retrieved from https://urn.kb.se/resolve?urn=urn:nbn:se:kth:diva-344440

class SpacecraftThrusterModel
	: public System,
	  public ISystemConfigure,
	  public ISystemPreUpdate
{
	/// \brief Constructor
public: SpacecraftThrusterModel();

	/// \brief Destructor
public: ~SpacecraftThrusterModel() override = default;

	// Documentation inherited
public: void Configure(const Entity &_entity,
			       const std::shared_ptr<const sdf::Element> &_sdf,
			       EntityComponentManager &_ecm,
			       EventManager &_eventMgr) override;

	// Documentation inherited
public: void PreUpdate(
		const gz::sim::UpdateInfo &_info,
		gz::sim::EntityComponentManager &_ecm) override;

	/// \brief Private data pointer
private: std::unique_ptr<SpacecraftThrusterModelPrivate> dataPtr;
};
}
}
}
}

#endif
