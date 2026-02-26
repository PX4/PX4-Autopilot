/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_BUOYANCY_SYSTEM_HPP_
#define GZ_SIM_SYSTEMS_BUOYANCY_SYSTEM_HPP_

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
class BuoyancyPrivate;

/// \brief A system that simulates buoyancy of objects immersed in fluid.
/// All SDF parameters are optional. This system must be attached to the
/// world and this system will apply buoyancy to all links that have collision
/// shapes.
///
/// The volume and center of volume will be computed for each link, and
/// stored as components. During each iteration, Archimedes' principle is
/// applied to each link with a volume and center of volume component.
///
/// Plane shapes are not handled by this plugin, and will not be affected
/// by buoyancy.
///
/// ## System Parameters
///
/// * `<uniform_fluid_density>` sets the density of the fluid that surrounds
/// the buoyant object. [Units: kgm^-3]
/// * `<graded_buoyancy>` allows you to define a world where the buoyancy
/// changes along the Z axis. An example of such a world could be if we are
/// simulating an open ocean with its surface and under water behaviour. This
/// mode slices the volume of each collision mesh according to where the water
/// line is set. When defining a `<graded_buoyancy>` tag, one must also define
/// `<default_density>` and `<density_change>` tags.
/// * `<default_density>` is the default fluid which the world should be
/// filled with. [Units: kgm^-3]
/// * `<density_change>` allows you to define a new layer.
/// * `<above_depth>` a child property of `<density_change>`. This determines
/// the height at which the next fluid layer should start. [Units: m]
/// * `<density>` the density of the fluid in this layer. [Units: kgm^-3]
/// * `<enable>` used to indicate which models will have buoyancy.
/// Add one enable element per model or link. This element accepts names
/// scoped from the top level model (i.e. `<model>::<nested_model>::<link>`).
/// If there are no enabled entities, all models in simulation will be
/// affected by buoyancy.
///
/// ## Examples
///
/// ### uniform_fluid_density world
///
/// The `buoyancy.sdf` SDF file contains three submarines. The first
/// submarine is neutrally buoyant, the second sinks, and the third
/// floats. To run:
///
/// ```
/// gz sim -v 4 buoyancy.sdf
/// ```
///
/// ### graded_buoyancy world
///
/// Often when simulating a maritime environment one may need to simulate both
/// surface and underwater vessels. This means the buoyancy plugin needs to
/// take into account two different fluids. One being water with a density of
/// 1000kgm^-3 and another being air with a very light density of say 1kgm^-3.
/// An example for such a configuration may be found in the
/// `graded_buoyancy.sdf` world.
///
/// ```
/// gz sim -v 4 graded_buoyancy.sdf
/// ```
///
/// You should be able to see a sphere bobbing up and down undergoing simple
/// harmonic motion on the surface of the fluid (this is expected behaviour
/// as the SHM is usually damped by the hydrodynamic forces. See the hydro-
/// dynamics plugin for an example of how to use it). The key part of this is
///
/// ```
/// <graded_buoyancy>
///   <default_density>1000</default_density>
///   <density_change>
///     <above_depth>0</above_depth>
///     <density>1</density>
///   </density_change>
/// </graded_buoyancy>
/// ```
/// The default density tag says that by default the world has a fluid density
/// of 1000kgm^-3. This essentially states that by default the world is filled
/// with dihydrogen monoxide (aka water). The `<density_change>` tag
/// essentially establishes the fact that there is a nother fluid. The
/// `<above_depth>` tag says that above z=0 there is another fluid with a
/// different density. The density of that fluid is defined by the `<density>`
/// tag. We will be simulating air with a fluid density of 1kgm^-3.
class Buoyancy
	: public System,
	  public ISystemConfigure,
	  public ISystemPreUpdate,
	  public ISystemPostUpdate
{
	/// \brief Constructor
public: Buoyancy();

	/// \brief Destructor
public: ~Buoyancy() override = default;

	// Documentation inherited
public: void Configure(const Entity &_entity,
			       const std::shared_ptr<const sdf::Element> &_sdf,
			       EntityComponentManager &_ecm,
			       EventManager &_eventMgr) override;

	// Documentation inherited
public: void PreUpdate(
		const UpdateInfo &_info,
		EntityComponentManager &_ecm) override;

	// Documentation inherited
public: void PostUpdate(
		const UpdateInfo &_info,
		const EntityComponentManager &_ecm) override;

	/// \brief Check if an entity is enabled or not.
	/// \param[in] _entity Target entity
	/// \param[in] _ecm Entity component manager
	/// \return True if buoyancy should be applied.
public: bool IsEnabled(Entity _entity,
			       const EntityComponentManager &_ecm) const;

	/// \brief Private data pointer
private: std::unique_ptr<BuoyancyPrivate> dataPtr;
};
}
}
}
}

#endif
