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

#pragma once

#include <gz/sim/Util.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/System.hh>
#include "gz/sim/components/LinearVelocity.hh"

#include <gz/transport/Node.hh>

#include <random>

namespace px4
{

static constexpr float DEFAULT_HOME_ALT_AMSL = 488.0; // altitude AMSL at Irchel Park, Zurich, Switzerland [m]

// international standard atmosphere (troposphere model - valid up to 11km) see [1]
static constexpr float TEMPERATURE_MSL = 288.15; // temperature at MSL [K] (15 [C])
static constexpr float PRESSURE_MSL = 101325.0; // pressure at MSL [Pa]
static constexpr float LAPSE_RATE = 0.0065; // reduction in temperature with altitude for troposphere [K/m]
static constexpr float AIR_DENSITY_MSL = 1.225; // air density at MSL [kg/m^3]

class AirSpeed:
	public gz::sim::System,
	public gz::sim::ISystemPreUpdate,
	public gz::sim::ISystemConfigure
{
public:
	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) final;

	void Configure(const gz::sim::Entity &entity,
		       const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm,
		       gz::sim::EventManager &eventMgr) override;

private:
	gz::sim::Entity _entity;
	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Entity _link_entity;
	gz::sim::Link _link;
	gz::sim::Entity _world_entity;
	gz::sim::World _world;

	gz::transport::Node _node;
	gz::transport::Node::Publisher _pub;

	gz::math::Quaterniond _vehicle_attitude;
	gz::math::Vector3d _vehicle_velocity{0., 0., 0.};
	gz::math::Vector3d _vehicle_position{0., 0., 0.};

	std::default_random_engine random_generator_;
	std::normal_distribution<float> standard_normal_distribution_;

	float diff_pressure_stddev_{0.01f}; // [hPa]
	float _alt_home{DEFAULT_HOME_ALT_AMSL};
};
} // end namespace px4
