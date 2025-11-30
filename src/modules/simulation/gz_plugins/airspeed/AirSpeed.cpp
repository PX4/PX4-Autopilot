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

#include "AirSpeed.hpp"

#include <gz/plugin/Register.hh>
#include <gz/msgs/air_speed.pb.h>

using namespace px4;

// Sign function taken from https://stackoverflow.com/a/4609795/8548472
template <typename T> int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

// Register the plugin
GZ_ADD_PLUGIN(
	AirSpeed,
	gz::sim::System,
	AirSpeed::ISystemPreUpdate,
	AirSpeed::ISystemConfigure
)

void AirSpeed::Configure(const gz::sim::Entity &entity,
			 const std::shared_ptr<const sdf::Element> &sdf,
			 gz::sim::EntityComponentManager &ecm,
			 gz::sim::EventManager &eventMgr)
{
	_entity = entity;
	_model = gz::sim::Model(entity);

	const std::string link_name = sdf->Get<std::string>("link_name");
	_link_entity = _model.LinkByName(ecm, link_name);

	if (!_link_entity) {
		throw std::runtime_error("Airspeed::Configure: Link \"" + link_name + "\" was not found. "
					 "Please ensure that your model contains the corresponding link.");
	}

	_link = gz::sim::Link(_link_entity);

	// Needed to report linear & angular velocity
	_link.EnableVelocityChecks(ecm, true);

	_world_entity = gz::sim::worldEntity(ecm);
	_world = gz::sim::World(_world_entity);

	std::string _world_name;
	std::string _model_name;
	std::string airspeed_topic = "/world/" + _world_name + "/model/" + _model_name +
				     "/link/airspeed_link/sensor/air_speed/air_speed";
	_pub = _node.Advertise<gz::msgs::AirSpeed>(airspeed_topic);

	///TODO: Read sdf for altitude home position
}

void AirSpeed::PreUpdate(const gz::sim::UpdateInfo &_info,
			 gz::sim::EntityComponentManager &_ecm)
{
	const auto optional_pose = _link.WorldPose(_ecm);

	if (optional_pose.has_value()) {
		_vehicle_position = optional_pose.value().Pos();
		_vehicle_attitude = optional_pose.value().Rot();

	} else {
		gzerr << "Unable to get pose" << std::endl;
	}

	const auto optional_vel = _link.WorldLinearVelocity(_ecm);

	if (optional_vel.has_value()) {
		_vehicle_velocity = optional_vel.value();

	} else {
		gzerr << "Unable to get linear velocity" << std::endl;
	}

	// Compute the air density at the local altitude / temperature
	const float alt_rel = _vehicle_position.Z(); // Z-component from ENU
	const float alt_amsl = (float)_alt_home + alt_rel;
	const float temperature_local = TEMPERATURE_MSL - LAPSE_RATE * alt_amsl;
	const float density_ratio = powf(TEMPERATURE_MSL / temperature_local, 4.256f);
	const float air_density = AIR_DENSITY_MSL / density_ratio;

	// Calculate differential pressure + noise in hPa
	const float diff_pressure_noise = standard_normal_distribution_(random_generator_) * diff_pressure_stddev_;
	// Body-relateive air velocity
	///TODO: Subscribe to wind velocity from the world
	gz::math::Vector3d _wind_velocity{0.0, 0.0, 0.0};
	gz::math::Vector3d air_relative_velocity = _vehicle_velocity - _wind_velocity;
	gz::math::Vector3d body_velocity = _vehicle_attitude.RotateVectorReverse(air_relative_velocity);
	double diff_pressure = sign(body_velocity.X()) * 0.005 * (double)air_density  * body_velocity.X() * body_velocity.X() +
			       (double)diff_pressure_noise;
	// Calculate differential pressure in hPa
	gz::msgs::AirSpeed airspeed_msg;
	// airspeed_msg.set_time_usec(last_time_.Double() * 1e6);
	airspeed_msg.set_diff_pressure(diff_pressure);
	_pub.Publish(airspeed_msg);
}
