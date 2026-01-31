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

#include <string>
#include <unordered_map>
#include <utility>

#include <gz/plugin/Register.hh>
#include <gz/sensors/SensorFactory.hh>
#include <sdf/Sensor.hh>

#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include "OpticalFlowSensor.hpp"
#include "OpticalFlowSystem.hpp"

using namespace custom;

void OpticalFlowSystem::PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm)
{
	// Register each new custom sensor
	_ecm.EachNew<gz::sim::components::CustomSensor, gz::sim::components::ParentEntity>(
		[&](const gz::sim::Entity & _entity,
		    const gz::sim::components::CustomSensor * _custom,
	const gz::sim::components::ParentEntity * _parent)->bool {
		auto sensorScopedName = gz::sim::removeParentScope(gz::sim::scopedName(_entity, _ecm, "::", false), "::");

		sdf::Sensor data = _custom->Data();
		data.SetName(sensorScopedName);

		if (data.Topic().empty())
		{
			std::string topic = scopedName(_entity, _ecm) + "/optical_flow";
			data.SetTopic(topic);
		}

		gz::sensors::SensorFactory sensorFactory;
		auto sensor = sensorFactory.CreateSensor<custom::OpticalFlowSensor>(data);

		if (sensor == nullptr)
		{
			gzerr << "Failed to create optical flow sensor [" << sensorScopedName << "]" << std::endl;
			return false;
		}

		auto parentName = _ecm.Component<gz::sim::components::Name>(_parent->Data())->Data();

		sensor->SetParent(parentName);

		_ecm.CreateComponent(_entity, gz::sim::components::SensorTopic(sensor->Topic()));

		this->entitySensorMap.insert(std::make_pair(_entity, std::move(sensor)));

		gzdbg << "OpticalFlowSystem PreUpdate" << std::endl;

		return true;
	});
}

void OpticalFlowSystem::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm)
{
	if (!_info.paused) {
		for (auto &[entity, sensor] : this->entitySensorMap) {
			sensor->Update(_info.simTime);
		}
	}

	this->RemoveSensorEntities(_ecm);
}

void OpticalFlowSystem::RemoveSensorEntities(const gz::sim::EntityComponentManager &_ecm)
{
	_ecm.EachRemoved<gz::sim::components::CustomSensor>(
		[&](const gz::sim::Entity & _entity,
	const gz::sim::components::CustomSensor *)->bool {
		if (this->entitySensorMap.erase(_entity) == 0)
		{
			gzerr << "Internal error, missing optical flow sensor for entity ["
			      << _entity << "]" << std::endl;
		}
		return true;
	});
}

GZ_ADD_PLUGIN(OpticalFlowSystem, gz::sim::System,
	      OpticalFlowSystem::ISystemPreUpdate,
	      OpticalFlowSystem::ISystemPostUpdate
	     )

GZ_ADD_PLUGIN_ALIAS(OpticalFlowSystem, "custom::OpticalFlowSystem")
