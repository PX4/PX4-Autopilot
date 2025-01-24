// OpticalFlowSystem.cc
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
			const gz::sim::components::ParentEntity * _parent)->bool
	{
		auto sensorScopedName = gz::sim::removeParentScope(gz::sim::scopedName(_entity, _ecm, "::", false), "::");

		sdf::Sensor data = _custom->Data();
		data.SetName(sensorScopedName);

		if (data.Topic().empty()) {
			std::string topic = scopedName(_entity, _ecm) + "/optical_flow";
			data.SetTopic(topic);
		}

		gz::sensors::SensorFactory sensorFactory;
		auto sensor = sensorFactory.CreateSensor<custom::OpticalFlowSensor>(data);

		if (sensor == nullptr) {
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
		const gz::sim::components::CustomSensor *)->bool
	{
		if (this->entitySensorMap.erase(_entity) == 0) {
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
