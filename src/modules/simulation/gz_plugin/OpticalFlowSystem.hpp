#pragma once

#include <gz/sim/System.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>

namespace custom
{
class OpticalFlowSystem:
	public gz::sim::System,
	public gz::sim::ISystemPreUpdate,
	public gz::sim::ISystemPostUpdate
{
public:
	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) final;

	void PostUpdate(const gz::sim::UpdateInfo &_info,
			const gz::sim::EntityComponentManager &_ecm) final;

private:
	void RemoveSensorEntities(const gz::sim::EntityComponentManager &_ecm);

	std::unordered_map<gz::sim::Entity, std::shared_ptr<OpticalFlowSensor>> entitySensorMap;
};
} // end namespace custom
