/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 * Copyright (C) 2024 Open Source Robotics Foundation
 * Copyright (C) 2024 Benjamin Perseghetti, Rudis Laboratories
 * Copyright (C) 2024 Pedro Roque, DCS, KTH, Sweden
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

#include "SpacecraftThrusterModel.hpp"

#include <mutex>
#include <string>
#include <optional>
#include <chrono>

#include <gz/msgs/actuators.pb.h>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::SpacecraftThrusterModelPrivate
{
	/// \brief Callback for actuator commands.
public: void OnActuatorMsg(const msgs::Actuators &_msg);

	/// \brief Apply link forces and moments based on propeller state.
public: void UpdateForcesAndMoments(EntityComponentManager &_ecm);

	/// \brief Link Entity
public: Entity linkEntity;

	/// \brief Link name
public: std::string linkName;

	/// \brief Model interface
public: Model model{kNullEntity};

	/// \brief sub topic for actuator commands.
public: std::string subTopic;

	/// \brief Topic namespace.
public: std::string topic;

	/// \brief Simulation time tracker
public: double simTime = 0.01;

	/// \brief Index of motor in Actuators msg on multirotor_base.
public: int actuatorNumber = 0;

	/// \brief Duty cycle frequency
public: double dutyCycleFrequency = 10.0;

	/// \brief Cycle start time
public: double cycleStartTime = 0.0;

	/// \brief Sampling time with the cycle period.
public: double samplingTime = 0.01;

	/// \brief Actuator maximum thrust
public: double maxThrust = 0.0;

	/// \brief Maximum duty cycle value. Use the same value as in PX4
	/// maximum PWM_<channel>_MAXi value. For the thruster, it is assumed
	/// that the minimum value is 0 (solenoid closed).
public: double maxDutyCycle = 10000.0;

	/// \brief Received Actuators message. This is nullopt if no message has been
	/// received.
public: std::optional<msgs::Actuators> recvdActuatorsMsg;

	/// \brief Mutex to protect recvdActuatorsMsg.
public: std::mutex recvdActuatorsMsgMutex;

	/// \brief Gazebo communication node.
public: transport::Node node;
};

//////////////////////////////////////////////////
SpacecraftThrusterModel::SpacecraftThrusterModel()
	: dataPtr(std::make_unique<SpacecraftThrusterModelPrivate>())
{
}

//////////////////////////////////////////////////
void SpacecraftThrusterModel::Configure(const Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					EntityComponentManager &_ecm,
					EventManager &/*_eventMgr*/)
{
	this->dataPtr->model = Model(_entity);

	if (!this->dataPtr->model.Valid(_ecm)) {
		gzerr << "SpacecraftThrusterModel plugin should be attached to a model "
		      << "entity. Failed to initialize." << std::endl;
		return;
	}

	auto sdfClone = _sdf->Clone();

	this->dataPtr->topic.clear();

	if (sdfClone->HasElement("topic")) {
		this->dataPtr->topic =
			sdfClone->Get<std::string>("topic");

	} else {
		gzwarn << "No topic set using entity name.\n";
		this->dataPtr->topic = this->dataPtr->model.Name(_ecm);
	}

	if (sdfClone->HasElement("link_name")) {
		this->dataPtr->linkName = sdfClone->Get<std::string>("link_name");
	}

	if (this->dataPtr->linkName.empty()) {
		gzerr << "SpacecraftThrusterModel found an empty link_name parameter. "
		      << "Failed to initialize.";
		return;
	}

	if (sdfClone->HasElement("actuator_number")) {
		this->dataPtr->actuatorNumber =
			sdfClone->GetElement("actuator_number")->Get<int>();

	} else {
		gzerr << "Please specify a actuator_number.\n";
	}

	if (sdfClone->HasElement("max_thrust")) {
		this->dataPtr->maxThrust =
			sdfClone->GetElement("max_thrust")->Get<double>();

	} else {
		gzerr << "Please specify actuator "
		      << this->dataPtr->actuatorNumber << " max_thrust.\n";
	}

	if (sdfClone->HasElement("duty_cycle_frequency")) {
		this->dataPtr->dutyCycleFrequency =
			sdfClone->GetElement("duty_cycle_frequency")->Get<double>();

	} else {
		gzerr << "Please specify actuator "
		      << this->dataPtr->actuatorNumber << " duty_cycle_frequency.\n";
	}

	if (sdfClone->HasElement("max_duty_cycle")) {
		this->dataPtr->maxDutyCycle =
			sdfClone->GetElement("max_duty_cycle")->Get<double>();

		if (this->dataPtr->maxDutyCycle <= 0.0) {
			gzerr << "Please specify a positive max_duty_cycle.\n";
			return;
		}

	} else {
		gzerr << "Please specify actuator "
		      << this->dataPtr->actuatorNumber << " max_duty_cycle.\n";
	}

	std::string topic;

	if (sdfClone->HasElement("sub_topic")) {
		this->dataPtr->subTopic =
			sdfClone->Get<std::string>("sub_topic");
		topic = transport::TopicUtils::AsValidTopic(
				this->dataPtr->topic + "/" + this->dataPtr->subTopic);

	} else {
		topic = transport::TopicUtils::AsValidTopic(
				this->dataPtr->topic);
	}

	// Subscribe to actuator command message
	if (topic.empty()) {
		gzerr << "Failed to create topic for [" << this->dataPtr->topic
		      << "]" << std::endl;
		return;

	} else {
		gzdbg << "Listening to topic: " << topic << std::endl;
	}

	this->dataPtr->node.Subscribe(topic,
				      &SpacecraftThrusterModelPrivate::OnActuatorMsg, this->dataPtr.get());

	// Look for components
	// If the link hasn't been identified yet, look for it
	if (this->dataPtr->linkEntity == kNullEntity) {
		this->dataPtr->linkEntity =
			this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
	}

	if (this->dataPtr->linkEntity == kNullEntity) {
		gzerr << "Failed to find link entity. "
		      << "Failed to initialize." << std::endl;
		return;
	}

	// skip UpdateForcesAndMoments if needed components are missing
	bool providedAllComponents = true;

	if (!_ecm.Component<components::WorldPose>(this->dataPtr->linkEntity)) {
		_ecm.CreateComponent(this->dataPtr->linkEntity, components::WorldPose());
	}

	if (!providedAllComponents) {
		gzdbg << "Created necessary components." << std::endl;
	}

}

//////////////////////////////////////////////////
void SpacecraftThrusterModelPrivate::OnActuatorMsg(
	const msgs::Actuators &_msg)
{
	std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);
	this->recvdActuatorsMsg = _msg;

	if (this->actuatorNumber == 0) {
		gzdbg << "Received actuator message!" << std::endl;
	}
}

//////////////////////////////////////////////////
void SpacecraftThrusterModel::PreUpdate(const UpdateInfo &_info,
					EntityComponentManager &_ecm)
{
	GZ_PROFILE("SpacecraftThrusterModel::PreUpdate");

	// \TODO(anyone) Support rewind
	if (_info.dt < std::chrono::steady_clock::duration::zero()) {
		gzwarn << "Detected jump back in time ["
		       << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
		       << "s]. System may not work properly." << std::endl;
	}

	// Nothing left to do if paused.
	if (_info.paused) {
		return;
	}

	this->dataPtr->simTime = std::chrono::duration<double>(_info.simTime).count();
	this->dataPtr->UpdateForcesAndMoments(_ecm);
}

//////////////////////////////////////////////////
void SpacecraftThrusterModelPrivate::UpdateForcesAndMoments(
	EntityComponentManager &_ecm)
{
	GZ_PROFILE("SpacecraftThrusterModelPrivate::UpdateForcesAndMoments");
	std::optional<msgs::Actuators> msg;
	auto actuatorMsgComp =
		_ecm.Component<components::Actuators>(this->model.Entity());

	// Actuators messages can come in from transport or via a component. If a
	// component is available, it takes precedence.
	if (actuatorMsgComp) {
		msg = actuatorMsgComp->Data();

	} else {
		std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);

		if (this->recvdActuatorsMsg.has_value()) {
			msg = *this->recvdActuatorsMsg;
		}
	}

	if (msg.has_value()) {
		if (this->actuatorNumber > msg->velocity_size() - 1) {
			gzerr << "You tried to access index " << this->actuatorNumber
			      << " of the Actuator array which is of size "
			      << msg->velocity_size() << std::endl;
			return;
		}

	} else {
		return;
	}

	// Normalize input
	double normalizedInput =
		msg->velocity(this->actuatorNumber) / this->maxDutyCycle;

	// METHOD:
	//
	// targetDutyCycle starts as a normalized value between 0 and 1, so we
	// need to convert it to the corresponding time in the duty cycle
	// period
	//   |________|    |________
	//   |    ^   |    |        |
	// __|    |   |____|        |__
	//   a    b    c   d
	// a: cycle start time
	// b: sampling time
	// c: target duty cycle
	// d: cycle period
	double targetDutyCycle = normalizedInput * (1.0 / this->dutyCycleFrequency);

	if (this->actuatorNumber == 0)
		gzdbg << this->actuatorNumber
		      << ": target duty cycle: " << targetDutyCycle << std::endl;

	// Calculate cycle start time
	if (this->samplingTime >= 1.0 / this->dutyCycleFrequency) {
		if (this->actuatorNumber == 0)
			gzdbg << this->actuatorNumber
			      << ": Cycle completed. Resetting cycle start time."
			      << std::endl;

		this->cycleStartTime = this->simTime;
	}

	// Calculate sampling time instant within the cycle
	this->samplingTime = this->simTime - this->cycleStartTime;

	if (this->actuatorNumber == 0)
		gzdbg << this->actuatorNumber
		      << ": PWM Period: " << 1.0 / this->dutyCycleFrequency
		      << " Cycle Start time: " << this->cycleStartTime
		      << " Sampling time: " << this->samplingTime << std::endl;

	// Apply force if the sampling time is less than the target ON duty cycle
	double force = this->samplingTime <= targetDutyCycle ? this->maxThrust : 0.0;

	if (targetDutyCycle < 1e-9) { force = 0.0; }

	if (this->actuatorNumber == 0)
		gzdbg << this->actuatorNumber
		      << ": Force: " << force
		      << "  Sampling time: " << this->samplingTime
		      << "  Tgt duty cycle: " << targetDutyCycle << std::endl;

	// Apply force to the link
	Link link(this->linkEntity);
	const auto worldPose = link.WorldPose(_ecm);
	link.AddWorldForce(_ecm,
			   worldPose->Rot().RotateVector(math::Vector3d(0, 0, force)));

	if (this->actuatorNumber == 0)
		gzdbg << this->actuatorNumber
		      << ": Input Value: " << msg->velocity(this->actuatorNumber)
		      << "  Calc. Force: " << force << std::endl;
}

GZ_ADD_PLUGIN(SpacecraftThrusterModel,
	      System,
	      SpacecraftThrusterModel::ISystemConfigure,
	      SpacecraftThrusterModel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(SpacecraftThrusterModel,
		    "gz::sim::systems::SpacecraftThrusterModel")
