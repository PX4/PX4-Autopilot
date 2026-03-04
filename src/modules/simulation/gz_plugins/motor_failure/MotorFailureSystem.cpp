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

#include "MotorFailureSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

using namespace gz;
using namespace sim;
using namespace systems;

//////////////////////////////////////////////////
void MotorFailureSystem::Configure(const Entity &_entity,
				   const std::shared_ptr<const sdf::Element> &_sdf,
				   EntityComponentManager &_ecm,
				   EventManager &/*_eventMgr*/)
{
	this->_model = Model(_entity);
	this->_model_entity = _entity;

	if (!this->_model.Valid(_ecm)) {
		gzerr << "[MotorFailureSystem] plugin should be attached to a model "
		      << "entity. Failed to initialize." << std::endl;
		return;
	}

	// Get model name to use as namespace
	std::string model_name = this->_model.Name(_ecm);

	// Get Gazebo Transport topic name for motor failure number subscription
	if (_sdf->HasElement("MotorFailureTopic")) {
		this->_gz_topic = _sdf->Get<std::string>("MotorFailureTopic");

	} else {
		// Use Gazebo model-scoped topic naming convention
		this->_gz_topic = "/model/" + model_name + "/motor_failure/motor_number";
	}

	// Subscribe to Gazebo Transport topic
	if (!this->_node.Subscribe(this->_gz_topic, &MotorFailureSystem::MotorFailureNumberCallback, this)) {
		gzerr << "[MotorFailureSystem] Error subscribing to topic [" << this->_gz_topic << "]" << std::endl;
		return;
	}

	gzmsg << "[MotorFailureSystem] Subscribed to Gazebo Transport topic: " << this->_gz_topic << std::endl;
	gzmsg << "[MotorFailureSystem] Initialized for model: " << model_name << std::endl;
}

//////////////////////////////////////////////////
void MotorFailureSystem::FindMotorJoints(EntityComponentManager &_ecm)
{
	if (this->_joints_found) {
		return;
	}

	// Find all joints with "rotor_X_joint" pattern
	this->_motor_joints.clear();

	// Get all joints in the model
	auto joints = this->_model.Joints(_ecm);

	// Regular expression to match rotor joints (e.g., "rotor_0_joint", "rotor_1_joint")
	std::regex motorPattern("rotor_(\\d+)_joint");
	std::smatch match;

	// Find rotor joints and sort by motor number
	std::map<int, Entity> motorMap;

	for (const auto &joint : joints) {
		auto nameComp = _ecm.Component<components::Name>(joint);

		if (nameComp) {
			std::string jointName = nameComp->Data();

			// Try to match the joint name against the pattern
			if (std::regex_match(jointName, match, motorPattern)) {
				// Extract motor number from the first capture group
				try {
					int motorNumber = std::stoi(match[1].str());
					motorMap[motorNumber] = joint;
					gzdbg << "[MotorFailureSystem] Found motor " << motorNumber
					      << ": " << jointName << std::endl;

				} catch (const std::exception &e) {
					gzwarn << "[MotorFailureSystem] Failed to parse motor number from: "
					       << jointName << std::endl;
				}
			}
		}
	}

	// Convert map to vector for indexed access
	for (const auto &pair : motorMap) {
		// Ensure vector is large enough
		if (pair.first >= static_cast<int>(this->_motor_joints.size())) {
			this->_motor_joints.resize(pair.first + 1, kNullEntity);
		}

		this->_motor_joints[pair.first] = pair.second;
	}

	if (!this->_motor_joints.empty()) {
		gzmsg << "[MotorFailureSystem] Found " << this->_motor_joints.size()
		      << " motor joints" << std::endl;
		this->_joints_found = true;

	} else {
		gzwarn << "[MotorFailureSystem] No motor joints found in model" << std::endl;
	}
}

//////////////////////////////////////////////////
void MotorFailureSystem::ApplyMotorFailure(EntityComponentManager &_ecm)
{
	int32_t current_failure;
	{
		std::lock_guard<std::mutex> lock(this->_motor_failure_mutex);
		current_failure = this->_motor_failure_number;
	}

	// Check if failure status changed
	if (current_failure != this->_prev_motor_failure_number) {
		if (current_failure > 0) {
			gzerr << "[MotorFailureSystem] Motor " << current_failure << " failed!" << std::endl;

		} else if (current_failure == 0 && this->_prev_motor_failure_number > 0) {
			gzerr << "[MotorFailureSystem] Motor " << this->_prev_motor_failure_number
			      << " recovered!" << std::endl;
		}

		this->_prev_motor_failure_number = current_failure;
	}

	// Apply motor failure if active (1-indexed motor number, convert to 0-indexed)
	if (current_failure > 0 && current_failure <= static_cast<int32_t>(this->_motor_joints.size())) {
		int motorIdx = current_failure - 1;
		Entity jointEntity = this->_motor_joints[motorIdx];

		if (jointEntity != kNullEntity) {
			// Force joint velocity command to 0
			// This is done in PreUpdate to override MulticopterMotorModel's commands
			auto jointVelCmd = _ecm.Component<components::JointVelocityCmd>(jointEntity);

			if (jointVelCmd) {
				*jointVelCmd = components::JointVelocityCmd({0.0});
			}
		}
	}
}

//////////////////////////////////////////////////
void MotorFailureSystem::PreUpdate(const UpdateInfo &_info,
				   EntityComponentManager &_ecm)
{
	// Skip if paused
	if (_info.paused) {
		return;
	}

	// Find motor joints on first update
	if (!this->_joints_found) {
		this->FindMotorJoints(_ecm);

	} else {
		this->ApplyMotorFailure(const_cast<EntityComponentManager &>(_ecm));
	}
}

//////////////////////////////////////////////////
void MotorFailureSystem::MotorFailureNumberCallback(const gz::msgs::Int32 &_msg)
{
	std::lock_guard<std::mutex> lock(this->_motor_failure_mutex);
	this->_motor_failure_number = _msg.data();
	gzdbg << "[MotorFailureSystem] Received motor failure number: "
	      << this->_motor_failure_number << std::endl;
}

// Register the plugin
GZ_ADD_PLUGIN(
	MotorFailureSystem,
	gz::sim::System,
	MotorFailureSystem::ISystemConfigure,
	MotorFailureSystem::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(MotorFailureSystem,
		    "gz::sim::systems::MotorFailureSystem")
