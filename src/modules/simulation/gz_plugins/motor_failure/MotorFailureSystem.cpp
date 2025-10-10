/****************************************************************************
 *
 *   Copyright (c) 2017 Nuno Marques, PX4 Pro Dev Team, Lisbon
 *   Copyright (c) 2017 Siddharth Patel, NTU Singapore
 *   Copyright (c) 2022 SungTae Moon, KOREATECH Korea
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
MotorFailureSystem::MotorFailureSystem()
{
}

//////////////////////////////////////////////////
MotorFailureSystem::~MotorFailureSystem()
{
	// Clean up ROS2 resources safely
	// If ROS2 has already been shutdown, reset() operations are safe
	if (this->ros_node_) {
		try {
			// Reset subscription first
			if (this->motor_failure_sub_) {
				this->motor_failure_sub_.reset();
			}

			// Then reset the node
			this->ros_node_.reset();

			gzdbg << "[MotorFailureSystem] Cleaned up ROS2 resources" << std::endl;

		} catch (const std::exception &e) {
			// Ignore exceptions during shutdown - ROS2 may have already shut down
			gzdbg << "[MotorFailureSystem] Exception during cleanup (expected if ROS2 already shutdown): "
			      << e.what() << std::endl;
		}
	}
}

//////////////////////////////////////////////////
void MotorFailureSystem::Configure(const Entity &_entity,
				   const std::shared_ptr<const sdf::Element> &_sdf,
				   EntityComponentManager &_ecm,
				   EventManager &/*_eventMgr*/)
{
	this->model_ = Model(_entity);
	this->model_entity_ = _entity;

	if (!this->model_.Valid(_ecm)) {
		gzerr << "[MotorFailureSystem] plugin should be attached to a model "
		      << "entity. Failed to initialize." << std::endl;
		return;
	}

	// Get robot namespace from SDF
	if (_sdf->HasElement("robotNamespace")) {
		this->robot_namespace_ = _sdf->Get<std::string>("robotNamespace");

	} else {
		gzwarn << "[MotorFailureSystem] No robotNamespace set, using entity name.\n";
		this->robot_namespace_ = this->model_.Name(_ecm);
	}

	// Get ROS2 topic name for motor failure number subscription
	if (_sdf->HasElement("ROSMotorNumSubTopic")) {
		this->ros_topic_ = _sdf->Get<std::string>("ROSMotorNumSubTopic");

	} else {
		// Use namespace + default topic name
		this->ros_topic_ = "/" + this->robot_namespace_ + "/motor_failure/motor_number";
	}

	// Initialize ROS2, if it has not already been initialized
	if (!rclcpp::ok()) {
		int argc = 0;
		char **argv = nullptr;
		rclcpp::init(argc, argv);
	}

	// Create ROS2 node
	this->ros_node_ = rclcpp::Node::make_shared("motor_failure");

	// Create ROS2 subscription
	this->motor_failure_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Int32>(
					   this->ros_topic_, 10,
					   std::bind(&MotorFailureSystem::MotorFailureNumberCallback, this, std::placeholders::_1));

	gzmsg << "[MotorFailureSystem] Subscribed to ROS2 topic: " << this->ros_topic_ << std::endl;
	gzmsg << "[MotorFailureSystem] Initialized for model: " << this->robot_namespace_ << std::endl;
}

//////////////////////////////////////////////////
void MotorFailureSystem::FindMotorJoints(EntityComponentManager &_ecm)
{
	if (this->joints_found_) {
		return;
	}

	// Find all joints with "rotor_X_joint" pattern
	this->motor_joints_.clear();

	// Get all joints in the model
	auto joints = this->model_.Joints(_ecm);

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
		if (pair.first >= static_cast<int>(this->motor_joints_.size())) {
			this->motor_joints_.resize(pair.first + 1, kNullEntity);
		}

		this->motor_joints_[pair.first] = pair.second;
	}

	if (!this->motor_joints_.empty()) {
		gzmsg << "[MotorFailureSystem] Found " << this->motor_joints_.size()
		      << " motor joints" << std::endl;
		this->joints_found_ = true;

	} else {
		gzwarn << "[MotorFailureSystem] No motor joints found in model" << std::endl;
	}
}

//////////////////////////////////////////////////
void MotorFailureSystem::ApplyMotorFailure(EntityComponentManager &_ecm)
{
	int32_t current_failure;
	{
		std::lock_guard<std::mutex> lock(this->motor_failure_mutex_);
		current_failure = this->motor_failure_number_;
	}

	// Check if failure status changed
	if (current_failure != this->prev_motor_failure_number_) {
		if (current_failure > 0) {
			gzerr << "[MotorFailureSystem] Motor " << current_failure << " failed!" << std::endl;

		} else if (current_failure == 0 && this->prev_motor_failure_number_ > 0) {
			gzerr << "[MotorFailureSystem] Motor " << this->prev_motor_failure_number_
			      << " recovered!" << std::endl;
		}

		this->prev_motor_failure_number_ = current_failure;
	}

	// Apply motor failure if active (1-indexed from ROS2, convert to 0-indexed)
	if (current_failure > 0 && current_failure <= static_cast<int32_t>(this->motor_joints_.size())) {
		int motorIdx = current_failure - 1;
		Entity jointEntity = this->motor_joints_[motorIdx];

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

	// Spin ROS2 to process callbacks
	// Check if ROS2 context is still valid before spinning
	if (this->ros_node_ && rclcpp::ok()) {
		rclcpp::spin_some(this->ros_node_);
	}

	// Find motor joints on first update
	if (!this->joints_found_) {
		this->FindMotorJoints(_ecm);

	} else {
		this->ApplyMotorFailure(const_cast<EntityComponentManager &>(_ecm));
	}
}

//////////////////////////////////////////////////
void MotorFailureSystem::MotorFailureNumberCallback(const std_msgs::msg::Int32::SharedPtr _msg)
{
	std::lock_guard<std::mutex> lock(this->motor_failure_mutex_);
	this->motor_failure_number_ = _msg->data;
	gzdbg << "[MotorFailureSystem] Received motor failure number: "
	      << this->motor_failure_number_ << std::endl;
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
