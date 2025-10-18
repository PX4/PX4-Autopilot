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

#ifndef GZ_SIM_SYSTEMS_MOTORFAILURESYSTEM_HPP_
#define GZ_SIM_SYSTEMS_MOTORFAILURESYSTEM_HPP_

#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <map>
#include <regex>

// Gazebo Transport includes
#include <gz/transport/Node.hh>
#include <gz/msgs/int32.pb.h>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{

/// \brief This system subscribes to a Gazebo Transport topic to receive motor failure
/// commands and directly controls motor joints to simulate failures.
/// This allows simulating motor failures in multirotor vehicles.
class MotorFailureSystem:
	public System,
	public ISystemConfigure,
	public ISystemPreUpdate
{
public:
	/// \brief Constructor
	MotorFailureSystem();

	/// \brief Destructor
	~MotorFailureSystem() override;

	// Documentation inherited
	void Configure(const Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       EntityComponentManager &_ecm,
		       EventManager &_eventMgr) override;

	// Documentation inherited
	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) override;

private:
	/// \brief Callback for Gazebo Transport motor failure number subscription
	void MotorFailureNumberCallback(const gz::msgs::Int32 &_msg);

	/// \brief Find all motor joints in the model
	void FindMotorJoints(gz::sim::EntityComponentManager &_ecm);

	/// \brief Apply motor failure (set velocity to 0)
	void ApplyMotorFailure(gz::sim::EntityComponentManager &_ecm);

	/// \brief Gazebo Transport node for communication
	gz::transport::Node node_;

	/// \brief Model entity
	gz::sim::Entity model_entity_;

	/// \brief Model interface
	gz::sim::Model model_;

	/// \brief Vector of motor joint entities (indexed by motor number)
	std::vector<gz::sim::Entity> motor_joints_;

	/// \brief Current motor failure number (-1 or 0 means no failure, 1-indexed motor number)
	int32_t motor_failure_number_{-1};

	/// \brief Previous motor failure number to detect changes
	int32_t prev_motor_failure_number_{-1};

	/// \brief Gazebo Transport topic name for subscribing to motor failure commands
	/// Defaults to /model/<model_name>/motor_failure/motor_number if not specified in SDF
	std::string gz_topic_{"/motor_failure/motor_number"};

	/// \brief Mutex to protect motor_failure_number_
	std::mutex motor_failure_mutex_;

	/// \brief Flag to indicate if motor joints have been found
	bool joints_found_{false};
};

}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_SYSTEMS_MOTORFAILURESYSTEM_HPP_
