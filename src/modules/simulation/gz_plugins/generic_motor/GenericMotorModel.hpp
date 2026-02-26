/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_GENERICMOTORMODEL_HPP_
#define GZ_SIM_SYSTEMS_GENERICMOTORMODEL_HPP_

#include <gz/sim/System.hh>
#include <memory>
#include <string>
#include <vector>

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
class GenericMotorModelPrivate;

/// \brief This system applies a thrust force to models with spinning
/// propellers. See examples/worlds/quadcopter.sdf for a demonstration.
class GenericMotorModel
	: public System,
	  public ISystemConfigure,
	  public ISystemPreUpdate
{
	/// \brief Constructor
public: GenericMotorModel();

	/// \brief Destructor
public: ~GenericMotorModel() override = default;

	// Documentation inherited
public: void Configure(const Entity &_entity,
			       const std::shared_ptr<const sdf::Element> &_sdf,
			       EntityComponentManager &_ecm,
			       EventManager &_eventMgr) override;

	// Documentation inherited
public: void PreUpdate(
		const gz::sim::UpdateInfo &_info,
		gz::sim::EntityComponentManager &_ecm) override;

	/// \brief Private data pointer
private: std::unique_ptr<GenericMotorModelPrivate> dataPtr;
private:
	std::vector<double> ParsePolynomial(const std::string &input)
	{
		std::vector<double> result;
		std::string trimmed = input;

		// Optional: remove brackets
		trimmed.erase(std::remove(trimmed.begin(), trimmed.end(), '['), trimmed.end());
		trimmed.erase(std::remove(trimmed.begin(), trimmed.end(), ']'), trimmed.end());

		std::stringstream ss(trimmed);
		std::string token;

		while (std::getline(ss, token, ',')) {
			try {
				result.push_back(std::stod(token));

			} catch (const std::invalid_argument &e) {
				gzerr << "[YourPlugin] Invalid number: " << token << std::endl;
			}
		}

		return result;
	}

};
}
}
}
}

#endif
