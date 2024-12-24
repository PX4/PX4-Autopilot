/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef ODOMETER_HH_
#define ODOMETER_HH_

#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/transport/Node.hh>

namespace custom
{
  /// \brief Example sensor that publishes the total distance travelled by a
  /// robot, with noise.
  class Odometer : public gz::sensors::Sensor
  {
    /// \brief Load the sensor with SDF parameters.
    /// \param[in] _sdf SDF Sensor parameters.
    /// \return True if loading was successful
    public: virtual bool Load(const sdf::Sensor &_sdf) override;

    /// \brief Update the sensor and generate data
    /// \param[in] _now The current time
    /// \return True if the update was successfull
    public: virtual bool Update(
      const std::chrono::steady_clock::duration &_now) override;

    /// \brief Set the current postiion of the robot, so the odometer can
    /// calculate the distance travelled.
    /// \param[in] _pos Current position in world coordinates.
    public: void NewPosition(const gz::math::Vector3d &_pos);

    /// \brief Get the latest world postiion of the robot.
    /// \return The latest position given to the odometer.
    public: const gz::math::Vector3d &Position() const;

    /// \brief Previous position of the robot.
    private: gz::math::Vector3d prevPos{std::nan(""), std::nan(""),
        std::nan("")};

    /// \brief Latest total distance.
    private: double totalDistance{0.0};

    /// \brief Noise that will be applied to the sensor data
    private: gz::sensors::NoisePtr noise{nullptr};

    /// \brief Node for communication
    private: gz::transport::Node node;

    /// \brief Publishes sensor data
    private: gz::transport::Node::Publisher pub;
  };
}

#endif
