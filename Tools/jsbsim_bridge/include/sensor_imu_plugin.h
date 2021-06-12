/****************************************************************************
 *
 *   Copyright (c) 2020 Auterion AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
/**
 * @brief JSBSim IMU Plugin
 *
 * This is a IMU plugin for the JSBSim
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Roman Bapst <roman@auterion.com>
 */

#pragma once

#include "common.h"
#include "sensor_plugin.h"

// Default values for use with ADIS16448 IMU
static constexpr double kDefaultAdisGyroscopeNoiseDensity = 2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeRandomWalk = 2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeBiasCorrelationTime = 1.0e+3;
static constexpr double kDefaultAdisGyroscopeTurnOnBiasSigma = 0.5 / 180.0 * M_PI;
static constexpr double kDefaultAdisAccelerometerNoiseDensity = 2.0 * 2.0e-3;
static constexpr double kDefaultAdisAccelerometerRandomWalk = 2.0 * 3.0e-3;
static constexpr double kDefaultAdisAccelerometerBiasCorrelationTime = 300.0;
static constexpr double kDefaultAdisAccelerometerTurnOnBiasSigma = 20.0e-3 * 9.8;
// Earth's gravity in Zurich (lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84)
static constexpr double kDefaultGravityMagnitude = 9.8068;

class SensorImuPlugin : public SensorPlugin {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SensorImuPlugin(JSBSim::FGFDMExec* jsbsim);
  ~SensorImuPlugin();
  void setSensorConfigs(const TiXmlElement& configs);
  SensorData::Imu getData();

 private:
  Eigen::Vector3d getAccelFromJSBSim();
  Eigen::Vector3d getGyroFromJSBSim();
  void addNoise(Eigen::Vector3d* linear_acceleration, Eigen::Vector3d* angular_velocity, const double dt);

  /// Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)]
  double gyroscope_noise_density{kDefaultAdisGyroscopeNoiseDensity};
  /// Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
  double gyroscope_random_walk{kDefaultAdisGyroscopeRandomWalk};
  /// Gyroscope bias correlation time constant [s]
  double gyroscope_bias_correlation_time{kDefaultAdisGyroscopeBiasCorrelationTime};
  /// Gyroscope turn on bias standard deviation [rad/s]
  double gyroscope_turn_on_bias_sigma{kDefaultAdisGyroscopeTurnOnBiasSigma};
  /// Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)]
  double accelerometer_noise_density{kDefaultAdisAccelerometerNoiseDensity};
  /// Accelerometer bias random walk. [m/s^2/s/sqrt(Hz)]
  double accelerometer_random_walk{kDefaultAdisAccelerometerRandomWalk};
  /// Accelerometer bias correlation time constant [s]
  double accelerometer_bias_correlation_time{kDefaultAdisAccelerometerBiasCorrelationTime};
  /// Accelerometer turn on bias standard deviation [m/s^2]
  double accelerometer_turn_on_bias_sigma{kDefaultAdisAccelerometerTurnOnBiasSigma};
  /// Norm of the gravitational acceleration [m/s^2]
  double gravity_magnitude{kDefaultGravityMagnitude};

  Eigen::Vector3d _gyroscope_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d _accelerometer_bias{Eigen::Vector3d::Zero()};

  Eigen::Vector3d _gyroscope_turn_on_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d _accelerometer_turn_on_bias{Eigen::Vector3d::Zero()};

  std::normal_distribution<double> _standard_normal_distribution;

  /** Accelerations are affected by JSBSim airframe configuration <location name="EYEPOINT">
   * ensure you have set the eyepoint location as to where you expect accelerometer measurements
   * or more appropriately, use the px4_default_imu_sensor.xml configuration.
   */
  std::string _jsb_acc_x = "accelerations/a-pilot-x-ft_sec2";
  std::string _jsb_acc_y = "accelerations/a-pilot-y-ft_sec2";
  std::string _jsb_acc_z = "accelerations/a-pilot-z-ft_sec2";

  // PX4 requires axis angular acceleration vs. body frame acceleration.
  std::string _jsb_gyro_x = "velocities/p-rad_sec";
  std::string _jsb_gyro_y = "velocities/q-rad_sec";
  std::string _jsb_gyro_z = "velocities/r-rad_sec";
};
