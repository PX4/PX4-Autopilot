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
 * @brief JSBSim Magnetometer Plugin
 *
 * This is a plugin modeling a magnetometer for JSBSim
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Roman Bapst <roman@auterion.com>
 */

#pragma once

#include "common.h"
#include "geo_mag_declination.h"
#include "sensor_plugin.h"

static constexpr auto kDefaultPubRate = 100.0;  // [Hz]. Note: corresponds to most of the mag devices supported in PX4

// Default values for use with ADIS16448 IMU
static constexpr auto kDefaultNoiseDensity = 0.4 * 1e-3;     // [gauss / sqrt(hz)]
static constexpr auto kDefaultRandomWalk = 6.4 * 1e-6;       // [gauss * sqrt(hz)]
static constexpr auto kDefaultBiasCorrelationTime = 6.0e+2;  // [s]

class SensorMagPlugin : public SensorPlugin {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SensorMagPlugin(JSBSim::FGFDMExec* jsbsim);
  ~SensorMagPlugin();
  void setSensorConfigs(const TiXmlElement& configs);
  SensorData::Magnetometer getData();

 private:
  Eigen::Vector3d getMagFromJSBSim();
  void addNoise(Eigen::Vector3d* magnetic_field, const double dt);

  std::normal_distribution<double> _standard_normal_distribution;
  double _noise_density{kDefaultNoiseDensity};
  double _random_walk{kDefaultRandomWalk};
  double _bias_correlation_time{kDefaultBiasCorrelationTime};

  Eigen::Vector3d _bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d _mag_g{Eigen::Vector3d::Zero()};

  // Default settings
  std::string _jsb_mag_lat = "position/lat-gc-deg";
  std::string _jsb_mag_lon = "position/lon-gc-deg";
  std::string _jsb_mag_roll = "attitude/roll-rad";
  std::string _jsb_mag_pitch = "attitude/pitch-rad";
  std::string _jsb_mag_hdg = "attitude/heading-true-rad";

  // For use with px4_mag_sensor system file
  std::string _jsb_mag_x = "none";
  std::string _jsb_mag_y = "none";
  std::string _jsb_mag_z = "none";
};
