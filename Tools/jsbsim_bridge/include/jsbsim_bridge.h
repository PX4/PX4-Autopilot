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
 * @brief JSBSim Bridge
 *
 * JSBSim Bridge object that runs the simulation
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 */

#pragma once

#include "mavlink_interface.h"

#include "actuator_plugin.h"
#include "configuration_parser.h"
#include "sensor_airspeed_plugin.h"
#include "sensor_baro_plugin.h"
#include "sensor_gps_plugin.h"
#include "sensor_imu_plugin.h"
#include "sensor_mag_plugin.h"

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <input_output/FGScript.h>

#include <tinyxml.h>
#include <chrono>

static constexpr int kDefaultSITLTcpPort = 4560;

class JSBSimBridge {
 public:
  JSBSimBridge(JSBSim::FGFDMExec *fdmexec, ConfigurationParser &cfg);
  ~JSBSimBridge();
  void Run();

 private:
  bool SetFdmConfigs(ConfigurationParser &cfg);
  bool SetMavlinkInterfaceConfigs(std::unique_ptr<MavlinkInterface> &interface, TiXmlHandle &config);

  JSBSim::FGFDMExec *_fdmexec;  // FDMExec pointer
  ConfigurationParser &_cfg;

  std::unique_ptr<MavlinkInterface> _mavlink_interface;
  std::unique_ptr<SensorImuPlugin> _imu_sensor;
  std::unique_ptr<SensorGpsPlugin> _gps_sensor;
  std::unique_ptr<SensorBaroPlugin> _baro_sensor;
  std::unique_ptr<SensorMagPlugin> _mag_sensor;
  std::unique_ptr<SensorAirspeedPlugin> _airspeed_sensor;
  std::unique_ptr<ActuatorPlugin> _actuators;

  double _dt{0.004};
  double _realtime_factor{1.0};
  bool _result{true};
};
