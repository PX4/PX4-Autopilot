
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
 * @file jsbsim_bridge.cpp
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Roman Bapst <roman@auterion.com>
 *
 * Mavlink HIL message interface to FlightGear and PX4
 */

#include "jsbsim_bridge.h"

JSBSimBridge::JSBSimBridge(JSBSim::FGFDMExec *fdmexec, ConfigurationParser &cfg) : _fdmexec(fdmexec), _cfg(cfg) {
  TiXmlHandle config = *_cfg.XmlHandle();

  // Config JSBSim FDM
  SetFdmConfigs(_cfg);
  _fdmexec->Setdt(_dt);
  _fdmexec->RunIC();

  // Configure Mavlink HIL interface
  _mavlink_interface = std::make_unique<MavlinkInterface>();
  SetMavlinkInterfaceConfigs(_mavlink_interface, config);

  _mavlink_interface->Load();

  // Instantiate sensors
  if (CheckConfigElement(config, "sensors", "imu")) {
    _imu_sensor = std::make_unique<SensorImuPlugin>(_fdmexec);
    _imu_sensor->setSensorConfigs(GetXmlElement(config, "sensors", "imu"));
  } else {
    std::cerr << "Could not find IMU sensor " << std::endl;
    return;
  }

  if (CheckConfigElement(config, "sensors", "gps")) {
    _gps_sensor = std::make_unique<SensorGpsPlugin>(_fdmexec);
    _gps_sensor->setSensorConfigs(GetXmlElement(config, "sensors", "gps"));
  }

  if (CheckConfigElement(config, "sensors", "barometer")) {
    _baro_sensor = std::make_unique<SensorBaroPlugin>(_fdmexec);
    _baro_sensor->setSensorConfigs(GetXmlElement(config, "sensors", "barometer"));
  }

  if (CheckConfigElement(config, "sensors", "magnetometer")) {
    _mag_sensor = std::make_unique<SensorMagPlugin>(_fdmexec);
    _mag_sensor->setSensorConfigs(GetXmlElement(config, "sensors", "magnetometer"));
  }

  if (CheckConfigElement(config, "sensors", "airspeed")) {
    _airspeed_sensor = std::make_unique<SensorAirspeedPlugin>(_fdmexec);
    _airspeed_sensor->setSensorConfigs(GetXmlElement(config, "sensors", "airspeed"));
  }

  _actuators = std::make_unique<ActuatorPlugin>(_fdmexec);
  _actuators->SetActuatorConfigs(config);

  _realtime_factor = _cfg.getRealtimeFactor();
}

JSBSimBridge::~JSBSimBridge() {}

bool JSBSimBridge::SetFdmConfigs(ConfigurationParser &cfg) {
  const TiXmlElement *config = cfg.XmlHandle()->FirstChild("jsbsimbridge").Element();

  _fdmexec->SetRootDir(SGPath(JSBSIM_ROOT_DIR));

  // Define aircraft path configuration
  std::string aircraft_path;
  if (config && CheckConfigElement(*config, "aircraft_directory")) {
    GetConfigElement<std::string>(*config, "aircraft_directory", aircraft_path);
  } else {
    aircraft_path = "models/" + cfg.getModelName();
  }

  // Define aircraft model name configuration
  std::string aircraft_model;
  if (config && CheckConfigElement(*config, "aircraft_model")) {
    GetConfigElement<std::string>(*config, "aircraft_model", aircraft_model);
  } else {
    aircraft_model = cfg.getModelName();
  }

  // Check if HEADLESS mode is enabled
  if (!cfg.isHeadless()) {
    _fdmexec->SetOutputDirectives(SGPath("data_out/flightgear.xml"));
  }

  // Define JSBSim initialization script (scene or world)
  SGPath init_script_path = SGPath::fromLocal8Bit((cfg.getInitScriptPath()).c_str());

  // Set JSBSim paths
  _fdmexec->SetEnginePath(SGPath("Engines"));
  _fdmexec->SetSystemsPath(SGPath("systems"));

  // Select & Load JSBSim Run Configuration
  std::string jsb_script;
  if (config && CheckConfigElement(*config, "jsb_script")) {
    std::size_t found = aircraft_path.rfind(aircraft_model);
    if (found == std::string::npos) {
      std::cout << "JSBSIM SCRIPT LOADING DOES NOT SUPPORT: " << aircraft_path << " <> " << aircraft_model << std::endl;
      return false;
    } else {
      _fdmexec->SetAircraftPath(SGPath("models/"));
      GetConfigElement<std::string>(*config, "jsb_script", jsb_script);
      _fdmexec->LoadScript(SGPath("scenario/" + jsb_script), _dt, SGPath(init_script_path));
      return true;
    }
  } else {
    _fdmexec->SetAircraftPath(SGPath(aircraft_path.c_str()));
    _fdmexec->LoadModel(aircraft_model.c_str(), false);
    auto initial_condition = _fdmexec->GetIC();
    initial_condition->Load(SGPath(init_script_path), false);
    return true;
  }
}

bool JSBSimBridge::SetMavlinkInterfaceConfigs(std::unique_ptr<MavlinkInterface> &interface, TiXmlHandle &config) {
  TiXmlElement *mavlink_configs = config.FirstChild("mavlink_interface").Element();

  if (!mavlink_configs) return true;  // Nothing to set

  int tcp_port = kDefaultSITLTcpPort;
  GetConfigElement<int>(config, "mavlink_interface", "tcp_port", tcp_port);
  bool enable_lockstep = true;
  GetConfigElement(config, "mavlink_interface", "enable_lockstep", enable_lockstep);

  interface->SetMavlinkTcpPort(tcp_port);
  interface->SetUseTcp(true);
  interface->SetEnableLockstep(enable_lockstep);

  return true;
}

void JSBSimBridge::Run() {
  // Get Simulation time from JSBSim
  auto step_start_time = std::chrono::system_clock::now();
  double simtime = _fdmexec->GetSimTime();

  // Update sensor messages
  if (_imu_sensor && _imu_sensor->updated()) {
    // Only send sensor messages when the imu sensor is updated.
    // This is needed for lockstep
    _mavlink_interface->UpdateIMU(_imu_sensor->getData());

    if (_mag_sensor && _mag_sensor->updated()) {
      _mavlink_interface->UpdateMag(_mag_sensor->getData());
    }

    if (_baro_sensor && _baro_sensor->updated()) {
      _mavlink_interface->UpdateBarometer(_baro_sensor->getData());
    }

    if (_airspeed_sensor && _airspeed_sensor->updated()) {
      _mavlink_interface->UpdateAirspeed(_airspeed_sensor->getData());
    }

    // Send Mavlink HIL_SENSOR message
    _mavlink_interface->SendSensorMessages(simtime * 1e6);
  }

  // Send Mavlink HIL_GPS message
  if (_gps_sensor && _gps_sensor->updated()) {
    _mavlink_interface->SendGpsMessages(_gps_sensor->getData());
  }

  // Receive and handle actuator controls
  _mavlink_interface->pollForMAVLinkMessages();
  Eigen::VectorXd actuator_controls = _mavlink_interface->GetActuatorControls();

  if (actuator_controls.size() >= 16) {
    _actuators->SetActuatorCommands(actuator_controls);
  }

  _result = _fdmexec->Run();

  auto step_stop_time = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_time = step_start_time - step_stop_time;
  if (_realtime_factor > 0) {
    double sleep = _dt / _realtime_factor - elapsed_time.count();
    if (sleep > 0) usleep(sleep * 1e6);
  }
}
