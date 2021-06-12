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
/*
 * @brief JSBSim Actuator Plugin
 *
 * This is a class for the JSBSim actuator plugin
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Roman Bapst <roman@auterion.com>
 */

#include "actuator_plugin.h"

ActuatorPlugin::ActuatorPlugin(JSBSim::FGFDMExec *jsbsim) : _sim_ptr(jsbsim), _last_sim_time(0.0) {}

ActuatorPlugin::~ActuatorPlugin() {}

bool ActuatorPlugin::SetActuatorCommands(const Eigen::VectorXd &actuator_commands) {
  for (size_t i = 0; i < _actuator_configs.size(); i++) {
    size_t index = _actuator_configs[i].index;
    std::string property = _actuator_configs[i].property;
    double scale = _actuator_configs[i].scale;
    SetCommandToProperty(scale * actuator_commands[index], property);
  }
  return true;
}

bool ActuatorPlugin::SetCommandToProperty(float value, std::string property) {
  _sim_ptr->SetPropertyValue(property, value);
  return true;
}

bool ActuatorPlugin::SetActuatorConfigs(const TiXmlHandle &config) {
  TiXmlElement *actuators = config.FirstChild("actuators").Element();

  if (!actuators) return false;

  for (TiXmlElement *e = actuators->FirstChildElement("channel"); e != NULL; e = e->NextSiblingElement("channel")) {
    ActuatorMap actuator_mapping;
    actuator_mapping.index = std::stoi(e->FirstChildElement("index")->GetText());
    actuator_mapping.scale = std::stoi(e->FirstChildElement("scale")->GetText());
    actuator_mapping.property = e->FirstChildElement("property")->GetText();
    _actuator_configs.push_back(actuator_mapping);
  }

  return true;
}
