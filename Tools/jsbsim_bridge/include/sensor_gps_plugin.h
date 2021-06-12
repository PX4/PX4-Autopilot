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
 * @brief JSBSim GPS Plugin
 *
 * This is a GPS plugin for the JSBSim
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Roman Bapst <roman@auterion.com>
 */

#pragma once

#include "common.h"
#include "sensor_plugin.h"

class SensorGpsPlugin : public SensorPlugin {
 public:
  SensorGpsPlugin(JSBSim::FGFDMExec* jsbsim);
  ~SensorGpsPlugin();
  void setSensorConfigs(const TiXmlElement& configs);
  SensorData::Gps getData();

 private:
  SensorData::Gps getGpsFromJSBSim();
  std::string _jsb_gps_fix_type = "none";
  std::string _jsb_gps_lat = "position/lat-geod-deg";
  std::string _jsb_gps_lon = "position/long-gc-deg";
  std::string _jsb_gps_alt = "position/h-sl-meters";
  std::string _jsb_gps_eph = "none";
  std::string _jsb_gps_epv = "none";
  std::string _jsb_gps_v_north = "velocities/v-north-fps";
  std::string _jsb_gps_v_east = "velocities/v-east-fps";
  std::string _jsb_gps_v_down = "velocities/v-down-fps";
  std::string _jsb_gps_velocity = "velocities/ned-velocity-mag-fps";
  std::string _jsb_gps_satellites = "none";
};
