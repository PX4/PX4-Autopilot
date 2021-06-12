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

#include "sensor_gps_plugin.h"
#include "common.h"

SensorGpsPlugin::SensorGpsPlugin(JSBSim::FGFDMExec* jsbsim) : SensorPlugin(jsbsim) { _update_rate = 1.0; }

SensorGpsPlugin::~SensorGpsPlugin() {}

void SensorGpsPlugin::setSensorConfigs(const TiXmlElement& configs) {
  GetConfigElement<std::string>(configs, "jsb_gps_fix_type", _jsb_gps_fix_type);
  GetConfigElement<std::string>(configs, "jsb_gps_lat", _jsb_gps_lat);
  GetConfigElement<std::string>(configs, "jsb_gps_lon", _jsb_gps_lon);
  GetConfigElement<std::string>(configs, "jsb_gps_alt", _jsb_gps_alt);
  GetConfigElement<std::string>(configs, "jsb_gps_eph", _jsb_gps_eph);
  GetConfigElement<std::string>(configs, "jsb_gps_epv", _jsb_gps_epv);
  GetConfigElement<std::string>(configs, "jsb_gps_v_north", _jsb_gps_v_north);
  GetConfigElement<std::string>(configs, "jsb_gps_v_east", _jsb_gps_v_east);
  GetConfigElement<std::string>(configs, "jsb_gps_v_down", _jsb_gps_v_down);
  GetConfigElement<std::string>(configs, "jsb_gps_velocity", _jsb_gps_velocity);
  GetConfigElement<std::string>(configs, "jsb_gps_satellites", _jsb_gps_satellites);
}

SensorData::Gps SensorGpsPlugin::getData() {
  double sim_time = _sim_ptr->GetSimTime();
  double dt = sim_time - _last_sim_time;

  SensorData::Gps data;

  data = getGpsFromJSBSim();

  _last_sim_time = sim_time;
  return data;
}

SensorData::Gps SensorGpsPlugin::getGpsFromJSBSim() {
  SensorData::Gps ret;
  ret.time_utc_usec = _sim_ptr->GetSimTime() * 1e6;

  if (_jsb_gps_fix_type == "none") {
    ret.fix_type = 3;
  } else {
    ret.fix_type = _sim_ptr->GetPropertyValue(_jsb_gps_fix_type);
  }

  if (_jsb_gps_eph == "none") {
    ret.eph = 1 * 100;
  } else {
    ret.eph = _sim_ptr->GetPropertyValue(_jsb_gps_eph) * 100;
  }

  if (_jsb_gps_epv == "none") {
    ret.epv = 2 * 100;
  } else {
    ret.epv = _sim_ptr->GetPropertyValue(_jsb_gps_epv) * 100;
  }

  if (_jsb_gps_satellites == "none") {
    ret.satellites_visible = 16;
  } else {
    ret.satellites_visible = _sim_ptr->GetPropertyValue(_jsb_gps_satellites);
  }

  ret.latitude_deg = _sim_ptr->GetPropertyValue(_jsb_gps_lat) * 1e7;
  ret.longitude_deg = _sim_ptr->GetPropertyValue(_jsb_gps_lon) * 1e7;
  ret.altitude = _sim_ptr->GetPropertyValue(_jsb_gps_alt) * 1e3;
  ret.velocity_north = ftToM(_sim_ptr->GetPropertyValue(_jsb_gps_v_north)) * 100;
  ret.velocity_east = ftToM(_sim_ptr->GetPropertyValue(_jsb_gps_v_east)) * 100;
  ret.velocity_down = ftToM(_sim_ptr->GetPropertyValue(_jsb_gps_v_down)) * 100;
  ret.velocity = ftToM(_sim_ptr->GetPropertyValue(_jsb_gps_velocity)) * 100;
  ret.cog = wrap_pi_deg(atan2f(ret.velocity_east, ret.velocity_north) * (180 / M_PI)) * 100;

  ret.id = 1;

  return ret;
}
