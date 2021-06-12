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
 * @brief JSBSim Barometer Plugin
 *
 * This is a Barometer plugin for the JSBSim
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 * @author Roman Bapst <roman@auterion.com>
 */

#include "sensor_baro_plugin.h"

SensorBaroPlugin::SensorBaroPlugin(JSBSim::FGFDMExec* jsbsim) : SensorPlugin(jsbsim) {
  _standard_normal_distribution = std::normal_distribution<double>(0.0, 1.0);
}

SensorBaroPlugin::~SensorBaroPlugin() {}

void SensorBaroPlugin::setSensorConfigs(const TiXmlElement& configs) {
  GetConfigElement<double>(configs, "baro_drift_mbar_per_sec", _baro_drift_mbar_per_sec);
  GetConfigElement<double>(configs, "baro_mbar_rms_noise", _baro_mbar_rms_noise);
  GetConfigElement<double>(configs, "temperature_stddev", _temperature_stddev);
  GetConfigElement<std::string>(configs, "jsb_baro_temp", _jsb_baro_temp);
  GetConfigElement<std::string>(configs, "jsb_baro_pressure_alt", _jsb_baro_pressure_alt);
  GetConfigElement<std::string>(configs, "jsb_baro_air_pressure", _jsb_baro_air_pressure);
}

SensorData::Barometer SensorBaroPlugin::getData() {
  double sim_time = _sim_ptr->GetSimTime();
  double dt = sim_time - _last_sim_time;

  double temperature = getAirTemperature() + _temperature_stddev * _standard_normal_distribution(_random_generator);
  double pressure_alt = getPressureAltitude();  // No noise added as PX4 does not utilize this data [27 Oct 20]

  _abs_pressure = getAirPressure();

  SensorData::Barometer data;

  addNoise(_abs_pressure, dt);

  data.temperature = temperature;
  data.abs_pressure = _abs_pressure;
  data.pressure_alt = pressure_alt;

  _last_sim_time = sim_time;
  return data;
}

float SensorBaroPlugin::getAirTemperature() { return rankineToCelsius(_sim_ptr->GetPropertyValue(_jsb_baro_temp)); }

float SensorBaroPlugin::getPressureAltitude() { return ftToM(_sim_ptr->GetPropertyValue(_jsb_baro_pressure_alt)); }

float SensorBaroPlugin::getAirPressure() { return psfToMbar(_sim_ptr->GetPropertyValue(_jsb_baro_air_pressure)); }

void SensorBaroPlugin::addNoise(double abs_pressure, const double dt) {
  if (dt <= 0.0) return;

  // generate Gaussian noise sequence using polar form of Box-Muller transformation
  double y1;
  {
    double x1, x2, w;
    if (!_baro_rnd_use_last) {
      do {
        x1 = 2.0 * _standard_normal_distribution(_random_generator) - 1.0;
        x2 = 2.0 * _standard_normal_distribution(_random_generator) - 1.0;
        w = x1 * x1 + x2 * x2;
      } while (w >= 1.0);
      w = sqrt((-2.0 * log(w)) / w);
      // calculate two values - the second value can be used next time because it is uncorrelated
      y1 = x1 * w;
      _baro_rnd_y2 = x2 * w;
      _baro_rnd_use_last = true;
    } else {
      // no need to repeat the calculation - use the second value from last update
      y1 = _baro_rnd_y2;
      _baro_rnd_use_last = false;
    }
  }

  // Apply noise and drift
  const float abs_pressure_noise = _baro_mbar_rms_noise * (float)y1;
  _baro_drift_mbar += _baro_drift_mbar_per_sec * dt;

  _abs_pressure = _abs_pressure + abs_pressure_noise + _baro_drift_mbar;
}
