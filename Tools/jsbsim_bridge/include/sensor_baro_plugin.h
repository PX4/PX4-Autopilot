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

#pragma once

#include "common.h"
#include "sensor_plugin.h"

class SensorBaroPlugin : public SensorPlugin {
 public:
  SensorBaroPlugin(JSBSim::FGFDMExec* jsbsim);
  ~SensorBaroPlugin();
  void setSensorConfigs(const TiXmlElement& configs);
  SensorData::Barometer getData();

 private:
  float getAirTemperature();
  float getPressureAltitude();
  float getAirPressure();

  double _abs_pressure;

  void addNoise(double abs_pressure, const double dt);

  std::normal_distribution<double> _standard_normal_distribution;

  // state variables for baro pressure sensor random noise generator
  double _baro_rnd_y2{0.};
  double _baro_mbar_rms_noise{1.};
  bool _baro_rnd_use_last{false};
  double _baro_drift_mbar{0.};
  double _baro_drift_mbar_per_sec{0.};

  // Variable for temperature sensor random noise
  double _temperature_stddev{0.02};

  // JSBSim default variables
  std::string _jsb_baro_temp = "atmosphere/T-R";
  std::string _jsb_baro_pressure_alt = "atmosphere/pressure-altitude";
  std::string _jsb_baro_air_pressure = "atmosphere/P-psf";
};
