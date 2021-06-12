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

#include <tinyxml.h>
#include <cmath>

inline double ftToM(double ft) { return 0.3048 * ft; }
inline double rankineToCelsius(double temp) { return (temp - 491.67) * 5 / 9; }
inline double psfToBar(double pressure) { return 0.000478802588 * pressure; }
inline double psfToMbar(double pressure) { return 0.478802588 * pressure; }

inline double wrap_pi(double x) {
  while (x > M_PI) {
    x -= 2 * M_PI;
  }

  while (x < -M_PI) {
    x += 2 * M_PI;
  }

  return x;
}

inline double wrap_pi_deg(double x) {
  while (x < 0) {
    x += 360;
  }
  return x;
}

inline bool CheckConfigElement(const TiXmlElement &config, std::string param) {
  const TiXmlElement *e = config.FirstChildElement(param);
  return e != nullptr;
}

inline bool CheckConfigElement(const TiXmlHandle &config, std::string group, std::string param) {
  const TiXmlElement *group_element = config.FirstChild(group).Element();
  if (!group_element) {
    return false;
  }

  const TiXmlElement *e = group_element->FirstChildElement(param);
  return e != nullptr;
}

inline TiXmlElement GetXmlElement(const TiXmlHandle &config, std::string group, std::string param) {
  const TiXmlElement *group_element = config.FirstChild(group).Element();
  if (!group_element) {
    return nullptr;
  }

  const TiXmlElement *e = group_element->FirstChildElement(param);
  if (!e) {
    return nullptr;
  }
  return *e;
}

template <typename T>
bool GetConfigElement(const TiXmlHandle &config, const std::string &group, const std::string &param, T &param_value) {
  const TiXmlElement *group_element = config.FirstChild(group).Element();
  if (!group_element) {
    return false;
  }

  const TiXmlElement *e = group_element->FirstChildElement(param);
  if (e) {
    std::istringstream iss(e->GetText());
    iss >> param_value;
    return true;
  }
  return false;
}

template <typename T>
bool GetConfigElement(const TiXmlElement &element, const std::string &param, T &param_value) {
  const TiXmlElement *e = element.FirstChildElement(param);
  if (e) {
    std::istringstream iss(e->GetText());
    iss >> param_value;
    return true;
  }
  return false;
}

template <typename T>
bool GetConfigElement(const TiXmlElement &config, const std::string &group, const std::string &param, T &param_value) {
  const TiXmlElement *group_element = config.FirstChildElement(group);
  if (!group_element) {
    return false;
  }

  const TiXmlElement *e = group_element->FirstChildElement(param);
  if (e) {
    std::istringstream iss(e->GetText());
    iss >> param_value;
    return true;
  }
  return false;
}

inline bool GetConfigElement(const TiXmlHandle &config, const std::string &group, const std::string &param,
                             bool &param_value) {
  const TiXmlElement *group_element = config.FirstChild(group).Element();
  if (!group_element) {
    return false;
  }

  const TiXmlElement *e = group_element->FirstChildElement(param);
  if (e) {
    std::istringstream iss(e->GetText());
    iss >> std::boolalpha >> param_value;
    return true;
  }
  return false;
}
