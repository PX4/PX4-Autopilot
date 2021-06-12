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
 * @brief JSBSim Bridge Configuration Parser
 *
 * This is a class for the JSBSim actuator plugin
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 */

#pragma once

#include "common.h"

#include <tinyxml.h>
#include <Eigen/Eigen>
#include <memory>

enum class ArgResult { Success, Help, Error };

class ConfigurationParser {
 public:
  ConfigurationParser() = default;
  ~ConfigurationParser() = default;
  bool ParseEnvironmentVariables();
  bool ParseConfigFile(const std::string& path);
  ArgResult ParseArgV(int argc, char* const argv[]);
  bool isHeadless() { return _headless; }
  std::shared_ptr<TiXmlHandle> XmlHandle() { return _config; }
  std::string getInitScriptPath() { return _init_script_path; }
  std::string getModelName() { return _model_name; }
  int getRealtimeFactor() { return _realtime_factor; }
  void setHeadless(bool headless) { _headless = headless; }
  void setInitScriptPath(std::string path) { _init_script_path = path; }
  static void PrintHelpMessage(char* argv[]);

 private:
  TiXmlDocument _doc;
  std::shared_ptr<TiXmlHandle> _config;

  bool _headless{false};
  std::string _init_script_path;
  std::string _model_name;
  float _realtime_factor{1.0};
};
