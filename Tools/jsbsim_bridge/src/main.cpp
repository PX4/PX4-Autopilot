
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
 * @file main.cpp
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 *
 */

#include "jsbsim_bridge.h"

int main(int argc, char *argv[]) {
  // Parse Configurations
  ConfigurationParser config;
  if (argc > 1) {
    // Path to config file
    std::string path = std::string(JSBSIM_ROOT_DIR) + "/configs/" + std::string(argv[1]) + ".xml";
    config.ParseConfigFile(path);
  }
  switch (config.ParseArgV(argc, argv)) {
    case ArgResult::Success: {
      break;
    }
    case ArgResult::Help: {
      ConfigurationParser::PrintHelpMessage(argv);
      return 0;
    }
    default:
    case ArgResult::Error: {
      ConfigurationParser::PrintHelpMessage(argv);
      return 1;
    }
  }
  config.ParseEnvironmentVariables();

  // Configure JSBSim
  JSBSim::FGFDMExec *fdmexec = new JSBSim::FGFDMExec();

  std::unique_ptr<JSBSimBridge> jsbsim_bridge = std::make_unique<JSBSimBridge>(fdmexec, config);

  while (true) {
    jsbsim_bridge->Run();
  }
}
