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

#include "configuration_parser.h"

#include <getopt.h>

bool ConfigurationParser::ParseEnvironmentVariables() {
  if (const char* headless_char = std::getenv("HEADLESS")) {
    _headless = !std::strcmp(headless_char, "1");
  }

  if (const char* realtimefactor_char = std::getenv("PX4_SIM_SPEED_FACTOR")) {
    _realtime_factor = std::stod(realtimefactor_char);
  }
  return true;
}

ArgResult ConfigurationParser::ParseArgV(int argc, char* const argv[]) {
  static const struct option options[] = {
      {"scene", required_argument, nullptr, 's'},
  };

  int c;
  while ((c = getopt_long(argc, argv, "s:h", options, nullptr)) >= 0) {
    switch (c) {
      case 'h': {
        return ArgResult::Help;
        break;
      }
      case 's': {
        _init_script_path = std::string(optarg);
        break;
      }
      case '?':
      default: {
        std::cout << "Unknown Options" << std::endl;
        return ArgResult::Error;
      }
    }
  }

  return ArgResult::Success;
}

bool ConfigurationParser::ParseConfigFile(const std::string& path) {
  _doc = TiXmlDocument(path);
  if (!_doc.LoadFile()) {
    std::cerr << "[ConfigurationParser] Could not load actuator configs from configuration file: " << path << std::endl;
    return false;
  }
  _config = std::make_shared<TiXmlHandle>(_doc.RootElement());

  TiXmlElement* model_config = _config->Element();
  if (model_config) {
    _model_name = model_config->Attribute("name");
  } else {
    std::cerr << "[ConfigurationParser] Incorrect or invalid model name" << std::endl;
    return false;
  }

  return true;
}

void ConfigurationParser::PrintHelpMessage(char* argv[]) {
  std::cout << argv[0] << " aircraft [options]\n\n"
            << "  aircraft      Aircraft config file name e.g. rascal"
            << "  -h | --help   Print available options\n"
            << "  -s | --scene  Location / scene where the vehicle should be spawned in e.g. LSZH\n";
}
