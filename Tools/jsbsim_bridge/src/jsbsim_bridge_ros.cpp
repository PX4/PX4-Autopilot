
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
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 *
 */

#include "jsbsim_bridge_ros.h"

using namespace Eigen;
using namespace std;
// Constructor
JSBSimBridgeRos::JSBSimBridgeRos(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // Path to config file
  std::string path;
  double dt;
  nh_private_.param<string>("config", path, std::string(JSBSIM_ROOT_DIR) + "/configs/rascal.xml");
  nh_private_.param<string>("script", script_path, std::string(JSBSIM_ROOT_DIR) + "/scene/LSZH.xml");
  nh_private_.param<double>("dt", dt, 0.004);

  simloop_timer_ = nh_.createTimer(ros::Duration(dt), &JSBSimBridgeRos::simloopCallback,
                                   this);  // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &JSBSimBridgeRos::statusloopCallback,
                                      this);  // Define timer for constant loop rate

  // Parse Configurations
  config_.ParseEnvironmentVariables();
  config_.ParseConfigFile(path);
  config_.setInitScriptPath(script_path);
  config_.setHeadless(true);

  fdmexec_ = new JSBSim::FGFDMExec();
  jsbsim_bridge_ = std::make_unique<JSBSimBridge>(fdmexec_, config_);
}
JSBSimBridgeRos::~JSBSimBridgeRos() {
  // Destructor
}

void JSBSimBridgeRos::simloopCallback(const ros::TimerEvent &event) {
  if (jsbsim_bridge_) {
    jsbsim_bridge_->Run();
  }
}

void JSBSimBridgeRos::statusloopCallback(const ros::TimerEvent &event) {
  // TODO: Publish simulation status
}
