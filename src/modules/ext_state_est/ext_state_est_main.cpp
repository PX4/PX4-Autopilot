/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "ext_state_est_main.h"

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/ext_core_state.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>

int ExtStateEst::print_status() {
  perf_print_counter(_ext_state_perf);
  return 0;
}

int ExtStateEst::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int ExtStateEst::task_spawn(int argc, char *argv[]) {
  ExtStateEst *instance = new ExtStateEst();

  if (instance) {
    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->init()) {
      return PX4_OK;
    }

  } else {
    PX4_ERR("alloc failed");
  }

  delete instance;
  _object.store(nullptr);
  _task_id = -1;

  return PX4_ERROR;
}

bool ExtStateEst::init() {
  bool callback_status = true;
  if (!_mocap_odom_sub.registerCallback()) {
    callback_status = false;
  }
  if (!_ext_state_sub.registerCallback()) {
    callback_status = false;
  }

  if (callback_status) {
    PX4_INFO("Callbacks registered");
    _callback_registered = true;
    return true;
  }

  PX4_WARN("failed to register callback, retrying in 1 second");
  ScheduleDelayed(1000000); // retry in 1 second

  return false;
}

ExtStateEst::ExtStateEst()
    : ModuleParams(nullptr),
      ScheduledWorkItem(MODULE_NAME,
                        px4::wq_configurations::navigation_and_controllers),
      _ext_state_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": update")),
      _ext_state_sub{this, ORB_ID(ext_core_state)},
      _mocap_odom_sub{this, ORB_ID(vehicle_mocap_odometry)},
      _att_pub{ORB_ID(vehicle_attitude)},
      _vehicle_global_position_pub{ORB_ID(vehicle_global_position)},
      _vehicle_local_position_pub{ORB_ID(vehicle_local_position)} {}

ExtStateEst::~ExtStateEst() { perf_free(_ext_state_perf); }

void ExtStateEst::Run() {

  if (should_exit()) {
    _ext_state_sub.unregisterCallback();
    _mocap_odom_sub.unregisterCallback();
    exit_and_cleanup();
    return;
  }

  if (!_callback_registered) {
    init();
    return;
  }

  perf_begin(_ext_state_perf);

  vehicle_odometry_s ext_state_in;
  if (_mocap_odom_sub.update(&ext_state_in)) {
  }

  if (_ext_state_sub.update(&ext_state_in)) {

    uint64_t timestamp = ext_state_in.timestamp;

    vehicle_local_position_s &position = _vehicle_local_position_pub.get();
    position.timestamp = timestamp;
    position.x = ext_state_in.x;
    position.y = ext_state_in.y;
    position.z = ext_state_in.z;

    vehicle_attitude_s attitude;
    attitude.timestamp = timestamp;
    attitude.q[0] = ext_state_in.q[0];
    attitude.q[1] = ext_state_in.q[1];
    attitude.q[2] = ext_state_in.q[2];
    attitude.q[3] = ext_state_in.q[3];

    _vehicle_local_position_pub.update();
    _att_pub.publish(attitude);
  }

  perf_end(_ext_state_perf);
}

int ExtStateEst::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  return 0;
}
