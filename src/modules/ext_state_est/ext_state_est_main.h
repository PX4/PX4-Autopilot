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

/**
 * @file ext_state_est_main.h
 * Passtrough for external state to the controller
 *
 * @author Christian Brommer and Alessandro Fornasier
 */

#pragma once

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/ext_core_state.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

class ExtStateEst final : public ModuleBase<ExtStateEst>,
                          public ModuleParams,
                          public px4::ScheduledWorkItem {

public:
  explicit ExtStateEst();
  ~ExtStateEst() override;

  /** @see ModuleBase */
  static int task_spawn(int argc, char *argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char *argv[]);

  /** @see ModuleBase */
  static int print_usage(const char *reason = nullptr);

  bool init();

  int print_status() override;

private:
  /** @see ModuleBase::run() */
  void Run() override;

  perf_counter_t _ext_state_perf;

  uORB::SubscriptionCallbackWorkItem _ext_state_sub;
  uORB::Publication<vehicle_attitude_s> _att_pub;
  uORB::PublicationData<vehicle_global_position_s> _vehicle_global_position_pub;
  uORB::PublicationData<vehicle_local_position_s> _vehicle_local_position_pub;

  bool _callback_registered{false};
};

extern "C" __EXPORT int ext_state_est_main(int argc, char *argv[]) {
  return ExtStateEst::main(argc, argv);
}
