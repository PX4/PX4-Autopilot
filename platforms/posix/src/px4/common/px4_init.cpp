/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_manager.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform_common/tasks.h>
#include <set>
#include <uORB/uORB.h>
#include <hrt_work.h>
#include <px4_platform_common/workqueue.h>

#if defined(CONFIG_MODULES_MUORB_APPS)
extern "C" {
int muorb_init();
}
#endif

#define PX4_CRITICAL(v) { int ret = v; if (ret != PX4_OK) PX4_PANIC(#v  "failed > %d" , ret);}

int px4_platform_init(void) {
  hrt_init();

  PX4_INFO("Ici");
  PX4_CRITICAL(px4::WorkQueueManagerStart());

// MUORB has slightly different startup requirements
#if defined(CONFIG_MODULES_MUORB_APPS)
  // Put sleeper in here to allow wq to finish initializing before param_init is called
  usleep(10000);

  uorb_start();

  muorb_init();

  // Give muorb some time to setup the DSP
  usleep(100000);

  param_init();
#else

  uorb_start();
  px4_log_initialize();
  param_init();
#endif

  return PX4_OK;
}

int px4_platform_fini(void) {
  param_fini();

  std::set<std::string> simulator_names = {
    "simulator_sih",
    "gz_bridge",
  };
  for (auto &x : ModuleManager::get_modules()) {

    if (!simulator_names.count(x.name) && x.is_running()) {
      PX4_INFO("Stopping running module %s", x.name.c_str());
      x.stop_command();
      PX4_INFO("done Stopping running module %s", x.name.c_str());
    }
  }

  PX4_INFO("Quitting HRT");
  px4_show_tasks();
  PX4_INFO("Quitting HRT");
  hrt_fini();
  hrt_work_queue_fini();

  PX4_INFO("Done stoping modules");

  for (auto sim_name : simulator_names) {
    auto sih_mod = ModuleManager::get_module(sim_name);
    if (sih_mod != nullptr && sih_mod->is_running()) {

      sih_mod->request_stop();
      while (sih_mod->is_running()) {
        // cannot use usleep
        usleep(10);
      }
    }
  }



  px4_show_tasks();

  struct timespec ts = {.tv_sec= 0, .tv_nsec =0};
  px4_clock_settime(CLOCK_MONOTONIC, &ts);
  PX4_INFO("Work queue stop");
  auto ret = px4::WorkQueueManagerStop();
  PX4_INFO("Work queue stop done");
  sleep(1);
  px4_show_tasks();

  printf("WQ STOP >> %d\n", ret);
  px4_log_finalize();
  uorb_stop();
  // param_init();

  return PX4_OK;
}
