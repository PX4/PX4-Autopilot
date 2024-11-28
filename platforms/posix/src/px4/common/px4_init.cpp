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
#include <hrt_work.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_manager.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/events.h>
#include <set>
#include <uORB/uORB.h>

#if defined(CONFIG_MODULES_MUORB_APPS)
extern "C" {
	int muorb_init();
}
#endif

#define PX4_CRITICAL(v)                                                                            \
	{                                                                                                \
		int ret = v;                                                                                   \
		if (ret != PX4_OK) PX4_PANIC(#v "failed > %d", ret);                                           \
	}

int px4_platform_init(void)
{
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

int px4_platform_fini(void)
{

	px4_show_tasks();

	for (auto &x : ModuleManager::get_modules()) {
		PX4_INFO("Requesting stop for %s", x.name.c_str());
		x.request_stop();
	}

	while (true) {
		bool seen = false;
		PX4_INFO("");

		for (auto &x : ModuleManager::get_modules()) {
			if (x.is_running()) {
				PX4_INFO("Still running module %s", x.name.c_str());
				seen = true;
			}
		}

		px4::WorkQueueManagerStatus();

		if (!seen) { break; }

		sleep(3);
	}

	//Finally, delete stuff
	for (auto &x : ModuleManager::get_modules()) {
		if (x.stop_command()) {
			PX4_PANIC("Command was stopped but stop_command returns %s", x.name.c_str());
		}
	}

	PX4_INFO("Quitting HRT");
	px4_show_tasks();
	PX4_INFO("Quitting HRT");
	param_fini();

	hrt_fini();
	hrt_work_queue_fini();

	PX4_INFO("Done stoping modules");

	px4_show_tasks();

	PX4_INFO("Work queue stop");
	auto ret = px4::WorkQueueManagerStop();
	PX4_INFO("Work queue stop done");
	sleep(1);
	px4_show_tasks();

	PX4_INFO("WQ STOP >> %d", ret);
	px4_log_finalize();
	uorb_stop();
	ModuleManager::cleanup();;

	while (true) {
		int task_count = px4_running_task_count();

		if (task_count == 0) { break; }

		system_usleep(10);
	}

	px4_show_files();
	px4_cleanup();
	events::reset_events();
	PX4_INFO("px4_fini done");
	// param_init();


	return PX4_OK;
}
