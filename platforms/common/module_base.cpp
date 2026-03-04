/****************************************************************************
 *
 *   Copyright (c) 2017-2025 PX4 Development Team. All rights reserved.
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
 * @file module_base.cpp
 *
 * Shared implementations of ModuleBase static methods. One copy linked
 * for all modules, replacing per-module CRTP template instantiations.
 */

#ifndef MODULE_NAME
#define MODULE_NAME "module"
#endif

#include <px4_platform_common/module.h>

#include <pthread.h>
#include <cstring>

extern pthread_mutex_t px4_modules_mutex;

int ModuleBase::print_status()
{
	printf("running\n");
	return 0;
}

int ModuleBase::main(Descriptor &desc, int argc, char *argv[])
{
	if (argc <= 1 ||
	    strcmp(argv[1], "-h")    == 0 ||
	    strcmp(argv[1], "help")  == 0 ||
	    strcmp(argv[1], "info")  == 0 ||
	    strcmp(argv[1], "usage") == 0) {
		return desc.print_usage(nullptr);
	}

	if (strcmp(argv[1], "start") == 0) {
		// Pass the 'start' argument too, because later on px4_getopt() will ignore the first argument.
		return start_command(desc, argc - 1, argv + 1);
	}

	if (strcmp(argv[1], "status") == 0) {
		return status_command(desc);
	}

	if (strcmp(argv[1], "stop") == 0) {
		return stop_command(desc);
	}

	pthread_mutex_lock(&px4_modules_mutex);
	int ret = desc.custom_command(argc - 1, argv + 1);
	pthread_mutex_unlock(&px4_modules_mutex);

	return ret;
}

int ModuleBase::start_command(Descriptor &desc, int argc, char *argv[])
{
	int ret = 0;
	pthread_mutex_lock(&px4_modules_mutex);

	if (is_running(desc)) {
		ret = -1;
		PX4_ERR("Task already running");

	} else {
		ret = desc.task_spawn(argc, argv);

		if (ret < 0) {
			PX4_ERR("Task start failed (%i)", ret);
		}
	}

	pthread_mutex_unlock(&px4_modules_mutex);
	return ret;
}

int ModuleBase::stop_command(Descriptor &desc)
{
	int ret = 0;
	pthread_mutex_lock(&px4_modules_mutex);

	if (is_running(desc)) {
		ModuleBase *object = desc.object.load();

		if (object) {
			object->request_stop();

			unsigned int i = 0;

			do {
				pthread_mutex_unlock(&px4_modules_mutex);
				px4_usleep(10000); // 10 ms
				pthread_mutex_lock(&px4_modules_mutex);

				if (++i > 500 && desc.task_id != -1) { // wait at most 5 sec
					PX4_ERR("timeout, forcing stop");

					if (desc.task_id != task_id_is_work_queue) {
						px4_task_delete(desc.task_id);
					}

					desc.task_id = -1;

					delete desc.object.load();
					desc.object.store(nullptr);

					ret = -1;
					break;
				}
			} while (desc.task_id != -1);

		} else {
			// In the very unlikely event that can only happen on work queues,
			// if the starting thread does not wait for the work queue to initialize,
			// and inside the work queue, the allocation of _object fails
			// and exit_and_cleanup() is not called, set the task_id as invalid.
			desc.task_id = -1;
		}
	}

	pthread_mutex_unlock(&px4_modules_mutex);
	return ret;
}

int ModuleBase::status_command(Descriptor &desc)
{
	int ret = -1;
	pthread_mutex_lock(&px4_modules_mutex);

	if (is_running(desc) && desc.object.load()) {
		ModuleBase *object = desc.object.load();
		ret = object->print_status();

	} else {
		PX4_INFO("not running");
	}

	pthread_mutex_unlock(&px4_modules_mutex);
	return ret;
}

void ModuleBase::exit_and_cleanup(Descriptor &desc)
{
	// Take the lock here:
	// - if startup fails and we're faster than the parent thread, it will set
	//   task_id and subsequently it will look like the task is running.
	// - deleting the object must take place inside the lock.
	pthread_mutex_lock(&px4_modules_mutex);

	delete desc.object.load();
	desc.object.store(nullptr);

	desc.task_id = -1; // Signal a potentially waiting thread for the module to exit that it can continue.
	pthread_mutex_unlock(&px4_modules_mutex);
}

int ModuleBase::wait_until_running(Descriptor &desc, int timeout_ms)
{
	int i = 0;

	do {
		px4_usleep(2000);

	} while (!desc.object.load() && ++i < timeout_ms / 2);

	if (i >= timeout_ms / 2) {
		PX4_ERR("Timed out while waiting for thread to start");
		return -1;
	}

	return 0;
}

int ModuleBase::run_trampoline_impl(Descriptor &desc, instantiate_fn instantiate,
				    int argc, char *argv[])
{
	int ret = 0;

	// We don't need the task name at this point.
	argc -= 1;
	argv += 1;

	ModuleBase *object = instantiate(argc, argv);
	desc.object.store(object);

	if (object) {
		object->run();

	} else {
		PX4_ERR("failed to instantiate object");
		ret = -1;
	}

	exit_and_cleanup(desc);

	return ret;
}
