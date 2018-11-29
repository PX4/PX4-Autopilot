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

#include "WorkQueueManager.hpp"

#include "WorkQueue.hpp"

#include <string.h>

#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <drivers/drv_hrt.h>
#include <px4_atomic.h>
#include <containers/BlockingQueue.hpp>
#include <containers/List.hpp>

#include <lib/drivers/device/Device.hpp>

using namespace time_literals;

namespace px4
{

// list of current work queues
static List<WorkQueue *> _px4_work_queues_list;
static pthread_mutex_t _px4_work_queues_list_mutex = PTHREAD_MUTEX_INITIALIZER;

// queue of WorkQueues to be created (as threads in the wq manager task)
static BlockingQueue<wq_config *, 1> _px4_wq_manager_new;

static px4::atomic_bool _wq_task_should_exit{false};


static WorkQueue *find_work_queue(const char *name)
{
	pthread_mutex_lock(&_px4_work_queues_list_mutex);

	// search list
	for (WorkQueue *wq = _px4_work_queues_list.getHead(); wq != nullptr; wq = wq->getSibling()) {
		if (strcmp(wq->get_name(), name) == 0) {
			pthread_mutex_unlock(&_px4_work_queues_list_mutex);
			return wq;
		}
	}

	pthread_mutex_unlock(&_px4_work_queues_list_mutex);

	return nullptr;
}

WorkQueue *work_queue_create(const wq_config &new_wq)
{
	// search list for existing work queue
	WorkQueue *wq = find_work_queue(new_wq.name);

	// create work queue if it doesn't exist
	if (wq == nullptr) {

		// add to list
		// main thread wakes up, creates the thread
		_px4_wq_manager_new.push((wq_config *)&new_wq);

		// we wait until new wq is created, then return
		uint64_t t = 0;

		while (wq == nullptr && t < 10_s) {
			wq = find_work_queue(new_wq.name);

			// Wait up to 10s, checking every 2.5ms
			t += 2500;
			px4_usleep(2500);
		}

		if (wq == nullptr) {
			PX4_ERR("work queue: %s failed to create", new_wq.name);
		}
	}

	return wq;
}

const wq_config &device_bus_to_wq(uint32_t device_id_int)
{
	union device::Device::DeviceId device_id;
	device_id.devid = device_id_int;

	const device::Device::DeviceBusType bus_type = device_id.devid_s.bus_type;
	const uint8_t bus = device_id.devid_s.bus;

	if (bus_type == device::Device::DeviceBusType_I2C) {
		if (bus == 1) {
			return wq_configurations[I2C1];

		} else if (bus == 2) {
			return wq_configurations[I2C2];
		}

	} else if (bus_type == device::Device::DeviceBusType_SPI) {
		if (bus == 1) {
			return wq_configurations[SPI1];

		} else if (bus == 2) {
			return wq_configurations[SPI2];
		}
	}

	// otherwise use high priority
	return wq_configurations[hp_default];
};

static void *work_queue_runner(void *context)
{
	wq_config *config = static_cast<wq_config *>(context);
	WorkQueue wq(*config);

	// add to work queue list
	pthread_mutex_lock(&_px4_work_queues_list_mutex);
	_px4_work_queues_list.add(&wq);
	pthread_mutex_unlock(&_px4_work_queues_list_mutex);

	wq.Run();

	// remove from work queue list
	pthread_mutex_lock(&_px4_work_queues_list_mutex);
	_px4_work_queues_list.remove(&wq);
	pthread_mutex_unlock(&_px4_work_queues_list_mutex);

	return nullptr;
}

static void work_queue_manager_run()
{
	while (!_wq_task_should_exit.load()) {

		// create new work queues as needed
		wq_config *wq = _px4_wq_manager_new.pop();

		if (wq != nullptr) {
			// create new work queue

			pthread_attr_t attr{};
			sched_param param{};
			pthread_attr_init(&attr);
			pthread_attr_getschedparam(&attr, &param);

			if (pthread_attr_setstacksize(&attr, PX4_STACK_ADJUSTED(wq->stacksize)) != OK) {
				PX4_ERR("setting stack size failed");
			}

			param.sched_priority = SCHED_PRIORITY_MAX - wq->priority;

			if (pthread_attr_setschedparam(&attr, &param) != OK) {
				PX4_ERR("setting sched params failed");
			}

			PX4_INFO("New WQ: %s priority: %d stack: %d", wq->name, param.sched_priority, PX4_STACK_ADJUSTED(wq->stacksize));

			pthread_t thread;
			int ret_create = pthread_create(&thread, &attr, work_queue_runner, (void *)wq);

			if (ret_create != 0) {
				PX4_ERR("failed to create thread");
			}

			pthread_attr_destroy(&attr);
		}
	}
}

int work_queue_manager_start()
{
	int task_id = px4_task_spawn_cmd("wq_manager",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 1200,
					 (px4_main_t)&work_queue_manager_run,
					 nullptr);

	if (task_id < 0) {
		PX4_ERR("start failed");
		return -errno;

	} else {
		PX4_INFO("starting");
	}

	return 0;
}

int work_queue_manager_stop()
{
	// ask all work queues (threads) to stop
	// NOTE: not currently safe without all WorkItems stopping first
	for (WorkQueue *wq = _px4_work_queues_list.getHead(); wq != nullptr; wq = wq->getSibling()) {
		wq->request_stop();
	}

	_wq_task_should_exit.store(true);

	// push nullptr to wake the wq manager task
	_px4_wq_manager_new.push(nullptr);

	return PX4_OK;
}

} // namespace px4
