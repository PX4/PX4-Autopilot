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

#pragma once

#include <stdint.h>

namespace px4
{

class WorkQueue; // forward declaration

struct wq_config {
	const char *name;
	uint16_t stacksize;
	int8_t priority; // relative to max
};

// work queues
enum PX4_WQS {
	// SPI devices
	SPI1 = 0,
	SPI2,

	// PX4 controllers
	rate_ctrl,

	// I2C devices
	I2C1,
	I2C2,

	// PX4 misc
	hp_default,

	// testing
	test1,
	test2,
};

// TODO: set priorities appropriately for linux, macos, qurt (unify with px4_tasks.h)
static constexpr wq_config wq_configurations[] = {
	[SPI1] = { "wq:SPI1", 1200, 0 },
	[SPI2] = { "wq:SPI2", 1200, -1 },

	[rate_ctrl] = { "wq:rate_ctrl", 1500, -2 },  // highest priority after primary IMU (SPI1), TODO:

	[I2C1] = { "wq:I2C1", 1200, -2 },
	[I2C2] = { "wq:I2C2", 1200, -3 },

	[hp_default] = { "wq:hp_default", 1500, -4 },

	[test1] = { "wq:test1", 800, 0 },
	[test2] = { "wq:test2", 800, 0 },
};



/**
 * Start the work queue manager task.
 */
int work_queue_manager_start();

/**
 * Stop the work queue manager task.
 */
int work_queue_manager_stop();

/**
 * Create (or find) a work queue with a particular configuration.
 *
 * @param new_wq		The work queue configuration (see WorkQueueManager.hpp).
 * @return		A pointer to the WorkQueue, or nullptr on failure.
 */
WorkQueue *work_queue_create(const wq_config &new_wq);

/**
 * Map a PX4 driver device id to a work queue (by sensor bus).
 *
 * @param device_id		The PX4 driver's device id.
 * @return		A work queue configuration.
 */
const wq_config &device_bus_to_wq(uint32_t device_id);


} // namespace px4
