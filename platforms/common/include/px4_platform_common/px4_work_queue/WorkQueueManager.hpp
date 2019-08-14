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

struct wq_config_t {
	const char *name;
	uint16_t stacksize;
	int8_t relative_priority; // relative to max
};

namespace wq_configurations
{
static constexpr wq_config_t rate_ctrl{"wq:rate_ctrl", 1600, 0}; // PX4 inner loop highest priority

static constexpr wq_config_t SPI1{"wq:SPI1", 1400, -1};
static constexpr wq_config_t SPI2{"wq:SPI2", 1400, -2};
static constexpr wq_config_t SPI3{"wq:SPI3", 1400, -3};
static constexpr wq_config_t SPI4{"wq:SPI4", 1400, -4};
static constexpr wq_config_t SPI5{"wq:SPI5", 1400, -5};
static constexpr wq_config_t SPI6{"wq:SPI6", 1400, -6};

static constexpr wq_config_t I2C1{"wq:I2C1", 1250, -7};
static constexpr wq_config_t I2C2{"wq:I2C2", 1250, -8};
static constexpr wq_config_t I2C3{"wq:I2C3", 1250, -9};
static constexpr wq_config_t I2C4{"wq:I2C4", 1250, -10};

static constexpr wq_config_t att_pos_ctrl{"wq:att_pos_ctrl", 6600, -11}; // PX4 att/pos controllers, highest priority after sensors

static constexpr wq_config_t hp_default{"wq:hp_default", 1500, -12};
static constexpr wq_config_t lp_default{"wq:lp_default", 1700, -50};

static constexpr wq_config_t test1{"wq:test1", 800, 0};
static constexpr wq_config_t test2{"wq:test2", 800, 0};

} // namespace wq_configurations

/**
 * Start the work queue manager task.
 */
int WorkQueueManagerStart();

/**
 * Stop the work queue manager task.
 */
int WorkQueueManagerStop();

/**
 * Work queue manager status.
 */
int WorkQueueManagerStatus();

/**
 * Create (or find) a work queue with a particular configuration.
 *
 * @param new_wq		The work queue configuration (see WorkQueueManager.hpp).
 * @return		A pointer to the WorkQueue, or nullptr on failure.
 */
WorkQueue *WorkQueueFindOrCreate(const wq_config_t &new_wq);

/**
 * Map a PX4 driver device id to a work queue (by sensor bus).
 *
 * @param device_id		The PX4 driver's device id.
 * @return		A work queue configuration.
 */
const wq_config_t &device_bus_to_wq(uint32_t device_id);


} // namespace px4
