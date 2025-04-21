/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
static constexpr wq_config_t rate_ctrl{"wq:rate_ctrl", 3150, 0}; // PX4 inner loop highest priority

static constexpr wq_config_t SPI0{"wq:SPI0", 2392, -1};
static constexpr wq_config_t SPI1{"wq:SPI1", 2392, -2};
static constexpr wq_config_t SPI2{"wq:SPI2", 2392, -3};
static constexpr wq_config_t SPI3{"wq:SPI3", 2392, -4};
static constexpr wq_config_t SPI4{"wq:SPI4", 2392, -5};
static constexpr wq_config_t SPI5{"wq:SPI5", 2392, -6};
static constexpr wq_config_t SPI6{"wq:SPI6", 2392, -7};

static constexpr wq_config_t I2C0{"wq:I2C0", 2336, -8};
static constexpr wq_config_t I2C1{"wq:I2C1", 2336, -9};
static constexpr wq_config_t I2C2{"wq:I2C2", 2336, -10};
static constexpr wq_config_t I2C3{"wq:I2C3", 2336, -11};
static constexpr wq_config_t I2C4{"wq:I2C4", 2336, -12};

// PX4 att/pos controllers, highest priority after sensors.
static constexpr wq_config_t nav_and_controllers{"wq:nav_and_controllers", 2240, -13};

static constexpr wq_config_t INS0{"wq:INS0", 6000, -14};
static constexpr wq_config_t INS1{"wq:INS1", 6000, -15};
static constexpr wq_config_t INS2{"wq:INS2", 6000, -16};
static constexpr wq_config_t INS3{"wq:INS3", 6000, -17};

static constexpr wq_config_t hp_default{"wq:hp_default", 2800, -18};

static constexpr wq_config_t uavcan{"wq:uavcan", 3624, -19};

static constexpr wq_config_t ttyS0{"wq:ttyS0", 1728, -21};
static constexpr wq_config_t ttyS1{"wq:ttyS1", 1728, -22};
static constexpr wq_config_t ttyS2{"wq:ttyS2", 1728, -23};
static constexpr wq_config_t ttyS3{"wq:ttyS3", 1728, -24};
static constexpr wq_config_t ttyS4{"wq:ttyS4", 1728, -25};
static constexpr wq_config_t ttyS5{"wq:ttyS5", 1728, -26};
static constexpr wq_config_t ttyS6{"wq:ttyS6", 1728, -27};
static constexpr wq_config_t ttyS7{"wq:ttyS7", 1728, -28};
static constexpr wq_config_t ttyS8{"wq:ttyS8", 1728, -29};
static constexpr wq_config_t ttyS9{"wq:ttyS9", 1728, -30};
static constexpr wq_config_t ttyACM0{"wq:ttyACM0", 1728, -31};
static constexpr wq_config_t ttyUnknown{"wq:ttyUnknown", 1728, -32};

static constexpr wq_config_t lp_default{"wq:lp_default", 3500, -50};

static constexpr wq_config_t test1{"wq:test1", 2000, 0};
static constexpr wq_config_t test2{"wq:test2", 2000, 0};

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

/**
 * Map a serial device path (eg /dev/ttyS1) to a work queue.
 *
 * @param device_id		The device path.
 * @return		A work queue configuration.
 */
const wq_config_t &serial_port_to_wq(const char *serial);

const wq_config_t &ins_instance_to_wq(uint8_t instance);


} // namespace px4
