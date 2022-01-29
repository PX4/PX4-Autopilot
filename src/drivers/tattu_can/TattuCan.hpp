/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include <nuttx/can/can.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/battery_status.h>

using namespace time_literals;

typedef struct __attribute__((packed))
{
	int16_t     manufacturer;
	uint16_t    sku;
	uint16_t    voltage;
	int16_t     current;
	int16_t     temperature;
	uint16_t    remaining_percent;
	uint16_t    cycle_life;
	int16_t     health_status;
	uint16_t    cell_1_voltage;
	uint16_t    cell_2_voltage;
	uint16_t    cell_3_voltage;
	uint16_t    cell_4_voltage;
	uint16_t    cell_5_voltage;
	uint16_t    cell_6_voltage;
	uint16_t    cell_7_voltage;
	uint16_t    cell_8_voltage;
	uint16_t    cell_9_voltage;
	uint16_t    cell_10_voltage;
	uint16_t    cell_11_voltage;
	uint16_t    cell_12_voltage;
	uint16_t    standard_capacity;
	uint16_t    remaining_capacity_mah;
	uint32_t    error_info;
} Tattu12SBatteryMessage;

typedef struct {
	uint64_t timestamp_usec;
	uint32_t extended_can_id;
	size_t      payload_size;
	const void *payload;
} CanFrame;

class TattuCan : public ModuleBase<TattuCan>, public px4::ScheduledWorkItem
{
public:
	TattuCan();

	virtual ~TattuCan();

	static int print_usage(const char *reason = nullptr);
	static int custom_command(int argc, char *argv[]);

	static int task_spawn(int argc, char *argv[]);

	int start();

	int16_t receive(CanFrame *received_frame);

private:
	static constexpr uint32_t SAMPLE_RATE{100}; // samples per second (10ms)
	static constexpr size_t TAIL_BYTE_START_OF_TRANSFER{128};

	void Run() override;

	int _fd{-1};
	bool _initialized{false};

	uORB::Publication<battery_status_s> _battery_status_pub{ORB_ID::battery_status};
};
