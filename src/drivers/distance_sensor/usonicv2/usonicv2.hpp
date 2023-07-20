/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file usonicv2.hpp
 * @author Andrew McFarland <andrew@steamfoundry.ca
 * @author David Sidrane <david.sidrane@nscdg.com>
 * Adapted from the SRF-05 Ultrasonic driver
 */

#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <uORB/Subscription.hpp>
#include <board_config.h>
#include <drivers/device/device.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>

//Still needs both pins on the FMU
#if defined(GPIO_ULTRASOUND_TRIGGER) && defined(GPIO_ULTRASOUND_ECHO)
#  define HAVE_ULTRASOUND
#endif

using namespace time_literals;

// Device limits from SeeedStudio Ultrasonic Ranger v2.0
static constexpr float USONICV2_MIN_DISTANCE{0.03f};
static constexpr float USONICV2_MAX_DISTANCE{3.5f};

//Test distance to publish - Unused now
//static constexpr float USONICV2_TEST_DISTANCE{0.42f};

// Normal conversion wait time.
static constexpr uint32_t USONICV2_CONVERSION_INTERVAL{50_ms};

// Maximum time to wait for a conversion to complete.
static constexpr uint32_t USONICV2_CONVERSION_TIMEOUT{30_ms};


class USONICV2 : public ModuleBase<USONICV2>, public px4::ScheduledWorkItem
{
public:
	USONICV2(const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~USONICV2() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int init();

	int print_status() override;

protected:
	void stop();
	int collect();
	int measure();
//	int testSample(); //Used during testing
//	void printState();
	void Run() override;


private:
	uint32_t get_measure_interval() const { return USONICV2_CONVERSION_INTERVAL; };

	static int EchoInterruptCallback(int irq, void *context, void *arg);
	void OnEdge(bool state);

	hrt_abstime _rising_edge_time{0};
	hrt_abstime _falling_edge_time{0};
	hrt_abstime _falling_trigger_time{0};

	enum class STATE : uint8_t {
		TRIGGER,
		WAIT_FOR_RISING,
		WAIT_FOR_FALLING,
		SAMPLE,
		MEASURE,
		EXIT,
	};

	STATE _state{STATE::TRIGGER};



	PX4Rangefinder	_px4_rangefinder;

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME" comms errors")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME" read")};
	perf_counter_t _sensor_resets{perf_alloc(PC_COUNT, MODULE_NAME" resets")};
};
