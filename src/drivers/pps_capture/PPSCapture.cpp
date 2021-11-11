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

/**
 * @file PPSCapture.cpp
 *
 * This is driver for capturing GNSS Pulse Per Second (PPS) signal.
 *
 */

#include "PPSCapture.hpp"
#include "board_config.h"
#include <px4_arch/io_timer.h>

PPSCapture::PPSCapture() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

PPSCapture::~PPSCapture()
{
#if defined(PPS_CAPTURE_CHANNEL)
	io_timer_unallocate_channel(PPS_CAPTURE_CHANNEL);
	px4_arch_gpiosetevent(_pps_capture_gpio, false, false, false, nullptr, nullptr);
#endif
}

bool PPSCapture::init()
{
	bool success = false;

#if defined(PPS_CAPTURE_CHANNEL)

	int ret = io_timer_allocate_channel(PPS_CAPTURE_CHANNEL, IOTimerChanMode_PPS);

	if (ret != PX4_OK) {
		PX4_ERR("gpio alloc failed (%i) for PPS at channel (%d)", ret, PPS_CAPTURE_CHANNEL);
		return false;
	}

	_pps_capture_gpio = PX4_MAKE_GPIO_EXTI(io_timer_channel_get_as_pwm_input(PPS_CAPTURE_CHANNEL));
	int ret_val = px4_arch_gpiosetevent(_pps_capture_gpio, true, false, true, &PPSCapture::gpio_interrupt_callback, this);

	if (ret_val == PX4_OK) {
		success = true;
	}

#else
#error Driver requires PPS_CAPTURE_CHANNEL to be enabled
#endif

	return success;
}

void PPSCapture::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	sensor_gps_s sensor_gps;

	if (_sensor_gps_sub.update(&sensor_gps)) {
		_last_gps_utc_timestamp = sensor_gps.time_utc_usec;
		_last_gps_timestamp = sensor_gps.timestamp;
	}

	pps_capture_s pps_capture;
	pps_capture.timestamp = _hrt_timestamp;
	// GPS UTC time when the GPIO interrupt was triggered
	// UTC time + elapsed time since the UTC time was received - eleapsed time since the PPS interrupt
	// event
	uint64_t gps_utc_time = _last_gps_utc_timestamp + hrt_elapsed_time(&_last_gps_timestamp)
				- hrt_elapsed_time(&_hrt_timestamp);

	// (For ubx F9P) The rising edge of the PPS pulse is aligned to the top of second GPS time base.
	// So, remove the fraction of second and shift to the next second. The interrupt is triggered
	// before the matching timestamp tranfered, which means the last received GPS time is always
	// behind.
	pps_capture.rtc_timestamp = gps_utc_time - (gps_utc_time % USEC_PER_SEC) + USEC_PER_SEC;

	_pps_capture_pub.publish(pps_capture);
}

int PPSCapture::gpio_interrupt_callback(int irq, void *context, void *arg)
{
	PPSCapture *instance = static_cast<PPSCapture *>(arg);

	instance->_hrt_timestamp = hrt_absolute_time();
	instance->ScheduleNow(); // schedule work queue to publish PPS captured time

	return PX4_OK;
}

int PPSCapture::task_spawn(int argc, char *argv[])
{
	PPSCapture *instance = new PPSCapture();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int PPSCapture::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PPSCapture::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements capturing PPS information from the GNSS module and calculates the drift between PPS and Real-time clock.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pps_capture", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void PPSCapture::stop()
{
	exit_and_cleanup();
}

extern "C" __EXPORT int pps_capture_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "stop")) {
		PPSCapture::stop();
	}

	return PPSCapture::main(argc, argv);
}
