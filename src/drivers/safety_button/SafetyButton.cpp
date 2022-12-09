/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

#include <board_config.h>
#include "SafetyButton.hpp"

#ifndef GPIO_BTN_SAFETY
#error "board needs to define a safety button gpio pin to use this module"
#endif

using namespace time_literals;

static constexpr uint8_t CYCLE_COUNT{30}; /* safety switch must be held for 1 second to activate */

// Define the various LED flash sequences for each system state.
enum class LED_PATTERN : uint16_t {
	FMU_OK_TO_ARM		= 0x0003,	/**< slow blinking */
	FMU_REFUSE_TO_ARM	= 0x5555,	/**< fast blinking */
	IO_ARMED		= 0x5050,	/**< long off, then double blink */
	FMU_ARMED		= 0x5500,	/**< long off, then quad blink */
	IO_FMU_ARMED		= 0xffff,	/**< constantly on */
};

SafetyButton::SafetyButton() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_has_px4io = PX4_MFT_HW_SUPPORTED(PX4_MFT_PX4IO);
}

SafetyButton::~SafetyButton()
{
	ScheduleClear();
}

void
SafetyButton::CheckSafetyRequest(bool button_pressed)
{
	/* Keep button pressed for one second to turn off safety */
	if (button_pressed) {

		if (_button_counter <= CYCLE_COUNT) {
			_button_counter++;
		}

		if (_button_counter == CYCLE_COUNT) {
			_button_publisher.safetyButtonTriggerEvent();
		}

	} else {
		_button_counter = 0;
	}
}

void
SafetyButton::CheckPairingRequest(bool button_pressed)
{
	// Need to press the button 3 times within 2 seconds
	const hrt_abstime now = hrt_absolute_time();

	if (now - _pairing_start > 2_s) {
		// reset state
		_pairing_start = 0;
		_pairing_button_counter = 0;
	}

	if (!_button_prev_sate && button_pressed) {
		if (_pairing_start == 0) {
			_pairing_start = now;
		}

		++_pairing_button_counter;
	}

	if (_pairing_button_counter == ButtonPublisher::PAIRING_BUTTON_EVENT_COUNT) {
		_button_publisher.pairingButtonTriggerEvent();
		// reset state
		_pairing_start = 0;
		_pairing_button_counter = 0;
	}
}

void
SafetyButton::FlashButton()
{
#if defined(GPIO_LED_SAFETY)
	actuator_armed_s armed;

	if (_armed_sub.copy(&armed)) {
		// Select the appropriate LED flash pattern depending on the current arm state
		LED_PATTERN pattern = LED_PATTERN::FMU_REFUSE_TO_ARM;

		// cycle the blink state machine
		if (_button_prev_sate) {
			if (armed.armed) {
				pattern = LED_PATTERN::IO_FMU_ARMED;

			} else {
				pattern = LED_PATTERN::IO_ARMED;
			}

		} else if (armed.armed) {
			pattern = LED_PATTERN::FMU_ARMED;

		} else {
			pattern = LED_PATTERN::FMU_OK_TO_ARM;
		}

		// Turn the LED on if we have a 1 at the current bit position
		px4_arch_gpiowrite(GPIO_LED_SAFETY, !((uint16_t)pattern & (1 << (_blink_counter++ / 3))));

		if (_blink_counter > 45) {
			_blink_counter = 0;
		}
	}

#endif // GPIO_LED_SAFETY
}

void
SafetyButton::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	const bool button_pressed = px4_arch_gpioread(GPIO_BTN_SAFETY);

	// control safety switch LED & safety button
	if (!_has_px4io) {
		FlashButton();
		CheckSafetyRequest(button_pressed);
	}

	CheckPairingRequest(button_pressed);
	_button_prev_sate = button_pressed;
}

int
SafetyButton::task_spawn(int argc, char *argv[])
{
	SafetyButton *instance = new SafetyButton();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	int ret = instance->Start();

	if (ret != PX4_OK) {
		delete instance;
		return ret;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	return ret;
}

int
SafetyButton::Start()
{
	ScheduleOnInterval(33_ms); // run at 30 Hz

	return PX4_OK;
}

int
SafetyButton::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
SafetyButton::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for the safety button.
Pressing the safety button 3 times quickly will trigger a GCS pairing request.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("safety_button", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int safety_button_main(int argc, char *argv[]);

int
safety_button_main(int argc, char *argv[])
{
	return SafetyButton::main(argc, argv);
}
