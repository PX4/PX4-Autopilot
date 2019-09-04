/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

#include "SafetyButton.hpp"

using namespace time_literals;

static constexpr uint8_t CYCLE_COUNT{10}; /* safety switch must be held for 1 second to activate */

// Define the various LED flash sequences for each system state.
enum class LED_PATTERN : uint16_t {
	FMU_OK_TO_ARM		= 0x0003,	/**< slow blinking */
	FMU_REFUSE_TO_ARM	= 0x5555,	/**< fast blinking */
	IO_ARMED		= 0x5050,	/**< long off, then double blink */
	FMU_ARMED		= 0x5500,	/**< long off, then quad blink */
	IO_FMU_ARMED		= 0xffff,	/**< constantly on */
};

SafetyButton::~SafetyButton()
{
	ScheduleClear();
}

void
SafetyButton::CheckButton()
{
	// Debounce the safety button, change state if it has been held for long enough.
	bool safety_button_pressed = px4_arch_gpioread(GPIO_BTN_SAFETY);

	/*
	 * Keep pressed for a while to arm.
	 *
	 * Note that the counting sequence has to be same length
	 * for arming / disarming in order to end up as proper
	 * state machine, keep ARM_COUNTER_THRESHOLD the same
	 * length in all cases of the if/else struct below.
	 */
	if (safety_button_pressed && !_safety_btn_off) {
		if (_button_counter < CYCLE_COUNT) {
			_button_counter++;

		} else if (_button_counter == CYCLE_COUNT) {
			// switch to armed state
			_safety_btn_off = true;
			_button_counter++;
		}

	} else if (safety_button_pressed && _safety_btn_off) {

		if (_button_counter < CYCLE_COUNT) {
			_button_counter++;

		} else if (_button_counter == CYCLE_COUNT) {
			// change to disarmed state and notify
			_safety_btn_off = false;
			_button_counter++;
		}

	} else {
		_button_counter = 0;
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

		// cycle the blink state machine at 10Hz
		if (_safety_btn_off) {
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
		px4_arch_gpiowrite(GPIO_LED_SAFETY, !((uint16_t)pattern & (1 << _blink_counter++)));

		if (_blink_counter > 15) {
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
	}

	// read safety switch input and control safety switch LED at 10Hz
	CheckButton();

	// Make the safety button flash anyway, no matter if it's used or not.
	FlashButton();

	safety_s safety{};
	safety.timestamp = hrt_absolute_time();
	safety.safety_switch_available = true;
	safety.safety_off = _safety_btn_off;

	// publish the safety status
	_to_safety.publish(safety);
}

int
SafetyButton::task_spawn(int argc, char *argv[])
{
	if (PX4_MFT_HW_SUPPORTED(PX4_MFT_PX4IO)) {
		PX4_ERR("not starting (use px4io for safety button)");

		return PX4_ERROR;

	} else if (circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY)) {
		PX4_WARN("disabled by CBRK_IO_SAFETY, exiting");
		return PX4_ERROR;

	} else {
		SafetyButton *instance = new SafetyButton();

		if (instance) {
			_object.store(instance);
			_task_id = task_id_is_work_queue;

			if (instance->Start() == PX4_OK) {
				return PX4_OK;
			}

		} else {
			PX4_ERR("alloc failed");
		}

		delete instance;
		_object.store(nullptr);
		_task_id = -1;
	}

	return PX4_ERROR;
}

int
SafetyButton::Start()
{
	ScheduleOnInterval(100_ms); // run at 10 Hz

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

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("safety_button", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the safety button driver");

	return 0;
}

int
SafetyButton::print_status()
{
	PX4_INFO("Safety State (from button): %s", _safety_btn_off ? "off" : "on");

	return 0;
}

extern "C" __EXPORT int safety_button_main(int argc, char *argv[]);

int
safety_button_main(int argc, char *argv[])
{
	return SafetyButton::main(argc, argv);
}
