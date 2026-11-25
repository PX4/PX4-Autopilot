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

#ifdef BOARD_SAFETY_BUTTON_IO_TIMER_CHANNEL
#include <px4_arch/io_timer.h>
static_assert(BOARD_SAFETY_BUTTON_IO_TIMER_CHANNEL < MAX_TIMER_IO_CHANNELS, "safety input timer channel out of range");
#endif

// BOARD_SAFETY_BUTTON() is true while the safety button is pressed, and BOARD_SAFETY_LED(on_true) turns the safety
// LED on and off: by default, the GPIO_BTN_SAFETY input, high while pressed, and the active low GPIO_LED_SAFETY
#if !defined(BOARD_SAFETY_BUTTON) && defined(GPIO_BTN_SAFETY)
#define BOARD_SAFETY_BUTTON() px4_arch_gpioread(GPIO_BTN_SAFETY)
#endif

#if !defined(BOARD_SAFETY_LED) && defined(GPIO_LED_SAFETY)
#define BOARD_SAFETY_LED(on_true) px4_arch_gpiowrite(GPIO_LED_SAFETY, !(on_true))
#endif

#ifndef BOARD_SAFETY_BUTTON
#error "board needs to define a safety button (BOARD_SAFETY_BUTTON or GPIO_BTN_SAFETY) to use this module"
#endif

using namespace time_literals;

ModuleBase::Descriptor SafetyButton::desc{task_spawn, custom_command, print_usage};

// Number of consecutive 30 Hz samples the safety switch must be active to turn safety off.
// Boards can override the default of 1 second, e.g. for a switch that is not operated by hand.
#ifndef BOARD_SAFETY_BUTTON_CYCLE_COUNT
#define BOARD_SAFETY_BUTTON_CYCLE_COUNT 30
#endif

static constexpr uint8_t CYCLE_COUNT{BOARD_SAFETY_BUTTON_CYCLE_COUNT};

// Number of consecutive 30 Hz samples the safety switch must be inactive before it is accepted as an input.
// Boards with a pull-out safety plug set this, so that a plug that is missing at boot does not turn safety off.
// Such boards also get no pairing request, as anything that toggles the input would trigger it.
// The default of 0 accepts the switch right away.
#ifndef BOARD_SAFETY_BUTTON_FITTED_COUNT
#define BOARD_SAFETY_BUTTON_FITTED_COUNT 0
#endif

static constexpr uint8_t FITTED_COUNT{BOARD_SAFETY_BUTTON_FITTED_COUNT};
static constexpr uint8_t NOT_FITTED_WARN_COUNT{150}; /* warn if still not fitted 5 seconds after start */

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
	_input_accepted = (FITTED_COUNT == 0);
}

SafetyButton::~SafetyButton()
{
	ScheduleClear();

#ifdef BOARD_SAFETY_BUTTON_IO_TIMER_CHANNEL

	if (_timer_channel_reserved) {
		io_timer_unallocate_channel(BOARD_SAFETY_BUTTON_IO_TIMER_CHANNEL);
	}

#endif
}

void
SafetyButton::CheckSafetyRequest(bool button_pressed)
{
	if (!_input_accepted) {
		/* The switch has to be seen inactive first, e.g. a safety plug has to be fitted */
		if (!button_pressed) {
			if (++_fitted_counter >= FITTED_COUNT) {
				_input_accepted = true;
			}

		} else {
			_fitted_counter = 0;

			if (_not_fitted_counter < NOT_FITTED_WARN_COUNT) {
				if (++_not_fitted_counter == NOT_FITTED_WARN_COUNT) {
					mavlink_log_critical(&_mavlink_log_pub, "Safety plug not fitted, safety stays on\t");
					events::send(events::ID("safety_button_plug_not_fitted"), events::Log::Critical,
						     "Safety plug not fitted, safety stays on");
				}
			}
		}

		return;
	}

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
#if defined(BOARD_SAFETY_LED)
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_safety_off = vehicle_status.safety_off;
	}

	actuator_armed_s armed;

	if (_armed_sub.copy(&armed)) {
		// Select the appropriate LED flash pattern depending on the current safety and arm states
		LED_PATTERN pattern = LED_PATTERN::FMU_REFUSE_TO_ARM;

		// cycle the blink state machine
		if (_safety_off) {
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
		const bool lit = (uint16_t)pattern & (1 << (_blink_counter++ / 3));
		BOARD_SAFETY_LED(lit);

		if (_blink_counter >= 48) {
			_blink_counter = 0;
		}
	}

#endif // BOARD_SAFETY_LED
}

void
SafetyButton::Run()
{
	if (should_exit()) {
		exit_and_cleanup(desc);
		return;
	}

	const bool button_pressed = BOARD_SAFETY_BUTTON();

	// control safety switch LED & safety button
	if (!_has_px4io) {
		FlashButton();
		CheckSafetyRequest(button_pressed);
	}

	// Pressing 3 times to pair only makes sense for a hand operated button, not for a plug
	if (FITTED_COUNT == 0) {
		CheckPairingRequest(button_pressed);
	}

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

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	return ret;
}

int
SafetyButton::Start()
{
#ifdef BOARD_SAFETY_BUTTON_IO_TIMER_CHANNEL
	// Reserve the timer channel of the input pin, so that pwm_out does not turn it into an output
	int ret = io_timer_allocate_channel(BOARD_SAFETY_BUTTON_IO_TIMER_CHANNEL, IOTimerChanMode_Other);

	if (ret != 0) {
		PX4_ERR("safety input timer channel %d is in use (%i)", BOARD_SAFETY_BUTTON_IO_TIMER_CHANNEL, ret);
		return ret;
	}

	_timer_channel_reserved = true;
#endif

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
	return ModuleBase::main(SafetyButton::desc, argc, argv);
}
