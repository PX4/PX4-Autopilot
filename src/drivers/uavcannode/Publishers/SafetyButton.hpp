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

#include "UavcanPublisherBase.hpp"

#include <ardupilot/indication/Button.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/button_event.h>

#ifndef GPIO_BTN_SAFETY
#error "board needs to define a safety button gpio pin to use this module"
#endif

namespace uavcannode
{

class SafetyButton :
	public UavcanPublisherBase,
	private uavcan::Publisher<ardupilot::indication::Button>
{
public:
	SafetyButton(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(ardupilot::indication::Button::DefaultDataTypeID),
		uavcan::Publisher<ardupilot::indication::Button>(node)
	{
		this->setPriority(uavcan::TransferPriority::Default);
	}

	void PrintInfo() override
	{
		printf("\tsafety_button_gpio -> %s:%d\n",
		       ardupilot::indication::Button::getDataTypeFullName(),
		       ardupilot::indication::Button::DefaultDataTypeID);
	}

	void BroadcastAnyUpdates() override
	{
		ardupilot::indication::Button Button{};
		Button.button = ardupilot::indication::Button::BUTTON_SAFETY;

		const bool button_pressed = px4_arch_gpioread(GPIO_BTN_SAFETY);
		bool publish = false;

		if (_last_button_state == false && button_pressed) {
			_button_press_start = hrt_absolute_time();
			_last_button_state = true;
		}

		// Calculate the time the button has been pressed in units of 0.1s with a max of 255
		if (_last_button_state && button_pressed) {
			const hrt_abstime button_press_duration = hrt_absolute_time() - _button_press_start;
			Button.press_time = (uint8_t)math::min((button_press_duration / 100000), (hrt_abstime)255);

			// Publish the button state every 0.1s
			if (hrt_elapsed_time(&_last_button_publish) > 100_ms) {
				publish = true;
			}

		} else {
			Button.press_time = 0;
			_last_button_state = false;

			// Publish the button state every 1
			if (hrt_elapsed_time(&_last_button_publish) > 1_s) {
				publish = true;
			}
		}

		if (publish) {
			_last_button_publish = hrt_absolute_time();
			uavcan::Publisher<ardupilot::indication::Button>::broadcast(Button);
		}
	}
private:
	bool        _last_button_state{false};
	hrt_abstime _button_press_start{0};
	hrt_abstime _last_button_publish{0};
};
} // namespace uavcannode
