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
#include <uORB/topics/safety.h>

namespace uavcannode
{

class Button :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<ardupilot::indication::Button>
{
public:
	Button(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(ardupilot::indication::Button::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(safety)), // technically unused
		uavcan::Publisher<ardupilot::indication::Button>(node)
	{
		this->setPriority(uavcan::TransferPriority::Default);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       ardupilot::indication::Button::getDataTypeFullName(),
			       ardupilot::indication::Button::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		const bool pressed = px4_arch_gpioread(GPIO_BTN_SAFETY);

		if (pressed && !_button_pressed) {
			// Button pressed
			_button_start = hrt_absolute_time();

		} else if (!pressed && _button_pressed) {
			// Button released
			hrt_abstime pressed_micros = hrt_absolute_time() - _button_start;
			PX4_INFO("Button pressed for %f seconds", double(pressed_micros / 1e6));
			// Publish
			ardupilot::indication::Button Button = {};
			Button.button = ardupilot::indication::Button::BUTTON_SAFETY;
			Button.press_time = (hrt_absolute_time() - _button_start) / 1e5; // units are 0.1s
			uavcan::Publisher<ardupilot::indication::Button>::broadcast(Button);
		}

		_button_pressed = pressed;

	}
private:
	bool _button_pressed {};
	hrt_abstime _start_time {};
};
} // namespace uavcannode
