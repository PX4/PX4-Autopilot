/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Author: Pavel Kirienko <pavel.kirienko@gmail.com>
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

#include <px4_platform_common/px4_config.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include "indication_controller.hpp"
#include <uavcan/equipment/indication/LightsCommand.hpp>
#include <uavcan/equipment/indication/BeepCommand.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/led_control.h>
#include <uORB/topics/tune_control.h>
#include <lib/tunes/tunes.h>

namespace
{
unsigned self_light_index = 0;

void cb_light_command(const uavcan::ReceivedDataStructure<uavcan::equipment::indication::LightsCommand> &msg)
{
	uavcan::uint32_t red = 0;
	uavcan::uint32_t green = 0;
	uavcan::uint32_t blue = 0;

	for (auto &cmd : msg.commands) {
		if (cmd.light_id == self_light_index) {
			using uavcan::equipment::indication::RGB565;

			red = uavcan::uint32_t(float(cmd.color.red) *
					       (255.0F / float(RGB565::FieldTypes::red::max())) + 0.5F);

			green = uavcan::uint32_t(float(cmd.color.green) *
						 (255.0F / float(RGB565::FieldTypes::green::max())) + 0.5F);

			blue = uavcan::uint32_t(float(cmd.color.blue) *
						(255.0F / float(RGB565::FieldTypes::blue::max())) + 0.5F);

			red   = uavcan::min<uavcan::uint32_t>(red, 0xFFU);
			green = uavcan::min<uavcan::uint32_t>(green, 0xFFU);
			blue  = uavcan::min<uavcan::uint32_t>(blue, 0xFFU);

			led_control_s led_control;
			led_control.num_blinks = 0;
			led_control.priority = led_control_s::MAX_PRIORITY;
			led_control.mode = led_control_s::MODE_OFF;
			led_control.led_mask = 0xff;
			led_control.color = led_control_s::COLOR_OFF;

			if (red != 0 && blue == 0 && green == 0) {
				led_control.color = led_control_s::COLOR_RED;

			} else if (red == 0 && blue != 0 && green == 0) {
				led_control.color = led_control_s::COLOR_BLUE;

			} else if (red == 0 && blue == 0 && green != 0) {
				led_control.color = led_control_s::COLOR_GREEN;

			} else if (red != 0 && blue == 0 && green != 0) {
				led_control.color = led_control_s::COLOR_YELLOW;

			} else if (red != 0 && blue != 0 && green == 0) {
				led_control.color = led_control_s::COLOR_PURPLE;

			} else if (red != 0 && blue == 0 && green != 0 && red > green) {
				led_control.color = led_control_s::COLOR_AMBER;

			} else if (red == 0 && blue != 0 && green != 0) {
				led_control.color = led_control_s::COLOR_CYAN;

			} else if (red != 0 && blue != 0 && green != 0) {
				led_control.color = led_control_s::COLOR_WHITE;
			}

			if (led_control.color !=  led_control_s::COLOR_OFF) {
				led_control.mode = led_control_s::MODE_ON;
			}

			led_control.timestamp = hrt_absolute_time();
			uORB::Publication<led_control_s> led_control_pub{ORB_ID(led_control)};
			led_control_pub.publish(led_control);
		}
	}
}

void cb_beep_command(const uavcan::ReceivedDataStructure<uavcan::equipment::indication::BeepCommand> &msg)
{
	tune_control_s tune_control{};
	tune_control.tune_id = 0;
	tune_control.frequency = (uint16_t)msg.frequency;
	tune_control.duration = uavcan::uint32_t(1000000 * msg.duration);
	tune_control.volume = 0xff;
	uORB::Publication<tune_control_s> tune_control_pub{ORB_ID(tune_control)};
	tune_control.timestamp = hrt_absolute_time();
	tune_control_pub.publish(tune_control);
}


}
int init_indication_controller(uavcan::INode &node)
{
	static uavcan::Subscriber<uavcan::equipment::indication::LightsCommand> sub_light(node);
	static uavcan::Subscriber<uavcan::equipment::indication::BeepCommand> sub_beep(node);

	self_light_index = 0;

	int res = 0;

	res = sub_light.start(cb_light_command);

	if (res != 0) {
		return res;
	}

	res = sub_beep.start(cb_beep_command);

	if (res != 0) {
		return res;
	}

	return 0;
}
