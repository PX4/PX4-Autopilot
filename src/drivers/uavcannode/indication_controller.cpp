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

#include "indication_controller.hpp"
#include <uavcan/equipment/indication/LightsCommand.hpp>
#include "led.hpp"

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
		}

		if (cmd.light_id == self_light_index + 1) {
			static int c = 0;

			if (c++ % 100 == 0) {
				::syslog(LOG_INFO, "rgb:%d %d %d hz %d\n", red, green, blue,  int(cmd.color.red));
			}

			rgb_led(red, green, blue, int(cmd.color.red));
			break;
		}
	}
}
}

int init_indication_controller(uavcan::INode &node)
{
	static uavcan::Subscriber<uavcan::equipment::indication::LightsCommand> sub_light(node);

	self_light_index = 0;

	int res = 0;

	res = sub_light.start(cb_light_command);

	if (res != 0) {
		return res;
	}

	return 0;
}
