/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "px4_sdk/components/overrides.h"

#include <cassert>

using namespace px4_sdk;

ConfigOverrides::ConfigOverrides(rclcpp::Node &node, const std::string &topic_namespace_prefix)
	: _node(node)
{
	_config_overrides_pub = _node.create_publisher<px4_msgs::msg::ConfigOverrides>(
					topic_namespace_prefix + "/fmu/in/config_overrides_request", 1);
}

void ConfigOverrides::controlAutoDisarm(bool enabled)
{
	_current_overrides.disable_auto_disarm = !enabled;
	update();
}

void ConfigOverrides::deferFailsafes(bool enabled, int timeout_s)
{
	_current_overrides.defer_failsafes = enabled;
	_current_overrides.defer_failsafes_timeout_s = timeout_s;
	update();
}

void ConfigOverrides::update()
{
	if (_is_setup) {
		_current_overrides.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
		_config_overrides_pub->publish(_current_overrides);

	} else {
		_require_update_after_setup = true;
	}
}

void ConfigOverrides::setup(uint8_t type, uint8_t id)
{
	assert(!_is_setup);
	_current_overrides.source_type = type;
	_current_overrides.source_id = id;
	_is_setup = true;

	if (_require_update_after_setup) {
		update();
		_require_update_after_setup = false;
	}
}
