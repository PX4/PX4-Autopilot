/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "RoverControllerWorkItem.hpp"

RoverControllerWorkItem::RoverControllerWorkItem(const char *name, const orb_metadata *trigger) :
	WorkItem(name, px4::wq_configurations::nav_and_controllers),
	_trigger_sub{this, trigger}
{
}

bool RoverControllerWorkItem::init()
{
	const bool registered = _trigger_sub.registerCallback();
	_trigger_sub.unregisterCallback();
	return registered;
}

bool RoverControllerWorkItem::setEnabled(bool enabled)
{
	if (enabled && !_enabled) {
		_enabled = _trigger_sub.registerCallback();

		if (_enabled) {
			onEnable();
			ScheduleNow();
		}

	} else if (!enabled && _enabled) {
		_trigger_sub.unregisterCallback();
		_enabled = false;
	}

	return _enabled == enabled;
}

bool RoverControllerWorkItem::reregisterTrigger()
{
	if (!_enabled) {
		return true; // Not registered, setEnabled() registers it at the end of the list
	}

	_trigger_sub.unregisterCallback();
	_enabled = _trigger_sub.registerCallback();
	return _enabled;
}

void RoverControllerWorkItem::Run()
{
	if (_enabled) {
		runController();
	}
}
