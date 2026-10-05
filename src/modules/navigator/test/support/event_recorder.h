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


/**
 * @file event_recorder.h
 *
 * Records the events sent after construction, independent of test order.
 */

#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/topics/event.h>
#include <uORB/uORB.h>

namespace navigator_test
{

class EventRecorder
{
public:
	EventRecorder()
	{
		// A late first subscription would only see the newest event, so make sure the topic exists.
		if (!_event_sub.advertised()) {
			event_s event{};
			(void)orb_advertise(ORB_ID(event), &event);
		}

		event_s event{};

		while (_event_sub.update(&event)) {}
	}

	/** Return whether an event with this id was sent since the last call. */
	bool sent(uint32_t event_id)
	{
		bool found = false;
		event_s event{};

		while (_event_sub.update(&event)) {
			found |= event.id == event_id;
		}

		return found;
	}

private:
	uORB::Subscription _event_sub{ORB_ID(event)};
};

} // namespace navigator_test
