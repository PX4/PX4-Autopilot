/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <pthread.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/events.h>
#include <uORB/uORB.h>

static orb_advert_t orb_event_pub = nullptr;
static pthread_mutex_t publish_event_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint16_t event_sequence{events::initial_event_sequence};

namespace events
{

void send(EventType &event)
{
	event.timestamp = hrt_absolute_time();

	// We need some synchronization here because:
	// - modifying orb_event_pub
	// - the update of event_sequence needs to be atomic
	// - we need to ensure ordering of the sequence numbers: the sequence we set here
	//   has to be the one published next.
	pthread_mutex_lock(&publish_event_mutex);
	event.event_sequence = ++event_sequence; // Set the sequence here so we're able to detect uORB queue overflows

	if (orb_event_pub != nullptr) {
		orb_publish(ORB_ID(event), orb_event_pub, &event);

	} else {
		orb_event_pub = orb_advertise(ORB_ID(event), &event);
	}

	pthread_mutex_unlock(&publish_event_mutex);
}

} /* namespace events */
