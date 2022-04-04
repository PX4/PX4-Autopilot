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

/**
 * @file Subscription.cpp
 *
 */

#include <uORB/uORB.h>
#include "Subscription.hpp"
#include "uORBManager.hpp"

namespace uORB
{

bool Subscription::subscribe(bool advertise)
{
	if (orb_advert_valid(_node)) {
		return true;
	}

	if (_orb_id != ORB_ID::INVALID) {
		_node = uORB::Manager::orb_add_internal_subscriber(_orb_id, _instance, &_last_generation, advertise);

		if (orb_advert_valid(_node)) {
			_advertiser = advertise;
			return true;
		}
	}

	return false;
}

void Subscription::unsubscribe()
{
	if (orb_advert_valid(_node)) {
		uORB::Manager::orb_remove_internal_subscriber(_node, _advertiser);
	}

	_last_generation = 0;
}

bool Subscription::ChangeInstance(uint8_t instance)
{
	if (instance != _instance) {
		// Subscribe to the new existing node
		unsigned generation;

		if (orb_exists(get_topic(), instance) != PX4_OK) {
			return false;
		}

		orb_advert_t new_node = uORB::Manager::orb_add_internal_subscriber(_orb_id, instance, &generation, false);

		if (orb_advert_valid(new_node)) {
			unsubscribe();
			_node = new_node;
			_instance = instance;
			_last_generation = generation;
			return true;
		}

	} else {
		// already on desired index
		return true;
	}

	return false;
}

} // namespace uORB
