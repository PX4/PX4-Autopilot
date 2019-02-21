/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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

#include "Subscription.hpp"
#include <px4_defines.h>

namespace uORB
{

SubscriptionBase::SubscriptionBase(const struct orb_metadata *meta, unsigned interval, unsigned instance) :
	_meta(meta),
	_instance(instance)
{
	if (instance > 0) {
		_handle = orb_subscribe_multi(_meta, instance);

	} else {
		_handle = orb_subscribe(_meta);
	}

	if (_handle < 0) {
		PX4_ERR("%s sub failed", _meta->o_name);
	}

	if (interval > 0) {
		orb_set_interval(_handle, interval);
	}
}

bool SubscriptionBase::updated()
{
	bool isUpdated = false;

	if (orb_check(_handle, &isUpdated) != PX4_OK) {
		PX4_ERR("%s check failed", _meta->o_name);
	}

	return isUpdated;
}

bool SubscriptionBase::update(void *data)
{
	bool orb_updated = false;

	if (updated()) {
		if (orb_copy(_meta, _handle, data) != PX4_OK) {
			PX4_ERR("%s copy failed", _meta->o_name);

		} else {
			orb_updated = true;
		}
	}

	return orb_updated;
}

SubscriptionBase::~SubscriptionBase()
{
	if (orb_unsubscribe(_handle) != PX4_OK) {
		PX4_ERR("%s unsubscribe failed", _meta->o_name);
	}
}

SubscriptionNode::SubscriptionNode(const struct orb_metadata *meta, unsigned interval, unsigned instance,
				   List<SubscriptionNode *> *list)
	: SubscriptionBase(meta, interval, instance)
{
	if (list != nullptr) {
		list->add(this);
	}
}

} // namespace uORB
