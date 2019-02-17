/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include "SubscriptionArray.hpp"

#include <string.h>

SubscriptionArray::~SubscriptionArray()
{
	cleanup();
}

void SubscriptionArray::cleanup()
{
	for (int i = 0; i < _subscriptions_count; ++i) {
		delete _subscriptions[i];
	}

	delete[] _subscriptions;
	_subscriptions = nullptr;
}

bool SubscriptionArray::resizeSubscriptions()
{
	const int new_size = _subscriptions_size == 0 ? 4 : _subscriptions_size * 2;
	uORB::SubscriptionNode **new_array = new uORB::SubscriptionNode*[new_size];

	if (!new_array) {
		return false;
	}

	if (_subscriptions) {
		memcpy(new_array, _subscriptions, sizeof(uORB::SubscriptionNode *)*_subscriptions_count);
		delete[] _subscriptions;
	}

	_subscriptions = new_array;
	_subscriptions_size = new_size;

	return true;
}

void SubscriptionArray::update()
{
	for (int i = 0; i < _subscriptions_count; ++i) {
		_subscriptions[i]->update();
	}
}

void SubscriptionArray::forcedUpdate()
{
	for (int i = 0; i < _subscriptions_count; ++i) {
		_subscriptions[i]->forcedUpdate();
	}
}
