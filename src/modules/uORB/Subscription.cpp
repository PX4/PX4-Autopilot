/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
#include "topics/parameter_update.h"
#include "topics/actuator_controls.h"
#include "topics/vehicle_gps_position.h"
#include "topics/sensor_combined.h"

namespace uORB
{

bool __EXPORT SubscriptionBase::updated()
{
	bool isUpdated = false;
	orb_check(_handle, &isUpdated);
	return isUpdated;
}

template<class T>
Subscription<T>::Subscription(
	List<SubscriptionBase *> * list,
	const struct orb_metadata *meta, unsigned interval) :
	T(), // initialize data structure to zero
	SubscriptionBase(list, meta) {
	setHandle(orb_subscribe(getMeta()));
	orb_set_interval(getHandle(), interval);
}

template<class T>
Subscription<T>::~Subscription() {}

template<class T>
void * Subscription<T>::getDataVoidPtr() {
	return (void *)(T *)(this);
}

template<class T>
T Subscription<T>::getData() {
	return T(*this);
}

template class __EXPORT Subscription<parameter_update_s>;
template class __EXPORT Subscription<actuator_controls_s>;
template class __EXPORT Subscription<vehicle_gps_position_s>;
template class __EXPORT Subscription<sensor_combined_s>;

} // namespace uORB
