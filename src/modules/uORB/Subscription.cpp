/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
#include "topics/satellite_info.h"
#include "topics/sensor_combined.h"
#include "topics/vehicle_attitude.h"
#include "topics/vehicle_global_position.h"
#include "topics/encoders.h"
#include "topics/position_setpoint_triplet.h"
#include "topics/vehicle_status.h"
#include "topics/manual_control_setpoint.h"
#include "topics/vehicle_local_position_setpoint.h"
#include "topics/vehicle_local_position.h"
#include "topics/vehicle_attitude_setpoint.h"
#include "topics/vehicle_rates_setpoint.h"
#include "topics/rc_channels.h"

namespace uORB
{

template<class T>
Subscription<T>::Subscription(
	const struct orb_metadata *meta,
	unsigned interval,
	List<SubscriptionNode *> * list) :
	T(), // initialize data structure to zero
	SubscriptionNode(meta, interval, list) {
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
template class __EXPORT Subscription<satellite_info_s>;
template class __EXPORT Subscription<sensor_combined_s>;
template class __EXPORT Subscription<vehicle_attitude_s>;
template class __EXPORT Subscription<vehicle_global_position_s>;
template class __EXPORT Subscription<encoders_s>;
template class __EXPORT Subscription<position_setpoint_triplet_s>;
template class __EXPORT Subscription<vehicle_status_s>;
template class __EXPORT Subscription<manual_control_setpoint_s>;
template class __EXPORT Subscription<vehicle_local_position_setpoint_s>;
template class __EXPORT Subscription<vehicle_local_position_s>;
template class __EXPORT Subscription<vehicle_attitude_setpoint_s>;
template class __EXPORT Subscription<vehicle_rates_setpoint_s>;
template class __EXPORT Subscription<rc_channels_s>;
template class __EXPORT Subscription<vehicle_control_mode_s>;
template class __EXPORT Subscription<actuator_armed_s>;

} // namespace uORB
