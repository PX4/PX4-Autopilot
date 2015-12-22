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
#include "topics/hil_sensor.h"
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
#include "topics/battery_status.h"
#include "topics/optical_flow.h"
#include "topics/distance_sensor.h"
#include "topics/home_position.h"
#include "topics/vehicle_control_mode.h"
#include "topics/actuator_armed.h"
#include "topics/att_pos_mocap.h"
#include "topics/vision_position_estimate.h"

#include <px4_defines.h>

namespace uORB
{

SubscriptionBase::SubscriptionBase(const struct orb_metadata *meta,
				   unsigned interval, unsigned instance) :
	_meta(meta),
	_instance(instance),
	_handle()
{
	if (_instance > 0) {
		_handle =  orb_subscribe_multi(
				   getMeta(), instance);

	} else {
		_handle =  orb_subscribe(getMeta());
	}

	if (_handle < 0) { warnx("sub failed"); }

	if (interval > 0) {
		orb_set_interval(getHandle(), interval);
	}
}

bool SubscriptionBase::updated()
{
	bool isUpdated = false;
	int ret = orb_check(_handle, &isUpdated);

	if (ret != PX4_OK) { warnx("orb check failed"); }

	return isUpdated;
}

void SubscriptionBase::update(void *data)
{
	if (updated()) {
		int ret = orb_copy(_meta, _handle, data);

		if (ret != PX4_OK) { warnx("orb copy failed"); }
	}
}

SubscriptionBase::~SubscriptionBase()
{
	int ret = orb_unsubscribe(_handle);

	if (ret != PX4_OK) { warnx("orb unsubscribe failed"); }
}

template <class T>
Subscription<T>::Subscription(const struct orb_metadata *meta,
			      unsigned interval,
			      int instance,
			      List<SubscriptionNode *> *list) :
	SubscriptionNode(meta, interval, instance, list),
	_data() // initialize data structure to zero
{
}

template <class T>
Subscription<T>::~Subscription()
{
}

template <class T>
void Subscription<T>::update()
{
	SubscriptionBase::update((void *)(&_data));
}

template <class T>
const T &Subscription<T>::get() { return _data; }

template class __EXPORT Subscription<parameter_update_s>;
template class __EXPORT Subscription<actuator_controls_s>;
template class __EXPORT Subscription<vehicle_gps_position_s>;
template class __EXPORT Subscription<satellite_info_s>;
template class __EXPORT Subscription<sensor_combined_s>;
template class __EXPORT Subscription<hil_sensor_s>;
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
template class __EXPORT Subscription<battery_status_s>;
template class __EXPORT Subscription<home_position_s>;
template class __EXPORT Subscription<optical_flow_s>;
template class __EXPORT Subscription<distance_sensor_s>;
template class __EXPORT Subscription<att_pos_mocap_s>;
template class __EXPORT Subscription<vision_position_estimate_s>;

} // namespace uORB
