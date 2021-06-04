/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file SubscriptionManager.cpp
 *
 * Manages the UAVCAN subscriptions
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */


#include "SubscriptionManager.hpp"



void SubscriptionManager::subscribe()
{
	_heartbeat_sub.subscribe();
	_getinfo_rsp.subscribe();
	_access_rsp.subscribe();

	for (auto &sub : _uavcan_subs) {
		param_t param_handle = param_find(sub.px4_name);

		if (param_handle == PARAM_INVALID) {
			PX4_ERR("Param %s not found", sub.px4_name);
			break;
		}

		if ((param_type(param_handle) == PARAM_TYPE_INT32)) {
			int32_t port_id {};
			param_get(param_handle, &port_id);

			if (port_id >= 0) { // PortID is set create a subscriber
				UavcanDynamicPortSubscriber *dynsub = sub.create_sub(_canard_instance, _param_manager);

				if (_dynsubscribers != NULL) {
					_dynsubscribers->setNext(dynsub);
				}

				_dynsubscribers = dynsub;
				dynsub->updateParam();
			}
		}
	}
}

void SubscriptionManager::printInfo()
{

}

void SubscriptionManager::updateParams()
{
	//TODO dynamically update params and unsubscribe
}
