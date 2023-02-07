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
 * @file DynamicPortSubscriber.hpp
 *
 * Defines basic functionality of Cyphal subscriber class with Non-fixed unregulated port identifier
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>

#include <lib/parameters/param.h>

#include <uavcan/_register/Access_1_0.h>

#include "../CanardInterface.hpp"
#include "../ParamManager.hpp"
#include "BaseSubscriber.hpp"

class UavcanDynamicPortSubscriber : public UavcanBaseSubscriber
{
public:
	UavcanDynamicPortSubscriber(CanardHandle &handle, UavcanParamManager &pmgr, const char *prefix_name,
				    const char *subject_name,  uint8_t instance = 0) :
		UavcanBaseSubscriber(handle, prefix_name, subject_name, instance), _param_manager(pmgr) { };

	void updateParam()
	{
		SubjectSubscription *curSubj = &_subj_sub;
		bool unsubscribeRequired = false;
		bool subscribeRequired = false;

		while (curSubj != nullptr) {
			char uavcan_param[90];
			snprintf(uavcan_param, sizeof(uavcan_param), "uavcan.sub.%s%s.%d.id", _prefix_name, curSubj->_subject_name, _instance);

			// Set _port_id from _uavcan_param
			uavcan_register_Value_1_0 value;

			if (_param_manager.GetParamByName(uavcan_param, value)) {
				uint16_t new_id = value.natural16.value.elements[0];

				/* FIXME how about partial subscribing */
				if (curSubj->_canard_sub.port_id != new_id) {
					if (new_id == CANARD_PORT_ID_UNSET) {
						// Cancel subscription
						unsubscribeRequired = true;

					} else {
						if (curSubj->_canard_sub.port_id != CANARD_PORT_ID_UNSET) {
							// Already active; unsubscribe first
							unsubscribeRequired = true;
						}

						// Subscribe on the new port ID
						curSubj->_canard_sub.port_id = (CanardPortID)new_id;
						PX4_INFO("Subscribing %s%s.%d on port %d", _prefix_name, curSubj->_subject_name, _instance,
							 curSubj->_canard_sub.port_id);
						subscribeRequired = true;
					}
				}

			} else if (curSubj->_canard_sub.port_id != CANARD_PORT_ID_UNSET) { // No valid sub id unsubscribe when neccesary
				// Already active; unsubscribe first
				unsubscribeRequired = true;
			}

			curSubj = curSubj->next;
		}

		if (unsubscribeRequired) {
			unsubscribe();
		}

		if (subscribeRequired) {
			subscribe();
		}
	};

	UavcanDynamicPortSubscriber *next()
	{
		return _next_sub;
	}

	void setNext(UavcanDynamicPortSubscriber *next)
	{
		_next_sub = next;
	}

protected:
	UavcanParamManager &_param_manager;
	UavcanDynamicPortSubscriber *_next_sub {nullptr};
};
