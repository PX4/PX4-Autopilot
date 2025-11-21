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
 * @file SubscriptionManager.hpp
 *
 * Manages the UAVCAN subscriptions
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#ifndef CONFIG_CYPHAL_ESC_SUBSCRIBER
#define CONFIG_CYPHAL_ESC_SUBSCRIBER 0
#endif

#ifndef CONFIG_CYPHAL_ESC_CONTROLLER
#define CONFIG_CYPHAL_ESC_CONTROLLER 0
#endif

#ifndef CONFIG_CYPHAL_GNSS_SUBSCRIBER_0
#define CONFIG_CYPHAL_GNSS_SUBSCRIBER_0 0
#endif

#ifndef CONFIG_CYPHAL_GNSS_SUBSCRIBER_1
#define CONFIG_CYPHAL_GNSS_SUBSCRIBER_1 0
#endif

#ifndef CONFIG_CYPHAL_BMS_SUBSCRIBER
#define CONFIG_CYPHAL_BMS_SUBSCRIBER 0
#endif

#ifndef CONFIG_CYPHAL_UORB_SENSOR_GPS_SUBSCRIBER
#define CONFIG_CYPHAL_UORB_SENSOR_GPS_SUBSCRIBER 0
#endif

/* Preprocessor calculation of Subscribers count */

#define UAVCAN_SUB_COUNT CONFIG_CYPHAL_ESC_SUBSCRIBER + \
	CONFIG_CYPHAL_GNSS_SUBSCRIBER_0 + \
	8 * CONFIG_CYPHAL_ESC_CONTROLLER + \
	CONFIG_CYPHAL_GNSS_SUBSCRIBER_1 + \
	CONFIG_CYPHAL_BMS_SUBSCRIBER + \
	CONFIG_CYPHAL_UORB_SENSOR_GPS_SUBSCRIBER

#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <uavcan/node/port/List_0_1.h>
#include "Actuators/EscClient.hpp"
#include "Subscribers/DynamicPortSubscriber.hpp"
#include "CanardInterface.hpp"

#include "ServiceClients/GetInfo.hpp"
#include "ServiceClients/Access.hpp"
#include "ServiceClients/List.hpp"
#include "Subscribers/BaseSubscriber.hpp"
#include "Subscribers/Heartbeat.hpp"
#include "Subscribers/udral/Battery.hpp"
#include "Subscribers/udral/Esc.hpp"
#include "Subscribers/udral/Gnss.hpp"
#include "Subscribers/legacy/LegacyBatteryInfo.hpp"
#include "Subscribers/uORB/uorb_subscriber.hpp"

typedef struct {
	UavcanDynamicPortSubscriber *(*create_sub)(CanardHandle &handle, UavcanParamManager &pmgr) {};
	const char *subject_name;
	const uint8_t instance;
} UavcanDynSubBinder;

class SubscriptionManager
{
public:
	SubscriptionManager(CanardHandle &handle, UavcanParamManager &pmgr) : _canard_handle(handle), _param_manager(pmgr) {}
	~SubscriptionManager();

	void subscribe();
	void printInfo();
	void updateParams();
	void fillSubjectIdList(uavcan_node_port_SubjectIDList_0_1 &subscribers_list);

private:
	void updateDynamicSubscriptions();

	CanardHandle &_canard_handle;
	UavcanParamManager &_param_manager;
	UavcanDynamicPortSubscriber *_dynsubscribers {nullptr};

	UavcanHeartbeatSubscriber _heartbeat_sub {_canard_handle};

#if CONFIG_CYPHAL_GETINFO_RESPONDER
	// GetInfo response
	UavcanGetInfoResponse _getinfo_rsp {_canard_handle};
#endif

	// Process register requests
	UavcanAccessResponse  _access_rsp {_canard_handle, _param_manager};
	UavcanListResponse  _list_rsp {_canard_handle, _param_manager};

	const UavcanDynSubBinder _uavcan_subs[UAVCAN_SUB_COUNT] {
#if CONFIG_CYPHAL_ESC_SUBSCRIBER
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanEscSubscriber(handle, pmgr, 0);
			},
			"udral.esc",
			0
		},
#endif
#if CONFIG_CYPHAL_ESC_CONTROLLER
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanEscFeedbackSubscriber(handle, pmgr, 0);
			},
			"zubax.feedback",
			0
		},
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanEscFeedbackSubscriber(handle, pmgr, 1);
			},
			"zubax.feedback",
			1
		},
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanEscFeedbackSubscriber(handle, pmgr, 2);
			},
			"zubax.feedback",
			2
		},
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanEscFeedbackSubscriber(handle, pmgr, 3);
			},
			"zubax.feedback",
			3
		},
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanEscFeedbackSubscriber(handle, pmgr, 4);
			},
			"zubax.feedback",
			4
		},
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanEscFeedbackSubscriber(handle, pmgr, 5);
			},
			"zubax.feedback",
			5
		},
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanEscFeedbackSubscriber(handle, pmgr, 6);
			},
			"zubax.feedback",
			6
		},
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanEscFeedbackSubscriber(handle, pmgr, 7);
			},
			"zubax.feedback",
			7
		},
#endif
#if CONFIG_CYPHAL_GNSS_SUBSCRIBER_0
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanGnssSubscriber(handle, pmgr, 0);
			},
			"udral.gps",
			0
		},
#endif
#if CONFIG_CYPHAL_GNSS_SUBSCRIBER_1 //FIXME decouple handletanceing
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanGnssSubscriber(handle, pmgr, 1);
			},
			"udral.gps",
			1
		},
#endif
#if CONFIG_CYPHAL_BMS_SUBSCRIBER
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanBmsSubscriber(handle, pmgr, 0);
			},
			"udral.energy_source",
			0
		},
#endif
#if 0 //Obsolete to be removed
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanLegacyBatteryInfoSubscriber(handle, pmgr, 0);
			},
			"udral.legacy_bms",
			0
		},
#endif
#if CONFIG_CYPHAL_UORB_SENSOR_GPS_SUBSCRIBER
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new uORB_over_UAVCAN_Subscriber<sensor_gps_s>(handle, pmgr, ORB_ID(sensor_gps));
			},
			"uorb.sensor_gps",
			0
		},
#endif
	};
};
