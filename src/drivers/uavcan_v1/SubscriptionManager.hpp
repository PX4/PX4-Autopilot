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

#ifndef CONFIG_UAVCAN_V1_ESC_SUBSCRIBER
#define CONFIG_UAVCAN_V1_ESC_SUBSCRIBER 0
#endif

#ifndef CONFIG_UAVCAN_V1_GNSS_SUBSCRIBER_0
#define CONFIG_UAVCAN_V1_GNSS_SUBSCRIBER_0 0
#endif

#ifndef CONFIG_UAVCAN_V1_GNSS_SUBSCRIBER_1
#define CONFIG_UAVCAN_V1_GNSS_SUBSCRIBER_1 0
#endif

#ifndef CONFIG_UAVCAN_V1_BMS_SUBSCRIBER
#define CONFIG_UAVCAN_V1_BMS_SUBSCRIBER 0
#endif

#ifndef CONFIG_UAVCAN_V1_UORB_SENSOR_GPS_SUBSCRIBER
#define CONFIG_UAVCAN_V1_UORB_SENSOR_GPS_SUBSCRIBER 0
#endif

/* Preprocessor calculation of Subscribers count */

#define UAVCAN_SUB_COUNT CONFIG_UAVCAN_V1_ESC_SUBSCRIBER + \
	CONFIG_UAVCAN_V1_GNSS_SUBSCRIBER_0 + \
	CONFIG_UAVCAN_V1_GNSS_SUBSCRIBER_1 + \
	CONFIG_UAVCAN_V1_BMS_SUBSCRIBER + \
	CONFIG_UAVCAN_V1_UORB_SENSOR_GPS_SUBSCRIBER

#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include "Subscribers/DynamicPortSubscriber.hpp"
#include "CanardInterface.hpp"

#include "ServiceClients/GetInfo.hpp"
#include "ServiceClients/Access.hpp"
#include "Subscribers/BaseSubscriber.hpp"
#include "Subscribers/Heartbeat.hpp"
#include "Subscribers/DS-015/Battery.hpp"
#include "Subscribers/DS-015/Esc.hpp"
#include "Subscribers/DS-015/Gnss.hpp"
#include "Subscribers/legacy/LegacyBatteryInfo.hpp"
#include "Subscribers/uORB/sensor_gps.hpp"

typedef struct {
	UavcanDynamicPortSubscriber *(*create_sub)(CanardInstance &ins, UavcanParamManager &pmgr) {};
	const char *subject_name;
	const uint8_t instance;
} UavcanDynSubBinder;

class SubscriptionManager
{
public:
	SubscriptionManager(CanardInstance &ins, UavcanParamManager &pmgr) : _canard_instance(ins), _param_manager(pmgr) {}
	~SubscriptionManager();

	void subscribe();
	void printInfo();
	void updateParams();

private:
	void updateDynamicSubscriptions();

	CanardInstance &_canard_instance;
	UavcanParamManager &_param_manager;
	UavcanDynamicPortSubscriber *_dynsubscribers {nullptr};

	UavcanHeartbeatSubscriber _heartbeat_sub {_canard_instance};

#if UAVCAN_V1_GETINFO_RESPONDER
	// GetInfo response
	UavcanGetInfoResponse _getinfo_rsp {_canard_instance};
#endif

	// Process register requests
	UavcanAccessResponse  _access_rsp {_canard_instance, _param_manager};

	const UavcanDynSubBinder _uavcan_subs[UAVCAN_SUB_COUNT] {
#if CONFIG_UAVCAN_V1_ESC_SUBSCRIBER
		{
			[](CanardInstance & ins, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanEscSubscriber(ins, pmgr, 0);
			},
			"esc",
			0
		},
#endif
#if CONFIG_UAVCAN_V1_GNSS_SUBSCRIBER_0
		{
			[](CanardInstance & ins, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanGnssSubscriber(ins, pmgr, 0);
			},
			"gps",
			0
		},
#endif
#if CONFIG_UAVCAN_V1_GNSS_SUBSCRIBER_1 //FIXME decouple instanceing
		{
			[](CanardInstance & ins, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanGnssSubscriber(ins, pmgr, 1);
			},
			"gps",
			1
		},
#endif
#if CONFIG_UAVCAN_V1_BMS_SUBSCRIBER
		{
			[](CanardInstance & ins, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanBmsSubscriber(ins, pmgr, 0);
			},
			"energy_source",
			0
		},
#endif
#if 0 //Obsolete to be removed
		{
			[](CanardInstance & ins, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UavcanLegacyBatteryInfoSubscriber(ins, pmgr, 0);
			},
			"legacy_bms",
			0
		},
#endif
#if CONFIG_UAVCAN_V1_UORB_SENSOR_GPS_SUBSCRIBER
		{
			[](CanardInstance & ins, UavcanParamManager & pmgr) -> UavcanDynamicPortSubscriber *
			{
				return new UORB_over_UAVCAN_sensor_gps_Subscriber(ins, pmgr, 0);
			},
			"uorb.sensor_gps",
			0
		},
#endif
	};
};
