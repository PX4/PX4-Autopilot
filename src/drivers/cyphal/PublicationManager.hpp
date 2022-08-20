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
 * @file PublicationManager.hpp
 *
 * Manages the dynamic (run-time configurable) UAVCAN publications
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>

#ifndef CONFIG_CYPHAL_GNSS_PUBLISHER
#define CONFIG_CYPHAL_GNSS_PUBLISHER 0
#endif

#ifndef CONFIG_CYPHAL_ESC_CONTROLLER
#define CONFIG_CYPHAL_ESC_CONTROLLER 0
#endif

#ifndef CONFIG_CYPHAL_READINESS_PUBLISHER
#define CONFIG_CYPHAL_READINESS_PUBLISHER 0
#endif

#ifndef CONFIG_CYPHAL_UORB_ACTUATOR_OUTPUTS_PUBLISHER
#define CONFIG_CYPHAL_UORB_ACTUATOR_OUTPUTS_PUBLISHER 0
#endif

#ifndef CONFIG_CYPHAL_UORB_SENSOR_GPS_PUBLISHER
#define CONFIG_CYPHAL_UORB_SENSOR_GPS_PUBLISHER 0
#endif

/* Preprocessor calculation of publisher count */

#define UAVCAN_PUB_COUNT CONFIG_CYPHAL_GNSS_PUBLISHER + \
	CONFIG_CYPHAL_ESC_CONTROLLER + \
	CONFIG_CYPHAL_READINESS_PUBLISHER + \
	CONFIG_CYPHAL_UORB_ACTUATOR_OUTPUTS_PUBLISHER + \
	CONFIG_CYPHAL_UORB_SENSOR_GPS_PUBLISHER

#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include "Publishers/Publisher.hpp"
#include "CanardInterface.hpp"

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/sensor_gps.h>

#include "Actuators/EscClient.hpp"
#include "Publishers/udral/Readiness.hpp"
#include "Publishers/udral/Gnss.hpp"
#include "Publishers/uORB/uorb_publisher.hpp"

typedef struct {
	UavcanPublisher *(*create_pub)(CanardHandle &handle, UavcanParamManager &pmgr) {};
	const char *subject_name;
	const uint8_t instance;
} UavcanDynPubBinder;

class PublicationManager
{
public:
	PublicationManager(CanardHandle &handle, UavcanParamManager &pmgr) : _canard_handle(handle), _param_manager(pmgr) {}
	~PublicationManager();

	void update();
	void printInfo();
	void updateParams();

private:
	void updateDynamicPublications();

	CanardHandle &_canard_handle;
	UavcanParamManager &_param_manager;
	List<UavcanPublisher *> _dynpublishers;


	const UavcanDynPubBinder _uavcan_pubs[UAVCAN_PUB_COUNT] {
#if CONFIG_CYPHAL_GNSS_PUBLISHER
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanPublisher *
			{
				return new UavcanGnssPublisher(handle, pmgr, 0);
			},
			"gps",
			0
		},
#endif
#if CONFIG_CYPHAL_ESC_CONTROLLER
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanPublisher *
			{
				return new UavcanEscController(handle, pmgr);
			},
			"esc",
			0
		},
#endif
#if CONFIG_CYPHAL_READINESS_PUBLISHER
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanPublisher *
			{
				return new UavcanReadinessPublisher(handle, pmgr, 0);
			},
			"readiness",
			0
		},
#endif
#if CONFIG_CYPHAL_UORB_ACTUATOR_OUTPUTS_PUBLISHER
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanPublisher *
			{
				return new uORB_over_UAVCAN_Publisher<actuator_outputs_s>(handle, pmgr, ORB_ID(actuator_outputs));
			},
			"uorb.actuator_outputs",
			0
		},
#endif
#if CONFIG_CYPHAL_UORB_SENSOR_GPS_PUBLISHER
		{
			[](CanardHandle & handle, UavcanParamManager & pmgr) -> UavcanPublisher *
			{
				return new uORB_over_UAVCAN_Publisher<sensor_gps_s>(handle, pmgr, ORB_ID(sensor_gps));
			},
			"uorb.sensor_gps",
			0
		},
#endif
	};
};
