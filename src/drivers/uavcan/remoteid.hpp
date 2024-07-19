/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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

#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/open_drone_id_operator_id.h>
#include <uORB/topics/open_drone_id_arm_status.h>
#include <uORB/topics/open_drone_id_self_id.h>
#include <uORB/topics/open_drone_id_system.h>

#include <uavcan/uavcan.hpp>
#include <dronecan/remoteid/BasicID.hpp>
#include <dronecan/remoteid/Location.hpp>
#include <dronecan/remoteid/SelfID.hpp>
#include <dronecan/remoteid/System.hpp>
#include <dronecan/remoteid/ArmStatus.hpp>
#include <dronecan/remoteid/OperatorID.hpp>

#include <px4_platform_common/module_params.h>

class UavcanRemoteIDController : public ModuleParams
{
public:
	UavcanRemoteIDController(uavcan::INode &node);
	~UavcanRemoteIDController() = default;

	int init();

private:
	typedef uavcan::MethodBinder<UavcanRemoteIDController *, void (UavcanRemoteIDController::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;

	static constexpr unsigned MAX_RATE_HZ = 1;
	uavcan::TimerEventForwarder<TimerCbBinder> _timer;

	void periodic_update(const uavcan::TimerEvent &);

	void send_basic_id();
	void send_location();
	void send_self_id();
	void send_system();
	void send_operator_id();

	void arm_status_sub_cb(const uavcan::ReceivedDataStructure<dronecan::remoteid::ArmStatus> &msg);

	uavcan::INode &_node;

	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _open_drone_id_operator_id{ORB_ID(open_drone_id_operator_id)};
	uORB::Subscription _open_drone_id_self_id{ORB_ID(open_drone_id_self_id)};
	uORB::Subscription _open_drone_id_system{ORB_ID(open_drone_id_system)};
	uORB::Publication<open_drone_id_arm_status_s> _open_drone_id_arm_status_pub{ORB_ID(open_drone_id_arm_status)};

	uavcan::Publisher<dronecan::remoteid::BasicID> _uavcan_pub_remoteid_basicid;
	uavcan::Publisher<dronecan::remoteid::Location> _uavcan_pub_remoteid_location;
	uavcan::Publisher<dronecan::remoteid::SelfID> _uavcan_pub_remoteid_self_id;
	uavcan::Publisher<dronecan::remoteid::System> _uavcan_pub_remoteid_system;
	uavcan::Publisher<dronecan::remoteid::OperatorID> _uavcan_pub_remoteid_operator_id;

	using ArmStatusBinder = uavcan::MethodBinder < UavcanRemoteIDController *,
	      void (UavcanRemoteIDController::*)(const uavcan::ReceivedDataStructure<dronecan::remoteid::ArmStatus> &) >;

	uavcan::Subscriber<dronecan::remoteid::ArmStatus, ArmStatusBinder> _uavcan_sub_arm_status;
};
