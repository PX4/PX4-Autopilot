/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once
#include <drivers/drv_hrt.h>
#include <lib/remote_id/RidState.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/rid_identity_request.h>
#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/param/GetSet.hpp>

class RidIdentityClient
{
public:
	explicit RidIdentityClient(uavcan::INode &node);
	int init();
	bool acceptArmStatus(uint8_t node_id, hrt_abstime now);
	void update();
private:
	using ParamCallback =
		uavcan::MethodBinder<RidIdentityClient *, void (RidIdentityClient::*)(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &)>;
	using InfoCallback =
		uavcan::MethodBinder<RidIdentityClient *, void (RidIdentityClient::*)(const uavcan::ServiceCallResult<uavcan::protocol::GetNodeInfo> &)>;
	using StatusCallback =
		uavcan::MethodBinder<RidIdentityClient *, void (RidIdentityClient::*)(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus> &)>;
	void nodeStatus(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus> &msg);
	void paramResponse(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result);
	void infoResponse(const uavcan::ServiceCallResult<uavcan::protocol::GetNodeInfo> &result);
	void fail(uint8_t result, const char *text);
	void publish(uint8_t result, const char *text, bool invalidate = false);
	void start();
	uavcan::ServiceClient<uavcan::protocol::param::GetSet, ParamCallback> _param_client;
	uavcan::ServiceClient<uavcan::protocol::GetNodeInfo, InfoCallback> _info_client;
	uavcan::Subscriber<uavcan::protocol::NodeStatus, StatusCallback> _status_sub;
	uORB::Subscription _request_sub{ORB_ID(rid_identity_request)};
	uORB::Publication<rid_identity_s> _identity_pub{ORB_ID(rid_identity)};
	rid_identity_request_s _request{};
	rid_identity_s _cached{};
	rid::IdentityReader _reader{};
	hrt_abstime _last_success{0};
	hrt_abstime _last_arm{0};
	hrt_abstime _other_arm{0};
	hrt_abstime _next_read{0};
	hrt_abstime _cycle_started{0};
	hrt_abstime _transaction_started{0};
	uint8_t _node_id{0};
	uint8_t _uid[6] {};
	uint32_t _last_uptime{0};
	bool _have_uptime{false};
	unsigned _attempts{0};
	bool _published_valid{false};
	bool _active{false};
	bool _inflight{false};
	bool _need_info{true};
};
