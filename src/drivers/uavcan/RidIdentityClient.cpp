/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "RidIdentityClient.hpp"
#include <string.h>
using namespace time_literals;

RidIdentityClient::RidIdentityClient(uavcan::INode &node) : _param_client(node), _info_client(node), _status_sub(node)
{
	_param_client.setCallback(ParamCallback(this, &RidIdentityClient::paramResponse));
	_info_client.setCallback(InfoCallback(this, &RidIdentityClient::infoResponse));
	_param_client.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(600));
	_info_client.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(600));
}

int RidIdentityClient::init()
{
	return _status_sub.start(StatusCallback(this, &RidIdentityClient::nodeStatus));
}

void RidIdentityClient::nodeStatus(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus> &msg)
{
	if (msg.getSrcNodeID().get() != _node_id) { return; }

	if (_have_uptime && msg.uptime_sec < _last_uptime) {
		_param_client.cancelAllCalls(); _info_client.cancelAllCalls();
		_cached = {}; _last_success = 0; _last_arm = 0;
		_active = false; _inflight = false; _next_read = hrt_absolute_time();

		if (_request.timestamp) { publish(rid_identity_s::RESULT_READING, "RID reboot detected", _published_valid); }
	}

	_last_uptime = msg.uptime_sec;
	_have_uptime = true;
}

bool RidIdentityClient::acceptArmStatus(uint8_t node_id, hrt_abstime now)
{
	if (node_id == 0 || node_id > 127) { return false; }

	if (_node_id == 0 || (node_id != _node_id && !rid::recent(_last_arm, now, rid::HEARTBEAT_TIMEOUT_US))) {
		_param_client.cancelAllCalls();
		_info_client.cancelAllCalls();
		_node_id = node_id;
		_have_uptime = false;
		_last_success = 0;
		_last_arm = now;
		_other_arm = 0;
		_active = false;
		_inflight = false;
		_cached = {};
		_next_read = now;

		if (_request.timestamp) { publish(rid_identity_s::RESULT_READING, "RID binding changed", _published_valid); }
	}

	if (node_id != _node_id) {
		_other_arm = now;
		return false;
	}

	_last_arm = now;
	return true;
}

void RidIdentityClient::publish(uint8_t result, const char *text, bool invalidate)
{
	rid_identity_s out = result == rid_identity_s::RESULT_OK ? _cached : rid_identity_s{};
	// Cached replies retain the CAN verification time; requests cannot extend validity.
	out.timestamp = result == rid_identity_s::RESULT_OK ? _last_success : hrt_absolute_time();
	out.boot_id = _request.boot_id;
	out.session_id = _request.session_id;
	out.request_id = invalidate ? 0 : _request.request_id;
	out.target_system = _request.target_system;
	out.target_component = _request.target_component;
	out.rid_node_id = _node_id;
	out.result = result;
	strncpy(out.error, text, sizeof(out.error) - 1);
	_identity_pub.publish(out);
	_published_valid = result == rid_identity_s::RESULT_OK;

	if (invalidate) { _request = {}; }
}

void RidIdentityClient::start()
{
	_reader.reset();
	_need_info = true;
	_active = true;
	_inflight = false;
	_cycle_started = hrt_absolute_time();

	if (_attempts == 0) { _transaction_started = _cycle_started; }
}

void RidIdentityClient::fail(uint8_t result, const char *text)
{
	_inflight = false;
	_active = false;

	if ((result == rid_identity_s::RESULT_TIMEOUT || result == rid_identity_s::RESULT_CONFLICT) && ++_attempts < 3) {
		_next_read = hrt_absolute_time() + 200_ms;
		return;
	}

	_attempts = 0;
	_cached = {};
	publish(result, text, _published_valid);
	_next_read = hrt_absolute_time() + 2_s;
}

void RidIdentityClient::update()
{
	const hrt_abstime now = hrt_absolute_time();
	rid_identity_request_s request{};

	if (_request_sub.update(&request)) {
		_request = request;
		const bool cache_valid = rid::recent(_last_success, now, rid::IDENTITY_TIMEOUT_US)
					 && rid::recent(_last_arm, now, rid::HEARTBEAT_TIMEOUT_US)
					 && !rid::recent(_other_arm, now, rid::HEARTBEAT_TIMEOUT_US) && _cached.rid_boot_id != 0;

		if (cache_valid) { publish(rid_identity_s::RESULT_OK, ""); }

		if ((!cache_valid && !_active && _attempts == 0) || (request.flags & 1)) {
			_param_client.cancelAllCalls();
			_info_client.cancelAllCalls();
			_attempts = 0;
			_active = false;
			_inflight = false;
			_next_read = now;

			if (!cache_valid) { publish(rid_identity_s::RESULT_READING, "Reading RID identity"); }
		}
	}

	if (_request.timestamp == 0) { return; }

	if (!rid::recent(_last_arm, now, rid::HEARTBEAT_TIMEOUT_US)
	    || rid::recent(_other_arm, now, rid::HEARTBEAT_TIMEOUT_US)) {
		if (_active || now >= _next_read) {
			_param_client.cancelAllCalls(); _info_client.cancelAllCalls();
			_active = false; _inflight = false; _cached = {};
			publish(rid::recent(_other_arm, now, rid::HEARTBEAT_TIMEOUT_US) ? rid_identity_s::RESULT_CONFLICT : rid_identity_s::RESULT_NOT_FOUND,
				"RID missing or multiple RID nodes", _published_valid);
			_next_read = now + 2_s;
		}

		return;
	}

	if ((_active || _attempts != 0) && now - _transaction_started > 20_s) {
		_param_client.cancelAllCalls(); _info_client.cancelAllCalls();
		_attempts = 2;
		fail(rid_identity_s::RESULT_TIMEOUT, "RID transaction deadline");
		return;
	}

	if (_inflight) { return; }

	if (!_active) {
		if (now < _next_read) { return; }

		start();
	}

	if (now - _cycle_started > 6_s) { fail(rid_identity_s::RESULT_TIMEOUT, "RID snapshot deadline"); return; }

	int result;

	if (_need_info) {
		uavcan::protocol::GetNodeInfo::Request req{};
		result = _info_client.call(uavcan::NodeID(_node_id), req);

	} else {
		uavcan::protocol::param::GetSet::Request req{};
		req.name = _reader.name();
		result = _param_client.call(uavcan::NodeID(_node_id), req);
	}

	if (result < 0) { fail(rid_identity_s::RESULT_TIMEOUT, "RID request failed"); } else { _inflight = true; }
}

void RidIdentityClient::infoResponse(const uavcan::ServiceCallResult<uavcan::protocol::GetNodeInfo> &result)
{
	if (!_active || result.getCallID().server_node_id.get() != _node_id) { return; }

	_inflight = false;

	if (!result.isSuccessful()) { fail(rid_identity_s::RESULT_TIMEOUT, "RID node info timeout"); return; }

	const auto &uid = result.getResponse().hardware_version.unique_id;
	memcpy(_uid, &uid.front(), sizeof(_uid));
	_need_info = false;
}

void RidIdentityClient::paramResponse(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result)
{
	if (!_active || result.getCallID().server_node_id.get() != _node_id) { return; }

	_inflight = false;

	if (!result.isSuccessful()) { fail(rid_identity_s::RESULT_TIMEOUT, "RID GetSet timeout"); return; }

	uavcan::protocol::param::GetSet::Response response = result.getResponse();

	if (strcmp(response.name.c_str(), _reader.name()) != 0) { fail(rid_identity_s::RESULT_UNSUPPORTED, "RID parameter unavailable"); return; }

	bool accepted = false;

	if (response.value.is(uavcan::protocol::param::Value::Tag::integer_value)) {
		accepted = _reader.integer(response.value.to<uavcan::protocol::param::Value::Tag::integer_value>());

	} else if (response.value.is(uavcan::protocol::param::Value::Tag::string_value)) {
		const auto &value = response.value.to<uavcan::protocol::param::Value::Tag::string_value>();
		accepted = _reader.string(reinterpret_cast<const uint8_t *>(value.c_str()), value.size());
	}

	if (!accepted) {
		fail(_reader.result() == rid_identity_s::RESULT_OK ? rid_identity_s::RESULT_INVALID : _reader.result(), "RID snapshot invalid or changed");
		return;
	}

	if (_reader.done()) {
		const auto &id = _reader.identity();

		for (unsigned i = 0; i < sizeof(_uid); ++i) {
			if (((rid::IdentityReader::hex(id.rid_uid[2 * i]) << 4) | rid::IdentityReader::hex(id.rid_uid[2 * i + 1])) != _uid[i]) {
				fail(rid_identity_s::RESULT_INVALID, "RID UID differs from NodeInfo"); return;
			}
		}

		const bool changed = _cached.rid_boot_id != 0 && (_cached.rid_boot_id != id.rid_boot_id
				     || _cached.identity_revision != id.identity_revision || _cached.activation_state != id.activation_state
				     || memcmp(_cached.identity_hash, id.identity_hash, sizeof(id.identity_hash)) != 0
				     || memcmp(_cached.rid_uid, id.rid_uid, sizeof(id.rid_uid)) != 0);
		_cached = id;
		_last_success = hrt_absolute_time();

		if (changed && _published_valid) {
			publish(rid_identity_s::RESULT_READING, "RID identity changed; renew session", true);
			_request = {}; // Wait for QGC's new session; never revive the old binding.

		} else {
			publish(rid_identity_s::RESULT_OK, "");
		}

		_active = false;
		_attempts = 0;
		_next_read = hrt_absolute_time() + 2_s;
	}
}
