/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file cannode_configurator.cpp
 *
 * Applies airframe-style parameter defaults to UAVCAN peripheral nodes.
 * Config files live at /etc/init.d/cannode_airframes/<node_name> in ROMFS.
 * Format: PARAM_NAME=type:value per line; type is i(nteger), f(loat), b(oolean), s(tring).
 * Lines starting with # are comments.
 */

#include "cannode_configurator.hpp"

#include <cstdio>
#include <sys/stat.h>
#include <cstring>
#include <cstdlib>
#include <cerrno>

#include <px4_platform_common/log.h>

static constexpr char CANNODE_AIRFRAMES_DIR[] = "/etc/init.d/cannode_airframes/";

CanNodeConfigurator::CanNodeConfigurator(uavcan::INode &node, uavcan::NodeInfoRetriever &retriever)
	: _getset_client(node),
	  _save_client(node),
	  _retriever(retriever)
{
	_getset_client.setCallback(GetSetCb(this, &CanNodeConfigurator::cbGetSet));
	_save_client.setCallback(OpcodeCb(this, &CanNodeConfigurator::cbOpcode));
	_retriever.addListener(this);
}

CanNodeConfigurator::~CanNodeConfigurator()
{
	_retriever.removeListener(this);

	if (_config_fp != nullptr) {
		fclose(_config_fp);
		_config_fp = nullptr;
	}
}

void CanNodeConfigurator::handleNodeInfoRetrieved(uavcan::NodeID node_id,
		const uavcan::protocol::GetNodeInfo_::Response &node_info)
{
	// Build config file path from node name
	char path[sizeof(CANNODE_AIRFRAMES_DIR) + uavcan::protocol::GetNodeInfo_::Response::FieldTypes::name::MaxSize];
	const int n = snprintf(path, sizeof(path), "%s%s", CANNODE_AIRFRAMES_DIR, node_info.name.c_str());

	if (n <= 0 || static_cast<size_t>(n) >= sizeof(path)) {
		return;
	}

	// Only queue if a config file exists for this node type
	struct stat sb;

	if (stat(path, &sb) != 0) {
		return;
	}

	queuePush(node_id.get(), path);
}

void CanNodeConfigurator::handleNodeInfoUnavailable(uavcan::NodeID node_id)
{
	if (_state != State::IDLE && _current_node == node_id) {
		PX4_WARN("cannode_cfg: node %u went offline mid-config, requeueing", node_id.get());
		abortCurrent(true);
	}
}

void CanNodeConfigurator::queuePush(uint8_t node_id, const char *path)
{
	if (_queue_len >= QueueSize) {
		PX4_WARN("cannode_cfg: queue full, dropping node %u", node_id);
		return;
	}

	_queue[_queue_tail].node_id = node_id;
	strncpy(_queue[_queue_tail].path, path, MaxPathLen - 1);
	_queue[_queue_tail].path[MaxPathLen - 1] = '\0';
	_queue_tail = (_queue_tail + 1) % QueueSize;
	_queue_len++;
}

void CanNodeConfigurator::update()
{
	switch (_state) {

	case State::IDLE:
		if (_queue_len > 0) {
			startNextNode();
		}

		break;

	case State::SETTING:
		if (!_cb_fired) { break; } // no response yet

		_cb_fired = false;

		if (!_cb_success) { // got a response but it was an error
			PX4_ERR("cannode_cfg: SET failed for node %u, aborting", _current_node.get());
			abortCurrent(true);
			break;
		}

		if (readNextLine()) {
			sendSet();

		} else {
			sendSave();
		}

		break;

	case State::SAVING:
		if (!_cb_fired) { break; }

		_cb_fired = false;

		if (!_cb_success) {
			PX4_WARN("cannode_cfg: SAVE failed for node %u", _current_node.get());
		}

		PX4_INFO("cannode_cfg: configuration complete for node %u", _current_node.get());
		abortCurrent(false); // closes fd, resets to IDLE (no requeue)
		break;
	}
}

void CanNodeConfigurator::startNextNode()
{
	// Pop from the front of the queue (FIFO)
	const QueueEntry entry = _queue[_queue_head];
	_queue_head = (_queue_head + 1) % QueueSize;
	_queue_len--;

	_config_fp = ::fopen(entry.path, "r");

	if (_config_fp == nullptr) {
		PX4_ERR("cannode_cfg: cannot open '%s': %d", entry.path, errno);
		return; // stay IDLE, this node is dropped
	}

	_current_node = uavcan::NodeID(entry.node_id);
	memcpy(_current_path, entry.path, sizeof(_current_path));
	PX4_INFO("cannode_cfg: configuring node %u from '%s'", entry.node_id, entry.path);

	if (readNextLine()) {
		sendSet();

	} else {
		// Empty config file — nothing to do
		fclose(_config_fp);
		_config_fp = nullptr;
	}
}

void CanNodeConfigurator::sendSet()
{
	uavcan::protocol::param::GetSet::Request req;
	req.name = _param_name;
	req.value = _param_value_typed;

	const int res = _getset_client.call(_current_node, req);

	if (res < 0) {
		PX4_ERR("cannode_cfg: SET call failed (%d), aborting node %u", res, _current_node.get());
		abortCurrent(true);

	} else {
		_state = State::SETTING;
	}
}

void CanNodeConfigurator::sendSave()
{
	uavcan::protocol::param::ExecuteOpcode::Request req;
	req.opcode = uavcan::protocol::param::ExecuteOpcode::Request::OPCODE_SAVE;

	fclose(_config_fp);
	_config_fp = nullptr;

	const int res = _save_client.call(_current_node, req);

	if (res < 0) {
		PX4_ERR("cannode_cfg: SAVE call failed (%d) for node %u", res, _current_node.get());
		_state = State::IDLE;

	} else {
		_state = State::SAVING;
	}
}

void CanNodeConfigurator::abortCurrent(bool requeue)
{
	if (_config_fp != nullptr) {
		fclose(_config_fp);
		_config_fp = nullptr;
	}

	if (requeue) {
		queuePush(_current_node.get(), _current_path);
	}

	_cb_fired = false;
	_state = State::IDLE;
}

void CanNodeConfigurator::cbGetSet(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result)
{
	_cb_success = result.isSuccessful();
	_cb_fired = true;
}

void CanNodeConfigurator::cbOpcode(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &result)
{
	_cb_success = result.isSuccessful();
	_cb_fired = true;
}

bool CanNodeConfigurator::readNextLine()
{
	char line[MaxParamName + MaxParamValue + 6]; // name=t:value\n\0

	while (fgets(line, sizeof(line), _config_fp) != nullptr) {
		// Strip trailing newline/carriage-return
		line[strcspn(line, "\r\n")] = '\0';

		// Skip blank lines and comments
		if (line[0] == '\0' || line[0] == '#') { continue; }

		// Parse NAME=type:value  (type is one of: i f b s)
		char *eq = strchr(line, '=');

		if (!eq) { continue; } // malformed line, skip

		const size_t name_len = static_cast<size_t>(eq - line);

		if (name_len == 0 || name_len >= MaxParamName) { continue; }

		// Expect "type:value" after '='
		const char type_char = eq[1];

		if (eq[2] != ':') { continue; } // malformed, skip

		if (type_char != 'i' && type_char != 'f' && type_char != 'b' && type_char != 's') {
			PX4_WARN("cannode_cfg: unknown type '%c' in config, skipping line", type_char);
			continue;
		}

		const char *value_start = eq + 3;
		const size_t value_len = strlen(value_start);

		if (value_len >= MaxParamValue) { continue; }

		memcpy(_param_name, line, name_len);
		_param_name[name_len] = '\0';

		using Tag = uavcan::protocol::param::Value::Tag;

		switch (type_char) {
		case 'i':
			_param_value_typed.to<Tag::integer_value>() = static_cast<int64_t>(std::strtoll(value_start, nullptr, 10));
			break;

		case 'f':
			_param_value_typed.to<Tag::real_value>() = static_cast<float>(std::atof(value_start));
			break;

		case 'b':
			_param_value_typed.to<Tag::boolean_value>() = (value_start[0] == '1' || value_start[0] == 't') ? 1 : 0;
			break;

		case 's':
			_param_value_typed.to<Tag::string_value>() = value_start;
			break;

		default:
			continue;
		}

		PX4_INFO("cannode_cfg: node %u  SET %s = %c:%s", _current_node.get(), _param_name, type_char, value_start);

		return true;
	}

	return false;
}
