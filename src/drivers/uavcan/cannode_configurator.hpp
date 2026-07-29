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
 * @file cannode_configurator.hpp
 *
 * Applies airframe-style parameter defaults to UAVCAN peripheral nodes
 * by reading a config file from ROMFS when a node is discovered.
 */

#pragma once

#include <uavcan/protocol/node_info_retriever.hpp>
#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/param/ExecuteOpcode.hpp>

class CanNodeConfigurator : private uavcan::INodeInfoListener
{
public:
	CanNodeConfigurator(uavcan::INode &node, uavcan::NodeInfoRetriever &retriever);
	~CanNodeConfigurator();

	/**
	 * Drive the configuration state machine.
	 * Must be called every Run() cycle, after node.spinOnce().
	 */
	void update();

private:
	// --- INodeInfoListener ---
	void handleNodeInfoRetrieved(uavcan::NodeID node_id,
				     const uavcan::protocol::GetNodeInfo_::Response &node_info) override;
	void handleNodeInfoUnavailable(uavcan::NodeID node_id) override;

	// --- ServiceClient callbacks ---
	void cbGetSet(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result);
	void cbOpcode(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &result);

	// --- Internal helpers ---
	void startNextNode();
	bool readNextLine();        // fills _param_name/_param_type/_param_value; returns false on EOF
	void sendSet();
	void sendSave();
	void abortCurrent(bool requeue);

	// --- State ---
	enum class State : uint8_t { IDLE, SETTING, SAVING };

	// Path/queue sizing constants (defined first so all members can use them)
	static constexpr size_t  MaxNodeNameLen = uavcan::protocol::GetNodeInfo_::Response::FieldTypes::name::MaxSize;
	static constexpr size_t  MaxPathLen     = 40 + MaxNodeNameLen; // dir prefix + name + NUL
	static constexpr size_t  MaxParamName   = 93; // UAVCAN max param name + NUL
	static constexpr size_t  MaxParamValue  = 32;
	static constexpr uint8_t QueueSize      = 16;

	State _state{State::IDLE};

	uavcan::NodeID _current_node{};
	FILE          *_config_fp{nullptr};
	char           _current_path[MaxPathLen]{};

	// Callback results (set by cbGetSet / cbOpcode, consumed by update())
	bool _cb_fired{false};
	bool _cb_success{false};

	// Current param being processed
	char                              _param_name[MaxParamName]{};
	uavcan::protocol::param::Value    _param_value_typed{};

	// Pending-node queue (nodes waiting to be configured)
	struct QueueEntry {
		uint8_t node_id;
		char    path[MaxPathLen];
	};
	QueueEntry _queue[QueueSize]{};
	uint8_t    _queue_head{0};
	uint8_t    _queue_tail{0};
	uint8_t    _queue_len{0};

	void queuePush(uint8_t node_id, const char *path);

	// --- ServiceClients (members — must outlive their callbacks) ---
	using GetSetCb = uavcan::MethodBinder<CanNodeConfigurator *,
		void (CanNodeConfigurator::*)(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &)>;
	using OpcodeCb = uavcan::MethodBinder<CanNodeConfigurator *,
		void (CanNodeConfigurator::*)(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &)>;

	uavcan::ServiceClient<uavcan::protocol::param::GetSet,       GetSetCb> _getset_client;
	uavcan::ServiceClient<uavcan::protocol::param::ExecuteOpcode, OpcodeCb> _save_client;

	uavcan::NodeInfoRetriever &_retriever;
};
