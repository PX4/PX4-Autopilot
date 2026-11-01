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
#include <containers/List.hpp>
#include <pthread.h>
#include <px4_platform_common/atomic.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/uavcan_firmware_update.h>
#include <px4_platform_common/time.h>

class CanNodeConfigurator : private uavcan::INodeInfoListener
{
public:
	explicit CanNodeConfigurator(uavcan::NodeInfoRetriever &retriever);
	~CanNodeConfigurator();

	void init();

private:
	// --- INodeInfoListener ---
	void handleNodeInfoRetrieved(uavcan::NodeID node_id,
				     const uavcan::protocol::GetNodeInfo_::Response &node_info) override;
	void handleNodeInfoUnavailable(uavcan::NodeID node_id) override {}

	// --- Thread ---
	static void *threadEntry(void *arg);
	void threadMain();
	void applyConfig(uint8_t node_id, const char *path);

	// --- Constants ---
	static constexpr size_t MaxNodeNameLen = uavcan::protocol::GetNodeInfo_::Response::FieldTypes::name::MaxSize;
	static constexpr size_t MaxPathLen     = 64 + MaxNodeNameLen;

	struct QueueEntry : public ListNode<QueueEntry *> {
		uint8_t     node_id;
		char        node_name[MaxNodeNameLen];
		hrt_abstime queued_at;
	};

	uavcan::NodeInfoRetriever &_retriever;

	uORB::Subscription _fw_update_sub{ORB_ID::uavcan_firmware_update};
	bool             _fw_update_pending{false};

	pthread_t        _thread{};
	pthread_mutex_t  _queue_mutex{};
	px4::atomic_bool _should_exit{false};

	List<QueueEntry *> _queue{};
};
