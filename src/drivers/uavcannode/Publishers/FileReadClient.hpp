/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file FileReadClient.hpp
 *
 * Generic DroneCAN (UAVCAN v0) file-read client, bridged to uORB.
 *
 * Subscribes the DronecanFileReadRequest topic and, for each request, issues the
 * matching file.GetInfo / file.Read service call to the flight controller's file
 * server. The service response callback (which fires on the uavcan work-queue
 * thread inside Node::spinOnce(), so it never races BroadcastAnyUpdates()) is
 * copied into a DronecanFileReadResponse and published back to the requesting
 * driver. No "firmware" concept lives here: this is a plain file proxy, reusable
 * for any cannode-side pull (firmware image, config, calibration).
 *
 * Security note (inherited): the UAVCAN v0 file server is unauthenticated. Any
 * node can read any file the server exposes; this client does not change that.
 */

#pragma once

#include <cstring>

#include "UavcanPublisherBase.hpp"

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/file/GetInfo.hpp>
#include <uavcan/protocol/file/Read.hpp>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/dronecan_file_read_request.h>
#include <uORB/topics/dronecan_file_read_response.h>

namespace uavcannode
{

class FileReadClient :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem
{
public:
	FileReadClient(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::protocol::file::Read::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(dronecan_file_read_request)),
		_getinfo_client(node),
		_read_client(node)
	{}

	void PrintInfo() override
	{
		printf("\t%s -> file.GetInfo/file.Read service client\n",
		       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name);
	}

	void BroadcastAnyUpdates() override
	{
		if (!_clients_inited && !initClients()) {
			return;
		}

		// One request in flight by construction (the driver is the session
		// master and waits for each response), so handling one per cycle is
		// sufficient.
		dronecan_file_read_request_s req;

		if (uORB::SubscriptionCallbackWorkItem::update(&req)) {
			handleRequest(req);
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}

private:
	using GetInfoService  = uavcan::protocol::file::GetInfo;
	using GetInfoResult   = uavcan::ServiceCallResult<GetInfoService>;
	using GetInfoCallback = uavcan::MethodBinder<FileReadClient *, void (FileReadClient::*)(const GetInfoResult &)>;
	using GetInfoClient   = uavcan::ServiceClient<GetInfoService, GetInfoCallback>;

	using ReadService  = uavcan::protocol::file::Read;
	using ReadResult   = uavcan::ServiceCallResult<ReadService>;
	using ReadCallback = uavcan::MethodBinder<FileReadClient *, void (FileReadClient::*)(const ReadResult &)>;
	using ReadClient   = uavcan::ServiceClient<ReadService, ReadCallback>;

	static constexpr int kRequestTimeoutMs = 500;

	bool initClients()
	{
		if (_getinfo_client.init() < 0) {
			return false;
		}

		if (_read_client.init() < 0) {
			return false;
		}

		_getinfo_client.setCallback(GetInfoCallback(this, &FileReadClient::cb_getinfo));
		_getinfo_client.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(kRequestTimeoutMs));
		_read_client.setCallback(ReadCallback(this, &FileReadClient::cb_read));
		_read_client.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(kRequestTimeoutMs));

		// Advertise up front so a requesting driver's blocking subscription can
		// register its wake callback before the first response is published.
		_response_pub.advertise();

		_clients_inited = true;
		return true;
	}

	void handleRequest(const dronecan_file_read_request_s &req)
	{
		// Latch the request identity so the async response callback can echo it.
		_session_id = req.session_id;
		_cmd        = req.cmd;
		_offset     = req.offset;

		if (req.cmd == dronecan_file_read_request_s::CMD_ABORT) {
			// Nothing is staged at this layer; the driver-side helper owns abort.
			return;
		}

		// O4: server node-id discovery is not implemented yet; the driver must
		// supply a valid id.
		if (req.server_node_id == 0 || req.server_node_id > 127) {
			publishError(-1);
			return;
		}

		if (req.cmd == dronecan_file_read_request_s::CMD_GETINFO) {
			GetInfoService::Request gi{};
			gi.path.path = req.path;

			if (_getinfo_client.call(req.server_node_id, gi) < 0) {
				publishError(-1);
			}

		} else if (req.cmd == dronecan_file_read_request_s::CMD_READ) {
			ReadService::Request rd{};
			rd.offset    = req.offset;
			rd.path.path = req.path;

			if (_read_client.call(req.server_node_id, rd) < 0) {
				publishError(-1);
			}
		}
	}

	void cb_getinfo(const GetInfoResult &result)
	{
		dronecan_file_read_response_s resp{};
		resp.session_id = _session_id;
		resp.cmd        = _cmd;

		if (!result.isSuccessful()) {
			resp.error = -1;

		} else {
			const auto &r = result.getResponse();
			resp.error     = r.error.value;
			resp.file_size = r.size;
		}

		publish(resp);
	}

	void cb_read(const ReadResult &result)
	{
		dronecan_file_read_response_s resp{};
		resp.session_id = _session_id;
		resp.cmd        = _cmd;
		resp.offset     = _offset;

		if (!result.isSuccessful()) {
			resp.error = -1;

		} else {
			const auto &r = result.getResponse();
			resp.error = r.error.value;

			if (resp.error == 0) {
				resp.len = r.data.size();

				if (resp.len > 0) {
					memcpy(resp.data, r.data.begin(), resp.len);
				}

				// A short read (< full payload) marks end-of-file.
				resp.eof = (resp.len < sizeof(resp.data)) ? 1 : 0;
			}
		}

		publish(resp);
	}

	void publishError(int16_t error)
	{
		dronecan_file_read_response_s resp{};
		resp.session_id = _session_id;
		resp.cmd        = _cmd;
		resp.offset     = _offset;
		resp.error      = error;
		publish(resp);
	}

	void publish(dronecan_file_read_response_s &resp)
	{
		resp.timestamp = hrt_absolute_time();
		_response_pub.publish(resp);
	}

	GetInfoClient _getinfo_client;
	ReadClient    _read_client;
	bool          _clients_inited{false};

	// Identity of the in-flight request, echoed back in the response.
	uint32_t _session_id{0};
	uint32_t _offset{0};
	uint8_t  _cmd{0};

	uORB::Publication<dronecan_file_read_response_s> _response_pub{ORB_ID(dronecan_file_read_response)};
};

} // namespace uavcannode
