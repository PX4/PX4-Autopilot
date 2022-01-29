/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>

#include <uavcan/protocol/dynamic_node_id_server/centralized.hpp>
#include <uavcan/protocol/file_server.hpp>
#include <uavcan/protocol/firmware_update_trigger.hpp>
#include <uavcan/protocol/node_info_retriever.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan_posix/basic_file_server_backend.hpp>
#include <uavcan_posix/dynamic_node_id_server/file_event_tracer.hpp>
#include <uavcan_posix/dynamic_node_id_server/file_storage_backend.hpp>
#include <uavcan_posix/firmware_version_checker.hpp>

#include "uavcan_module.hpp"

/**
 * @file uavcan_servers.hpp
 *
 * Defines basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author David Sidrane <david_s5@nscdg.com>
 */

/**
 * A UAVCAN Server Sub node.
 */
class UavcanServers
{
public:
	UavcanServers(uavcan::INode &node, uavcan::NodeInfoRetriever &node_info_retriever);
	~UavcanServers() = default;

	int init();

	bool guessIfAllDynamicNodesAreAllocated() { return _server_instance.guessIfAllDynamicNodesAreAllocated(); }

private:

	void unpackFwFromROMFS(const char *sd_path, const char *romfs_path);
	void migrateFWFromRoot(const char *sd_path, const char *sd_root_path);
	int copyFw(const char *dst, const char *src);

	uavcan_posix::dynamic_node_id_server::FileEventTracer _tracer;
	uavcan_posix::dynamic_node_id_server::FileStorageBackend _storage_backend;
	uavcan_posix::FirmwareVersionChecker _fw_version_checker;
	uavcan::dynamic_node_id_server::CentralizedServer _server_instance;  ///< server singleton pointer
	uavcan_posix::BasicFileServerBackend _fileserver_backend;
	uavcan::FirmwareUpdateTrigger _fw_upgrade_trigger;
	uavcan::BasicFileServer _fw_server;

	uavcan::NodeInfoRetriever &_node_info_retriever;
};
