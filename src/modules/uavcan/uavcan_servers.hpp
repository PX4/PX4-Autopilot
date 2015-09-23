/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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

#include <nuttx/config.h>

#include <uavcan_stm32/uavcan_stm32.hpp>
#include <drivers/device/device.h>
#include <systemlib/perf_counter.h>

#include <uavcan/node/sub_node.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>

# include <uavcan/protocol/dynamic_node_id_server/centralized.hpp>
# include <uavcan/protocol/node_info_retriever.hpp>
# include <uavcan_posix/basic_file_server_backend.hpp>
# include <uavcan/protocol/firmware_update_trigger.hpp>
# include <uavcan/protocol/file_server.hpp>
# include <uavcan_posix/dynamic_node_id_server/file_event_tracer.hpp>
# include <uavcan_posix/dynamic_node_id_server/file_storage_backend.hpp>
# include <uavcan_posix/firmware_version_checker.hpp>

# include "uavcan_virtual_can_driver.hpp"

/**
 * @file uavcan_servers.hpp
 *
 * Defines basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author David Sidrane <david_s5@nscdg.com>
 */

#define UAVCAN_DEVICE_PATH	"/dev/uavcan/esc"
#define UAVCAN_NODE_DB_PATH     "/fs/microsd/uavcan.db"
#define UAVCAN_FIRMWARE_PATH    "/fs/microsd/fw"
#define UAVCAN_LOG_FILE         UAVCAN_NODE_DB_PATH"/trace.log"

/**
 * A UAVCAN Server Sub node.
 */
class UavcanServers
{
	static constexpr unsigned NumIfaces = 1;  // UAVCAN_STM32_NUM_IFACES

	static constexpr unsigned MemPoolSize = 64 * uavcan::MemPoolBlockSize;

	static constexpr unsigned MaxCanFramesPerTransfer   =  63;

	/**
	 * This number is based on the worst case max number of frames per interface. With
	 * MemPoolBlockSize set at 48 this is 6048 Bytes.
	 *
	 * The servers can be forced to use the primary interface only, this can be achieved simply by passing
	 * 1 instead of UAVCAN_STM32_NUM_IFACES into the constructor of the virtual CAN driver.
	 */
	static constexpr unsigned QueuePoolSize =
		(NumIfaces * uavcan::MemPoolBlockSize * MaxCanFramesPerTransfer);

	static constexpr unsigned StackSize  = 3500;
	static constexpr unsigned Priority  =  120;

	typedef uavcan::SubNode<MemPoolSize> SubNode;

public:
	UavcanServers(uavcan::INode &main_node);

	virtual		~UavcanServers();

	static int      start(uavcan::INode &main_node);
	static int      stop();

	SubNode         &get_node() { return _subnode; }

	static UavcanServers *instance() { return _instance; }

	/*
	 *  Set the main node's pointer to to the injector
	 *  This is a work around as main_node.getDispatcher().remeveRxFrameListener();
	 *  would require a dynamic cast and rtti is not enabled.
	 */
	void attachITxQueueInjector(ITxQueueInjector **injector) {*injector = &_vdriver;}

	void requestCheckAllNodesFirmwareAndUpdate() { _check_fw = true; }

private:
	pthread_t         _subnode_thread;
	pthread_mutex_t   _subnode_mutex;

	int		init();

	pthread_addr_t	run(pthread_addr_t);

	static UavcanServers	*_instance;            ///< singleton pointer

	typedef VirtualCanDriver<QueuePoolSize> vCanDriver;

	vCanDriver    _vdriver;

	uavcan::SubNode<MemPoolSize>  _subnode;   ///< library instance
	uavcan::INode                &_main_node; ///< library instance

	uavcan_posix::dynamic_node_id_server::FileEventTracer _tracer;
	uavcan_posix::dynamic_node_id_server::FileStorageBackend _storage_backend;
	uavcan_posix::FirmwareVersionChecker _fw_version_checker;
	uavcan::dynamic_node_id_server::CentralizedServer _server_instance;  ///< server singleton pointer
	uavcan_posix::BasicFileSeverBackend  _fileserver_backend;
	uavcan::NodeInfoRetriever   _node_info_retriever;
	uavcan::FirmwareUpdateTrigger   _fw_upgrade_trigger;
	uavcan::BasicFileServer         _fw_server;

	bool _mutex_inited;
	volatile bool _check_fw;

};
