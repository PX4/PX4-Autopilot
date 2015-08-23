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

#include <nuttx/config.h>

#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <pthread.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/board_serial.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/git_version.h>
#include <version/version.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>

#include "uavcan_main.hpp"
#include "uavcan_servers.hpp"
#include "uavcan_virtual_can_driver.hpp"

# include <uavcan_posix/dynamic_node_id_server/file_event_tracer.hpp>
# include <uavcan_posix/dynamic_node_id_server/file_storage_backend.hpp>
# include <uavcan_posix/firmware_version_checker.hpp>

//todo:The Inclusion of file_server_backend is killing
// #include <sys/types.h> and leaving OK undefined
# define OK 0

/**
 * @file uavcan_servers.cpp
 *
 * Implements basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 *         David Sidrane <david_s5@nscdg.com>
 */

/*
 * UavcanNode
 */
UavcanServers *UavcanServers::_instance;
UavcanServers::UavcanServers(uavcan::INode &main_node) :
	_subnode_thread(-1),
	_vdriver(UAVCAN_STM32_NUM_IFACES, uavcan_stm32::SystemClock::instance()),
	_subnode(_vdriver, uavcan_stm32::SystemClock::instance()),
	_main_node(main_node),
	_tracer(),
	_storage_backend(),
	_fw_version_checker(),
	_server_instance(_subnode, _storage_backend, _tracer),
	_fileserver_backend(_subnode),
	_node_info_retriever(_subnode),
	_fw_upgrade_trigger(_subnode, _fw_version_checker),
	_fw_server(_subnode, _fileserver_backend),
	_mutex_inited(false),
	_check_fw(false)

{
}

UavcanServers::~UavcanServers()
{
	if (_mutex_inited) {
		(void)Lock::deinit(_subnode_mutex);
	}
	_main_node.getDispatcher().removeRxFrameListener();
}

int UavcanServers::stop(void)
{
	UavcanServers *server = instance();

	if (server == nullptr) {
		warnx("Already stopped");
		return -1;
	}

	_instance = nullptr;

	if (server->_subnode_thread != -1) {
		pthread_cancel(server->_subnode_thread);
		pthread_join(server->_subnode_thread, NULL);
	}

	server->_main_node.getDispatcher().removeRxFrameListener();

	delete server;
	return 0;
}

int UavcanServers::start(unsigned num_ifaces, uavcan::INode &main_node)
{
	if (_instance != nullptr) {
		warnx("Already started");
		return -1;
	}

	/*
	 * Node init
	 */
	_instance = new UavcanServers(main_node);

	if (_instance == nullptr) {
		warnx("Out of memory");
		return -2;
	}

	int rv  = _instance->init(num_ifaces);

	if (rv < 0) {
		warnx("Node init failed: %d", rv);
		delete _instance;
		_instance = nullptr;
		return rv;
	}

	/*
	 * Start the thread. Normally it should never exit.
	 */
	pthread_attr_t tattr;
	struct sched_param param;

	pthread_attr_init(&tattr);
	tattr.stacksize = StackSize;
	param.sched_priority = Priority;
	pthread_attr_setschedparam(&tattr, &param);

	static auto run_trampoline = [](void *) {return UavcanServers::_instance->run(_instance);};

	rv = pthread_create(&_instance->_subnode_thread, &tattr, static_cast<pthread_startroutine_t>(run_trampoline), NULL);

	if (rv < 0) {
		warnx("pthread_create() failed: %d", errno);
		rv =  -errno;
		delete _instance;
		_instance = nullptr;
	}

	return rv;
}

int UavcanServers::init(unsigned num_ifaces)
{
	errno = 0;

	/*
	 * Initialize the mutex.
	 * giving it its path
	 */

	int ret = Lock::init(_subnode_mutex);

	if (ret < 0) {
		warnx("Lock init: %d", errno);
		return ret;
	}

	_mutex_inited = true;

	_subnode.setNodeID(_main_node.getNodeID());
	_main_node.getDispatcher().installRxFrameListener(&_vdriver);


	/*
	 * Initialize the fw version checker.
	 * giving it its path
	 */
	ret = _fw_version_checker.createFwPaths(UAVCAN_FIRMWARE_PATH);

	if (ret < 0) {
		warnx("FirmwareVersionChecker init: %d, errno: %d", ret, errno);
		return ret;
	}

	/* Start fw file server back */

	ret = _fw_server.start();

	if (ret < 0) {
		warnx("BasicFileServer init: %d, errno: %d", ret, errno);
		return ret;
	}

	/* Initialize storage back end for the node allocator using UAVCAN_NODE_DB_PATH directory */

	ret = _storage_backend.init(UAVCAN_NODE_DB_PATH);

	if (ret < 0) {
		warnx("FileStorageBackend init: %d, errno: %d", ret, errno);
		return ret;
	}

	/* Initialize trace in the UAVCAN_NODE_DB_PATH directory */

	ret = _tracer.init(UAVCAN_LOG_FILE);

	if (ret < 0) {
		warnx("FileEventTracer init: %d, errno: %d", ret, errno);
		return ret;
	}

	/* hardware version */
	uavcan::protocol::HardwareVersion hwver;
	UavcanNode::getHardwareVersion(hwver);

	/* Initialize the dynamic node id server  */
	ret = _server_instance.init(hwver.unique_id);

	if (ret < 0) {
		warnx("CentralizedServer init: %d", ret);
		return ret;
	}

	/* Start node info retriever to fetch node info from new nodes */

	ret = _node_info_retriever.start();

	if (ret < 0) {
		warnx("NodeInfoRetriever init: %d", ret);
		return ret;
	}

	/* Start the fw version checker   */

	ret = _fw_upgrade_trigger.start(_node_info_retriever, _fw_version_checker.getFirmwarePath());

	if (ret < 0) {
		warnx("FirmwareUpdateTrigger init: %d", ret);
		return ret;
	}

	/*  Start the Node   */

	return OK;
}
__attribute__((optimize("-O0")))
pthread_addr_t UavcanServers::run(pthread_addr_t)
{
	Lock lock(_subnode_mutex);

	while (1) {

		if (_check_fw == true) {
			_check_fw = false;
			_node_info_retriever.invalidateAll();
		}

		const int spin_res = _subnode.spin(uavcan::MonotonicDuration::fromMSec(100));

		if (spin_res < 0) {
			warnx("node spin error %i", spin_res);
		}
	}

	warnx("exiting.");
	return (pthread_addr_t) 0;
}

