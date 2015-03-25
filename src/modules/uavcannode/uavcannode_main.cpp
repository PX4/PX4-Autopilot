/****************************************************************************
 *
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
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/board_serial.h>
#include <systemlib/scheduling_priorities.h>
#include <version/version.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>

#include "uavcannode_main.hpp"

/**
 * @file uavcan_main.cpp
 *
 * Implements basic functinality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

/*
 * UavcanNode
 */
UavcanNode *UavcanNode::_instance;

UavcanNode::UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock) :
	CDev("uavcan", UAVCAN_DEVICE_PATH),
	_node(can_driver, system_clock),
	_node_mutex()
{
	const int res = pthread_mutex_init(&_node_mutex, nullptr);
	if (res < 0) {
		std::abort();
	}

}

UavcanNode::~UavcanNode()
{
	if (_task != -1) {
		/* tell the task we want it to go away */
		_task_should_exit = true;

		unsigned i = 10;

		do {
			/* wait 5ms - it should wake every 10ms or so worst-case */
			usleep(5000);

			/* if we have given up, kill it */
			if (--i == 0) {
				task_delete(_task);
				break;
			}

		} while (_task != -1);
	}

	_instance = nullptr;

}

int UavcanNode::start(uavcan::NodeID node_id, uint32_t bitrate)
{
	if (_instance != nullptr) {
		warnx("Already started");
		return -1;
	}

	/*
	 * GPIO config.
	 * Forced pull up on CAN2 is required for Pixhawk v1 where the second interface lacks a transceiver.
	 * If no transceiver is connected, the RX pin will float, occasionally causing CAN controller to
	 * fail during initialization.
	 */
	stm32_configgpio(GPIO_CAN1_RX);
	stm32_configgpio(GPIO_CAN1_TX);
#if defined(GPIO_CAN2_RX)
	stm32_configgpio(GPIO_CAN2_RX | GPIO_PULLUP);
	stm32_configgpio(GPIO_CAN2_TX);
#endif
	/*
	 * CAN driver init
	 */
	static CanInitHelper can;
	static bool can_initialized = false;

	if (!can_initialized) {
		const int can_init_res = can.init(bitrate);

		if (can_init_res < 0) {
			warnx("CAN driver init failed %i", can_init_res);
			return can_init_res;
		}

		can_initialized = true;
	}

	/*
	 * Node init
	 */
	_instance = new UavcanNode(can.driver, uavcan_stm32::SystemClock::instance());

	if (_instance == nullptr) {
		warnx("Out of memory");
		return -1;
	}

	const int node_init_res = _instance->init(node_id);

	if (node_init_res < 0) {
		delete _instance;
		_instance = nullptr;
		warnx("Node init failed %i", node_init_res);
		return node_init_res;
	}

	/*
	 * Start the task. Normally it should never exit.
	 */
	static auto run_trampoline = [](int, char *[]) {return UavcanNode::_instance->run();};
	_instance->_task = task_spawn_cmd("uavcan", SCHED_DEFAULT, SCHED_PRIORITY_ACTUATOR_OUTPUTS, StackSize,
			      static_cast<main_t>(run_trampoline), nullptr);

	if (_instance->_task < 0) {
		warnx("start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void UavcanNode::fill_node_info()
{
	/* software version */
	uavcan::protocol::SoftwareVersion swver;

	// Extracting the first 8 hex digits of FW_GIT and converting them to int
	char fw_git_short[9] = {};
	std::memmove(fw_git_short, FW_GIT, 8);
	assert(fw_git_short[8] == '\0');
	char *end = nullptr;
	swver.vcs_commit = std::strtol(fw_git_short, &end, 16);
	swver.optional_field_mask |= swver.OPTIONAL_FIELD_MASK_VCS_COMMIT;

	warnx("SW version vcs_commit: 0x%08x", unsigned(swver.vcs_commit));

	_node.setSoftwareVersion(swver);

	/* hardware version */
	uavcan::protocol::HardwareVersion hwver;

	hwver.major = 1;

	uint8_t udid[12] = {};  // Someone seems to love magic numbers
	get_board_serial(udid);
	uavcan::copy(udid, udid + sizeof(udid), hwver.unique_id.begin());

	_node.setHardwareVersion(hwver);
}

int UavcanNode::init(uavcan::NodeID node_id)
{
	int ret = -1;

	// Do regular cdev init
	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	_node.setName("org.pixhawk.cannode");

	_node.setNodeID(node_id);

	fill_node_info();

	return _node.start();
}

void UavcanNode::node_spin_once()
{
	const int spin_res = _node.spin(uavcan::MonotonicTime());
	if (spin_res < 0) {
		warnx("node spin error %i", spin_res);
	}
}

/*
  add a fd to the list of polled events. This assumes you want
  POLLIN for now.
 */
int UavcanNode::add_poll_fd(int fd)
{
	int ret = _poll_fds_num;
	if (_poll_fds_num >= UAVCAN_NUM_POLL_FDS) {
		errx(1, "uavcan: too many poll fds, exiting");
	}
	_poll_fds[_poll_fds_num] = ::pollfd();
	_poll_fds[_poll_fds_num].fd = fd;
	_poll_fds[_poll_fds_num].events = POLLIN;
	_poll_fds_num += 1;
	return ret;
}


int UavcanNode::run()
{
	(void)pthread_mutex_lock(&_node_mutex);

	const unsigned PollTimeoutMs = 50;

	const int busevent_fd = ::open(uavcan_stm32::BusEvent::DevName, 0);
	if (busevent_fd < 0)
	{
		warnx("Failed to open %s", uavcan_stm32::BusEvent::DevName);
		_task_should_exit = true;
	}


	_node.setStatusOk();

	/*
	 * This event is needed to wake up the thread on CAN bus activity (RX/TX/Error).
	 * Please note that with such multiplexing it is no longer possible to rely only on
	 * the value returned from poll() to detect whether actuator control has timed out or not.
	 * Instead, all ORB events need to be checked individually (see below).
	 */
	add_poll_fd(busevent_fd);

	while (!_task_should_exit) {
		// Mutex is unlocked while the thread is blocked on IO multiplexing
		(void)pthread_mutex_unlock(&_node_mutex);

		const int poll_ret = ::poll(_poll_fds, _poll_fds_num, PollTimeoutMs);

		(void)pthread_mutex_lock(&_node_mutex);

		node_spin_once();  // Non-blocking


		// this would be bad...
		if (poll_ret < 0) {
			log("poll error %d", errno);
			continue;
		} else {
		    // Do Somthing
		}

	}

	teardown();
	warnx("exiting.");

	exit(0);
}

int
UavcanNode::teardown()
{
  return 0;
}



int
UavcanNode::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {



	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

void
UavcanNode::print_info()
{
	if (!_instance) {
		warnx("not running, start first");
	}

	(void)pthread_mutex_lock(&_node_mutex);


	(void)pthread_mutex_unlock(&_node_mutex);
}

/*
 * App entry point
 */
static void print_usage()
{
	warnx("usage: \n"
	      "\tuavcan {start|status|stop|arm|disarm}");
}

extern "C" __EXPORT int uavcannode_main(int argc, char *argv[]);

int uavcannode_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		::exit(1);
	}

	if (!std::strcmp(argv[1], "start")) {
		if (UavcanNode::instance()) {
			errx(1, "already started");
		}

		// Node ID
		int32_t node_id = 0;
		(void)param_get(param_find("UAVCAN_NODE_ID"), &node_id);

		if (node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast()) {
			warnx("Invalid Node ID %i", node_id);
			::exit(1);
		}

		// CAN bitrate
		int32_t bitrate = 0;
		(void)param_get(param_find("UAVCAN_BITRATE"), &bitrate);

		// Start
		warnx("Node ID %u, bitrate %u", node_id, bitrate);
		return UavcanNode::start(node_id, bitrate);
	}

	/* commands below require the app to be started */
	UavcanNode *const inst = UavcanNode::instance();

	if (!inst) {
		errx(1, "application not running");
	}

	if (!std::strcmp(argv[1], "status") || !std::strcmp(argv[1], "info")) {
		inst->print_info();
		::exit(0);
	}

	if (!std::strcmp(argv[1], "stop")) {
		delete inst;
		::exit(0);
	}

	print_usage();
	::exit(1);
}
