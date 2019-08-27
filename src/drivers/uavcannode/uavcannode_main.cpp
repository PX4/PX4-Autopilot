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

#include <px4_platform_common/config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#
#else
#include <px4_platform_common/workqueue.h>
#endif

#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <lib/mixer/mixer.h>
#include <version/version.h>
__BEGIN_DECLS
#include <nuttx/board.h>
#include <arch/chip/chip.h>
__END_DECLS

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>

#include "uavcannode_main.hpp"
#include "indication_controller.hpp"
#include "sim_controller.hpp"
#include "resources.hpp"
#include "led.hpp"

#include "boot_app_shared.h"

/**
 * @file uavcan_main.cpp
 *
 * Implements basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 *         David Sidrane <david_s5@nscdg.com>
 */

#define RESOURCE_DEBUG
#if defined(RESOURCE_DEBUG)
#define resources(s) ::syslog(LOG_INFO," %s\n",(s)); \
	if (UavcanNode::instance()) { \
		syslog(LOG_INFO,"UAVCAN  getPeakNumUsedBlocks() in bytes %d\n", \
		       UAVCAN_MEM_POOL_BLOCK_SIZE * UavcanNode::instance()->get_node().getAllocator().getPeakNumUsedBlocks()); \
	} \
	free_check(); \
	stack_check();
#else
#define resources(s)
#endif

/*
 * This is the AppImageDescriptor used
 * by the make_can_boot_descriptor.py tool to set
 * the application image's descriptor so that the
 * uavcan bootloader has the ability to validate the
 * image crc, size etc of this application
*/

boot_app_shared_section app_descriptor_t AppDescriptor = {
	.signature = {APP_DESCRIPTOR_SIGNATURE},
	.image_crc = 0,
	.image_size = 0,
	.vcs_commit = 0,
	.major_version = APP_VERSION_MAJOR,
	.minor_version = APP_VERSION_MINOR,
	.reserved = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
};

/*
 * UavcanNode
 */
UavcanNode *UavcanNode::_instance;

UavcanNode::UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock) :
	CDev(UAVCAN_DEVICE_PATH),
	active_bitrate(0),
	_node(can_driver, system_clock),
	_node_mutex(),
	_time_sync_slave(_node),
	_fw_update_listner(_node),
	_reset_timer(_node)
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
			::usleep(5000);

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


	resources("Before _instance->init:");
	const int node_init_res = _instance->init(node_id);
	resources("After _instance->init:");

	if (node_init_res < 0) {
		delete _instance;
		_instance = nullptr;
		warnx("Node init failed %i", node_init_res);
		return node_init_res;
	}


	/* Keep the bit rate for reboots on BenginFirmware updates */

	_instance->active_bitrate = bitrate;

	/*
	 * Start the task. Normally it should never exit.
	 */
	static auto run_trampoline = [](int, char *[]) {return UavcanNode::_instance->run();};
	_instance->_task = px4_task_spawn_cmd("uavcan", SCHED_DEFAULT, SCHED_PRIORITY_ACTUATOR_OUTPUTS, StackSize,
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

	// Extracting the first 8 hex digits of the git hash and converting them to int
	char fw_git_short[9] = {};
	std::memmove(fw_git_short, px4_firmware_version_string(), 8);
	char *end = nullptr;
	swver.vcs_commit = std::strtol(fw_git_short, &end, 16);
	swver.optional_field_flags |= swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
	swver.major = AppDescriptor.major_version;
	swver.minor = AppDescriptor.minor_version;
	swver.image_crc = AppDescriptor.image_crc;

	warnx("SW version vcs_commit: 0x%08x", unsigned(swver.vcs_commit));

	_node.setSoftwareVersion(swver);

	/* hardware version */
	uavcan::protocol::HardwareVersion hwver;

	hwver.major = HW_VERSION_MAJOR;
	hwver.minor = HW_VERSION_MINOR;

	mfguid_t mfgid = {};
	board_get_mfguid(mfgid);
	uavcan::copy(mfgid, mfgid + sizeof(mfgid), hwver.unique_id.begin());

	_node.setHardwareVersion(hwver);
}

static void cb_reboot(const uavcan::TimerEvent &)
{
	px4_systemreset(false);

}

void UavcanNode::cb_beginfirmware_update(const uavcan::ReceivedDataStructure<UavcanNode::BeginFirmwareUpdate::Request>
		&req,
		uavcan::ServiceResponseDataStructure<UavcanNode::BeginFirmwareUpdate::Response> &rsp)
{
	static bool inprogress = false;

	rsp.error = rsp.ERROR_UNKNOWN;

	if (req.image_file_remote_path.path.size()) {
		rsp.error = rsp.ERROR_IN_PROGRESS;

		if (!inprogress) {
			inprogress = true;
			bootloader_app_shared_t shared;
			shared.bus_speed = active_bitrate;
			shared.node_id = _node.getNodeID().get();
			bootloader_app_shared_write(&shared, App);
			rgb_led(255, 128, 0, 5);
			_reset_timer.setCallback(cb_reboot);
			_reset_timer.startOneShotWithDelay(uavcan::MonotonicDuration::fromMSec(1000));
			rsp.error = rsp.ERROR_OK;
		}
	}
}

int UavcanNode::init(uavcan::NodeID node_id)
{
	int ret = -1;

	// Do regular cdev init
	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	_node.setName(HW_UAVCAN_NAME);

	_node.setNodeID(node_id);

	fill_node_info();

	const int srv_start_res = _fw_update_listner.start(BeginFirmwareUpdateCallBack(this,
				  &UavcanNode::cb_beginfirmware_update));

	if (srv_start_res < 0) {
		return ret;
	}

	return _node.start();
}


/*
 * Restart handler
 */
class RestartRequestHandler: public uavcan::IRestartRequestHandler
{
	bool handleRestartRequest(uavcan::NodeID request_source) override
	{
		::syslog(LOG_INFO, "UAVCAN: Restarting by request from %i\n", int(request_source.get()));
		::usleep(20 * 1000 * 1000);
		px4_systemreset(false);
		return true; // Will never be executed BTW
	}
} restart_request_handler;

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

	get_node().setRestartRequestHandler(&restart_request_handler);

	while (init_indication_controller(get_node()) < 0) {
		::syslog(LOG_INFO, "UAVCAN: Indication controller init failed\n");
		::sleep(1);
	}

	while (init_sim_controller(get_node()) < 0) {
		::syslog(LOG_INFO, "UAVCAN: sim controller init failed\n");
		::sleep(1);
	}

	(void)pthread_mutex_lock(&_node_mutex);

	/*
	 * Set up the time synchronization
	 */

	const int slave_init_res = _time_sync_slave.start();

	if (slave_init_res < 0) {
		warnx("Failed to start time_sync_slave");
		_task_should_exit = true;
	}

	const unsigned PollTimeoutMs = 50;

	const int busevent_fd = ::open(uavcan_stm32::BusEvent::DevName, 0);

	if (busevent_fd < 0) {
		warnx("Failed to open %s", uavcan_stm32::BusEvent::DevName);
		_task_should_exit = true;
	}

	/* If we had an  RTC we would call uavcan_stm32::clock::setUtc()
	* but for now we use adjustUtc with a correction of 0
	*/
//        uavcan_stm32::clock::adjustUtc(uavcan::UtcDuration::fromUSec(0));

	_node.setModeOperational();

	/*
	 * This event is needed to wake up the thread on CAN bus activity (RX/TX/Error).
	 * Please note that with such multiplexing it is no longer possible to rely only on
	 * the value returned from poll() to detect whether actuator control has timed out or not.
	 * Instead, all ORB events need to be checked individually (see below).
	 */
	add_poll_fd(busevent_fd);

	uint32_t start_tick = clock_systimer();

	while (!_task_should_exit) {
		// Mutex is unlocked while the thread is blocked on IO multiplexing
		(void)pthread_mutex_unlock(&_node_mutex);

		const int poll_ret = ::poll(_poll_fds, _poll_fds_num, PollTimeoutMs);


		(void)pthread_mutex_lock(&_node_mutex);

		node_spin_once();  // Non-blocking


		// this would be bad...
		if (poll_ret < 0) {
			PX4_ERR("poll error %d", errno);
			continue;

		} else {
			// Do Something
		}

		if (clock_systimer() - start_tick > TICK_PER_SEC) {
			start_tick = clock_systimer();
			resources("Udate:");
			/*
			 *  Printing the slave status information once a second
			 */
			const bool active = _time_sync_slave.isActive();                      // Whether it can sync with a remote master

			const int master_node_id = _time_sync_slave.getMasterNodeID().get();  // Returns an invalid Node ID if (active == false)

			const long msec_since_last_adjustment = (_node.getMonotonicTime() - _time_sync_slave.getLastAdjustmentTime()).toMSec();
			const uavcan::UtcTime utc = uavcan_stm32::clock::getUtc();
			syslog(LOG_INFO, "Time:%lld\n"
			       "            Time sync slave status:\n"
			       "    Active: %d\n"
			       "    Master Node ID: %d\n"
			       "    Last clock adjustment was %ld ms ago\n",
			       utc .toUSec(), int(active), master_node_id, msec_since_last_adjustment);
			syslog(LOG_INFO, "UTC %lu sec   Rate corr: %fPPM   Jumps: %lu   Locked: %i\n\n",
			       static_cast<unsigned long>(utc.toMSec() / 1000),
			       static_cast<double>(uavcan_stm32::clock::getUtcRateCorrectionPPM()),
			       uavcan_stm32::clock::getUtcJumpCount(),
			       int(uavcan_stm32::clock::isUtcLocked()));
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
	      "\tuavcannode {start|status|stop|arm|disarm}");
}

extern "C" __EXPORT int uavcannode_start(int argc, char *argv[]);

int uavcannode_start(int argc, char *argv[])
{
	resources("Before board_app_initialize");

	board_app_initialize(NULL);

	resources("After board_app_initialize");

	// CAN bitrate
	int32_t bitrate = 0;
	// Node ID
	int32_t node_id = 0;

	// Did the bootloader auto baud and get a node ID Allocated

	bootloader_app_shared_t shared;
	int valid  = bootloader_app_shared_read(&shared, BootLoader);

	if (valid == 0) {

		bitrate = shared.bus_speed;
		node_id = shared.node_id;

		// Invalidate to prevent deja vu

		bootloader_app_shared_invalidate();

	} else {

		// Node ID
		(void)param_get(param_find("CANNODE_NODE_ID"), &node_id);
		(void)param_get(param_find("CANNODE_BITRATE"), &bitrate);
	}

	if (node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast()) {
		warnx("Invalid Node ID %i", node_id);
		::exit(1);
	}

	// Start
	warnx("Node ID %u, bitrate %u", node_id, bitrate);
	int rv = UavcanNode::start(node_id, bitrate);
	resources("After UavcanNode::start");
	::sleep(1);
	return rv;
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

		return uavcannode_start(argc, argv);
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
