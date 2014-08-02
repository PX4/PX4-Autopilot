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
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/board_serial.h>
#include <version/version.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>

#include "uavcan_main.hpp"

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
	_esc_controller(_node),
	_gnss_receiver(_node)
{
	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);

	// memset(_controls, 0, sizeof(_controls));
	// memset(_poll_fds, 0, sizeof(_poll_fds));
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

	/* clean up the alternate device node */
		// unregister_driver(PWM_OUTPUT_DEVICE_PATH);

	::close(_armed_sub);

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
	stm32_configgpio(GPIO_CAN2_RX | GPIO_PULLUP);
	stm32_configgpio(GPIO_CAN2_TX);

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
	_instance->_task = task_spawn_cmd("uavcan", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, StackSize,
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

	if (!std::strncmp(HW_ARCH, "PX4FMU_V1", 9)) {
		hwver.major = 1;
	} else if (!std::strncmp(HW_ARCH, "PX4FMU_V2", 9)) {
		hwver.major = 2;
	} else {
		; // All other values of HW_ARCH resolve to zero
	}

	uint8_t udid[12] = {};  // Someone seems to love magic numbers
	get_board_serial(udid);
	uavcan::copy(udid, udid + sizeof(udid), hwver.unique_id.begin());

	_node.setHardwareVersion(hwver);
}

int UavcanNode::init(uavcan::NodeID node_id)
{
	int ret = -1;

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK)
		return ret;

	_node.setName("org.pixhawk.pixhawk");

	_node.setNodeID(node_id);

	fill_node_info();

	/* initializing the bridges UAVCAN <--> uORB */
	ret = _esc_controller.init();
	if (ret < 0)
		return ret;

	ret = _gnss_receiver.init();
	if (ret < 0)
		return ret;

	return _node.start();
}

void UavcanNode::node_spin_once()
{
	const int spin_res = _node.spin(uavcan::MonotonicTime());
	if (spin_res < 0) {
		warnx("node spin error %i", spin_res);
	}
}

int UavcanNode::run()
{
	const unsigned PollTimeoutMs = 50;

	// XXX figure out the output count
	_output_count = 2;


	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	actuator_outputs_s outputs;
	memset(&outputs, 0, sizeof(outputs));

	const int busevent_fd = ::open(uavcan_stm32::BusEvent::DevName, 0);
	if (busevent_fd < 0)
	{
		warnx("Failed to open %s", uavcan_stm32::BusEvent::DevName);
		_task_should_exit = true;
	}

	/*
	 * XXX Mixing logic/subscriptions shall be moved into UavcanEscController::update();
	 *     IO multiplexing shall be done here.
	 */

	_node.setStatusOk();

	while (!_task_should_exit) {

		if (_groups_subscribed != _groups_required) {
			subscribe();
			_groups_subscribed = _groups_required;
			/*
			 * This event is needed to wake up the thread on CAN bus activity (RX/TX/Error).
			 * Please note that with such multiplexing it is no longer possible to rely only on
			 * the value returned from poll() to detect whether actuator control has timed out or not.
			 * Instead, all ORB events need to be checked individually (see below).
			 */
			_poll_fds[_poll_fds_num] = ::pollfd();
			_poll_fds[_poll_fds_num].fd = busevent_fd;
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num += 1;
		}

		const int poll_ret = ::poll(_poll_fds, _poll_fds_num, PollTimeoutMs);

		node_spin_once();  // Non-blocking

		// this would be bad...
		if (poll_ret < 0) {
			log("poll error %d", errno);
			continue;
		} else {
			// get controls for required topics
			bool controls_updated = false;
			unsigned poll_id = 0;
			for (unsigned i = 0; i < NUM_ACTUATOR_CONTROL_GROUPS; i++) {
				if (_control_subs[i] > 0) {
					if (_poll_fds[poll_id].revents & POLLIN) {
						controls_updated = true;
						orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);
					}
					poll_id++;
				}
			}

			if (!controls_updated) {
				// timeout: no control data, switch to failsafe values
				// XXX trigger failsafe
			}

			//can we mix?
			if (controls_updated && (_mixers != nullptr)) {

				// XXX one output group has 8 outputs max,
				// but this driver could well serve multiple groups.
				unsigned num_outputs_max = 8;

				// Do mixing
				outputs.noutputs = _mixers->mix(&outputs.output[0], num_outputs_max);
				outputs.timestamp = hrt_absolute_time();

				// iterate actuators
				for (unsigned i = 0; i < outputs.noutputs; i++) {
					// last resort: catch NaN, INF and out-of-band errors
					if (!isfinite(outputs.output[i])) {
						/*
						 * Value is NaN, INF or out of band - set to the minimum value.
						 * This will be clearly visible on the servo status and will limit the risk of accidentally
						 * spinning motors. It would be deadly in flight.
						 */
						outputs.output[i] = -1.0f;
					}

					// limit outputs to valid range

					// never go below min
					if (outputs.output[i] < -1.0f) {
						outputs.output[i] = -1.0f;
					}

					// never go below max
					if (outputs.output[i] > 1.0f) {
						outputs.output[i] = 1.0f;
					}
				}

				// Output to the bus
				_esc_controller.update_outputs(outputs.output, outputs.noutputs);
			}

		}

		// Check arming state
		bool updated = false;
		orb_check(_armed_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

			// Update the armed status and check that we're not locked down
			bool set_armed = _armed.armed && !_armed.lockdown;

			arm_actuators(set_armed);
		}
	}

	teardown();
	warnx("exiting.");

	exit(0);
}

int
UavcanNode::control_callback(uintptr_t handle,
			 uint8_t control_group,
			 uint8_t control_index,
			 float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];
	return 0;
}

int
UavcanNode::teardown()
{
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] > 0) {
			::close(_control_subs[i]);
			_control_subs[i] = -1;
		}
	}
	return ::close(_armed_sub);
}

int
UavcanNode::arm_actuators(bool arm)
{
	_is_armed = arm;
	_esc_controller.arm_esc(arm);
	return OK;
}

void
UavcanNode::subscribe()
{
	// Subscribe/unsubscribe to required actuator control groups
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	_poll_fds_num = 0;
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			warnx("subscribe to actuator_controls_%d", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}
		if (unsub_groups & (1 << i)) {
			warnx("unsubscribe from actuator_controls_%d", i);
			::close(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] > 0) {
			_poll_fds[_poll_fds_num].fd = _control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}
	}
}

int
UavcanNode::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		arm_actuators(true);
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
		// these are no-ops, as no safety switch
		break;

	case PWM_SERVO_DISARM:
		arm_actuators(false);
		break;

	case MIXERIOCGETOUTPUTCOUNT:
		*(unsigned *)arg = _output_count;
		break;

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
			_groups_required = 0;
		}

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == nullptr)
				_mixers = new MixerGroup(control_callback, (uintptr_t)_controls);

			if (_mixers == nullptr) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					warnx("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					_groups_required = 0;
					ret = -EINVAL;
				} else {

					_mixers->groups_required(_groups_required);
				}
			}

			break;
		}

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

	warnx("groups: sub: %u / req: %u / fds: %u", (unsigned)_groups_subscribed, (unsigned)_groups_required, _poll_fds_num);
	warnx("mixer: %s", (_mixers == nullptr) ? "FAIL" : "OK");
}

/*
 * App entry point
 */
static void print_usage()
{
	warnx("usage: uavcan start <node_id> [can_bitrate]");
}

extern "C" __EXPORT int uavcan_main(int argc, char *argv[]);

int uavcan_main(int argc, char *argv[])
{
	constexpr unsigned DEFAULT_CAN_BITRATE = 1000000;

	if (argc < 2) {
		print_usage();
		::exit(1);
	}

	if (!std::strcmp(argv[1], "start")) {
		if (argc < 3) {
			print_usage();
			::exit(1);
		}

		/*
		 * Node ID
		 */
		const int node_id = atoi(argv[2]);

		if (node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast()) {
			warnx("Invalid Node ID %i", node_id);
			::exit(1);
		}

		/*
		 * CAN bitrate
		 */
		unsigned bitrate = 0;

		if (argc > 3) {
			bitrate = atol(argv[3]);
		}

		if (bitrate <= 0) {
			bitrate = DEFAULT_CAN_BITRATE;
		}

		if (UavcanNode::instance()) {
			errx(1, "already started");
		}

		/*
		 * Start
		 */
		warnx("Node ID %u, bitrate %u", node_id, bitrate);
		return UavcanNode::start(node_id, bitrate);

	}

	/* commands below require the app to be started */
	UavcanNode *inst = UavcanNode::instance();

	if (!inst) {
		errx(1, "application not running");
	}

	if (!std::strcmp(argv[1], "status") || !std::strcmp(argv[1], "info")) {
		
			inst->print_info();
			return OK;
	}

	if (!std::strcmp(argv[1], "stop")) {
		
			delete inst;
			inst = nullptr;
			return OK;
	}

	print_usage();
	::exit(1);
}
