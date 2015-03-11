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
#include <systemlib/param/param.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/board_serial.h>
#include <systemlib/scheduling_priorities.h>
#include <version/version.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>

#include <uORB/topics/esc_status.h>

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
	_node_mutex(),
	_esc_controller(_node)
{
	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);

	const int res = pthread_mutex_init(&_node_mutex, nullptr);
	if (res < 0) {
		std::abort();
	}

	if (_perfcnt_node_spin_elapsed == nullptr) {
		errx(1, "uavcan: couldn't allocate _perfcnt_node_spin_elapsed");
	}

	if (_perfcnt_esc_mixer_output_elapsed == nullptr) {
		errx(1, "uavcan: couldn't allocate _perfcnt_esc_mixer_output_elapsed");
	}

	if (_perfcnt_esc_mixer_total_elapsed == nullptr) {
		errx(1, "uavcan: couldn't allocate _perfcnt_esc_mixer_total_elapsed");
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

	/* clean up the alternate device node */
	// unregister_driver(PWM_OUTPUT_DEVICE_PATH);

	::close(_armed_sub);

	// Removing the sensor bridges
	auto br = _sensor_bridges.getHead();
	while (br != nullptr) {
		auto next = br->getSibling();
		delete br;
		br = next;
	}

	_instance = nullptr;

	perf_free(_perfcnt_node_spin_elapsed);
	perf_free(_perfcnt_esc_mixer_output_elapsed);
	perf_free(_perfcnt_esc_mixer_total_elapsed);
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

	// Do regular cdev init
	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	_node.setName("org.pixhawk.pixhawk");

	_node.setNodeID(node_id);

	fill_node_info();

	// Actuators
	ret = _esc_controller.init();
	if (ret < 0) {
		return ret;
	}

	// Sensor bridges
	IUavcanSensorBridge::make_all(_node, _sensor_bridges);
	auto br = _sensor_bridges.getHead();
	while (br != nullptr) {
		ret = br->init();
		if (ret < 0) {
			warnx("cannot init sensor bridge '%s' (%d)", br->get_name(), ret);
			return ret;
		}
		warnx("sensor bridge '%s' init ok", br->get_name());
		br = br->getSibling();
	}

	return _node.start();
}

void UavcanNode::node_spin_once()
{
	perf_begin(_perfcnt_node_spin_elapsed);
	const int spin_res = _node.spin(uavcan::MonotonicTime());
	if (spin_res < 0) {
		warnx("node spin error %i", spin_res);
	}
	perf_end(_perfcnt_node_spin_elapsed);
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

	// XXX figure out the output count
	_output_count = 2;

	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_test_motor_sub = orb_subscribe(ORB_ID(test_motor));
	_actuator_direct_sub = orb_subscribe(ORB_ID(actuator_direct));

	memset(&_outputs, 0, sizeof(_outputs));

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

	/*
	 * This event is needed to wake up the thread on CAN bus activity (RX/TX/Error).
	 * Please note that with such multiplexing it is no longer possible to rely only on
	 * the value returned from poll() to detect whether actuator control has timed out or not.
	 * Instead, all ORB events need to be checked individually (see below).
	 */
	add_poll_fd(busevent_fd);

	/*
	 * setup poll to look for actuator direct input if we are
	 * subscribed to the topic
	 */
	if (_actuator_direct_sub != -1) {
		_actuator_direct_poll_fd_num = add_poll_fd(_actuator_direct_sub);
	}

	while (!_task_should_exit) {
		// update actuator controls subscriptions if needed
		if (_groups_subscribed != _groups_required) {
			subscribe();
			_groups_subscribed = _groups_required;
		}

		// Mutex is unlocked while the thread is blocked on IO multiplexing
		(void)pthread_mutex_unlock(&_node_mutex);

		perf_end(_perfcnt_esc_mixer_total_elapsed); // end goes first, it's not a mistake

		const int poll_ret = ::poll(_poll_fds, _poll_fds_num, PollTimeoutMs);

		perf_begin(_perfcnt_esc_mixer_total_elapsed);

		(void)pthread_mutex_lock(&_node_mutex);

		node_spin_once();  // Non-blocking

		bool new_output = false;

		// this would be bad...
		if (poll_ret < 0) {
			log("poll error %d", errno);
			continue;
		} else {
			// get controls for required topics
			bool controls_updated = false;
			for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
				if (_control_subs[i] > 0) {
					if (_poll_fds[_poll_ids[i]].revents & POLLIN) {
						controls_updated = true;
						orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);
					}
				}
			}

			/*
			  see if we have any direct actuator updates
			 */
			if (_actuator_direct_sub != -1 &&
			    (_poll_fds[_actuator_direct_poll_fd_num].revents & POLLIN) &&
			    orb_copy(ORB_ID(actuator_direct), _actuator_direct_sub, &_actuator_direct) == OK &&
			    !_test_in_progress) {
				if (_actuator_direct.nvalues > NUM_ACTUATOR_OUTPUTS) {
					_actuator_direct.nvalues = NUM_ACTUATOR_OUTPUTS;
				}
				memcpy(&_outputs.output[0], &_actuator_direct.values[0],
				       _actuator_direct.nvalues*sizeof(float));
				_outputs.noutputs = _actuator_direct.nvalues;
				new_output = true;
			}

			// can we mix?
			if (_test_in_progress) {
				memset(&_outputs, 0, sizeof(_outputs));
				if (_test_motor.motor_number < NUM_ACTUATOR_OUTPUTS) {
					_outputs.output[_test_motor.motor_number] = _test_motor.value*2.0f-1.0f;
					_outputs.noutputs = _test_motor.motor_number+1;
				}
				new_output = true;
			} else if (controls_updated && (_mixers != nullptr)) {

				// XXX one output group has 8 outputs max,
				// but this driver could well serve multiple groups.
				unsigned num_outputs_max = 8;

				// Do mixing
				_outputs.noutputs = _mixers->mix(&_outputs.output[0], num_outputs_max);

				new_output = true;
			}
		}

		if (new_output) {
			// iterate actuators, checking for valid values
			for (uint8_t i = 0; i < _outputs.noutputs; i++) {
				// last resort: catch NaN, INF and out-of-band errors
				if (!isfinite(_outputs.output[i])) {
					/*
					 * Value is NaN, INF or out of band - set to the minimum value.
					 * This will be clearly visible on the servo status and will limit the risk of accidentally
					 * spinning motors. It would be deadly in flight.
					 */
					_outputs.output[i] = -1.0f;
				}

				// never go below min
				if (_outputs.output[i] < -1.0f) {
					_outputs.output[i] = -1.0f;
				}

				// never go above max
				if (_outputs.output[i] > 1.0f) {
					_outputs.output[i] = 1.0f;
				}
			}
			// Output to the bus
			_outputs.timestamp = hrt_absolute_time();
			perf_begin(_perfcnt_esc_mixer_output_elapsed);
			_esc_controller.update_outputs(_outputs.output, _outputs.noutputs);
			perf_end(_perfcnt_esc_mixer_output_elapsed);
		}


		// Check motor test state
		bool updated = false;
		orb_check(_test_motor_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(test_motor), _test_motor_sub, &_test_motor);

			// Update the test status and check that we're not locked down
			_test_in_progress = (_test_motor.value > 0);
			_esc_controller.arm_single_esc(_test_motor.motor_number, _test_in_progress);
		}

		// Check arming state
		orb_check(_armed_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

			// Update the armed status and check that we're not locked down and motor
			// test is not running
			bool set_armed = _armed.armed && !_armed.lockdown && !_test_in_progress;

			arm_actuators(set_armed);
		}
	}

	teardown();
	warnx("exiting.");

	exit(0);
}

int
UavcanNode::control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];
	return 0;
}

int
UavcanNode::teardown()
{
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] > 0) {
			::close(_control_subs[i]);
			_control_subs[i] = -1;
		}
	}
	return (_armed_sub >= 0) ? ::close(_armed_sub) : 0;
}

int
UavcanNode::arm_actuators(bool arm)
{
	_is_armed = arm;
	_esc_controller.arm_all_escs(arm);
	return OK;
}

void
UavcanNode::subscribe()
{
	// Subscribe/unsubscribe to required actuator control groups
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	// the first fd used by CAN
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
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
			_poll_ids[i] = add_poll_fd(_control_subs[i]);
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

	(void)pthread_mutex_lock(&_node_mutex);

	// ESC mixer status
	printf("ESC actuators control groups: sub: %u / req: %u / fds: %u\n",
	       (unsigned)_groups_subscribed, (unsigned)_groups_required, _poll_fds_num);
	printf("ESC mixer: %s\n", (_mixers == nullptr) ? "NONE" : "OK");

	if (_outputs.noutputs != 0) {
		printf("ESC output: ");

		for (uint8_t i=0; i<_outputs.noutputs; i++) {
			printf("%d ", (int)(_outputs.output[i]*1000));
		}
		printf("\n");

		// ESC status
		int esc_sub = orb_subscribe(ORB_ID(esc_status));
		struct esc_status_s esc;
		memset(&esc, 0, sizeof(esc));
		orb_copy(ORB_ID(esc_status), esc_sub, &esc);

		printf("ESC Status:\n");
		printf("Addr\tV\tA\tTemp\tSetpt\tRPM\tErr\n");
		for (uint8_t i=0; i<_outputs.noutputs; i++) {
			printf("%d\t",    esc.esc[i].esc_address);
			printf("%3.2f\t", (double)esc.esc[i].esc_voltage);
			printf("%3.2f\t", (double)esc.esc[i].esc_current);
			printf("%3.2f\t", (double)esc.esc[i].esc_temperature);
			printf("%3.2f\t", (double)esc.esc[i].esc_setpoint);
			printf("%d\t",    esc.esc[i].esc_rpm);
			printf("%d",      esc.esc[i].esc_errorcount);
			printf("\n");
		}

		orb_unsubscribe(esc_sub);
	}

	// Sensor bridges
	auto br = _sensor_bridges.getHead();
	while (br != nullptr) {
		printf("Sensor '%s':\n", br->get_name());
		br->print_status();
		printf("\n");
		br = br->getSibling();
	}

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

extern "C" __EXPORT int uavcan_main(int argc, char *argv[]);

int uavcan_main(int argc, char *argv[])
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

	if (!std::strcmp(argv[1], "arm")) {
		inst->arm_actuators(true);
		::exit(0);
	}

	if (!std::strcmp(argv[1], "disarm")) {
		inst->arm_actuators(false);
		::exit(0);
	}

	if (!std::strcmp(argv[1], "stop")) {
		delete inst;
		::exit(0);
	}

	print_usage();
	::exit(1);
}
