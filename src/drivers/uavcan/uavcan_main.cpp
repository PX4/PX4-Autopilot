/****************************************************************************
 *
 *   Copyright (c) 2014-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
 * @file uavcan_main.cpp
 *
 * Implements basic functionality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author David Sidrane <david_s5@nscdg.com>
 * @author Andreas Jochum <Andreas@NicaDrone.com>
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <inttypes.h>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <version/version.h>

#include <arch/chip/chip.h>

#include <uORB/topics/esc_status.h>

#include <drivers/drv_hrt.h>

#include "uavcan_module.hpp"
#include "uavcan_main.hpp"
#include <uavcan/util/templates.hpp>

#include <uavcan/protocol/param/ExecuteOpcode.hpp>

//todo:The Inclusion of file_server_backend is killing
// #include <sys/types.h> and leaving OK undefined
# define OK 0

/*
 * UavcanNode
 */
UavcanNode *UavcanNode::_instance;

static UavcanNode::CanInitHelper *can = nullptr;

UavcanNode::UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	ModuleParams(nullptr),
	_node(can_driver, system_clock, _pool_allocator),
#if defined(CONFIG_UAVCAN_ARMING_CONTROLLER)
	_arming_status_controller(_node),
#endif
#if defined(CONFIG_UAVCAN_BEEP_CONTROLLER)
	_beep_controller(_node),
#endif
#if defined(CONFIG_UAVCAN_OUTPUTS_CONTROLLER)
	_esc_controller(_node),
	_servo_controller(_node),
#endif
#if defined(CONFIG_UAVCAN_HARDPOINT_CONTROLLER)
	_hardpoint_controller(_node),
#endif
#if defined(CONFIG_UAVCAN_SAFETY_STATE_CONTROLLER)
	_safety_state_controller(_node),
#endif
#if defined(CONFIG_UAVCAN_RGB_CONTROLLER)
	_rgbled_controller(_node),
#endif
	_log_message_controller(_node),
	_time_sync_master(_node),
	_time_sync_slave(_node),
	_node_status_monitor(_node),
	_node_info_retriever(_node),
	_master_timer(_node),
	_param_getset_client(_node),
	_param_opcode_client(_node),
	_param_restartnode_client(_node)
{
	int res = pthread_mutex_init(&_node_mutex, nullptr);

	if (res < 0) {
		std::abort();
	}

#if defined(CONFIG_UAVCAN_OUTPUTS_CONTROLLER)
	_mixing_interface_esc.mixingOutput().setMaxTopicUpdateRate(1000000 / UavcanEscController::MAX_RATE_HZ);
	_mixing_interface_servo.mixingOutput().setMaxTopicUpdateRate(1000000 / UavcanServoController::MAX_RATE_HZ);
#endif
}

UavcanNode::~UavcanNode()
{
	if (_servers != nullptr) {
		delete _servers;
		_servers = nullptr;
	}

	if (_instance) {

		/* tell the task we want it to go away */
		_task_should_exit.store(true);
		ScheduleNow();

		unsigned i = 10;

		do {
			/* wait 5ms - it should wake every 10ms or so worst-case */
			usleep(5000);

			if (--i == 0) {
				break;
			}

		} while (_instance);
	}

	// Removing the sensor bridges
	_sensor_bridges.clear();

	pthread_mutex_destroy(&_node_mutex);

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int
UavcanNode::getHardwareVersion(uavcan::protocol::HardwareVersion &hwver)
{
	int rv = -1;

	if (UavcanNode::instance()) {
		if (!std::strncmp(px4_board_name(), "PX4_FMU_V2", 9)) {
			hwver.major = 2;

		} else {
			; // All other values of px4_board_name() resolve to zero
		}

		mfguid_t mfgid = {};
		board_get_mfguid(mfgid);
		uavcan::copy(mfgid, mfgid + sizeof(mfgid), hwver.unique_id.begin());
		rv = 0;
	}

	return rv;
}

int
UavcanNode::print_params(uavcan::protocol::param::GetSet::Response &resp)
{
	if (resp.value.is(uavcan::protocol::param::Value::Tag::integer_value)) {
		return std::printf("name: %s %" PRId64 "\n", resp.name.c_str(),
				   resp.value.to<uavcan::protocol::param::Value::Tag::integer_value>());

	} else if (resp.value.is(uavcan::protocol::param::Value::Tag::real_value)) {
		return std::printf("name: %s %.4f\n", resp.name.c_str(),
				   static_cast<double>(resp.value.to<uavcan::protocol::param::Value::Tag::real_value>()));

	} else if (resp.value.is(uavcan::protocol::param::Value::Tag::boolean_value)) {
		return std::printf("name: %s %d\n", resp.name.c_str(),
				   resp.value.to<uavcan::protocol::param::Value::Tag::boolean_value>());

	} else if (resp.value.is(uavcan::protocol::param::Value::Tag::string_value)) {
		return std::printf("name: %s '%s'\n", resp.name.c_str(),
				   resp.value.to<uavcan::protocol::param::Value::Tag::string_value>().c_str());
	}

	return -1;
}

int
UavcanNode::save_params(int remote_node_id)
{
	uavcan::protocol::param::ExecuteOpcode::Request opcode_req;
	opcode_req.opcode = opcode_req.OPCODE_SAVE;
	uavcan::ServiceClient<uavcan::protocol::param::ExecuteOpcode, ExecuteOpcodeCallback> client(_node);
	client.setCallback(ExecuteOpcodeCallback(this, &UavcanNode::cb_opcode));
	_callback_success = false;
	int call_res = client.call(remote_node_id, opcode_req);

	if (call_res >= 0) {
		while (client.hasPendingCalls()) {
			usleep(10000);
		}
	}

	if (!_callback_success) {
		std::printf("Failed to save parameters: %d\n", call_res);
		return -1;
	}

	return 0;
}

int
UavcanNode::reset_node(int remote_node_id)
{
	uavcan::protocol::RestartNode::Request restart_req;
	restart_req.magic_number = restart_req.MAGIC_NUMBER;
	uavcan::ServiceClient<uavcan::protocol::RestartNode, RestartNodeCallback> client(_node);
	client.setCallback(RestartNodeCallback(this, &UavcanNode::cb_restart));
	_callback_success = false;
	int call_res = client.call(remote_node_id, restart_req);

	if (call_res >= 0) {
		while (client.hasPendingCalls()) {
			usleep(10000);
		}
	}

	if (!call_res) {
		std::printf("Failed to reset node: %d\n", remote_node_id);
		return -1;
	}

	return 0;
}

int
UavcanNode::list_params(int remote_node_id)
{
	int rv = 0;
	int index = 0;
	uavcan::protocol::param::GetSet::Response resp;
	set_setget_response(&resp);

	while (true) {
		uavcan::protocol::param::GetSet::Request req;
		req.index =  index++;
		_callback_success = false;
		int call_res = get_set_param(remote_node_id, nullptr, req);

		if (call_res < 0 || !_callback_success) {
			std::printf("Failed to get param: %d\n", call_res);
			rv = -1;
			break;
		}

		if (resp.name.empty()) { // Empty name means no such param, which means we're finished
			break;
		}

		print_params(resp);
	}

	free_setget_response();
	return rv;
}

void
UavcanNode::cb_setget(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result)
{
	_callback_success = result.isSuccessful();
	*_setget_response = result.getResponse();
}

int
UavcanNode::get_set_param(int remote_node_id, const char *name, uavcan::protocol::param::GetSet::Request &req)
{
	if (name != nullptr) {
		req.name = name;
	}

	uavcan::ServiceClient<uavcan::protocol::param::GetSet, GetSetCallback> client(_node);
	client.setCallback(GetSetCallback(this, &UavcanNode::cb_setget));
	_callback_success = false;
	int call_res = client.call(remote_node_id, req);

	if (call_res >= 0) {

		while (client.hasPendingCalls()) {
			usleep(10000);
		}

		if (!_callback_success) {
			call_res = -1;
		}
	}

	return call_res;
}

int
UavcanNode::set_param(int remote_node_id, const char *name, char *value)
{
	uavcan::protocol::param::GetSet::Request req;
	uavcan::protocol::param::GetSet::Response resp;
	set_setget_response(&resp);
	int rv = get_set_param(remote_node_id, name, req);

	if (rv < 0 || resp.name.empty()) {
		std::printf("Failed to retrieve param: %s\n", name);
		rv = -1;

	} else {

		rv = 0;
		req = {};

		if (resp.value.is(uavcan::protocol::param::Value::Tag::integer_value)) {
			int64_t i = std::strtoull(value, NULL, 10);
			int64_t min = resp.min_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>();
			int64_t max = resp.max_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>();

			if (i >= min &&  i <= max) {
				req.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = i;

			} else {
				std::printf("Invalid value for: %s must be between %" PRId64 " and %" PRId64 "\n", name, min, max);
				rv = -1;
			}

		} else if (resp.value.is(uavcan::protocol::param::Value::Tag::real_value)) {
			float f = static_cast<float>(std::atof(value));
			float min = resp.min_value.to<uavcan::protocol::param::NumericValue::Tag::real_value>();
			float max = resp.max_value.to<uavcan::protocol::param::NumericValue::Tag::real_value>();

			if (f >= min &&  f <= max) {
				req.value.to<uavcan::protocol::param::Value::Tag::real_value>() = f;

			} else {
				std::printf("Invalid value for: %s must be between %.4f and %.4f\n", name, static_cast<double>(min),
					    static_cast<double>(max));
				rv = -1;
			}

		} else if (resp.value.is(uavcan::protocol::param::Value::Tag::boolean_value)) {
			int8_t i = (value[0] == '1' || value[0] == 't') ? 1 : 0;
			req.value.to<uavcan::protocol::param::Value::Tag::boolean_value>() = i;

		} else if (resp.value.is(uavcan::protocol::param::Value::Tag::string_value)) {
			req.value.to<uavcan::protocol::param::Value::Tag::string_value>() = value;
		}

		if (rv == 0) {
			rv = get_set_param(remote_node_id, name, req);

			if (rv == 0) {
				// commit parameter change
				save_params(remote_node_id);

			} else if (rv < 0 || resp.name.empty()) {
				std::printf("Failed to set param: %s\n", name);
				return -1;
			}

			return 0;
		}
	}

	free_setget_response();
	return rv;
}

int
UavcanNode::get_param(int remote_node_id, const char *name)
{
	uavcan::protocol::param::GetSet::Request req;
	uavcan::protocol::param::GetSet::Response resp;
	set_setget_response(&resp);
	int rv = get_set_param(remote_node_id, name, req);

	if (rv < 0 || resp.name.empty()) {
		std::printf("Failed to get param: %s\n", name);
		rv = -1;

	} else {
		print_params(resp);
		rv = 0;
	}

	free_setget_response();
	return rv;
}

void
UavcanNode::update_params()
{
#if defined(CONFIG_UAVCAN_OUTPUTS_CONTROLLER)
	_mixing_interface_esc.updateParams();
	_mixing_interface_servo.updateParams();
#endif
}

int
UavcanNode::start(uavcan::NodeID node_id, uint32_t bitrate)
{
	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

	if (can == nullptr) {

		can = new CanInitHelper(board_get_can_interfaces());

		if (can == nullptr) {  // We don't have exceptions so bad_alloc cannot be thrown
			PX4_ERR("Out of memory");
			return -1;
		}

	}

	/*
	 * Node init
	 */
	_instance = new UavcanNode(can->driver, UAVCAN_DRIVER::SystemClock::instance());

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	_instance->ScheduleOnInterval(ScheduleIntervalMs * 1000);

#if defined(CONFIG_UAVCAN_OUTPUTS_CONTROLLER)
	_instance->_mixing_interface_esc.ScheduleNow();
	_instance->_mixing_interface_servo.ScheduleNow();
#endif

	return OK;
}

void
UavcanNode::fill_node_info()
{
	/* software version */
	uavcan::protocol::SoftwareVersion swver;

	// Extracting the first 8 hex digits of the git hash and converting them to int
	char fw_git_short[9] = {};
	std::memmove(fw_git_short, px4_firmware_version_string(), 8);
	char *end = nullptr;
	swver.vcs_commit = std::strtol(fw_git_short, &end, 16);
	swver.optional_field_flags |= swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;

	// Too verbose for normal operation
	//PX4_INFO("SW version vcs_commit: 0x%08x", unsigned(swver.vcs_commit));

	_node.setSoftwareVersion(swver);

	/* hardware version */
	uavcan::protocol::HardwareVersion hwver;
	getHardwareVersion(hwver);
	_node.setHardwareVersion(hwver);
}

void
UavcanNode::busevent_signal_trampoline()
{
	if (_instance) {
		// trigger the work queue (Note, this is called from IRQ context)
		_instance->ScheduleNow();
	}
}

int
UavcanNode::init(uavcan::NodeID node_id, UAVCAN_DRIVER::BusEvent &bus_events)
{
	bus_events.registerSignalCallback(UavcanNode::busevent_signal_trampoline);

	_node.setName("org.pixhawk.pixhawk");

	_node.setNodeID(node_id);

	fill_node_info();

	int ret;

	// UAVCAN_PUB_ARM
#if defined(CONFIG_UAVCAN_ARMING_CONTROLLER)
	int32_t uavcan_pub_arm = 0;
	param_get(param_find("UAVCAN_PUB_ARM"), &uavcan_pub_arm);

	if (uavcan_pub_arm == 1) {
		ret = _arming_status_controller.init();

		if (ret < 0) {
			return ret;
		}
	}

#endif

#if defined(CONFIG_UAVCAN_BEEP_CONTROLLER)
	ret = _beep_controller.init();

	if (ret < 0) {
		return ret;
	}

#endif

	// Actuators
#if defined(CONFIG_UAVCAN_OUTPUTS_CONTROLLER)
	ret = _esc_controller.init();

	if (ret < 0) {
		return ret;
	}

#endif

#if defined(CONFIG_UAVCAN_HARDPOINT_CONTROLLER)
	ret = _hardpoint_controller.init();

	if (ret < 0) {
		return ret;
	}

#endif

#if defined(CONFIG_UAVCAN_SAFETY_CONTROLLER)
	ret = _safety_state_controller.init();

	if (ret < 0) {
		return ret;
	}

#endif

	ret = _log_message_controller.init();

	if (ret < 0) {
		return ret;
	}

#if defined(CONFIG_UAVCAN_RGB_CONTROLLER)
	ret = _rgbled_controller.init();

	if (ret < 0) {
		return ret;
	}

#endif

	/* Start node info retriever to fetch node info from new nodes */
	ret = _node_info_retriever.start();

	if (ret < 0) {
		PX4_ERR("NodeInfoRetriever init: %d", ret);
		return ret;
	}

	// Sensor bridges
	IUavcanSensorBridge::make_all(_node, _sensor_bridges);

	for (const auto &br : _sensor_bridges) {
		ret = br->init();

		if (ret < 0) {
			PX4_ERR("cannot init sensor bridge '%s' (%d)", br->get_name(), ret);
			return ret;
		}

		PX4_DEBUG("sensor bridge '%s' init ok", br->get_name());
	}

#if defined(CONFIG_UAVCAN_OUTPUTS_CONTROLLER)

	// Ensure we don't exceed maximum limits and assumptions. FIXME: these should be static assertions
	if (UavcanEscController::max_output_value() >= UavcanEscController::DISARMED_OUTPUT_VALUE
	    || UavcanEscController::max_output_value() > (int)UINT16_MAX) {
		PX4_ERR("ESC max output value assertion failed");
		return -EINVAL;
	}

	_mixing_interface_esc.mixingOutput().setAllDisarmedValues(UavcanEscController::DISARMED_OUTPUT_VALUE);
#endif

	/* Set up shared service clients */
	_param_getset_client.setCallback(GetSetCallback(this, &UavcanNode::cb_getset));
	_param_opcode_client.setCallback(ExecuteOpcodeCallback(this, &UavcanNode::cb_opcode));
	_param_restartnode_client.setCallback(RestartNodeCallback(this, &UavcanNode::cb_restart));


	int32_t uavcan_enable = 1;
	(void)param_get(param_find("UAVCAN_ENABLE"), &uavcan_enable);

	if (uavcan_enable > 1) {
		_servers = new UavcanServers(_node, _node_info_retriever);

		if (_servers) {
			int rv = _servers->init();

			if (rv < 0) {
				PX4_ERR("UavcanServers init: %d", rv);
			}
		}
	}

	// Start the Node
	return _node.start();
}

void
UavcanNode::handle_time_sync(const uavcan::TimerEvent &)
{
	/*
	 * Check whether there are higher priority masters in the network.
	 * If there are, we need to activate the local slave in order to sync with them.
	 */
	if (_time_sync_slave.isActive()) { // "Active" means that the slave tracks at least one remote master in the network
		if (_node.getNodeID() < _time_sync_slave.getMasterNodeID()) {
			/*
			 * We're the highest priority master in the network.
			 * We need to suppress the slave now to prevent it from picking up unwanted sync messages from
			 * lower priority masters.
			 */
			_time_sync_slave.suppress(true);  // SUPPRESS

		} else {
			/*
			 * There is at least one higher priority master in the network.
			 * We need to allow the slave to adjust our local clock in order to be in sync.
			 */
			_time_sync_slave.suppress(false);  // UNSUPPRESS
		}

	} else {
		/*
		 * There are no other time sync masters in the network, so we're the only time source.
		 * The slave must be suppressed anyway to prevent it from disrupting the local clock if a new
		 * lower priority master suddenly appears in the network.
		 */
		_time_sync_slave.suppress(true);
	}

	/*
	 * Publish the sync message now, even if we're not a higher priority master.
	 * Other nodes will be able to pick the right master anyway.
	 */
	_time_sync_master.publish();
}

void
UavcanNode::Run()
{
	if (!_node_init) {
		// Node ID
		int32_t node_id = 1;
		(void)param_get(param_find("UAVCAN_NODE_ID"), &node_id);

		if (node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast()) {
			PX4_ERR("Invalid Node ID %" PRId32, node_id);
			::exit(1);
		}

		// CAN bitrate
		int32_t bitrate = 1000000;
		(void)param_get(param_find("UAVCAN_BITRATE"), &bitrate);

		/*
		* CAN driver init
		 * Note that we instantiate and initialize CanInitHelper only once, because the STM32's bxCAN driver
		 * shipped with libuavcan does not support deinitialization.
		 */
		const int can_init_res = can->init(bitrate);

		if (can_init_res < 0) {
			PX4_ERR("CAN driver init failed %i", can_init_res);
		}

		_instance->init(node_id, can->driver.updateEvent());

		_node_init = true;
	}

	pthread_mutex_lock(&_node_mutex);

	if (_output_count == 0) {
		// Set up the time synchronization
		const int slave_init_res = _time_sync_slave.start();

		if (slave_init_res < 0) {
			PX4_ERR("Failed to start time_sync_slave");
			ScheduleClear();
			return;
		}

		/* When we have a system wide notion of time update (i.e the transition from the initial
		 * System RTC setting to the GPS) we would call UAVCAN_DRIVER::clock::setUtc() when that
		 * happens, but for now we use adjustUtc with a correction of the hrt so that the
		 * time bases are the same
		 */
		UAVCAN_DRIVER::clock::adjustUtc(uavcan::UtcDuration::fromUSec(hrt_absolute_time()));
		_master_timer.setCallback(TimerCallback(this, &UavcanNode::handle_time_sync));
		_master_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000));

		_node_status_monitor.start();
		_node.setModeOperational();

		update_params();

		// XXX figure out the output count
		_output_count = 2;
	}


	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	for (auto &br : _sensor_bridges) {
		br->update();
	}

	if (_check_fw) {
		_check_fw = false;
		_node_info_retriever.invalidateAll();
	}

	_node.spinOnce(); // expected to be non-blocking

	// Publish status
	constexpr hrt_abstime status_pub_interval = 100_ms;

	if (hrt_absolute_time() - _last_can_status_pub >= status_pub_interval) {
		_last_can_status_pub = hrt_absolute_time();

		for (int i = 0; i < _node.getDispatcher().getCanIOManager().getCanDriver().getNumIfaces(); i++) {
			if (i > UAVCAN_NUM_IFACES) {
				break;
			}

			auto iface = _node.getDispatcher().getCanIOManager().getCanDriver().getIface(i);

			if (!iface) {
				continue;
			}

			auto iface_perf_cnt = _node.getDispatcher().getCanIOManager().getIfacePerfCounters(i);
			can_interface_status_s status{
				.timestamp = hrt_absolute_time(),
				.io_errors = iface_perf_cnt.errors,
				.frames_tx = iface_perf_cnt.frames_tx,
				.frames_rx = iface_perf_cnt.frames_rx,
				.interface = static_cast<uint8_t>(i),
			};

			if (_can_status_pub_handles[i] == nullptr) {
				int instance{0};
				_can_status_pub_handles[i] = orb_advertise_multi(ORB_ID(can_interface_status), nullptr, &instance);
			}

			(void)orb_publish(ORB_ID(can_interface_status), _can_status_pub_handles[i], &status);
		}
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		update_params();
	}

	// Check for parameter requests (get/set/list)
	if (_param_request_sub.updated() && !_param_list_in_progress && !_param_in_progress && !_count_in_progress) {
		uavcan_parameter_request_s request{};
		_param_request_sub.copy(&request);

		if (_param_counts[request.node_id]) {
			/*
			 * We know how many parameters are exposed by this node, so
			 * process the request.
			 */
			if (request.message_type == uavcan_parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_READ) {
				uavcan::protocol::param::GetSet::Request req;

				if (request.param_index >= 0) {
					req.index = request.param_index;

				} else {
					req.name = (char *)request.param_id;
				}

				int call_res = _param_getset_client.call(request.node_id, req);

				if (call_res < 0) {
					PX4_ERR("couldn't send GetSet: %d", call_res);

				} else {
					_param_in_progress = true;
					_param_index = request.param_index;
				}

			} else if (request.message_type == uavcan_parameter_request_s::MESSAGE_TYPE_PARAM_SET) {
				uavcan::protocol::param::GetSet::Request req;

				if (request.param_index >= 0) {
					req.index = request.param_index;

				} else {
					req.name = (char *)request.param_id;
				}

				if (request.param_type == uavcan_parameter_request_s::PARAM_TYPE_REAL32) {
					req.value.to<uavcan::protocol::param::Value::Tag::real_value>() = request.real_value;

				} else if (request.param_type == uavcan_parameter_request_s::PARAM_TYPE_UINT8) {
					req.value.to<uavcan::protocol::param::Value::Tag::boolean_value>() = request.int_value;

				} else {
					req.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = request.int_value;
				}

				// Set the dirty bit for this node
				set_node_params_dirty(request.node_id);

				int call_res = _param_getset_client.call(request.node_id, req);

				if (call_res < 0) {
					PX4_ERR("couldn't send GetSet: %d", call_res);

				} else {
					_param_in_progress = true;
					_param_index = request.param_index;
				}

			} else if (request.message_type == uavcan_parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_LIST) {
				// This triggers the _param_list_in_progress case below.
				_param_index = 0;
				_param_list_in_progress = true;
				_param_list_node_id = request.node_id;
				_param_list_all_nodes = false;

				PX4_DEBUG("starting component-specific param list");
			}

		} else if (request.node_id == uavcan_parameter_request_s::NODE_ID_ALL) {
			if (request.message_type == uavcan_parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_LIST) {
				/*
				 * This triggers the _param_list_in_progress case below,
				 * but additionally iterates over all active nodes.
				 */
				_param_index = 0;
				_param_list_in_progress = true;
				_param_list_node_id = get_next_active_node_id(0);
				_param_list_all_nodes = true;

				PX4_DEBUG("starting global param list with node %hhu", _param_list_node_id);

				if (_param_counts[_param_list_node_id] == 0) {
					param_count(_param_list_node_id);
				}
			}

		} else {
			/*
			 * Need to know how many parameters this node has before we can
			 * continue; count them now and then process the request.
			 */
			param_count(request.node_id);
		}
	}

	// Handle parameter listing index/node ID advancement
	if (_param_list_in_progress && !_param_in_progress && !_count_in_progress) {
		if (_param_index >= _param_counts[_param_list_node_id]) {
			PX4_DEBUG("completed param list for node %hhu", _param_list_node_id);
			// Reached the end of the current node's parameter set.
			_param_list_in_progress = false;

			if (_param_list_all_nodes) {
				// We're listing all parameters for all nodes -- get the next node ID
				uint8_t next_id = get_next_active_node_id(_param_list_node_id);

				if (next_id < 128) {
					_param_list_node_id = next_id;

					/*
					 * If there is a next node ID, check if that node's parameters
					 * have been counted before. If not, do it now.
					 */
					if (_param_counts[_param_list_node_id] == 0) {
						param_count(_param_list_node_id);
					}

					// Keep on listing.
					_param_index = 0;
					_param_list_in_progress = true;
					PX4_DEBUG("started param list for node %hhu", _param_list_node_id);
				}
			}
		}
	}

	// Check if we're still listing, and need to get the next parameter
	if (_param_list_in_progress && !_param_in_progress && !_count_in_progress) {
		// Ready to request the next value -- _param_index is incremented
		// after each successful fetch by cb_getset
		uavcan::protocol::param::GetSet::Request req;
		req.index = _param_index;

		int call_res = _param_getset_client.call(_param_list_node_id, req);

		if (call_res < 0) {
			_param_list_in_progress = false;
			PX4_ERR("couldn't send param list GetSet: %d", call_res);

		} else {
			_param_in_progress = true;
		}
	}

	if (_vcmd_sub.updated() && !_cmd_in_progress) {
		bool acknowledge = false;
		vehicle_command_s cmd{};
		_vcmd_sub.copy(&cmd);

		uint8_t cmd_ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

		if (cmd.command == vehicle_command_s::VEHICLE_CMD_PREFLIGHT_STORAGE) {
			acknowledge = true;
			int command_id = static_cast<int>(cmd.param1 + 0.5f);

			PX4_DEBUG("received storage command ID %d", command_id);

			switch (command_id) {
			case 1: {
					// Param save request
					int node_id;
					node_id = get_next_dirty_node_id(1);

					if (node_id < 128) {
						_param_save_opcode = uavcan::protocol::param::ExecuteOpcode::Request::OPCODE_SAVE;
						param_opcode(node_id);
					}

					break;
				}

			case 2: {
					// Command is a param erase request -- apply it to all active nodes by setting the dirty bit
					_param_save_opcode = uavcan::protocol::param::ExecuteOpcode::Request::OPCODE_ERASE;

					for (int i = 1; i < 128; i = get_next_active_node_id(i)) {
						set_node_params_dirty(i);
					}

					param_opcode(get_next_dirty_node_id(1));
					break;
				}
			}
		}

		if (acknowledge) {
			// Acknowledge the received command
			vehicle_command_ack_s ack{};
			ack.command = cmd.command;
			ack.result = cmd_ack_result;
			ack.target_system = cmd.source_system;
			ack.target_component = cmd.source_component;
			ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(ack);
		}
	}

	perf_end(_cycle_perf);

	pthread_mutex_unlock(&_node_mutex);

	if (_task_should_exit.load()) {

#if defined(CONFIG_UAVCAN_OUTPUTS_CONTROLLER)
		_mixing_interface_esc.mixingOutput().unregister();
		_mixing_interface_esc.ScheduleClear();

		_mixing_interface_servo.mixingOutput().unregister();
		_mixing_interface_servo.ScheduleClear();
#endif
		ScheduleClear();
		_instance = nullptr;
	}
}

#if defined(CONFIG_UAVCAN_OUTPUTS_CONTROLLER)
bool UavcanMixingInterfaceESC::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
		unsigned num_control_groups_updated)
{
	_esc_controller.update_outputs(stop_motors, outputs, num_outputs);
	return true;
}

void UavcanMixingInterfaceESC::Run()
{
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);
}

void UavcanMixingInterfaceESC::mixerChanged()
{
	int rotor_count = 0;

	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		rotor_count += _mixing_output.isFunctionSet(i);

		if (i < esc_status_s::CONNECTED_ESC_MAX) {
			_esc_controller.esc_status().esc[i].actuator_function = (uint8_t)_mixing_output.outputFunction(i);
		}
	}

	_esc_controller.set_rotor_count(rotor_count);
}

bool UavcanMixingInterfaceServo::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
		unsigned num_control_groups_updated)
{
	_servo_controller.update_outputs(stop_motors, outputs, num_outputs);
	return true;
}

void UavcanMixingInterfaceServo::Run()
{
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);
}
#endif

void
UavcanNode::print_info()
{
	(void)pthread_mutex_lock(&_node_mutex);

	// Memory status
	printf("Pool allocator status:\n");
	printf("\tCapacity hard/soft: %" PRIu16 "/%" PRIu16 " blocks\n",
	       _pool_allocator.getBlockCapacityHardLimit(), _pool_allocator.getBlockCapacity());
	printf("\tReserved:  %" PRIu16 " blocks\n", _pool_allocator.getNumReservedBlocks());
	printf("\tAllocated: %" PRIu16 " blocks\n", _pool_allocator.getNumAllocatedBlocks());

	printf("\n");

	// UAVCAN node perfcounters
	printf("UAVCAN node status:\n");
	printf("\tInternal failures: %" PRIu64 "\n", _node.getInternalFailureCount());
	printf("\tTransfer errors:   %" PRIu64 "\n", _node.getDispatcher().getTransferPerfCounter().getErrorCount());
	printf("\tRX transfers:      %" PRIu64 "\n", _node.getDispatcher().getTransferPerfCounter().getRxTransferCount());
	printf("\tTX transfers:      %" PRIu64 "\n", _node.getDispatcher().getTransferPerfCounter().getTxTransferCount());

	printf("\n");

	// CAN driver status
	for (unsigned i = 0; i < _node.getDispatcher().getCanIOManager().getCanDriver().getNumIfaces(); i++) {
		printf("CAN%u status:\n", unsigned(i + 1));

		auto iface = _node.getDispatcher().getCanIOManager().getCanDriver().getIface(i);

		if (iface) {
			printf("\tHW errors: %" PRIu64 "\n", iface->getErrorCount());

			auto iface_perf_cnt = _node.getDispatcher().getCanIOManager().getIfacePerfCounters(i);
			printf("\tIO errors: %" PRIu64 "\n", iface_perf_cnt.errors);
			printf("\tRX frames: %" PRIu64 "\n", iface_perf_cnt.frames_rx);
			printf("\tTX frames: %" PRIu64 "\n", iface_perf_cnt.frames_tx);
		}
	}

	printf("\n");

#if defined(CONFIG_UAVCAN_OUTPUTS_CONTROLLER)
	printf("ESC outputs:\n");
	_mixing_interface_esc.mixingOutput().printStatus();

	printf("Servo outputs:\n");
	_mixing_interface_servo.mixingOutput().printStatus();
#endif

	printf("\n");

	// Sensor bridges
	for (const auto &br : _sensor_bridges) {
		printf("Sensor '%s':\n", br->get_name());
		br->print_status();
		printf("\n");
	}

	// Printing all nodes that are online
	printf("Online nodes (Node ID, Health, Mode):\n");
	_node_status_monitor.forEachNode([](uavcan::NodeID nid, uavcan::NodeStatusMonitor::NodeStatus ns) {
		static constexpr const char *HEALTH[] = {"OK", "WARN", "ERR", "CRIT"};
		static constexpr const char *MODES[] = {"OPERAT", "INIT", "MAINT", "SW_UPD", "?", "?", "?", "OFFLN"};
		printf("\t% 3d %-10s %-10s\n", int(nid.get()), HEALTH[ns.health], MODES[ns.mode]);
	});

	printf("\n");

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	(void)pthread_mutex_unlock(&_node_mutex);
}

void
UavcanNode::shrink()
{
	(void)pthread_mutex_lock(&_node_mutex);
	_pool_allocator.shrink();
	(void)pthread_mutex_unlock(&_node_mutex);
}

void
UavcanNode::cb_getset(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result)
{
	if (_count_in_progress) {
		/*
		 * Currently in parameter count mode:
		 * Iterate over all parameters for the node to which the request was
		 * originally sent, in order to find the maximum parameter ID. If a
		 * request fails, set the node's parameter count to zero.
		 */
		uint8_t node_id = result.getCallID().server_node_id.get();

		if (result.isSuccessful()) {
			uavcan::protocol::param::GetSet::Response resp = result.getResponse();

			if (resp.name.size()) {
				_count_index++;
				_param_counts[node_id] = _count_index;

				uavcan::protocol::param::GetSet::Request req;
				req.index = _count_index;

				int call_res = _param_getset_client.call(result.getCallID().server_node_id, req);

				if (call_res < 0) {
					_count_in_progress = false;
					_count_index = 0;
					PX4_ERR("couldn't send GetSet during param count: %d", call_res);
				}

			} else {
				_count_in_progress = false;
				_count_index = 0;
				PX4_DEBUG("completed param count for node %hhu: %hhu", node_id, _param_counts[node_id]);
			}

		} else {
			_param_counts[node_id] = 0;
			_count_in_progress = false;
			_count_index = 0;
			PX4_ERR("GetSet error during param count");
		}

	} else {
		/*
		 * Currently in parameter get/set mode:
		 * Publish a uORB uavcan_parameter_value message containing the current value
		 * of the parameter.
		 */
		if (result.isSuccessful()) {
			uavcan::protocol::param::GetSet::Response param = result.getResponse();

			uavcan_parameter_value_s response{};
			response.node_id = result.getCallID().server_node_id.get();
			strncpy(response.param_id, param.name.c_str(), sizeof(response.param_id) - 1);
			response.param_id[16] = '\0';
			response.param_index = _param_index;
			response.param_count = _param_counts[response.node_id];

			if (param.value.is(uavcan::protocol::param::Value::Tag::integer_value)) {
				response.param_type = uavcan_parameter_request_s::PARAM_TYPE_INT64;
				response.int_value = param.value.to<uavcan::protocol::param::Value::Tag::integer_value>();

			} else if (param.value.is(uavcan::protocol::param::Value::Tag::real_value)) {
				response.param_type = uavcan_parameter_request_s::PARAM_TYPE_REAL32;
				response.real_value = param.value.to<uavcan::protocol::param::Value::Tag::real_value>();

			} else if (param.value.is(uavcan::protocol::param::Value::Tag::boolean_value)) {
				response.param_type = uavcan_parameter_request_s::PARAM_TYPE_UINT8;
				response.int_value = param.value.to<uavcan::protocol::param::Value::Tag::boolean_value>();
			}

			_param_response_pub.publish(response);

		} else {
			PX4_ERR("GetSet error");
		}

		_param_in_progress = false;
		_param_index++;
	}
}

void
UavcanNode::param_count(uavcan::NodeID node_id)
{
	uavcan::protocol::param::GetSet::Request req;
	req.index = 0;
	int call_res = _param_getset_client.call(node_id, req);

	// -ErrInvalidParam is returned when no UAVCAN device is connected to the CAN bus
	if ((call_res < 0) && (-uavcan::ErrInvalidParam != call_res)) {
		PX4_ERR("couldn't start parameter count: %d", call_res);

	} else {
		_count_in_progress = true;
		_count_index = 0;
		PX4_DEBUG("starting param count");
	}
}

void
UavcanNode::param_opcode(uavcan::NodeID node_id)
{
	uavcan::protocol::param::ExecuteOpcode::Request opcode_req;
	opcode_req.opcode = _param_save_opcode;
	int call_res = _param_opcode_client.call(node_id, opcode_req);

	if (call_res < 0) {
		PX4_ERR("couldn't send ExecuteOpcode: %d", call_res);

	} else {
		_cmd_in_progress = true;
		PX4_INFO("sent ExecuteOpcode");
	}
}

void
UavcanNode::cb_opcode(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &result)
{
	bool success = result.isSuccessful();
	uint8_t node_id = result.getCallID().server_node_id.get();
	uavcan::protocol::param::ExecuteOpcode::Response resp = result.getResponse();
	success &= resp.ok;
	_cmd_in_progress = false;

	if (!result.isSuccessful()) {
		PX4_ERR("save request for node %hhu timed out.", node_id);

	} else if (!result.getResponse().ok) {
		PX4_ERR("save request for node %hhu rejected.", node_id);

	} else {
		PX4_INFO("save request for node %hhu completed OK, restarting.", node_id);

		uavcan::protocol::RestartNode::Request restart_req;
		restart_req.magic_number = restart_req.MAGIC_NUMBER;
		int call_res = _param_restartnode_client.call(node_id, restart_req);

		if (call_res < 0) {
			PX4_ERR("couldn't send RestartNode: %d", call_res);

		} else {
			PX4_ERR("sent RestartNode");
			_cmd_in_progress = true;
		}
	}

	if (!_cmd_in_progress) {
		/*
		 * Something went wrong, so cb_restart is never going to be called as a result of this request.
		 * To ensure we try to execute the opcode on all nodes that permit it, get the next dirty node
		 * ID and keep processing here. The dirty bit on the current node is still set, so the
		 * save/erase attempt will occur when the next save/erase command is received over MAVLink.
		 */
		node_id = get_next_dirty_node_id(node_id);

		if (node_id < 128) {
			param_opcode(node_id);
		}
	}
}

void
UavcanNode::cb_restart(const uavcan::ServiceCallResult<uavcan::protocol::RestartNode> &result)
{
	bool success = result.isSuccessful();
	uint8_t node_id = result.getCallID().server_node_id.get();
	uavcan::protocol::RestartNode::Response resp = result.getResponse();
	success &= resp.ok;
	_cmd_in_progress = false;

	if (success) {
		PX4_DEBUG("restart request for node %hhu completed OK.", node_id);

		// Clear the dirty flag
		clear_node_params_dirty(node_id);

	} else {
		PX4_ERR("restart request for node %hhu failed.", node_id);
	}

	// Get the next dirty node ID and send the same command to it
	node_id = get_next_dirty_node_id(node_id);

	if (node_id < 128) {
		param_opcode(node_id);
	}
}

uint8_t
UavcanNode::get_next_active_node_id(uint8_t base)
{
	base++;

	for (; base < 128 && (!_node_info_retriever.isNodeKnown(base) || _node.getNodeID().get() == base); base++);

	return base;
}

uint8_t
UavcanNode::get_next_dirty_node_id(uint8_t base)
{
	base++;

	for (; base < 128 && !are_node_params_dirty(base); base++);

	return base;
}

/*
 * App entry point
 */
static void print_usage()
{
	PX4_INFO("usage: \n"
		 "\tuavcan {start|status|stop|shrink|update}\n"
		 "\t        param [set|get|list|save] <node-id> <name> <value>|reset <node-id>");
}

extern "C" __EXPORT int uavcan_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		::exit(1);
	}

	if (!std::strcmp(argv[1], "start")) {
		if (UavcanNode::instance()) {
			// Already running, no error
			PX4_INFO("already started");
			::exit(0);
		}

		// Node ID
		int32_t node_id = 1;
		(void)param_get(param_find("UAVCAN_NODE_ID"), &node_id);

		if (node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast()) {
			PX4_ERR("Invalid Node ID %" PRId32, node_id);
			::exit(1);
		}

		// CAN bitrate
		int32_t bitrate = 1000000;
		(void)param_get(param_find("UAVCAN_BITRATE"), &bitrate);

		// Start
		PX4_INFO("Node ID %" PRIu32 ", bitrate %" PRIu32, node_id, bitrate);
		return UavcanNode::start(node_id, bitrate);
	}

	/* commands below require the app to be started */
	UavcanNode *const inst = UavcanNode::instance();

	if (!inst) {
		errx(1, "application not running");
	}

	if (!std::strcmp(argv[1], "update")) {
		if (UavcanNode::instance() == nullptr) {
			errx(1, "firmware server is not running");
		}

		UavcanNode::instance()->requestCheckAllNodesFirmwareAndUpdate();
		::exit(0);
	}

	if (!std::strcmp(argv[1], "status") || !std::strcmp(argv[1], "info")) {
		inst->print_info();
		::exit(0);
	}

	if (!std::strcmp(argv[1], "shrink")) {
		inst->shrink();
		::exit(0);
	}

	/*
	 * Parameter setting commands
	 *
	 *  uavcan param list <node>
	 *  uavcan param save <node>
	 *  uavcan param get <node> <name>
	 *  uavcan param set <node> <name> <value>
	 *
	 */
	int node_arg = !std::strcmp(argv[1], "reset") ? 2 : 3;

	if (!std::strcmp(argv[1], "param") || node_arg == 2) {
		if (argc < node_arg + 1) {
			errx(1, "Node id required");
		}

		int nodeid = atoi(argv[node_arg]);

		if (nodeid  == 0 || nodeid  > 127 || nodeid  == inst->get_node().getNodeID().get()) {
			errx(1, "Invalid Node id");
		}

		if (node_arg == 2) {

			return inst->reset_node(nodeid);

		} else if (!std::strcmp(argv[2], "list")) {

			return inst->list_params(nodeid);

		} else if (!std::strcmp(argv[2], "save")) {

			return inst->save_params(nodeid);

		} else if (!std::strcmp(argv[2], "get")) {
			if (argc < 5) {
				errx(1, "Name required");
			}

			return inst->get_param(nodeid, argv[4]);

		} else if (!std::strcmp(argv[2], "set")) {
			if (argc < 5) {
				errx(1, "Name required");
			}

			if (argc < 6) {
				errx(1, "Value required");
			}

			return inst->set_param(nodeid, argv[4], argv[5]);
		}
	}

	if (!std::strcmp(argv[1], "stop")) {
		delete inst;
		::exit(0);
	}

	print_usage();
	::exit(1);
}
