#include "GL40IIDriver.hpp"

#include <board_config.h>
#include <cmath>
#include <cstring>
#include <inttypes.h>
#include <parameters/param.h>
#include <px4_platform_common/board_common.h>
#include <px4_platform_common/log.h>
#include <uavcan_stm32h7/can.hpp>
#include <uavcan_stm32h7/clock.hpp>

using namespace time_literals;

namespace
{

using CanHelper = uavcan_stm32h7::CanInitHelper<64>;
CanHelper *g_can{nullptr};
bool g_can_initialized{false};

}

ModuleBase::Descriptor GL40IIDriver::desc{task_spawn, custom_command, print_usage};

GL40IIDriver::GL40IIDriver() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
	for (uint8_t rotor = 0; rotor < gl40ii::NUM_MOTORS; ++rotor) {
		_joint_angle[rotor] = NAN;
		_joint_velocity[rotor] = NAN;
		_motor_torque[rotor] = NAN;
	}
}

GL40IIDriver::~GL40IIDriver()
{
	if (g_can_initialized) {
		sendSpecialAll(gl40ii::SpecialCommand::Disable);
	}

	perf_free(_cycle_perf);
}

bool GL40IIDriver::init()
{
	int32_t uavcan_enable = 0;
	const param_t uavcan_enable_handle = param_find("UAVCAN_ENABLE");

	if (uavcan_enable_handle != PARAM_INVALID) {
		param_get(uavcan_enable_handle, &uavcan_enable);
	}

	if (uavcan_enable != 0) {
		PX4_ERR("UAVCAN_ENABLE must be 0; GL40II owns CAN1/CAN2");
		return false;
	}

	const uint16_t can_interfaces = board_get_can_interfaces();

	if ((can_interfaces & 0x3u) != 0x3u) {
		PX4_ERR("CAN1 and CAN2 are required (mask=0x%x)", can_interfaces);
		return false;
	}

	if (g_can == nullptr) {
		g_can = new CanHelper(can_interfaces & 0x3u);

		if (g_can == nullptr) {
			PX4_ERR("CAN allocation failed");
			return false;
		}
	}

	if (!g_can_initialized) {
		uavcan_stm32h7::clock::init();
		const int result = g_can->init(gl40ii::CAN_BITRATE);

		if (result < 0) {
			PX4_ERR("CAN initialization failed (%d)", result);
			return false;
		}

		// The STM32H7 backend initializes both interfaces to accept unmatched
		// frames into FIFO0. Do not call its configureFilters() implementation:
		// in this PX4/libuavcan revision it treats the FDCAN message-RAM offset
		// register as a CPU pointer and hard-faults. These are dedicated GL40II
		// buses, and drainRx() applies strict frame, reply-ID, and embedded-ID
		// validation before any feedback is accepted.
		for (uint8_t bus = 0; bus < 2; ++bus) {
			if (g_can->driver.getIface(bus) == nullptr) {
				PX4_ERR("CAN%u interface unavailable", static_cast<unsigned>(bus + 1));
				return false;
			}
		}

		g_can_initialized = true;
	}

	updateParams();
	_limits = {_param_position_limit.get(), _param_velocity_limit.get(), _param_torque_limit.get()};

	if (!configurationValid()) {
		PX4_ERR("invalid GL40 parameter configuration");
		return false;
	}

	// GL40II drives may power up enabled. Establish a known non-driving state
	// before accepting any arming or setpoint state from the rest of PX4.
	sendSpecialAll(gl40ii::SpecialCommand::Disable);
	_last_disable_tx = hrt_absolute_time();
	ScheduleOnInterval(RUN_INTERVAL_US);
	return true;
}

uint8_t GL40IIDriver::activeMask() const
{
	return static_cast<uint8_t>(_param_active_mask.get()) & 0x3Fu;
}

float GL40IIDriver::motorSign(uint8_t rotor) const
{
	return (_param_reverse_mask.get() & (1 << rotor)) ? -1.f : 1.f;
}

bool GL40IIDriver::configurationValid() const
{
	return activeMask() != 0
	       && _limits.valid()
	       && PX4_ISFINITE(_param_ratio.get()) && _param_ratio.get() > 0.f
	       && PX4_ISFINITE(_param_kp.get()) && _param_kp.get() >= 0.f && _param_kp.get() <= 500.f
	       && PX4_ISFINITE(_param_kd.get()) && _param_kd.get() >= 0.f && _param_kd.get() <= 5.f
	       && _param_command_rate.get() >= 50 && _param_command_rate.get() <= 1'000
	       && PX4_ISFINITE(_param_setpoint_timeout.get()) && _param_setpoint_timeout.get() >= 0.02f
	       && PX4_ISFINITE(_param_feedback_timeout.get()) && _param_feedback_timeout.get() >= 0.01f;
}

bool GL40IIDriver::commandValid() const
{
	const uint8_t required = activeMask();

	if ((_tilt_setpoint.valid_mask & required) != required) {
		return false;
	}

	for (uint8_t rotor = 0; rotor < gl40ii::NUM_MOTORS; ++rotor) {
		if (required & (1u << rotor)) {
			const float motor_position = motorSign(rotor) * _param_ratio.get() * _tilt_setpoint.angle[rotor];

			// Never clamp this conversion: a range/scaling mismatch must inhibit the
			// motor rather than silently command a different joint angle.
			if (!PX4_ISFINITE(motor_position) || fabsf(motor_position) > _limits.position) {
				return false;
			}
		}
	}

	return true;
}

bool GL40IIDriver::feedbackFresh() const
{
	return (_valid_mask & activeMask()) == activeMask();
}

bool GL40IIDriver::enabledFeedback() const
{
	return (_enabled_mask & activeMask()) == activeMask();
}

bool GL40IIDriver::sendFrame(uint8_t bus, const gl40ii::Frame &frame)
{
	if (!g_can_initialized || bus >= 2 || frame.dlc > uavcan::CanFrame::MaxDataLen) {
		++_tx_error_count;
		return false;
	}

	uavcan::ICanIface *iface = g_can->driver.getIface(bus);

	if (iface == nullptr) {
		++_tx_error_count;
		return false;
	}

	const uavcan::CanFrame can_frame(frame.id, frame.data, frame.dlc);
	const uavcan::MonotonicTime deadline = uavcan_stm32h7::clock::getMonotonic()
			+ uavcan::MonotonicDuration::fromMSec(2);
	const int result = iface->send(can_frame, deadline, 0);

	if (result != 1) {
		++_tx_error_count;
		return false;
	}

	return true;
}

bool GL40IIDriver::sendSpecialAll(gl40ii::SpecialCommand command)
{
	bool success = true;

	for (uint8_t rotor = 0; rotor < gl40ii::NUM_MOTORS; ++rotor) {
		if (activeMask() & (1u << rotor)) {
			gl40ii::Frame frame{};
			success = gl40ii::packSpecial(gl40ii::Mode::MIT, rotor + 1, command, frame)
				  && sendFrame(gl40ii::busIndex(rotor), frame) && success;
		}
	}

	return success;
}

bool GL40IIDriver::sendMotorCommands()
{
	bool success = true;

	for (uint8_t rotor = 0; rotor < gl40ii::NUM_MOTORS; ++rotor) {
		if (activeMask() & (1u << rotor)) {
			gl40ii::Frame frame{};
			const float motor_position = motorSign(rotor) * _param_ratio.get() * _tilt_setpoint.angle[rotor];
			const bool packed = gl40ii::packMit(rotor + 1, _limits, motor_position, 0.f,
							 _param_kp.get(), _param_kd.get(), 0.f, frame);
			success = packed && sendFrame(gl40ii::busIndex(rotor), frame) && success;
		}
	}

	return success;
}

void GL40IIDriver::updateSubscriptions(hrt_abstime now)
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();
		_limits = {_param_position_limit.get(), _param_velocity_limit.get(), _param_torque_limit.get()};
	}

	vehicle_status_s vehicle_status{};

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}

	voliro_tilt_setpoint_s setpoint{};

	if (_tilt_setpoint_sub.update(&setpoint)) {
		_tilt_setpoint = setpoint;
		_last_setpoint_rx = now;
	}
}

void GL40IIDriver::drainRx(hrt_abstime now)
{
	for (uint8_t bus = 0; bus < 2; ++bus) {
		uavcan::ICanIface *iface = g_can->driver.getIface(bus);

		if (iface == nullptr) {
			continue;
		}

		while (true) {
			uavcan::CanFrame can_frame{};
			uavcan::MonotonicTime monotonic_timestamp = uavcan::MonotonicTime::fromUSec(0);
			uavcan::UtcTime utc_timestamp = uavcan::UtcTime::fromUSec(0);
			uavcan::CanIOFlags flags{0};
			const int result = iface->receive(can_frame, monotonic_timestamp, utc_timestamp, flags);

			if (result == 0) {
				break;
			}

			if (result < 0) {
				++_rx_error_count;
				break;
			}

			if (can_frame.isExtended() || can_frame.isRemoteTransmissionRequest()
			    || can_frame.isErrorFrame() || can_frame.dlc != 8) {
				++_invalid_rx_count;
				continue;
			}

			const uint16_t received_id = can_frame.id & uavcan::CanFrame::MaskStdID;
			int rotor_index = -1;

			for (uint8_t rotor = bus; rotor < gl40ii::NUM_MOTORS; rotor += 2) {
				if (received_id == gl40ii::masterId(rotor)) {
					rotor_index = rotor;
					break;
				}
			}

			if (rotor_index < 0 || !(activeMask() & (1u << rotor_index))) {
				++_invalid_rx_count;
				continue;
			}

			gl40ii::Frame frame{};
			frame.id = received_id;
			frame.dlc = can_frame.dlc;
			memcpy(frame.data, can_frame.data, sizeof(frame.data));
			gl40ii::Feedback feedback{};

			if (!gl40ii::parseFeedback(frame, gl40ii::masterId(rotor_index), rotor_index + 1,
							_limits, feedback)) {
				++_invalid_rx_count;
				continue;
			}

			const float sign = motorSign(rotor_index);
			_joint_angle[rotor_index] = sign * feedback.position / _param_ratio.get();
			_joint_velocity[rotor_index] = sign * feedback.velocity / _param_ratio.get();
			_motor_torque[rotor_index] = feedback.torque;
			_driver_temperature[rotor_index] = feedback.driver_temperature;
			_motor_temperature[rotor_index] = feedback.motor_temperature;
			_motor_state[rotor_index] = feedback.state;
			_last_rx[rotor_index] = now;
			++_rx_count[rotor_index];
			_feedback_dirty = true;
		}
	}
}

void GL40IIDriver::updateMasks(hrt_abstime now)
{
	_valid_mask = 0;
	_enabled_mask = 0;
	_fault_mask = 0;
	_stale_mask = 0;
	const uint8_t active = activeMask();
	const hrt_abstime timeout_us = static_cast<hrt_abstime>(_param_feedback_timeout.get() * 1e6f);

	for (uint8_t rotor = 0; rotor < gl40ii::NUM_MOTORS; ++rotor) {
		const uint8_t bit = 1u << rotor;

		if (!(active & bit)) {
			continue;
		}

		const bool fresh = _last_rx[rotor] != 0 && now - _last_rx[rotor] <= timeout_us;

		if (!fresh) {
			_stale_mask |= bit;
			continue;
		}

		_valid_mask |= bit;

		if (_motor_state[rotor] == voliro_tilt_feedback_s::STATE_ENABLED) {
			_enabled_mask |= bit;
		}

		if (_motor_state[rotor] >= voliro_tilt_feedback_s::STATE_OVERVOLTAGE) {
			_fault_mask |= bit;
		}
	}
}

void GL40IIDriver::publishFeedback(hrt_abstime now, bool force)
{
	if (!force && !_feedback_dirty && now - _last_feedback_publish < FEEDBACK_PUBLISH_INTERVAL_US) {
		return;
	}

	voliro_tilt_feedback_s feedback{};
	feedback.timestamp = now;
	feedback.timestamp_sample = now;
	feedback.active_mask = activeMask();
	feedback.valid_mask = _valid_mask;
	feedback.enabled_mask = _enabled_mask;
	feedback.fault_mask = _fault_mask;
	feedback.stale_mask = _stale_mask;

	for (uint8_t rotor = 0; rotor < gl40ii::NUM_MOTORS; ++rotor) {
		feedback.angle[rotor] = _joint_angle[rotor];
		feedback.angular_velocity[rotor] = _joint_velocity[rotor];
		feedback.motor_torque[rotor] = _motor_torque[rotor];
		feedback.driver_temperature[rotor] = _driver_temperature[rotor];
		feedback.motor_temperature[rotor] = _motor_temperature[rotor];
		feedback.state[rotor] = _motor_state[rotor];
		feedback.rx_count[rotor] = _rx_count[rotor];
	}

	_tilt_feedback_pub.publish(feedback);
	_feedback_dirty = false;
	_last_feedback_publish = now;
}

void GL40IIDriver::enterDisabled(hrt_abstime now)
{
	_control_state = ControlState::Disabled;
	sendSpecialAll(gl40ii::SpecialCommand::Disable);
	_last_disable_tx = now;
}

void GL40IIDriver::Run()
{
	if (should_exit()) {
		enterDisabled(hrt_absolute_time());
		publishFeedback(hrt_absolute_time(), true);
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);
	const hrt_abstime now = hrt_absolute_time();
	updateSubscriptions(now);
	drainRx(now);
	updateMasks(now);

	const hrt_abstime setpoint_timeout_us = static_cast<hrt_abstime>(_param_setpoint_timeout.get() * 1e6f);
	const bool setpoint_fresh = _last_setpoint_rx != 0 && now - _last_setpoint_rx <= setpoint_timeout_us;
	const bool control_requested = _param_enable.get() && _armed && setpoint_fresh
				       && configurationValid() && commandValid()
				       && feedbackFresh() && _fault_mask == 0;

	switch (_control_state) {
	case ControlState::Disabled:
		if (control_requested) {
			_control_state = ControlState::Enabling;
			sendSpecialAll(gl40ii::SpecialCommand::Enable);
			_last_tx = now;

		} else if (now - _last_disable_tx >= DISABLE_INTERVAL_US) {
			sendSpecialAll(gl40ii::SpecialCommand::Disable);
			_last_disable_tx = now;
		}
		break;

	case ControlState::Enabling:
		if (!control_requested) {
			enterDisabled(now);

		} else if (enabledFeedback()) {
			_control_state = ControlState::Active;
			_last_tx = 0;

		} else if (now - _last_tx >= ENABLE_INTERVAL_US) {
			sendSpecialAll(gl40ii::SpecialCommand::Enable);
			_last_tx = now;
		}
		break;

	case ControlState::Active: {
		const hrt_abstime command_interval_us = 1'000'000u / _param_command_rate.get();

		if (!control_requested || !enabledFeedback()) {
			enterDisabled(now);

		} else if (now - _last_tx >= command_interval_us) {
			if (!sendMotorCommands()) {
				enterDisabled(now);

			} else {
				_last_tx = now;
			}
		}
		break;
	}
	}

	publishFeedback(now);
	perf_end(_cycle_perf);
}

const char *GL40IIDriver::stateName() const
{
	switch (_control_state) {
	case ControlState::Disabled: return "disabled";
	case ControlState::Enabling: return "enabling";
	case ControlState::Active: return "active";
	}

	return "unknown";
}

int GL40IIDriver::print_status()
{
	PX4_INFO("state: %s, armed: %s, GL40_EN: %s", stateName(), _armed ? "yes" : "no",
		 _param_enable.get() ? "yes" : "no");
	PX4_INFO("CAN1 rotors 1/3/5, CAN2 rotors 2/4/6, 1 Mbit/s");
	PX4_INFO("active=0x%02x valid=0x%02x enabled=0x%02x fault=0x%02x stale=0x%02x",
		 activeMask(), _valid_mask, _enabled_mask, _fault_mask, _stale_mask);
	PX4_INFO("rate=%" PRId32 " Hz ratio=%.4f Pmax=%.3f Vmax=%.1f Tmax=%.2f reverse=0x%02x",
		 _param_command_rate.get(), static_cast<double>(_param_ratio.get()),
		 static_cast<double>(_limits.position), static_cast<double>(_limits.velocity),
		 static_cast<double>(_limits.torque), static_cast<unsigned>(_param_reverse_mask.get() & 0x3F));
	PX4_INFO("tx errors=%" PRIu32 ", rx errors=%" PRIu32 ", invalid rx=%" PRIu32,
		 _tx_error_count, _rx_error_count, _invalid_rx_count);

	for (uint8_t rotor = 0; rotor < gl40ii::NUM_MOTORS; ++rotor) {
		if (activeMask() & (1u << rotor)) {
			const double age_ms = _last_rx[rotor] == 0 ? static_cast<double>(INFINITY)
					      : static_cast<double>(hrt_elapsed_time(&_last_rx[rotor])) * 1e-3;
			PX4_INFO("rotor %u CAN%u slave=%u master=0x%03x state=%u age=%.1f ms rx=%" PRIu32,
				 static_cast<unsigned>(rotor + 1), static_cast<unsigned>(gl40ii::busIndex(rotor) + 1),
				 static_cast<unsigned>(rotor + 1), static_cast<unsigned>(gl40ii::masterId(rotor)),
				 static_cast<unsigned>(_motor_state[rotor]), age_ms, _rx_count[rotor]);
		}
	}

	perf_print_counter(_cycle_perf);
	return 0;
}

int GL40IIDriver::task_spawn(int argc, char *argv[])
{
	auto *instance = new GL40IIDriver();

	if (instance != nullptr) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int GL40IIDriver::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GL40IIDriver::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Dual-CAN GL40II tilt-actuator driver for the Voliro variable-tilt hexarotor.
The driver owns CAN1 and CAN2 at 1 Mbit/s; do not run DroneCAN concurrently.
It always polls configured drives with disable frames while GL40_EN is false or
the vehicle is disarmed. Motion additionally requires fresh typed setpoints,
fresh feedback, enabled-state feedback, and no drive faults.

Rotor mapping: CAN1 carries rotors 1/3/5 and CAN2 carries rotors 2/4/6.
Slave IDs are 1..6 and persistent master response IDs are 0x011..0x016.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("gl40ii", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int gl40ii_main(int argc, char *argv[])
{
	return ModuleBase::main(GL40IIDriver::desc, argc, argv);
}
