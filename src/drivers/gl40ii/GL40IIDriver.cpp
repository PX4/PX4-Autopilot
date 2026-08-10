#include "GL40IIDriver.hpp"
#include "GL40IIBench.hpp"

#include <board_config.h>
#include <cmath>
#include <cstdlib>
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
		_motor_position[rotor] = NAN;
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

bool GL40IIDriver::rotorEnabledFeedback(uint8_t rotor) const
{
	return rotor < gl40ii::NUM_MOTORS && (_enabled_mask & (1u << rotor));
}

bool GL40IIDriver::benchSafetyValid() const
{
	return _vehicle_status_received && !_armed
	       && !_param_enable.get() && !_param_vola_enable.get() && !_param_vctrl_enable.get()
	       && _bench_required_mask != 0 && activeMask() == _bench_required_mask
	       && configurationValid() && feedbackFresh() && _fault_mask == 0;
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

bool GL40IIDriver::sendSpecialRotor(uint8_t rotor, gl40ii::SpecialCommand command)
{
	if (rotor >= gl40ii::NUM_MOTORS) {
		return false;
	}

	gl40ii::Frame frame{};
	return gl40ii::packSpecial(gl40ii::Mode::MIT, rotor + 1, command, frame)
	       && sendFrame(gl40ii::busIndex(rotor), frame);
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

bool GL40IIDriver::sendBenchCommand(uint8_t rotor, float motor_position)
{
	if (rotor >= gl40ii::NUM_MOTORS) {
		return false;
	}

	gl40ii::Frame frame{};
	const float kp = fminf(_param_kp.get(), BENCH_KP_MAX);
	const float kd = fminf(_param_kd.get(), BENCH_KD_MAX);

	return gl40ii::packMit(rotor + 1, _limits, motor_position, 0.f, kp, kd, 0.f, frame)
	       && sendFrame(gl40ii::busIndex(rotor), frame);
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
		_vehicle_status_received = true;
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
			_motor_position[rotor_index] = feedback.position;
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

void GL40IIDriver::finishBenchTest(hrt_abstime now, bool success, const char *reason)
{
	// Address the selected rotor directly so a runtime GL40_MASK change cannot
	// suppress its disable frame. Follow with a best-effort disable to every
	// currently configured drive as an additional sequence-mode safeguard.
	sendSpecialRotor(_bench_rotor, gl40ii::SpecialCommand::Disable);
	sendSpecialAll(gl40ii::SpecialCommand::Disable);
	_control_state = ControlState::Disabled;
	_last_disable_tx = now;
	_bench_state = BenchState::Idle;
	_bench_state_start = 0;
	_bench_test_start = 0;
	_bench_target_position = NAN;
	_bench_required_mask = 0;
	_bench_active.store(false);

	if (success) {
		PX4_INFO("bench complete and all configured drives disabled: %s", reason);

	} else {
		PX4_ERR("bench aborted and all configured drives disabled: %s", reason);
	}
}

bool GL40IIDriver::startBenchRotor(hrt_abstime now)
{
	if (_bench_rotor >= gl40ii::NUM_MOTORS || !benchSafetyValid() || _enabled_mask != 0
	    || _motor_state[_bench_rotor] != voliro_tilt_feedback_s::STATE_DISABLED
	    || !PX4_ISFINITE(_motor_position[_bench_rotor])) {
		return false;
	}

	_bench_delta = _bench_all_sequence ? static_cast<float>(_bench_leg) * _bench_amplitude : _bench_delta;
	const bool range_valid = _bench_all_sequence
				 ? gl40ii::benchBidirectionalRangeValid(_motor_position[_bench_rotor], _bench_amplitude,
						 _limits.position, BENCH_RANGE_MARGIN_RAD)
				 : gl40ii::benchRangeValid(_motor_position[_bench_rotor], _bench_delta,
						 _limits.position, BENCH_RANGE_MARGIN_RAD);

	if (!range_valid) {
		return false;
	}

	_bench_start_position = _motor_position[_bench_rotor];
	_bench_target_position = _bench_start_position;
	_bench_phase_duration_s = gl40ii::minimumJerkDuration(
					  fabsf(_bench_delta), BENCH_PEAK_RATE_RAD_S, BENCH_MIN_PHASE_DURATION_S);

	if (!PX4_ISFINITE(_bench_phase_duration_s)) {
		return false;
	}

	_bench_state = BenchState::AwaitEnable;
	_bench_state_start = now;
	_last_tx = now;

	if (!sendSpecialRotor(_bench_rotor, gl40ii::SpecialCommand::Enable)) {
		return false;
	}

	PX4_WARN("BENCH START rotor %u: start=%.3f delta=%+.3f rad (%+.3f rev), phase=%.2f s; keep cutoff ready",
		 static_cast<unsigned>(_bench_rotor + 1), (double)_bench_start_position, (double)_bench_delta,
		 (double)(_bench_delta / TWO_PI_F), (double)_bench_phase_duration_s);
	return true;
}

void GL40IIDriver::processBenchRequest(hrt_abstime now)
{
	if (_bench_abort_requested.load()) {
		_bench_abort_requested.store(false);
		_bench_request_pending.store(false);

		if (_bench_state != BenchState::Idle) {
			finishBenchTest(now, false, "operator stop request");
		}

		return;
	}

	if (!_bench_request_pending.load()) {
		return;
	}

	_bench_request_pending.store(false);

	if (_bench_state != BenchState::Idle) {
		PX4_ERR("bench rejected: another bench test is active");
		return;
	}

	if (_control_state != ControlState::Disabled) {
		PX4_ERR("bench rejected: normal control is not disabled");
		return;
	}

	_bench_all_sequence = _bench_request_all.load();
	_bench_required_mask = _bench_all_sequence ? 0x3Fu : 0x01u;
	_bench_rotor = 0;
	_bench_leg = 1;
	const int32_t millirevolutions = _bench_request_millirevolutions.load();
	const float turns = static_cast<float>(millirevolutions) * 0.001f;
	_bench_amplitude = fabsf(turns * TWO_PI_F);
	_bench_delta = turns * TWO_PI_F;

	if (!benchSafetyValid()) {
		PX4_ERR("bench rejected: require disarmed, GL40/VOLA/VCTRL off, expected mask, fresh fault-free feedback");
		return;
	}

	if (_enabled_mask != 0) {
		PX4_ERR("bench rejected: one or more drives already report enabled (mask=0x%02x)", _enabled_mask);
		return;
	}

	for (uint8_t rotor = 0; rotor < gl40ii::NUM_MOTORS; ++rotor) {
		if (!(_bench_required_mask & (1u << rotor))) {
			continue;
		}

		if (_motor_state[rotor] != voliro_tilt_feedback_s::STATE_DISABLED
		    || !PX4_ISFINITE(_motor_position[rotor])) {
			PX4_ERR("bench rejected: rotor %u is not disabled with finite feedback",
				static_cast<unsigned>(rotor + 1));
			return;
		}

		const bool range_valid = _bench_all_sequence
					 ? gl40ii::benchBidirectionalRangeValid(_motor_position[rotor], _bench_amplitude,
							 _limits.position, BENCH_RANGE_MARGIN_RAD)
					 : gl40ii::benchRangeValid(_motor_position[rotor], _bench_delta,
							 _limits.position, BENCH_RANGE_MARGIN_RAD);

		if (!range_valid) {
			PX4_ERR("bench rejected: rotor %u start %.3f cannot fit requested path within Pmax %.3f and %.2f margin",
				static_cast<unsigned>(rotor + 1), (double)_motor_position[rotor],
				(double)_limits.position, (double)BENCH_RANGE_MARGIN_RAD);
			return;
		}
	}

	_bench_test_start = now;
	_bench_active.store(true);

	if (!startBenchRotor(now)) {
		finishBenchTest(now, false, "initial rotor enable or preflight failed");
		return;
	}

	if (_bench_all_sequence) {
		PX4_WARN("SIX-MOTOR BENCH sequence accepted: each rotor +1 rev/back then -1 rev/back, one at a time");
	}
}

void GL40IIDriver::runBenchTest(hrt_abstime now)
{
	if (_bench_state == BenchState::Idle) {
		return;
	}

	if (!benchSafetyValid()) {
		finishBenchTest(now, false, "safety gate changed");
		return;
	}

	const hrt_abstime timeout = _bench_all_sequence ? BENCH_SEQUENCE_TIMEOUT_US : BENCH_SINGLE_TIMEOUT_US;

	if (now - _bench_test_start > timeout) {
		finishBenchTest(now, false, "total timeout");
		return;
	}

	const uint8_t selected_bit = 1u << _bench_rotor;

	if (_enabled_mask & ~selected_bit) {
		finishBenchTest(now, false, "an unselected drive reported enabled");
		return;
	}

	if (_bench_state == BenchState::AwaitDisable) {
		if (!rotorEnabledFeedback(_bench_rotor)) {
			PX4_INFO("bench rotor %u complete and disabled", static_cast<unsigned>(_bench_rotor + 1));

			if (!_bench_all_sequence || _bench_rotor + 1 >= gl40ii::NUM_MOTORS) {
				finishBenchTest(now, true, _bench_all_sequence ? "six-motor bidirectional sequence finished"
						: "out-and-back trajectory finished");
				return;
			}

			++_bench_rotor;
			_bench_leg = 1;
			_bench_delta = _bench_amplitude;
			_bench_target_position = NAN;
			_bench_state = BenchState::InterMotorDwell;
			_bench_state_start = now;
			return;
		}

		if (now - _bench_state_start > BENCH_DISABLE_TIMEOUT_US) {
			finishBenchTest(now, false, "disable feedback timeout");
			return;
		}

		if (now - _last_tx >= DISABLE_INTERVAL_US) {
			if (!sendSpecialRotor(_bench_rotor, gl40ii::SpecialCommand::Disable)) {
				finishBenchTest(now, false, "disable retransmission failed");
				return;
			}

			_last_tx = now;
		}

		return;
	}

	if (_bench_state == BenchState::InterMotorDwell) {
		if (_enabled_mask != 0) {
			finishBenchTest(now, false, "drive enabled during inter-motor dwell");
			return;
		}

		if (now - _bench_state_start >= BENCH_INTER_MOTOR_DWELL_US && !startBenchRotor(now)) {
			finishBenchTest(now, false, "next rotor enable or preflight failed");
		}

		return;
	}

	if (_bench_state == BenchState::AwaitEnable) {
		if (rotorEnabledFeedback(_bench_rotor)) {
			_bench_state = BenchState::Settle;
			_bench_state_start = now;
			_bench_target_position = _bench_start_position;
			_last_tx = 0;

		} else if (now - _bench_state_start > BENCH_ENABLE_TIMEOUT_US) {
			finishBenchTest(now, false, "enable feedback timeout");
			return;

		} else if (now - _last_tx >= ENABLE_INTERVAL_US) {
			if (!sendSpecialRotor(_bench_rotor, gl40ii::SpecialCommand::Enable)) {
				finishBenchTest(now, false, "enable retransmission failed");
				return;
			}

			_last_tx = now;
		}

		if (_bench_state == BenchState::AwaitEnable) {
			return;
		}
	}

	if (!rotorEnabledFeedback(_bench_rotor)) {
		finishBenchTest(now, false, "drive left enabled state");
		return;
	}

	if (!PX4_ISFINITE(_motor_position[_bench_rotor])
	    || fabsf(_motor_position[_bench_rotor] - _bench_target_position) > BENCH_MAX_TRACKING_ERROR_RAD) {
		finishBenchTest(now, false, "tracking error exceeded 0.5 motor rad");
		return;
	}

	const hrt_abstime state_elapsed = now - _bench_state_start;

	switch (_bench_state) {
	case BenchState::Settle:
		_bench_target_position = _bench_start_position;

		if (state_elapsed >= BENCH_SETTLE_US) {
			_bench_state = BenchState::Outbound;
			_bench_state_start = now;
		}

		break;

	case BenchState::Outbound: {
			const float phase = static_cast<float>(state_elapsed) * 1e-6f / _bench_phase_duration_s;
			_bench_target_position = _bench_start_position + _bench_delta * gl40ii::minimumJerk(phase);

			if (phase >= 1.f) {
				_bench_target_position = _bench_start_position + _bench_delta;
				_bench_state = BenchState::EndpointDwell;
				_bench_state_start = now;
			}

			break;
		}

	case BenchState::EndpointDwell:
		_bench_target_position = _bench_start_position + _bench_delta;

		if (state_elapsed >= BENCH_DWELL_US) {
			_bench_state = BenchState::Return;
			_bench_state_start = now;
		}

		break;

	case BenchState::Return: {
			const float phase = static_cast<float>(state_elapsed) * 1e-6f / _bench_phase_duration_s;
			_bench_target_position = _bench_start_position + _bench_delta * (1.f - gl40ii::minimumJerk(phase));

			if (phase >= 1.f) {
				_bench_target_position = _bench_start_position;
				_bench_state = BenchState::ReturnDwell;
				_bench_state_start = now;
			}

			break;
		}

	case BenchState::ReturnDwell:
		_bench_target_position = _bench_start_position;

		if (state_elapsed >= BENCH_DWELL_US) {
			if (_bench_all_sequence && _bench_leg > 0) {
				_bench_leg = -1;
				_bench_delta = -_bench_amplitude;
				_bench_state = BenchState::Outbound;
				_bench_state_start = now;
				PX4_WARN("bench rotor %u starting negative one-revolution leg",
					 static_cast<unsigned>(_bench_rotor + 1));

			} else {
				if (!sendSpecialRotor(_bench_rotor, gl40ii::SpecialCommand::Disable)) {
					finishBenchTest(now, false, "final disable transmission failed");
					return;
				}

				_bench_state = BenchState::AwaitDisable;
				_bench_state_start = now;
				_last_tx = now;
				return;
			}
		}

		break;

	case BenchState::Idle:
	case BenchState::AwaitEnable:
	case BenchState::AwaitDisable:
	case BenchState::InterMotorDwell:
		return;
	}

	const hrt_abstime command_interval_us = 1'000'000u / _param_command_rate.get();

	if (now - _last_tx >= command_interval_us) {
		if (!sendBenchCommand(_bench_rotor, _bench_target_position)) {
			finishBenchTest(now, false, "position command transmission failed");
			return;
		}

		_last_tx = now;
	}
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
	processBenchRequest(now);

	if (_bench_state != BenchState::Idle) {
		runBenchTest(now);
		publishFeedback(now);
		perf_end(_cycle_perf);
		return;
	}

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

const char *GL40IIDriver::benchStateName() const
{
	switch (_bench_state) {
	case BenchState::Idle: return "idle";

	case BenchState::AwaitEnable: return "await-enable";

	case BenchState::Settle: return "settle";

	case BenchState::Outbound: return "outbound";

	case BenchState::EndpointDwell: return "endpoint-dwell";

	case BenchState::Return: return "return";

	case BenchState::ReturnDwell: return "return-dwell";

	case BenchState::AwaitDisable: return "await-disable";

	case BenchState::InterMotorDwell: return "inter-motor-dwell";
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
	PX4_INFO("bench=%s mode=%s rotor=%u leg=%s start=%.3f target=%.3f delta=%+.3f motor rad",
		 benchStateName(), _bench_all_sequence ? "all-bidirectional" : "single",
		 static_cast<unsigned>(_bench_rotor + 1), _bench_leg > 0 ? "positive" : "negative",
		 (double)_bench_start_position, (double)_bench_target_position, (double)_bench_delta);

	for (uint8_t rotor = 0; rotor < gl40ii::NUM_MOTORS; ++rotor) {
		if (activeMask() & (1u << rotor)) {
			const double age_ms = _last_rx[rotor] == 0 ? static_cast<double>(INFINITY)
					      : static_cast<double>(hrt_elapsed_time(&_last_rx[rotor])) * 1e-3;
			PX4_INFO("rotor %u CAN%u slave=%u master=0x%03x state=%u motor=%.3f joint=%.3f age=%.1f ms rx=%" PRIu32,
				 static_cast<unsigned>(rotor + 1), static_cast<unsigned>(gl40ii::busIndex(rotor) + 1),
				 static_cast<unsigned>(rotor + 1), static_cast<unsigned>(gl40ii::masterId(rotor)),
				 static_cast<unsigned>(_motor_state[rotor]), (double)_motor_position[rotor],
				 (double)_joint_angle[rotor], age_ms, _rx_count[rotor]);
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
	if (argc >= 1 && strcmp(argv[0], "test") == 0) {
		if (!is_running(desc)) {
			PX4_ERR("gl40ii is not running");
			return PX4_ERROR;
		}

		GL40IIDriver *instance = get_instance<GL40IIDriver>(desc);

		if (instance == nullptr) {
			PX4_ERR("driver instance unavailable");
			return PX4_ERROR;
		}

		if (argc == 2 && strcmp(argv[1], "stop") == 0) {
			instance->_bench_request_pending.store(false);
			instance->_bench_abort_requested.store(true);
			PX4_WARN("bench stop requested; use the physical power cutoff for any unexpected motion");
			return PX4_OK;
		}

		if (argc == 2 && strcmp(argv[1], "all") == 0) {
			if (instance->_bench_request_pending.load() || instance->_bench_active.load()) {
				PX4_ERR("a bench request or test is already active");
				return PX4_ERROR;
			}

			instance->_bench_request_all.store(true);
			instance->_bench_request_millirevolutions.store(1000);
			instance->_bench_request_pending.store(true);
			PX4_WARN("queued six-motor sequence: each rotor +1 rev/back then -1 rev/back; about 5 minutes");
			PX4_WARN("only one drive moves at a time; keep the physical power cutoff ready");
			return PX4_OK;
		}

		if (argc != 2) {
			return print_usage("test requires signed rotor-1 revolutions, 'all', or 'stop'");
		}

		char *end = nullptr;
		const float turns = strtof(argv[1], &end);

		if (end == argv[1] || end == nullptr || *end != '\0' || !PX4_ISFINITE(turns)
		    || fabsf(turns) < BENCH_MIN_TURNS || fabsf(turns) > BENCH_MAX_TURNS) {
			return print_usage("test turns must be in [-2.0,-0.01] or [0.01,2.0]");
		}

		if (instance->_bench_request_pending.load() || instance->_bench_active.load()) {
			PX4_ERR("a bench request or test is already active");
			return PX4_ERROR;
		}

		instance->_bench_request_all.store(false);
		instance->_bench_request_millirevolutions.store(static_cast<int32_t>(lroundf(turns * 1000.f)));
		instance->_bench_request_pending.store(true);
		PX4_WARN("queued relative rotor-1 bare-motor sweep: %+.3f rev out, then return; keep power cutoff ready",
			 (double)turns);
		return PX4_OK;
	}

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

The signed `test` command is a deliberately restricted rotor-1 bare-motor
bench mode requiring GL40_MASK=1. `gl40ii test all` instead requires
GL40_MASK=63 and preflights all six drives before moving rotor 1 through rotor
6 sequentially. Each rotor moves +1 raw motor revolution and returns, then -1
revolution and returns, before it is disabled and the next rotor is enabled.
Only one drive is enabled at a time. Both modes require PX4 disarmed,
GL40_EN/VOLA_EN/VCTRL_EN zero, fresh fault-free feedback, disabled-state
feedback, and complete trajectories inside the configured raw position range.
Use `gl40ii test stop` to request an immediate disable, but retain a physical
motor-power cutoff.

Rotor mapping: CAN1 carries rotors 1/3/5 and CAN2 carries rotors 2/4/6.
Slave IDs are 1..6 and persistent master response IDs are 0x011..0x016.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("gl40ii", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Run a bounded bare-motor bench trajectory");
	PRINT_MODULE_USAGE_ARG("<turns|all|stop>", "Rotor-1 signed revolutions, six-motor +/-1-rev sequence, or stop", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int gl40ii_main(int argc, char *argv[])
{
	return ModuleBase::main(GL40IIDriver::desc, argc, argv);
}
