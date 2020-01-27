/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

#include <stdint.h>

#if 0 // for debugging, prints event messages to console each time arming & health checks run (not necessarily change)
#include <px4_platform_common/log.h>
#define CONSOLE_PRINT_ARMING_CHECK_EVENT(log_level, id, str) PX4_INFO_RAW("Health/Arming Event 0x%08x: %s\n", id, str)
#else
#define CONSOLE_PRINT_ARMING_CHECK_EVENT(log_level, id, str)
#endif

using namespace time_literals;

class HealthAndArmingChecks;

enum class ModeCategory : uint8_t {
	None = 0, ///< Using ModeCategory = None means arming is still possible (optional check)

	Current = (1 << 0), ///< Check applies to current navigation mode only (avoid using it if possible)

//	Manual = (1<<1), // currently unused, enable only when needed
//	Rate = (1<<2),
//	Attitude = (1<<3),

	Altitude = (1 << 4),
	Position = (1 << 5),
	AllManualModes = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5),

	Autonomous = (1 << 6),
	Mission = (1 << 7),
	AllPositionModes = (1 << 5) | (1 << 6) | (1 << 7),

	All = 0xff
};

static_assert((uint8_t)ModeCategory::Current == (uint8_t)events::common::enums::navigation_mode_category_t::current,
	      "Enum def must match");
static_assert((uint8_t)ModeCategory::Altitude == (uint8_t)events::common::enums::navigation_mode_category_t::altitude,
	      "Enum def must match");
static_assert((uint8_t)ModeCategory::Position == (uint8_t)events::common::enums::navigation_mode_category_t::position,
	      "Enum def must match");
static_assert((uint8_t)ModeCategory::Autonomous == (uint8_t)
	      events::common::enums::navigation_mode_category_t::autonomous, "Enum def must match");
static_assert((uint8_t)ModeCategory::Mission == (uint8_t)events::common::enums::navigation_mode_category_t::mission,
	      "Enum def must match");

static inline ModeCategory operator|(ModeCategory a, ModeCategory b)
{
	return static_cast<ModeCategory>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

static inline ModeCategory operator&(ModeCategory a, ModeCategory b)
{
	return static_cast<ModeCategory>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}

static inline ModeCategory operator~(ModeCategory a)
{
	return static_cast<ModeCategory>(~static_cast<uint8_t>(a));
}

enum class HealthComponentIndex : uint8_t {
	sensor_imu = 0, ///< IMU
	sensor_absolute_pressure = 1, ///< Absolute pressure
	sensor_differential_pressure = 2, ///< Differential pressure
	sensor_gps = 3, ///< GPS
	sensor_optical_flow = 4, ///< Optical flow
	sensor_vision_position = 5, ///< Vision position estimate
	sensor_distance = 6, ///< Distance sensor
	manual_control_input = 7, ///< RC or virtual joystick input
	motors_escs = 8, ///< Motors/ESCs
	utm = 9, ///< UTM
	logging = 10, ///< Logging
	battery = 11, ///< Battery
	communication_links = 12, ///< Communication links
	rate_controller = 13, ///< Rate controller
	attitude_controller = 14, ///< Attitude controller
	position_controller = 15, ///< Position controller
	attitude_estimate = 16, ///< Attitude estimate
	local_position_estimate = 17, ///< Local position estimate
	mission = 18, ///< Mission
	avoidance = 19, ///< Avoidance
	system = 20, ///< System
	camera = 21, ///< Camera
	gimbal = 22, ///< Gimbal
	payload = 23, ///< Payload
	global_position_estimate = 24, ///< Global position estimate
	storage = 25, ///< Storage
};

static_assert((uint64_t)events::common::enums::health_component_t::storage ==
	      (uint64_t)(1 << (int)HealthComponentIndex::storage), "enum def mismatch");

/**
 * @class Context
 * Provides commonly used information for health and arming checks
 */
class Context
{
public:
	Context(const vehicle_status_flags_s &status_flags, const vehicle_status_s &status)
		: _status_flags(status_flags), _status(status)
	{}

	~Context() = default;

	bool isArmed() const { return _status.arming_state == vehicle_status_s::ARMING_STATE_ARMED; }

	const vehicle_status_flags_s &status_flags() const { return _status_flags; }
	const vehicle_status_s &status() const { return _status; }

private:
	const vehicle_status_flags_s &_status_flags;
	const vehicle_status_s &_status;
};


/**
 * @class Report
 * Keeps track of health and arming report and reports the results whenever something changes.
 */
class Report
{
public:

	struct HealthResults {
		HealthResults() { reset(); }

		events::common::enums::health_component_t is_present;
		events::common::enums::health_component_t error;
		events::common::enums::health_component_t warning;

		void reset()
		{
			is_present = {};
			error = {};
			warning = {};
		}
		bool operator!=(const HealthResults &other)
		{
			return is_present != other.is_present || error != other.error ||
			       warning != other.warning;
		}
	};

	struct ArmingCheckResults {
		ArmingCheckResults() { reset(); }

		events::common::enums::health_component_t error;
		events::common::enums::health_component_t warning;

		ModeCategory can_arm;

		bool valid; ///< whether can_arm is valid, i.e. can be used

		void setArmBit(ModeCategory mode)
		{
			can_arm = can_arm | mode;
		}
		void clearArmBit(ModeCategory mode)
		{
			can_arm = (ModeCategory)((uint8_t)can_arm & ~(uint8_t)mode);
		}

		void reset()
		{
			error = {};
			warning = {};
			can_arm = (ModeCategory)0xff; // bits are cleared for failed checks
			valid = false;
		}
		bool operator!=(const ArmingCheckResults &other)
		{
			return error != other.error || warning != other.warning ||
			       can_arm != other.can_arm || valid != other.valid;
		}
	};

	Report() = default;
	~Report() = default;

	/**
	 * Whether arming is possible for the current navigation mode
	 */
	bool canArm() const { return _results[_current_result].arming_checks.valid && (uint8_t)(_results[_current_result].arming_checks.can_arm & ModeCategory::Current) != 0; }


	/**
	 * Report a health failure. A health issue generally refers to a hardware issue, independent from environment
	 * or e.g. calibration.
	 */
	template<typename... Args>
	void healthFailure(ModeCategory required_modes, HealthComponentIndex component, uint32_t event_id,
			   const char *message, events::Log log_level, Args... args);

	void healthFailure(ModeCategory required_modes, HealthComponentIndex component, uint32_t event_id,
			   const char *message, events::Log log_level);

	void setIsPresent(HealthComponentIndex component);


	/**
	 * Report an arming check failure. If required_modes is None, arming is still possible.
	 */
	template<typename... Args>
	void armingCheckFailure(ModeCategory required_modes, HealthComponentIndex component, uint32_t event_id,
				const char *message, events::Log log_level, Args... args);

	void armingCheckFailure(ModeCategory required_modes, HealthComponentIndex component, uint32_t event_id,
				const char *message, events::Log log_level);


	const HealthResults &healthResults() const { return _results[_current_result].health; }
	const ArmingCheckResults &armingCheckResults() const { return _results[_current_result].arming_checks; }
private:

	/**
	 * Stores all results, and used to compare against results from the previous run,
	 * to know if we need to report. Note that only changed event arguments will not
	 * trigger reporting, which should generally be desirable
	 * (e.g. to avoid frequent updates due to changing floats).
	 */
	struct Results {
		Results() { reset(); }

		HealthResults health;
		ArmingCheckResults arming_checks;
		int num_events;
		uint32_t event_id_hash; ///< Simple hash over all current event ID's
		uint8_t nav_state;

		void reset() { health.reset(); arming_checks.reset(); num_events = 0; event_id_hash = 0; nav_state = 0; }

		bool operator!=(const Results &other)
		{
			return health != other.health || arming_checks != other.arming_checks ||
			       num_events != other.num_events || event_id_hash != other.event_id_hash ||
			       nav_state != other.nav_state;
		}
	};

	struct __attribute__((__packed__)) EventBufferHeader {
		uint8_t size; ///< arguments size
		uint32_t id;
		uint8_t log_level;
	};

	void healthFailure(ModeCategory required_modes, HealthComponentIndex component, events::Log log_level);
	void armingCheckFailure(ModeCategory required_modes, HealthComponentIndex component, events::Log log_level);

	template<typename... Args>
	bool addEvent(uint32_t event_id, const char *message, events::Log log_level, Args... args);

	ModeCategory applyCurrentNavState(ModeCategory modes) const;

	friend class HealthAndArmingChecks;
	/**
	 * Reset current results.
	 * The calling order needs to be:
	 * - reset()
	 * - run all checks
	 * - finalize()
	 * - report() (which can be called independently as well)
	 */
	void reset(uint8_t nav_state);
	void finalize();

	void report(bool force);

	static constexpr hrt_abstime min_reporting_interval{2_s};

	/// event buffer: stores current events + arguments.
	/// Since the amount of extra arguments varies, 4 bytes is used here as estimate
	uint8_t _event_buffer[(event_s::ORB_QUEUE_LENGTH - 2) * (sizeof(EventBufferHeader) + 1 + 1 + 4)];
	int _next_buffer_idx{0};
	bool _buffer_overflowed{false};

	bool _already_reported{false};
	bool _had_unreported_difference{false}; ///< true if there was a difference not reported yet (due to rate limitation)
	hrt_abstime _last_report{0};

	Results _results[2]; ///< Previous and current results to check for changes
	int _current_result{0};
};

template<typename... Args>
void Report::healthFailure(ModeCategory required_modes, HealthComponentIndex component, uint32_t event_id,
			   const char *message, events::Log log_level, Args... args)
{
	required_modes = applyCurrentNavState(required_modes);
	healthFailure(required_modes, component, log_level);
	addEvent(event_id, message, log_level, (uint8_t)required_modes, (uint8_t)component, args...);
}

template<typename... Args>
void Report::armingCheckFailure(ModeCategory required_modes, HealthComponentIndex component, uint32_t event_id,
				const char *message, events::Log log_level, Args... args)
{
	required_modes = applyCurrentNavState(required_modes);
	armingCheckFailure(required_modes, component, log_level);
	addEvent(event_id, message, log_level, (uint8_t)required_modes, (uint8_t)component, args...);
}

template<typename... Args>
bool Report::addEvent(uint32_t event_id, const char *message, events::Log log_level, Args... args)
{
	CONSOLE_PRINT_ARMING_CHECK_EVENT(log_level, event_id, message);

	constexpr unsigned args_size = events::util::sizeofArguments(args...);
	static_assert(args_size <= sizeof(events::EventType::arguments), "Too many arguments");
	unsigned total_size = sizeof(EventBufferHeader) + args_size;

	if (total_size > sizeof(_event_buffer) - _next_buffer_idx) {
		_buffer_overflowed = true;
		return false;
	}

	EventBufferHeader *header = (EventBufferHeader *)(_event_buffer + _next_buffer_idx);
	memcpy(&header->id, &event_id, sizeof(event_id)); // header might be unaligned
	header->log_level = (uint8_t)log_level;
	header->size = args_size;
	events::util::fillEventArguments(_event_buffer + _next_buffer_idx + sizeof(EventBufferHeader), args...);
	_next_buffer_idx += total_size;
	++_results[_current_result].num_events;
	_results[_current_result].event_id_hash ^= event_id; // very simple hash
	return true;
}


/**
 * @class HealthAndArmingCheckBase
 * Base class for all checks
 */
class HealthAndArmingCheckBase : public ModuleParams
{
public:
	HealthAndArmingCheckBase() : ModuleParams(nullptr) {};
	~HealthAndArmingCheckBase() = default;

	virtual void checkAndReport(const Context &context, Report &reporter) = 0;

	void updateParams() override { ModuleParams::updateParams(); }
};
