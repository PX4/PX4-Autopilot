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
#include <uORB/topics/health_report.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/failsafe_flags.h>
#include <systemlib/mavlink_log.h>
#include <drivers/drv_hrt.h>

#include <stdint.h>
#include <limits.h>

//#define CONSOLE_PRINT_ARMING_CHECK_EVENT // for debugging, print updated events whenever they change

#ifndef FRIEND_TEST // for gtest
#define FRIEND_TEST(a, b)
#endif

using namespace time_literals;

class HealthAndArmingChecks;

using navigation_mode_group_t = events::px4::enums::navigation_mode_group_t;
using health_component_t = events::px4::enums::health_component_t;

enum class NavModes : uint32_t {
	None = 0, ///< Using NavModes = None means arming is still possible (optional check)

	// Add the modes here as needed, but generally rather use mode requirements instead of checks for individual modes.
	Manual = (uint32_t)navigation_mode_group_t::manual,
	Stabilized = (uint32_t)navigation_mode_group_t::stab,
	PositionControl = (uint32_t)navigation_mode_group_t::posctl,
	Mission = (uint32_t)navigation_mode_group_t::mission,
	Takeoff = (uint32_t)navigation_mode_group_t::takeoff,

	All = 0xffffffff
};
static_assert(sizeof(navigation_mode_group_t) == sizeof(NavModes), "type mismatch");
static_assert(vehicle_status_s::NAVIGATION_STATE_MAX <= CHAR_BIT *sizeof(navigation_mode_group_t),
	      "type too small, use next larger type");

// Type to pass two mode groups in one struct to have the same number of function arguments to facilitate events parsing
struct NavModesMessageFail {
	NavModes message_modes; ///< modes in which there's user messageing but arming is allowed
	NavModes fail_modes; ///< modes in which checks fail which must be a subset of message_modes
};

static inline NavModes operator|(NavModes a, NavModes b)
{
	return static_cast<NavModes>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

static inline NavModes operator&(NavModes a, NavModes b)
{
	return static_cast<NavModes>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

static inline NavModes operator~(NavModes a)
{
	return static_cast<NavModes>(~static_cast<uint32_t>(a));
}

class HealthComponentIndex
{
public:
	__attribute__((always_inline)) constexpr uint8_t log2(uint64_t x)
	{
		uint8_t i = 0;

		while (x > 1) {
			x >>= 1;
			++i;
		}

		return i;
	}

	// The compiler is expected to evaluate this at compile-time, which generally works, but not
	// with GCC 9.3.1 for ARM, so we ensure it's inlined and optimized away.
	__attribute__((always_inline)) constexpr HealthComponentIndex(health_component_t component)
		: index(log2((uint64_t)component))
	{
	}
	const uint8_t index;
};

/**
 * @class Context
 * Provides commonly used information for health and arming checks
 */
class Context
{
public:
	Context(const vehicle_status_s &status)
		: _status(status)
	{}

	~Context() = default;

	bool isArmed() const { return _status.arming_state == vehicle_status_s::ARMING_STATE_ARMED; }

	bool isArmingRequest() const { return _is_arming_request; }

	void setIsArmingRequest(bool is_arming_request) { _is_arming_request = is_arming_request; }

	const vehicle_status_s &status() const { return _status; }

private:
	const vehicle_status_s &_status;
	bool _is_arming_request{false};	// true if we currently have an arming request
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

		health_component_t is_present;
		health_component_t error;
		health_component_t warning;

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

		health_component_t error;
		health_component_t warning;

		NavModes can_arm; ///< whether arming is possible for each mode group (bitset)
		NavModes can_run; ///< whether switching into a certain mode is possible (while armed)

		bool valid; ///< whether can_arm is valid, i.e. can be used

		void reset()
		{
			error = {};
			warning = {};
			can_arm = (NavModes) - 1; // bits are cleared for failed checks
			can_run = (NavModes) - 1;
			valid = false;
		}
		bool operator!=(const ArmingCheckResults &other)
		{
			return error != other.error || warning != other.warning ||
			       can_arm != other.can_arm || can_run != other.can_run || valid != other.valid;
		}
	};

	Report(failsafe_flags_s &failsafe_flags, hrt_abstime min_reporting_interval = 2_s)
		: _min_reporting_interval(min_reporting_interval), _failsafe_flags(failsafe_flags) { }
	~Report() = default;

	failsafe_flags_s &failsafeFlags() { return _failsafe_flags; }

	orb_advert_t *mavlink_log_pub() { return _mavlink_log_pub; }

	/**
	 * Whether arming is possible for a given navigation mode
	 */
	bool canArm(uint8_t nav_state) const
	{
		return _results[_current_result].arming_checks.valid &&
		       (uint32_t)(_results[_current_result].arming_checks.can_arm & getModeGroup(nav_state)) != 0;
	}

	/**
	 * Whether a navigation mode can be run (while already armed)
	 */
	bool canRun(uint8_t nav_state) const
	{
		return _results[_current_result].arming_checks.valid &&
		       (uint32_t)(_results[_current_result].arming_checks.can_run & getModeGroup(nav_state)) != 0;
	}

	void getHealthReport(health_report_s &report) const;

	/**
	 * Report a health failure. A health issue generally refers to a hardware issue, independent from environment
	 * or e.g. calibration.
	 */
	template<typename... Args>
	void healthFailure(NavModes required_modes, HealthComponentIndex component, uint32_t event_id,
			   const events::LogLevels &log_levels, const char *message, Args... args);

	void healthFailure(NavModes required_modes, HealthComponentIndex component, uint32_t event_id,
			   const events::LogLevels &log_levels, const char *message);

	void setIsPresent(health_component_t component);

	/**
	 * Directly set the health of a component. Using healthFailure() and setIsPresent() is preferred
	 */
	void setHealth(health_component_t component, bool is_present, bool warning, bool error);


	/**
	 * Report an arming check failure. If required_modes is None, arming is still possible.
	 */
	template<typename... Args>
	void armingCheckFailure(NavModes required_modes, HealthComponentIndex component, uint32_t event_id,
				const events::LogLevels &log_levels, const char *message, Args... args);

	void armingCheckFailure(NavModes required_modes, HealthComponentIndex component, uint32_t event_id,
				const events::LogLevels &log_levels, const char *message);

	/**
	 * Overloaded variant of armingCheckFailure() which allows to separately specify modes in which a message should be emitted and a subset in which arming is blocked
	 * @param required_modes .message_modes modes in which to put out the event and hence user message.
	 *                       .failing_modes modes in which to to fail arming. Has to be a subset of message_modes to never disallow arming without a reason.
	 */
	void armingCheckFailure(NavModesMessageFail required_modes, HealthComponentIndex component,
				uint32_t event_id, const events::LogLevels &log_levels, const char *message);

	void clearArmingBits(NavModes modes);

	/**
	 * Clear can_run bits for certain modes. This will prevent mode switching.
	 * For failsafe use the mode requirements instead, which then will clear the can_run bits.
	 * @param modes affected modes
	 */
	void clearCanRunBits(NavModes modes);

	const HealthResults &healthResults() const { return _results[_current_result].health; }
	const ArmingCheckResults &armingCheckResults() const { return _results[_current_result].arming_checks; }

	bool modePreventsArming(uint8_t nav_state) const { return _failsafe_flags.mode_req_prevent_arming & (1u << nav_state); }

	bool addExternalEvent(const event_s &event, NavModes modes);
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

		void reset() { health.reset(); arming_checks.reset(); num_events = 0; event_id_hash = 0; }

		bool operator!=(const Results &other)
		{
			return health != other.health || arming_checks != other.arming_checks ||
			       num_events != other.num_events || event_id_hash != other.event_id_hash;
		}
	};

	struct __attribute__((__packed__)) EventBufferHeader {
		uint8_t size; ///< arguments size
		uint32_t id;
		uint8_t log_levels;
#ifdef CONSOLE_PRINT_ARMING_CHECK_EVENT
		const char *message;
#endif
	};

	void healthFailure(NavModes required_modes, HealthComponentIndex component, events::Log log_level);
	void armingCheckFailure(NavModes required_modes, HealthComponentIndex component, events::Log log_level);

	template<typename... Args>
	bool addEvent(uint32_t event_id, const events::LogLevels &log_levels, const char *message, uint32_t modes,
		      Args... args);
	Report::EventBufferHeader *addEventToBuffer(uint32_t event_id, const events::LogLevels &log_levels, uint32_t modes,
			unsigned args_size);

	NavModes reportedModes(NavModes required_modes);

	NavModes getModeGroup(uint8_t nav_state) const;

	friend class HealthAndArmingChecks;
	friend class ExternalChecks;
	FRIEND_TEST(ReporterTest, basic_no_checks);
	FRIEND_TEST(ReporterTest, basic_fail_all_modes);
	FRIEND_TEST(ReporterTest, arming_checks_mode_category);
	FRIEND_TEST(ReporterTest, arming_checks_mode_category2);
	FRIEND_TEST(ReporterTest, reporting);
	FRIEND_TEST(ReporterTest, reporting_multiple);

	/**
	 * Reset current results.
	 * The calling order needs to be:
	 * - reset()
	 * - run all checks
	 * - finalize()
	 * - report() (which can be called independently as well)
	 */
	void reset();
	void prepare(uint8_t vehicle_type);
	/**
	 * Called after all checks are run. Returns true if the results changed
	 */
	bool finalize();

	bool report(bool force);

	/**
	 * Send out any unreported changes if there are any
	 */
	bool reportIfUnreportedDifferences();

	const hrt_abstime _min_reporting_interval;

	/// event buffer: stores current events + arguments.
	/// Since the amount of extra arguments varies, 4 bytes is used here as estimate
	uint8_t _event_buffer[(event_s::ORB_QUEUE_LENGTH - 2) * (sizeof(EventBufferHeader) + 1 + 1 + 4)];
	int _next_buffer_idx{0};
	bool _buffer_overflowed{false};

	bool _already_reported{false};
	bool _had_unreported_difference{false}; ///< true if there was a difference not reported yet (due to rate limitation)
	bool _results_changed{false};
	hrt_abstime _last_report{0};

	Results _results[2]; ///< Previous and current results to check for changes
	int _current_result{0};

	failsafe_flags_s &_failsafe_flags;

	orb_advert_t *_mavlink_log_pub{nullptr}; ///< mavlink log publication for legacy reporting
};

template<typename... Args>
void Report::healthFailure(NavModes required_modes, HealthComponentIndex component, uint32_t event_id,
			   const events::LogLevels &log_levels, const char *message, Args... args)
{
	healthFailure(required_modes, component, log_levels.external);
	addEvent(event_id, log_levels, message, (uint32_t)reportedModes(required_modes), (uint8_t)component.index, args...);
}

template<typename... Args>
void Report::armingCheckFailure(NavModes required_modes, HealthComponentIndex component, uint32_t event_id,
				const events::LogLevels &log_levels, const char *message, Args... args)
{
	armingCheckFailure(required_modes, component, log_levels.external);
	addEvent(event_id, log_levels, message, (uint32_t)reportedModes(required_modes), (uint8_t)component.index, args...);
}

template<typename... Args>
bool Report::addEvent(uint32_t event_id, const events::LogLevels &log_levels, const char *message, uint32_t modes,
		      Args... args)
{
	constexpr unsigned args_size = events::util::sizeofArguments(modes, args...);
	static_assert(args_size <= sizeof(event_s::arguments), "Too many arguments");
	unsigned total_size = sizeof(EventBufferHeader) + args_size;

	if (total_size > sizeof(_event_buffer) - _next_buffer_idx) {
		_buffer_overflowed = true;
		return false;
	}

	events::util::fillEventArguments(_event_buffer + _next_buffer_idx + sizeof(EventBufferHeader), modes, args...);
	// We split out the part of the code not requiring templating to reduce flash usage a bit
	EventBufferHeader *header = addEventToBuffer(event_id, log_levels, modes, args_size);
#ifdef CONSOLE_PRINT_ARMING_CHECK_EVENT
	memcpy(&header->message, &message, sizeof(message));
#else
	(void)header;
#endif
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
