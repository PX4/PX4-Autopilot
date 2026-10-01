/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/events/events.h>
#include <mavsdk/plugins/failure/failure.h>
#include <mavsdk/plugins/log_files/log_files.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>
#include <vector>

/**
 * GNSS failure injection and its observation over MAVLink, shared by the SIH failover tests and the gnss_failover
 * companion tool. It never arms or changes the mode. Injections go through MAV_CMD_INJECT_FAILURE, with the
 * SYS_FAIL_GPS_* parameters set first; the vehicle applies them where each receiver's sample is published.
 */
class GnssFailover
{
public:
	// SYS_FAIL_GPS_* payload of a wrong injection. Every field but the fix type is left unchanged at 0.
	struct WrongPayload {
		int32_t fix_type{2};       // SYS_FAIL_GPS_WRG, 0 = unchanged
		float eph{0.f};            // SYS_FAIL_GPS_EPH [m]
		float epv{0.f};            // SYS_FAIL_GPS_EPV [m]
		float speed_accuracy{0.f}; // SYS_FAIL_GPS_SAC [m/s]
		int32_t satellites{0};     // SYS_FAIL_GPS_SAT
		int32_t jamming_state{0};  // SYS_FAIL_GPS_JAM
		int32_t spoofing_state{0}; // SYS_FAIL_GPS_SPF
	};

	struct Injection {
		mavsdk::Failure::FailureType type{mavsdk::Failure::FailureType::Off};
		int instance{1};           // receiver, 1-based (sensor_gnss instance + 1); 0 = all
		WrongPayload wrong{};
		int32_t slow_divider{4};   // SYS_FAIL_GPS_DIV
	};

	// An ODOMETRY sample whose reset counter changed
	struct Reset {
		int64_t vehicle_time_us{0};
		unsigned counter_increase{0};
		std::array<float, 3> position_jump{}; // [m] NED, motion since the previous sample removed
		float yaw_jump{0.f};                  // [rad]
	};

	// GPS_RAW_INT (index 0) and GPS2_RAW (index 1): the receivers MAVLink keeps for the session
	struct Receiver {
		int64_t vehicle_time_us{0}; // last message, 0 before the first one
		uint8_t fix_type{0};        // GPS_FIX_TYPE
		uint8_t satellites{0};
		float eph{0.f};             // [m] 0 when unknown
	};

	struct PositionChange {
		int64_t vehicle_time_us{0};
		bool position_ok{false}; // ESTIMATOR_STATUS: relative and absolute horizontal and absolute vertical position
	};

	struct ModeChange {
		int64_t vehicle_time_us{0};
		mavsdk::Telemetry::FlightMode mode{mavsdk::Telemetry::FlightMode::Unknown};
	};

	struct EventRecord {
		int64_t vehicle_time_us{0};
		std::string name;    // namespace/name
		std::string message;
	};

	// subscribe_events: false when the caller already subscribes to events and forwards them to on_event()
	GnssFailover(std::shared_ptr<mavsdk::System> system, bool subscribe_events);
	~GnssFailover();

	GnssFailover(const GnssFailover &) = delete;
	GnssFailover &operator=(const GnssFailover &) = delete;

	// Streams ODOMETRY, ESTIMATOR_STATUS, GPS_RAW_INT and GPS2_RAW at the rates the observation needs
	bool request_streams();

	// Empty when the companion may inject, otherwise why not: SYS_FAILURE_EN set, both receivers report a 3D fix,
	// and, if require_flying, the vehicle is armed, in the air and in a position-controlled mode
	std::string injection_refusal(bool require_flying);

	// Sets the payload parameters, then injects. The parameters are restored by restore_parameters().
	bool inject(const Injection &injection);

	// Sends an off or ok injection without waiting for the acknowledgement, for timing that the round trip would
	// distort. The payload parameters aren't set.
	void inject_without_ack(mavsdk::Failure::FailureType type, int instance);

	// Ends every GNSS injection of the receiver (0 = all receivers)
	bool clear(int instance = 0);

	// Puts back the SYS_FAIL_GPS_* values that inject() changed
	bool restore_parameters();

	void on_event(const mavsdk::Events::Event &event);

	// Vehicle time of the latest ODOMETRY sample, 0 before the first one
	int64_t vehicle_time_us() const;

	// Waits in vehicle time, so that it follows the simulation speed. False on timeout.
	bool wait_until(const std::function<bool()> &condition, double timeout_s) const;
	void sleep(double seconds) const;

	// Sum of the ODOMETRY reset counter increases since the first sample: position, velocity and attitude resets
	unsigned reset_count() const;
	std::vector<Reset> resets() const;

	Receiver receiver(int index) const;

	std::vector<PositionChange> position_changes() const;
	std::vector<ModeChange> mode_changes() const;
	std::vector<EventRecord> events() const;

	// The estimator reports a valid position now, and did throughout [since_us, now]
	bool position_ok() const;
	bool position_ok_since(int64_t since_us) const;

	// Modes the vehicle was in during [since_us, now]
	std::vector<mavsdk::Telemetry::FlightMode> modes_since(int64_t since_us) const;

	// Downloads the newest log on the vehicle, false on failure or after timeout_s of host time
	bool download_last_log(const std::string &path, double timeout_s);

	// Every injection, acknowledgement and observed change, one CSV line each: wall time, vehicle time, what
	void set_record(std::ostream *record) { _record = record; }
	void record(const std::string &what);

private:
	void handle_odometry(const mavlink_message_t &message);
	void handle_gps_raw_int(const mavlink_message_t &message);
	void handle_gps2_raw(const mavlink_message_t &message);
	void handle_estimator_status(const mavlink_message_t &message);
	void set_message_interval(uint16_t message_id, float rate_hz);
	bool set_param_int(const std::string &name, int32_t value);
	bool set_param_float(const std::string &name, float value);

	std::shared_ptr<mavsdk::System> _system;
	std::unique_ptr<mavsdk::Failure> _failure;
	std::unique_ptr<mavsdk::Param> _param;
	std::unique_ptr<mavsdk::Telemetry> _telemetry;
	std::unique_ptr<mavsdk::MavlinkPassthrough> _passthrough;
	std::unique_ptr<mavsdk::Events> _events;
	std::unique_ptr<mavsdk::LogFiles> _log_files;

	mavsdk::MavlinkPassthrough::MessageHandle _odometry_handle{};
	mavsdk::MavlinkPassthrough::MessageHandle _gps_raw_int_handle{};
	mavsdk::MavlinkPassthrough::MessageHandle _gps2_raw_handle{};
	mavsdk::MavlinkPassthrough::MessageHandle _estimator_status_handle{};
	mavsdk::Telemetry::FlightModeHandle _flight_mode_handle{};
	mavsdk::Events::EventsHandle _events_handle{};

	mutable std::mutex _mutex;

	struct OdometrySample {
		int64_t time_us{0};
		std::array<float, 3> position{};
		float yaw{0.f};
		uint8_t reset_counter{0};
	};

	OdometrySample _odometry[3] {}; // newest first
	unsigned _odometry_samples{0};
	unsigned _reset_count{0};
	std::vector<Reset> _resets;
	Receiver _receivers[2] {};
	std::vector<PositionChange> _position_changes;
	std::vector<ModeChange> _mode_changes;
	std::vector<EventRecord> _event_records;

	// SYS_FAIL_GPS_* values before the first inject() that changed them
	std::map<std::string, int32_t> _saved_int_params;
	std::map<std::string, float> _saved_float_params;

	std::ostream *_record{nullptr};
	std::mutex _record_mutex;
};

std::ostream &operator<<(std::ostream &str, const GnssFailover::Injection &injection);
