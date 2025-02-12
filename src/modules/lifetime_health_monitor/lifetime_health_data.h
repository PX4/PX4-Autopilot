/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include <stdint.h>
#include <float.h>

#include "bson_data_keys.h"
#include "bson_storage.h"

static constexpr uint32_t MILLISECONDS_PER_SECOND = 1000;
static constexpr uint32_t MICROSECONDS_PER_MILLISECOND = 1000;

static constexpr uint8_t NUM_MOTORS{4};
static constexpr uint8_t RECORD_FORMAT_VERSION{1};

typedef struct {
	// Maximum temperature per motor
	BsonKeyValueArray<float, NUM_MOTORS> max_temp;
	// Boot instances where temperature reached range WARN < temp < MAX_TEMP
	BsonKeyValueArray<uint16_t, NUM_MOTORS> instance_counts_warn;
	// Boot instances where temperature reached range temp > MAX_TEMP
	BsonKeyValueArray<uint16_t, NUM_MOTORS> instance_counts_max;
	// Total duration where temperature was in range WARN < temp < MAX_TEMP
	BsonKeyValueArray<uint32_t, NUM_MOTORS> temperature_duration_warn_s;
	// Total duration where temperature was in range temp > MAX_TEMP
	BsonKeyValueArray<uint32_t, NUM_MOTORS> temperature_duration_max_s;

	int encode(bson_encoder_s *encoder)
	{
		return !(max_temp.encode(encoder) == 0 &&
			 instance_counts_warn.encode(encoder) == 0 &&
			 instance_counts_max.encode(encoder) == 0 &&
			 temperature_duration_warn_s.encode(encoder) == 0 &&
			 temperature_duration_max_s.encode(encoder) == 0);
	}
} temperature_history_s;


typedef struct {
	// store max voltage per motor
	BsonKeyValueArray<float, NUM_MOTORS> max_voltage {KEY_ELECTRICAL_MAX_VOLTAGE, FLT_MIN};
	// store min voltage per motor
	BsonKeyValueArray<float, NUM_MOTORS> min_voltage {KEY_ELECTRICAL_MIN_VOLTAGE, FLT_MAX};
	// store max current per motor
	BsonKeyValueArray<float, NUM_MOTORS> max_current {KEY_ELECTRICAL_MAX_CURRENT, FLT_MIN};
	// store min current per motor
	BsonKeyValueArray<float, NUM_MOTORS> min_current {KEY_ELECTRICAL_MIN_CURRENT, FLT_MAX};
	// Boot instances where motor current reached range WARN < current < MAX_CURRENT
	BsonKeyValueArray<uint16_t, NUM_MOTORS> instance_counts_warn {KEY_ELECTRICAL_WARN_CURRENT_COUNT};
	// Boot instances where motor current reached range current > MAX_CURRENT
	BsonKeyValueArray<uint16_t, NUM_MOTORS> instance_counts_max {KEY_ELECTRICAL_MAX_CURRENT_COUNT};
	// Total duration where motor current was in range WARN < current < MAX_CURRENT
	BsonKeyValueArray<uint32_t, NUM_MOTORS> current_duration_warn_s {KEY_ELECTRICAL_WARN_CURRENT_DURATION};
	// Total duration where motor current was in range current > MAX_CURRENT
	BsonKeyValueArray<uint32_t, NUM_MOTORS> current_duration_max_s {KEY_ELECTRICAL_MAX_CURRENT_DURATION};

	int encode(bson_encoder_s *encoder)
	{
		return !(max_voltage.encode(encoder) == 0 &&
			 min_voltage.encode(encoder) == 0 &&
			 max_current.encode(encoder) == 0 &&
			 min_current.encode(encoder) == 0 &&
			 instance_counts_warn.encode(encoder) == 0 &&
			 instance_counts_max.encode(encoder) == 0 &&
			 current_duration_warn_s.encode(encoder) == 0 &&
			 current_duration_max_s.encode(encoder) == 0);
	}
} electrical_history_s;

typedef struct {
	// Total revolutions per motor
	BsonKeyValueArray<uint64_t, NUM_MOTORS> total_revolutions {KEY_RPM_TOTAL_REVOLUTIONS};
	// Maximum RPM per motor
	BsonKeyValueArray<uint16_t, NUM_MOTORS> max_rpm {KEY_RPM_MAX};
	// Total duration per motor where RPM was in range MED < rpm < HIGH
	BsonKeyValueArray<uint32_t, NUM_MOTORS> rpm_duration_medium_s {KEY_RPM_MEDIUM_DURATION};
	// Total duration per motor where RPM was in range rpm > HIGH
	BsonKeyValueArray<uint32_t, NUM_MOTORS> rpm_duration_high_s {KEY_RPM_HIGH_DURATION};

	int encode(bson_encoder_s *encoder)
	{
		return !(total_revolutions.encode(encoder) == 0 &&
			 max_rpm.encode(encoder) == 0 &&
			 rpm_duration_medium_s.encode(encoder) == 0 &&
			 rpm_duration_high_s.encode(encoder) == 0);
	}
} rpm_history_s;

typedef struct {
	// Total duration per motor where propeller flopping severity was in range MODERATE
	BsonKeyValueArray<uint32_t, NUM_MOTORS> flop_severity_moderate_s {KEY_PROP_MODERATE_FLOP_DURATION};
	// Total duration per motor where propeller flopping severity was in range SEVERE
	BsonKeyValueArray<uint32_t, NUM_MOTORS> flop_severity_severe_s {KEY_PROP_SEVERE_FLOP_DURATION};
	// Total duration per motor where asymmetric lift severity was in range MODERATE
	BsonKeyValueArray<uint32_t, NUM_MOTORS> asymmetric_lift_moderate_s {KEY_PROP_MODERATE_LIFT_ASYMMETRY};
	// Total duration per motor where asymmetric lift severity was in range SEVERE
	BsonKeyValueArray<uint32_t, NUM_MOTORS> asymmetric_lift_severe_s {KEY_PROP_SEVERE_LIFT_ASYMMETRY};

	int encode(bson_encoder_s *encoder)
	{
		return !(flop_severity_moderate_s.encode(encoder) == 0 &&
			 flop_severity_severe_s.encode(encoder) == 0 &&
			 asymmetric_lift_moderate_s.encode(encoder) == 0 &&
			 asymmetric_lift_severe_s.encode(encoder) == 0);
	}
} prop_dynamics_history_s;

typedef struct {
	electrical_history_s electrical_history;

	// ESC temperature history with keys initialized directly
	temperature_history_s esc_temp_history{
		.max_temp{KEY_ESC_TEMP_MAX, FLT_MIN},
		.instance_counts_warn{KEY_ESC_TEMP_WARN_COUNT},
		.instance_counts_max{KEY_ESC_TEMP_MAX_COUNT},
		.temperature_duration_warn_s{KEY_ESC_TEMP_WARN_DURATION},
		.temperature_duration_max_s{KEY_ESC_TEMP_MAX_DURATION}
	};

	// Motor temperature history with keys initialized directly
	temperature_history_s motor_temp_history{
		.max_temp{KEY_MOTOR_TEMP_MAX, FLT_MIN},
		.instance_counts_warn{KEY_MOTOR_TEMP_WARN_COUNT},
		.instance_counts_max{KEY_MOTOR_TEMP_MAX_COUNT},
		.temperature_duration_warn_s{KEY_MOTOR_TEMP_WARN_DURATION},
		.temperature_duration_max_s{KEY_MOTOR_TEMP_MAX_DURATION}
	};

	rpm_history_s rpm_history;
	prop_dynamics_history_s prop_dynamics_history;

	int encode(bson_encoder_s *encoder)
	{
		if (electrical_history.encode(encoder) != 0) {
			PX4_ERR("Failed to encode electrical_history");
			return -1;
		}

		if (esc_temp_history.encode(encoder) != 0) {
			PX4_ERR("Failed to encode esc_temp_history");
			return -1;
		}

		if (motor_temp_history.encode(encoder) != 0) {
			PX4_ERR("Failed to encode motor_temp_history");
			return -1;
		}

		if (rpm_history.encode(encoder) != 0) {
			PX4_ERR("Failed to encode rpm_history");
			return -1;
		}

		if (prop_dynamics_history.encode(encoder) != 0) {
			PX4_ERR("Failed to encode prop_dynamics_history");
			return -1;
		}

		return 0;
	}
} motor_history_s;


typedef struct {
	BsonKeyValue<uint32_t> arm_cycles{KEY_ARM_CYCLES, 0};
	BsonKeyValue<uint32_t> flight_duration_s{KEY_FLIGHT_TOTAL_DURATION}; // flight duration

	// don't include altitude for less than a certain amount since it's inferred from total flight time
	BsonKeyValue<uint32_t> high_altitude_duration_s{KEY_FLIGHT_HIGH_ALTITUDE}; // high altitude duration
	BsonKeyValue<uint32_t> medium_altitude_duration_s{KEY_FLIGHT_MEDIUM_ALTITUDE}; // medium altitude duration
	BsonKeyValue<uint32_t> low_altitude_duration_s{KEY_FLIGHT_LOW_ALTITUDE}; // low altitude duration

	BsonKeyValue<uint32_t> high_ambient_temperature_duration_s{KEY_FLIGHT_HIGH_AMBIENT_TEMP}; // high temperature duration
	BsonKeyValue<uint32_t> low_ambient_temperature_duration_s{KEY_FLIGHT_LOW_AMBIENT_TEMP}; // low temperature duration

	int encode(bson_encoder_s *encoder)
	{
		return !(arm_cycles.encode(encoder) == 0 &&
			 flight_duration_s.encode(encoder) == 0 &&
			 high_altitude_duration_s.encode(encoder) == 0 &&
			 medium_altitude_duration_s.encode(encoder) == 0 &&
			 low_altitude_duration_s.encode(encoder) == 0 &&
			 high_ambient_temperature_duration_s.encode(encoder) == 0 &&
			 low_ambient_temperature_duration_s.encode(encoder) == 0);
	}
} flight_history_s;

typedef struct {
	BsonKeyValue<uint32_t> high_vibration_duration_s{KEY_STRUCTURAL_HIGH_VIBRATION};
	BsonKeyValue<uint32_t> moderate_vibration_duration_s{KEY_STRUCTURAL_MODERATE_VIBRATION};
	BsonKeyValue<uint32_t> low_vibration_duration_s{KEY_STRUCTURAL_LOW_VIBRATION};

	int encode(bson_encoder_s *encoder)
	{
		return !(high_vibration_duration_s.encode(encoder) == 0 &&
			 moderate_vibration_duration_s.encode(encoder) == 0 &&
			 low_vibration_duration_s.encode(encoder) == 0);
	}
} structural_history_s;

typedef struct {
	BsonKeyValue<uint16_t> format_version{KEY_FORMAT_VERSION, RECORD_FORMAT_VERSION}; // format version, to track in case record format needs to be updated
	BsonKeyValue<uint64_t> init_timestamp{KEY_INIT_TIMESTAMP, 0}; // timestamp for when module first started running
	BsonKeyValue<uint32_t> boot_cycles{KEY_BOOT_CYCLES, 0}; // number of times the system has booted
	BsonKeyValue<uint32_t> on_duration_s{KEY_ON_DURATION, 0}; // total duration the system has been on

	flight_history_s flight_history;
	structural_history_s structural_history;
	motor_history_s motor_history;

	int encode(bson_encoder_s *encoder)
	{
		if (format_version.encode(encoder) != 0 ||
		    init_timestamp.encode(encoder) != 0 ||
		    boot_cycles.encode(encoder) != 0 ||
		    on_duration_s.encode(encoder) != 0)  {
			PX4_ERR("Failed to encode aircraft core lifetime data");
			return -1;
		}

		if (flight_history.encode(encoder) != 0) {
			PX4_ERR("Failed to encode flight_history");
			return -1;
		}

		if (structural_history.encode(encoder) != 0) {
			PX4_ERR("Failed to encode structural_history");
			return -1;
		}

		if (motor_history.encode(encoder) != 0) {
			PX4_ERR("Failed to encode motor_history");
			return -1;
		}

		return 0;
	}
} lifetime_health_data_s;


/**
 * Calculate the FNV-1a hash for a string, for up to 4 significant values in the hash
 * we are guaranteed to not have a hash collision, which works out for us
 * @param str the string to hash
 * @return the hash
 */
constexpr uint32_t fnv1a_hash(const char *key)
{
	uint32_t hash = 2166136261u;

	for (size_t i = 0; i < KEY_LENGTH; i++) { // 3 chars + null terminator = 4
		hash = (hash ^ key[i]) * 16777619u;
	}

	return hash;
}


/**
 * Calculate the checksum for the lifetime health data
 * @param data pointer to the lifetime health data
 * @return the checksum
 */
uint32_t calculate_checksum(lifetime_health_data_s *data);

/**
 * Update the checksum for the lifetime health data
 * @param data pointer to the lifetime health data
 */
void update_checksum(lifetime_health_data_s *data);

/**
 * Check if the checksum is valid
 * @param data pointer to the lifetime health data
 * @return true if the checksum is valid, false otherwise
 */
bool verify_checksum(lifetime_health_data_s *data);
