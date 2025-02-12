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

#include "LifetimeHealthMonitor.hpp"

ModuleBase::Descriptor LifetimeHealthMonitor::desc{LifetimeHealthMonitor::task_spawn, LifetimeHealthMonitor::custom_command, LifetimeHealthMonitor::print_usage};

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>

int LifetimeHealthMonitor::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(
		       desc, [](int ac, char *av[]) -> ModuleBase * { return LifetimeHealthMonitor::instantiate(ac, av); }, argc, argv);
}

int LifetimeHealthMonitor::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("lifetime_health_monitor",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MIN + 9,
					  PX4_STACK_ADJUSTED(2400),
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	// wait until task is up & running
	if (wait_until_running(desc) < 0) {
		desc.task_id = -1;
		return -1;
	}

	return 0;
}

LifetimeHealthMonitor *LifetimeHealthMonitor::instantiate(int argc, char *argv[])
{
	return new LifetimeHealthMonitor();
}

LifetimeHealthMonitor::LifetimeHealthMonitor()
	: ModuleParams(nullptr)
{
}

void LifetimeHealthMonitor::run()
{
	_determine_threshold_values();

	if (!_persistent_storage_accessible()) {
		PX4_ERR("Persistent storage not accessible");
		return;
	}

	if (!_lifetime_health_data_dir_exists()) {
		_mkdir_health_data_path();

	} else {
		if (_lifetime_health_data_import() != 0) {
			PX4_ERR("Failed to load persistent health data, resetting data!");
		}
	}

	bool _prev_armed = false;

	uint32_t _disarmed_timestamp_ms{0};

	uint32_t _prev_esc_timestamp_ms{0};
	uint16_t _armed_esc_elapsed_time_ms{0};

	uint32_t _task_current_time_ms{0};
	uint32_t _task_prev_timestamp_ms{0};
	uint16_t _task_elapsed_time_ms{0};

	uint32_t _last_health_stats_publish_time_ms{0};

	// For SD card write operation
	uint32_t _last_sd_write_time_ms{0};

	// assume that the system has booted once when the module starts
	_latest_health_data.boot_cycles.value()++;

	while (!should_exit()) {
		// update on duration
		_task_current_time_ms = hrt_absolute_time() / MICROSECONDS_PER_MILLISECOND;

		// if no initial timestamp is set, then assume first boot and set utc timestamp in seconds
		if (_latest_health_data.init_timestamp.value() == 0) {
			_set_initial_timestamp();
		}

		if (_task_prev_timestamp_ms > 0) {
			_task_elapsed_time_ms += math::min(_task_current_time_ms - _task_prev_timestamp_ms, MAX_INTEGRATION_DELTA_MS);
			_latest_health_data.on_duration_s.value() += _task_elapsed_time_ms / MILLISECONDS_PER_SECOND;
			_task_elapsed_time_ms = _task_elapsed_time_ms % MILLISECONDS_PER_SECOND;
		}

		// data that's independent of ESC status
		_update_structural_history();

		if (_esc_status_sub.updated() && _vehicle_odometry_sub.updated()) {
			esc_status_s esc_status;
			vehicle_odometry_s vehicle_odometry;

			if (_esc_status_sub.copy(&esc_status) && _vehicle_odometry_sub.copy(&vehicle_odometry)) {
				uint32_t _current_esc_timestamp_ms = esc_status.timestamp / MICROSECONDS_PER_MILLISECOND;
				bool _armed = esc_status.esc_armed_flags;

				_update_all_esc_and_motor_history(esc_status, vehicle_odometry);

				if (_armed) {
					_update_flight_history();

					if (_prev_esc_timestamp_ms > 0) {
						_armed_esc_elapsed_time_ms += math::min(_current_esc_timestamp_ms - _prev_esc_timestamp_ms, MAX_INTEGRATION_DELTA_MS);
						_latest_health_data.flight_history.flight_duration_s.value() += _armed_esc_elapsed_time_ms / MILLISECONDS_PER_SECOND;
						_armed_esc_elapsed_time_ms = _armed_esc_elapsed_time_ms % MILLISECONDS_PER_SECOND;
					}

					if (!_prev_armed) {
						_latest_health_data.flight_history.arm_cycles.value()++;
						_prev_armed = true;
					}

				} else {
					if (_prev_armed) {
						_disarmed_timestamp_ms = _current_esc_timestamp_ms;
					}

					_prev_armed = false;
				}

				_prev_esc_timestamp_ms = _current_esc_timestamp_ms;

				if (_task_current_time_ms - _last_health_stats_publish_time_ms > UORB_PUBLISH_INTERVAL_MS) {
					_publish_lifetime_health_stats(_latest_health_data);
					_last_health_stats_publish_time_ms = _task_current_time_ms;
				}

				// SD CARD OPERATION
				if ((!esc_status.esc_armed_flags && _task_current_time_ms - _last_sd_write_time_ms > SD_WRITE_INTERVAL_MS
				     && (_current_esc_timestamp_ms - _disarmed_timestamp_ms > WAIT_TIME_AFTER_DISARM_TO_WRITE_MS))) {
					if (_bson_encode_and_write_lifetime_health_data(&_latest_health_data) == -1) {
						PX4_ERR("Failed to update persistent health data");
					}

					_last_sd_write_time_ms = _task_current_time_ms;
				}
			}
		}

		_task_prev_timestamp_ms = _task_current_time_ms;

		px4_usleep(TASK_UPDATE_INTERVAL);
	}

	exit_and_cleanup(desc);
}


int LifetimeHealthMonitor::_verify_record(int fd)
{
	PX4_DEBUG("Verifying lifetime health data record");

	if (fd < 0) {
		return -1;
	}

	if (lseek(fd, 0, SEEK_SET) != 0) {
		PX4_ERR("verify: seek failed");
		return -1;
	}

	bson_decoder_s decoder{};

	if (bson_decoder_init_file(&decoder, fd, bson_decode_lifetime_health_data_callback) == 0) {
		int result = -1;

		_is_loading_to_verify_record = true;

		do {
			result = bson_decoder_next(&decoder);

		} while (result > 0);

		_is_loading_to_verify_record = false;

		if (result == 0) {
			if (decoder.total_document_size != decoder.total_decoded_size) {
				PX4_ERR("BSON document size (%" PRId32 ") doesn't match bytes decoded (%" PRId32 ")", decoder.total_document_size,
					decoder.total_decoded_size);

			} else {
				return 0;
			}

		} else if (result == -ENODATA) {
			PX4_ERR("verify: no BSON data");

		} else {
			PX4_ERR("verify: failed (%d)", result);
		}
	}

	return -1;
}


/**
 * Encode and write lifetime health data to a record. Implemented like parameters.cpp param_export_internal.
 * Verifies the first record write, but not the second write
 * @param data Lifetime health data to write.
 * @return 0 on success, -1 on failure.
 */
int LifetimeHealthMonitor::_bson_encode_and_write_lifetime_health_data(lifetime_health_data_s *data)
{
	int res = PX4_ERROR;

	for (int i = 0; i < LIFETIME_HEALTH_DATA_NUM_RECORDS; i++) {
		int fd = ::open(record_paths[i], O_WRONLY | O_CREAT | O_TRUNC, PX4_O_MODE_666);

		if (fd > -1) {
			res = _write_to_record(fd, data);
			::close(fd);
		}

		if (i == 1) {
			// do not verify the second record since verification is expensive
			res = PX4_OK;
			break;
		}

		if (res == PX4_OK) {
			int fd_verify = ::open(record_paths[i], O_RDONLY, PX4_O_MODE_666);
			res = _verify_record(fd_verify) || lseek(fd_verify, 0, SEEK_SET)
			      || _verify_record(fd_verify);
			::close(fd_verify);
		}

		if (res != PX4_OK) {
			PX4_ERR("Failed to write lifetime health data to record %d", i);
			// do not touch the next record if writes are failing, as it could corrupt that file too
			break;
		}

		px4_usleep(10000); // wait at least 10 milliseconds before writing to the next record
	}

	if (res != PX4_OK) {
		PX4_ERR("Failed to write lifetime health data to any record");
	}

	return res;
}

int LifetimeHealthMonitor::_write_to_record(int fd, lifetime_health_data_s *data)
{
	PX4_DEBUG("Writing lifetime health data to record");

	int result = -1;
	bson_encoder_s encoder{};
	uint8_t bson_buffer[256];

	if (lseek(fd, 0, SEEK_SET) != 0) {
		PX4_ERR("export seek failed %d", errno);
		return -1;
	}

	if (bson_encoder_init_buf_file(&encoder, fd, &bson_buffer, sizeof(bson_buffer)) != 0) {
		goto out;
	}

	if (data->encode(&encoder) != 0) {
		goto out;
	}

	result = 0;

out:

	// same mechanism as used in param_export_internal in parameters.cpp
	if (result == 0) {
		if (bson_encoder_fini(&encoder) != PX4_OK) {
			PX4_ERR("BSON encoder finalize failed");
			result = -1;
		}

	} else {
		PX4_ERR("BSON encode failed");
	}

	return result;
}


int LifetimeHealthMonitor::_lifetime_health_data_import()
{
	static constexpr int MAX_ATTEMPTS = 2;

	for (int record_index = 0; record_index < LIFETIME_HEALTH_DATA_NUM_RECORDS; record_index++) {
		int fd = open(record_paths[record_index], O_RDONLY);

		if (fd < 0) {
			if (record_index == LIFETIME_HEALTH_DATA_NUM_RECORDS - 1) {
				PX4_ERR("Failed to load any persistent data records");
				return -1;
			}

			PX4_ERR("Failed to open persistent data record, trying next record");
			px4_usleep(10000); // wait at least 10 milliseconds before trying again
			continue;
		}

		for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
			bson_decoder_s decoder{};

			if (bson_decoder_init_file(&decoder, fd, bson_decode_lifetime_health_data_callback) == 0) {
				int result = -1;

				do {
					result = bson_decoder_next(&decoder);

				} while (result > 0);

				if (result == 0) {
					if (decoder.total_document_size == decoder.total_decoded_size) {
						PX4_DEBUG("BSON document size %" PRId32 " bytes, decoded %" PRId32 " bytes (INT32:%" PRIu16 ", FLOAT:%" PRIu16 ")",
							  decoder.total_document_size, decoder.total_decoded_size,
							  decoder.count_node_int32, decoder.count_node_double);

						return 0;

					} else {
						PX4_ERR("BSON document size (%" PRId32 ") doesn't match bytes decoded (%" PRId32 ")",
							decoder.total_document_size, decoder.total_decoded_size);
					}

				} else if (result == -ENODATA) {
					// silently retry as a precaution unless this is our last attempt
					if (attempt == MAX_ATTEMPTS) {
						PX4_DEBUG("BSON: no data");
						return 0;
					}

				} else {
					PX4_ERR("Lifetime health data import failed (%d) attempt %d", result, attempt);
				}

			} else {
				PX4_ERR("Lifetime health data import bson decoder init failed (attempt %d)", attempt);
			}

			if (attempt != MAX_ATTEMPTS) {
				if (lseek(fd, 0, SEEK_SET) != 0) {
					PX4_ERR("Lifetime health data import lseek failed (%d)", errno);
				}

				px4_usleep(10000); // wait at least 10 milliseconds before trying again
			}
		}
	}

	return -1;
}

void LifetimeHealthMonitor::_set_initial_timestamp()
{
	if (_sensor_gps_sub.updated()) {
		sensor_gps_s sensor_gps;
		_sensor_gps_sub.copy(&sensor_gps);

		if (sensor_gps.time_utc_usec > 0) {
			_latest_health_data.init_timestamp.value() = sensor_gps.time_utc_usec / MICROSECONDS_PER_MILLISECOND /
					MILLISECONDS_PER_SECOND;
		}
	}
}


// TODO: make sure all parameters are being imported correctly
int LifetimeHealthMonitor::bson_decode_lifetime_health_data_callback(bson_decoder_t decoder, bson_node_t node)
{
	/*
	 * Copied from parameters.cpp.
	 * EOO means the end of the parameter object. (Currently not supporting
	 * nested BSON objects).
	 */
	if (node->type == BSON_EOO) {
		PX4_DEBUG("end of parameters");
		return 0;
	}

	// skip since we are only decoding data for verification, we don't
	// want to override loaded data in memory which is always latest
	if (_is_loading_to_verify_record) {
		return 1;
	}

	uint32_t bson_key_hash = fnv1a_hash(node->name);

	// split based on data type for readability
	if (node->type == BSON_INT32) {
		return handle_bson_int32_key(decoder, node, bson_key_hash);

	} else if (node->type == BSON_INT64) {
		return handle_bson_int64_key(decoder, node, bson_key_hash);

	} else if (node->type == BSON_BINDATA) {
		// all arrays are being stored as Binary data for this implementation
		return handle_bson_binary_key(decoder, node, bson_key_hash);
	}

	// by default return 1 to indicate success
	return 1;
}

int LifetimeHealthMonitor::handle_bson_int32_key(bson_decoder_t decoder, bson_node_t node, uint32_t bson_key_hash)
{
	// all uint8, uint16, uint32 BsonKeyValues are stored as int32
	switch (bson_key_hash) {
	case fnv1a_hash(KEY_FORMAT_VERSION):
		_latest_health_data.format_version.decode(node);
		return 1;

	case fnv1a_hash(KEY_BOOT_CYCLES):
		_latest_health_data.boot_cycles.decode(node);
		return 1;

	case fnv1a_hash(KEY_ON_DURATION):
		_latest_health_data.on_duration_s.decode(node);
		return 1;

	case fnv1a_hash(KEY_ARM_CYCLES):
		_latest_health_data.flight_history.arm_cycles.decode(node);
		return 1;

	case fnv1a_hash(KEY_FLIGHT_TOTAL_DURATION):
		_latest_health_data.flight_history.flight_duration_s.decode(node);
		return 1;

	case fnv1a_hash(KEY_STRUCTURAL_HIGH_VIBRATION):
		_latest_health_data.structural_history.high_vibration_duration_s.decode(node);
		return 1;

	case fnv1a_hash(KEY_STRUCTURAL_MODERATE_VIBRATION):
		_latest_health_data.structural_history.moderate_vibration_duration_s.decode(node);
		return 1;

	case fnv1a_hash(KEY_STRUCTURAL_LOW_VIBRATION):
		_latest_health_data.structural_history.low_vibration_duration_s.decode(node);
		return 1;

	case fnv1a_hash(KEY_FLIGHT_HIGH_AMBIENT_TEMP):
		_latest_health_data.flight_history.high_ambient_temperature_duration_s.decode(node);
		return 1;

	case fnv1a_hash(KEY_FLIGHT_LOW_AMBIENT_TEMP):
		_latest_health_data.flight_history.low_ambient_temperature_duration_s.decode(node);
		return 1;

	case fnv1a_hash(KEY_FLIGHT_HIGH_ALTITUDE):
		_latest_health_data.flight_history.high_altitude_duration_s.decode(node);
		return 1;

	case fnv1a_hash(KEY_FLIGHT_MEDIUM_ALTITUDE):
		_latest_health_data.flight_history.medium_altitude_duration_s.decode(node);
		return 1;

	case fnv1a_hash(KEY_FLIGHT_LOW_ALTITUDE):
		_latest_health_data.flight_history.low_altitude_duration_s.decode(node);
		return 1;
	}

	// by default return 1 to indicate success
	return 1;
}

int LifetimeHealthMonitor::handle_bson_int64_key(bson_decoder_t decoder, bson_node_t node, uint32_t bson_key_hash)
{
	switch (bson_key_hash) {
	case fnv1a_hash(KEY_INIT_TIMESTAMP):
		_latest_health_data.init_timestamp.decode(node);
		return 1;
	}

	return 1;
}

int LifetimeHealthMonitor::handle_bson_binary_key(bson_decoder_t decoder, bson_node_t node, uint32_t bson_key_hash)
{
	switch (bson_key_hash) {

	// MOTOR TEMPERATURE HISTORY
	case fnv1a_hash(KEY_MOTOR_TEMP_MAX):
		if (_latest_health_data.motor_history.motor_temp_history.max_temp.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode max temperature");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_MOTOR_TEMP_WARN_COUNT):
		if (_latest_health_data.motor_history.motor_temp_history.instance_counts_warn.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode temperature warning count");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_MOTOR_TEMP_MAX_COUNT):
		if (_latest_health_data.motor_history.motor_temp_history.instance_counts_max.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode temperature max count");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_MOTOR_TEMP_WARN_DURATION):
		if (_latest_health_data.motor_history.motor_temp_history.temperature_duration_warn_s.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode temperature warning duration");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_MOTOR_TEMP_MAX_DURATION):
		if (_latest_health_data.motor_history.motor_temp_history.temperature_duration_max_s.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode temperature max duration");
			return -1;
		}

		return 1;

	// ESC TEMPERATURE HISTORY
	case fnv1a_hash(KEY_ESC_TEMP_MAX):
		if (_latest_health_data.motor_history.esc_temp_history.max_temp.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode max temperature");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_ESC_TEMP_WARN_COUNT):
		if (_latest_health_data.motor_history.esc_temp_history.instance_counts_warn.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode temperature warning count");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_ESC_TEMP_MAX_COUNT):
		if (_latest_health_data.motor_history.esc_temp_history.instance_counts_max.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode temperature max count");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_ESC_TEMP_WARN_DURATION):
		if (_latest_health_data.motor_history.esc_temp_history.temperature_duration_warn_s.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode temperature warning duration");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_ESC_TEMP_MAX_DURATION):
		if (_latest_health_data.motor_history.esc_temp_history.temperature_duration_max_s.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode temperature max duration");
			return -1;
		}

		return 1;

	// RPM HISTORY
	case fnv1a_hash(KEY_RPM_TOTAL_REVOLUTIONS):
		if (_latest_health_data.motor_history.rpm_history.total_revolutions.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode total revolutions");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_RPM_MAX):
		if (_latest_health_data.motor_history.rpm_history.max_rpm.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode max rpm");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_RPM_MEDIUM_DURATION):
		if (_latest_health_data.motor_history.rpm_history.rpm_duration_medium_s.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode medium rpm duration");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_RPM_HIGH_DURATION):
		if (_latest_health_data.motor_history.rpm_history.rpm_duration_high_s.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode high rpm duration");
			return -1;
		}

		return 1;

	// PROP DYNAMICS HISTORY
	case fnv1a_hash(KEY_PROP_MODERATE_FLOP_DURATION):
		if (_latest_health_data.motor_history.prop_dynamics_history.flop_severity_moderate_s.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode moderate flop duration");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_PROP_SEVERE_FLOP_DURATION):
		if (_latest_health_data.motor_history.prop_dynamics_history.flop_severity_severe_s.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode severe flop duration");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_PROP_MODERATE_LIFT_ASYMMETRY):
		if (_latest_health_data.motor_history.prop_dynamics_history.asymmetric_lift_moderate_s.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode moderate lift asymmetry");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_PROP_SEVERE_LIFT_ASYMMETRY):
		if (_latest_health_data.motor_history.prop_dynamics_history.asymmetric_lift_severe_s.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode severe lift asymmetry");
			return -1;
		}

		return 1;

	// ELECTRICAL HISTORY
	case fnv1a_hash(KEY_ELECTRICAL_MAX_VOLTAGE):
		if (_latest_health_data.motor_history.electrical_history.max_voltage.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode max voltage");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_ELECTRICAL_MIN_VOLTAGE):
		if (_latest_health_data.motor_history.electrical_history.min_voltage.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode min voltage");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_ELECTRICAL_MAX_CURRENT):
		if (_latest_health_data.motor_history.electrical_history.max_current.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode max current");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_ELECTRICAL_MIN_CURRENT):
		if (_latest_health_data.motor_history.electrical_history.min_current.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode min current");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_ELECTRICAL_WARN_CURRENT_COUNT):
		if (_latest_health_data.motor_history.electrical_history.instance_counts_warn.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode warning count");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_ELECTRICAL_MAX_CURRENT_COUNT):
		if (_latest_health_data.motor_history.electrical_history.instance_counts_max.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode max count");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_ELECTRICAL_WARN_CURRENT_DURATION):
		if (_latest_health_data.motor_history.electrical_history.current_duration_warn_s.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode warning duration");
			return -1;
		}

		return 1;

	case fnv1a_hash(KEY_ELECTRICAL_MAX_CURRENT_DURATION):
		if (_latest_health_data.motor_history.electrical_history.current_duration_max_s.decode(decoder, node) != 0) {
			PX4_ERR("Failed to decode max duration");
			return -1;
		}

		return 1;

	}

	return 1;
}

void LifetimeHealthMonitor::_update_electrical_history(const esc_report_s &esc_report, uint16_t esc_index,
		uint32_t delta_ms)
{
	if (esc_report.esc_current > _latest_health_data.motor_history.electrical_history.max_current[esc_index]) {
		_latest_health_data.motor_history.electrical_history.max_current[esc_index] = esc_report.esc_current;
	}

	if (esc_report.esc_current < _latest_health_data.motor_history.electrical_history.min_current[esc_index]) {
		_latest_health_data.motor_history.electrical_history.min_current[esc_index] = esc_report.esc_current;
	}

	if (esc_report.esc_voltage > _latest_health_data.motor_history.electrical_history.max_voltage[esc_index]) {
		_latest_health_data.motor_history.electrical_history.max_voltage[esc_index] = esc_report.esc_voltage;
	}

	if (esc_report.esc_voltage < _latest_health_data.motor_history.electrical_history.min_voltage[esc_index]) {
		_latest_health_data.motor_history.electrical_history.min_voltage[esc_index] = esc_report.esc_voltage;
	}

	static bool current_max_thresh_hit[NUM_MOTORS] = {false};
	static bool current_warn_thresh_hit[NUM_MOTORS] = {false};

	static uint16_t current_duration_max_ms[NUM_MOTORS] = {0};
	static uint16_t current_duration_warn_ms[NUM_MOTORS] = {0};

	if (esc_report.esc_current >= _motor_max_current) {
		if (!current_max_thresh_hit[esc_index]) {
			_latest_health_data.motor_history.electrical_history.instance_counts_max[esc_index]++;
			current_max_thresh_hit[esc_index] = true;
		}

		current_duration_max_ms[esc_index] += delta_ms;
		_latest_health_data.motor_history.electrical_history.current_duration_max_s[esc_index] +=
			current_duration_max_ms[esc_index] / MILLISECONDS_PER_SECOND;
		current_duration_max_ms[esc_index] = current_duration_max_ms[esc_index] % MILLISECONDS_PER_SECOND;

	} else if (esc_report.esc_current >= _motor_warn_current) {
		if (!current_warn_thresh_hit[esc_index]) {
			_latest_health_data.motor_history.electrical_history.instance_counts_warn[esc_index]++;
			current_warn_thresh_hit[esc_index] = true;
		}

		current_duration_warn_ms[esc_index] += delta_ms;
		_latest_health_data.motor_history.electrical_history.current_duration_warn_s[esc_index] +=
			current_duration_warn_ms[esc_index] / MILLISECONDS_PER_SECOND;
		current_duration_warn_ms[esc_index] = current_duration_warn_ms[esc_index] % MILLISECONDS_PER_SECOND;
	}
}

void LifetimeHealthMonitor::_update_esc_temp_history(const esc_report_s &esc_report, uint16_t esc_index,
		uint32_t delta_ms)
{
	if (esc_report.esc_temperature > _latest_health_data.motor_history.esc_temp_history.max_temp[esc_index]) {
		_latest_health_data.motor_history.esc_temp_history.max_temp[esc_index] = esc_report.esc_temperature;
	}

	static bool esc_temp_warn_thresh_hit[NUM_MOTORS] = {false};
	static bool esc_temp_max_thresh_hit[NUM_MOTORS] = {false};

	static uint16_t temperature_duration_warn_ms[NUM_MOTORS] = {0};
	static uint16_t temperature_duration_max_ms[NUM_MOTORS] = {0};

	if (esc_report.esc_temperature >= _esc_max_temp) {
		if (!esc_temp_max_thresh_hit[esc_index]) {
			_latest_health_data.motor_history.esc_temp_history.instance_counts_max[esc_index]++;
			esc_temp_max_thresh_hit[esc_index] = true;
		}

		temperature_duration_max_ms[esc_index] += delta_ms;
		_latest_health_data.motor_history.esc_temp_history.temperature_duration_max_s[esc_index] +=
			temperature_duration_max_ms[esc_index] / MILLISECONDS_PER_SECOND;
		temperature_duration_max_ms[esc_index] = temperature_duration_max_ms[esc_index] % MILLISECONDS_PER_SECOND;

	} else if (esc_report.esc_temperature >= _esc_warn_temp) {
		if (!esc_temp_warn_thresh_hit[esc_index]) {
			_latest_health_data.motor_history.esc_temp_history.instance_counts_warn[esc_index]++;
			esc_temp_warn_thresh_hit[esc_index] = true;
		}

		temperature_duration_warn_ms[esc_index] += delta_ms;
		_latest_health_data.motor_history.esc_temp_history.temperature_duration_warn_s[esc_index] +=
			temperature_duration_warn_ms[esc_index] / MILLISECONDS_PER_SECOND;
		temperature_duration_warn_ms[esc_index] = temperature_duration_warn_ms[esc_index] % MILLISECONDS_PER_SECOND;
	}
}

void LifetimeHealthMonitor::_update_motor_temp_history(const esc_report_s &esc_report, uint16_t esc_index,
		uint32_t delta_ms)
{
	if (esc_report.motor_temperature > _latest_health_data.motor_history.motor_temp_history.max_temp[esc_index]) {
		_latest_health_data.motor_history.motor_temp_history.max_temp[esc_index] = esc_report.motor_temperature;
	}

	static bool motor_temp_warn_thresh_hit[NUM_MOTORS] = {false};
	static bool motor_temp_max_thresh_hit[NUM_MOTORS] = {false};

	static uint16_t temperature_duration_warn_ms[NUM_MOTORS] = {0};
	static uint16_t temperature_duration_max_ms[NUM_MOTORS] = {0};

	if (esc_report.motor_temperature >= _motor_max_temp) {
		if (!motor_temp_max_thresh_hit[esc_index]) {
			_latest_health_data.motor_history.motor_temp_history.instance_counts_max[esc_index]++;
			motor_temp_max_thresh_hit[esc_index] = true;
		}

		temperature_duration_max_ms[esc_index] += delta_ms;
		_latest_health_data.motor_history.motor_temp_history.temperature_duration_max_s[esc_index] +=
			temperature_duration_max_ms[esc_index] / MILLISECONDS_PER_SECOND;
		temperature_duration_max_ms[esc_index] = temperature_duration_max_ms[esc_index] % MILLISECONDS_PER_SECOND;

	} else if (esc_report.motor_temperature >= _motor_warn_temp) {
		if (!motor_temp_warn_thresh_hit[esc_index]) {
			_latest_health_data.motor_history.motor_temp_history.instance_counts_warn[esc_index]++;
			motor_temp_warn_thresh_hit[esc_index] = true;
		}

		temperature_duration_warn_ms[esc_index] += delta_ms;
		_latest_health_data.motor_history.motor_temp_history.temperature_duration_warn_s[esc_index] +=
			temperature_duration_warn_ms[esc_index] / MILLISECONDS_PER_SECOND;
		temperature_duration_warn_ms[esc_index] = temperature_duration_warn_ms[esc_index] % MILLISECONDS_PER_SECOND;
	}
}

void LifetimeHealthMonitor::_update_rpm_history(const esc_report_s &esc_report, uint16_t esc_index, uint32_t delta_ms)
{
	static uint16_t rpm_duration_high_ms[NUM_MOTORS] = {0};
	static uint16_t rpm_duration_medium_ms[NUM_MOTORS] = {0};

	// Update time spent at RPM ranges
	if (abs(esc_report.esc_rpm) >= _high_rpm) {
		rpm_duration_high_ms[esc_index] += delta_ms;
		_latest_health_data.motor_history.rpm_history.rpm_duration_high_s[esc_index] += rpm_duration_high_ms[esc_index] /
				MILLISECONDS_PER_SECOND;
		rpm_duration_high_ms[esc_index] = rpm_duration_high_ms[esc_index] % MILLISECONDS_PER_SECOND;

	} else if (abs(esc_report.esc_rpm) >= _med_rpm) {
		rpm_duration_medium_ms[esc_index] += delta_ms;
		_latest_health_data.motor_history.rpm_history.rpm_duration_medium_s[esc_index] += rpm_duration_medium_ms[esc_index] /
				MILLISECONDS_PER_SECOND;
		rpm_duration_medium_ms[esc_index] = rpm_duration_medium_ms[esc_index] % MILLISECONDS_PER_SECOND;
	}

	// Update total revolutions count
	_latest_health_data.motor_history.rpm_history.total_revolutions[esc_index] += ((uint64_t)abs(esc_report.esc_rpm) *
			delta_ms) / (60000ull);
}

void LifetimeHealthMonitor::_update_prop_dynamics_history(const esc_report_s &esc_report,
		const vehicle_odometry_s &vehicle_odometry, uint16_t esc_index, uint32_t delta_ms)
{
	wind_s wind;

	// set wind to 0 if not updated, because it may not be published all the time (e.g. when grounded)
	if (_wind_sub.updated()) {
		_wind_sub.copy(&wind);

	} else {
		wind = {};
	}

	float rad_per_s = (float)(abs(esc_report.esc_rpm) * 2.0 * M_PI / 60.0);

	static uint16_t max_flop_severity_duration_ms[NUM_MOTORS] = {0};
	static uint16_t moderate_flop_severity_duration_ms[NUM_MOTORS] = {0};

	// instantaneous_angular_velocity perpendicular to the prop axis = sqrt(w_x^2 + w_y^2)
	float instantaneous_w = sqrt(pow(vehicle_odometry.angular_velocity[0], 2) + pow(vehicle_odometry.angular_velocity[1],
				     2));

	// approximating ratio of gyroscopic torque prop rad/s * w_perpendicular (rad/s)
	float instantaneous_w_perpendicular = instantaneous_w * rad_per_s;

	if (instantaneous_w_perpendicular >= _high_gyroscopic_torque_threshold) {
		max_flop_severity_duration_ms[esc_index] += delta_ms;
		_latest_health_data.motor_history.prop_dynamics_history.flop_severity_severe_s[esc_index] +=
			max_flop_severity_duration_ms[esc_index] / MILLISECONDS_PER_SECOND;
		max_flop_severity_duration_ms[esc_index] = max_flop_severity_duration_ms[esc_index] % MILLISECONDS_PER_SECOND;

	} else if (instantaneous_w_perpendicular >= _moderate_gyroscopic_torque_threshold) {
		moderate_flop_severity_duration_ms[esc_index] += delta_ms;
		_latest_health_data.motor_history.prop_dynamics_history.flop_severity_moderate_s[esc_index] +=
			moderate_flop_severity_duration_ms[esc_index] / MILLISECONDS_PER_SECOND;
		moderate_flop_severity_duration_ms[esc_index] = moderate_flop_severity_duration_ms[esc_index] % MILLISECONDS_PER_SECOND;
	}

	static uint16_t max_asymmetric_lift_duration_ms[NUM_MOTORS] = {0};
	static uint16_t moderate_asymmetric_lift_duration_ms[NUM_MOTORS] = {0};

	// track prop asymmetric lift cost, taking wind into account
	float vehicle_velocity = sqrt(pow(vehicle_odometry.velocity[0] - wind.windspeed_north,
					  2) + pow(vehicle_odometry.velocity[1] - wind.windspeed_east, 2));

	float instantaneous_asymmetric_lift = vehicle_velocity * rad_per_s;

	if (instantaneous_asymmetric_lift >= _high_prop_asymmetric_lift_threshold) {
		max_asymmetric_lift_duration_ms[esc_index] += delta_ms;
		_latest_health_data.motor_history.prop_dynamics_history.asymmetric_lift_severe_s[esc_index] +=
			max_asymmetric_lift_duration_ms[esc_index] / MILLISECONDS_PER_SECOND;
		max_asymmetric_lift_duration_ms[esc_index] = max_asymmetric_lift_duration_ms[esc_index] % MILLISECONDS_PER_SECOND;

	} else if (instantaneous_asymmetric_lift >= _moderate_prop_asymmetric_lift_threshold) {
		moderate_asymmetric_lift_duration_ms[esc_index] += delta_ms;
		_latest_health_data.motor_history.prop_dynamics_history.asymmetric_lift_moderate_s[esc_index] +=
			moderate_asymmetric_lift_duration_ms[esc_index] / MILLISECONDS_PER_SECOND;
		moderate_asymmetric_lift_duration_ms[esc_index] = moderate_asymmetric_lift_duration_ms[esc_index] %
				MILLISECONDS_PER_SECOND;
	}
}


void LifetimeHealthMonitor::_update_all_esc_and_motor_history(const esc_status_s &esc_status,
		const vehicle_odometry_s &vehicle_odometry)
{
	static uint32_t _last_esc_report_time_ms[NUM_MOTORS] {0};

	if (esc_status.esc_count != NUM_MOTORS) {
		PX4_ERR("Unexpected number of ESCs: %d", esc_status.esc_count);
		return;
	}

	// only update history for the number of ESCs that are actually present
	for (int esc_index = 0; esc_index < esc_status.esc_count; esc_index++) {
		const esc_report_s &esc_report = esc_status.esc[esc_index];
		uint32_t latest_esc_timestamp_ms = esc_report.timestamp / MICROSECONDS_PER_MILLISECOND;

		if (_last_esc_report_time_ms[esc_index] > 0) {
			// only start accumulating once second esc_status is received
			// cap to 500 ms t_delta to reduce irreversible data corruption, though theoretically t_delta should only be 100 ms max
			uint32_t delta_ms = math::min(latest_esc_timestamp_ms - _last_esc_report_time_ms[esc_index], MAX_INTEGRATION_DELTA_MS);
			_update_electrical_history(esc_report, esc_index, delta_ms);
			_update_motor_temp_history(esc_report, esc_index, delta_ms);
			_update_esc_temp_history(esc_report, esc_index, delta_ms);
			_update_rpm_history(esc_report, esc_index, delta_ms);
			_update_prop_dynamics_history(esc_report, vehicle_odometry, esc_index, delta_ms);
		}

		_last_esc_report_time_ms[esc_index] = latest_esc_timestamp_ms;
	}
}

void LifetimeHealthMonitor::_update_flight_history()
{
	vehicle_air_data_s air_data{0};

	// only worth updating data when vehicle_air_data is updated
	if (_vehicle_air_data_sub.updated()) {
		_vehicle_air_data_sub.copy(&air_data);

	} else {
		return;
	}

	static uint64_t _previous_timestamp_us = 0;

	if (_previous_timestamp_us == 0) {
		_previous_timestamp_us = air_data.timestamp;
		return;
	}

	uint32_t delta_ms = (air_data.timestamp - _previous_timestamp_us) / MICROSECONDS_PER_MILLISECOND;
	delta_ms = math::min(delta_ms, MAX_INTEGRATION_DELTA_MS);

	static uint16_t altitude_high_duration_ms = 0;
	static uint16_t altitude_med_duration_ms = 0;
	static uint16_t altitude_low_duration_ms = 0;

	if (air_data.baro_alt_meter >= _altitude_m_very_high_threshold) {
		altitude_high_duration_ms += delta_ms;
		_latest_health_data.flight_history.high_altitude_duration_s.value() += altitude_high_duration_ms /
				MILLISECONDS_PER_SECOND;
		altitude_high_duration_ms = altitude_high_duration_ms % MILLISECONDS_PER_SECOND;

	} else if (air_data.baro_alt_meter >= _altitude_m_high_threshold) {
		altitude_med_duration_ms += delta_ms;
		_latest_health_data.flight_history.medium_altitude_duration_s.value() += altitude_med_duration_ms /
				MILLISECONDS_PER_SECOND;
		altitude_med_duration_ms = altitude_med_duration_ms % MILLISECONDS_PER_SECOND;

	} else if (air_data.baro_alt_meter >= _altitude_m_med_threshold) {
		altitude_low_duration_ms += delta_ms;
		_latest_health_data.flight_history.low_altitude_duration_s.value() += altitude_low_duration_ms /
				MILLISECONDS_PER_SECOND;
		altitude_low_duration_ms = altitude_low_duration_ms % MILLISECONDS_PER_SECOND;
	}


	static uint16_t ambient_temp_high_duration_ms = 0;
	static uint16_t ambient_temp_low_duration_ms = 0;

	if (air_data.ambient_temperature >= _ambient_temp_c_high_threshold) {
		ambient_temp_high_duration_ms += delta_ms;
		_latest_health_data.flight_history.high_ambient_temperature_duration_s.value() += ambient_temp_high_duration_ms /
				MILLISECONDS_PER_SECOND;
		ambient_temp_high_duration_ms = ambient_temp_high_duration_ms % MILLISECONDS_PER_SECOND;

	} else if (air_data.ambient_temperature <= _ambient_temp_c_low_threshold) {
		ambient_temp_low_duration_ms += delta_ms;
		_latest_health_data.flight_history.low_ambient_temperature_duration_s.value() += ambient_temp_low_duration_ms /
				MILLISECONDS_PER_SECOND;
		ambient_temp_low_duration_ms = ambient_temp_low_duration_ms % MILLISECONDS_PER_SECOND;
	}

	_previous_timestamp_us = air_data.timestamp;
}

void LifetimeHealthMonitor::_update_structural_history()
{
	static uint16_t vibration_duration_high_ms = 0;
	static uint16_t vibration_duration_moderate_ms = 0;
	static uint16_t vibration_duration_low_ms = 0;

	static uint64_t _previous_timestamp_us = 0;

	vehicle_imu_status_s imu_status;

	// only update if there is new data available
	if (_vehicle_imu_status_sub.updated()) {
		_vehicle_imu_status_sub.copy(&imu_status);

	} else {
		return;
	}

	if (_previous_timestamp_us == 0) {
		_previous_timestamp_us = imu_status.timestamp;
		return;
	}

	uint32_t delta_ms = (imu_status.timestamp - _previous_timestamp_us) / MICROSECONDS_PER_MILLISECOND;
	delta_ms = math::min(delta_ms, MAX_INTEGRATION_DELTA_MS);

	if (imu_status.accel_vibration_metric >= _vibration_severity_high) {
		vibration_duration_high_ms += delta_ms;
		_latest_health_data.structural_history.high_vibration_duration_s.value() += vibration_duration_high_ms /
				MILLISECONDS_PER_SECOND;
		vibration_duration_high_ms = vibration_duration_high_ms % MILLISECONDS_PER_SECOND;

	} else if (imu_status.accel_vibration_metric >= _vibration_severity_moderate) {
		vibration_duration_moderate_ms += delta_ms;
		_latest_health_data.structural_history.moderate_vibration_duration_s.value() += vibration_duration_moderate_ms /
				MILLISECONDS_PER_SECOND;
		vibration_duration_moderate_ms = vibration_duration_moderate_ms % MILLISECONDS_PER_SECOND;

	} else {
		vibration_duration_low_ms += delta_ms;
		_latest_health_data.structural_history.low_vibration_duration_s.value() += vibration_duration_low_ms /
				MILLISECONDS_PER_SECOND;
		vibration_duration_low_ms = vibration_duration_low_ms % MILLISECONDS_PER_SECOND;
	}

	_previous_timestamp_us = imu_status.timestamp;
}

/**
 * Determine thresholds based on SYS_AUTOSTART value
 */
void LifetimeHealthMonitor::_determine_threshold_values()
{
	param_t param_sys_autostart = param_find("SYS_AUTOSTART");
	int32_t param_sys_autostart_value = 0;

	if (param_sys_autostart != PARAM_INVALID) {
		param_get(param_sys_autostart, &param_sys_autostart_value);
	}

	switch (param_sys_autostart_value) {
	case 1002003: // Astro Black
	case 1002004: // Astro Blue
		break;

	case 1002013: // Astro Max
	case 1002014: // Astro Max Blue
		break;

	case 4510: // Alta X Futaba
	case 4511: // Alta X Pilot Pro
		break;

	default:
		// Use default values
		PX4_INFO("Using default threshold values for SYS_AUTOSTART: %d", (int)param_sys_autostart_value);
		break;
	}
}

void LifetimeHealthMonitor::_publish_lifetime_health_stats(lifetime_health_data_s &health_data)
{
	lifetime_health_stats_s health_stats{};
	health_stats.timestamp = hrt_absolute_time();

	// Core data
	health_stats.format_version = health_data.format_version.value();
	health_stats.boot_cycles = health_data.boot_cycles.value();
	health_stats.init_timestamp_utc_s = health_data.init_timestamp.value();
	health_stats.on_duration_s = health_data.on_duration_s.value();
	health_stats.arm_cycles = health_data.flight_history.arm_cycles.value();
	health_stats.flight_duration_s = health_data.flight_history.flight_duration_s.value();

	// Flight history
	health_stats.altitude_duration_s[0] = health_data.flight_history.low_altitude_duration_s.value();
	health_stats.altitude_duration_s[1] = health_data.flight_history.medium_altitude_duration_s.value();
	health_stats.altitude_duration_s[2] = health_data.flight_history.high_altitude_duration_s.value();

	health_stats.ambient_temperature_duration_s[0] = health_data.flight_history.low_ambient_temperature_duration_s.value();
	health_stats.ambient_temperature_duration_s[1] = health_data.flight_history.high_ambient_temperature_duration_s.value();

	// Structural history
	health_stats.vibration_severity_duration_s[0] = health_data.structural_history.low_vibration_duration_s.value();
	health_stats.vibration_severity_duration_s[1] = health_data.structural_history.moderate_vibration_duration_s.value();
	health_stats.vibration_severity_duration_s[2] = health_data.structural_history.high_vibration_duration_s.value();

	// Motor history
	for (int i = 0; i < NUM_MOTORS; ++i) {
		// RPM history
		health_stats.revs_per_motor[i] = health_data.motor_history.rpm_history.total_revolutions[i];
		health_stats.med_rpm_duration_s[i] = health_data.motor_history.rpm_history.rpm_duration_medium_s[i];
		health_stats.high_rpm_duration_s[i] = health_data.motor_history.rpm_history.rpm_duration_high_s[i];

		// ESC electrical history
		health_stats.esc_voltage_max[i] = health_data.motor_history.electrical_history.max_voltage[i];
		health_stats.esc_voltage_min[i] = health_data.motor_history.electrical_history.min_voltage[i];
		health_stats.esc_current_max[i] = health_data.motor_history.electrical_history.max_current[i];
		health_stats.esc_current_min[i] = health_data.motor_history.electrical_history.min_current[i];
		health_stats.esc_current_warn_events[i] = health_data.motor_history.electrical_history.instance_counts_warn[i];
		health_stats.esc_current_max_events[i] = health_data.motor_history.electrical_history.instance_counts_max[i];
		health_stats.esc_current_warn_duration_s[i] = health_data.motor_history.electrical_history.current_duration_warn_s[i];
		health_stats.esc_current_max_duration_s[i] = health_data.motor_history.electrical_history.current_duration_max_s[i];

		// ESC temperature history
		health_stats.esc_temp_max[i] = health_data.motor_history.esc_temp_history.max_temp[i];
		health_stats.esc_temp_warn_events[i] = health_data.motor_history.esc_temp_history.instance_counts_warn[i];
		health_stats.esc_temp_max_events[i] = health_data.motor_history.esc_temp_history.instance_counts_max[i];
		health_stats.esc_temp_warn_duration_s[i] = health_data.motor_history.esc_temp_history.temperature_duration_warn_s[i];
		health_stats.esc_temp_max_duration_s[i] = health_data.motor_history.esc_temp_history.temperature_duration_max_s[i];

		// Motor temperature history
		health_stats.motor_temp_max[i] = health_data.motor_history.motor_temp_history.max_temp[i];
		health_stats.motor_temp_warn_events[i] = health_data.motor_history.motor_temp_history.instance_counts_warn[i];
		health_stats.motor_temp_max_events[i] = health_data.motor_history.motor_temp_history.instance_counts_max[i];
		health_stats.motor_temp_warn_duration_s[i] =
			health_data.motor_history.motor_temp_history.temperature_duration_warn_s[i];
		health_stats.motor_temp_max_duration_s[i] = health_data.motor_history.motor_temp_history.temperature_duration_max_s[i];

		// Prop dynamics history
		health_stats.med_prop_flop_duration_s[i] =
			health_data.motor_history.prop_dynamics_history.flop_severity_moderate_s[i];
		health_stats.high_prop_flop_duration_s[i] = health_data.motor_history.prop_dynamics_history.flop_severity_severe_s[i];
		health_stats.med_prop_asymmetric_lift_duration_s[i] =
			health_data.motor_history.prop_dynamics_history.asymmetric_lift_moderate_s[i];
		health_stats.high_prop_asymmetric_lift_duration_s[i] =
			health_data.motor_history.prop_dynamics_history.asymmetric_lift_severe_s[i];
	}

	_lifetime_health_stats_pub.publish(health_stats);
}

int LifetimeHealthMonitor::print_status()
{
	PX4_INFO("Lifetime Health Metrics:");
	PX4_INFO("Module Init Timestamp: %" PRIu64 "s", _latest_health_data.init_timestamp.value());
	PX4_INFO("Total On Time: %" PRIu32 "s", _latest_health_data.on_duration_s.value());
	PX4_INFO("Total Flight Time: %" PRIu32 "s", _latest_health_data.flight_history.flight_duration_s.value());
	PX4_INFO("Total Arm Cycles: %" PRIu32, _latest_health_data.flight_history.arm_cycles.value());

	// PX4_INFO("Use 'listener lifetime_health_stats' to see detailed statistics");
	return 0;
}

int LifetimeHealthMonitor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LifetimeHealthMonitor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
    ### Description
    Information about lifetime health of drone.

    )DESCR_STR");

	PRINT_MODULE_USAGE_NAME("lifetime_health_monitor", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int lifetime_health_monitor_main(int argc, char *argv[])
{
	return ModuleBase::main(LifetimeHealthMonitor::desc, argc, argv);
}
