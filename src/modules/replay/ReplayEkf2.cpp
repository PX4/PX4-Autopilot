/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>

#include <lib/parameters/param.h>

// for ekf2 replay
#include <uORB/topics/airspeed.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_optical_flow.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_odometry.h>

#include "ReplayEkf2.hpp"

namespace px4
{

bool
ReplayEkf2::handleTopicUpdate(Subscription &sub, void *data, std::ifstream &replay_file)
{
	if (sub.orb_meta == ORB_ID(ekf2_timestamps)) {
		ekf2_timestamps_s ekf2_timestamps;
		memcpy(&ekf2_timestamps, data, sub.orb_meta->o_size);

		if (!publishEkf2Topics(ekf2_timestamps, replay_file)) {
			return false;
		}

		// Wait for modules to process the data
		px4_lockstep_wait_for_components();

		return true;

	} else if (sub.orb_meta == ORB_ID(sensor_combined) && !_ekf2_timestamps_exists) {
		// No ekf2_timestamps topic, publish with approximate timestamps
		sensor_combined_s sensor_combined;
		memcpy(&sensor_combined, data, sub.orb_meta->o_size);

		if (!publishEkf2Topics(sensor_combined, replay_file)) {
			return false;
		}

		// Wait for modules to process the data
		px4_lockstep_wait_for_components();

		return true;

	} else if (sub.orb_meta == ORB_ID(vehicle_status) || sub.orb_meta == ORB_ID(vehicle_land_detected)
		   || sub.orb_meta == ORB_ID(vehicle_gps_position)) {
		return publishTopic(sub, data);
	} // else: do not publish

	return false;
}

void
ReplayEkf2::onSubscriptionAdded(Subscription &sub, uint16_t msg_id)
{
	if (sub.orb_meta == ORB_ID(sensor_combined)) {
		_sensor_combined_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(airspeed)) {
		_airspeed_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(airspeed_validated)) {
		_airspeed_validated_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(distance_sensor)) {
		_distance_sensor_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(vehicle_optical_flow)) {
		_optical_flow_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(vehicle_air_data)) {
		_vehicle_air_data_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(vehicle_magnetometer)) {
		_vehicle_magnetometer_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(vehicle_visual_odometry)) {
		_vehicle_visual_odometry_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(aux_global_position)) {
		_aux_global_position_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(vehicle_local_position_groundtruth)) {
		_vehicle_local_position_groundtruth_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(vehicle_attitude_groundtruth)) {
		_vehicle_attitude_groundtruth_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(vehicle_global_position_groundtruth)) {
		_vehicle_global_position_groundtruth_msg_id = msg_id;

	} else if (sub.orb_meta == ORB_ID(ekf2_timestamps)) {
		_ekf2_timestamps_exists = true;
	}

	// the main loop should only handle publication of the following topics, the sensor topics are
	// handled separately in publishEkf2Topics()
	// Note: the GPS is not treated here since not missing data is more important than the accuracy of the timestamp
	sub.ignored = sub.orb_meta != ORB_ID(ekf2_timestamps) && sub.orb_meta != ORB_ID(vehicle_status)
		      && sub.orb_meta != ORB_ID(vehicle_land_detected) && sub.orb_meta != ORB_ID(vehicle_gps_position)
		      && sub.orb_meta != ORB_ID(sensor_combined);
}

bool
ReplayEkf2::publishEkf2Topics(sensor_combined_s &sensor_combined, std::ifstream &replay_file)
{
	findTimestampAndPublish(sensor_combined.timestamp, _airspeed_msg_id, replay_file);
	findTimestampAndPublish(sensor_combined.timestamp, _distance_sensor_msg_id, replay_file);
	findTimestampAndPublish(sensor_combined.timestamp, _optical_flow_msg_id, replay_file);
	findTimestampAndPublish(sensor_combined.timestamp, _vehicle_air_data_msg_id, replay_file);
	findTimestampAndPublish(sensor_combined.timestamp, _vehicle_magnetometer_msg_id, replay_file);
	findTimestampAndPublish(sensor_combined.timestamp, _vehicle_visual_odometry_msg_id, replay_file);
	findTimestampAndPublish(sensor_combined.timestamp, _aux_global_position_msg_id, replay_file);

	// sensor_combined: publish last because ekf2 is polling on this
	if (_last_sensor_combined_timestamp > 0) {
		// Some samples might be missing so compensate for this by holding the last value
		const uint32_t true_dt = static_cast<uint32_t>(sensor_combined.timestamp - _last_sensor_combined_timestamp);
		sensor_combined.gyro_integral_dt = true_dt;
		sensor_combined.accelerometer_integral_dt = true_dt;
	}

	_last_sensor_combined_timestamp = sensor_combined.timestamp;

	publishTopic(*_subscriptions[_sensor_combined_msg_id], &sensor_combined);

	return true;
}

bool
ReplayEkf2::publishEkf2Topics(const ekf2_timestamps_s &ekf2_timestamps, std::ifstream &replay_file)
{
	auto handle_sensor_publication = [&](int16_t timestamp_relative, uint16_t msg_id) {
		if (timestamp_relative != ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID) {
			// timestamp_relative is given in 0.1 ms
			uint64_t t = timestamp_relative * 100 + ekf2_timestamps.timestamp;
			findTimestampAndPublish(t, msg_id, replay_file);
		}
	};

	handle_sensor_publication(ekf2_timestamps.airspeed_timestamp_rel, _airspeed_msg_id);
	handle_sensor_publication(ekf2_timestamps.airspeed_validated_timestamp_rel, _airspeed_validated_msg_id);
	handle_sensor_publication(ekf2_timestamps.distance_sensor_timestamp_rel, _distance_sensor_msg_id);
	handle_sensor_publication(ekf2_timestamps.optical_flow_timestamp_rel, _optical_flow_msg_id);
	handle_sensor_publication(ekf2_timestamps.vehicle_air_data_timestamp_rel, _vehicle_air_data_msg_id);
	handle_sensor_publication(ekf2_timestamps.vehicle_magnetometer_timestamp_rel, _vehicle_magnetometer_msg_id);
	handle_sensor_publication(ekf2_timestamps.visual_odometry_timestamp_rel, _vehicle_visual_odometry_msg_id);
	handle_sensor_publication(0, _aux_global_position_msg_id);
	handle_sensor_publication(0, _vehicle_local_position_groundtruth_msg_id);
	handle_sensor_publication(0, _vehicle_global_position_groundtruth_msg_id);
	handle_sensor_publication(0, _vehicle_attitude_groundtruth_msg_id);

	// sensor_combined: publish last because ekf2 is polling on this
	if (!findTimestampAndPublish(ekf2_timestamps.timestamp, _sensor_combined_msg_id, replay_file)) {
		if (_sensor_combined_msg_id == msg_id_invalid) {
			// subscription not found yet or sensor_combined not contained in log
			return false;

		} else if (!_subscriptions[_sensor_combined_msg_id]->orb_meta) {
			return false; // read past end of file

		} else {
			// we should publish a topic, just publish the same again
			readTopicDataToBuffer(*_subscriptions[_sensor_combined_msg_id], replay_file);
			publishTopic(*_subscriptions[_sensor_combined_msg_id], _read_buffer.data());
		}
	}

	return true;
}

bool
ReplayEkf2::findTimestampAndPublish(uint64_t timestamp, uint16_t msg_id, std::ifstream &replay_file)
{
	if (msg_id == msg_id_invalid) {
		// could happen if a topic is not logged
		return false;
	}

	Subscription &sub = *_subscriptions[msg_id];

	bool topic_published = false;

	while (sub.next_timestamp <= timestamp && sub.orb_meta) {
		if (!sub.published) {
			if (sub.next_timestamp != timestamp) {
				// Not the exact sample, publish but notify error
				PX4_DEBUG("No timestamp match found for topic %s (%" PRIu64 ", %" PRIu64 ")\n", sub.orb_meta->o_name,
					  sub.next_timestamp,
					  timestamp);
				++sub.approx_timestamp_counter;
			}

			readTopicDataToBuffer(sub, replay_file);
			publishTopic(sub, _read_buffer.data());
			topic_published = true;
		}

		nextDataMessage(replay_file, sub, msg_id);
	}

	return topic_published;
}

void
ReplayEkf2::onEnterMainLoop()
{
	_speed_factor = 0.f; // iterate as fast as possible

	// disable parameter auto save
	param_control_autosave(false);
}

void
ReplayEkf2::onExitMainLoop()
{
	// print statistics
	auto print_sensor_statistics = [this](uint16_t msg_id, const char *name) {
		if (msg_id != msg_id_invalid) {
			Subscription &sub = *_subscriptions[msg_id];

			if (sub.publication_counter > 0 || sub.approx_timestamp_counter > 0) {
				PX4_INFO("%s: %i (%i)", name, sub.publication_counter, sub.approx_timestamp_counter);
			}
		}
	};

	PX4_INFO("");

	if (!_ekf2_timestamps_exists) {
		PX4_INFO("/!\\ Approximate replay (ekf2_timestamps not found)\n");
	}

	PX4_INFO("Topic, Num Published (Num approximate timestamp found):");

	print_sensor_statistics(_airspeed_msg_id, "airspeed");
	print_sensor_statistics(_airspeed_validated_msg_id, "airspeed_validated");
	print_sensor_statistics(_distance_sensor_msg_id, "distance_sensor");
	print_sensor_statistics(_optical_flow_msg_id, "vehicle_optical_flow");
	print_sensor_statistics(_sensor_combined_msg_id, "sensor_combined");
	print_sensor_statistics(_vehicle_air_data_msg_id, "vehicle_air_data");
	print_sensor_statistics(_vehicle_magnetometer_msg_id, "vehicle_magnetometer");
	print_sensor_statistics(_vehicle_visual_odometry_msg_id, "vehicle_visual_odometry");
	print_sensor_statistics(_aux_global_position_msg_id, "aux_global_position");
}

} // namespace px4
