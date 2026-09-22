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

#pragma once

#include "Replay.hpp"

#include <pthread.h>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/sensor_combined.h>

namespace px4
{

/**
 * @class ReplayEkf2
 * replay specialization for Ekf2 replay
 */
class ReplayEkf2 : public Replay
{
public:
protected:

	void onEnterMainLoop() override;
	void onExitMainLoop() override;

	/**
	 * handle ekf2 topic publication in ekf2 replay mode
	 * @param sub
	 * @param data
	 * @param replay_file file currently replayed (file seek position should be considered arbitrary after this call)
	 * @return true if published, false otherwise
	 */
	bool handleTopicUpdate(Subscription &sub, void *data, std::ifstream &replay_file) override;

	void onSubscriptionAdded(Subscription &sub, uint16_t msg_id) override;

	uint64_t getTimestampOffset() override
	{
		// avoid offsetting timestamps as we use them to compare against the log
		return 0;
	}
private:

	/**
	 * Counts ekf2_timestamps publications (one per IMU sample processed by ekf2)
	 */
	class Ekf2UpdateSignal : public uORB::SubscriptionCallback
	{
	public:
		Ekf2UpdateSignal() : SubscriptionCallback(ORB_ID(ekf2_timestamps)) {}
		~Ekf2UpdateSignal() override;

		void call(unsigned generation) override;

		unsigned updates();

		/**
		 * block until more than updates_before publications were seen
		 * @param timeout_ms wall-clock timeout (lockstep time only advances through the replay thread)
		 * @return false on timeout
		 */
		bool waitForUpdateAfter(unsigned updates_before, unsigned timeout_ms);

	private:
		pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;
		pthread_cond_t _cond = PTHREAD_COND_INITIALIZER;
		unsigned _updates{0};
	};

	/**
	 * publish a sensor_combined sample and block until ekf2 has processed it
	 */
	void publishSensorCombined(Subscription &sub, void *data);

	bool publishEkf2Topics(const ekf2_timestamps_s &ekf2_timestamps, std::ifstream &replay_file);

	bool publishEkf2Topics(sensor_combined_s &sensors_combined, std::ifstream &replay_file);

	/**
	 * find the next message for a subscription that matches a given timestamp and publish it
	 * @param timestamp in microseconds
	 * @param msg_id
	 * @param replay_file file currently replayed (file seek position should be considered arbitrary after this call)
	 * @return true if timestamp found and published
	 */
	bool findTimestampAndPublish(uint64_t timestamp, uint16_t msg_id, std::ifstream &replay_file);

	/** same for a topic with multiple instances */
	bool findTimestampAndPublish(uint64_t timestamp, const std::vector<uint16_t> &msg_ids, std::ifstream &replay_file);

	/**
	 * publish the sensor_combined sample of the given ekf2 cycle. Logged samples without an ekf2_timestamps entry are
	 * published in a cycle of their own (ekf2 processed them in flight, only their ekf2_timestamps message is missing),
	 * a sample missing from the log is synthesized from the previous one with the integration time covering the gap.
	 * @return true if a sample was published
	 */
	bool publishMatchingSensorCombined(uint64_t timestamp, std::ifstream &replay_file);

	static constexpr uint16_t msg_id_invalid = 0xffff;

	uint16_t _airspeed_msg_id = msg_id_invalid;
	uint16_t _airspeed_validated_msg_id = msg_id_invalid;
	uint16_t _sensor_combined_msg_id = msg_id_invalid;
	uint16_t _vehicle_air_data_msg_id = msg_id_invalid;
	uint16_t _vehicle_magnetometer_msg_id = msg_id_invalid;
	uint16_t _vehicle_visual_odometry_msg_id = msg_id_invalid;
	uint16_t _ranging_beacon_msg_id = msg_id_invalid;
	uint16_t _vehicle_local_position_groundtruth_msg_id = msg_id_invalid;
	uint16_t _vehicle_global_position_groundtruth_msg_id = msg_id_invalid;
	uint16_t _vehicle_attitude_groundtruth_msg_id = msg_id_invalid;
	uint16_t _vehicle_gps_position_msg_id = msg_id_invalid;
	uint16_t _vehicle_land_detected_msg_id = msg_id_invalid;
	uint16_t _vehicle_status_msg_id = msg_id_invalid;
	uint16_t _sensor_selection_msg_id = msg_id_invalid;
	uint16_t _launch_detection_status_msg_id = msg_id_invalid;
	uint16_t _estimator_fusion_control_msg_id = msg_id_invalid;

	// multi-instance topics: one msg_id per instance
	std::vector<uint16_t> _distance_sensor_msg_ids;
	std::vector<uint16_t> _optical_flow_msg_ids;
	std::vector<uint16_t> _aux_global_position_msg_ids;

	bool _ekf2_timestamps_exists{false};
	uint64_t _last_sensor_combined_timestamp{0};

	static constexpr unsigned kEkf2UpdateTimeoutMs = 1000;
	static constexpr unsigned kEkf2MaxConsecutiveTimeouts = 3;

	Ekf2UpdateSignal _ekf2_update_signal;
	bool _ekf2_sync_enabled{true};
	unsigned _ekf2_update_timeouts{0};
	unsigned _ekf2_consecutive_timeouts{0};

	unsigned _sensor_combined_unmatched{0};
	unsigned _sensor_combined_missing{0};
	sensor_combined_s _last_sensor_combined{};
};

} //namespace px4
