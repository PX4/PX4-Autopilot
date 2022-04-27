/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <uORB/uORB.h>
#include <uORB/topics/uORBTopics.hpp>

namespace px4
{
namespace logger
{

enum class SDLogProfileMask : int32_t {
	DEFAULT =               1 << 0,
	ESTIMATOR_REPLAY =      1 << 1,
	THERMAL_CALIBRATION =   1 << 2,
	SYSTEM_IDENTIFICATION = 1 << 3,
	HIGH_RATE =             1 << 4,
	DEBUG_TOPICS =          1 << 5,
	SENSOR_COMPARISON =     1 << 6,
	VISION_AND_AVOIDANCE =  1 << 7,
	RAW_IMU_GYRO_FIFO =     1 << 8,
	RAW_IMU_ACCEL_FIFO =    1 << 9,
	MAVLINK_TUNNEL =        1 << 10
};

enum class MissionLogType : int32_t {
	Disabled =               0,
	Complete =               1,
	Geotagging =             2
};

inline bool operator&(SDLogProfileMask a, SDLogProfileMask b)
{
	return static_cast<int32_t>(a) & static_cast<int32_t>(b);
}

/**
 * @class LoggedTopics
 * Contains the list of configured topics
 */
class LoggedTopics
{
public:
	static constexpr int MAX_TOPICS_NUM = 255; /**< Maximum number of logged topics */

	static constexpr int MAX_EXCLUDED_OPTIONAL_TOPICS_NUM = 40;

	struct RequestedSubscription {
		uint16_t interval_ms;
		uint8_t instance;
		ORB_ID id{ORB_ID::INVALID};
	};
	struct RequestedSubscriptionArray {
		RequestedSubscription sub[MAX_TOPICS_NUM];
		int count{0};

		uint8_t excluded_optional_topic_ids[MAX_EXCLUDED_OPTIONAL_TOPICS_NUM];
		int num_excluded_optional_topic_ids{0};
	};

	LoggedTopics() = default;
	~LoggedTopics() = default;

	/**
	 * Add topic subscriptions based on the configured mission log type.
	 * Must be initialized first
	 */
	void initialize_mission_topics(MissionLogType mission_log_type);

	bool initialize_logged_topics(SDLogProfileMask profile);

	const RequestedSubscriptionArray &subscriptions() const { return _subscriptions; }
	int numMissionSubscriptions() const { return _num_mission_subs; }

	void set_rate_factor(float rate_factor) { _rate_factor = rate_factor; }

private:

	/**
	 * Add a topic to be logged.
	 * @param name topic name
	 * @param interval limit in milliseconds if >0, otherwise log as fast as the topic is updated.
	 * @param instance orb topic instance
	 * @param optional if true, the topic is only added if it exists
	 * @return true on success
	 */
	bool add_topic(const char *name, uint16_t interval_ms = 0, uint8_t instance = 0, bool optional = false);

	bool add_optional_topic(const char *name, uint16_t interval_ms = 0, uint8_t instance = 0)
	{
		return add_topic(name, interval_ms, instance, true);
	}

	/**
	 * Add a topic to be logged.
	 * @param name topic name
	 * @param interval limit in milliseconds if >0, otherwise log as fast as the topic is updated.
	 * @param instance orb topic instance
	 * @param max_num_instances the max multi-instance to add.
	 * @param optional if true, the topic is only added if it exists
	 * @return true on success
	 */
	bool add_topic_multi(const char *name, uint16_t interval_ms = 0, uint8_t max_num_instances = ORB_MULTI_MAX_INSTANCES,
			     bool optional = false);

	bool add_optional_topic_multi(const char *name, uint16_t interval_ms = 0,
				      uint8_t max_num_instances = ORB_MULTI_MAX_INSTANCES)
	{
		return add_topic_multi(name, interval_ms, max_num_instances, true);
	}

	/**
	 * Parse a file containing a list of uORB topics to log, calling add_topic for each
	 * @param fname name of file
	 * @return number of topics added
	 */
	int add_topics_from_file(const char *fname);

	/**
	 * Add a topic to be logged for the mission log (it's also added to the full log).
	 * The interval is expected to be 0 or large (in the order of 0.1 seconds or higher).
	 * Must be called before all other topics are added.
	 * @param name topic name
	 * @param interval limit rate if >0 [ms], otherwise log as fast as the topic is updated.
	 */
	void add_mission_topic(const char *name, uint16_t interval_ms = 0);

	/**
	 * Add topic subscriptions based on the profile configuration
	 */
	void initialize_configured_topics(SDLogProfileMask profile);

	void add_default_topics();
	void add_estimator_replay_topics();
	void add_thermal_calibration_topics();
	void add_system_identification_topics();
	void add_high_rate_topics();
	void add_debug_topics();
	void add_sensor_comparison_topics();
	void add_vision_and_avoidance_topics();
	void add_raw_imu_gyro_fifo();
	void add_raw_imu_accel_fifo();
	void add_mavlink_tunnel();

	/**
	 * add a logged topic (called by add_topic() above).
	 * @return true on success
	 */
	bool add_topic(const orb_metadata *topic, uint16_t interval_ms = 0, uint8_t instance = 0, bool optional = false);

	RequestedSubscriptionArray _subscriptions;
	int _num_mission_subs{0};
	float _rate_factor{1.0f};

	bool _dynamic_control_allocation{false};
};

} //namespace logger
} //namespace px4
