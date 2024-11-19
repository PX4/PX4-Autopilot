/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file mavlink_mission.h
 * Implementation of the MAVLink mission protocol.
 * Documentation:
 * - http://qgroundcontrol.org/mavlink/mission_interface
 * - http://qgroundcontrol.org/mavlink/waypoint_protocol
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

#pragma once

#include <dataman_client/DatamanClient.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/mission_result.h>

#include "mavlink_bridge_header.h"
#include "mavlink_rate_limiter.h"

enum MAVLINK_WPM_STATES {
	MAVLINK_WPM_STATE_IDLE = 0,
	MAVLINK_WPM_STATE_SENDLIST,
	MAVLINK_WPM_STATE_GETLIST,
	MAVLINK_WPM_STATE_ENUM_END
};

enum MAVLINK_WPM_CODES {
	MAVLINK_WPM_CODE_OK = 0,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_ACTION_NOT_SUPPORTED,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_FRAME_NOT_SUPPORTED,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_OUT_OF_BOUNDS,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_MAX_NUMBER_EXCEEDED,
	MAVLINK_WPM_CODE_ENUM_END
};

static constexpr uint64_t MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT = 5000000; ///< Protocol action timeout in us
static constexpr uint64_t MAVLINK_MISSION_RETRY_TIMEOUT_DEFAULT = 250000; ///< Protocol retry timeout in us

class Mavlink;

class MavlinkMissionManager
{
public:
	explicit MavlinkMissionManager(Mavlink &mavlink);

	~MavlinkMissionManager() = default;

	/**
	 * Handle sending of messages. Call this regularly at a fixed frequency.
	 * @param t current time
	 */
	void send();

	void handle_message(const mavlink_message_t *msg);

	void check_active_mission(void);

private:
	enum MAVLINK_WPM_STATES _state {MAVLINK_WPM_STATE_IDLE};	///< Current state
	enum MAV_MISSION_TYPE _mission_type {MAV_MISSION_TYPE_MISSION};	///< mission type of current transmission (only one at a time possible)

	DatamanClient _dataman_client{};

	uint64_t		_time_last_recv{0};
	uint64_t		_time_last_sent{0};

	uint8_t			_reached_sent_count{0};			///< last time when the vehicle reached a waypoint

	bool			_int_mode{false};			///< Use accurate int32 instead of float

	unsigned		_filesystem_errcount{0};		///< File system error count

	static dm_item_t	_mission_dataman_id;			///< Global Dataman storage ID for active mission
	static dm_item_t 	_safepoint_dataman_id; 			///< Global dataman storage id for active safepoints
	static dm_item_t 	_fence_dataman_id; 			///< Global dataman storage id for active geofence
	dm_item_t		_my_mission_dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_0};		///< class Dataman storage ID for mission
	dm_item_t		_my_safepoint_dataman_id{DM_KEY_SAFE_POINTS_0};			///< class Dataman storage ID for safepoints
	dm_item_t		_my_fence_dataman_id{DM_KEY_FENCE_POINTS_0};			///< class Dataman storage ID for geofence

	static bool		_dataman_init;				///< Dataman initialized

	static uint16_t		_count[3];				///< Count of items in (active) mission for each MAV_MISSION_TYPE
	static uint32_t		_crc32[3];				///< Checksum of items in (active) mission for each MAV_MISSION_TYPE
	static int32_t		_current_seq;				///< Current item sequence in active mission

	int32_t			_last_reached{-1};			///< Last reached waypoint in active mission (-1 means nothing reached)

	dm_item_t		_transfer_dataman_id{DM_KEY_WAYPOINTS_OFFBOARD_1};	///< Dataman storage ID for current transmission

	uint16_t		_transfer_count{0};			///< Items count in current transmission
	uint32_t		_transfer_current_crc32{0};		///< Current CRC32 checksum of current transmission
	uint16_t		_transfer_seq{0};			///< Item sequence in current transmission

	int32_t			_transfer_current_seq{-1};		///< Current item ID for current transmission (-1 means not initialized)

	uint8_t			_transfer_partner_sysid{0};		///< Partner system ID for current transmission
	uint8_t			_transfer_partner_compid{0};		///< Partner component ID for current transmission
	int32_t 		_transfer_land_start_marker{-1}; 	///< index of land start mission item in current transmission (if unavailable, index of land mission item, -1 otherwise)
	int32_t 		_transfer_land_marker{-1}; 		///< index of land mission item in current transmission (-1 if unavailable)

	static bool		_transfer_in_progress;			///< Global variable checking for current transmission

	uORB::Subscription	_mission_result_sub{ORB_ID(mission_result)};
	uORB::SubscriptionData<mission_s> 	_mission_sub{ORB_ID(mission)};

	uORB::Publication<mission_s>	_offboard_mission_pub{ORB_ID(mission)};

	int32_t 		_land_start_marker{-1}; 	///< index of loaded land start mission item (if unavailable, index of land mission item, -1 otherwise)
	int32_t 		_land_marker{-1}; 		///< index of loaded land mission item (-1 if unavailable)

	MavlinkRateLimiter	_slow_rate_limiter{1000 * 1000};		///< Rate limit sending of the current WP sequence to 1 Hz

	Mavlink &_mavlink;

	static constexpr unsigned int	FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT =
		2;	///< Error count limit before stopping to report FS errors
	static constexpr uint16_t	MAX_COUNT[] = {
		DM_KEY_WAYPOINTS_OFFBOARD_0_MAX,
		DM_KEY_FENCE_POINTS_MAX,
		DM_KEY_SAFE_POINTS_MAX
	};	/**< Maximum number of mission items for each type
					(fence & safe points use the first item for the stats) */

	/** get the maximum number of item count for the current _mission_type */
	uint16_t current_max_item_count();

	/** get the number of item count for the current _mission_type */
	uint16_t current_item_count();

	/** get the crc32 checksum for the current _mission_type */
	uint32_t get_current_mission_type_crc();

	/* do not allow top copying this class */
	MavlinkMissionManager(MavlinkMissionManager &);
	MavlinkMissionManager &operator = (const MavlinkMissionManager &);

	void init_offboard_mission(const mission_s &mission_state);

	void update_active_mission(dm_item_t mission_dataman_id, uint16_t count, int32_t seq, uint32_t crc32,
				   bool write_to_dataman = true);

	/** store the geofence count to dataman */
	int update_geofence_count(dm_item_t fence_dataman_id, unsigned count, uint32_t crc32);

	/** store the safepoint count to dataman */
	int update_safepoint_count(dm_item_t safepoint_dataman_id, unsigned count, uint32_t crc32);

	/** load geofence stats from dataman */
	bool load_geofence_stats();

	/** load safe point stats from dataman */
	bool load_safepoint_stats();

	/**
	 *  @brief Sends an waypoint ack message
	 */
	void send_mission_ack(uint8_t sysid, uint8_t compid, uint8_t type, uint32_t opaque_id = 0U);

	/**
	 *  @brief Broadcasts the new target waypoint and directs the MAV to fly there
	 *
	 *  This function broadcasts its new active waypoint sequence number and
	 *  sends a message to the controller, advising it to fly to the coordinates
	 *  of the waypoint with a given orientation
	 *
	 *  @param seq The waypoint sequence number the MAV should fly to.
	 */
	void send_mission_current(uint16_t seq);

	void send_mission_count(uint8_t sysid, uint8_t compid, uint16_t count, MAV_MISSION_TYPE mission_type,
				uint32_t opaque_id);

	void send_mission_item(uint8_t sysid, uint8_t compid, uint16_t seq);

	void send_mission_request(uint8_t sysid, uint8_t compid, uint16_t seq);

	/**
	 *  @brief emits a message that a waypoint reached
	 *
	 *  This function broadcasts a message that a waypoint is reached.
	 *
	 *  @param seq The waypoint sequence number the MAV has reached.
	 */
	void send_mission_item_reached(uint16_t seq);

	void handle_mission_ack(const mavlink_message_t *msg);

	void handle_mission_set_current(const mavlink_message_t *msg);

	void handle_mission_request_list(const mavlink_message_t *msg);

	void handle_mission_request(const mavlink_message_t *msg);
	void handle_mission_request_int(const mavlink_message_t *msg);
	void handle_mission_request_both(const mavlink_message_t *msg);

	void handle_mission_count(const mavlink_message_t *msg);

	void handle_mission_item(const mavlink_message_t *msg);
	void handle_mission_item_int(const mavlink_message_t *msg);
	void handle_mission_item_both(const mavlink_message_t *msg);

	void handle_mission_clear_all(const mavlink_message_t *msg);

	/**
	 * Parse mavlink MISSION_ITEM message to get mission_item_s.
	 *
	 * @param mavlink_mission_item pointer to mavlink_mission_item_t or mavlink_mission_item_int_t
	 *			       depending on _int_mode
	 * @param mission_item	       pointer to mission_item to construct
	 */
	int parse_mavlink_mission_item(const mavlink_mission_item_t *mavlink_mission_item, struct mission_item_s *mission_item);

	/**
	 * Format mission_item_s as mavlink MISSION_ITEM(_INT) message.
	 *
	 * @param mission_item:		pointer to the existing mission item
	 * @param mavlink_mission_item: pointer to mavlink_mission_item_t or mavlink_mission_item_int_t
	 *				depending on _int_mode.
	 */
	int format_mavlink_mission_item(const struct mission_item_s *mission_item,
					mavlink_mission_item_t *mavlink_mission_item);

	/**
	 * set _state to idle (and do necessary cleanup)
	 */
	void switch_to_idle_state();

	/**
	 * Copies the specified range [1, 7] of param of MAVLink mission to params[] array of
	 * the Mission item struct (Very useful for mission items for non-navigation
	 * like MAV_CMD_DO*, as they use a parameter mapping 1 ~ 7, which directly
	 * gets the value from MAVlink's Mission Item parameters.
	 *
	 * @param start_idx [1, 7] Start index of Param to copy from Mavlink Mission Item
	 * @param end_idx [1, 7] End index of Param to copy from Mavlink Mission Item
	 *
	 * Note: The Index is in range [1, 7], so if you want to copy param2 ~ param5 into
	 * params[1] and params[4], you need to call with 'start_idx = 2' and 'end_idx = 5'!
	 */
	void copy_params_from_mavlink_to_mission_item(struct mission_item_s *mission_item,
			const mavlink_mission_item_t *mavlink_mission_item, int8_t start_idx = 1, int8_t end_idx = 7);

	/**
	 * Update crc calculation including new mission item
	 * @param[in] mission_item new mission item
	 * @param[in] prev_crc32 crc32 checksum of all previous mission item
	 * @return updated crc32 checksum of mission items
	 */
	static uint32_t crc32_for_mission_item(const mavlink_mission_item_t &mission_item, uint32_t prev_crc32);
};
