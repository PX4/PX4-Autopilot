/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * MAVLink mission manager interface definition.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#pragma once

#include <uORB/uORB.h>

#include "mavlink_bridge_header.h"
#include "mavlink_rate_limiter.h"
#include "mavlink_stream.h"

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

#define MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT 5000000    ///< Protocol communication action timeout in useconds
#define MAVLINK_MISSION_RETRY_TIMEOUT_DEFAULT 500000        ///< Protocol communication retry timeout in useconds

class MavlinkMissionManager : public MavlinkStream {
public:
	~MavlinkMissionManager();

	const char *get_name() const
	{
		return MavlinkMissionManager::get_name_static();
	}

	static const char *get_name_static()
	{
		return "MISSION_ITEM";
	}

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_MISSION_ITEM;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkMissionManager(mavlink);
	}

	unsigned get_size();

	void handle_message(const mavlink_message_t *msg);

	void set_verbose(bool v) { _verbose = v; }

private:
	enum MAVLINK_WPM_STATES _state;					///< Current state

	uint64_t		_time_last_recv;
	uint64_t		_time_last_sent;

	uint32_t		_action_timeout;
	uint32_t		_retry_timeout;
	unsigned		_max_count;				///< Maximum number of mission items

	int			_dataman_id;				///< Dataman storage ID for active mission
	unsigned		_count;					///< Count of items in active mission
	int			_current_seq;				///< Current item sequence in active mission

	int			_transfer_dataman_id;			///< Dataman storage ID for current transmission
	unsigned		_transfer_count;			///< Items count in current transmission
	unsigned		_transfer_seq;				///< Item sequence in current transmission
	unsigned		_transfer_current_seq;			///< Current item ID for current transmission (-1 means not initialized)
	unsigned		_transfer_partner_sysid;		///< Partner system ID for current transmission
	unsigned		_transfer_partner_compid;		///< Partner component ID for current transmission

	int			_offboard_mission_sub;
	int			_mission_result_sub;
	orb_advert_t		_offboard_mission_pub;

	MavlinkRateLimiter	_slow_rate_limiter;

	bool _verbose;

	uint8_t			_comp_id;

	/* do not allow top copying this class */
	MavlinkMissionManager(MavlinkMissionManager &);
	MavlinkMissionManager& operator = (const MavlinkMissionManager &);

	void init_offboard_mission();

	int update_active_mission(int dataman_id, unsigned count, int seq);

	/**
	 *  @brief Sends an waypoint ack message
	 */
	void send_mission_ack(uint8_t sysid, uint8_t compid, uint8_t type);

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

	void send_mission_count(uint8_t sysid, uint8_t compid, uint16_t count);

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

	void handle_mission_count(const mavlink_message_t *msg);

	void handle_mission_item(const mavlink_message_t *msg);

	void handle_mission_clear_all(const mavlink_message_t *msg);

	/**
	 * Parse mavlink MISSION_ITEM message to get mission_item_s.
	 */
	int parse_mavlink_mission_item(const mavlink_mission_item_t *mavlink_mission_item, struct mission_item_s *mission_item);

	/**
	 * Format mission_item_s as mavlink MISSION_ITEM message.
	 */
	int format_mavlink_mission_item(const struct mission_item_s *mission_item, mavlink_mission_item_t *mavlink_mission_item);

protected:
	explicit MavlinkMissionManager(Mavlink *mavlink);

	void send(const hrt_abstime t);
};
