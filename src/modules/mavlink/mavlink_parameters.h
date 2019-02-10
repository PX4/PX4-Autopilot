/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_parameters.h
 * Mavlink parameters manager definition.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Beat Kueng <beat@px4.io>
 */

#pragma once

#include <parameters/param.h>

#include "mavlink_bridge_header.h"
#include <uORB/uORB.h>
#include <uORB/topics/rc_parameter_map.h>
#include <uORB/topics/uavcan_parameter_request.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>

class Mavlink;

class MavlinkParametersManager
{
public:
	explicit MavlinkParametersManager(Mavlink *mavlink);
	~MavlinkParametersManager();

	/**
	 * Handle sending of messages. Call this regularly at a fixed frequency.
	 * @param t current time
	 */
	void send(const hrt_abstime t);

	unsigned get_size();

	void handle_message(const mavlink_message_t *msg);

private:
	int		_send_all_index;

	/* do not allow top copying this class */
	MavlinkParametersManager(MavlinkParametersManager &);
	MavlinkParametersManager &operator = (const MavlinkParametersManager &);

protected:
	/// send a single param if a PARAM_REQUEST_LIST is in progress
	/// @return true if a parameter was sent
	bool send_one();

	/**
	 * Handle any open param send transfer
	 */
	bool send_params();

	/**
	 * Send UAVCAN params
	 */
	bool send_uavcan();

	/**
	 * Send untransmitted params
	 */
	bool send_untransmitted();

	int send_param(param_t param, int component_id = -1);

	// Item of a single-linked list to store requested uavcan parameters
	struct _uavcan_open_request_list_item {
		uavcan_parameter_request_s req;
		struct _uavcan_open_request_list_item *next;
	};

	/**
	 * Request the next uavcan parameter
	 */
	void request_next_uavcan_parameter();

	/**
	 * Enqueue one uavcan parameter reqest. We store 10 at max.
	 */
	void enque_uavcan_request(uavcan_parameter_request_s *req);

	/**
	 * Drop the first reqest from the list
	 */
	void dequeue_uavcan_request();

	_uavcan_open_request_list_item *_uavcan_open_request_list; ///< Pointer to the first item in the linked list
	bool _uavcan_waiting_for_request_response; ///< We have reqested a parameter and wait for the response
	uint16_t _uavcan_queued_request_items;	///< Number of stored parameter requests currently in the list

	orb_advert_t _rc_param_map_pub;
	struct rc_parameter_map_s _rc_param_map;

	orb_advert_t _uavcan_parameter_request_pub;
	int _uavcan_parameter_value_sub;
	int _mavlink_parameter_sub;
	hrt_abstime _param_update_time;
	int _param_update_index;

	Mavlink *_mavlink;
};
