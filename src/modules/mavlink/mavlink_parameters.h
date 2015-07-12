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
 * @file mavlink_parameters.h
 * Mavlink parameters manager definition.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include <systemlib/param/param.h>

#include "mavlink_bridge_header.h"
#include "mavlink_stream.h"
#include <uORB/uORB.h>
#include <uORB/topics/rc_parameter_map.h>

class MavlinkParametersManager : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkParametersManager::get_name_static();
	}

	static const char *get_name_static()
	{
		return "PARAM_VALUE";
	}

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_PARAM_VALUE;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkParametersManager(mavlink);
	}

	unsigned get_size();

	void handle_message(const mavlink_message_t *msg);

	/**
	 * Send one parameter identified by index.
	 *
	 * @param index		The index of the parameter to send.
	 * @return		zero on success, nonzero else.
	 */
	void		start_send_one(int index);


	/**
	 * Send one parameter identified by name.
	 *
	 * @param name		The index of the parameter to send.
	 * @return		zero on success, nonzero else.
	 */
	int			start_send_for_name(const char *name);

	/**
	 * Start sending the parameter queue.
	 *
	 * This function will not directly send parameters, but instead
	 * activate the sending of one parameter on each call of
	 * mavlink_pm_queued_send().
	 * @see 		mavlink_pm_queued_send()
	 */
	void		start_send_all();

private:
	int		_send_all_index;

	/* do not allow top copying this class */
	MavlinkParametersManager(MavlinkParametersManager &);
	MavlinkParametersManager& operator = (const MavlinkParametersManager &);

protected:
	explicit MavlinkParametersManager(Mavlink *mavlink);

	void send(const hrt_abstime t);

	int send_param(param_t param);

	orb_advert_t _rc_param_map_pub;
	struct rc_parameter_map_s _rc_param_map;
};
