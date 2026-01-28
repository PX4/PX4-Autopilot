/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#ifndef FIGURE_EIGHT_EXECUTION_STATUS_HPP
#define FIGURE_EIGHT_EXECUTION_STATUS_HPP

#include <uORB/topics/figure_eight_status.h>
#include <mavlink.h>
#include <mavlink/mavlink_stream.h>

class MavlinkStreamFigureEightStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamFigureEightStatus(mavlink); }

	static constexpr const char *get_name_static() { return "FIGURE_EIGHT_EXECUTION_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _figure_eight_status_subs.advertised() ? MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN +
		       MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamFigureEightStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _figure_eight_status_subs{ORB_ID::figure_eight_status};

	bool send() override
	{
		figure_eight_status_s figure_eight_status;

		if ((_mavlink->get_free_tx_buf() >= get_size()) && _figure_eight_status_subs.update(&figure_eight_status)) {
			mavlink_figure_eight_execution_status_t msg_figure_eight_execution_status{};

			msg_figure_eight_execution_status.time_usec = figure_eight_status.timestamp;
			msg_figure_eight_execution_status.major_radius = figure_eight_status.major_radius;
			msg_figure_eight_execution_status.minor_radius = figure_eight_status.minor_radius;
			msg_figure_eight_execution_status.frame = figure_eight_status.frame;
			msg_figure_eight_execution_status.orientation = figure_eight_status.orientation;
			msg_figure_eight_execution_status.x = figure_eight_status.x;
			msg_figure_eight_execution_status.y = figure_eight_status.y;
			msg_figure_eight_execution_status.z = figure_eight_status.z;

			mavlink_msg_figure_eight_execution_status_send_struct(_mavlink->get_channel(), &msg_figure_eight_execution_status);

			return true;
		}

		return false;
	}
};

#endif // FIGURE_EIGHT_EXECUTION_STATUS_HPP
