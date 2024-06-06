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

#ifndef TIME_ESTIMATE_TO_TARGET_HPP
#define TIME_ESTIMATE_TO_TARGET_HPP

#include <uORB/topics/rtl_time_estimate.h>

class MavlinkStreamTimeEstimateToTarget : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamTimeEstimateToTarget(mavlink); }

	static constexpr const char *get_name_static() { return "TIME_ESTIMATE_TO_TARGET"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamTimeEstimateToTarget(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _rtl_estimate_sub{ORB_ID(rtl_time_estimate)};

	bool send() override
	{
		if (_rtl_estimate_sub.updated()) {
			rtl_time_estimate_s rtl_estimate{};
			_rtl_estimate_sub.copy(&rtl_estimate);

			mavlink_time_estimate_to_target_t msg{};
			msg.safe_return = static_cast<int32_t>(rtl_estimate.safe_time_estimate);

			// Set to -1 explicitly because not supported (yet)
			msg.land = -1;
			msg.mission_next_item = -1;
			msg.mission_end = -1;
			msg.commanded_action = -1;

			mavlink_msg_time_estimate_to_target_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // TIME_ESTIMATE_TO_TARGET_HPP
