#pragma once

#define STATE_SHARING_HPP

#include <uORB/topics/state_sharing_msg.h>

#if defined(MAVLINK_MSG_ID_STATE_SHARING)
class MavlinkStreamStateSharing : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamStateSharing(mavlink);
	}
	const char *get_name() const
	{
		return MavlinkStreamStateSharing::get_name_static();
	}
	static const char *get_name_static()
	{
		return "STATE_SHARING";
	}
	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_STATE_SHARING;
	}
	uint16_t get_id()
	{
		return get_id_static();
	}
	unsigned get_size()
	{
		return MAVLINK_MSG_ID_STATE_SHARING_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	uORB::Subscription		_out_state_sharing_msg_sub{ORB_ID(outgoing_state_sharing)};

	/* do not allow to copy this class */
	MavlinkStreamStateSharing(MavlinkStreamStateSharing &) = delete;
	MavlinkStreamStateSharing &operator = (const MavlinkStreamStateSharing &) = delete;

protected:
	explicit MavlinkStreamStateSharing(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		bool updated = false;

		// publish only if the status has changed
		if (_out_state_sharing_msg_sub.updated()) {
			state_sharing_msg_s outgoing_state;

			if (_out_state_sharing_msg_sub.copy(&outgoing_state)) {
				mavlink_state_sharing_t mav_state_sharing_msg;
				mav_state_sharing_msg.timestamp = outgoing_state.timestamp;
				mav_state_sharing_msg.timestamp_real_time = outgoing_state.timestamp_real_time;
				mav_state_sharing_msg.agent_id = outgoing_state.agent_id;
				mav_state_sharing_msg.global_position_lon = outgoing_state.global_position_lon;
				mav_state_sharing_msg.global_position_lat = outgoing_state.global_position_lat;
				mav_state_sharing_msg.global_position_alt = outgoing_state.global_position_alt;
				memcpy(mav_state_sharing_msg.q, outgoing_state.q, sizeof(outgoing_state.q));

				//Send the message
				mavlink_msg_state_sharing_send_struct(_mavlink->get_channel(), &mav_state_sharing_msg);
				updated = true;
			}
		}


		return updated;
	}

};
#endif //MAVLINK_MSG_ID_STATE_SHARING
