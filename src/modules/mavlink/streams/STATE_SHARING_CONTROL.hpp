#pragma once

#define STATE_SHARING_CONTROL_HPP

#include <uORB/topics/state_sharing_control.h>

#if defined(MAVLINK_MSG_ID_STATE_SHARING_CONTROL)
class MavlinkStreamStateSharingControl : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamStateSharingControl(mavlink);
	}
	const char *get_name() const
	{
		return MavlinkStreamStateSharingControl::get_name_static();
	}
	static const char *get_name_static()
	{
		return "STATE_SHARING_CONTROL";
	}
	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_STATE_SHARING_CONTROL;
	}
	uint16_t get_id()
	{
		return get_id_static();
	}
	unsigned get_size()
	{
		return MAVLINK_MSG_ID_STATE_SHARING_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	uORB::Subscription _out_state_sharing_control_sub{ORB_ID(outgoing_state_sharing_control)};

	/* do not allow to copy this class */
	MavlinkStreamStateSharingControl(MavlinkStreamStateSharingControl &) = delete;
	MavlinkStreamStateSharingControl &operator = (const MavlinkStreamStateSharingControl &) = delete;

protected:
	explicit MavlinkStreamStateSharingControl(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send() override
	{
		bool updated = false;

		if (_out_state_sharing_control_sub.updated()) {
			state_sharing_control_s outgoing_state_control;

			if (_out_state_sharing_control_sub.copy(&outgoing_state_control)) {
				mavlink_state_sharing_control_t mav_state_control_msg;
				mav_state_control_msg.timestamp = outgoing_state_control.timestamp;
				mav_state_control_msg.command = outgoing_state_control.command;
				memcpy(mav_state_control_msg.args, outgoing_state_control.args, sizeof(mav_state_control_msg.args));

				mavlink_msg_state_sharing_control_send_struct(_mavlink->get_channel(), &mav_state_control_msg);
				updated = true;
			}
		}

		return updated;
	}

};
#endif //MAVLINK_MSG_ID_STATE_SHARING_CONTROL
