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
	const char *get_name() const override
	{
		return MavlinkStreamStateSharingControl::get_name_static();
	}
	static constexpr const char *get_name_static()
	{
		return "STATE_SHARING_CONTROL";
	}
	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_STATE_SHARING_CONTROL;
	}
	uint16_t get_id() override
	{
		return get_id_static();
	}
	unsigned get_size() override
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
				mavlink_state_sharing_control_t mav_state_control_msg{};
				mav_state_control_msg.timestamp = outgoing_state_control.timestamp;
				mav_state_control_msg.sync_epoch_us = outgoing_state_control.sync_epoch_us;
				mav_state_control_msg.sharing_period = outgoing_state_control.sharing_period;
				mav_state_control_msg.delay_start = outgoing_state_control.delay_start;
				mav_state_control_msg.data_flags = outgoing_state_control.data_flags;
				mav_state_control_msg.target_system = outgoing_state_control.target_system;
				mav_state_control_msg.command = outgoing_state_control.command;

				mavlink_msg_state_sharing_control_send_struct(_mavlink->get_channel(), &mav_state_control_msg);
				updated = true;
			}
		}

		return updated;
	}

};
#endif //MAVLINK_MSG_ID_STATE_SHARING_CONTROL
