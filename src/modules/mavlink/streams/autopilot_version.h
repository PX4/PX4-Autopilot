#ifndef MAVLINK_STREAM_AUTOPILOT_VERSION_H
#define MAVLINK_STREAM_AUTOPILOT_VERSION_H

#include "../mavlink_messages.h"

class MavlinkStreamAutopilotVersion : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamAutopilotVersion::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "AUTOPILOT_VERSION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_AUTOPILOT_VERSION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAutopilotVersion(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool request_message(float param2, float param3, float param4,
			     float param5, float param6, float param7) override
	{
		return send(hrt_absolute_time());
	}
private:
	/* do not allow top copying this class */
	MavlinkStreamAutopilotVersion(MavlinkStreamAutopilotVersion &) = delete;
	MavlinkStreamAutopilotVersion &operator = (const MavlinkStreamAutopilotVersion &) = delete;


protected:
	explicit MavlinkStreamAutopilotVersion(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		return _mavlink->send_autopilot_capabilities();
	}
};

#endif
