#ifndef MAVLINK_STREAM_FLIGHT_INFORMATION_H
#define MAVLINK_STREAM_FLIGHT_INFORMATION_H

#include "../mavlink_messages.h"

#include <uORB/topics/actuator_armed.h>

class MavlinkStreamFlightInformation : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamFlightInformation::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "FLIGHT_INFORMATION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_FLIGHT_INFORMATION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamFlightInformation(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}
private:
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};

	/* do not allow top copying this class */
	MavlinkStreamFlightInformation(MavlinkStreamFlightInformation &) = delete;
	MavlinkStreamFlightInformation &operator = (const MavlinkStreamFlightInformation &) = delete;
protected:
	explicit MavlinkStreamFlightInformation(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		actuator_armed_s actuator_armed{};
		bool ret = _armed_sub.copy(&actuator_armed);

		if (ret && actuator_armed.timestamp != 0) {
			const param_t param_com_flight_uuid = param_find("COM_FLIGHT_UUID");
			int32_t flight_uuid;
			param_get(param_com_flight_uuid, &flight_uuid);

			mavlink_flight_information_t flight_info{};
			flight_info.flight_uuid = static_cast<uint64_t>(flight_uuid);
			flight_info.arming_time_utc = flight_info.takeoff_time_utc = actuator_armed.armed_time_ms;
			flight_info.time_boot_ms = hrt_absolute_time() / 1000;
			mavlink_msg_flight_information_send_struct(_mavlink->get_channel(), &flight_info);
		}

		return ret;
	}
};

#endif
