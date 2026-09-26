#ifndef WHEEL_ENCODER_HPP
#define WHEEL_ENCODER_HPP

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/wheel_encoders.h>

class MavlinkStreamWheelEncoder : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamWheelEncoder(mavlink); }

	static constexpr const char *get_name_static() { return "WHEEL_ENCODER"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_WHEEL_DISTANCE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return (MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}
private:
	explicit MavlinkStreamWheelEncoder(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::SubscriptionMultiArray<wheel_encoders_s> _wheel_encoders_subs{ORB_ID::wheel_encoders};

	bool send() override
	{
		bool updated = false;

		mavlink_wheel_distance_t msg{};
		float wheel_radius = 0.f; // Wheel radius in meters
		param_get(param_find("RO_WHEEL_RAD"), &wheel_radius);
		int count = 0;

		for (int i = 0; i < _wheel_encoders_subs.size(); i++) {
			wheel_encoders_s wheel_encoders;

			if (_wheel_encoders_subs[i].update(&wheel_encoders)) {
				msg.time_usec = wheel_encoders.timestamp;
				msg.distance[i] = 2.f * M_PI_F * wheel_radius * wheel_encoders.encoder_position[0] / wheel_encoders.counts_per_rev[0];
				count += 1;
			}
		}

		if (count > 0) {
			msg.count = count;
			mavlink_msg_wheel_distance_send_struct(_mavlink->get_channel(), &msg);

			updated = true;
		}

		return updated;

	}
};

#endif // WHEEL_ENCODER_HPP
