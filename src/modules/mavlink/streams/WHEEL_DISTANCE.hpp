#ifndef WHEEL_DISTANCE_HPP
#define WHEEL_DISTANCE_HPP

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/wheel_encoders.h>

class MavlinkStreamWheelDistance : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamWheelDistance(mavlink); }

	static constexpr const char *get_name_static() { return "WHEEL_DISTANCE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_WHEEL_DISTANCE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _wheel_encoders_subs.advertised() ? (MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}
private:
	explicit MavlinkStreamWheelDistance(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
		_param_ro_wheel_rad = param_find("RO_WHEEL_RAD");
	}

	static constexpr int NUM_WHEELS = sizeof(wheel_encoders_s::counts_per_rev) / sizeof(wheel_encoders_s::counts_per_rev[0]);

	uORB::SubscriptionMultiArray<wheel_encoders_s> _wheel_encoders_subs{ORB_ID::wheel_encoders};
	param_t _param_ro_wheel_rad{PARAM_INVALID};

	// Only one publisher is supported: every instance fills the same slots, so the last updated one wins
	bool send() override
	{
		bool updated = false;

		mavlink_wheel_distance_t msg{};
		float wheel_radius = 0.f; // Wheel radius in meters
		param_get(_param_ro_wheel_rad, &wheel_radius);
		int count = 0;

		for (int i = 0; i < _wheel_encoders_subs.size(); i++) {
			wheel_encoders_s wheel_encoders;

			if (_wheel_encoders_subs[i].update(&wheel_encoders)) {

				msg.time_usec = wheel_encoders.timestamp;

				for (int j = 0; j < NUM_WHEELS; j++) {
					if (wheel_encoders.counts_per_rev[j] > 0) {
						msg.distance[j] = (double)2.0 * (double)M_PI_F * (double)wheel_radius * wheel_encoders.encoder_counts[j] /
								  (double)wheel_encoders.counts_per_rev[j];
						count = j + 1;
					}
				}
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

#endif // WHEEL_DISTANCE_HPP
