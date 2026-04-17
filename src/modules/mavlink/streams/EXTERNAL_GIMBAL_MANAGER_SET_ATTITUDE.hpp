#ifndef EXTERNAL_GIMBAL_MANAGER_SET_ATTITUDE_HPP
#define EXTERNAL_GIMBAL_MANAGER_SET_ATTITUDE_HPP

#include <uORB/topics/external_gimbal_manager_set_attitude.h>

class MavlinkStreamExternalGimbalManagerSetAttitude : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamExternalGimbalManagerSetAttitude(mavlink); }

	static constexpr const char *get_name_static() { return "EXTERNAL_GIMBAL_MANAGER_SET_ATTITUDE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override { return _external_gimbal_manager_set_attitude_sub.advertised() ? MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0; }

private:
	explicit MavlinkStreamExternalGimbalManagerSetAttitude(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _external_gimbal_manager_set_attitude_sub{ORB_ID(external_gimbal_manager_set_attitude)};

	bool send() override
	{
		external_gimbal_manager_set_attitude_s sp{};

		if (_external_gimbal_manager_set_attitude_sub.update(&sp)) {

			//Avoid sending commands addressed to ourselves (common when this topic is used as input)
			if ((sp.target_system == mavlink_system.sysid) &&
			    (sp.target_component == mavlink_system.compid)) {
				return false;
			}

			mavlink_gimbal_manager_set_attitude_t msg{};
			msg.target_system       = sp.target_system;
			msg.target_component    = sp.target_component;

			msg.flags               = sp.flags;
			msg.gimbal_device_id    = sp.gimbal_device_id;

			msg.q[0]                = sp.q[0];
			msg.q[1]                = sp.q[1];
			msg.q[2]                = sp.q[2];
			msg.q[3]                = sp.q[3];

			msg.angular_velocity_x  = sp.angular_velocity_x;
			msg.angular_velocity_y  = sp.angular_velocity_y;
			msg.angular_velocity_z  = sp.angular_velocity_z;

			mavlink_msg_gimbal_manager_set_attitude_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif //EXTERNAL_GIMBAL_MANAGER_SET_ATTITUDE_HPP
