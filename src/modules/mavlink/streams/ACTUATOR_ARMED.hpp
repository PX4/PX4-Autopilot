#ifndef ACTUATOR_ARMED_HPP
#define ACTUATOR_ARMED_HPP

#include <uORB/topics/actuator_armed.h>

class MavlinkStreamActuatorArmed : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamActuatorArmed(mavlink); }

	static constexpr const char *get_name_static() { return "ACTUATOR_ARMED"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ACTUATOR_ARMED; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_actuator_armed_sub.advertised()) {
			return MAVLINK_MSG_ID_ACTUATOR_ARMED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamActuatorArmed(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

	bool send() override
	{
		if (_actuator_armed_sub.updated()) {

			actuator_armed_s actuator_armed{};
			_actuator_armed_sub.copy(&actuator_armed);

			mavlink_actuator_armed_t msg{};

			msg.armed = actuator_armed.armed;			// Set to true if system is armed
			msg.prearmed = actuator_armed.prearmed;			// Set to true if the actuator safety is disabled but motors are not armed
			msg.ready_to_arm = actuator_armed.ready_to_arm;			// Set to true if system is ready to be armed
			msg.lockdown = actuator_armed.lockdown;			// Set to true if actuators are forced to being disabled (due to emergency or HIL)
			msg.manual_lockdown = actuator_armed.manual_lockdown;			// Set to true if manual throttle kill switch is engaged
			msg.force_failsafe = actuator_armed.force_failsafe;			// Set to true if the actuators are forced to the failsafe position
			msg.in_esc_calibration_mode = actuator_armed.in_esc_calibration_mode;			// IO/FMU should ignore messages from the actuator controls topics

			mavlink_msg_actuator_armed_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}
		return false;
	}
};

#endif // ACTUATOR_ARMED_HPP
