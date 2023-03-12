#ifndef VEHICLE_CONTROL_MODE_HPP
#define VEHICLE_CONTROL_MODE_HPP

#include <uORB/topics/vehicle_control_mode.h>

class MavlinkStreamVehicleControlMode : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamVehicleControlMode(mavlink); }

	static constexpr const char *get_name_static() { return "VEHICLE_CONTROL_MODE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_VEHICLE_CONTROL_MODE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_vehicle_control_mode_sub.advertised()) {
			return MAVLINK_MSG_ID_VEHICLE_CONTROL_MODE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamVehicleControlMode(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	bool send() override
	{
		if (_vehicle_control_mode_sub.updated()) {

			vehicle_control_mode_s vehicle_control_mode{};
			_vehicle_control_mode_sub.copy(&vehicle_control_mode);

			mavlink_vehicle_control_mode_t msg{};

			msg.flag_armed = vehicle_control_mode.flag_armed;			// synonym for actuator_armed.armed
			msg.flag_multicopter_position_control_enabled = vehicle_control_mode.flag_multicopter_position_control_enabled;
			msg.flag_control_manual_enabled = vehicle_control_mode.flag_control_manual_enabled;			// true if manual input is mixed in
			msg.flag_control_auto_enabled = vehicle_control_mode.flag_control_auto_enabled;			// true if onboard autopilot should act
			msg.flag_control_offboard_enabled = vehicle_control_mode.flag_control_offboard_enabled;			// true if offboard control should be used
			msg.flag_control_rates_enabled = vehicle_control_mode.flag_control_rates_enabled;			// true if rates are stabilized
			msg.flag_control_attitude_enabled = vehicle_control_mode.flag_control_attitude_enabled;			// true if attitude stabilization is mixed in
			msg.flag_control_acceleration_enabled = vehicle_control_mode.flag_control_acceleration_enabled;			// true if acceleration is controlled
			msg.flag_control_velocity_enabled = vehicle_control_mode.flag_control_velocity_enabled;			// true if horizontal velocity (implies direction) is controlled
			msg.flag_control_position_enabled = vehicle_control_mode.flag_control_position_enabled;			// true if position is controlled
			msg.flag_control_altitude_enabled = vehicle_control_mode.flag_control_altitude_enabled;			// true if altitude is controlled
			msg.flag_control_climb_rate_enabled = vehicle_control_mode.flag_control_climb_rate_enabled;			// true if climb rate is controlled
			msg.flag_control_termination_enabled = vehicle_control_mode.flag_control_termination_enabled;			// true if flighttermination is enabled

			mavlink_msg_vehicle_control_mode_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}
		return false;
	}
};

#endif // VEHICLE_CONTROL_MODE_HPP
