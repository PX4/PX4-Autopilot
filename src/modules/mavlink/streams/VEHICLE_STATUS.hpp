#ifndef VEHICLE_STATUS_HPP
#define VEHICLE_STATUS_HPP

#include <uORB/topics/vehicle_status.h>

class MavlinkStreamVehicleStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamVehicleStatus(mavlink); }

	static constexpr const char *get_name_static() { return "VEHICLE_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_VEHICLE_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_vehicle_status_sub.advertised()) {
			return MAVLINK_MSG_ID_VEHICLE_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamVehicleStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	bool send() override
	{
		if (_vehicle_status_sub.updated()) {

			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			mavlink_vehicle_status_t msg{};

			msg.armed_time = vehicle_status.armed_time;			// Arming timestamp
			msg.takeoff_time = vehicle_status.takeoff_time;			// Takeoff timestamp
			msg.arming_state = vehicle_status.arming_state;
			msg.latest_arming_reason = vehicle_status.latest_arming_reason;
			msg.latest_disarming_reason = vehicle_status.latest_disarming_reason;
			msg.nav_state_timestamp = vehicle_status.nav_state_timestamp;			// time when current nav_state activated
			msg.nav_state_user_intention = vehicle_status.nav_state_user_intention;			// Mode that the user selected (might be different from nav_state in a failsafe situation)
			msg.nav_state = vehicle_status.nav_state;			// Currently active mode
			msg.failure_detector_status = vehicle_status.failure_detector_status;			// Bitmask of detected failures
			msg.hil_state = vehicle_status.hil_state;			// Bitmask of detected failures
			msg.vehicle_type = vehicle_status.vehicle_type;
			msg.failsafe = vehicle_status.failsafe;			// true if system is in failsafe state (e.g.:RTL, Hover, Terminate, ...)
			msg.failsafe_and_user_took_over = vehicle_status.failsafe_and_user_took_over;			// true if system is in failsafe state but the user took over control
			msg.gcs_connection_lost = vehicle_status.gcs_connection_lost;
			msg.gcs_connection_lost_counter = vehicle_status.gcs_connection_lost_counter;
			msg.high_latency_data_link_lost = vehicle_status.high_latency_data_link_lost;
			msg.is_vtol = vehicle_status.is_vtol;
			msg.is_vtol_tailsitter = vehicle_status.is_vtol_tailsitter;
			msg.in_transition_mode = vehicle_status.in_transition_mode;
			msg.in_transition_to_fw = vehicle_status.in_transition_to_fw;

			mavlink_msg_vehicle_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}
		return false;
	}
};

#endif // VEHICLE_STATUS_HPP
