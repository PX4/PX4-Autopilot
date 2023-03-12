#ifndef VEHICLE_LOCAL_POSITION_HPP
#define VEHICLE_LOCAL_POSITION_HPP

#include <uORB/topics/vehicle_local_position.h>

class MavlinkStreamVehicleLocalPosition : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamVehicleLocalPosition(mavlink); }

	static constexpr const char *get_name_static() { return "VEHICLE_LOCAL_POSITION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_VEHICLE_LOCAL_POSITION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_vehicle_local_position_sub.advertised()) {
			return MAVLINK_MSG_ID_VEHICLE_LOCAL_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamVehicleLocalPosition(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	bool send() override
	{
		if (_vehicle_local_position_sub.updated()) {

			vehicle_local_position_s vehicle_local_position{};
			_vehicle_local_position_sub.copy(&vehicle_local_position);

			mavlink_vehicle_local_position_t msg{};

			msg.xy_valid = vehicle_local_position.xy_valid;			// true if x and y are valid
			msg.z_valid = vehicle_local_position.z_valid;			// true if z is valid
			msg.v_xy_valid = vehicle_local_position.v_xy_valid;			// true if vx and vy are valid
			msg.v_z_valid = vehicle_local_position.v_z_valid;			// true if vz is valid
			msg.x = vehicle_local_position.x;			// North position in NED earth-fixed frame
			msg.y = vehicle_local_position.y;			// East position in NED earth-fixed frame
			msg.z = vehicle_local_position.z;			// Down position (negative altitude) in NED earth-fixed frame
			msg.delta_xy[0] = vehicle_local_position.delta_xy[0];			// xy position reset delta
			msg.delta_xy[1] = vehicle_local_position.delta_xy[1];			// xy position reset delta
			msg.xy_reset_counter = vehicle_local_position.xy_reset_counter;			// xy position reset counter
			msg.delta_z = vehicle_local_position.delta_z;			// z position reset delta
			msg.z_reset_counter = vehicle_local_position.z_reset_counter;			// z position reset counter
			msg.vx = vehicle_local_position.vx;			// North velocity in NED earth-fixed frame
			msg.vy = vehicle_local_position.vy;			// East velocity in NED earth-fixed frame
			msg.vz = vehicle_local_position.vz;			// Down velocity in NED earth-fixed frame
			msg.z_deriv = vehicle_local_position.z_deriv;			// Down position time derivative in NED earth-fixed frame
			msg.delta_vxy[0] = vehicle_local_position.delta_vxy[0];			// xy velocity reset delta
			msg.delta_vxy[1] = vehicle_local_position.delta_vxy[1];			// xy velocity reset delta
			msg.vxy_reset_counter = vehicle_local_position.vxy_reset_counter;			// xy velocity reset counter
			msg.delta_vz = vehicle_local_position.delta_vz;			// z velocity reset delta
			msg.vz_reset_counter = vehicle_local_position.vz_reset_counter;			// z velocity reset counter
			msg.ax = vehicle_local_position.ax;			// North velocity derivative in NED earth-fixed frame.
			msg.ay = vehicle_local_position.ay;			// East velocity derivative in NED earth-fixed frame.
			msg.az = vehicle_local_position.az;			// Down velocity derivative in NED earth-fixed frame.
			msg.xy_global = vehicle_local_position.xy_global;			// true if position (x, y) has a valid global reference (ref_lat, ref_lon)
			msg.z_global = vehicle_local_position.z_global;			// true if z has a valid global reference (ref_alt)
			msg.ref_lat = vehicle_local_position.ref_lat;			// Reference point latitude
			msg.ref_lon = vehicle_local_position.ref_lon;			// Reference point longitude
			msg.ref_alt = vehicle_local_position.ref_alt;			// Reference altitude AMSL
			msg.dist_bottom = vehicle_local_position.dist_bottom;			// Distance from from bottom surface to ground
			msg.dist_bottom_valid = vehicle_local_position.dist_bottom_valid;			// true if distance to bottom surface is valid
			msg.dist_bottom_sensor_bitfield = vehicle_local_position.dist_bottom_sensor_bitfield;			// bitfield indicating what type of sensor is used to estimate dist_bottom
			msg.eph = vehicle_local_position.eph;			// Standard deviation of horizontal position error
			msg.epv = vehicle_local_position.epv;			// Standard deviation of vertical position error
			msg.evh = vehicle_local_position.evh;			// Standard deviation of horizontal velocity error
			msg.evv = vehicle_local_position.evv;			// Standard deviation of vertical velocity error
			msg.dead_reckoning = vehicle_local_position.dead_reckoning;			// True if this position is estimated through dead-reckoning
			msg.vxy_max = vehicle_local_position.vxy_max;			// maximum horizontal speed
			msg.vz_max = vehicle_local_position.vz_max;			// maximum vertical speed
			msg.hagl_min = vehicle_local_position.hagl_min;			// minimum height above ground level
			msg.hagl_max = vehicle_local_position.hagl_max;			// maximum height above ground level

			mavlink_msg_vehicle_local_position_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}
		return false;
	}
};

#endif // VEHICLE_LOCAL_POSITION_HPP
