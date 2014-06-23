/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_messages.cpp
 * MAVLink 1.0 message formatters implementation.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <stdio.h>
#include <commander/px4_custom_mode.h>
#include <lib/geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/navigation_capabilities.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_range_finder.h>

#include <systemlib/err.h>

#include "mavlink_messages.h"


static uint16_t cm_uint16_from_m_float(float m);
static void get_mavlink_mode_state(struct vehicle_status_s *status, struct position_setpoint_triplet_s *pos_sp_triplet,
				   uint8_t *mavlink_state, uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode);

uint16_t
cm_uint16_from_m_float(float m)
{
	if (m < 0.0f) {
		return 0;

	} else if (m > 655.35f) {
		return 65535;
	}

	return (uint16_t)(m * 100.0f);
}

void get_mavlink_mode_state(struct vehicle_status_s *status, struct position_setpoint_triplet_s *pos_sp_triplet,
			    uint8_t *mavlink_state, uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode)
{
	*mavlink_state = 0;
	*mavlink_base_mode = 0;
	*mavlink_custom_mode = 0;

	/* HIL */
	if (status->hil_state == HIL_STATE_ON) {
		*mavlink_base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}

	/* arming state */
	if (status->arming_state == ARMING_STATE_ARMED
	    || status->arming_state == ARMING_STATE_ARMED_ERROR) {
		*mavlink_base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	/* main state */
	*mavlink_base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	union px4_custom_mode custom_mode;
	custom_mode.data = 0;

	if (pos_sp_triplet->nav_state == NAV_STATE_NONE) {
		/* use main state when navigator is not active */
		if (status->main_state == MAIN_STATE_MANUAL) {
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | (status->is_rotary_wing ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0);
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;

		} else if (status->main_state == MAIN_STATE_ALTCTL) {
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;

		} else if (status->main_state == MAIN_STATE_POSCTL) {
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;

		} else if (status->main_state == MAIN_STATE_AUTO) {
			*mavlink_base_mode |= MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_READY;

		} else if (status->main_state == MAIN_STATE_ACRO) {
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
		}

	} else {
		/* use navigation state when navigator is active */
		*mavlink_base_mode |= MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;

		if (pos_sp_triplet->nav_state == NAV_STATE_READY) {
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_READY;

		} else if (pos_sp_triplet->nav_state == NAV_STATE_LOITER) {
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;

		} else if (pos_sp_triplet->nav_state == NAV_STATE_MISSION) {
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;

		} else if (pos_sp_triplet->nav_state == NAV_STATE_RTL) {
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;

		} else if (pos_sp_triplet->nav_state == NAV_STATE_LAND) {
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		}
	}

	*mavlink_custom_mode = custom_mode.data;

	/* set system state */
	if (status->arming_state == ARMING_STATE_INIT
	    || status->arming_state == ARMING_STATE_IN_AIR_RESTORE
	    || status->arming_state == ARMING_STATE_STANDBY_ERROR) {	// TODO review
		*mavlink_state = MAV_STATE_UNINIT;

	} else if (status->arming_state == ARMING_STATE_ARMED) {
		*mavlink_state = MAV_STATE_ACTIVE;

	} else if (status->arming_state == ARMING_STATE_ARMED_ERROR) {
		*mavlink_state = MAV_STATE_CRITICAL;

	} else if (status->arming_state == ARMING_STATE_STANDBY) {
		*mavlink_state = MAV_STATE_STANDBY;

	} else if (status->arming_state == ARMING_STATE_REBOOT) {
		*mavlink_state = MAV_STATE_POWEROFF;

	} else {
		*mavlink_state = MAV_STATE_CRITICAL;
	}
}


class MavlinkStreamHeartbeat : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHeartbeat::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HEARTBEAT";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamHeartbeat();
	}

private:
	MavlinkOrbSubscription *status_sub;
	MavlinkOrbSubscription *pos_sp_triplet_sub;

protected:
	void subscribe(Mavlink *mavlink)
	{
		status_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_status));
		pos_sp_triplet_sub = mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status;
		struct position_setpoint_triplet_s pos_sp_triplet;

		if (status_sub->update(&status) && pos_sp_triplet_sub->update(&pos_sp_triplet)) {
			uint8_t mavlink_state = 0;
			uint8_t mavlink_base_mode = 0;
			uint32_t mavlink_custom_mode = 0;
			get_mavlink_mode_state(&status, &pos_sp_triplet, &mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

			mavlink_msg_heartbeat_send(_channel,
						   mavlink_system.type,
						   MAV_AUTOPILOT_PX4,
						   mavlink_base_mode,
						   mavlink_custom_mode,
						   mavlink_state);
		}
	}
};


class MavlinkStreamSysStatus : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamSysStatus::get_name_static();
	}

	static const char *get_name_static ()
	{
		return "SYS_STATUS";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamSysStatus();
	}

private:
	MavlinkOrbSubscription *status_sub;

protected:
	void subscribe(Mavlink *mavlink)
	{
		status_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_status));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status;

		if (status_sub->update(&status)) {
			mavlink_msg_sys_status_send(_channel,
							status.onboard_control_sensors_present,
							status.onboard_control_sensors_enabled,
							status.onboard_control_sensors_health,
							status.load * 1000.0f,
							status.battery_voltage * 1000.0f,
							status.battery_current * 100.0f,
							status.battery_remaining * 100.0f,
							status.drop_rate_comm,
							status.errors_comm,
							status.errors_count1,
							status.errors_count2,
							status.errors_count3,
							status.errors_count4);
		}
	}
};


class MavlinkStreamHighresIMU : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHighresIMU::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HIGHRES_IMU";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamHighresIMU();
	}

private:
	MavlinkOrbSubscription *sensor_sub;
	uint64_t sensor_time;

	uint64_t accel_timestamp;
	uint64_t gyro_timestamp;
	uint64_t mag_timestamp;
	uint64_t baro_timestamp;

protected:
	explicit MavlinkStreamHighresIMU() : MavlinkStream(),
		sensor_time(0),
		accel_timestamp(0),
		gyro_timestamp(0),
		mag_timestamp(0),
		baro_timestamp(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		sensor_sub = mavlink->add_orb_subscription(ORB_ID(sensor_combined));
	}

	void send(const hrt_abstime t)
	{
		struct sensor_combined_s sensor;

		if (sensor_sub->update(&sensor_time, &sensor)) {
			uint16_t fields_updated = 0;

			if (accel_timestamp != sensor.accelerometer_timestamp) {
				/* mark first three dimensions as changed */
				fields_updated |= (1 << 0) | (1 << 1) | (1 << 2);
				accel_timestamp = sensor.accelerometer_timestamp;
			}

			if (gyro_timestamp != sensor.timestamp) {
				/* mark second group dimensions as changed */
				fields_updated |= (1 << 3) | (1 << 4) | (1 << 5);
				gyro_timestamp = sensor.timestamp;
			}

			if (mag_timestamp != sensor.magnetometer_timestamp) {
				/* mark third group dimensions as changed */
				fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);
				mag_timestamp = sensor.magnetometer_timestamp;
			}

			if (baro_timestamp != sensor.baro_timestamp) {
				/* mark last group dimensions as changed */
				fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);
				baro_timestamp = sensor.baro_timestamp;
			}

			mavlink_msg_highres_imu_send(_channel,
						     sensor.timestamp,
						     sensor.accelerometer_m_s2[0], sensor.accelerometer_m_s2[1], sensor.accelerometer_m_s2[2],
						     sensor.gyro_rad_s[0], sensor.gyro_rad_s[1], sensor.gyro_rad_s[2],
						     sensor.magnetometer_ga[0], sensor.magnetometer_ga[1], sensor.magnetometer_ga[2],
						     sensor.baro_pres_mbar, sensor.differential_pressure_pa,
						     sensor.baro_alt_meter, sensor.baro_temp_celcius,
						     fields_updated);
		}
	}
};


class MavlinkStreamAttitude : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAttitude::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ATTITUDE";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamAttitude();
	}

private:
	MavlinkOrbSubscription *att_sub;
	uint64_t att_time;

protected:
	explicit MavlinkStreamAttitude() : MavlinkStream(),
		att_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		att_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_attitude));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_attitude_s att;

		if (att_sub->update(&att_time, &att)) {
			mavlink_msg_attitude_send(_channel,
						  att.timestamp / 1000,
						  att.roll, att.pitch, att.yaw,
						  att.rollspeed, att.pitchspeed, att.yawspeed);
		}
	}
};


class MavlinkStreamAttitudeQuaternion : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAttitudeQuaternion::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ATTITUDE_QUATERNION";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamAttitudeQuaternion();
	}

private:
	MavlinkOrbSubscription *att_sub;
	uint64_t att_time;

protected:
	explicit MavlinkStreamAttitudeQuaternion() : MavlinkStream(),
		att_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		att_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_attitude));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_attitude_s att;

		if (att_sub->update(&att_time, &att)) {
			mavlink_msg_attitude_quaternion_send(_channel,
							     att.timestamp / 1000,
							     att.q[0],
							     att.q[1],
							     att.q[2],
							     att.q[3],
							     att.rollspeed,
							     att.pitchspeed,
							     att.yawspeed);
		}
	}
};


class MavlinkStreamVFRHUD : public MavlinkStream
{
public:

	const char *get_name() const
	{
		return MavlinkStreamVFRHUD::get_name_static();
	}

	static const char *get_name_static()
	{
		return "VFR_HUD";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamVFRHUD();
	}

private:
	MavlinkOrbSubscription *att_sub;
	uint64_t att_time;

	MavlinkOrbSubscription *pos_sub;
	uint64_t pos_time;

	MavlinkOrbSubscription *armed_sub;
	uint64_t armed_time;

	MavlinkOrbSubscription *act_sub;
	uint64_t act_time;

	MavlinkOrbSubscription *airspeed_sub;
	uint64_t airspeed_time;

protected:
	explicit MavlinkStreamVFRHUD() : MavlinkStream(),
		att_time(0),
		pos_time(0),
		armed_time(0),
		act_time(0),
		airspeed_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		att_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_attitude));
		pos_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_global_position));
		armed_sub = mavlink->add_orb_subscription(ORB_ID(actuator_armed));
		act_sub = mavlink->add_orb_subscription(ORB_ID(actuator_controls_0));
		airspeed_sub = mavlink->add_orb_subscription(ORB_ID(airspeed));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_attitude_s att;
		struct vehicle_global_position_s pos;
		struct actuator_armed_s armed;
		struct actuator_controls_s act;
		struct airspeed_s airspeed;

		bool updated = att_sub->update(&att_time, &att);
		updated |= pos_sub->update(&pos_time, &pos);
		updated |= armed_sub->update(&armed_time, &armed);
		updated |= act_sub->update(&act_time, &act);
		updated |= airspeed_sub->update(&airspeed_time, &airspeed);

		if (updated) {
			float groundspeed = sqrtf(pos.vel_n * pos.vel_n + pos.vel_e * pos.vel_e);
			uint16_t heading = _wrap_2pi(att.yaw) * M_RAD_TO_DEG_F;
			float throttle = armed.armed ? act.control[3] * 100.0f : 0.0f;

			mavlink_msg_vfr_hud_send(_channel,
						 airspeed.true_airspeed_m_s,
						 groundspeed,
						 heading,
						 throttle,
						 pos.alt,
						 -pos.vel_d);
		}
	}
};


class MavlinkStreamGPSRawInt : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGPSRawInt::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GPS_RAW_INT";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamGPSRawInt();
	}

private:
	MavlinkOrbSubscription *gps_sub;
	uint64_t gps_time;

protected:
	explicit MavlinkStreamGPSRawInt() : MavlinkStream(),
		gps_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		gps_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_gps_position));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_gps_position_s gps;

		if (gps_sub->update(&gps_time, &gps)) {
			mavlink_msg_gps_raw_int_send(_channel,
						     gps.timestamp_position,
						     gps.fix_type,
						     gps.lat,
						     gps.lon,
						     gps.alt,
						     cm_uint16_from_m_float(gps.eph_m),
						     cm_uint16_from_m_float(gps.epv_m),
						     gps.vel_m_s * 100.0f,
						     _wrap_2pi(gps.cog_rad) * M_RAD_TO_DEG_F * 1e2f,
						     gps.satellites_visible);
		}
	}
};


class MavlinkStreamGlobalPositionInt : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGlobalPositionInt::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GLOBAL_POSITION_INT";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamGlobalPositionInt();
	}

private:
	MavlinkOrbSubscription *pos_sub;
	uint64_t pos_time;

	MavlinkOrbSubscription *home_sub;
	uint64_t home_time;

protected:
	explicit MavlinkStreamGlobalPositionInt() : MavlinkStream(),
		pos_time(0),
		home_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		pos_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_global_position));
		home_sub = mavlink->add_orb_subscription(ORB_ID(home_position));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_global_position_s pos;
		struct home_position_s home;

		bool updated = pos_sub->update(&pos_time, &pos);
		updated |= home_sub->update(&home_time, &home);

		if (updated) {
			mavlink_msg_global_position_int_send(_channel,
							     pos.timestamp / 1000,
							     pos.lat * 1e7,
							     pos.lon * 1e7,
							     pos.alt * 1000.0f,
							     (pos.alt - home.alt) * 1000.0f,
							     pos.vel_n * 100.0f,
							     pos.vel_e * 100.0f,
							     pos.vel_d * 100.0f,
							     _wrap_2pi(pos.yaw) * M_RAD_TO_DEG_F * 100.0f);
		}
	}
};


class MavlinkStreamLocalPositionNED : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamLocalPositionNED::get_name_static();
	}

	static const char *get_name_static()
	{
		return "LOCAL_POSITION_NED";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamLocalPositionNED();
	}

private:
	MavlinkOrbSubscription *pos_sub;
	uint64_t pos_time;

protected:
	explicit MavlinkStreamLocalPositionNED() : MavlinkStream(),
		pos_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		pos_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_local_position));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_local_position_s pos;

		if (pos_sub->update(&pos_time, &pos)) {
			mavlink_msg_local_position_ned_send(_channel,
							    pos.timestamp / 1000,
							    pos.x,
							    pos.y,
							    pos.z,
							    pos.vx,
							    pos.vy,
							    pos.vz);
		}
	}
};



class MavlinkStreamViconPositionEstimate : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamViconPositionEstimate::get_name_static();
	}

	static const char *get_name_static()
	{
		return "VICON_POSITION_ESTIMATE";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamViconPositionEstimate();
	}

private:
	MavlinkOrbSubscription *pos_sub;
	uint64_t pos_time;

protected:
	explicit MavlinkStreamViconPositionEstimate() : MavlinkStream(),
		pos_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		pos_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_vicon_position));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_vicon_position_s pos;

		if (pos_sub->update(&pos_time, &pos)) {
			mavlink_msg_vicon_position_estimate_send(_channel,
								pos.timestamp / 1000,
								pos.x,
								pos.y,
								pos.z,
								pos.roll,
								pos.pitch,
								pos.yaw);
		}
	}
};


class MavlinkStreamGPSGlobalOrigin : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGPSGlobalOrigin::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GPS_GLOBAL_ORIGIN";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamGPSGlobalOrigin();
	}

private:
	MavlinkOrbSubscription *home_sub;

protected:
	void subscribe(Mavlink *mavlink)
	{
		home_sub = mavlink->add_orb_subscription(ORB_ID(home_position));
	}

	void send(const hrt_abstime t)
	{
		/* we're sending the GPS home periodically to ensure the
		 * the GCS does pick it up at one point */
		if (home_sub->is_published()) {
			struct home_position_s home;

			if (home_sub->update(&home)) {
				mavlink_msg_gps_global_origin_send(_channel,
								   (int32_t)(home.lat * 1e7),
								   (int32_t)(home.lon * 1e7),
								   (int32_t)(home.alt) * 1000.0f);
			}
		}
	}
};

template <int N>
class MavlinkStreamServoOutputRaw : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamServoOutputRaw<N>::get_name_static();
	}

	static const char *get_name_static()
	{
		switch (N) {
			case 0:
			return "SERVO_OUTPUT_RAW_0";

			case 1:
			return "SERVO_OUTPUT_RAW_1";

			case 2:
			return "SERVO_OUTPUT_RAW_2";

			case 3:
			return "SERVO_OUTPUT_RAW_3";
		}
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamServoOutputRaw<N>();
	}

private:
	MavlinkOrbSubscription *act_sub;
	uint64_t act_time;

protected:
	explicit MavlinkStreamServoOutputRaw() : MavlinkStream(),
		act_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		orb_id_t act_topics[] = {
			ORB_ID(actuator_outputs_0),
			ORB_ID(actuator_outputs_1),
			ORB_ID(actuator_outputs_2),
			ORB_ID(actuator_outputs_3)
		};

		act_sub = mavlink->add_orb_subscription(act_topics[N]);
	}

	void send(const hrt_abstime t)
	{
		struct actuator_outputs_s act;

		if (act_sub->update(&act_time, &act)) {
			mavlink_msg_servo_output_raw_send(_channel,
							  act.timestamp / 1000,
							  N,
							  act.output[0],
							  act.output[1],
							  act.output[2],
							  act.output[3],
							  act.output[4],
							  act.output[5],
							  act.output[6],
							  act.output[7]);
		}
	}
};


class MavlinkStreamHILControls : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHILControls::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HIL_CONTROLS";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamHILControls();
	}

private:
	MavlinkOrbSubscription *status_sub;
	uint64_t status_time;

	MavlinkOrbSubscription *pos_sp_triplet_sub;
	uint64_t pos_sp_triplet_time;

	MavlinkOrbSubscription *act_sub;
	uint64_t act_time;

protected:
	explicit MavlinkStreamHILControls() : MavlinkStream(),
		status_time(0),
		pos_sp_triplet_time(0),
		act_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		status_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_status));
		pos_sp_triplet_sub = mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet));
		act_sub = mavlink->add_orb_subscription(ORB_ID(actuator_outputs_0));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status;
		struct position_setpoint_triplet_s pos_sp_triplet;
		struct actuator_outputs_s act;

		bool updated = act_sub->update(&act_time, &act);
		updated |= pos_sp_triplet_sub->update(&pos_sp_triplet_time, &pos_sp_triplet);
		updated |= status_sub->update(&status_time, &status);

		if (updated && (status.arming_state == ARMING_STATE_ARMED)) {
			/* translate the current syste state to mavlink state and mode */
			uint8_t mavlink_state;
			uint8_t mavlink_base_mode;
			uint32_t mavlink_custom_mode;
			get_mavlink_mode_state(&status, &pos_sp_triplet, &mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

			if (mavlink_system.type == MAV_TYPE_QUADROTOR ||
				mavlink_system.type == MAV_TYPE_HEXAROTOR ||
				mavlink_system.type == MAV_TYPE_OCTOROTOR) {
				/* set number of valid outputs depending on vehicle type */
				unsigned n;

				switch (mavlink_system.type) {
				case MAV_TYPE_QUADROTOR:
					n = 4;
					break;

				case MAV_TYPE_HEXAROTOR:
					n = 6;
					break;

				default:
					n = 8;
					break;
				}

				/* scale / assign outputs depending on system type */
				float out[8];

				for (unsigned i = 0; i < 8; i++) {
					if (i < n) {
						if (mavlink_base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
							/* scale fake PWM out 900..2100 us to 0..1 for normal multirotors */
							out[i] = (act.output[i] - PWM_LOWEST_MIN) / (PWM_HIGHEST_MAX - PWM_LOWEST_MIN);

						} else {
							/* send 0 when disarmed */
							out[i] = 0.0f;
						}

					} else {
						out[i] = -1.0f;
					}
				}

				mavlink_msg_hil_controls_send(_channel,
							      hrt_absolute_time(),
							      out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7],
							      mavlink_base_mode,
							      0);
			} else {

				/* fixed wing: scale all channels except throttle -1 .. 1
				 * because we know that we set the mixers up this way
				 */

				float out[8];

				const float pwm_center = (PWM_HIGHEST_MAX + PWM_LOWEST_MIN) / 2;

				for (unsigned i = 0; i < 8; i++) {
					if (i != 3) {
						/* scale fake PWM out 900..2100 us to -1..+1 for normal channels */
						out[i] = (act.output[i] - pwm_center) / ((PWM_HIGHEST_MAX - PWM_LOWEST_MIN) / 2);

					} else {

						/* scale fake PWM out 900..2100 us to 0..1 for throttle */
						out[i] = (act.output[i] - PWM_LOWEST_MIN) / (PWM_HIGHEST_MAX - PWM_LOWEST_MIN);
					}

				}

				mavlink_msg_hil_controls_send(_channel,
							      hrt_absolute_time(),
							      out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7],
							      mavlink_base_mode,
							      0);
			}
		}
	}
};


class MavlinkStreamGlobalPositionSetpointInt : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGlobalPositionSetpointInt::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GLOBAL_POSITION_SETPOINT_INT";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamGlobalPositionSetpointInt();
	}

private:
	MavlinkOrbSubscription *pos_sp_triplet_sub;

protected:
	void subscribe(Mavlink *mavlink)
	{
		pos_sp_triplet_sub = mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet));
	}

	void send(const hrt_abstime t)
	{
		struct position_setpoint_triplet_s pos_sp_triplet;

		if (pos_sp_triplet_sub->update(&pos_sp_triplet)) {
			mavlink_msg_global_position_setpoint_int_send(_channel,
					MAV_FRAME_GLOBAL,
					(int32_t)(pos_sp_triplet.current.lat * 1e7),
					(int32_t)(pos_sp_triplet.current.lon * 1e7),
					(int32_t)(pos_sp_triplet.current.alt * 1000),
					(int16_t)(pos_sp_triplet.current.yaw * M_RAD_TO_DEG_F * 100.0f));
		}
	}
};


class MavlinkStreamLocalPositionSetpoint : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamLocalPositionSetpoint::get_name_static();
	}

	static const char *get_name_static()
	{
		return "LOCAL_POSITION_SETPOINT";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamLocalPositionSetpoint();
	}

private:
	MavlinkOrbSubscription *pos_sp_sub;
	uint64_t pos_sp_time;

protected:
	explicit MavlinkStreamLocalPositionSetpoint() : MavlinkStream(),
		pos_sp_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		pos_sp_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_local_position_setpoint));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_local_position_setpoint_s pos_sp;

		if (pos_sp_sub->update(&pos_sp_time, &pos_sp)) {
			mavlink_msg_local_position_setpoint_send(_channel,
					MAV_FRAME_LOCAL_NED,
					pos_sp.x,
					pos_sp.y,
					pos_sp.z,
					pos_sp.yaw);
		}
	}
};


class MavlinkStreamRollPitchYawThrustSetpoint : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamRollPitchYawThrustSetpoint::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ROLL_PITCH_YAW_THRUST_SETPOINT";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamRollPitchYawThrustSetpoint();
	}

private:
	MavlinkOrbSubscription *att_sp_sub;
	uint64_t att_sp_time;

protected:
	explicit MavlinkStreamRollPitchYawThrustSetpoint() : MavlinkStream(),
		att_sp_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		att_sp_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_attitude_setpoint));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_attitude_setpoint_s att_sp;

		if (att_sp_sub->update(&att_sp_time, &att_sp)) {
			mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(_channel,
					att_sp.timestamp / 1000,
					att_sp.roll_body,
					att_sp.pitch_body,
					att_sp.yaw_body,
					att_sp.thrust);
		}
	}
};


class MavlinkStreamRollPitchYawRatesThrustSetpoint : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamRollPitchYawRatesThrustSetpoint::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ROLL_PITCH_YAW_RATES_THRUST_SETPOINT";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamRollPitchYawRatesThrustSetpoint();
	}

private:
	MavlinkOrbSubscription *att_rates_sp_sub;
	uint64_t att_rates_sp_time;

protected:
	explicit MavlinkStreamRollPitchYawRatesThrustSetpoint() : MavlinkStream(),
		att_rates_sp_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		att_rates_sp_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_rates_setpoint));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_rates_setpoint_s att_rates_sp;

		if (att_rates_sp_sub->update(&att_rates_sp_time, &att_rates_sp)) {
			mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_send(_channel,
					att_rates_sp.timestamp / 1000,
					att_rates_sp.roll,
					att_rates_sp.pitch,
					att_rates_sp.yaw,
					att_rates_sp.thrust);
		}
	}
};


class MavlinkStreamRCChannelsRaw : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamRCChannelsRaw::get_name_static();
	}

	static const char *get_name_static()
	{
		return "RC_CHANNELS_RAW";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamRCChannelsRaw();
	}

private:
	MavlinkOrbSubscription *rc_sub;
	uint64_t rc_time;

protected:
	explicit MavlinkStreamRCChannelsRaw() : MavlinkStream(),
		rc_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		rc_sub = mavlink->add_orb_subscription(ORB_ID(input_rc));
	}

	void send(const hrt_abstime t)
	{
		struct rc_input_values rc;

		if (rc_sub->update(&rc_time, &rc)) {
			const unsigned port_width = 8;

			// Deprecated message (but still needed for compatibility!)
			for (unsigned i = 0; (i * port_width) < rc.channel_count; i++) {
				/* Channels are sent in MAVLink main loop at a fixed interval */
				mavlink_msg_rc_channels_raw_send(_channel,
								 rc.timestamp_publication / 1000,
								 i,
								 (rc.channel_count > (i * port_width) + 0) ? rc.values[(i * port_width) + 0] : UINT16_MAX,
								 (rc.channel_count > (i * port_width) + 1) ? rc.values[(i * port_width) + 1] : UINT16_MAX,
								 (rc.channel_count > (i * port_width) + 2) ? rc.values[(i * port_width) + 2] : UINT16_MAX,
								 (rc.channel_count > (i * port_width) + 3) ? rc.values[(i * port_width) + 3] : UINT16_MAX,
								 (rc.channel_count > (i * port_width) + 4) ? rc.values[(i * port_width) + 4] : UINT16_MAX,
								 (rc.channel_count > (i * port_width) + 5) ? rc.values[(i * port_width) + 5] : UINT16_MAX,
								 (rc.channel_count > (i * port_width) + 6) ? rc.values[(i * port_width) + 6] : UINT16_MAX,
								 (rc.channel_count > (i * port_width) + 7) ? rc.values[(i * port_width) + 7] : UINT16_MAX,
								 rc.rssi);
			}

			// New message
			mavlink_msg_rc_channels_send(_channel,
					rc.timestamp_publication / 1000,
					rc.channel_count,
					((rc.channel_count > 0) ? rc.values[0] : UINT16_MAX),
					((rc.channel_count > 1) ? rc.values[1] : UINT16_MAX),
					((rc.channel_count > 2) ? rc.values[2] : UINT16_MAX),
					((rc.channel_count > 3) ? rc.values[3] : UINT16_MAX),
					((rc.channel_count > 4) ? rc.values[4] : UINT16_MAX),
					((rc.channel_count > 5) ? rc.values[5] : UINT16_MAX),
					((rc.channel_count > 6) ? rc.values[6] : UINT16_MAX),
					((rc.channel_count > 7) ? rc.values[7] : UINT16_MAX),
					((rc.channel_count > 8) ? rc.values[8] : UINT16_MAX),
					((rc.channel_count > 9) ? rc.values[9] : UINT16_MAX),
					((rc.channel_count > 10) ? rc.values[10] : UINT16_MAX),
					((rc.channel_count > 11) ? rc.values[11] : UINT16_MAX),
					((rc.channel_count > 12) ? rc.values[12] : UINT16_MAX),
					((rc.channel_count > 13) ? rc.values[13] : UINT16_MAX),
					((rc.channel_count > 14) ? rc.values[14] : UINT16_MAX),
					((rc.channel_count > 15) ? rc.values[15] : UINT16_MAX),
					((rc.channel_count > 16) ? rc.values[16] : UINT16_MAX),
					((rc.channel_count > 17) ? rc.values[17] : UINT16_MAX),
					rc.rssi);
		}
	}
};


class MavlinkStreamManualControl : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamManualControl::get_name_static();
	}

	static const char *get_name_static()
	{
		return "MANUAL_CONTROL";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamManualControl();
	}

private:
	MavlinkOrbSubscription *manual_sub;
	uint64_t manual_time;

protected:
	explicit MavlinkStreamManualControl() : MavlinkStream(),
		manual_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		manual_sub = mavlink->add_orb_subscription(ORB_ID(manual_control_setpoint));
	}

	void send(const hrt_abstime t)
	{
		struct manual_control_setpoint_s manual;

		if (manual_sub->update(&manual_time, &manual)) {
			mavlink_msg_manual_control_send(_channel,
							mavlink_system.sysid,
							manual.x * 1000,
							manual.y * 1000,
							manual.z * 1000,
							manual.r * 1000,
							0);
		}
	}
};


class MavlinkStreamOpticalFlow : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamOpticalFlow::get_name_static();
	}

	static const char *get_name_static()
	{
		return "OPTICAL_FLOW";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamOpticalFlow();
	}

private:
	MavlinkOrbSubscription *flow_sub;
	uint64_t flow_time;

protected:
	explicit MavlinkStreamOpticalFlow() : MavlinkStream(),
		flow_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		flow_sub = mavlink->add_orb_subscription(ORB_ID(optical_flow));
	}

	void send(const hrt_abstime t)
	{
		struct optical_flow_s flow;

		if (flow_sub->update(&flow_time, &flow)) {
			mavlink_msg_optical_flow_send(_channel,
						      flow.timestamp,
						      flow.sensor_id,
						      flow.flow_raw_x, flow.flow_raw_y,
						      flow.flow_comp_x_m, flow.flow_comp_y_m,
						      flow.quality,
						      flow.ground_distance_m);
		}
	}
};

class MavlinkStreamAttitudeControls : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAttitudeControls::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ATTITUDE_CONTROLS";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamAttitudeControls();
	}

private:
	MavlinkOrbSubscription *att_ctrl_sub;
	uint64_t att_ctrl_time;

protected:
	explicit MavlinkStreamAttitudeControls() : MavlinkStream(),
		att_ctrl_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		att_ctrl_sub = mavlink->add_orb_subscription(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	}

	void send(const hrt_abstime t)
	{
		struct actuator_controls_s att_ctrl;

		if (att_ctrl_sub->update(&att_ctrl_time, &att_ctrl)) {
			/* send, add spaces so that string buffer is at least 10 chars long */
			mavlink_msg_named_value_float_send(_channel,
							   att_ctrl.timestamp / 1000,
							   "rll ctrl    ",
							   att_ctrl.control[0]);
			mavlink_msg_named_value_float_send(_channel,
							   att_ctrl.timestamp / 1000,
							   "ptch ctrl    ",
							   att_ctrl.control[1]);
			mavlink_msg_named_value_float_send(_channel,
							   att_ctrl.timestamp / 1000,
							   "yaw ctrl     ",
							   att_ctrl.control[2]);
			mavlink_msg_named_value_float_send(_channel,
							   att_ctrl.timestamp / 1000,
							   "thr ctrl     ",
							   att_ctrl.control[3]);
		}
	}
};

class MavlinkStreamNamedValueFloat : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamNamedValueFloat::get_name_static();
	}

	static const char *get_name_static()
	{
		return "NAMED_VALUE_FLOAT";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamNamedValueFloat();
	}

private:
	MavlinkOrbSubscription *debug_sub;
	uint64_t debug_time;

protected:
	explicit MavlinkStreamNamedValueFloat() : MavlinkStream(),
		debug_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		debug_sub = mavlink->add_orb_subscription(ORB_ID(debug_key_value));
	}

	void send(const hrt_abstime t)
	{
		struct debug_key_value_s debug;

		if (debug_sub->update(&debug_time, &debug)) {
			/* enforce null termination */
			debug.key[sizeof(debug.key) - 1] = '\0';

			mavlink_msg_named_value_float_send(_channel,
							   debug.timestamp_ms,
							   debug.key,
							   debug.value);
		}
	}
};

class MavlinkStreamCameraCapture : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamCameraCapture::get_name_static();
	}

	static const char *get_name_static()
	{
		return "CAMERA_CAPTURE";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamCameraCapture();
	}

private:
	MavlinkOrbSubscription *status_sub;

protected:
	void subscribe(Mavlink *mavlink)
	{
		status_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_status));
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status;
		(void)status_sub->update(&status);

		if (status.arming_state == ARMING_STATE_ARMED
		    || status.arming_state == ARMING_STATE_ARMED_ERROR) {

			/* send camera capture on */
			mavlink_msg_command_long_send(_channel, mavlink_system.sysid, 0, MAV_CMD_DO_CONTROL_VIDEO, 0, 0, 0, 0, 1, 0, 0, 0);

		} else {
			/* send camera capture off */
			mavlink_msg_command_long_send(_channel, mavlink_system.sysid, 0, MAV_CMD_DO_CONTROL_VIDEO, 0, 0, 0, 0, 0, 0, 0, 0);
		}
	}
};

class MavlinkStreamDistanceSensor : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamDistanceSensor::get_name_static();
	}

	static const char *get_name_static()
	{
		return "DISTANCE_SENSOR";
	}

	static MavlinkStream *new_instance()
	{
		return new MavlinkStreamDistanceSensor();
	}

private:
	MavlinkOrbSubscription *range_sub;
	uint64_t range_time;

protected:
	explicit MavlinkStreamDistanceSensor() : MavlinkStream(),
		range_time(0)
	{}

	void subscribe(Mavlink *mavlink)
	{
		range_sub = mavlink->add_orb_subscription(ORB_ID(sensor_range_finder));
	}

	void send(const hrt_abstime t)
	{
		struct range_finder_report range;

		if (range_sub->update(&range_time, &range)) {

			uint8_t type;

			switch (range.type) {
				case RANGE_FINDER_TYPE_LASER:
				type = MAV_DISTANCE_SENSOR_LASER;
				break;
			}

			uint8_t id = 0;
			uint8_t orientation = 0;
			uint8_t covariance = 20;

			mavlink_msg_distance_sensor_send(_channel, range.timestamp / 1000, type, id, orientation,
				range.minimum_distance*100, range.maximum_distance*100, range.distance*100, covariance);
		}
	}
};


StreamListItem *streams_list[] = {
	new StreamListItem(&MavlinkStreamHeartbeat::new_instance, &MavlinkStreamHeartbeat::get_name_static),
	new StreamListItem(&MavlinkStreamSysStatus::new_instance, &MavlinkStreamSysStatus::get_name_static),
	new StreamListItem(&MavlinkStreamHighresIMU::new_instance, &MavlinkStreamHighresIMU::get_name_static),
	new StreamListItem(&MavlinkStreamAttitude::new_instance, &MavlinkStreamAttitude::get_name_static),
	new StreamListItem(&MavlinkStreamAttitudeQuaternion::new_instance, &MavlinkStreamAttitudeQuaternion::get_name_static),
	new StreamListItem(&MavlinkStreamVFRHUD::new_instance, &MavlinkStreamVFRHUD::get_name_static),
	new StreamListItem(&MavlinkStreamGPSRawInt::new_instance, &MavlinkStreamGPSRawInt::get_name_static),
	new StreamListItem(&MavlinkStreamGlobalPositionInt::new_instance, &MavlinkStreamGlobalPositionInt::get_name_static),
	new StreamListItem(&MavlinkStreamLocalPositionNED::new_instance, &MavlinkStreamLocalPositionNED::get_name_static),
	new StreamListItem(&MavlinkStreamGPSGlobalOrigin::new_instance, &MavlinkStreamGPSGlobalOrigin::get_name_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<0>::new_instance, &MavlinkStreamServoOutputRaw<0>::get_name_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<1>::new_instance, &MavlinkStreamServoOutputRaw<1>::get_name_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<2>::new_instance, &MavlinkStreamServoOutputRaw<2>::get_name_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<3>::new_instance, &MavlinkStreamServoOutputRaw<3>::get_name_static),
	new StreamListItem(&MavlinkStreamHILControls::new_instance, &MavlinkStreamHILControls::get_name_static),
	new StreamListItem(&MavlinkStreamGlobalPositionSetpointInt::new_instance, &MavlinkStreamGlobalPositionSetpointInt::get_name_static),
	new StreamListItem(&MavlinkStreamLocalPositionSetpoint::new_instance, &MavlinkStreamLocalPositionSetpoint::get_name_static),
	new StreamListItem(&MavlinkStreamRollPitchYawThrustSetpoint::new_instance, &MavlinkStreamRollPitchYawThrustSetpoint::get_name_static),
	new StreamListItem(&MavlinkStreamRollPitchYawRatesThrustSetpoint::new_instance, &MavlinkStreamRollPitchYawRatesThrustSetpoint::get_name_static),
	new StreamListItem(&MavlinkStreamRCChannelsRaw::new_instance, &MavlinkStreamRCChannelsRaw::get_name_static),
	new StreamListItem(&MavlinkStreamManualControl::new_instance, &MavlinkStreamManualControl::get_name_static),
	new StreamListItem(&MavlinkStreamOpticalFlow::new_instance, &MavlinkStreamOpticalFlow::get_name_static),
	new StreamListItem(&MavlinkStreamAttitudeControls::new_instance, &MavlinkStreamAttitudeControls::get_name_static),
	new StreamListItem(&MavlinkStreamNamedValueFloat::new_instance, &MavlinkStreamNamedValueFloat::get_name_static),
	new StreamListItem(&MavlinkStreamCameraCapture::new_instance, &MavlinkStreamCameraCapture::get_name_static),
	new StreamListItem(&MavlinkStreamDistanceSensor::new_instance, &MavlinkStreamDistanceSensor::get_name_static),
	new StreamListItem(&MavlinkStreamViconPositionEstimate::new_instance, &MavlinkStreamViconPositionEstimate::get_name_static),
	nullptr
};
