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

		} else if (status->main_state == MAIN_STATE_SEATBELT) {
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_SEATBELT;

		} else if (status->main_state == MAIN_STATE_EASY) {
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_EASY;

		} else if (status->main_state == MAIN_STATE_AUTO) {
			*mavlink_base_mode |= MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_READY;
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
	const char *get_name()
	{
		return "HEARTBEAT";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamHeartbeat();
	}

private:
	MavlinkOrbSubscription *status_sub;
	struct vehicle_status_s *status;

	MavlinkOrbSubscription *pos_sp_triplet_sub;
	struct position_setpoint_triplet_s *pos_sp_triplet;

protected:
	void subscribe(Mavlink *mavlink)
	{
		status_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_status));
		status = (struct vehicle_status_s *)status_sub->get_data();

		pos_sp_triplet_sub = mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet));
		pos_sp_triplet = (struct position_setpoint_triplet_s *)pos_sp_triplet_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		status_sub->update(t);
		pos_sp_triplet_sub->update(t);

		uint8_t mavlink_state = 0;
		uint8_t mavlink_base_mode = 0;
		uint32_t mavlink_custom_mode = 0;
		get_mavlink_mode_state(status, pos_sp_triplet, &mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

		mavlink_msg_heartbeat_send(_channel,
					   mavlink_system.type,
					   MAV_AUTOPILOT_PX4,
					   mavlink_base_mode,
					   mavlink_custom_mode,
					   mavlink_state);

	}
};


class MavlinkStreamSysStatus : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "SYS_STATUS";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamSysStatus();
	}

private:
	MavlinkOrbSubscription *status_sub;
	struct vehicle_status_s *status;

protected:
	void subscribe(Mavlink *mavlink)
	{
		status_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_status));
		status = (struct vehicle_status_s *)status_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		status_sub->update(t);

		mavlink_msg_sys_status_send(_channel,
					    status->onboard_control_sensors_present,
					    status->onboard_control_sensors_enabled,
					    status->onboard_control_sensors_health,
					    status->load * 1000.0f,
					    status->battery_voltage * 1000.0f,
					    status->battery_current * 1000.0f,
					    status->battery_remaining,
					    status->drop_rate_comm,
					    status->errors_comm,
					    status->errors_count1,
					    status->errors_count2,
					    status->errors_count3,
					    status->errors_count4);
	}
};


class MavlinkStreamHighresIMU : public MavlinkStream
{
public:
	MavlinkStreamHighresIMU() : MavlinkStream(), accel_counter(0), gyro_counter(0), mag_counter(0), baro_counter(0)
	{
	}

	const char *get_name()
	{
		return "HIGHRES_IMU";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamHighresIMU();
	}

private:
	MavlinkOrbSubscription *sensor_sub;
	struct sensor_combined_s *sensor;

	uint32_t accel_counter;
	uint32_t gyro_counter;
	uint32_t mag_counter;
	uint32_t baro_counter;

protected:
	void subscribe(Mavlink *mavlink)
	{
		sensor_sub = mavlink->add_orb_subscription(ORB_ID(sensor_combined));
		sensor = (struct sensor_combined_s *)sensor_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		sensor_sub->update(t);

		uint16_t fields_updated = 0;

		if (accel_counter != sensor->accelerometer_counter) {
			/* mark first three dimensions as changed */
			fields_updated |= (1 << 0) | (1 << 1) | (1 << 2);
			accel_counter = sensor->accelerometer_counter;
		}

		if (gyro_counter != sensor->gyro_counter) {
			/* mark second group dimensions as changed */
			fields_updated |= (1 << 3) | (1 << 4) | (1 << 5);
			gyro_counter = sensor->gyro_counter;
		}

		if (mag_counter != sensor->magnetometer_counter) {
			/* mark third group dimensions as changed */
			fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);
			mag_counter = sensor->magnetometer_counter;
		}

		if (baro_counter != sensor->baro_counter) {
			/* mark last group dimensions as changed */
			fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);
			baro_counter = sensor->baro_counter;
		}

		mavlink_msg_highres_imu_send(_channel,
					     sensor->timestamp,
					     sensor->accelerometer_m_s2[0], sensor->accelerometer_m_s2[1], sensor->accelerometer_m_s2[2],
					     sensor->gyro_rad_s[0], sensor->gyro_rad_s[1], sensor->gyro_rad_s[2],
					     sensor->magnetometer_ga[0], sensor->magnetometer_ga[1], sensor->magnetometer_ga[2],
					     sensor->baro_pres_mbar, sensor->differential_pressure_pa,
					     sensor->baro_alt_meter, sensor->baro_temp_celcius,
					     fields_updated);
	}
};


class MavlinkStreamAttitude : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "ATTITUDE";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamAttitude();
	}

private:
	MavlinkOrbSubscription *att_sub;
	struct vehicle_attitude_s *att;

protected:
	void subscribe(Mavlink *mavlink)
	{
		att_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_attitude));
		att = (struct vehicle_attitude_s *)att_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		att_sub->update(t);

		mavlink_msg_attitude_send(_channel,
					  att->timestamp / 1000,
					  att->roll, att->pitch, att->yaw,
					  att->rollspeed, att->pitchspeed, att->yawspeed);
	}
};


class MavlinkStreamAttitudeQuaternion : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "ATTITUDE_QUATERNION";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamAttitudeQuaternion();
	}

private:
	MavlinkOrbSubscription *att_sub;
	struct vehicle_attitude_s *att;

protected:
	void subscribe(Mavlink *mavlink)
	{
		att_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_attitude));
		att = (struct vehicle_attitude_s *)att_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		att_sub->update(t);

		mavlink_msg_attitude_quaternion_send(_channel,
						     att->timestamp / 1000,
						     att->q[0],
						     att->q[1],
						     att->q[2],
						     att->q[3],
						     att->rollspeed,
						     att->pitchspeed,
						     att->yawspeed);
	}
};


class MavlinkStreamVFRHUD : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "VFR_HUD";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamVFRHUD();
	}

private:
	MavlinkOrbSubscription *att_sub;
	struct vehicle_attitude_s *att;

	MavlinkOrbSubscription *pos_sub;
	struct vehicle_global_position_s *pos;

	MavlinkOrbSubscription *armed_sub;
	struct actuator_armed_s *armed;

	MavlinkOrbSubscription *act_sub;
	struct actuator_controls_s *act;

	MavlinkOrbSubscription *airspeed_sub;
	struct airspeed_s *airspeed;

protected:
	void subscribe(Mavlink *mavlink)
	{
		att_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_attitude));
		att = (struct vehicle_attitude_s *)att_sub->get_data();

		pos_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_global_position));
		pos = (struct vehicle_global_position_s *)pos_sub->get_data();

		armed_sub = mavlink->add_orb_subscription(ORB_ID(actuator_armed));
		armed = (struct actuator_armed_s *)armed_sub->get_data();

		act_sub = mavlink->add_orb_subscription(ORB_ID(actuator_controls_0));
		act = (struct actuator_controls_s *)act_sub->get_data();

		airspeed_sub = mavlink->add_orb_subscription(ORB_ID(airspeed));
		airspeed = (struct airspeed_s *)airspeed_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		att_sub->update(t);
		pos_sub->update(t);
		armed_sub->update(t);
		act_sub->update(t);
		airspeed_sub->update(t);

		float groundspeed = sqrtf(pos->vel_n * pos->vel_n + pos->vel_e * pos->vel_e);
		uint16_t heading = _wrap_2pi(att->yaw) * M_RAD_TO_DEG_F;
		float throttle = armed->armed ? act->control[3] * 100.0f : 0.0f;

		mavlink_msg_vfr_hud_send(_channel,
					 airspeed->true_airspeed_m_s,
					 groundspeed,
					 heading,
					 throttle,
					 pos->alt,
					 -pos->vel_d);
	}
};


class MavlinkStreamGPSRawInt : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "GPS_RAW_INT";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamGPSRawInt();
	}

private:
	MavlinkOrbSubscription *gps_sub;
	struct vehicle_gps_position_s *gps;

protected:
	void subscribe(Mavlink *mavlink)
	{
		gps_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_gps_position));
		gps = (struct vehicle_gps_position_s *)gps_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		gps_sub->update(t);

		mavlink_msg_gps_raw_int_send(_channel,
					     gps->timestamp_position,
					     gps->fix_type,
					     gps->lat,
					     gps->lon,
					     gps->alt,
					     cm_uint16_from_m_float(gps->eph_m),
					     cm_uint16_from_m_float(gps->epv_m),
					     gps->vel_m_s * 100.0f,
					     _wrap_2pi(gps->cog_rad) * M_RAD_TO_DEG_F * 1e2f,
					     gps->satellites_visible);
	}
};


class MavlinkStreamGlobalPositionInt : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "GLOBAL_POSITION_INT";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamGlobalPositionInt();
	}

private:
	MavlinkOrbSubscription *pos_sub;
	struct vehicle_global_position_s *pos;

	MavlinkOrbSubscription *home_sub;
	struct home_position_s *home;

protected:
	void subscribe(Mavlink *mavlink)
	{
		pos_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_global_position));
		pos = (struct vehicle_global_position_s *)pos_sub->get_data();

		home_sub = mavlink->add_orb_subscription(ORB_ID(home_position));
		home = (struct home_position_s *)home_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		pos_sub->update(t);
		home_sub->update(t);

		mavlink_msg_global_position_int_send(_channel,
						     pos->timestamp / 1000,
						     pos->lat * 1e7,
						     pos->lon * 1e7,
						     pos->alt * 1000.0f,
						     (pos->alt - home->alt) * 1000.0f,
						     pos->vel_n * 100.0f,
						     pos->vel_e * 100.0f,
						     pos->vel_d * 100.0f,
						     _wrap_2pi(pos->yaw) * M_RAD_TO_DEG_F * 100.0f);
	}
};


class MavlinkStreamLocalPositionNED : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "LOCAL_POSITION_NED";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamLocalPositionNED();
	}

private:
	MavlinkOrbSubscription *pos_sub;
	struct vehicle_local_position_s *pos;

protected:
	void subscribe(Mavlink *mavlink)
	{
		pos_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_local_position));
		pos = (struct vehicle_local_position_s *)pos_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		pos_sub->update(t);

		mavlink_msg_local_position_ned_send(_channel,
						    pos->timestamp / 1000,
						    pos->x,
						    pos->y,
						    pos->z,
						    pos->vx,
						    pos->vy,
						    pos->vz);
	}
};


class MavlinkStreamGPSGlobalOrigin : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "GPS_GLOBAL_ORIGIN";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamGPSGlobalOrigin();
	}

private:
	MavlinkOrbSubscription *home_sub;
	struct home_position_s *home;

protected:
	void subscribe(Mavlink *mavlink)
	{
		home_sub = mavlink->add_orb_subscription(ORB_ID(home_position));
		home = (struct home_position_s *)home_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		home_sub->update(t);

		mavlink_msg_gps_global_origin_send(_channel,
						   (int32_t)(home->lat * 1e7),
						   (int32_t)(home->lon * 1e7),
						   (int32_t)(home->alt) * 1000.0f);
	}
};


class MavlinkStreamServoOutputRaw : public MavlinkStream
{
public:
	MavlinkStreamServoOutputRaw(unsigned int n) : MavlinkStream(), _n(n)
	{
		sprintf(_name, "SERVO_OUTPUT_RAW_%d", _n);
	}

	const char *get_name()
	{
		return _name;
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamServoOutputRaw(_n);
	}

private:
	MavlinkOrbSubscription *act_sub;
	struct actuator_outputs_s *act;

	char _name[20];
	unsigned int _n;

protected:
	void subscribe(Mavlink *mavlink)
	{
		orb_id_t act_topics[] = {
			ORB_ID(actuator_outputs_0),
			ORB_ID(actuator_outputs_1),
			ORB_ID(actuator_outputs_2),
			ORB_ID(actuator_outputs_3)
		};

		act_sub = mavlink->add_orb_subscription(act_topics[_n]);
		act = (struct actuator_outputs_s *)act_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		act_sub->update(t);

		mavlink_msg_servo_output_raw_send(_channel,
						  act->timestamp / 1000,
						  _n,
						  act->output[0],
						  act->output[1],
						  act->output[2],
						  act->output[3],
						  act->output[4],
						  act->output[5],
						  act->output[6],
						  act->output[7]);
	}
};


class MavlinkStreamHILControls : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "HIL_CONTROLS";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamHILControls();
	}

private:
	MavlinkOrbSubscription *status_sub;
	struct vehicle_status_s *status;

	MavlinkOrbSubscription *pos_sp_triplet_sub;
	struct position_setpoint_triplet_s *pos_sp_triplet;

	MavlinkOrbSubscription *act_sub;
	struct actuator_outputs_s *act;

protected:
	void subscribe(Mavlink *mavlink)
	{
		status_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_status));
		status = (struct vehicle_status_s *)status_sub->get_data();

		pos_sp_triplet_sub = mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet));
		pos_sp_triplet = (struct position_setpoint_triplet_s *)pos_sp_triplet_sub->get_data();

		act_sub = mavlink->add_orb_subscription(ORB_ID(actuator_outputs_0));
		act = (struct actuator_outputs_s *)act_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		status_sub->update(t);
		pos_sp_triplet_sub->update(t);
		act_sub->update(t);

		/* translate the current syste state to mavlink state and mode */
		uint8_t mavlink_state;
		uint8_t mavlink_base_mode;
		uint32_t mavlink_custom_mode;
		get_mavlink_mode_state(status, pos_sp_triplet, &mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

		/* HIL message as per MAVLink spec */

		/* scale / assign outputs depending on system type */
		if (mavlink_system.type == MAV_TYPE_QUADROTOR) {
			mavlink_msg_hil_controls_send(_channel,
						      hrt_absolute_time(),
						      ((act->output[0] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[1] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[2] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[3] - 900.0f) / 600.0f) / 2.0f,
						      -1,
						      -1,
						      -1,
						      -1,
						      mavlink_base_mode,
						      0);

		} else if (mavlink_system.type == MAV_TYPE_HEXAROTOR) {
			mavlink_msg_hil_controls_send(_channel,
						      hrt_absolute_time(),
						      ((act->output[0] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[1] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[2] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[3] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[4] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[5] - 900.0f) / 600.0f) / 2.0f,
						      -1,
						      -1,
						      mavlink_base_mode,
						      0);

		} else if (mavlink_system.type == MAV_TYPE_OCTOROTOR) {
			mavlink_msg_hil_controls_send(_channel,
						      hrt_absolute_time(),
						      ((act->output[0] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[1] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[2] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[3] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[4] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[5] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[6] - 900.0f) / 600.0f) / 2.0f,
						      ((act->output[7] - 900.0f) / 600.0f) / 2.0f,
						      mavlink_base_mode,
						      0);

		} else {
			mavlink_msg_hil_controls_send(_channel,
						      hrt_absolute_time(),
						      (act->output[0] - 1500.0f) / 500.0f,
						      (act->output[1] - 1500.0f) / 500.0f,
						      (act->output[2] - 1500.0f) / 500.0f,
						      (act->output[3] - 1000.0f) / 1000.0f,
						      (act->output[4] - 1500.0f) / 500.0f,
						      (act->output[5] - 1500.0f) / 500.0f,
						      (act->output[6] - 1500.0f) / 500.0f,
						      (act->output[7] - 1500.0f) / 500.0f,
						      mavlink_base_mode,
						      0);
		}
	}
};


class MavlinkStreamGlobalPositionSetpointInt : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "GLOBAL_POSITION_SETPOINT_INT";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamGlobalPositionSetpointInt();
	}

private:
	MavlinkOrbSubscription *pos_sp_triplet_sub;
	struct position_setpoint_triplet_s *pos_sp_triplet;

protected:
	void subscribe(Mavlink *mavlink)
	{
		pos_sp_triplet_sub = mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet));
		pos_sp_triplet = (struct position_setpoint_triplet_s *)pos_sp_triplet_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		pos_sp_triplet_sub->update(t);

		mavlink_msg_global_position_setpoint_int_send(_channel,
				MAV_FRAME_GLOBAL,
				(int32_t)(pos_sp_triplet->current.lat * 1e7),
				(int32_t)(pos_sp_triplet->current.lon * 1e7),
				(int32_t)(pos_sp_triplet->current.alt * 1000),
				(int16_t)(pos_sp_triplet->current.yaw * M_RAD_TO_DEG_F * 100.0f));
	}
};


class MavlinkStreamLocalPositionSetpoint : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "LOCAL_POSITION_SETPOINT";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamLocalPositionSetpoint();
	}

private:
	MavlinkOrbSubscription *pos_sp_sub;
	struct vehicle_local_position_setpoint_s *pos_sp;

protected:
	void subscribe(Mavlink *mavlink)
	{
		pos_sp_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_local_position_setpoint));
		pos_sp = (struct vehicle_local_position_setpoint_s *)pos_sp_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		pos_sp_sub->update(t);

		mavlink_msg_local_position_setpoint_send(_channel,
				MAV_FRAME_LOCAL_NED,
				pos_sp->x,
				pos_sp->y,
				pos_sp->z,
				pos_sp->yaw);
	}
};


class MavlinkStreamRollPitchYawThrustSetpoint : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "ROLL_PITCH_YAW_THRUST_SETPOINT";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamRollPitchYawThrustSetpoint();
	}

private:
	MavlinkOrbSubscription *att_sp_sub;
	struct vehicle_attitude_setpoint_s *att_sp;

protected:
	void subscribe(Mavlink *mavlink)
	{
		att_sp_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_attitude_setpoint));
		att_sp = (struct vehicle_attitude_setpoint_s *)att_sp_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		att_sp_sub->update(t);

		mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(_channel,
				att_sp->timestamp / 1000,
				att_sp->roll_body,
				att_sp->pitch_body,
				att_sp->yaw_body,
				att_sp->thrust);
	}
};


class MavlinkStreamRollPitchYawRatesThrustSetpoint : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "ROLL_PITCH_YAW_RATES_THRUST_SETPOINT";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamRollPitchYawRatesThrustSetpoint();
	}

private:
	MavlinkOrbSubscription *att_rates_sp_sub;
	struct vehicle_rates_setpoint_s *att_rates_sp;

protected:
	void subscribe(Mavlink *mavlink)
	{
		att_rates_sp_sub = mavlink->add_orb_subscription(ORB_ID(vehicle_rates_setpoint));
		att_rates_sp = (struct vehicle_rates_setpoint_s *)att_rates_sp_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		att_rates_sp_sub->update(t);

		mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_send(_channel,
				att_rates_sp->timestamp / 1000,
				att_rates_sp->roll,
				att_rates_sp->pitch,
				att_rates_sp->yaw,
				att_rates_sp->thrust);
	}
};


class MavlinkStreamRCChannelsRaw : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "RC_CHANNELS_RAW";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamRCChannelsRaw();
	}

private:
	MavlinkOrbSubscription *rc_sub;
	struct rc_input_values *rc;

protected:
	void subscribe(Mavlink *mavlink)
	{
		rc_sub = mavlink->add_orb_subscription(ORB_ID(input_rc));
		rc = (struct rc_input_values *)rc_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		rc_sub->update(t);

		const unsigned port_width = 8;

		for (unsigned i = 0; (i * port_width) < rc->channel_count; i++) {
			/* Channels are sent in MAVLink main loop at a fixed interval */
			mavlink_msg_rc_channels_raw_send(_channel,
							 rc->timestamp_publication / 1000,
							 i,
							 (rc->channel_count > (i * port_width) + 0) ? rc->values[(i * port_width) + 0] : UINT16_MAX,
							 (rc->channel_count > (i * port_width) + 1) ? rc->values[(i * port_width) + 1] : UINT16_MAX,
							 (rc->channel_count > (i * port_width) + 2) ? rc->values[(i * port_width) + 2] : UINT16_MAX,
							 (rc->channel_count > (i * port_width) + 3) ? rc->values[(i * port_width) + 3] : UINT16_MAX,
							 (rc->channel_count > (i * port_width) + 4) ? rc->values[(i * port_width) + 4] : UINT16_MAX,
							 (rc->channel_count > (i * port_width) + 5) ? rc->values[(i * port_width) + 5] : UINT16_MAX,
							 (rc->channel_count > (i * port_width) + 6) ? rc->values[(i * port_width) + 6] : UINT16_MAX,
							 (rc->channel_count > (i * port_width) + 7) ? rc->values[(i * port_width) + 7] : UINT16_MAX,
							 rc->rssi);
		}
	}
};


class MavlinkStreamManualControl : public MavlinkStream
{
public:
	const char *get_name()
	{
		return "MANUAL_CONTROL";
	}

	MavlinkStream *new_instance()
	{
		return new MavlinkStreamManualControl();
	}

private:
	MavlinkOrbSubscription *manual_sub;
	struct manual_control_setpoint_s *manual;

protected:
	void subscribe(Mavlink *mavlink)
	{
		manual_sub = mavlink->add_orb_subscription(ORB_ID(manual_control_setpoint));
		manual = (struct manual_control_setpoint_s *)manual_sub->get_data();
	}

	void send(const hrt_abstime t)
	{
		manual_sub->update(t);

		mavlink_msg_manual_control_send(_channel,
						mavlink_system.sysid,
						manual->roll * 1000,
						manual->pitch * 1000,
						manual->yaw * 1000,
						manual->throttle * 1000,
						0);
	}
};


MavlinkStream *streams_list[] = {
	new MavlinkStreamHeartbeat(),
	new MavlinkStreamSysStatus(),
	new MavlinkStreamHighresIMU(),
	new MavlinkStreamAttitude(),
	new MavlinkStreamAttitudeQuaternion(),
	new MavlinkStreamVFRHUD(),
	new MavlinkStreamGPSRawInt(),
	new MavlinkStreamGlobalPositionInt(),
	new MavlinkStreamLocalPositionNED(),
	new MavlinkStreamGPSGlobalOrigin(),
	new MavlinkStreamServoOutputRaw(0),
	new MavlinkStreamServoOutputRaw(1),
	new MavlinkStreamServoOutputRaw(2),
	new MavlinkStreamServoOutputRaw(3),
	new MavlinkStreamHILControls(),
	new MavlinkStreamGlobalPositionSetpointInt(),
	new MavlinkStreamLocalPositionSetpoint(),
	new MavlinkStreamRollPitchYawThrustSetpoint(),
	new MavlinkStreamRollPitchYawRatesThrustSetpoint(),
	new MavlinkStreamRCChannelsRaw(),
	new MavlinkStreamManualControl(),
	nullptr
};







//
//
//
//
//
//
//void
//MavlinkOrbListener::l_vehicle_attitude_controls(const struct listener *l)
//{
//	orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, l->mavlink->get_subs()->actuators_sub, &l->listener->actuators_0);
//
//	if (l->mavlink->get_mode() == Mavlink::MODE_OFFBOARD) {
//		/* send, add spaces so that string buffer is at least 10 chars long */
//		mavlink_msg_named_value_float_send(l->mavlink->get_chan(),
//						   l->listener->last_sensor_timestamp / 1000,
//						   "ctrl0    ",
//						   l->listener->actuators_0.control[0]);
//		mavlink_msg_named_value_float_send(l->mavlink->get_chan(),
//						   l->listener->last_sensor_timestamp / 1000,
//						   "ctrl1    ",
//						   l->listener->actuators_0.control[1]);
//		mavlink_msg_named_value_float_send(l->mavlink->get_chan(),
//						   l->listener->last_sensor_timestamp / 1000,
//						   "ctrl2     ",
//						   l->listener->actuators_0.control[2]);
//		mavlink_msg_named_value_float_send(l->mavlink->get_chan(),
//						   l->listener->last_sensor_timestamp / 1000,
//						   "ctrl3     ",
//						   l->listener->actuators_0.control[3]);
//	}
//}
//
//void
//MavlinkOrbListener::l_debug_key_value(const struct listener *l)
//{
//	struct debug_key_value_s debug;
//
//	orb_copy(ORB_ID(debug_key_value), l->mavlink->get_subs()->debug_key_value, &debug);
//
//	/* Enforce null termination */
//	debug.key[sizeof(debug.key) - 1] = '\0';
//
//	mavlink_msg_named_value_float_send(l->mavlink->get_chan(),
//					   l->listener->last_sensor_timestamp / 1000,
//					   debug.key,
//					   debug.value);
//}
//
//void
//MavlinkOrbListener::l_optical_flow(const struct listener *l)
//{
//	struct optical_flow_s flow;
//
//	orb_copy(ORB_ID(optical_flow), l->mavlink->get_subs()->optical_flow, &flow);
//
//	mavlink_msg_optical_flow_send(l->mavlink->get_chan(), flow.timestamp, flow.sensor_id, flow.flow_raw_x, flow.flow_raw_y,
//				      flow.flow_comp_x_m, flow.flow_comp_y_m, flow.quality, flow.ground_distance_m);
//}
//
//void
//MavlinkOrbListener::l_nav_cap(const struct listener *l)
//{
//
//	orb_copy(ORB_ID(navigation_capabilities), l->mavlink->get_subs()->navigation_capabilities_sub, &l->listener->nav_cap);
//
//	mavlink_msg_named_value_float_send(l->mavlink->get_chan(),
//				   hrt_absolute_time() / 1000,
//				   "turn dist",
//				   l->listener->nav_cap.turn_distance);
//
//}
