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
 * @file orb_listener.cpp
 * Monitors ORB topics and sends update messages as appropriate.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 */

// XXX trim includes
#include <nuttx/config.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include "mavlink_bridge_header.h"
#include <drivers/drv_hrt.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <stdlib.h>
#include <poll.h>
#include <lib/geo/geo.h>
#include <systemlib/err.h>


#include <mavlink/mavlink_log.h>

#include "mavlink_orb_listener.h"
#include "mavlink_main.h"



uint16_t cm_uint16_from_m_float(float m);


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

MavlinkOrbListener::MavlinkOrbListener(Mavlink* parent) :

	_loop_perf(perf_alloc(PC_ELAPSED, "mavlink orb")),
	_mavlink(parent),
	_listeners(nullptr),
	_n_listeners(0)
{
	add_listener(MavlinkOrbListener::l_sensor_combined,		&(_mavlink->get_subs()->sensor_sub),			0);
	add_listener(MavlinkOrbListener::l_vehicle_attitude,		&(_mavlink->get_subs()->att_sub),			0);
	add_listener(MavlinkOrbListener::l_vehicle_gps_position,	&(_mavlink->get_subs()->gps_sub),			0);
	add_listener(MavlinkOrbListener::l_vehicle_status,		&(_mavlink->get_subs()->status_sub),			0);
	add_listener(MavlinkOrbListener::l_home,			&(_mavlink->get_subs()->home_sub),			0);
	add_listener(MavlinkOrbListener::l_rc_channels,			&(_mavlink->get_subs()->rc_sub),			0);
	add_listener(MavlinkOrbListener::l_input_rc,			&(_mavlink->get_subs()->input_rc_sub),			0);
	add_listener(MavlinkOrbListener::l_global_position,		&(_mavlink->get_subs()->global_pos_sub),		0);
	add_listener(MavlinkOrbListener::l_local_position,		&(_mavlink->get_subs()->local_pos_sub),			0);
	add_listener(MavlinkOrbListener::l_global_position_setpoint,	&(_mavlink->get_subs()->triplet_sub),			0);
	add_listener(MavlinkOrbListener::l_local_position_setpoint,	&(_mavlink->get_subs()->spl_sub),			0);
	add_listener(MavlinkOrbListener::l_attitude_setpoint,		&(_mavlink->get_subs()->spa_sub),			0);
	add_listener(MavlinkOrbListener::l_actuator_outputs,		&(_mavlink->get_subs()->act_0_sub),			0);
	add_listener(MavlinkOrbListener::l_actuator_outputs,		&(_mavlink->get_subs()->act_1_sub),			1);
	add_listener(MavlinkOrbListener::l_actuator_outputs,		&(_mavlink->get_subs()->act_2_sub),			2);
	add_listener(MavlinkOrbListener::l_actuator_outputs,		&(_mavlink->get_subs()->act_3_sub),			3);
	add_listener(MavlinkOrbListener::l_actuator_armed,		&(_mavlink->get_subs()->armed_sub),			0);
	add_listener(MavlinkOrbListener::l_manual_control_setpoint,	&(_mavlink->get_subs()->man_control_sp_sub), 		0);
	add_listener(MavlinkOrbListener::l_vehicle_attitude_controls,	&(_mavlink->get_subs()->actuators_sub),			0);
	add_listener(MavlinkOrbListener::l_debug_key_value,		&(_mavlink->get_subs()->debug_key_value),		0);
	add_listener(MavlinkOrbListener::l_optical_flow,		&(_mavlink->get_subs()->optical_flow),			0);
	add_listener(MavlinkOrbListener::l_vehicle_rates_setpoint,	&(_mavlink->get_subs()->rates_setpoint_sub),		0);
	add_listener(MavlinkOrbListener::l_airspeed,			&(_mavlink->get_subs()->airspeed_sub),			0);
	add_listener(MavlinkOrbListener::l_nav_cap,			&(_mavlink->get_subs()->navigation_capabilities_sub),	0);

}

void MavlinkOrbListener::add_listener(void (*callback)(const struct listener *l), int *subp, uintptr_t arg)
{
	struct listener *nl = new listener;

	nl->callback = callback;
	nl->subp = subp;
	nl->arg = arg;
	nl->next = nullptr;
	nl->mavlink = _mavlink;
	nl->listener = this;

	if (_listeners == nullptr) {
		_listeners = nl;
	} else {
		struct listener *next = _listeners;
		while (next->next != nullptr) {
			next = next->next;
		}
		next->next = nl;
	}
	_n_listeners++;
}

void
MavlinkOrbListener::l_sensor_combined(const struct listener *l)
{
	struct sensor_combined_s raw;

	/* copy sensors raw data into local buffer */
	orb_copy(ORB_ID(sensor_combined), l->mavlink->get_subs()->sensor_sub, &raw);

	/* mark individual fields as _mavlink->get_chan()ged */
	uint16_t fields_updated = 0;

	// if (accel_counter != raw.accelerometer_counter) {
	// 	/* mark first three dimensions as _mavlink->get_chan()ged */
	// 	fields_updated |= (1 << 0) | (1 << 1) | (1 << 2);
	// 	accel_counter = raw.accelerometer_counter;
	// }

	// if (gyro_counter != raw.gyro_counter) {
	// 	/* mark second group dimensions as _mavlink->get_chan()ged */
	// 	fields_updated |= (1 << 3) | (1 << 4) | (1 << 5);
	// 	gyro_counter = raw.gyro_counter;
	// }

	// if (mag_counter != raw.magnetometer_counter) {
	// 	/* mark third group dimensions as _mavlink->get_chan()ged */
	// 	fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);
	// 	mag_counter = raw.magnetometer_counter;
	// }

	// if (baro_counter != raw.baro_counter) {
	// 	/* mark last group dimensions as _mavlink->get_chan()ged */
	// 	fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);
	// 	baro_counter = raw.baro_counter;
	// }

	if (l->mavlink->get_mode() == Mavlink::MODE_OFFBOARD)
		mavlink_msg_highres_imu_send(l->mavlink->get_chan(), l->listener->last_sensor_timestamp,
					     raw.accelerometer_m_s2[0], raw.accelerometer_m_s2[1],
					     raw.accelerometer_m_s2[2], raw.gyro_rad_s[0],
					     raw.gyro_rad_s[1], raw.gyro_rad_s[2],
					     raw.magnetometer_ga[0],
					     raw.magnetometer_ga[1], raw.magnetometer_ga[2],
					     raw.baro_pres_mbar, raw.differential_pressure_pa,
					     raw.baro_alt_meter, raw.baro_temp_celcius,
					     fields_updated);

	l->listener->sensors_raw_counter++;
}

void
MavlinkOrbListener::l_vehicle_attitude(const struct listener *l)
{
	/* copy attitude data into local buffer */
	orb_copy(ORB_ID(vehicle_attitude), l->mavlink->get_subs()->att_sub, &l->listener->att);

	if (l->mavlink->get_mode() == Mavlink::MODE_OFFBOARD) {
		/* send sensor values */
		mavlink_msg_attitude_send(l->mavlink->get_chan(),
					  l->listener->last_sensor_timestamp / 1000,
					  l->listener->att.roll,
					  l->listener->att.pitch,
					  l->listener->att.yaw,
					  l->listener->att.rollspeed,
					  l->listener->att.pitchspeed,
					  l->listener->att.yawspeed);
					  	
		/* limit VFR message rate to 10Hz */
		hrt_abstime t = hrt_absolute_time();
		if (t >= l->listener->last_sent_vfr + 100000) {
			l->listener->last_sent_vfr = t;
			float groundspeed = sqrtf(l->listener->global_pos.vel_n * l->listener->global_pos.vel_n + l->listener->global_pos.vel_e * l->listener->global_pos.vel_e);
			uint16_t heading = _wrap_2pi(l->listener->att.yaw) * M_RAD_TO_DEG_F;
			float throttle = l->listener->armed.armed ? l->listener->actuators_0.control[3] * 100.0f : 0.0f;
			mavlink_msg_vfr_hud_send(l->mavlink->get_chan(), l->listener->airspeed.true_airspeed_m_s, groundspeed, heading, throttle, l->listener->global_pos.alt, -l->listener->global_pos.vel_d);
		}
		
		/* send quaternion values if it exists */
		if(l->listener->att.q_valid) {
			mavlink_msg_attitude_quaternion_send(l->mavlink->get_chan(),
												l->listener->last_sensor_timestamp / 1000,
												l->listener->att.q[0],
												l->listener->att.q[1],
												l->listener->att.q[2],
												l->listener->att.q[3],
												l->listener->att.rollspeed,
												l->listener->att.pitchspeed,
												l->listener->att.yawspeed);
		}
	}

	l->listener->attitude_counter++;
}

void
MavlinkOrbListener::l_vehicle_gps_position(const struct listener *l)
{
	struct vehicle_gps_position_s gps;

	/* copy gps data into local buffer */
	orb_copy(ORB_ID(vehicle_gps_position), l->mavlink->get_subs()->gps_sub, &gps);

	/* GPS COG is 0..2PI in degrees * 1e2 */
	float cog_deg = _wrap_2pi(gps.cog_rad) * M_RAD_TO_DEG_F;

	/* GPS position */
	mavlink_msg_gps_raw_int_send(l->mavlink->get_chan(),
				     gps.timestamp_position,
				     gps.fix_type,
				     gps.lat,
				     gps.lon,
				     gps.alt,
				     cm_uint16_from_m_float(gps.eph_m),
				     cm_uint16_from_m_float(gps.epv_m),
				     gps.vel_m_s * 1e2f, // from m/s to cm/s
				     cog_deg * 1e2f, // from deg to deg * 100
				     gps.satellites_visible);

	/* update SAT info every 10 seconds */
	if (gps.satellite_info_available && (l->listener->gps_counter % 50 == 0)) {
		mavlink_msg_gps_status_send(l->mavlink->get_chan(),
					    gps.satellites_visible,
					    gps.satellite_prn,
					    gps.satellite_used,
					    gps.satellite_elevation,
					    gps.satellite_azimuth,
					    gps.satellite_snr);
	}

	l->listener->gps_counter++;
}

void
MavlinkOrbListener::l_vehicle_status(const struct listener *l)
{
	/* immediately communicate state _mavlink->get_chan()ges back to user */
	orb_copy(ORB_ID(vehicle_status), l->mavlink->get_subs()->status_sub, &(l->mavlink->v_status));
	orb_copy(ORB_ID(actuator_armed), l->mavlink->get_subs()->armed_sub, &l->listener->armed);
	orb_copy(ORB_ID(position_setpoint_triplet), l->mavlink->get_subs()->position_setpoint_triplet_sub, &(l->mavlink->pos_sp_triplet));

	/* enable or disable HIL */
	if (l->listener->v_status.hil_state == HIL_STATE_ON)
		l->mavlink->set_hil_enabled(true);
	else if (l->listener->v_status.hil_state == HIL_STATE_OFF)
		l->mavlink->set_hil_enabled(false);

	/* translate the current syste state to mavlink state and mode */
	uint8_t mavlink_state = 0;
	uint8_t mavlink_base_mode = 0;
	uint32_t mavlink_custom_mode = 0;
	l->mavlink->get_mavlink_mode_and_state(&mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

	/* send heartbeat */
	mavlink_msg_heartbeat_send(l->mavlink->get_chan(),
				   mavlink_system.type,
				   MAV_AUTOPILOT_PX4,
				   mavlink_base_mode,
				   mavlink_custom_mode,
				   mavlink_state);
}

void
MavlinkOrbListener::l_rc_channels(const struct listener *l)
{
	/* copy rc channels into local buffer */
	orb_copy(ORB_ID(rc_channels), l->mavlink->get_subs()->rc_sub, &l->listener->rc);
	// XXX Add RC channels scaled message here
}

void
MavlinkOrbListener::l_input_rc(const struct listener *l)
{
	/* copy rc _mavlink->get_chan()nels into local buffer */
	orb_copy(ORB_ID(input_rc), l->mavlink->get_subs()->input_rc_sub, &l->listener->rc_raw);

	if (l->mavlink->get_mode() == Mavlink::MODE_OFFBOARD) {

		const unsigned port_width = 8;

		for (unsigned i = 0; (i * port_width) < l->listener->rc_raw.channel_count; i++) {
			/* Channels are sent in MAVLink main loop at a fixed interval */
			mavlink_msg_rc_channels_raw_send(l->mavlink->get_chan(),
							 l->listener->rc_raw.timestamp_publication / 1000,
							 i,
							 (l->listener->rc_raw.channel_count > (i * port_width) + 0) ? l->listener->rc_raw.values[(i * port_width) + 0] : UINT16_MAX,
							 (l->listener->rc_raw.channel_count > (i * port_width) + 1) ? l->listener->rc_raw.values[(i * port_width) + 1] : UINT16_MAX,
							 (l->listener->rc_raw.channel_count > (i * port_width) + 2) ? l->listener->rc_raw.values[(i * port_width) + 2] : UINT16_MAX,
							 (l->listener->rc_raw.channel_count > (i * port_width) + 3) ? l->listener->rc_raw.values[(i * port_width) + 3] : UINT16_MAX,
							 (l->listener->rc_raw.channel_count > (i * port_width) + 4) ? l->listener->rc_raw.values[(i * port_width) + 4] : UINT16_MAX,
							 (l->listener->rc_raw.channel_count > (i * port_width) + 5) ? l->listener->rc_raw.values[(i * port_width) + 5] : UINT16_MAX,
							 (l->listener->rc_raw.channel_count > (i * port_width) + 6) ? l->listener->rc_raw.values[(i * port_width) + 6] : UINT16_MAX,
							 (l->listener->rc_raw.channel_count > (i * port_width) + 7) ? l->listener->rc_raw.values[(i * port_width) + 7] : UINT16_MAX,
							  l->listener->rc_raw.rssi);
		}
	}
}

void
MavlinkOrbListener::l_global_position(const struct listener *l)
{
	/* copy global position data into local buffer */
	orb_copy(ORB_ID(vehicle_global_position), l->mavlink->get_subs()->global_pos_sub, &l->listener->global_pos);

	mavlink_msg_global_position_int_send(l->mavlink->get_chan(),
						 l->listener->global_pos.timestamp / 1000,
					     l->listener->global_pos.lat * 1e7,
					     l->listener->global_pos.lon * 1e7,
					     l->listener->global_pos.alt * 1000.0f,
					     (l->listener->global_pos.alt - l->listener->home.alt) * 1000.0f,
					     l->listener->global_pos.vel_n * 100.0f,
					     l->listener->global_pos.vel_e * 100.0f,
					     l->listener->global_pos.vel_d * 100.0f,
					     _wrap_2pi(l->listener->global_pos.yaw) * M_RAD_TO_DEG_F * 100.0f);
}

void
MavlinkOrbListener::l_local_position(const struct listener *l)
{
	/* copy local position data into local buffer */
	orb_copy(ORB_ID(vehicle_local_position), l->mavlink->get_subs()->local_pos_sub, &l->listener->local_pos);

	if (l->mavlink->get_mode() == Mavlink::MODE_OFFBOARD)
		mavlink_msg_local_position_ned_send(l->mavlink->get_chan(),
						    l->listener->local_pos.timestamp / 1000,
						    l->listener->local_pos.x,
						    l->listener->local_pos.y,
						    l->listener->local_pos.z,
						    l->listener->local_pos.vx,
						    l->listener->local_pos.vy,
						    l->listener->local_pos.vz);
}

void
MavlinkOrbListener::l_global_position_setpoint(const struct listener *l)
{
	struct position_setpoint_triplet_s triplet;
	orb_copy(ORB_ID(position_setpoint_triplet), l->mavlink->get_subs()->triplet_sub, &triplet);

	if (!triplet.current.valid)
		return;

	if (l->mavlink->get_mode() == Mavlink::MODE_OFFBOARD)
		mavlink_msg_global_position_setpoint_int_send(l->mavlink->get_chan(),
				MAV_FRAME_GLOBAL,
				(int32_t)(triplet.current.lat * 1e7d),
				(int32_t)(triplet.current.lon * 1e7d),
				(int32_t)(triplet.current.alt * 1e3f),
				(int16_t)(triplet.current.yaw * M_RAD_TO_DEG_F * 1e2f));
}

void
MavlinkOrbListener::l_local_position_setpoint(const struct listener *l)
{
	struct vehicle_local_position_setpoint_s local_sp;

	/* copy local position data into local buffer */
	orb_copy(ORB_ID(vehicle_local_position_setpoint), l->mavlink->get_subs()->spl_sub, &local_sp);

	if (l->mavlink->get_mode() == Mavlink::MODE_OFFBOARD)
		mavlink_msg_local_position_setpoint_send(l->mavlink->get_chan(),
				MAV_FRAME_LOCAL_NED,
				local_sp.x,
				local_sp.y,
				local_sp.z,
				local_sp.yaw);
}

void
MavlinkOrbListener::l_attitude_setpoint(const struct listener *l)
{
	struct vehicle_attitude_setpoint_s att_sp;

	/* copy local position data into local buffer */
	orb_copy(ORB_ID(vehicle_attitude_setpoint), l->mavlink->get_subs()->spa_sub, &att_sp);

	if (l->mavlink->get_mode() == Mavlink::MODE_OFFBOARD)
		mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(l->mavlink->get_chan(),
				att_sp.timestamp / 1000,
				att_sp.roll_body,
				att_sp.pitch_body,
				att_sp.yaw_body,
				att_sp.thrust);
}

void
MavlinkOrbListener::l_vehicle_rates_setpoint(const struct listener *l)
{
	struct vehicle_rates_setpoint_s rates_sp;

	/* copy local position data into local buffer */
	orb_copy(ORB_ID(vehicle_rates_setpoint), l->mavlink->get_subs()->rates_setpoint_sub, &rates_sp);

	if (l->mavlink->get_mode() == Mavlink::MODE_OFFBOARD)
		mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_send(l->mavlink->get_chan(),
				rates_sp.timestamp / 1000,
				rates_sp.roll,
				rates_sp.pitch,
				rates_sp.yaw,
				rates_sp.thrust);
}

void
MavlinkOrbListener::l_actuator_outputs(const struct listener *l)
{
	struct actuator_outputs_s act_outputs;

	orb_id_t ids[] = {
		ORB_ID(actuator_outputs_0),
		ORB_ID(actuator_outputs_1),
		ORB_ID(actuator_outputs_2),
		ORB_ID(actuator_outputs_3)
	};

	/* copy actuator data into local buffer */
	orb_copy(ids[l->arg], *l->subp, &act_outputs);

	if (l->mavlink->get_mode() == Mavlink::MODE_OFFBOARD) {
		mavlink_msg_servo_output_raw_send(l->mavlink->get_chan(), l->listener->last_sensor_timestamp / 1000,
						  l->arg /* port number - needs GCS support */,
							 /* QGC has port number support already */
						  act_outputs.output[0],
						  act_outputs.output[1],
						  act_outputs.output[2],
						  act_outputs.output[3],
						  act_outputs.output[4],
						  act_outputs.output[5],
						  act_outputs.output[6],
						  act_outputs.output[7]);

		/* only send in HIL mode and only send first group for HIL */
		if (l->mavlink->get_hil_enabled() && l->listener->armed.armed && ids[l->arg] == ORB_ID(actuator_outputs_0)) {

			/* translate the current syste state to mavlink state and mode */
			uint8_t mavlink_state = 0;
			uint8_t mavlink_base_mode = 0;
			uint32_t mavlink_custom_mode = 0;
			l->mavlink->get_mavlink_mode_and_state(&mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

			/* HIL message as per MAVLink spec */

			/* scale / assign outputs depending on system type */

			if (mavlink_system.type == MAV_TYPE_QUADROTOR) {
				mavlink_msg_hil_controls_send(l->mavlink->get_chan(),
							      hrt_absolute_time(),
							      ((act_outputs.output[0] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[1] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[2] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[3] - 900.0f) / 600.0f) / 2.0f,
							      -1,
							      -1,
							      -1,
							      -1,
							      mavlink_base_mode,
							      0);

			} else if (mavlink_system.type == MAV_TYPE_HEXAROTOR) {
				mavlink_msg_hil_controls_send(l->mavlink->get_chan(),
							      hrt_absolute_time(),
							      ((act_outputs.output[0] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[1] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[2] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[3] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[4] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[5] - 900.0f) / 600.0f) / 2.0f,
							      -1,
							      -1,
							      mavlink_base_mode,
							      0);

			} else if (mavlink_system.type == MAV_TYPE_OCTOROTOR) {
				mavlink_msg_hil_controls_send(l->mavlink->get_chan(),
							      hrt_absolute_time(),
							      ((act_outputs.output[0] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[1] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[2] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[3] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[4] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[5] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[6] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[7] - 900.0f) / 600.0f) / 2.0f,
							      mavlink_base_mode,
							      0);

			} else {
				mavlink_msg_hil_controls_send(l->mavlink->get_chan(),
							      hrt_absolute_time(),
							      (act_outputs.output[0] - 1500.0f) / 500.0f,
							      (act_outputs.output[1] - 1500.0f) / 500.0f,
							      (act_outputs.output[2] - 1500.0f) / 500.0f,
							      (act_outputs.output[3] - 1000.0f) / 1000.0f,
							      (act_outputs.output[4] - 1500.0f) / 500.0f,
							      (act_outputs.output[5] - 1500.0f) / 500.0f,
							      (act_outputs.output[6] - 1500.0f) / 500.0f,
							      (act_outputs.output[7] - 1500.0f) / 500.0f,
							      mavlink_base_mode,
							      0);
			}
		}
	}
}

void
MavlinkOrbListener::l_actuator_armed(const struct listener *l)
{
	orb_copy(ORB_ID(actuator_armed), l->mavlink->get_subs()->armed_sub, &l->listener->armed);
}

void
MavlinkOrbListener::l_manual_control_setpoint(const struct listener *l)
{
	struct manual_control_setpoint_s man_control;

	/* copy manual control data into local buffer */
	orb_copy(ORB_ID(manual_control_setpoint), l->mavlink->get_subs()->man_control_sp_sub, &man_control);

	if (l->mavlink->get_mode() == Mavlink::MODE_OFFBOARD)
		mavlink_msg_manual_control_send(l->mavlink->get_chan(),
						mavlink_system.sysid,
						man_control.roll * 1000,
						man_control.pitch * 1000,
						man_control.yaw * 1000,
						man_control.throttle * 1000,
						0);
}

void
MavlinkOrbListener::l_vehicle_attitude_controls(const struct listener *l)
{
	orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, l->mavlink->get_subs()->actuators_sub, &l->listener->actuators_0);

	if (l->mavlink->get_mode() == Mavlink::MODE_OFFBOARD) {
		/* send, add spaces so that string buffer is at least 10 chars long */
		mavlink_msg_named_value_float_send(l->mavlink->get_chan(),
						   l->listener->last_sensor_timestamp / 1000,
						   "ctrl0    ",
						   l->listener->actuators_0.control[0]);
		mavlink_msg_named_value_float_send(l->mavlink->get_chan(),
						   l->listener->last_sensor_timestamp / 1000,
						   "ctrl1    ",
						   l->listener->actuators_0.control[1]);
		mavlink_msg_named_value_float_send(l->mavlink->get_chan(),
						   l->listener->last_sensor_timestamp / 1000,
						   "ctrl2     ",
						   l->listener->actuators_0.control[2]);
		mavlink_msg_named_value_float_send(l->mavlink->get_chan(),
						   l->listener->last_sensor_timestamp / 1000,
						   "ctrl3     ",
						   l->listener->actuators_0.control[3]);
	}
}

void
MavlinkOrbListener::l_debug_key_value(const struct listener *l)
{
	struct debug_key_value_s debug;

	orb_copy(ORB_ID(debug_key_value), l->mavlink->get_subs()->debug_key_value, &debug);

	/* Enforce null termination */
	debug.key[sizeof(debug.key) - 1] = '\0';

	mavlink_msg_named_value_float_send(l->mavlink->get_chan(),
					   l->listener->last_sensor_timestamp / 1000,
					   debug.key,
					   debug.value);
}

void
MavlinkOrbListener::l_optical_flow(const struct listener *l)
{
	struct optical_flow_s flow;

	orb_copy(ORB_ID(optical_flow), l->mavlink->get_subs()->optical_flow, &flow);

	mavlink_msg_optical_flow_send(l->mavlink->get_chan(), flow.timestamp, flow.sensor_id, flow.flow_raw_x, flow.flow_raw_y,
				      flow.flow_comp_x_m, flow.flow_comp_y_m, flow.quality, flow.ground_distance_m);
}

void
MavlinkOrbListener::l_home(const struct listener *l)
{
	orb_copy(ORB_ID(home_position), l->mavlink->get_subs()->home_sub, &l->listener->home);

	mavlink_msg_gps_global_origin_send(l->mavlink->get_chan(), (int32_t)(l->listener->home.lat*1e7d), (int32_t)(l->listener->home.lon*1e7d), (int32_t)(l->listener->home.alt)*1e3f);
}

void
MavlinkOrbListener::l_airspeed(const struct listener *l)
{
	orb_copy(ORB_ID(airspeed), l->mavlink->get_subs()->airspeed_sub, &l->listener->airspeed);
}

void
MavlinkOrbListener::l_nav_cap(const struct listener *l)
{

	orb_copy(ORB_ID(navigation_capabilities), l->mavlink->get_subs()->navigation_capabilities_sub, &l->listener->nav_cap);

	mavlink_msg_named_value_float_send(l->mavlink->get_chan(),
				   hrt_absolute_time() / 1000,
				   "turn dist",
				   l->listener->nav_cap.turn_distance);

}

void *
MavlinkOrbListener::uorb_receive_thread(void *arg)
{
	/* Set thread name */
	char thread_name[18];
	sprintf(thread_name, "mavlink_uorb_rcv_%d", _mavlink->get_channel());
	prctl(PR_SET_NAME, thread_name, getpid());

	/*
	 * set up poll to block for new data,
	 * wait for a maximum of 1000 ms (1 second)
	 */
	const int timeout = 1000;

	/*
	 * Initialise listener array.
	 *
	 * Might want to invoke each listener once to set initial state.
	 */
	struct pollfd fds[_max_listeners];

	struct listener* next = _listeners;
	unsigned i = 0;
	while (next != nullptr) {

		fds[i].fd = *next->subp;
		fds[i].events = POLLIN;

		next = next->next;
		i++;
	}
		/* Invoke callback to set initial state */
		//listeners[i].callback(&listener[i]);

	while (!_mavlink->_task_should_exit) {

		int poll_ret = poll(fds, _n_listeners, timeout);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* silent */

		} else if (poll_ret < 0) {
			//mavlink_missionlib_send_gcs_string("[mavlink] ERROR reading uORB data");

		} else {

			i = 0;
			struct listener* cb = _listeners;
			while (cb != nullptr) {

				if (fds[i].revents & POLLIN)
					cb->callback(cb);

				cb = cb->next;
				i++;
			}
		}
	}

	return NULL;
}

void * MavlinkOrbListener::uorb_start_helper(void *context)
{
	MavlinkOrbListener* urcv = new MavlinkOrbListener(((Mavlink *)context));
	return urcv->uorb_receive_thread(NULL);
}

pthread_t
MavlinkOrbListener::uorb_receive_start(Mavlink* mavlink)
{

	/* --- SENSORS RAW VALUE --- */
	mavlink->get_subs()->sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	/* rate limit set externally based on interface speed, set a basic default here */
	orb_set_interval(mavlink->get_subs()->sensor_sub, 200);	/* 5Hz updates */

	/* --- ATTITUDE VALUE --- */
	mavlink->get_subs()->att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	/* rate limit set externally based on interface speed, set a basic default here */
	orb_set_interval(mavlink->get_subs()->att_sub, 200);	/* 5Hz updates */

	/* --- GPS VALUE --- */
	mavlink->get_subs()->gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	orb_set_interval(mavlink->get_subs()->gps_sub, 200);	/* 5Hz updates */

	/* --- HOME POSITION --- */
	mavlink->get_subs()->home_sub = orb_subscribe(ORB_ID(home_position));
	orb_set_interval(mavlink->get_subs()->home_sub, 1000);	/* 1Hz updates */

	/* --- SYSTEM STATE --- */
	mavlink->get_subs()->status_sub = orb_subscribe(ORB_ID(vehicle_status));
	orb_set_interval(mavlink->get_subs()->status_sub, 300);		/* max 3.33 Hz updates */

	/* --- POSITION SETPOINT TRIPLET --- */
	mavlink->get_subs()->position_setpoint_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	orb_set_interval(mavlink->get_subs()->position_setpoint_triplet_sub, 0);		/* not polled, don't limit */

	/* --- RC CHANNELS VALUE --- */
	mavlink->get_subs()->rc_sub = orb_subscribe(ORB_ID(rc_channels));
	orb_set_interval(mavlink->get_subs()->rc_sub, 100);			/* 10Hz updates */

	/* --- RC RAW VALUE --- */
	mavlink->get_subs()->input_rc_sub = orb_subscribe(ORB_ID(input_rc));
	orb_set_interval(mavlink->get_subs()->input_rc_sub, 100);

	/* --- GLOBAL POS VALUE --- */
	mavlink->get_subs()->global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	orb_set_interval(mavlink->get_subs()->global_pos_sub, 100);	/* 10 Hz active updates */

	/* --- LOCAL POS VALUE --- */
	mavlink->get_subs()->local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	orb_set_interval(mavlink->get_subs()->local_pos_sub, 1000);	/* 1Hz active updates */

	/* --- GLOBAL SETPOINT VALUE --- */
	mavlink->get_subs()->triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	orb_set_interval(mavlink->get_subs()->triplet_sub, 2000);	/* 0.5 Hz updates */
	
	/* --- LOCAL SETPOINT VALUE --- */
	mavlink->get_subs()->spl_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	orb_set_interval(mavlink->get_subs()->spl_sub, 2000);	/* 0.5 Hz updates */

	/* --- ATTITUDE SETPOINT VALUE --- */
	mavlink->get_subs()->spa_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	orb_set_interval(mavlink->get_subs()->spa_sub, 2000);	/* 0.5 Hz updates */

	/* --- RATES SETPOINT VALUE --- */
	mavlink->get_subs()->rates_setpoint_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	orb_set_interval(mavlink->get_subs()->rates_setpoint_sub, 2000);  /* 0.5 Hz updates */

	/* --- ACTUATOR OUTPUTS --- */
	mavlink->get_subs()->act_0_sub = orb_subscribe(ORB_ID(actuator_outputs_0));
	mavlink->get_subs()->act_1_sub = orb_subscribe(ORB_ID(actuator_outputs_1));
	mavlink->get_subs()->act_2_sub = orb_subscribe(ORB_ID(actuator_outputs_2));
	mavlink->get_subs()->act_3_sub = orb_subscribe(ORB_ID(actuator_outputs_3));
	/* rate limits set externally based on interface speed, set a basic default here */
	orb_set_interval(mavlink->get_subs()->act_0_sub, 100);	/* 10Hz updates */
	orb_set_interval(mavlink->get_subs()->act_1_sub, 100);	/* 10Hz updates */
	orb_set_interval(mavlink->get_subs()->act_2_sub, 100);	/* 10Hz updates */
	orb_set_interval(mavlink->get_subs()->act_3_sub, 100);	/* 10Hz updates */

	/* --- ACTUATOR ARMED VALUE --- */
	mavlink->get_subs()->armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	orb_set_interval(mavlink->get_subs()->armed_sub, 100);	/* 10Hz updates */

	/* --- MAPPED MANUAL CONTROL INPUTS --- */
	mavlink->get_subs()->man_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	/* rate limits set externally based on interface speed, set a basic default here */
	orb_set_interval(mavlink->get_subs()->man_control_sp_sub, 100);	/* 10Hz updates */

	/* --- ACTUATOR CONTROL VALUE --- */
	mavlink->get_subs()->actuators_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	orb_set_interval(mavlink->get_subs()->actuators_sub, 100);	/* 10Hz updates */

	/* --- DEBUG VALUE OUTPUT --- */
	mavlink->get_subs()->debug_key_value = orb_subscribe(ORB_ID(debug_key_value));
	orb_set_interval(mavlink->get_subs()->debug_key_value, 100);	/* 10Hz updates */

	/* --- FLOW SENSOR --- */
	mavlink->get_subs()->optical_flow = orb_subscribe(ORB_ID(optical_flow));
	orb_set_interval(mavlink->get_subs()->optical_flow, 200); 	/* 5Hz updates */

	/* --- AIRSPEED --- */
	mavlink->get_subs()->airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	orb_set_interval(mavlink->get_subs()->airspeed_sub, 200); 	/* 5Hz updates */

	/* --- NAVIGATION CAPABILITIES --- */
	mavlink->get_subs()->navigation_capabilities_sub = orb_subscribe(ORB_ID(navigation_capabilities));
	orb_set_interval(mavlink->get_subs()->navigation_capabilities_sub, 500); 	/* 2Hz updates */

	/* start the listener loop */
	pthread_attr_t uorb_attr;
	pthread_attr_init(&uorb_attr);

	/* Set stack size, needs less than 2k */
	pthread_attr_setstacksize(&uorb_attr, 2048);

	pthread_t thread;
	pthread_create(&thread, &uorb_attr, MavlinkOrbListener::uorb_start_helper, (void*)mavlink);

	pthread_attr_destroy(&uorb_attr);
	return thread;
}
