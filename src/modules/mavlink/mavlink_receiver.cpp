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
 * @file mavlink_receiver.cpp
 * MAVLink protocol message receive and dispatch
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

/* XXX trim includes */
#include <nuttx/config.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <mathlib/mathlib.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/airspeed.h>
#include <mavlink/mavlink_log.h>
#include <commander/px4_custom_mode.h>
#include <geo/geo.h>

__BEGIN_DECLS

#include "mavlink_bridge_header.h"
#include "mavlink_receiver.h"
#include "mavlink_main.h"

__END_DECLS

static const float mg2ms2 = CONSTANTS_ONE_G / 1000.0f;

MavlinkReceiver::MavlinkReceiver(Mavlink *parent) :
	_mavlink(parent),
	status{},
	hil_local_pos{},
	_control_mode{},
	_global_pos_pub(-1),
	_local_pos_pub(-1),
	_attitude_pub(-1),
	_gps_pub(-1),
	_sensors_pub(-1),
	_gyro_pub(-1),
	_accel_pub(-1),
	_mag_pub(-1),
	_baro_pub(-1),
	_airspeed_pub(-1),
	_battery_pub(-1),
	_cmd_pub(-1),
	_flow_pub(-1),
	_offboard_control_sp_pub(-1),
	_local_pos_sp_pub(-1),
	_global_vel_sp_pub(-1),
	_att_sp_pub(-1),
	_rates_sp_pub(-1),
	_vicon_position_pub(-1),
	_telemetry_status_pub(-1),
	_rc_pub(-1),
	_manual_pub(-1),
	_radio_status_available(false),
	_control_mode_sub(orb_subscribe(ORB_ID(vehicle_control_mode))),
	_hil_frames(0),
	_old_timestamp(0),
	_hil_local_proj_inited(0),
	_hil_local_alt0(0.0f),
	_hil_local_proj_ref{}
{

	// make sure the FTP server is started
	(void)MavlinkFTP::getServer();
}

MavlinkReceiver::~MavlinkReceiver()
{
}

void
MavlinkReceiver::handle_message(mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_COMMAND_LONG:
		handle_message_command_long(msg);
		break;

	case MAVLINK_MSG_ID_OPTICAL_FLOW:
		handle_message_optical_flow(msg);
		break;

	case MAVLINK_MSG_ID_SET_MODE:
		handle_message_set_mode(msg);
		break;

	case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
		handle_message_vicon_position_estimate(msg);
		break;

	case MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST:
		handle_message_quad_swarm_roll_pitch_yaw_thrust(msg);
		break;

	case MAVLINK_MSG_ID_RADIO_STATUS:
		handle_message_radio_status(msg);
		break;

	case MAVLINK_MSG_ID_MANUAL_CONTROL:
		handle_message_manual_control(msg);
		break;

	case MAVLINK_MSG_ID_HEARTBEAT:
		handle_message_heartbeat(msg);
		break;

	case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
		handle_message_request_data_stream(msg);
		break;

	case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
		MavlinkFTP::getServer()->handle_message(_mavlink, msg);
		break;

	default:
		break;
	}

	/*
	 * Only decode hil messages in HIL mode.
	 *
	 * The HIL mode is enabled by the HIL bit flag
	 * in the system mode. Either send a set mode
	 * COMMAND_LONG message or a SET_MODE message
	 *
	 * Accept HIL GPS messages if use_hil_gps flag is true.
	 * This allows to provide fake gps measurements to the system.
	 */
	if (_mavlink->get_hil_enabled()) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_SENSOR:
			handle_message_hil_sensor(msg);
			break;

		case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
			handle_message_hil_state_quaternion(msg);
			break;

		default:
			break;
		}
	}


   	if (_mavlink->get_hil_enabled() || (_mavlink->get_use_hil_gps() && msg->sysid == mavlink_system.sysid)) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_GPS:
			handle_message_hil_gps(msg);
			break;

		default:
			break;
		}

	}

    /* If we've received a valid message, mark the flag indicating so.
	   This is used in the '-w' command-line flag. */
	_mavlink->set_has_received_messages(true);
}

void
MavlinkReceiver::handle_message_command_long(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_long_t cmd_mavlink;
	mavlink_msg_command_long_decode(msg, &cmd_mavlink);

	if (cmd_mavlink.target_system == mavlink_system.sysid && ((cmd_mavlink.target_component == mavlink_system.compid)
			|| (cmd_mavlink.target_component == MAV_COMP_ID_ALL))) {
		//check for MAVLINK terminate command
		if (cmd_mavlink.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && ((int)cmd_mavlink.param1) == 3) {
			/* This is the link shutdown command, terminate mavlink */
			warnx("terminated by remote command");
			fflush(stdout);
			usleep(50000);

			/* terminate other threads and this thread */
			_mavlink->_task_should_exit = true;

		} else {

			if (msg->sysid == mavlink_system.sysid && msg->compid == mavlink_system.compid) {
				warnx("ignoring CMD spoofed with same SYS/COMP (%d/%d) ID",
				      mavlink_system.sysid, mavlink_system.compid);
				return;
			}

			struct vehicle_command_s vcmd;
			memset(&vcmd, 0, sizeof(vcmd));

			/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
			vcmd.param1 = cmd_mavlink.param1;
			vcmd.param2 = cmd_mavlink.param2;
			vcmd.param3 = cmd_mavlink.param3;
			vcmd.param4 = cmd_mavlink.param4;
			vcmd.param5 = cmd_mavlink.param5;
			vcmd.param6 = cmd_mavlink.param6;
			vcmd.param7 = cmd_mavlink.param7;
			// XXX do proper translation
			vcmd.command = (enum VEHICLE_CMD)cmd_mavlink.command;
			vcmd.target_system = cmd_mavlink.target_system;
			vcmd.target_component = cmd_mavlink.target_component;
			vcmd.source_system = msg->sysid;
			vcmd.source_component = msg->compid;
			vcmd.confirmation =  cmd_mavlink.confirmation;

			if (_cmd_pub < 0) {
				_cmd_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);

			} else {
				orb_publish(ORB_ID(vehicle_command), _cmd_pub, &vcmd);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_optical_flow(mavlink_message_t *msg)
{
	/* optical flow */
	mavlink_optical_flow_t flow;
	mavlink_msg_optical_flow_decode(msg, &flow);

	struct optical_flow_s f;
	memset(&f, 0, sizeof(f));

	f.timestamp = hrt_absolute_time();
	f.flow_timestamp = flow.time_usec;
	f.flow_raw_x = flow.flow_x;
	f.flow_raw_y = flow.flow_y;
	f.flow_comp_x_m = flow.flow_comp_m_x;
	f.flow_comp_y_m = flow.flow_comp_m_y;
	f.ground_distance_m = flow.ground_distance;
	f.quality = flow.quality;
	f.sensor_id = flow.sensor_id;

	if (_flow_pub < 0) {
		_flow_pub = orb_advertise(ORB_ID(optical_flow), &f);

	} else {
		orb_publish(ORB_ID(optical_flow), _flow_pub, &f);
	}
}

void
MavlinkReceiver::handle_message_set_mode(mavlink_message_t *msg)
{
	mavlink_set_mode_t new_mode;
	mavlink_msg_set_mode_decode(msg, &new_mode);

	struct vehicle_command_s vcmd;
	memset(&vcmd, 0, sizeof(vcmd));

	union px4_custom_mode custom_mode;
	custom_mode.data = new_mode.custom_mode;
	/* copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
	vcmd.param1 = new_mode.base_mode;
	vcmd.param2 = custom_mode.main_mode;
	vcmd.param3 = 0;
	vcmd.param4 = 0;
	vcmd.param5 = 0;
	vcmd.param6 = 0;
	vcmd.param7 = 0;
	vcmd.command = VEHICLE_CMD_DO_SET_MODE;
	vcmd.target_system = new_mode.target_system;
	vcmd.target_component = MAV_COMP_ID_ALL;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = 1;

	if (_cmd_pub < 0) {
		_cmd_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);

	} else {
		orb_publish(ORB_ID(vehicle_command), _cmd_pub, &vcmd);
	}
}

void
MavlinkReceiver::handle_message_vicon_position_estimate(mavlink_message_t *msg)
{
	mavlink_vicon_position_estimate_t pos;
	mavlink_msg_vicon_position_estimate_decode(msg, &pos);

	struct vehicle_vicon_position_s vicon_position;
	memset(&vicon_position, 0, sizeof(vicon_position));

	vicon_position.timestamp = hrt_absolute_time();
	vicon_position.x = pos.x;
	vicon_position.y = pos.y;
	vicon_position.z = pos.z;
	vicon_position.roll = pos.roll;
	vicon_position.pitch = pos.pitch;
	vicon_position.yaw = pos.yaw;

	if (_vicon_position_pub < 0) {
		_vicon_position_pub = orb_advertise(ORB_ID(vehicle_vicon_position), &vicon_position);

	} else {
		orb_publish(ORB_ID(vehicle_vicon_position), _vicon_position_pub, &vicon_position);
	}
}

void
MavlinkReceiver::handle_message_quad_swarm_roll_pitch_yaw_thrust(mavlink_message_t *msg)
{
	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t swarm_offboard_control;
	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(msg, &swarm_offboard_control);

	/* Only accept system IDs from 1 to 4 */
	if (mavlink_system.sysid >= 1 && mavlink_system.sysid <= 4) {
		struct offboard_control_setpoint_s offboard_control_sp;
		memset(&offboard_control_sp, 0, sizeof(offboard_control_sp));

		/* Convert values * 1000 back */
		offboard_control_sp.p1 = (float)swarm_offboard_control.roll[mavlink_system.sysid - 1] / 1000.0f;
		offboard_control_sp.p2 = (float)swarm_offboard_control.pitch[mavlink_system.sysid - 1] / 1000.0f;
		offboard_control_sp.p3 = (float)swarm_offboard_control.yaw[mavlink_system.sysid - 1] / 1000.0f;
		offboard_control_sp.p4 = (float)swarm_offboard_control.thrust[mavlink_system.sysid - 1] / 1000.0f;

		offboard_control_sp.mode = (enum OFFBOARD_CONTROL_MODE)swarm_offboard_control.mode;

		offboard_control_sp.timestamp = hrt_absolute_time();

		if (_offboard_control_sp_pub < 0) {
			_offboard_control_sp_pub = orb_advertise(ORB_ID(offboard_control_setpoint), &offboard_control_sp);

		} else {
			orb_publish(ORB_ID(offboard_control_setpoint), _offboard_control_sp_pub, &offboard_control_sp);
		}
	}
}

void
MavlinkReceiver::handle_message_radio_status(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	if (_mavlink->get_channel() < TELEMETRY_STATUS_ORB_ID_NUM) {
		mavlink_radio_status_t rstatus;
		mavlink_msg_radio_status_decode(msg, &rstatus);

		struct telemetry_status_s &tstatus = _mavlink->get_rx_status();

		tstatus.timestamp = hrt_absolute_time();
		tstatus.telem_time = tstatus.timestamp;
		/* tstatus.heartbeat_time is set by system heartbeats */
		tstatus.type = TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO;
		tstatus.rssi = rstatus.rssi;
		tstatus.remote_rssi = rstatus.remrssi;
		tstatus.txbuf = rstatus.txbuf;
		tstatus.noise = rstatus.noise;
		tstatus.remote_noise = rstatus.remnoise;
		tstatus.rxerrors = rstatus.rxerrors;
		tstatus.fixed = rstatus.fixed;

		if (_telemetry_status_pub < 0) {
			_telemetry_status_pub = orb_advertise(telemetry_status_orb_id[_mavlink->get_channel()], &tstatus);

		} else {
			orb_publish(telemetry_status_orb_id[_mavlink->get_channel()], _telemetry_status_pub, &tstatus);
		}

		/* this means that heartbeats alone won't be published to the radio status no more */
		_radio_status_available = true;
	}
}

void
MavlinkReceiver::handle_message_manual_control(mavlink_message_t *msg)
{
	mavlink_manual_control_t man;
	mavlink_msg_manual_control_decode(msg, &man);

	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));

	manual.timestamp = hrt_absolute_time();
	manual.x = man.x / 1000.0f;
	manual.y = man.y / 1000.0f;
	manual.r = man.r / 1000.0f;
	manual.z = man.z / 1000.0f;

	warnx("pitch: %.2f, roll: %.2f, yaw: %.2f, throttle: %.2f", (double)manual.x, (double)manual.y, (double)manual.r, (double)manual.z);

	if (_manual_pub < 0) {
		_manual_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual);

	} else {
		orb_publish(ORB_ID(manual_control_setpoint), _manual_pub, &manual);
	}
}

void
MavlinkReceiver::handle_message_heartbeat(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	if (_mavlink->get_channel() < TELEMETRY_STATUS_ORB_ID_NUM) {
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);

		/* ignore own heartbeats, accept only heartbeats from GCS */
		if (msg->sysid != mavlink_system.sysid && hb.type == MAV_TYPE_GCS) {

			struct telemetry_status_s &tstatus = _mavlink->get_rx_status();

			hrt_abstime tnow = hrt_absolute_time();

			/* always set heartbeat, publish only if telemetry link not up */
			tstatus.heartbeat_time = tnow;

			/* if no radio status messages arrive, lets at least publish that heartbeats were received */
			if (!_radio_status_available) {

				tstatus.timestamp = tnow;
				/* telem_time indicates the timestamp of a telemetry status packet and we got none */
				tstatus.telem_time = 0;
				tstatus.type = TELEMETRY_STATUS_RADIO_TYPE_GENERIC;

				if (_telemetry_status_pub < 0) {
					_telemetry_status_pub = orb_advertise(telemetry_status_orb_id[_mavlink->get_channel()], &tstatus);

				} else {
					orb_publish(telemetry_status_orb_id[_mavlink->get_channel()], _telemetry_status_pub, &tstatus);
				}
			}
		}
	}
}

void
MavlinkReceiver::handle_message_request_data_stream(mavlink_message_t *msg)
{
	mavlink_request_data_stream_t req;
	mavlink_msg_request_data_stream_decode(msg, &req);

	if (req.target_system == mavlink_system.sysid && req.target_component == mavlink_system.compid) {
		float rate = req.start_stop ? (1000.0f / req.req_message_rate) : 0.0f;

		MavlinkStream *stream;
		LL_FOREACH(_mavlink->get_streams(), stream) {
			if (req.req_stream_id == stream->get_id()) {
				_mavlink->configure_stream_threadsafe(stream->get_name(), rate);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_hil_sensor(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t imu;
	mavlink_msg_hil_sensor_decode(msg, &imu);

	uint64_t timestamp = hrt_absolute_time();

	/* airspeed */
	{
		struct airspeed_s airspeed;
		memset(&airspeed, 0, sizeof(airspeed));

		float ias = calc_indicated_airspeed(imu.diff_pressure * 1e2f);
		// XXX need to fix this
		float tas = ias;

		airspeed.timestamp = timestamp;
		airspeed.indicated_airspeed_m_s = ias;
		airspeed.true_airspeed_m_s = tas;

		if (_airspeed_pub < 0) {
			_airspeed_pub = orb_advertise(ORB_ID(airspeed), &airspeed);

		} else {
			orb_publish(ORB_ID(airspeed), _airspeed_pub, &airspeed);
		}
	}

	/* gyro */
	{
		struct gyro_report gyro;
		memset(&gyro, 0, sizeof(gyro));

		gyro.timestamp = timestamp;
		gyro.x_raw = imu.xgyro * 1000.0f;
		gyro.y_raw = imu.ygyro * 1000.0f;
		gyro.z_raw = imu.zgyro * 1000.0f;
		gyro.x = imu.xgyro;
		gyro.y = imu.ygyro;
		gyro.z = imu.zgyro;
		gyro.temperature = imu.temperature;

		if (_gyro_pub < 0) {
			_gyro_pub = orb_advertise(ORB_ID(sensor_gyro0), &gyro);

		} else {
			orb_publish(ORB_ID(sensor_gyro0), _gyro_pub, &gyro);
		}
	}

	/* accelerometer */
	{
		struct accel_report accel;
		memset(&accel, 0, sizeof(accel));

		accel.timestamp = timestamp;
		accel.x_raw = imu.xacc / mg2ms2;
		accel.y_raw = imu.yacc / mg2ms2;
		accel.z_raw = imu.zacc / mg2ms2;
		accel.x = imu.xacc;
		accel.y = imu.yacc;
		accel.z = imu.zacc;
		accel.temperature = imu.temperature;

		if (_accel_pub < 0) {
			_accel_pub = orb_advertise(ORB_ID(sensor_accel0), &accel);

		} else {
			orb_publish(ORB_ID(sensor_accel0), _accel_pub, &accel);
		}
	}

	/* magnetometer */
	{
		struct mag_report mag;
		memset(&mag, 0, sizeof(mag));

		mag.timestamp = timestamp;
		mag.x_raw = imu.xmag * 1000.0f;
		mag.y_raw = imu.ymag * 1000.0f;
		mag.z_raw = imu.zmag * 1000.0f;
		mag.x = imu.xmag;
		mag.y = imu.ymag;
		mag.z = imu.zmag;

		if (_mag_pub < 0) {
			_mag_pub = orb_advertise(ORB_ID(sensor_mag0), &mag);

		} else {
			orb_publish(ORB_ID(sensor_mag0), _mag_pub, &mag);
		}
	}

	/* baro */
	{
		struct baro_report baro;
		memset(&baro, 0, sizeof(baro));

		baro.timestamp = timestamp;
		baro.pressure = imu.abs_pressure;
		baro.altitude = imu.pressure_alt;
		baro.temperature = imu.temperature;

		if (_baro_pub < 0) {
			_baro_pub = orb_advertise(ORB_ID(sensor_baro0), &baro);

		} else {
			orb_publish(ORB_ID(sensor_baro0), _baro_pub, &baro);
		}
	}

	/* sensor combined */
	{
		struct sensor_combined_s hil_sensors;
		memset(&hil_sensors, 0, sizeof(hil_sensors));

		hil_sensors.timestamp = timestamp;

		hil_sensors.gyro_raw[0] = imu.xgyro * 1000.0f;
		hil_sensors.gyro_raw[1] = imu.ygyro * 1000.0f;
		hil_sensors.gyro_raw[2] = imu.zgyro * 1000.0f;
		hil_sensors.gyro_rad_s[0] = imu.xgyro;
		hil_sensors.gyro_rad_s[1] = imu.ygyro;
		hil_sensors.gyro_rad_s[2] = imu.zgyro;

		hil_sensors.accelerometer_raw[0] = imu.xacc / mg2ms2;
		hil_sensors.accelerometer_raw[1] = imu.yacc / mg2ms2;
		hil_sensors.accelerometer_raw[2] = imu.zacc / mg2ms2;
		hil_sensors.accelerometer_m_s2[0] = imu.xacc;
		hil_sensors.accelerometer_m_s2[1] = imu.yacc;
		hil_sensors.accelerometer_m_s2[2] = imu.zacc;
		hil_sensors.accelerometer_mode = 0; // TODO what is this?
		hil_sensors.accelerometer_range_m_s2 = 32.7f; // int16
		hil_sensors.accelerometer_timestamp = timestamp;

		hil_sensors.adc_voltage_v[0] = 0.0f;
		hil_sensors.adc_voltage_v[1] = 0.0f;
		hil_sensors.adc_voltage_v[2] = 0.0f;

		hil_sensors.magnetometer_raw[0] = imu.xmag * 1000.0f;
		hil_sensors.magnetometer_raw[1] = imu.ymag * 1000.0f;
		hil_sensors.magnetometer_raw[2] = imu.zmag * 1000.0f;
		hil_sensors.magnetometer_ga[0] = imu.xmag;
		hil_sensors.magnetometer_ga[1] = imu.ymag;
		hil_sensors.magnetometer_ga[2] = imu.zmag;
		hil_sensors.magnetometer_range_ga = 32.7f; // int16
		hil_sensors.magnetometer_mode = 0; // TODO what is this
		hil_sensors.magnetometer_cuttoff_freq_hz = 50.0f;
		hil_sensors.magnetometer_timestamp = timestamp;

		hil_sensors.baro_pres_mbar = imu.abs_pressure;
		hil_sensors.baro_alt_meter = imu.pressure_alt;
		hil_sensors.baro_temp_celcius = imu.temperature;
		hil_sensors.baro_timestamp = timestamp;

		hil_sensors.differential_pressure_pa = imu.diff_pressure * 1e2f; //from hPa to Pa
		hil_sensors.differential_pressure_timestamp = timestamp;

		/* publish combined sensor topic */
		if (_sensors_pub < 0) {
			_sensors_pub = orb_advertise(ORB_ID(sensor_combined), &hil_sensors);

		} else {
			orb_publish(ORB_ID(sensor_combined), _sensors_pub, &hil_sensors);
		}
	}

	/* battery status */
	{
		struct battery_status_s hil_battery_status;
		memset(&hil_battery_status, 0, sizeof(hil_battery_status));

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 11.1f;
		hil_battery_status.voltage_filtered_v = 11.1f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;

		if (_battery_pub < 0) {
			_battery_pub = orb_advertise(ORB_ID(battery_status), &hil_battery_status);

		} else {
			orb_publish(ORB_ID(battery_status), _battery_pub, &hil_battery_status);
		}
	}

	/* increment counters */
	_hil_frames++;

	/* print HIL sensors rate */
	if ((timestamp - _old_timestamp) > 10000000) {
		printf("receiving HIL sensors at %d hz\n", _hil_frames / 10);
		_old_timestamp = timestamp;
		_hil_frames = 0;
	}
}

void
MavlinkReceiver::handle_message_hil_gps(mavlink_message_t *msg)
{
	mavlink_hil_gps_t gps;
	mavlink_msg_hil_gps_decode(msg, &gps);

	uint64_t timestamp = hrt_absolute_time();

	struct vehicle_gps_position_s hil_gps;
	memset(&hil_gps, 0, sizeof(hil_gps));

	hil_gps.timestamp_time = timestamp;
	hil_gps.time_gps_usec = gps.time_usec;

	hil_gps.timestamp_position = timestamp;
	hil_gps.lat = gps.lat;
	hil_gps.lon = gps.lon;
	hil_gps.alt = gps.alt;
	hil_gps.eph = (float)gps.eph * 1e-2f; // from cm to m
	hil_gps.epv = (float)gps.epv * 1e-2f; // from cm to m

	hil_gps.timestamp_variance = timestamp;
	hil_gps.s_variance_m_s = 5.0f;

	hil_gps.timestamp_velocity = timestamp;
	hil_gps.vel_m_s = (float)gps.vel * 1e-2f; // from cm/s to m/s
	hil_gps.vel_n_m_s = gps.vn * 1e-2f; // from cm to m
	hil_gps.vel_e_m_s = gps.ve * 1e-2f; // from cm to m
	hil_gps.vel_d_m_s = gps.vd * 1e-2f; // from cm to m
	hil_gps.vel_ned_valid = true;
	hil_gps.cog_rad = _wrap_pi(gps.cog * M_DEG_TO_RAD_F * 1e-2f);

	hil_gps.fix_type = gps.fix_type;
	hil_gps.satellites_used = gps.satellites_visible;  //TODO: rename mavlink_hil_gps_t sats visible to used?

	if (_gps_pub < 0) {
		_gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &hil_gps);

	} else {
		orb_publish(ORB_ID(vehicle_gps_position), _gps_pub, &hil_gps);
	}
}

void
MavlinkReceiver::handle_message_hil_state_quaternion(mavlink_message_t *msg)
{
	mavlink_hil_state_quaternion_t hil_state;
	mavlink_msg_hil_state_quaternion_decode(msg, &hil_state);

	uint64_t timestamp = hrt_absolute_time();

	/* airspeed */
	{
		struct airspeed_s airspeed;
		memset(&airspeed, 0, sizeof(airspeed));

		airspeed.timestamp = timestamp;
		airspeed.indicated_airspeed_m_s = hil_state.ind_airspeed * 1e-2f;
		airspeed.true_airspeed_m_s = hil_state.true_airspeed * 1e-2f;

		if (_airspeed_pub < 0) {
			_airspeed_pub = orb_advertise(ORB_ID(airspeed), &airspeed);

		} else {
			orb_publish(ORB_ID(airspeed), _airspeed_pub, &airspeed);
		}
	}

	/* attitude */
	struct vehicle_attitude_s hil_attitude;
	{
		memset(&hil_attitude, 0, sizeof(hil_attitude));
		math::Quaternion q(hil_state.attitude_quaternion);
		math::Matrix<3, 3> C_nb = q.to_dcm();
		math::Vector<3> euler = C_nb.to_euler();

		hil_attitude.timestamp = timestamp;
		memcpy(hil_attitude.R, C_nb.data, sizeof(hil_attitude.R));
		hil_attitude.R_valid = true;

		hil_attitude.q[0] = q(0);
		hil_attitude.q[1] = q(1);
		hil_attitude.q[2] = q(2);
		hil_attitude.q[3] = q(3);
		hil_attitude.q_valid = true;

		hil_attitude.roll = euler(0);
		hil_attitude.pitch = euler(1);
		hil_attitude.yaw = euler(2);
		hil_attitude.rollspeed = hil_state.rollspeed;
		hil_attitude.pitchspeed = hil_state.pitchspeed;
		hil_attitude.yawspeed = hil_state.yawspeed;

		if (_attitude_pub < 0) {
			_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &hil_attitude);

		} else {
			orb_publish(ORB_ID(vehicle_attitude), _attitude_pub, &hil_attitude);
		}
	}

	/* global position */
	{
		struct vehicle_global_position_s hil_global_pos;
		memset(&hil_global_pos, 0, sizeof(hil_global_pos));

		hil_global_pos.timestamp = timestamp;
		hil_global_pos.lat = hil_state.lat;
		hil_global_pos.lon = hil_state.lon;
		hil_global_pos.alt = hil_state.alt / 1000.0f;
		hil_global_pos.vel_n = hil_state.vx / 100.0f;
		hil_global_pos.vel_e = hil_state.vy / 100.0f;
		hil_global_pos.vel_d = hil_state.vz / 100.0f;
		hil_global_pos.yaw = hil_attitude.yaw;
		hil_global_pos.eph = 2.0f;
		hil_global_pos.epv = 4.0f;

		if (_global_pos_pub < 0) {
			_global_pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &hil_global_pos);

		} else {
			orb_publish(ORB_ID(vehicle_global_position), _global_pos_pub, &hil_global_pos);
		}
	}

	/* local position */
	{
		double lat = hil_state.lat * 1e-7;
		double lon = hil_state.lon * 1e-7;

		if (!_hil_local_proj_inited) {
			_hil_local_proj_inited = true;
			_hil_local_alt0 = hil_state.alt / 1000.0f;
			map_projection_init(&_hil_local_proj_ref, hil_state.lat, hil_state.lon);
			hil_local_pos.ref_timestamp = timestamp;
			hil_local_pos.ref_lat = lat;
			hil_local_pos.ref_lon = lon;
			hil_local_pos.ref_alt = _hil_local_alt0;
		}

		float x;
		float y;
		map_projection_project(&_hil_local_proj_ref, lat, lon, &x, &y);
		hil_local_pos.timestamp = timestamp;
		hil_local_pos.xy_valid = true;
		hil_local_pos.z_valid = true;
		hil_local_pos.v_xy_valid = true;
		hil_local_pos.v_z_valid = true;
		hil_local_pos.x = x;
		hil_local_pos.y = y;
		hil_local_pos.z = _hil_local_alt0 - hil_state.alt / 1000.0f;
		hil_local_pos.vx = hil_state.vx / 100.0f;
		hil_local_pos.vy = hil_state.vy / 100.0f;
		hil_local_pos.vz = hil_state.vz / 100.0f;
		hil_local_pos.yaw = hil_attitude.yaw;
		hil_local_pos.xy_global = true;
		hil_local_pos.z_global = true;

		bool landed = (float)(hil_state.alt) / 1000.0f < (_hil_local_alt0 + 0.1f); // XXX improve?
		hil_local_pos.landed = landed;

		if (_local_pos_pub < 0) {
			_local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &hil_local_pos);

		} else {
			orb_publish(ORB_ID(vehicle_local_position), _local_pos_pub, &hil_local_pos);
		}
	}

	/* accelerometer */
	{
		struct accel_report accel;
		memset(&accel, 0, sizeof(accel));

		accel.timestamp = timestamp;
		accel.x_raw = hil_state.xacc / CONSTANTS_ONE_G * 1e3f;
		accel.y_raw = hil_state.yacc / CONSTANTS_ONE_G * 1e3f;
		accel.z_raw = hil_state.zacc / CONSTANTS_ONE_G * 1e3f;
		accel.x = hil_state.xacc;
		accel.y = hil_state.yacc;
		accel.z = hil_state.zacc;
		accel.temperature = 25.0f;

		if (_accel_pub < 0) {
			_accel_pub = orb_advertise(ORB_ID(sensor_accel0), &accel);

		} else {
			orb_publish(ORB_ID(sensor_accel0), _accel_pub, &accel);
		}
	}

	/* battery status */
	{
		struct battery_status_s	hil_battery_status;
		memset(&hil_battery_status, 0, sizeof(hil_battery_status));

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 11.1f;
		hil_battery_status.voltage_filtered_v = 11.1f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;

		if (_battery_pub < 0) {
			_battery_pub = orb_advertise(ORB_ID(battery_status), &hil_battery_status);

		} else {
			orb_publish(ORB_ID(battery_status), _battery_pub, &hil_battery_status);
		}
	}
}


/**
 * Receive data from UART.
 */
void *
MavlinkReceiver::receive_thread(void *arg)
{
	int uart_fd = _mavlink->get_uart_fd();

	const int timeout = 500;
	uint8_t buf[32];

	mavlink_message_t msg;

	/* set thread name */
	char thread_name[24];
	sprintf(thread_name, "mavlink_rcv_if%d", _mavlink->get_instance_id());
	prctl(PR_SET_NAME, thread_name, getpid());

	struct pollfd fds[1];
	fds[0].fd = uart_fd;
	fds[0].events = POLLIN;

	ssize_t nread = 0;

	while (!_mavlink->_task_should_exit) {
		if (poll(fds, 1, timeout) > 0) {

			/* non-blocking read. read may return negative values */
			if ((nread = read(uart_fd, buf, sizeof(buf))) < (ssize_t)sizeof(buf)) {
				/* to avoid reading very small chunks wait for data before reading */
				usleep(1000);
			}

			/* if read failed, this loop won't execute */
			for (ssize_t i = 0; i < nread; i++) {
				if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &status)) {
					/* handle generic messages and commands */
					handle_message(&msg);

					/* handle packet with parent object */
					_mavlink->handle_message(&msg);
				}
			}

			/* count received bytes */
			_mavlink->count_rxbytes(nread);
		}
	}

	return NULL;
}

void MavlinkReceiver::print_status()
{

}

void *MavlinkReceiver::start_helper(void *context)
{
	MavlinkReceiver *rcv = new MavlinkReceiver((Mavlink *)context);

	rcv->receive_thread(NULL);

	delete rcv;

	return nullptr;
}

pthread_t
MavlinkReceiver::receive_start(Mavlink *parent)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	// set to non-blocking read
	int flags = fcntl(parent->get_uart_fd(), F_GETFL, 0);
	fcntl(parent->get_uart_fd(), F_SETFL, flags | O_NONBLOCK);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&receiveloop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_MAX - 40;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, 2900);
	pthread_t thread;
	pthread_create(&thread, &receiveloop_attr, MavlinkReceiver::start_helper, (void *)parent);

	pthread_attr_destroy(&receiveloop_attr);
	return thread;
}
