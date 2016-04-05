/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file ekf2_replay_main.cpp
 * Replay module for ekf2. This module reads ekf2 replay messages from a px4 logfile.
 * It uses this data to create sensor data for the ekf2 module. It also subscribes to the
 * output data of the estimator and writes it to a replay log file.
 *
 * @author Roman Bapst
*/

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include <uORB/topics/ekf2_replay.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>

#include <sdlog2/sdlog2_messages.h>


extern "C" __EXPORT int ekf2_replay_main(int argc, char *argv[]);

#define PRINT_READ_ERROR 	PX4_WARN("error reading from log file");

// union for log messages to write to log file
#pragma pack(push, 1)
struct {
	uint8_t head1, head2, type;
	union {
		struct log_ATT_s att;
		struct log_LPOS_s lpos;
		struct log_CTS_s control_state;
		struct log_EST0_s est0;
		struct log_EST1_s est1;
		struct log_EST2_s est2;
		struct log_EST3_s est3;
		struct log_EST4_s innov;
		struct log_EST5_s innov2;
		struct log_EST6_s innov3;
	} body;
} log_message;

#pragma pack(pop)

class Ekf2Replay;


namespace ekf2_replay
{
Ekf2Replay *instance = nullptr;
}

class Ekf2Replay
{
public:
	// Constructor
	Ekf2Replay(char *logfile);

	// Destructor, also kills task
	~Ekf2Replay();

	// Start task.
	// @return		OK on success.
	int		start();

	void exit() { _task_should_exit = true; }

	static void	task_main_trampoline(int argc, char *argv[]);

	void		task_main();

private:
	int		_control_task = -1;			//task handle for task
	bool	_task_should_exit = false;

	orb_advert_t _sensors_pub;
	orb_advert_t _gps_pub;
	orb_advert_t _status_pub;
	orb_advert_t _flow_pub;
	orb_advert_t _range_pub;

	int _att_sub;
	int _estimator_status_sub;
	int _innov_sub;
	int _lpos_sub;
	int _control_state_sub;

	char *_file_name;

	struct log_format_s _formats[100];
	struct sensor_combined_s _sensors;
	struct vehicle_gps_position_s _gps;
	struct vehicle_status_s _status;
	struct optical_flow_s _flow;
	struct distance_sensor_s _range;

	unsigned _message_counter; // counter which will increase with every message read from the log
	unsigned _part1_counter_ref;		// this is the value of _message_counter when the part1 of the replay message is read (imu data)
	bool _read_part2;				// indicates if part 2 of replay message has been read
	bool _read_part3;
	bool _read_part4;

	int _write_fd = -1;
	px4_pollfd_struct_t _fds[1];

	// parse replay message from buffer
	// @source 			pointer to log message data (excluding header)
	// @destination 	pointer to message struct of type @type
	// @type 			message type
	void parseMessage(uint8_t *source, uint8_t *destination, uint8_t type);

	// copy the replay data from the logs into the topic structs which
	// will be puplished after
	// @data 	pointer to the message struct of type @type
	// @type 	message type
	void setEstimatorInput(uint8_t *data, uint8_t type);

	// publish input data for estimator
	void publishEstimatorInput();

	// write a message to log file
	// @fd 		file descriptor
	// @data 	pointer to log message
	// @data 	size of data to be written
	void writeMessage(int &fd, void *data, size_t size);

	// determins if we need so write a specific message to the replay log
	// messages which are not regenerated by the estimator copied from the original log file
	// @type 	message type
	bool needToSaveMessage(uint8_t type);

	// get estimator output messages and write them to replay log
	void logIfUpdated();

	// this will call the method to publish the input data for the estimator
	// it will then wait for the output data from the estimator and call the propoper
	// functions to handle it
	void publishAndWaitForEstimator();

	void setUserParams(const char *filename);
};

Ekf2Replay::Ekf2Replay(char *logfile) :
	_sensors_pub(nullptr),
	_gps_pub(nullptr),
	_status_pub(nullptr),
	_flow_pub(nullptr),
	_range_pub(nullptr),
	_att_sub(-1),
	_estimator_status_sub(-1),
	_innov_sub(-1),
	_lpos_sub(-1),
	_control_state_sub(-1),
	_formats{},
	_sensors{},
	_gps{},
	_status{},
	_flow{},
	_range{},
	_message_counter(0),
	_part1_counter_ref(0),
	_read_part2(false),
	_read_part3(false),
	_read_part4(false),
	_write_fd(-1)
{
	// build the path to the log
	char tmp[] = "./rootfs/";
	char *path_to_log = (char *) malloc(1 + strlen(tmp) + strlen(logfile));
	strcpy(path_to_log, tmp);
	strcat(path_to_log, logfile);
	_file_name = path_to_log;

	// we always start landed
	_status.condition_landed = true;
}

Ekf2Replay::~Ekf2Replay()
{

}

void Ekf2Replay::publishEstimatorInput()
{
	if (_gps_pub == nullptr && _read_part2) {
		_gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_gps);

	} else if (_gps_pub != nullptr && _read_part2) {
		orb_publish(ORB_ID(vehicle_gps_position), _gps_pub, &_gps);
	}

	_read_part2 = false;

	if (_flow_pub == nullptr && _read_part3) {
		_flow_pub = orb_advertise(ORB_ID(optical_flow), &_flow);

	} else if (_flow_pub != nullptr && _read_part3) {
		orb_publish(ORB_ID(optical_flow), _flow_pub, &_flow);
	}

	_read_part3 = false;

	if (_range_pub == nullptr && _read_part4) {
		_range_pub = orb_advertise(ORB_ID(distance_sensor), &_range);

	} else if (_range_pub != nullptr && _read_part4) {
		orb_publish(ORB_ID(distance_sensor), _range_pub, &_range);
	}

	_read_part4 = false;

	if (_sensors_pub == nullptr) {
		_sensors_pub = orb_advertise(ORB_ID(sensor_combined), &_sensors);

	} else if (_sensors_pub != nullptr) {
		orb_publish(ORB_ID(sensor_combined), _sensors_pub, &_sensors);
	}
}

void Ekf2Replay::parseMessage(uint8_t *source, uint8_t *destination, uint8_t type)
{
	int i = 0;
	int write_index = 0;

	while (_formats[type].format[i] != '\0') {
		char data_type = _formats[type].format[i];

		switch (data_type) {
		case 'f':
			memcpy(&destination[write_index], &source[write_index], sizeof(float));
			write_index += sizeof(float);
			break;

		case 'Q':
			memcpy(&destination[write_index], &source[write_index], sizeof(uint64_t));
			write_index += sizeof(uint64_t);
			break;

		case 'L':
			memcpy(&destination[write_index], &source[write_index], sizeof(int32_t));
			write_index += sizeof(int32_t);
			break;

		case 'M':
			memcpy(&destination[write_index], &source[write_index], sizeof(uint8_t));
			write_index += sizeof(uint8_t);
			break;

		case 'B':
			memcpy(&destination[write_index], &source[write_index], sizeof(uint8_t));
			write_index += sizeof(uint8_t);
			break;

		case 'I':
			memcpy(&destination[write_index], &source[write_index], sizeof(uint32_t));
			write_index += sizeof(uint32_t);
			break;

		case 'i':
			memcpy(&destination[write_index], &source[write_index], sizeof(int32_t));
			write_index += sizeof(int32_t);
			break;

		default:
			PX4_WARN("found unsupported data type in replay message, exiting!");
			_task_should_exit = true;
			break;
		}

		i++;
	}
}

void Ekf2Replay::setEstimatorInput(uint8_t *data, uint8_t type)
{
	struct log_RPL1_s replay_part1 = {};
	struct log_RPL2_s replay_part2 = {};
	struct log_RPL3_s replay_part3 = {};
	struct log_RPL4_s replay_part4 = {};
	struct log_STAT_s vehicle_status = {};

	if (type == LOG_RPL1_MSG) {

		uint8_t *dest_ptr = (uint8_t *)&replay_part1.time_ref;
		parseMessage(data, dest_ptr, type);
		_sensors.timestamp = replay_part1.time_ref;
		_sensors.gyro_integral_dt[0] = replay_part1.gyro_integral_dt;
		_sensors.accelerometer_integral_dt[0] = replay_part1.accelerometer_integral_dt;
		_sensors.magnetometer_timestamp[0] = replay_part1.magnetometer_timestamp;
		_sensors.baro_timestamp[0] = replay_part1.baro_timestamp;
		_sensors.gyro_integral_rad[0] = replay_part1.gyro_integral_x_rad;
		_sensors.gyro_integral_rad[1] = replay_part1.gyro_integral_y_rad;
		_sensors.gyro_integral_rad[2] = replay_part1.gyro_integral_z_rad;
		_sensors.accelerometer_integral_m_s[0] = replay_part1.accelerometer_integral_x_m_s;
		_sensors.accelerometer_integral_m_s[1] = replay_part1.accelerometer_integral_y_m_s;
		_sensors.accelerometer_integral_m_s[2] = replay_part1.accelerometer_integral_z_m_s;
		_sensors.magnetometer_ga[0] = replay_part1.magnetometer_x_ga;
		_sensors.magnetometer_ga[1] = replay_part1.magnetometer_y_ga;
		_sensors.magnetometer_ga[2] = replay_part1.magnetometer_z_ga;
		_sensors.baro_alt_meter[0] = replay_part1.baro_alt_meter;
		_part1_counter_ref = _message_counter;

	} else if (type == LOG_RPL2_MSG) {
		uint8_t *dest_ptr = (uint8_t *)&replay_part2.time_pos_usec;
		parseMessage(data, dest_ptr, type);
		_gps.timestamp_position = replay_part2.time_pos_usec;
		_gps.timestamp_velocity = replay_part2.time_vel_usec;
		_gps.lat = replay_part2.lat;
		_gps.lon = replay_part2.lon;
		_gps.fix_type = replay_part2.fix_type;
		_gps.satellites_used = replay_part2.nsats;
		_gps.eph = replay_part2.eph;
		_gps.epv = replay_part2.epv;
		_gps.s_variance_m_s = replay_part2.sacc;
		_gps.vel_m_s = replay_part2.vel_m_s;
		_gps.vel_n_m_s = replay_part2.vel_n_m_s;
		_gps.vel_e_m_s = replay_part2.vel_e_m_s;
		_gps.vel_d_m_s = replay_part2.vel_d_m_s;
		_gps.vel_ned_valid = replay_part2.vel_ned_valid;
		_read_part2 = true;

	} else if (type == LOG_RPL3_MSG) {
		uint8_t *dest_ptr = (uint8_t *)&replay_part3.time_flow_usec;
		parseMessage(data, dest_ptr, type);
		_flow.timestamp = replay_part3.time_flow_usec;
		_flow.pixel_flow_x_integral = replay_part3.flow_integral_x;
		_flow.pixel_flow_y_integral = replay_part3.flow_integral_y;
		_flow.gyro_x_rate_integral = replay_part3.gyro_integral_x;
		_flow.gyro_y_rate_integral = replay_part3.gyro_integral_y;
		_flow.integration_timespan = replay_part3.flow_time_integral;
		_flow.quality = replay_part3.flow_quality;
		_read_part3 = true;

	} else if (type == LOG_RPL4_MSG) {
		uint8_t *dest_ptr = (uint8_t *)&replay_part4.time_rng_usec;
		parseMessage(data, dest_ptr, type);
		_range.timestamp = replay_part4.time_rng_usec;
		_range.current_distance = replay_part4.range_to_ground;
		_read_part4 = true;

	} else if (type == LOG_STAT_MSG) {
		uint8_t *dest_ptr = (uint8_t *)&vehicle_status.main_state;
		parseMessage(data, dest_ptr, type);
		_status.arming_state = vehicle_status.arming_state;
		_status.condition_landed = (bool)vehicle_status.landed;

		if (_status_pub == nullptr) {
			_status_pub = orb_advertise(ORB_ID(vehicle_status), &_status);

		} else if (_status_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_status), _status_pub, &_status);
		}
	}
}

void Ekf2Replay::writeMessage(int &fd, void *data, size_t size)
{
	if (size != ::write(fd, data, size)) {
		PX4_WARN("error writing to file");
	}
}

bool Ekf2Replay::needToSaveMessage(uint8_t type)
{
	if (type == LOG_ATT_MSG ||
	    type == LOG_LPOS_MSG ||
	    type == LOG_EST0_MSG ||
	    type == LOG_EST1_MSG ||
	    type == LOG_EST2_MSG ||
	    type == LOG_EST3_MSG ||
	    type == LOG_EST4_MSG ||
	    type == LOG_EST5_MSG ||
	    type == LOG_EST6_MSG ||
	    type == LOG_CTS_MSG) {
		return false;
	}

	return true;
}

// update all estimator topics and write them to log file
void Ekf2Replay::logIfUpdated()
{
	bool updated = false;

	// update attitude
	struct vehicle_attitude_s att = {};
	orb_copy(ORB_ID(vehicle_attitude), _att_sub, &att);

	// if the timestamp of the attitude is zero, then this means that the ekf did not
	// do an update so we can ignore this message and just continue
	if (att.timestamp == 0) {
		return;
	}

	memset(&log_message.body.att.q_w, 0, sizeof(log_ATT_s));

	log_message.type = LOG_ATT_MSG;
	log_message.head1 = HEAD_BYTE1;
	log_message.head2 = HEAD_BYTE2;
	log_message.body.att.q_w = att.q[0];
	log_message.body.att.q_x = att.q[1];
	log_message.body.att.q_y = att.q[2];
	log_message.body.att.q_z = att.q[3];
	log_message.body.att.roll = att.roll;
	log_message.body.att.pitch = att.pitch;
	log_message.body.att.yaw = att.yaw;
	log_message.body.att.roll_rate = att.rollspeed;
	log_message.body.att.pitch_rate = att.pitchspeed;
	log_message.body.att.yaw_rate = att.yawspeed;
	log_message.body.att.gx = att.g_comp[0];
	log_message.body.att.gy = att.g_comp[1];
	log_message.body.att.gz = att.g_comp[2];

	writeMessage(_write_fd, (void *)&log_message.head1, _formats[LOG_ATT_MSG].length);

	// update local position
	orb_check(_lpos_sub, &updated);

	if (updated) {
		struct vehicle_local_position_s lpos = {};
		orb_copy(ORB_ID(vehicle_local_position), _lpos_sub, &lpos);

		log_message.type = LOG_LPOS_MSG;
		log_message.head1 = HEAD_BYTE1;
		log_message.head2 = HEAD_BYTE2;
		log_message.body.lpos.x = lpos.x;
		log_message.body.lpos.y = lpos.y;
		log_message.body.lpos.z = lpos.z;
		log_message.body.lpos.ground_dist = lpos.dist_bottom;
		log_message.body.lpos.ground_dist_rate = lpos.dist_bottom_rate;
		log_message.body.lpos.vx = lpos.vx;
		log_message.body.lpos.vy = lpos.vy;
		log_message.body.lpos.vz = lpos.vz;
		log_message.body.lpos.ref_lat = lpos.ref_lat * 1e7;
		log_message.body.lpos.ref_lon = lpos.ref_lon * 1e7;
		log_message.body.lpos.ref_alt = lpos.ref_alt;
		log_message.body.lpos.pos_flags = (lpos.xy_valid ? 1 : 0) |
						  (lpos.z_valid ? 2 : 0) |
						  (lpos.v_xy_valid ? 4 : 0) |
						  (lpos.v_z_valid ? 8 : 0) |
						  (lpos.xy_global ? 16 : 0) |
						  (lpos.z_global ? 32 : 0);
		log_message.body.lpos.ground_dist_flags = (lpos.dist_bottom_valid ? 1 : 0);
		log_message.body.lpos.eph = lpos.eph;
		log_message.body.lpos.epv = lpos.epv;

		writeMessage(_write_fd, (void *)&log_message.head1, _formats[LOG_LPOS_MSG].length);
	}

	// update estimator status
	orb_check(_estimator_status_sub, &updated);

	if (updated) {
		struct estimator_status_s est_status = {};
		orb_copy(ORB_ID(estimator_status), _estimator_status_sub, &est_status);
		unsigned maxcopy0 = (sizeof(est_status.states) < sizeof(log_message.body.est0.s)) ? sizeof(est_status.states) : sizeof(
					    log_message.body.est0.s);
		log_message.type = LOG_EST0_MSG;
		log_message.head1 = HEAD_BYTE1;
		log_message.head2 = HEAD_BYTE2;
		memset(&(log_message.body.est0.s), 0, sizeof(log_message.body.est0));
		memcpy(&(log_message.body.est0.s), est_status.states, maxcopy0);
		log_message.body.est0.n_states = est_status.n_states;
		log_message.body.est0.nan_flags = est_status.nan_flags;
		log_message.body.est0.health_flags = est_status.health_flags;
		log_message.body.est0.timeout_flags = est_status.timeout_flags;
		writeMessage(_write_fd, (void *)&log_message.head1, _formats[LOG_EST0_MSG].length);

		log_message.type = LOG_EST1_MSG;
		log_message.head1 = HEAD_BYTE1;
		log_message.head2 = HEAD_BYTE2;
		unsigned maxcopy1 = ((sizeof(est_status.states) - maxcopy0) < sizeof(log_message.body.est1.s)) ? (sizeof(
					    est_status.states) - maxcopy0) : sizeof(log_message.body.est1.s);
		memset(&(log_message.body.est1.s), 0, sizeof(log_message.body.est1.s));
		memcpy(&(log_message.body.est1.s), ((char *)est_status.states) + maxcopy0, maxcopy1);
		writeMessage(_write_fd, (void *)&log_message.head1, _formats[LOG_EST1_MSG].length);

		log_message.type = LOG_EST2_MSG;
		log_message.head1 = HEAD_BYTE1;
		log_message.head2 = HEAD_BYTE2;
		unsigned maxcopy2 = (sizeof(est_status.covariances) < sizeof(log_message.body.est2.cov)) ? sizeof(
					    est_status.covariances) : sizeof(log_message.body.est2.cov);
		memset(&(log_message.body.est2.cov), 0, sizeof(log_message.body.est2.cov));
		memcpy(&(log_message.body.est2.cov), est_status.covariances, maxcopy2);
		writeMessage(_write_fd, (void *)&log_message.head1, _formats[LOG_EST2_MSG].length);

		log_message.type = LOG_EST3_MSG;
		log_message.head1 = HEAD_BYTE1;
		log_message.head2 = HEAD_BYTE2;
		unsigned maxcopy3 = ((sizeof(est_status.covariances) - maxcopy2) < sizeof(log_message.body.est3.cov)) ? (sizeof(
					    est_status.covariances) - maxcopy2) : sizeof(log_message.body.est3.cov);
		memset(&(log_message.body.est3.cov), 0, sizeof(log_message.body.est3.cov));
		memcpy(&(log_message.body.est3.cov), ((char *)est_status.covariances) + maxcopy2, maxcopy3);
		writeMessage(_write_fd, (void *)&log_message.head1, _formats[LOG_EST3_MSG].length);

	}

	// update ekf2 innovations
	orb_check(_innov_sub, &updated);

	if (updated) {
		struct ekf2_innovations_s innov = {};
		orb_copy(ORB_ID(ekf2_innovations), _innov_sub, &innov);
		memset(&log_message.body.innov.s, 0, sizeof(log_message.body.innov.s));

		log_message.type = LOG_EST4_MSG;
		log_message.head1 = HEAD_BYTE1;
		log_message.head2 = HEAD_BYTE2;

		for (unsigned i = 0; i < 6; i++) {
			log_message.body.innov.s[i] = innov.vel_pos_innov[i];
			log_message.body.innov.s[i + 6] = innov.vel_pos_innov_var[i];
		}

		writeMessage(_write_fd, (void *)&log_message.head1, _formats[LOG_EST4_MSG].length);

		log_message.type = LOG_EST5_MSG;
		log_message.head1 = HEAD_BYTE1;
		log_message.head2 = HEAD_BYTE2;
		memset(&(log_message.body.innov2.s), 0, sizeof(log_message.body.innov2.s));

		for (unsigned i = 0; i < 3; i++) {
			log_message.body.innov2.s[i] = innov.mag_innov[i];
			log_message.body.innov2.s[i + 3] = innov.mag_innov_var[i];
		}

		log_message.body.innov2.s[6] = innov.heading_innov;
		log_message.body.innov2.s[7] = innov.heading_innov_var;
		writeMessage(_write_fd, (void *)&log_message.head1, _formats[LOG_EST5_MSG].length);

		// optical flow innovations and innovation variances
		log_message.type = LOG_EST6_MSG;
		log_message.head1 = HEAD_BYTE1;
		log_message.head2 = HEAD_BYTE2;
		memset(&(log_message.body.innov3.s), 0, sizeof(log_message.body.innov3.s));

		for (unsigned i = 0; i < 2; i++) {
			log_message.body.innov3.s[i] = innov.flow_innov[i];
			log_message.body.innov3.s[i + 2] = innov.flow_innov_var[i];
		}

		log_message.body.innov3.s[4] = innov.hagl_innov;
		log_message.body.innov3.s[5] = innov.hagl_innov_var;
		writeMessage(_write_fd, (void *)&log_message.head1, _formats[LOG_EST6_MSG].length);
	}

	// update control state
	orb_check(_control_state_sub, &updated);

	if (updated) {
		struct control_state_s control_state = {};
		orb_copy(ORB_ID(control_state), _control_state_sub, &control_state);
		log_message.type = LOG_CTS_MSG;
		log_message.head1 = HEAD_BYTE1;
		log_message.head2 = HEAD_BYTE2;
		log_message.body.control_state.vx_body = control_state.x_vel;
		log_message.body.control_state.vy_body = control_state.y_vel;
		log_message.body.control_state.vz_body = control_state.z_vel;
		log_message.body.control_state.airspeed = control_state.airspeed;
		log_message.body.control_state.roll_rate = control_state.roll_rate;
		log_message.body.control_state.pitch_rate = control_state.pitch_rate;
		log_message.body.control_state.yaw_rate = control_state.yaw_rate;
		writeMessage(_write_fd, (void *)&log_message.head1, _formats[LOG_CTS_MSG].length);
	}
}

void Ekf2Replay::publishAndWaitForEstimator()
{
	// reset the counter reference for the imu replay topic
	_part1_counter_ref = 0;

	publishEstimatorInput();

	// wait for estimator output to arrive
	int pret = px4_poll(&_fds[0], (sizeof(_fds) / sizeof(_fds[0])), 1000);

	if (pret == 0) {
		PX4_WARN("timeout");
	}

	if (pret < 0) {
		PX4_WARN("poll error");
	}

	if (_fds[0].revents & POLLIN) {
		// write all estimator messages to replay log file
		logIfUpdated();
	}
}

void Ekf2Replay::setUserParams(const char *filename)
{
	std::string line;
	std::ifstream myfile(filename);
	std::string param_name;
	std::string value_string;

	if (myfile.is_open()) {
		while (! myfile.eof()) {
			getline(myfile, line);

			if (line.empty()) {
				continue;
			}

			std::istringstream mystrstream(line);
			mystrstream >> param_name;
			mystrstream >> value_string;

			double param_value_double = std::stod(value_string);

			param_t handle = param_find(param_name.c_str());
			param_type_t param_format = param_type(handle);

			if (param_format == PARAM_TYPE_INT32) {
				int32_t value = 0;
				value = (int32_t)param_value_double;
				param_set(handle, (const void *)&value);

			} else if (param_format == PARAM_TYPE_FLOAT) {
				float value = 0;
				value = (float)param_value_double;
				param_set(handle, (const void *)&value);
			}
		}

		myfile.close();
	}
}

void Ekf2Replay::task_main()
{
	// formats
	const int _k_max_data_size = 1024;	// 16x16 bytes
	uint8_t data[_k_max_data_size] = {};
	const char param_file[] = "./rootfs/replay_params.txt";

	// Open log file from which we read data
	// TODO Check if file exists
	int fd = ::open(_file_name, O_RDONLY);

	// create path to write a replay file
	char *replay_log_name;
	replay_log_name = strtok(_file_name, ".");
	char tmp[] = "_replayed.px4log";
	char *path_to_replay_log = (char *) malloc(1 + strlen(tmp) + strlen(replay_log_name));
	strcpy(path_to_replay_log, ".");
	strcat(path_to_replay_log, replay_log_name);
	strcat(path_to_replay_log, tmp);

	// create path which tells user location of replay file
	char tmp2[] = "./build_posix_sitl_replay/src/firmware/posix";
	char *replay_file_location = (char *) malloc(1 + strlen(tmp) + strlen(tmp2) + strlen(replay_log_name));
	strcat(replay_file_location, tmp2);
	strcat(replay_file_location, replay_log_name);
	strcat(replay_file_location, tmp);

	// open logfile to write
	_write_fd = ::open(path_to_replay_log, O_WRONLY | O_CREAT, S_IRWXU);

	std::ifstream tmp_file;
	tmp_file.open(param_file);

	std::string line;
	bool set_default_params_in_file = false;

	if (tmp_file.is_open() && ! tmp_file.eof()) {
		getline(tmp_file, line);

		if (line.empty()) {
			// the parameter file is empty so write the default values to it
			set_default_params_in_file = true;
		}
	}

	tmp_file.close();

	std::ofstream myfile(param_file, std::ios::app);

	// subscribe to estimator topics
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_estimator_status_sub = orb_subscribe(ORB_ID(estimator_status));
	_innov_sub = orb_subscribe(ORB_ID(ekf2_innovations));
	_lpos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_control_state_sub = orb_subscribe(ORB_ID(control_state));

	// we use attitude updates from the estimator for synchronisation
	_fds[0].fd = _att_sub;
	_fds[0].events = POLLIN;

	bool read_first_header = false;
	bool set_user_params = false;

	PX4_INFO("Replay in progress... \n");
	PX4_INFO("Log data will be written to %s\n", replay_file_location);

	while (!_task_should_exit) {
		_message_counter++;
		uint8_t header[3] = {};

		if (::read(fd, header, 3) != 3) {
			if (!read_first_header) {
				PX4_WARN("error reading log file, is the path printed above correct?");

			} else {
				PX4_INFO("Done!");
			}

			_task_should_exit = true;
			continue;
		}

		read_first_header = true;

		if ((header[0] != HEAD_BYTE1 || header[1] != HEAD_BYTE2)) {
			// we assume that the log file is finished here
			PX4_WARN("Done!");
			_task_should_exit = true;
			continue;
		}

		// write header but only for messages which are not generated by the estimator
		if (needToSaveMessage(header[2])) {
			writeMessage(_write_fd, &header[0], 3);
		}

		if (header[2] == LOG_FORMAT_MSG) {
			// format message
			struct log_format_s f;

			if (::read(fd, &f.type, sizeof(f)) != sizeof(f)) {
				PRINT_READ_ERROR;
				_task_should_exit = true;
				continue;
			}

			writeMessage(_write_fd, &f.type, sizeof(log_format_s));

			memcpy(&_formats[f.type], &f, sizeof(f));

		} else if (header[2] == LOG_PARM_MSG) {
			// parameter message
			if (::read(fd, &data[0], sizeof(log_PARM_s)) != sizeof(log_PARM_s)) {
				PRINT_READ_ERROR;
				_task_should_exit = true;
				continue;
			}

			writeMessage(_write_fd, &data[0], sizeof(log_PARM_s));

			// apply the parameters
			char param_name[16];

			for (unsigned i = 0; i < 16; i++) {
				param_name[i] = data[i];

				if (data[i] == '\0') {
					break;
				}
			}

			float param_data = 0;
			memcpy(&param_data, &data[16], sizeof(float));
			param_t handle = param_find(param_name);
			param_type_t param_format = param_type(handle);

			if (param_format == PARAM_TYPE_INT32) {
				int32_t value = 0;
				value = (int32_t)param_data;
				param_set(handle, (const void *)&value);

			} else if (param_format == PARAM_TYPE_FLOAT) {
				param_set(handle, (const void *)&param_data);
			}

			// if the user param file was empty then we fill it with the ekf2 parameter values from
			// the log file
			if (set_default_params_in_file) {
				if (strncmp(param_name, "EKF2", 4) == 0) {
					std::ostringstream os;
					double value = (double)param_data;
					os << std::string(param_name) << " ";
					os << value << "\n";
					myfile << os.str();
				}
			}

		} else if (header[2] == LOG_VER_MSG) {
			// version message
			if (::read(fd, &data[0], sizeof(log_VER_s)) != sizeof(log_VER_s)) {
				PRINT_READ_ERROR;
				_task_should_exit = true;
				continue;
			}

			writeMessage(_write_fd, &data[0], sizeof(log_VER_s));

		} else if (header[2] == LOG_TIME_MSG) {
			// time message
			if (::read(fd, &data[0], sizeof(log_TIME_s)) != sizeof(log_TIME_s)) {
				// assume that this is because we have reached the end of the file
				PX4_INFO("Done!");
				_task_should_exit = true;
				continue;
			}

			writeMessage(_write_fd, &data[0], sizeof(log_TIME_s));

		} else {
			// the first time we arrive here we should apply the parameters specified in the user file
			// this makes sure they are applied after the parameter values of the log file
			if (!set_user_params) {
				myfile.close();
				setUserParams(param_file);
				set_user_params = true;
			}

			// data message
			if (::read(fd, &data[0], _formats[header[2]].length - 3) != _formats[header[2]].length - 3) {
				PX4_INFO("Done!");
				_task_should_exit = true;
				continue;
			}

			// all messages which we are not getting from the estimator are written
			// back into the replay log file
			if (needToSaveMessage(header[2])) {
				writeMessage(_write_fd, &data[0], _formats[header[2]].length - 3);
			}

			if (header[2] == LOG_RPL1_MSG && _part1_counter_ref > 0) {
				// we have found another imu replay message while we still have one waiting to be published.
				// so publish that now
				publishAndWaitForEstimator();
			}

			// set estimator input data
			setEstimatorInput(&data[0], header[2]);

			// we have read the imu replay message (part 1) and have waited 3 more cycles for other replay message parts
			// e.g. flow, gps or range. we know that in case they were written to the log file they should come right after
			// the first replay message, therefore, we can kick the estimator now
			if (_part1_counter_ref > 0 && _part1_counter_ref < _message_counter - 3) {
				publishAndWaitForEstimator();
			}
		}
	}

	::close(_write_fd);
	::close(fd);
	delete ekf2_replay::instance;
	ekf2_replay::instance = nullptr;
}

void Ekf2Replay::task_main_trampoline(int argc, char *argv[])
{
	ekf2_replay::instance->task_main();
}

int Ekf2Replay::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("ekf2_replay",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   3000,
					   (px4_main_t)&Ekf2Replay::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

int ekf2_replay_main(int argc, char *argv[])
{
	if (argc < 1) {
		PX4_WARN("usage: ekf2_replay {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (ekf2_replay::instance != nullptr) {
			PX4_WARN("already running");
			return 1;
		}

		ekf2_replay::instance = new Ekf2Replay(argv[2]);

		if (ekf2_replay::instance == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		if (OK != ekf2_replay::instance->start()) {
			delete ekf2_replay::instance;
			ekf2_replay::instance = nullptr;
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (ekf2_replay::instance == nullptr) {
			PX4_WARN("not running");
			return 1;
		}

		ekf2_replay::instance->exit();

		// wait for the destruction of the instance
		while (ekf2_replay::instance != nullptr) {
			usleep(50000);
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (ekf2_replay::instance) {
			PX4_WARN("running");
			return 0;

		} else {
			PX4_WARN("not running");
			return 1;
		}
	}

	PX4_WARN("unrecognized command");
	return 1;
}
