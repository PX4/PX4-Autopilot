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
 * Replay module for ekf2.
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
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>

#include <uORB/topics/ekf2_replay.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>

extern "C" __EXPORT int ekf2_replay_main(int argc, char *argv[]);

#define LOG_RPL1_MSG 51
#define LOG_RPL2_MSG 52
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149
#define LOG_FORMAT_MSG	0x80
#define LOG_VER_MSG 	130
#define LOG_PARM_MSG 	131
#define LOG_TIME_MSG 	129

class Ekf2Replay;

#pragma pack(push, 1)
struct log_format_s {
	uint8_t type;
	uint8_t length;		// full packet length including header
	char name[4];
	char format[16];
	char labels[64];
};

struct log_VER_s {
	char arch[16];
	char fw_git[64];
};

struct log_PARM_s {
	char name[16];
	float value;
};

struct log_TIME_s {
	uint64_t t;
};

struct log_RPL1_s {
	uint64_t time_ref;
	uint64_t gyro_integral_dt;
	uint64_t accelerometer_integral_dt;
	uint64_t magnetometer_timestamp;
	uint64_t baro_timestamp;
	float gyro_integral_x_rad;
	float gyro_integral_y_rad;
	float gyro_integral_z_rad;
	float accelerometer_integral_x_m_s;
	float accelerometer_integral_y_m_s;
	float accelerometer_integral_z_m_s;
	float magnetometer_x_ga;
	float magnetometer_y_ga;
	float magnetometer_z_ga;
	float baro_alt_meter;
};

struct log_RPL2_s {
	uint64_t time_pos_usec;
	uint64_t time_vel_usec;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	uint8_t fix_type;
	float eph;
	float epv;
	float vel_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float vel_d_m_s;
	bool vel_ned_valid;
};

struct log_ATT_s {
	float q_w;
	float q_x;
	float q_y;
	float q_z;
	float roll;
	float pitch;
	float yaw;
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
	float gx;
	float gy;
	float gz;
};

#pragma pack(pop)

namespace ekf2_replay
{
Ekf2Replay *instance = nullptr;
}

class Ekf2Replay {
public:
	/**
	 * Constructor
	 */
	Ekf2Replay(char *logfile);

	/**
	 * Destructor, also kills task.
	 */
	~Ekf2Replay();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	void exit() { _task_should_exit = true; }

	static void	task_main_trampoline(int argc, char *argv[]);

	void		task_main();

private:
	int		_control_task = -1;			/**< task handle for task */
	bool	_task_should_exit = false;

	orb_advert_t _sensors_pub;
	orb_advert_t _gps_pub;

	char *_file_name;

	struct log_format_s _formats[100];
	struct sensor_combined_s _sensors;
	struct vehicle_gps_position_s _gps;

	bool _read_part1;
	bool _read_part2;

	void parseMessage(uint8_t *data, uint8_t type);

	void readData(uint8_t * source, uint8_t *destination, uint8_t type);

	void publishSensorData();
};

Ekf2Replay::Ekf2Replay(char *logfile) :
_sensors_pub(nullptr),
_gps_pub(nullptr),
_formats{},
_sensors{},
_gps{},
_read_part1(false),
_read_part2(false)
{
	// build the path to the log
	char tmp[] = "./rootfs/";
	char *path_to_log = (char *) malloc(1 + strlen(tmp)+ strlen(logfile) );
	strcpy(path_to_log, tmp);
	strcat(path_to_log, logfile);
	_file_name = path_to_log;

}

Ekf2Replay::~Ekf2Replay()
{

}

void Ekf2Replay::publishSensorData()
{
	if (_sensors_pub == nullptr && _read_part1) {
		_sensors_pub = orb_advertise(ORB_ID(sensor_combined), &_sensors);
	} else if (_sensors_pub != nullptr && _read_part1) {
		orb_publish(ORB_ID(sensor_combined), _sensors_pub, &_sensors);
	}

	if (_gps_pub == nullptr && _gps.timestamp_position > 0) {
		_gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_gps);
	} else if (_gps_pub != nullptr && _gps.timestamp_position > 0) {
		orb_publish(ORB_ID(vehicle_gps_position), _gps_pub, &_gps);
	}
}

void Ekf2Replay::readData(uint8_t * source, uint8_t *destination, uint8_t type) 
{
	int i = 0;
	int write_ptr = 0;

	while (_formats[type].format[i] != '\0') {
		char data_type = _formats[type].format[i];
		switch(data_type) {
			case 'f':
				memcpy(&destination[write_ptr], &source[write_ptr], sizeof(float));
				write_ptr += sizeof(float);
				break;
			case 'Q':
				memcpy(&destination[write_ptr], &source[write_ptr], sizeof(uint64_t));
				write_ptr += sizeof(uint64_t);
				break;
			case 'L':
				memcpy(&destination[write_ptr], &source[write_ptr], sizeof(int32_t));
				write_ptr += sizeof(int32_t);
				break;
			case 'M':
				memcpy(&destination[write_ptr], &source[write_ptr], sizeof(uint8_t));
				write_ptr += sizeof(uint8_t);
				break;
		}

		i++;
	}
}

void Ekf2Replay::parseMessage(uint8_t *data, uint8_t type)
{
	struct log_RPL1_s replay_part1 = {};
	struct log_RPL2_s replay_part2 = {};

	if (type == LOG_RPL1_MSG) {
		
		uint8_t *dest_ptr = (uint8_t *)&replay_part1.time_ref;
		readData(data, dest_ptr, type);
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
		_read_part1 = true;


	} else if (type == LOG_RPL2_MSG) {
		uint8_t *dest_ptr = (uint8_t *)&replay_part2.time_pos_usec;
		readData(data, dest_ptr, type);
		_gps.timestamp_position = replay_part2.time_pos_usec;
		_gps.timestamp_velocity = replay_part2.time_vel_usec;
		_gps.lat = replay_part2.lat;
		_gps.lon = replay_part2.lon;
		_gps.fix_type = replay_part2.fix_type;
		_gps.eph = replay_part2.eph;
		_gps.epv = replay_part2.epv;
		_gps.vel_m_s = replay_part2.vel_m_s;
		_gps.vel_n_m_s = replay_part2.vel_n_m_s;
		_gps.vel_e_m_s = replay_part2.vel_e_m_s;
		_gps.vel_d_m_s = replay_part2.vel_d_m_s;
		_gps.vel_ned_valid = replay_part2.vel_ned_valid;
		_read_part2 = true;
	}
}

void Ekf2Replay::task_main()
{
	// formats
	const int _k_max_data_size = 1024;	// 16x16 bytes
	uint8_t data[_k_max_data_size] = {};

	printf("opening %s for ekf2 replay\n", _file_name);
	// TODO Check if file exists
	int fd = ::open(_file_name, O_RDONLY);
	bool reached_end = false;

	while(!_task_should_exit) {
		do {
			uint8_t header[3];

			if (::read(fd, header, 3) != 3) {
				PX4_WARN("error reading log file, is the path printed above correct?");
				reached_end = true;
				_task_should_exit = true;
				return;
			}

			if (header[0] != HEAD_BYTE1 || header[1] != HEAD_BYTE2) {
				PX4_WARN("bad log header\n");
				_task_should_exit = true;
			}

			if (header[2] == LOG_FORMAT_MSG) {
				struct log_format_s f;
				if(::read(fd, &f.type, sizeof(f)) != sizeof(f)) {
					PX4_WARN("error reading from log file");
					_task_should_exit = true;
				}
				memcpy(&_formats[f.type], &f, sizeof(f));
			} else if (header[2] == LOG_PARM_MSG) {
				if(::read(fd, &data[0], sizeof(log_PARM_s)) != sizeof(log_PARM_s)) {
					PX4_WARN("error reading from log file");
				}
			} else if (header[2] == LOG_VER_MSG) {
				if(::read(fd, &data[0], sizeof(log_VER_s)) != sizeof(log_VER_s)) {
					PX4_WARN("error reading from log file");
				}
			} else if (header[2] == LOG_TIME_MSG) {
				if(::read(fd, &data[0], sizeof(log_TIME_s)) != sizeof(log_TIME_s)) {
					PX4_WARN("error reading from log file");
				}
			} else {
				if(::read(fd, &data[0], _formats[header[2]].length - 3) != _formats[header[2]].length - 3) {
					PX4_WARN("Done, check the posix log directory for the latest log!");
					return;
				}
				
				parseMessage(&data[0], header[2]);

				if (_read_part1 || _read_part2) {
					publishSensorData();
					_read_part1 = _read_part2 = false;
					// TODO: Make this variable
					usleep(2000);
				}
			}
		} while (!reached_end || _task_should_exit);
	}
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