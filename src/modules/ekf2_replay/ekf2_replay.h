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
 * @file ekf2_replay.h
 *
 * @author Roman Bapst
*/

// TODO These struct are already defined in sdlog2_messages.h
// However, the file cannot be included because of different use of struct
// initialization (C99 not available for C++).

#define LOG_RPL1_MSG 51
#define LOG_RPL2_MSG 52
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149
#define LOG_FORMAT_MSG	0x80
#define LOG_VER_MSG 	130
#define LOG_PARM_MSG 	131
#define LOG_TIME_MSG 	129

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

#define LOG_ATT_MSG 2
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

#define LOG_EST0_MSG 32
struct log_EST0_s {
	float s[12];
	uint8_t n_states;
	uint8_t nan_flags;
	uint8_t health_flags;
	uint8_t timeout_flags;
};

#define LOG_EST1_MSG 33
struct log_EST1_s {
	float s[16];
};

#define LOG_EST2_MSG 34
struct log_EST2_s {
	float cov[12];
};

#define LOG_EST3_MSG 35
struct log_EST3_s {
	float cov[16];
};

#define LOG_EST4_MSG 48
struct log_EST4_s {
	float s[12];
};

#define LOG_EST5_MSG 49
struct log_EST5_s {
	float s[8];
};

#define LOG_LPOS_MSG 6
struct log_LPOS_s {
	float x;
	float y;
	float z;
	float ground_dist;
	float ground_dist_rate;
	float vx;
	float vy;
	float vz;
	int32_t ref_lat;
	int32_t ref_lon;
	float ref_alt;
	uint8_t pos_flags;
	uint8_t ground_dist_flags;
	float eph;
	float epv;
};

#define LOG_CTS_MSG 47
struct log_CTS_s {
	float vx_body;
	float vy_body;
	float vz_body;
	float airspeed;
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
};

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
	} body;
} log_message;

#pragma pack(pop)