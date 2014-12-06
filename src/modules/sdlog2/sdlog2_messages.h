/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file sdlog2_messages.h
 *
 * Log messages and structures definition.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef SDLOG2_MESSAGES_H_
#define SDLOG2_MESSAGES_H_

#include "sdlog2_format.h"

/* define message formats */

#pragma pack(push, 1)
/* --- ATT - ATTITUDE --- */
#define LOG_ATT_MSG 2
struct log_ATT_s {
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

/* --- ATSP - ATTITUDE SET POINT --- */
#define LOG_ATSP_MSG 3
struct log_ATSP_s {
	float roll_sp;
	float pitch_sp;
	float yaw_sp;
	float thrust_sp;
};

/* --- IMU - IMU SENSORS --- */
#define LOG_IMU_MSG 4
#define LOG_IMU1_MSG 22
#define LOG_IMU2_MSG 23
struct log_IMU_s {
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
};

/* --- SENS - OTHER SENSORS --- */
#define LOG_SENS_MSG 5
struct log_SENS_s {
	float baro_pres;
	float baro_alt;
	float baro_temp;
	float diff_pres;
	float diff_pres_filtered;
};

/* --- LPOS - LOCAL POSITION --- */
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
	uint8_t landed;
	uint8_t ground_dist_flags;
	float eph;
	float epv;
};

/* --- LPSP - LOCAL POSITION SETPOINT --- */
#define LOG_LPSP_MSG 7
struct log_LPSP_s {
	float x;
	float y;
	float z;
	float yaw;
};

/* --- GPS - GPS POSITION --- */
#define LOG_GPS_MSG 8
struct log_GPS_s {
	uint64_t gps_time;
	uint8_t fix_type;
	float eph;
	float epv;
	int32_t lat;
	int32_t lon;
	float alt;
	float vel_n;
	float vel_e;
	float vel_d;
	float cog;
	uint8_t sats;
	uint16_t snr_mean;
	uint16_t noise_per_ms;
	uint16_t jamming_indicator;
};

/* --- ATTC - ATTITUDE CONTROLS (ACTUATOR_0 CONTROLS)--- */
#define LOG_ATTC_MSG 9
struct log_ATTC_s {
	float roll;
	float pitch;
	float yaw;
	float thrust;
};

/* --- STAT - VEHICLE STATE --- */
#define LOG_STAT_MSG 10
struct log_STAT_s {
	uint8_t main_state;
	uint8_t arming_state;
	uint8_t failsafe_state;
	float battery_remaining;
	uint8_t battery_warning;
	uint8_t landed;
};

/* --- RC - RC INPUT CHANNELS --- */
#define LOG_RC_MSG 11
struct log_RC_s {
	float channel[8];
	uint8_t channel_count;
	uint8_t signal_lost;
};

/* --- OUT0 - ACTUATOR_0 OUTPUT --- */
#define LOG_OUT0_MSG 12
struct log_OUT0_s {
	float output[8];
};

/* --- AIRS - AIRSPEED --- */
#define LOG_AIRS_MSG 13
struct log_AIRS_s {
	float indicated_airspeed;
	float true_airspeed;
	float air_temperature_celsius;
};

/* --- ARSP - ATTITUDE RATE SET POINT --- */
#define LOG_ARSP_MSG 14
struct log_ARSP_s {
	float roll_rate_sp;
	float pitch_rate_sp;
	float yaw_rate_sp;
};

/* --- FLOW - OPTICAL FLOW --- */
#define LOG_FLOW_MSG 15
struct log_FLOW_s {
	uint64_t timestamp;
	uint8_t sensor_id;
	float pixel_flow_x_integral;
	float pixel_flow_y_integral;
	float gyro_x_rate_integral;
	float gyro_y_rate_integral;
	float gyro_z_rate_integral;
	float ground_distance_m;
	uint32_t integration_timespan;
	uint32_t time_since_last_sonar_update;
	uint16_t frame_count_since_last_readout;
	int16_t gyro_temperature;
	uint8_t	quality;
};

/* --- GPOS - GLOBAL POSITION ESTIMATE --- */
#define LOG_GPOS_MSG 16
struct log_GPOS_s {
	int32_t lat;
	int32_t lon;
	float alt;
	float vel_n;
	float vel_e;
	float vel_d;
	float eph;
	float epv;
	float terrain_alt;
};

/* --- GPSP - GLOBAL POSITION SETPOINT --- */
#define LOG_GPSP_MSG 17
struct log_GPSP_s {
	uint8_t nav_state;
	int32_t lat;
	int32_t lon;
	float alt;
	float yaw;
	uint8_t type;
	float loiter_radius;
	int8_t loiter_direction;
	float pitch_min;
};

/* --- ESC - ESC STATE --- */
#define LOG_ESC_MSG 18
struct log_ESC_s {
	uint16_t counter;
	uint8_t  esc_count;
	uint8_t  esc_connectiontype;
	uint8_t  esc_num;
	uint16_t esc_address;
	uint16_t esc_version;
	float    esc_voltage;
	float    esc_current;
	int32_t  esc_rpm;
	float    esc_temperature;
	float    esc_setpoint;
	uint16_t esc_setpoint_raw;
};

/* --- GVSP - GLOBAL VELOCITY SETPOINT --- */
#define LOG_GVSP_MSG 19
struct log_GVSP_s {
	float vx;
	float vy;
	float vz;
};

/* --- BATT - BATTERY --- */
#define LOG_BATT_MSG 20
struct log_BATT_s {
	float voltage;
	float voltage_filtered;
	float current;
	float discharged;
};

/* --- DIST - DISTANCE TO SURFACE --- */
#define LOG_DIST_MSG 21
struct log_DIST_s {
	float bottom;
	float bottom_rate;
	uint8_t flags;
};

/* LOG IMU1 and IMU2 MSGs consume IDs 22 and 23 */


/* --- PWR - ONBOARD POWER SYSTEM --- */
#define LOG_PWR_MSG 24
struct log_PWR_s {
	float peripherals_5v;
	float servo_rail_5v;
	float servo_rssi;
	uint8_t usb_ok;
	uint8_t brick_ok;
	uint8_t servo_ok;
	uint8_t low_power_rail_overcurrent;
	uint8_t high_power_rail_overcurrent;
};

/* --- VICN - VICON POSITION --- */
#define LOG_VICN_MSG 25
struct log_VICN_s {
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
};

/* --- GS0A - GPS SNR #0, SAT GROUP A --- */
#define LOG_GS0A_MSG 26
struct log_GS0A_s {
	uint8_t satellite_snr[16];			/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99 */
};

/* --- GS0B - GPS SNR #0, SAT GROUP B --- */
#define LOG_GS0B_MSG 27
struct log_GS0B_s {
	uint8_t satellite_snr[16];			/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99 */
};

/* --- GS1A - GPS SNR #1, SAT GROUP A --- */
#define LOG_GS1A_MSG 28
struct log_GS1A_s {
	uint8_t satellite_snr[16];			/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99 */
};

/* --- GS1B - GPS SNR #1, SAT GROUP B --- */
#define LOG_GS1B_MSG 29
struct log_GS1B_s {
	uint8_t satellite_snr[16];			/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99 */
};

/* --- TECS - TECS STATUS --- */
#define LOG_TECS_MSG 30
struct log_TECS_s {
	float altitudeSp;
	float altitudeFiltered;
	float flightPathAngleSp;
	float flightPathAngle;
	float flightPathAngleFiltered;
	float airspeedSp;
	float airspeedFiltered;
	float airspeedDerivativeSp;
	float airspeedDerivative;

	float totalEnergyRateSp;
	float totalEnergyRate;
	float energyDistributionRateSp;
	float energyDistributionRate;

	uint8_t mode;
};

/* --- WIND - WIND ESTIMATE --- */
#define LOG_WIND_MSG 31
struct log_WIND_s {
	float x;
	float y;
	float cov_x;
	float cov_y;
};

/* --- EST0 - ESTIMATOR STATUS --- */
#define LOG_EST0_MSG 32
struct log_EST0_s {
	float s[12];
	uint8_t n_states;
	uint8_t nan_flags;
	uint8_t health_flags;
	uint8_t timeout_flags;
};

/* --- EST1 - ESTIMATOR STATUS --- */
#define LOG_EST1_MSG 33
struct log_EST1_s {
	float s[16];
};

/* --- TEL0..3 - TELEMETRY STATUS --- */
#define LOG_TEL0_MSG 34
#define LOG_TEL1_MSG 35
#define LOG_TEL2_MSG 36
#define LOG_TEL3_MSG 37
struct log_TEL_s {
	uint8_t rssi;
	uint8_t remote_rssi;
	uint8_t noise;
	uint8_t remote_noise;
	uint16_t rxerrors;
	uint16_t fixed;
	uint8_t txbuf;
	uint64_t heartbeat_time;
};

/* --- VISN - VISION POSITION --- */
#define LOG_VISN_MSG 38
struct log_VISN_s {
	float x;
	float y;
	float z;
	float vx;
	float vy;
	float vz;
	float qx;
	float qy;
	float qz;
	float qw;
};

/* --- ENCODERS - ENCODER DATA --- */
#define LOG_ENCD_MSG 39
struct log_ENCD_s {
	int64_t cnt0;
	float vel0;
	int64_t cnt1;
	float vel1;
};


/********** SYSTEM MESSAGES, ID > 0x80 **********/

/* --- TIME - TIME STAMP --- */
#define LOG_TIME_MSG 129
struct log_TIME_s {
	uint64_t t;
};

/* --- VER - VERSION --- */
#define LOG_VER_MSG 130
struct log_VER_s {
	char arch[16];
	char fw_git[64];
};

/* --- PARM - PARAMETER --- */
#define LOG_PARM_MSG 131
struct log_PARM_s {
	char name[16];
	float value;
};

#pragma pack(pop)

/* construct list of all message formats */
static const struct log_format_s log_formats[] = {
	/* business-level messages, ID < 0x80 */
	LOG_FORMAT(ATT, "fffffffff",		"Roll,Pitch,Yaw,RollRate,PitchRate,YawRate,GX,GY,GZ"),
	LOG_FORMAT(ATSP, "ffff",		"RollSP,PitchSP,YawSP,ThrustSP"),
	LOG_FORMAT_S(IMU, IMU, "fffffffff",		"AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ"),
	LOG_FORMAT_S(IMU1, IMU, "fffffffff",		"AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ"),
	LOG_FORMAT_S(IMU2, IMU, "fffffffff",		"AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ"),
	LOG_FORMAT(SENS, "fffff",		"BaroPres,BaroAlt,BaroTemp,DiffPres,DiffPresFilt"),
	LOG_FORMAT(LPOS, "ffffffffLLfBBBff",	"X,Y,Z,Dist,DistR,VX,VY,VZ,RLat,RLon,RAlt,PFlg,LFlg,GFlg,EPH,EPV"),
	LOG_FORMAT(LPSP, "ffff",		"X,Y,Z,Yaw"),
	LOG_FORMAT(GPS, "QBffLLfffffBHHH",	"GPSTime,Fix,EPH,EPV,Lat,Lon,Alt,VelN,VelE,VelD,Cog,nSat,SNR,N,J"),
	LOG_FORMAT(ATTC, "ffff",		"Roll,Pitch,Yaw,Thrust"),
	LOG_FORMAT(STAT, "BBBfBB",		"MainState,ArmState,FailsafeState,BatRem,BatWarn,Landed"),
	LOG_FORMAT(RC, "ffffffffBB",		"Ch0,Ch1,Ch2,Ch3,Ch4,Ch5,Ch6,Ch7,Count,SignalLost"),
	LOG_FORMAT(OUT0, "ffffffff",		"Out0,Out1,Out2,Out3,Out4,Out5,Out6,Out7"),
	LOG_FORMAT(AIRS, "fff",			"IndSpeed,TrueSpeed,AirTemp"),
	LOG_FORMAT(ARSP, "fff",			"RollRateSP,PitchRateSP,YawRateSP"),
	LOG_FORMAT(FLOW, "hhfffBB",		"RawX,RawY,CompX,CompY,Dist,Q,SensID"),
	LOG_FORMAT(GPOS, "LLfffffff",		"Lat,Lon,Alt,VelN,VelE,VelD,EPH,EPV,TALT"),
	LOG_FORMAT(GPSP, "BLLffBfbf",		"NavState,Lat,Lon,Alt,Yaw,Type,LoitR,LoitDir,PitMin"),
	LOG_FORMAT(ESC, "HBBBHHffiffH",		"count,nESC,Conn,N,Ver,Adr,Volt,Amp,RPM,Temp,SetP,SetPRAW"),
	LOG_FORMAT(GVSP, "fff",			"VX,VY,VZ"),
	LOG_FORMAT(BATT, "ffff",		"V,VFilt,C,Discharged"),
	LOG_FORMAT(DIST, "ffB",			"Bottom,BottomRate,Flags"),
	LOG_FORMAT_S(TEL0, TEL, "BBBBHHBQ",		"RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime"),
	LOG_FORMAT_S(TEL1, TEL, "BBBBHHBQ",		"RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime"),
	LOG_FORMAT_S(TEL2, TEL, "BBBBHHBQ",		"RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime"),
	LOG_FORMAT_S(TEL3, TEL, "BBBBHHBQ",		"RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime"),
	LOG_FORMAT(EST0, "ffffffffffffBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,nStat,fNaN,fHealth,fTOut"),
	LOG_FORMAT(EST1, "ffffffffffffffff",	"s12,s13,s14,s15,s16,s17,s18,s19,s20,s21,s22,s23,s24,s25,s26,s27"),
	LOG_FORMAT(PWR, "fffBBBBB",		"Periph5V,Servo5V,RSSI,UsbOk,BrickOk,ServoOk,PeriphOC,HipwrOC"),
	LOG_FORMAT(VICN, "ffffff",		"X,Y,Z,Roll,Pitch,Yaw"),
	LOG_FORMAT(VISN, "ffffffffff",		"X,Y,Z,VX,VY,VZ,QuatX,QuatY,QuatZ,QuatW"),
	LOG_FORMAT(GS0A, "BBBBBBBBBBBBBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15"),
	LOG_FORMAT(GS0B, "BBBBBBBBBBBBBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15"),
	LOG_FORMAT(GS1A, "BBBBBBBBBBBBBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15"),
	LOG_FORMAT(GS1B, "BBBBBBBBBBBBBBBB",	"s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15"),
	LOG_FORMAT(TECS, "fffffffffffffB",	"ASP,AF,FSP,F,FF,AsSP,AsF,AsDSP,AsD,TERSP,TER,EDRSP,EDR,M"),
	LOG_FORMAT(WIND, "ffff",	"X,Y,CovX,CovY"),
	LOG_FORMAT(ENCD, "qfqf",	"cnt0,vel0,cnt1,vel1"),

	/* system-level messages, ID >= 0x80 */
	/* FMT: don't write format of format message, it's useless */
	LOG_FORMAT(TIME, "Q", "StartTime"),
	LOG_FORMAT(VER, "NZ", "Arch,FwGit"),
	LOG_FORMAT(PARM, "Nf", "Name,Value")
};

static const unsigned log_formats_num = sizeof(log_formats) / sizeof(log_formats[0]);

#endif /* SDLOG2_MESSAGES_H_ */
