/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdint.h>

#include <matrix/matrix/math.hpp>

#define PACKED __attribute__((packed))

namespace InertialLabs {

// NOLINTBEGIN(clang-analyzer-optin.performance.Padding, altera-struct-pack-align, misc-non-private-member-variables-in-classes)
struct PACKED vec3_16_t {
	int16_t x, y, z;

	matrix::Vector3f toFloat() const {
		return matrix::Vector3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
	}
};

struct PACKED vec3_32_t {
	int32_t x, y, z;

	matrix::Vector3f toFloat() const {
		return matrix::Vector3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
	}
};

enum DataType : uint8_t {
	GPS_INS_TIME_MS       = 0x01,
	GPS_WEEK              = 0x3C,
	ACCEL_DATA_HR         = 0x23,
	GYRO_DATA_HR          = 0x21,
	BARO_DATA             = 0x25,
	MAG_DATA              = 0x24,
	SENSOR_BIAS           = 0x26,
	ORIENTATION_ANGLES    = 0x07,
	VELOCITIES            = 0x12,
	POSITION              = 0x10,
	UNIT_STATUS           = 0x53,
	GNSS_EXTENDED_INFO    = 0x4A,
	GNSS_POSITION         = 0x30,
	GNSS_VEL_TRACK        = 0x32,
	GNSS_POS_TIMESTAMP    = 0x3E,
	GNSS_NEW_DATA         = 0x41,
	GNSS_JAM_STATUS       = 0xC0,
	DIFFERENTIAL_PRESSURE = 0x28,
	TRUE_AIRSPEED         = 0x86,
	CALIBRATED_AIRSPEED   = 0x85,
	WIND_SPEED            = 0x8A,
	AIR_DATA_STATUS       = 0x8D,
	SUPPLY_VOLTAGE        = 0x50,
	TEMPERATURE           = 0x52,
	UNIT_STATUS2          = 0x5A,
	GNSS_DOP              = 0x42,
	INS_SOLUTION_STATUS   = 0x54,
	INS_POS_VEL_ACCURACY  = 0x5F,
	FULL_SAT_INFO         = 0x37,
	USED_SAT_COUNT        = 0x3B,
	GNSS_VEL_LATENCY      = 0x3D,
	GNSS_SOL_STATUS       = 0x38,
	GNSS_POS_VEL_TYPE     = 0x39,
	NEW_AIDING_DATA       = 0x65,
	NEW_AIDING_DATA2      = 0xA1,
	EXT_SPEED             = 0x61,
	EXT_HOR_POS           = 0x6E,
	EXT_ALT               = 0x6C,
	EXT_HEADING           = 0x66,
	EXT_AMBIENT_DATA      = 0x6B,
	EXT_WIND_DATA         = 0x62,
	MAG_CLB_ACCURACY      = 0x9A,
};

// Ins Solutiob Status
enum InsSolution {
	GOOD                       = 0,
	KF_CONVERGENCE_IN_PROGRESS = 1,
	GNSS_ABSENCE               = 2,
	NO_HEADING_CORRECTION      = 3,
	AUTONOMOUS_MODE            = 4,
	NO_GNSS_AIDING_DATA        = 5,
	AUTONIMUS_TIME_EXCEEDED    = 6,
	ZUPT_MODE                  = 7,
	INVALID                    = 8,
};

// Unit Status Word bits
enum USW {
	INITIAL_ALIGNMENT_FAIL = (1U << 0),
	OPERATION_FAIL         = (1U << 1),
	GYRO_FAIL              = (1U << 2),
	ACCEL_FAIL             = (1U << 3),
	MAG_FAIL               = (1U << 4),
	ELECTRONICS_FAIL       = (1U << 5),
	GNSS_FAIL              = (1U << 6),
	MAG_VG3D_CLB_RUNTIME   = (1U << 7),
	VOLTAGE_LOW            = (1U << 8),
	VOLTAGE_HIGH           = (1U << 9),
	GYRO_X_RATE_HIGH       = (1U << 10),
	GYRO_Y_RATE_HIGH       = (1U << 11),
	GYRO_Z_RATE_HIGH       = (1U << 12),
	MAG_FIELD_HIGH         = (1U << 13),
	TEMP_RANGE_ERR         = (1U << 14),
	MAG_VG3D_CLB_SUCCESS   = (1U << 15),
};

// Unit Status Word2 bits
enum USW2 {
	ACCEL_X_HIGH            = (1U << 0),
	ACCEL_Y_HIGH            = (1U << 1),
	ACCEL_Z_HIGH            = (1U << 2),
	ADU_BARO_FAIL           = (1U << 3),
	ADU_DIFF_PRESS_FAIL     = (1U << 4),
	MAG_AUTO_CAL_2D_RUNTIME = (1U << 5),
	MAG_AUTO_CAL_3D_RUNTIME = (1U << 6),
	GNSS_FUSION_OFF         = (1U << 7),
	DIFF_PRESS_FUSION_OFF   = (1U << 8),
	GNSS_POS_VALID          = (1U << 10),
};

// Air Data Status bits
enum ADU {
	BARO_INIT_FAIL           = (1U << 0),
	DIFF_PRESS_INIT_FAIL     = (1U << 1),
	BARO_FAIL                = (1U << 2),
	DIFF_PRESS_FAIL          = (1U << 3),
	BARO_RANGE_ERR           = (1U << 4),
	DIFF_PRESS_RANGE_ERR     = (1U << 5),
	BARO_ALT_FAIL            = (1U << 8),
	AIRSPEED_FAIL            = (1U << 9),
	AIRSPEED_BELOW_THRESHOLD = (1U << 10),
};

// New GPS indicator
enum NewGpsData {
	NEW_GNSS_POSITION    = (1U << 0),
	NEW_GNSS_VELOCITY    = (1U << 1),
	NEW_GNSS_HEADING     = (1U << 2),
	NEW_VALID_PPS        = (1U << 3),
	NEW_GNSS_BESTXYZ_LOG = (1U << 4),
	NEW_GNSS_PSRDOP_LOG  = (1U << 5),
	NEW_PPS              = (1U << 6),
	NEW_RANGE_LOG        = (1U << 7),
};

enum NewAidingData {
	NEW_AIRSPEED    = (1U << 1),
	NEW_WIND        = (1U << 2),
	NEW_EXT_POS     = (1U << 3),
	NEW_HEADING     = (1U << 5),
	NEW_AMBIENT     = (1U << 10),
	NEW_ALTITUDE    = (1U << 11),
	NEW_EXT_HOR_POS = (1U << 13),
	NEW_ADC         = (1U << 15),
};

enum JammingStatus : uint8_t {
	UNKOWN_OR_DISABLED = 0,
	OK                 = 1,  // No significant jamming
	WARNING            = 2,  // Interference visible but fix ok
	CRITICAL           = 3,  // Interference visible and no fix
};

enum SpoofingStatus : uint8_t {
	UNKOWN_OR_DEACTIVATED = 0,
	NO                    = 1,
	INDICATED             = 2,
	MULTIPLE_INCIDATIONS  = 3,
};

/*
	Every data package consist of:
		MessageHeader (6 bytes)
		Payload
		Checksum (2 bytes)

	Whole data package size = MessageHeader->msgLen + 2(0x55AA)

	User Defined Data payload consist of:
		Number of data structures
		List of data structures ID
		List of data structures Values

*/
struct PACKED MessageHeader {
	uint16_t packageHeader;  // 0x55AA
	uint8_t  msgType;        // always 1 for INS data
	uint8_t  msgId;          // always 0x95
	uint16_t msgLen;         // payload + 6(msgType + msgId + msgLen + checksum). Not included packageHeader
};

struct PACKED SensorBias {
	int8_t gyroX;  // deg/s*0.5*1e5
	int8_t gyroY;  // deg/s*0.5*1e5
	int8_t gyroZ;  // deg/s*0.5*1e5
	int8_t accX;   // g*0.5*1e6
	int8_t accY;   // g*0.5*1e6
	int8_t accZ;   // g*0.5*1e6
	int8_t reserved;
};

struct PACKED GnssExtendedInfo {
	uint8_t fixType;
	uint8_t spoofingStatus;
};

struct PACKED GnssDop {
	uint16_t gdop;  // *1000
	uint16_t pdop;  // *1000
	uint16_t hdop;  // *1000
	uint16_t vdop;  // *1000
	uint16_t tdop;  // *1000
};

struct PACKED InsAccuracy {
	int32_t lat;       // m*1000
	int32_t lon;       // m*1000
	int32_t alt;       // m*1000
	int32_t eastVel;   // m/s*1000
	int32_t northVel;  // m/s*1000
	int32_t verVel;    // m/s*1000
};

struct PACKED ExtHorPos {
	int32_t  lat;         // deg*1.0e7
	int32_t  lon;         // deg*1.0e7
	uint16_t latStd;      // m*100
	uint16_t lonStd;      // m*100
	uint16_t posLatency;  // sec*1000
};

struct PACKED ExtAlt {
	int32_t  alt;     // m*1000, can be WGS84 or AMSL
	uint16_t altStd;  // m*100
};

struct PACKED ExtHeading {
	uint16_t heading;  // deg*100
	uint16_t std;      // deg*100
	uint16_t latency;  // sec*1000
};

struct PACKED ExtAmbientData {
	int16_t  airTemp;   // degC*10
	int32_t  alt;       // m*100, can be WGS84 or AMSL
	uint16_t absPress;  // Pa/2
};

struct PACKED ExtWindData {
	int16_t  nWindVel;  // kt*100
	int16_t  eWindVel;  // kt*100
	uint16_t nStdWind;  // kt*100
	uint16_t eStdWind;  // kt*100
};

union PACKED UDDMessageData {
	uint32_t  gpsTimeMs;  // ms since start of GPS week
	uint16_t  gpsWeek;
	vec3_32_t accelDataHr;  // g * 1e6
	vec3_32_t gyroDataHr;   // deg/s * 1e5
	struct PACKED {
		uint16_t pressurePa2;  // Pascals/2
		int32_t  baroAlt;      // meters*100, can be WGS84 or AMSL
	} baroData;
	vec3_16_t  magData;  // nT/10
	SensorBias sensorBias;
	struct PACKED {
		uint16_t yaw;    // deg*100
		int16_t  pitch;  // deg*100
		int16_t  roll;   // deg*100
	} orientationAngles;     // 321 euler order?
	vec3_32_t velocity;      // m/s * 100
	struct PACKED {
		int32_t lat;  // deg*1e7
		int32_t lon;  // deg*1e7
		int32_t alt;  // m*100, can be WGS84 or AMSL
	} position;
	uint16_t         unitStatus;  // set ILABS_UNIT_STATUS_*
	GnssExtendedInfo gnssExtendedInfo;
	struct PACKED {
		int32_t lat;  // deg*1e7
		int32_t lon;  // deg*1e7
		int32_t alt;  // m*100 can be WGS84 or AMSL
	} gnssPosition;
	struct PACKED {
		int32_t  horSpeed;         // m/s*100
		uint16_t trackOverGround;  // deg*100
		int32_t  verSpeed;         // m/s*100
	} gnssVelTrack;
	uint32_t       gnssPosTimestamp;  // ms
	uint8_t        gnssNewData;
	uint8_t        gnssJamStatus;
	int32_t        differentialPressure;  // mbar*1e4
	int16_t        trueAirspeed;          // m/s*100
	int16_t        calibratedAirspeed;    // m/s*100
	vec3_16_t      windSpeed;             // m/s*100
	uint16_t       airDataStatus;
	uint16_t       supplyVoltage;  // V*100
	int16_t        temperature;    // degC*10
	uint16_t       unitStatus2;
	GnssDop        gnssDop;
	uint8_t        insSolStatus;
	InsAccuracy    insAccuracy;
	uint8_t        usedSatCount;
	uint16_t       gnssVelLatency;  // ms
	uint8_t        gnssSolStatus;
	uint8_t        gnssPosVelType;
	uint16_t       newAidingData;
	uint16_t       newAidingData2;
	int16_t        externalSpeed;  // kt*100
	ExtHorPos      extHorPos;
	ExtAlt         extAlt;
	ExtHeading     extHeading;
	ExtAmbientData extAmbientAirData;
	ExtWindData    extWindData;
	uint8_t        magClbAccuracy;  // deg*10
};

struct GpsData {
	uint32_t    msTow;  // ms
	uint16_t    gpsWeek;
	double      latitude;         // deg
	double      longitude;        // deg
	float       altitude;         // m, can be WGS84 or AMSL
	float       horSpeed;         // m/s
	float       verSpeed;         // m/s
	float       trackOverGround;  // deg
	GnssDop     dop;
	uint8_t     newData;
	uint8_t     fixType;
	uint8_t     spoofingStatus;
	uint8_t     jamStatus;
	uint8_t     usedSatCount;
	uint16_t    velLatency;  // ms
	uint8_t     gnssSolStatus;
	uint8_t     gnssPosVelType;
};

struct InsData {
	float            yaw;        // deg
	float            pitch;      // deg
	float            roll;       // deg
	uint32_t         msTow;      // ms
	double           latitude;   // deg
	double           longitude;  // deg
	float            altitude;   // m, can be WGS84 or AMSL
	matrix::Vector3f velocity;   // NED, in m/s
	uint16_t         unitStatus;
	uint16_t         unitStatus2;
	uint8_t          solutionStatus;
	InsAccuracy      accuracy;
	float            baroAlt;             // m
	float            trueAirspeed;        // m/s
	float            calibratedAirspeed;  // m/s
	matrix::Vector3f windSpeed;           // NED, in m/s
	float            airspeedSf;
	uint16_t         airDataStatus;
	SensorBias       sensorBias;
	float            magClbAccuracy;  // deg
};

struct ExtData {
	uint16_t       newAidingData;
	uint16_t       newAidingData2;
	int16_t        speed;
	ExtHorPos      horPos;
	ExtAlt         altitudeData;
	ExtHeading     headingData;
	ExtAmbientData ambientAirData;
	ExtWindData    windData;
};

struct SensorsData {
	matrix::Vector3f accel;  // NED, in m/s^2
	matrix::Vector3f gyro;   // NED, in rad/s
	matrix::Vector3f mag;    // NED, in Gauss
	InsData          ins{};
	GpsData          gps{};
	ExtData          ext{};
	float            pressure;              // Pa
	float            differentialPressure;  // Pa
	float            temperature;           // degC
	float            supplyVoltage;         // V
};

struct AverageSensorsData {
	float            pressure;              // Pa
	float            temperature;           // degC
	uint8_t          count;                 // number of samples
};

// NOLINTEND(clang-analyzer-optin.performance.Padding, altera-struct-pack-align, misc-non-private-member-variables-in-classes)

}  // namespace InertialLabs
