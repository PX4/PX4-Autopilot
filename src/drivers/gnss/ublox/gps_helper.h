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
 * @file gps_helper.h
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <cstdint>
#include <cstring>

#ifndef GPS_DEFINITIONS_HEADER
#define GPS_DEFINITIONS_HEADER "definitions.h"
#endif

#include GPS_DEFINITIONS_HEADER

#ifndef GPS_READ_BUFFER_SIZE
#define GPS_READ_BUFFER_SIZE 150 ///< buffer size for the read() call. Messages can be longer than that.
#endif

#ifndef M_PI_F
# define M_PI_F 3.14159265358979323846f
#endif

enum class GPSCallbackType {
	/**
	 * Read data from device. This is a blocking operation with a timeout.
	 * data1: points to a buffer to be written to. The first sizeof(int) bytes contain the
	 *        timeout in ms when calling the method.
	 * data2: buffer length in bytes. Less bytes than this can be read.
	 * return: num read bytes, 0 on timeout (the method can actually also return 0 before
	 *         the timeout happens).
	 */
	readDeviceData = 0,

	/**
	 * Write data to device
	 * data1: data to be written
	 * data2: number of bytes to write
	 * return: num written bytes
	 */
	writeDeviceData,

	/**
	 * set Baudrate
	 * data1: ignored
	 * data2: baudrate
	 * return: 0 on success
	 */
	setBaudrate,

	/**
	 * Got an RTCM message from the device.
	 * data1: pointer to the message
	 * data2: message length
	 * return: ignored
	 */
	gotRTCMMessage,

	/**
	 * Got a relative position message from the device.
	 * data1: pointer to the message
	 * data2: message length
	 * return: ignored
	 */
	gotRelativePositionMessage,

	/**
	 * message about current survey-in status
	 * data1: points to a SurveyInStatus struct
	 * data2: ignored
	 * return: ignored
	 */
	surveyInStatus,

	/**
	 * can be used to set the current clock accurately
	 * data1: pointer to a timespec struct
	 * data2: ignored
	 * return: ignored
	 */
	setClock,
};

enum class GPSRestartType {
	None,

	/**
	 * In hot start mode, the receiver was powered down only for a short time (4 hours or less),
	 * so that its ephemeris is still valid. Since the receiver doesn't need to download ephemeris
	 * again, this is the fastest startup method.
	 */
	Hot,

	/**
	 * In warm start mode, the receiver has approximate information for time, position, and coarse
	 * satellite position data (Almanac). In this mode, after power-up, the receiver normally needs
	 * to download ephemeris before it can calculate position and velocity data.
	 */
	Warm,

	/**
	 * In cold start mode, the receiver has no information from the last position at startup.
	 * Therefore, the receiver must search the full time and frequency space, and all possible
	 * satellite numbers. If a satellite signal is found, it is tracked to decode the ephemeris,
	 * whereas the other channels continue to search satellites. Once there is a sufficient number
	 * of satellites with valid ephemeris, the receiver can calculate position and velocity data.
	 */
	Cold
};

/** Callback function for platform-specific stuff.
 * data1 and data2 depend on type and user is the custom user-supplied argument.
 * @return <0 on error, >=0 on success (depending on type)
 */
typedef int (*GPSCallbackPtr)(GPSCallbackType type, void *data1, int data2, void *user);


struct SurveyInStatus {
	double latitude;              /**< NAN if unknown/not set [deg] */
	double longitude;             /**< NAN if unknown/not set [deg] */
	float altitude;               /**< NAN if unknown/not set [m] */
	uint32_t mean_accuracy;       /**< [mm] */
	uint32_t duration;            /**< [s] */
	uint8_t flags;                /**< bit 0: valid, bit 1: active */
};

// TODO: this number seems wrong
#define GPS_EPOCH_SECS ((time_t)1234567890ULL)

class GPSHelper
{
public:
	enum class OutputMode : uint8_t {
		GPS = 0,    ///< normal GPS output
		GPSAndRTCM, ///< normal GPS+RTCM output
		RTCM        ///< request RTCM output. This is used for (fixed position) base stations
	};

	enum class Interface : uint8_t {
		UART = 0,
		SPI
	};

	/**
	 * Bitmask for GPS_1_GNSS and GPS_2_GNSS
	 * No bits set should keep the receiver's default config
	 */
	enum class GNSSSystemsMask : int32_t {
		RECEIVER_DEFAULTS = 0,
		ENABLE_GPS =        1 << 0,
		ENABLE_SBAS =       1 << 1,
		ENABLE_GALILEO =    1 << 2,
		ENABLE_BEIDOU =     1 << 3,
		ENABLE_GLONASS =    1 << 4,
		ENABLE_NAVIC =      1 << 5
	};

	enum class InterfaceProtocolsMask : int32_t {
		ALL_DISABLED =        0,
		I2C_IN_PROT_UBX =     1 << 0,
		I2C_IN_PROT_NMEA =    1 << 1,
		I2C_IN_PROT_RTCM3X =  1 << 2,
		I2C_OUT_PROT_UBX =    1 << 3,
		I2C_OUT_PROT_NMEA =   1 << 4,
		I2C_OUT_PROT_RTCM3X = 1 << 5
	};

	struct GPSConfig {
		OutputMode output_mode;
		GNSSSystemsMask gnss_systems;
		InterfaceProtocolsMask interface_protocols;
		bool cfg_wipe;
	};


	GPSHelper(GPSCallbackPtr callback, void *callback_user);
	virtual ~GPSHelper() = default;

	/**
	 * configure the device
	 * @param baud Input and output parameter: if set to 0, the baudrate will be automatically detected and set to
	 *             the detected baudrate. If not 0, a fixed baudrate is used.
	 * @param config GPS Config
	 * @return 0 on success, <0 otherwise
	 */
	virtual int configure(unsigned &baud, const GPSConfig &config) = 0;

	/**
	 * receive & handle new data from the device
	 * @param timeout [ms]
	 * @return <0 on error, otherwise a bitset:
	 *         bit 0 set: got gps position update
	 *         bit 1 set: got satellite info update
	 */
	virtual int receive(unsigned timeout) = 0;

	/**
	 * Reset GPS device
	 * @param restart_type
	 * @return <0 failure
	 *         -1 not implemented
	 * 	    0 success
	 */
	virtual int reset(GPSRestartType restart_type)	{ (void)restart_type; return -1; }

	float getPositionUpdateRate() { return _rate_lat_lon; }
	float getVelocityUpdateRate() { return _rate_vel; }
	void resetUpdateRates();
	void storeUpdateRates();

	/**
	 * Allow a driver to disable RTCM injection
	 */
	virtual bool shouldInjectRTCM() { return true; }

protected:

	/**
	 * read from device
	 * @param buf: pointer to read buffer
	 * @param buf_length: size of read buffer
	 * @param timeout: timeout in ms
	 * @return: 0 for nothing read, or poll timed out
	 *	    < 0 for error
	 *	    > 0 number of bytes read
	 */
	int read(uint8_t *buf, int buf_length, int timeout)
	{
		memcpy(buf, &timeout, sizeof(timeout));
		return _callback(GPSCallbackType::readDeviceData, buf, buf_length, _callback_user);
	}

	/**
	 * write to the device
	 * @param buf
	 * @param buf_length
	 * @return num written bytes, -1 on error
	 */
	int write(const void *buf, int buf_length)
	{
		return _callback(GPSCallbackType::writeDeviceData, (void *)buf, buf_length, _callback_user);
	}

	/**
	 * set the Baudrate
	 * @param baudrate
	 * @return 0 on success, <0 otherwise
	 */
	int setBaudrate(int baudrate)
	{
		return _callback(GPSCallbackType::setBaudrate, nullptr, baudrate, _callback_user);
	}

	void surveyInStatus(SurveyInStatus &status)
	{
		_callback(GPSCallbackType::surveyInStatus, &status, 0, _callback_user);
	}

	/** got an RTCM message from the device */
	void gotRTCMMessage(uint8_t *buf, int buf_length)
	{
		_callback(GPSCallbackType::gotRTCMMessage, buf, buf_length, _callback_user);
	}

	/** got a relative position message from the device */
	void gotRelativePositionMessage(sensor_gnss_relative_s &gnss_relative)
	{
		_callback(GPSCallbackType::gotRelativePositionMessage, &gnss_relative, sizeof(sensor_gnss_relative_s), _callback_user);
	}

	void setClock(timespec &t)
	{
		_callback(GPSCallbackType::setClock, &t, 0, _callback_user);
	}

	/**
	 * Convert an ECEF (Earth Centered Earth Fixed) coordinate to LLA WGS84 (Lat, Lon, Alt).
	 * Ported from: https://stackoverflow.com/a/25428344
	 * @param ecef_x ECEF X-coordinate [m]
	 * @param ecef_y ECEF Y-coordinate [m]
	 * @param ecef_z ECEF Z-coordinate [m]
	 * @param latitude [deg]
	 * @param longitude [deg]
	 * @param altitude [m]
	 */
	static void ECEF2lla(double ecef_x, double ecef_y, double ecef_z, double &latitude, double &longitude, float &altitude);

	GPSCallbackPtr _callback{nullptr};
	void *_callback_user{};

	uint8_t _rate_count_lat_lon{};
	uint8_t _rate_count_vel{};

	float _rate_lat_lon{0.0f};
	float _rate_vel{0.0f};

	uint64_t _interval_rate_start{0};
};

inline bool operator&(GPSHelper::GNSSSystemsMask a, GPSHelper::GNSSSystemsMask b)
{
	return static_cast<int32_t>(a) & static_cast<int32_t>(b);
}

inline bool operator&(GPSHelper::InterfaceProtocolsMask a, GPSHelper::InterfaceProtocolsMask b)
{
	return static_cast<int32_t>(a) & static_cast<int32_t>(b);
}
