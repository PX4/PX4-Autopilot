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
 * @author Julian Oes <joes@student.ethz.ch>
 */

#pragma once

#include <cstdint>
#include "../../definitions.h"

#ifndef GPS_READ_BUFFER_SIZE
#define GPS_READ_BUFFER_SIZE 128
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

/** Callback function for platform-specific stuff.
 * data1 and data2 depend on type and user is the custom user-supplied argument.
 * @return <0 on error, >=0 on success (depending on type)
 */
typedef int (*GPSCallbackPtr)(GPSCallbackType type, void *data1, int data2, void *user);


struct SurveyInStatus {
	uint32_t mean_accuracy;       /**< [mm] */
	uint32_t duration;            /**< [s] */
	uint8_t flags;                /**< bit 0: valid, bit 1: active */
};

// TODO: this number seems wrong
#define GPS_EPOCH_SECS ((time_t)1234567890ULL)

class GPSHelper
{
public:
	enum class OutputMode {
		GPS = 0,    ///< normal GPS output
		RTCM        ///< request RTCM output. This is used for (fixed position) base stations
	};

	GPSHelper(GPSCallbackPtr callback, void *callback_user);
	virtual ~GPSHelper();

	/**
	 * configure the device
	 * @param baud will be set to the baudrate (output parameter)
	 * @return 0 on success, <0 otherwise
	 */
	virtual int configure(unsigned &baud, OutputMode output_mode) = 0;

	/**
	 * receive & handle new data from the device
	 * @param timeout [ms]
	 * @return <0 on error, otherwise a bitset:
	 *         bit 0 set: got gps position update
	 *         bit 1 set: got satellite info update
	 */
	virtual int receive(unsigned timeout) = 0;

	float getPositionUpdateRate() { return _rate_lat_lon; }
	float getVelocityUpdateRate() { return _rate_vel; }
	void resetUpdateRates();
	void storeUpdateRates();

	/**
	 * Start or restart the survey-in procees. This is only used in RTCM ouput mode.
	 * It will be called automatically after configuring.
	 * @return 0 on success, <0 on error
	 */
	virtual int restartSurveyIn() { return 0; }


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
		*((int *)buf) = timeout;
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

	void setClock(timespec &t)
	{
		_callback(GPSCallbackType::setClock, &t, 0, _callback_user);
	}

	GPSCallbackPtr _callback;
	void *_callback_user;

	uint8_t _rate_count_lat_lon;
	uint8_t _rate_count_vel;

	float _rate_lat_lon = 0.0f;
	float _rate_vel = 0.0f;

	uint64_t _interval_rate_start;
};
