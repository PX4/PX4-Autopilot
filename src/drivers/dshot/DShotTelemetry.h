/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/Serial.hpp>
#include <drivers/drv_hrt.h>

class DShotTelemetry
{
public:
	struct EscData {
		hrt_abstime time;
		int8_t temperature;  ///< [deg C]
		int16_t voltage;     ///< [0.01V]
		int16_t current;     ///< [0.01A]
		int16_t consumption; ///< [mAh]
		int16_t erpm;        ///< [100ERPM]
	};

	~DShotTelemetry();

	int init(const char *uart_device, bool swap_rxtx);
	int update(int num_motors);
	int getNextMotorIndex();
	const EscData &latestESCData() const { return _latest_data; }
	void printStatus() const;

	bool requestInProgress() { return _current_request_start > 0; }

private:
	static constexpr int ESC_FRAME_SIZE = 10;

	void requestNextMotor(int num_motors);

	/**
	 * Decode a single byte from an ESC feedback frame
	 * @param byte
	 * @param successful_decoding set to true if checksum matches
	 * @return true if received the expected amount of bytes and the next motor can be requested
	 */
	bool decodeByte(uint8_t byte, bool &successful_decoding);

	static uint8_t crc8(const uint8_t *buf, uint8_t len);

	device::Serial _uart {};

	uint8_t _frame_buffer[ESC_FRAME_SIZE];
	int _frame_position{0};

	EscData _latest_data;

	int _current_motor_index_request{-1};
	hrt_abstime _current_request_start{0};

	// statistics
	int _num_timeouts{0};
	int _num_successful_responses{0};
	int _num_checksum_errors{0};
};
