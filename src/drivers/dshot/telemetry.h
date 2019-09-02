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

	int init(const char *uart_device);

	void deinit();

	void setNumMotors(int num_motors) { _num_motors = num_motors; }
	int numMotors() const { return _num_motors; }

	/**
	 * Read telemetry from the UART (non-blocking) and handle timeouts.
	 * @return -1 if no update, >= 0 for the motor index. Use @latestESCData() to get the data.
	 */
	int update();

	/**
	 * Get the motor index for which telemetry should be requested.
	 * @return -1 if no request should be made, motor index otherwise
	 */
	int getRequestMotorIndex();

	const EscData &latestESCData() const { return _latest_data; }

private:
	static constexpr int ESC_FRAME_SIZE = 10;

	/**
	 * set the Baudrate
	 * @param baud
	 * @return 0 on success, <0 on error
	 */
	int setBaudrate(unsigned baud);

	void requestNextMotor();

	bool decodeByte(uint8_t byte);

	static inline uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed);
	static uint8_t crc8(const uint8_t *buf, uint8_t len);

	int _uart_fd{-1};
	int _num_motors{0};
	uint8_t _frame_buffer[ESC_FRAME_SIZE];
	int _frame_position{0};

	EscData _latest_data;

	int _current_motor_index_request{-1};
	hrt_abstime _current_request_start{0};
};
