/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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
 * @file grf_commands.h
 * @author Andrew Brahim
 *
 * Declarations of grf serial commands for the Lightware grf/b series
 */

#pragma once
#define GRF_MAX_PAYLOAD 256
#define GRF_CRC_FIELDS 2

enum GRF_SERIAL_CMD {
	GRF_PRODUCT_NAME = 0,
	GRF_HARDWARE_VERSION = 1,
	GRF_FIRMWARE_VERSION = 2,
	GRF_SERIAL_NUMBER = 3,
	GRF_TEXT_MESSAGE = 7,
	GRF_USER_DATA = 9,
	GRF_TOKEN = 10,
	GRF_SAVE_PARAMETERS = 12,
	GRF_RESET = 14,
	GRF_STAGE_FIRMWARE = 16,
	GRF_COMMIT_FIRMWARE = 17,
	GRF_DISTANCE_OUTPUT = 27,
	GRF_STREAM = 30,
	GRF_DISTANCE_DATA_CM = 44,
	GRF_DISTANCE_DATA_MM = 45,
	GRF_LASER_FIRING = 50,
	GRF_TEMPERATURE = 55,
	GRF_AUTO_EXPOSURE = 70,
	GRF_UPDATE_RATE = 74,
	GRF_LOST_SIGNAL_COUNT = 78,
	GRF_GPIO_MODE = 83,
	GRF_GPIO_ALARM = 84,
	GRF_MEDIAN_FILTER_EN = 86,
	GRF_MEDIAN_FILETER_S = 87,
	GRF_SMOOTH_FILTER_EN = 88,
	GRF_SMOOTH_FACTOR = 89,
	GRF_BAUD_RATE = 91,
	GRF_I2C_ADDRESS = 92,
	GRF_ROLL_AVG_EN = 93,
	GRF_ROLL_AVG_SIZE = 94,
	GRF_SLEEP_COMMAND = 98,
	GRF_ZERO_OFFSET = 114
};

// Store contents of rx'd frame
struct {
	const uint8_t data_fields = 2; // useful for breaking crc's into byte separated fields
	uint16_t data_len{0};   // last message payload length (1+ bytes in payload)
	uint8_t data[GRF_MAX_PAYLOAD];   // payload size limited by posix serial
	uint8_t msg_id{0};          // latest message's message id
	uint8_t flags_lo{0};      // flags low byte
	uint8_t flags_hi{0};     // flags high byte
	uint16_t crc[GRF_CRC_FIELDS] = {0, 0};
	uint8_t crc_lo{0};        // crc low byte
	uint8_t crc_hi{0};       // crc high byte
} rx_field;
