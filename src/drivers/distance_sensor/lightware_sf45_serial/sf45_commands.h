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
 * @file sf45_commands.h
 * @author Andrew Brahim
 *
 * Declarations of sf45 serial commands for the Lightware sf45/b series
 */

#pragma once
#define SF45_MAX_PAYLOAD 256
#define SF45_CRC_FIELDS 2

enum SF_SERIAL_CMD {
	SF_PRODUCT_NAME = 0,
	SF_HARDWARE_VERSION = 1,
	SF_FIRMWARE_VERSION = 2,
	SF_SERIAL_NUMBER = 3,
	SF_TEXT_MESSAGE = 7,
	SF_USER_DATA = 9,
	SF_TOKEN = 10,
	SF_SAVE_PARAMETERS = 12,
	SF_RESET = 14,
	SF_STAGE_FIRMWARE = 16,
	SF_COMMIT_FIRMWARE = 17,
	SF_DISTANCE_OUTPUT = 27,
	SF_STREAM = 30,
	SF_DISTANCE_DATA_CM = 44,
	SF_DISTANCE_DATA_MM = 45,
	SF_LASER_FIRING = 50,
	SF_TEMPERATURE = 57,
	SF_UPDATE_RATE = 66,
	SF_NOISE = 74,
	SF_ZERO_OFFSET = 75,
	SF_LOST_SIGNAL_COUNTER = 76,
	SF_BAUD_RATE = 79,
	SF_I2C_ADDRESS = 80,
	SF_SCAN_SPEED = 85,
	SF_STEPPER_STATUS = 93,
	SF_SCAN_ON_STARTUP = 94,
	SF_SCAN_ENABLE = 96,
	SF_SCAN_POSITION = 97,
	SF_SCAN_LOW_ANGLE = 98,
	SF_HIGH_ANGLE = 99
};

// Store contents of rx'd frame
struct {
	const uint8_t data_fields = 2; // useful for breaking crc's into byte separated fields
	uint16_t data_len{0};   // last message payload length (1+ bytes in payload)
	uint8_t data[SF45_MAX_PAYLOAD];   // payload size limited by posix serial
	uint8_t msg_id{0};          // latest message's message id
	uint8_t flags_lo{0};      // flags low byte
	uint8_t flags_hi{0};     // flags high byte
	uint16_t crc[SF45_CRC_FIELDS] = {0, 0};
	uint8_t crc_lo{0};        // crc low byte
	uint8_t crc_hi{0};       // crc high byte
} rx_field;
