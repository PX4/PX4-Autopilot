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

#include <drivers/drv_hrt.h>

#define DSHOT_DEBUG_COMMANDS    1  // Command handling debug output
#define DSHOT_DEBUG_TELEMETRY   0  // Telemetry parsing and processing

// Command debug macro
#if DSHOT_DEBUG_COMMANDS
#define DSHOT_CMD_DEBUG(fmt, ...) PX4_INFO("[CMD] " fmt, ##__VA_ARGS__)
#else
#define DSHOT_CMD_DEBUG(fmt, ...) do { } while(0)
#endif

// Telemetry debug macro
#if DSHOT_DEBUG_TELEMETRY
#define DSHOT_TELEM_DEBUG(fmt, ...) PX4_INFO("[TELEM] " fmt, ##__VA_ARGS__)
#else
#define DSHOT_TELEM_DEBUG(fmt, ...) do { } while(0)
#endif

enum class TelemetrySource {
	Serial = 0,
	BDShot = 1,
};

struct EscData {
	int motor_index;     // Motors 1-8
	bool online;		 // Motor communicating
	hrt_abstime time;	 // Sample time
	TelemetrySource source;

	int8_t temperature;  // [deg C]
	int16_t voltage;     // [0.01V]
	int16_t current;     // [0.01A]
	int16_t consumption; // [mAh]
	int16_t erpm;        // [100ERPM]
};

enum class TelemetryStatus {
	NotStarted = 0,
	NotReady = 1,
	Ready = 2,
	Timeout = 3,
	ParseError = 4,
};

