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

#include <px4_platform_common/defines.h>

#include <stdint.h>
#include <sys/ioctl.h>
#include <board_config.h>

#include "drv_orb_dev.h"

__BEGIN_DECLS

// https://brushlesswhoop.com/dshot-and-bidirectional-dshot/#special-commands
enum {
	DSHOT_CMD_MOTOR_STOP = 0,
	DSHOT_CMD_BEEP1 = 1,
	DSHOT_CMD_ESC_INFO = 6,
	DSHOT_CMD_SPIN_DIRECTION_1 = 7,
	DSHOT_CMD_SPIN_DIRECTION_2 = 8,
	DSHOT_CMD_3D_MODE_OFF = 9,
	DSHOT_CMD_3D_MODE_ON = 10,
	DSHOT_CMD_SAVE_SETTINGS = 12,
	DSHOT_EXTENDED_TELEMETRY_ENABLE = 13,
	DSHOT_CMD_ENTER_PROGRAMMING_MODE = 36,
	DSHOT_CMD_EXIT_PROGRAMMING_MODE = 37,
	DSHOT_CMD_MAX          = 47,     // >47 are throttle values
	DSHOT_CMD_MIN_THROTTLE = 48,
	DSHOT_CMD_MAX_THROTTLE = 2047
};

// Extended DShot Telemetry
enum {
	DSHOT_EDT_ERPM = 0x00,
	DSHOT_EDT_TEMPERATURE = 0x02, // C
	DSHOT_EDT_VOLTAGE = 0x04, // 0.25V per step
	DSHOT_EDT_CURRENT = 0x06, // A
	DSHOT_EDT_DEBUG1 = 0x08,
	DSHOT_EDT_DEBUG2 = 0x0A,
	DSHOT_EDT_DEBUG3 = 0x0C,
	DSHOT_EDT_STATE_EVENT = 0x0E,
};

struct BDShotTelemetry {
	int type;
	int32_t value;
};

/**
 * Intialise the Dshot outputs using the specified configuration.
 *
 * @param channel_mask		Bitmask of channels (LSB = channel 0) to enable.
 *				This allows some of the channels to remain configured
 *				as GPIOs or as another function. Already used channels/timers will not be configured as DShot
 * @param dshot_pwm_freq	Frequency of DSHOT signal. Usually DSHOT150, DSHOT300, or DSHOT600
 * @return <0 on error, the initialized channels mask.
 */
__EXPORT extern int up_dshot_init(uint32_t channel_mask, uint32_t bdshot_channel_mask, unsigned dshot_pwm_freq, bool edt_enable);

/**
 * Set Dshot motor data, used by up_dshot_motor_data_set() and up_dshot_motor_command() (internal method)
 */
__EXPORT extern void dshot_motor_data_set(uint8_t channel, uint16_t throttle, bool telemetry);

/**
 * Set the current dshot throttle value for a channel (motor).
 *
 * @param channel	The channel to set.
 * @param throttle	The output dshot throttle value in [0 = DSHOT_DISARM_VALUE, 1 = DSHOT_MIN_THROTTLE, 1999 = DSHOT_MAX_THROTTLE].
 * @param telemetry	If true, request telemetry from that motor
 */
static inline void up_dshot_motor_data_set(uint8_t channel, uint16_t throttle, bool telemetry)
{
	dshot_motor_data_set(channel, throttle + DSHOT_CMD_MIN_THROTTLE, telemetry);
}

/**
 * Send DShot command to a channel (motor).
 *
 * @param channel	The channel to set.
 * @param command	dshot_command_t
 * @param telemetry	If true, request telemetry from that motor
 */
static inline void up_dshot_motor_command(unsigned channel, uint16_t command, bool telemetry)
{
	dshot_motor_data_set(channel, command, telemetry);
}

/**
 * Trigger dshot data transfer.
 */
__EXPORT extern void up_dshot_trigger(void);

/**
 * Arm or disarm dshot outputs (This will enable/disable complete timer for safety purpose.).
 *
 * When disarmed, dshot output no pulse.
 *
 * @param armed		If true, outputs are armed; if false they
 *			are disarmed.
 */
__EXPORT extern int up_dshot_arm(bool armed);

/**
 * Print bidrectional dshot status
 */
__EXPORT extern void up_bdshot_status(void);

/**
 * Get how many bidirectional erpm channels are ready
 *
 * When we get the erpm round-robin style, we need to get
 * and publish the erpms less often.
 *
 * @return <0 on error, OK on succes
 */
__EXPORT extern int up_bdshot_num_channels_ready(void);

/**
 * Get the total number of errors for a channel
 * @param channel	Dshot channel
 * @return The total number of recorded errors
 */
__EXPORT extern int up_bdshot_num_errors(uint8_t channel);

/**
 * Get bidrectional dshot erpm for a channel
 * @param channel	Dshot channel
 * @param erpm		pointer to write the erpm value
 * @return <0 on error, OK on succes
 */
__EXPORT extern int up_bdshot_get_erpm(uint8_t channel, int *erpm);

/**
 * Get bidrectional dshot extended telemetry for a channel
 * @param channel	Dshot channel
 * @param type		The type of telemetry value to get
 * @param value		pointer to write the telemetry value
 * @return <0 on error, OK on succes
 */
__EXPORT extern int up_bdshot_get_extended_telemetry(uint8_t channel, int type, uint8_t *value);

/**
 * Get bidrectional dshot status for a channel
 * @param channel	Dshot channel
 * @param erpm		pointer to write the erpm value
 * @return <0 on error / not supported, 0 on offline, 1 on online
 */
__EXPORT extern int up_bdshot_channel_online(uint8_t channel);

__END_DECLS
