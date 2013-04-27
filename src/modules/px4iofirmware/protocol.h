/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

/**
 * @file protocol.h
 *
 * PX4IO I2C interface protocol.
 *
 * Communication is performed via writes to and reads from 16-bit virtual
 * registers organised into pages of 255 registers each.
 *
 * The first two bytes of each write select a page and offset address
 * respectively. Subsequent reads and writes increment the offset within
 * the page. 
 *
 * Most pages are readable or writable but not both.
 *
 * Note that some pages may permit offset values greater than 255, which
 * can only be achieved by long writes. The offset does not wrap.
 *
 * Writes to unimplemented registers are ignored. Reads from unimplemented
 * registers return undefined values.
 *
 * As convention, values that would be floating point in other parts of 
 * the PX4 system are expressed as signed integer values scaled by 10000,
 * e.g. control values range from -10000..10000.  Use the REG_TO_SIGNED and
 * SIGNED_TO_REG macros to convert between register representation and
 * the signed version, and REG_TO_FLOAT/FLOAT_TO_REG to convert to float.
 *
 * Note that the implementation of readable pages prefers registers within
 * readable pages to be densely packed. Page numbers do not need to be
 * packed.
 */

#define PX4IO_CONTROL_CHANNELS			8
#define PX4IO_INPUT_CHANNELS			12
#define PX4IO_RELAY_CHANNELS			4

/* Per C, this is safe for all 2's complement systems */
#define REG_TO_SIGNED(_reg)	((int16_t)(_reg))
#define SIGNED_TO_REG(_signed)	((uint16_t)(_signed))

#define REG_TO_FLOAT(_reg)	((float)REG_TO_SIGNED(_reg) / 10000.0f)
#define FLOAT_TO_REG(_float)	SIGNED_TO_REG((int16_t)((_float) * 10000.0f))

/* static configuration page */
#define PX4IO_PAGE_CONFIG		0
#define PX4IO_P_CONFIG_PROTOCOL_VERSION		0	/* magic numbers TBD */
#define PX4IO_P_CONFIG_SOFTWARE_VERSION		1	/* magic numbers TBD */
#define PX4IO_P_CONFIG_BOOTLOADER_VERSION	2	/* get this how? */
#define PX4IO_P_CONFIG_MAX_TRANSFER		3	/* maximum I2C transfer size */
#define PX4IO_P_CONFIG_CONTROL_COUNT		4	/* hardcoded max control count supported */
#define PX4IO_P_CONFIG_ACTUATOR_COUNT		5	/* hardcoded max actuator output count */
#define PX4IO_P_CONFIG_RC_INPUT_COUNT		6	/* hardcoded max R/C input count supported */
#define PX4IO_P_CONFIG_ADC_INPUT_COUNT		7	/* hardcoded max ADC inputs */
#define PX4IO_P_CONFIG_RELAY_COUNT		8	/* harcoded # of relay outputs */

/* dynamic status page */
#define PX4IO_PAGE_STATUS		1
#define PX4IO_P_STATUS_FREEMEM			0
#define PX4IO_P_STATUS_CPULOAD			1

#define PX4IO_P_STATUS_FLAGS			2	 /* monitoring flags */
#define PX4IO_P_STATUS_FLAGS_ARMED		(1 << 0) /* arm-ok and locally armed */
#define PX4IO_P_STATUS_FLAGS_OVERRIDE		(1 << 1) /* in manual override */
#define PX4IO_P_STATUS_FLAGS_RC_OK		(1 << 2) /* RC input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_PPM		(1 << 3) /* PPM input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_DSM		(1 << 4) /* DSM input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_SBUS		(1 << 5) /* SBUS input is valid */
#define PX4IO_P_STATUS_FLAGS_FMU_OK		(1 << 6) /* controls from FMU are valid */
#define PX4IO_P_STATUS_FLAGS_RAW_PWM		(1 << 7) /* raw PWM from FMU is bypassing the mixer */
#define PX4IO_P_STATUS_FLAGS_MIXER_OK		(1 << 8) /* mixer is OK */
#define PX4IO_P_STATUS_FLAGS_ARM_SYNC		(1 << 9) /* the arming state between IO and FMU is in sync */
#define PX4IO_P_STATUS_FLAGS_INIT_OK		(1 << 10) /* initialisation of the IO completed without error */

#define PX4IO_P_STATUS_ALARMS			3	 /* alarm flags - alarms latch, write 1 to a bit to clear it */
#define PX4IO_P_STATUS_ALARMS_VBATT_LOW		(1 << 0) /* VBatt is very close to regulator dropout */
#define PX4IO_P_STATUS_ALARMS_TEMPERATURE	(1 << 1) /* board temperature is high */
#define PX4IO_P_STATUS_ALARMS_SERVO_CURRENT	(1 << 2) /* servo current limit was exceeded */
#define PX4IO_P_STATUS_ALARMS_ACC_CURRENT	(1 << 3) /* accessory current limit was exceeded */
#define PX4IO_P_STATUS_ALARMS_FMU_LOST		(1 << 4) /* timed out waiting for controls from FMU */
#define PX4IO_P_STATUS_ALARMS_RC_LOST		(1 << 5) /* timed out waiting for RC input */
#define PX4IO_P_STATUS_ALARMS_PWM_ERROR		(1 << 6) /* PWM configuration or output was bad */

#define PX4IO_P_STATUS_VBATT			4	/* battery voltage in mV */
#define PX4IO_P_STATUS_IBATT			5	/* battery current in cA */

/* array of post-mix actuator outputs, -10000..10000 */
#define PX4IO_PAGE_ACTUATORS		2		/* 0..CONFIG_ACTUATOR_COUNT-1 */

/* array of PWM servo output values, microseconds */
#define PX4IO_PAGE_SERVOS		3		/* 0..CONFIG_ACTUATOR_COUNT-1 */

/* array of raw RC input values, microseconds */
#define PX4IO_PAGE_RAW_RC_INPUT		4
#define PX4IO_P_RAW_RC_COUNT			0	/* number of valid channels */
#define PX4IO_P_RAW_RC_BASE			1	/* CONFIG_RC_INPUT_COUNT channels from here */

/* array of scaled RC input values, -10000..10000 */
#define PX4IO_PAGE_RC_INPUT		5
#define PX4IO_P_RC_VALID			0	/* bitmask of valid controls */
#define PX4IO_P_RC_BASE				1	/* CONFIG_RC_INPUT_COUNT controls from here */

/* array of raw ADC values */
#define PX4IO_PAGE_RAW_ADC_INPUT	6		/* 0..CONFIG_ADC_INPUT_COUNT-1 */

/* PWM servo information */
#define PX4IO_PAGE_PWM_INFO		7
#define PX4IO_RATE_MAP_BASE			0	/* 0..CONFIG_ACTUATOR_COUNT bitmaps of PWM rate groups */

/* setup page */
#define PX4IO_PAGE_SETUP			100
#define PX4IO_P_SETUP_FEATURES			0

#define PX4IO_P_SETUP_ARMING			1	 /* arming controls */
#define PX4IO_P_SETUP_ARMING_ARM_OK		(1 << 0) /* OK to arm */
#define PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK	(1 << 2) /* OK to switch to manual override via override RC channel */
#define PX4IO_P_SETUP_ARMING_VECTOR_FLIGHT_OK	(1 << 3) /* OK to perform position / vector control (= position lock) */
#define PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK	(1 << 4) /* OK to try in-air restart */

#define PX4IO_P_SETUP_PWM_RATES			2	/* bitmask, 0 = low rate, 1 = high rate */
#define PX4IO_P_SETUP_PWM_DEFAULTRATE		3	/* 'low' PWM frame output rate in Hz */
#define PX4IO_P_SETUP_PWM_ALTRATE		4	/* 'high' PWM frame output rate in Hz */
#define PX4IO_P_SETUP_RELAYS			5	/* bitmask of relay/switch outputs, 0 = off, 1 = on */
#define PX4IO_P_SETUP_VBATT_SCALE		6	/* battery voltage correction factor (float) */
#define PX4IO_P_SETUP_IBATT_SCALE		7	/* battery current scaling factor (float) */
#define PX4IO_P_SETUP_IBATT_BIAS		8	/* battery current bias value */
#define PX4IO_P_SETUP_SET_DEBUG			9	/* debug level for IO board */

/* autopilot control values, -10000..10000 */
#define PX4IO_PAGE_CONTROLS			101	/* 0..CONFIG_CONTROL_COUNT */

/* raw text load to the mixer parser - ignores offset */
#define PX4IO_PAGE_MIXERLOAD			102

/* R/C channel config */
#define PX4IO_PAGE_RC_CONFIG			103	/* R/C input configuration */
#define PX4IO_P_RC_CONFIG_MIN			0	/* lowest input value */
#define PX4IO_P_RC_CONFIG_CENTER		1	/* center input value */
#define PX4IO_P_RC_CONFIG_MAX			2	/* highest input value */
#define PX4IO_P_RC_CONFIG_DEADZONE		3	/* band around center that is ignored */
#define PX4IO_P_RC_CONFIG_ASSIGNMENT		4	/* mapped input value */
#define PX4IO_P_RC_CONFIG_OPTIONS		5	/* channel options bitmask */
#define PX4IO_P_RC_CONFIG_OPTIONS_ENABLED	(1 << 0)
#define PX4IO_P_RC_CONFIG_OPTIONS_REVERSE	(1 << 1)
#define PX4IO_P_RC_CONFIG_STRIDE		6	/* spacing between channel config data */

/* PWM output - overrides mixer */
#define PX4IO_PAGE_DIRECT_PWM			104	/* 0..CONFIG_ACTUATOR_COUNT-1 */

/* PWM failsafe values - zero disables the output */
#define PX4IO_PAGE_FAILSAFE_PWM			105	/* 0..CONFIG_ACTUATOR_COUNT-1 */

/**
 * As-needed mixer data upload.
 *
 * This message adds text to the mixer text buffer; the text
 * buffer is drained as the definitions are consumed.
 */
#pragma pack(push, 1)
struct px4io_mixdata {
	uint16_t	f2i_mixer_magic;
#define F2I_MIXER_MAGIC		0x6d74

	uint8_t		action;
#define F2I_MIXER_ACTION_RESET			0
#define F2I_MIXER_ACTION_APPEND			1

	char		text[0];	/* actual text size may vary */
};
#pragma pack(pop)

