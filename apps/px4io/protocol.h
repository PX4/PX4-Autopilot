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
 * e.g. control values range from -10000..10000.
 *
 * Note that the implementation of readable pages prefers registers within
 * readable pages to be densely packed. Page numbers do not need to be
 * packed.
 */

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
#define PX4IO_P_CONFIG_POWERSW_COUNT		9	/* harcoded # of switched power outputs */

/* dynamic status page */
#define PX4IO_PAGE_STATUS		1
#define PX4IO_P_STATUS_FREEMEM			0
#define PX4IO_P_STATUS_CPULOAD			1

#define PX4IO_P_STATUS_FLAGS			2	/* monitoring flags */
#define PX4IO_P_STATUS_FLAGS_ARMED		(1 << 0) /* arm-ok and locally armed */
#define PX4IO_P_STATUS_FLAGS_OVERRIDE		(1 << 1) /* in manual override */
#define PX4IO_P_STATUS_FLAGS_RC_PPM		(1 << 2) /* PPM input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_DSM		(1 << 3) /* DSM input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_SBUS		(1 << 4) /* SBUS input is valid */

#define PX4IO_P_STATUS_ALARMS			3	/* alarm flags - alarms latch, write 1 to a bit to clear it */
#define PX4IO_P_STATUS_ALARMS_VBATT_LOW		(1 << 0) /* VBatt is very close to regulator dropout */
#define PX4IO_P_STATUS_ALARMS_TEMPERATURE	(1 << 1) /* board temperature is high */
#define PX4IO_P_STATUS_ALARMS_SERVO_CURRENT	(1 << 2) /* servo current limit was exceeded */
#define PX4IO_P_STATUS_ALARMS_ACC_CURRENT	(1 << 3) /* accessory current limit was exceeded */
#define PX4IO_P_STATUS_ALARMS_AP_LOST		(1 << 4)

#define PX4IO_P_STATUS_VBATT			4	/* battery voltage in mV */
#define PX4IO_P_STATUS_TEMPERATURE		5	/* temperature in (units tbd) */

/* array of post-mix actuator outputs, -10000..10000 */
#define PX4IO_PAGE_ACTUATORS		2		/* 0..CONFIG_ACTUATOR_COUNT-1 */

/* array of PWM servo output values, microseconds */
#define PX4IO_PAGE_SERVOS		3		/* 0..CONFIG_ACTUATOR_COUNT-1 */

/* array of raw RC input values, microseconds */
#define PX4IO_PAGE_RAW_RC_INPUT		4		/* 0..CONFIG_RC_INPUT_COUNT-1 */

/* array of scaled RC input values, -10000..10000 */
#define PX4IO_PAGE_RAW_RC_INPUT		5		/* 0..CONFIG_RC_INPUT_COUNT-1 */

/* array of raw ADC values */
#define PX4IO_PAGE_RAW_ADC_INPUT	6		/* 0..CONFIG_ADC_INPUT_COUNT-1 */

/* setup page */
#define PX4IO_PAGE_SETUP		100
#define PX4IO_P_SETUP_ARMING			1	/* arming controls */
#define PX4IO_P_SETUP_ARMING_ARM_OK		(1 << 0) /* OK to arm */
#define PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE	(1 << 1) /* request local override */

#define PX4IO_P_SETUP_PWM_RATES			2	/* bitmask, 0 = low rate, 1 = high rate */

#define PX4IO_P_SETUP_PWM_LOWRATE		3	/* 'low' PWM frame output rate in Hz */
#define PX4IO_P_SETUP_PWM_HIGHRATE		4	/* 'high' PWM frame output rate in Hz */

#define PX4IO_P_SETUP_RELAYS			5	/* bitmask of relay outputs, 0 = off, 1 = on */
#define PX4IO_P_SETUP_POWERSW			6	/* bitmask of switched power outputs, 0 = off, 1 = on */

/* autopilot control values, -10000..10000 */
#define PX4IO_PAGE_CONTROLS		101		/* 0..STATUS_CONTROL_COUNT */

/* raw text load to the mixer parser - ignores offset */
#define PX4IO_PAGE_MIXERLOAD		102



/*
 * Old serial PX4FMU <-> PX4IO messaging protocol.
 *
 * This initial version of the protocol is very simple; each side transmits a
 * complete update with each frame.  This avoids the sending of many small
 * messages and the corresponding complexity involved.
 */


#define PX4IO_CONTROL_CHANNELS	8
#define PX4IO_INPUT_CHANNELS	12
#define PX4IO_RELAY_CHANNELS	4

#pragma pack(push, 1)

/**
 * Periodic command from FMU to IO.
 */
struct px4io_command {
	uint16_t	f2i_magic;
#define F2I_MAGIC		0x636d

	uint16_t	servo_rate;
	uint16_t	output_control[PX4IO_CONTROL_CHANNELS]; /**< PWM output rate in Hz */
	bool		relay_state[PX4IO_RELAY_CHANNELS];	/**< relay states as requested by FMU */
	bool		arm_ok;					/**< FMU allows full arming */
	bool		vector_flight_mode_ok;			/**< FMU aquired a valid position lock, ready for pos control */
	bool		manual_override_ok;			/**< if true, IO performs a direct manual override */
};

/**
 * Periodic report from IO to FMU
 */
struct px4io_report {
	uint16_t	i2f_magic;
#define I2F_MAGIC		0x7570

	uint16_t	rc_channel[PX4IO_INPUT_CHANNELS];
	bool		armed;
	uint8_t		channel_count;

	uint16_t	battery_mv;
	uint16_t	adc_in;
	uint8_t		overcurrent;
};

/**
 * As-needed config message from FMU to IO
 */
struct px4io_config {
	uint16_t	f2i_config_magic;
#define F2I_CONFIG_MAGIC	 0x6366

	uint8_t		rc_map[4];	/**< channel ordering of roll, pitch, yaw, throttle */
	uint16_t	rc_min[4];	/**< min value for each channel */
	uint16_t	rc_trim[4];	/**< trim value for each channel */
	uint16_t	rc_max[4];	/**< max value for each channel */
	int8_t		rc_rev[4];	/**< rev value for each channel */
	uint16_t	rc_dz[4];	/**< dz value for each channel */
};

/**
 * As-needed mixer data upload.
 *
 * This message adds text to the mixer text buffer; the text
 * buffer is drained as the definitions are consumed.
 */
struct px4io_mixdata {
	uint16_t	f2i_mixer_magic;
#define F2I_MIXER_MAGIC		0x6d74

	uint8_t		action;
#define F2I_MIXER_ACTION_RESET		0
#define F2I_MIXER_ACTION_APPEND		1

	char		text[0];	/* actual text size may vary */
};

/* maximum size is limited by the HX frame size */
#define F2I_MIXER_MAX_TEXT	(HX_STREAM_MAX_FRAME - sizeof(struct px4io_mixdata))

#pragma pack(pop)