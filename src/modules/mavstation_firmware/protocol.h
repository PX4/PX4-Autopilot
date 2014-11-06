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

#ifndef __MAVSTATION_FIRMWARE_PROTOCOL_H__
#define __MAVSTATION_FIRMWARE_PROTOCOL_H__

/**
 * @file protocol.h
 *
 * Mavstation interface protocol
 * =============================
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
 *
 * Mavstation I2C interface notes
 * -------------------------
 *
 * Register read/write operations are mapped directly to PX4IO register
 * read/write operations.
 *
 */

#define PX4IO_CONTROL_CHANNELS			8
#define PX4IO_INPUT_CHANNELS			12
#define PX4IO_RELAY_CHANNELS			4

/* Per C, this is safe for all 2's complement systems */
#define REG_TO_SIGNED(_reg)	((int16_t)(_reg))
#define SIGNED_TO_REG(_signed)	((uint16_t)(_signed))

#define REG_TO_FLOAT(_reg)	((float)REG_TO_SIGNED(_reg) / 10000.0f)
#define FLOAT_TO_REG(_float)	SIGNED_TO_REG((int16_t)((_float) * 10000.0f))

#define PX4IO_PAGE_WRITE		(1<<7)

/* static configuration page */
#define PX4IO_PAGE_CONFIG		0
#define PX4IO_P_CONFIG_PROTOCOL_VERSION		0	/* magic numbers TBD */
#define PX4IO_P_CONFIG_SOFTWARE_VERSION		1	/* magic numbers TBD */
#define PX4IO_P_CONFIG_BOOTLOADER_VERSION	2	/* get this how? */
#define PX4IO_P_CONFIG_MAX_TRANSFER		3	/* maximum packet transfer size */

/* dynamic status page */
#define PX4IO_PAGE_STATUS		1
#define PX4IO_P_STATUS_FREEMEM			0
#define PX4IO_P_STATUS_CPULOAD			1

#define PX4IO_P_STATUS_FLAGS			2	 /* monitoring flags */
#define PX4IO_P_STATUS_FLAGS_INIT_OK		(1 << 10) /* initialisation of the IO completed without error */

#define PX4IO_P_STATUS_ALARMS			3	 /* alarm flags - alarms latch, write 1 to a bit to clear it */
#define PX4IO_P_STATUS_ALARMS_VBATT_LOW		(1 << 0) /* VBatt is very close to regulator dropout */
#define PX4IO_P_STATUS_ALARMS_TEMPERATURE	(1 << 1) /* board temperature is high */
#define PX4IO_P_STATUS_ALARMS_PWM_ERROR		(1 << 6) /* PWM configuration or output was bad */

#define PX4IO_P_STATUS_VBATT			4	/* battery voltage in mV */
#define PX4IO_P_STATUS_IBATT			5	/* battery current (raw ADC) */

/* array of PWM servo output values, microseconds */
#define PX4IO_PAGE_SERVOS		3		/* 0..CONFIG_ACTUATOR_COUNT-1 */

/* array of raw ADC values */
#define PX4IO_PAGE_RAW_ADC_INPUT	6		/* 0..CONFIG_ADC_INPUT_COUNT-1 */

/* PWM servo information */
#define PX4IO_PAGE_PWM_INFO		7
#define PX4IO_RATE_MAP_BASE			0	/* 0..CONFIG_ACTUATOR_COUNT bitmaps of PWM rate groups */

/* setup page */
#define PX4IO_PAGE_SETUP		64
#define PX4IO_P_SETUP_FEATURES			0

#define PX4IO_P_SETUP_PWM_RATES			2	/* bitmask, 0 = low rate, 1 = high rate */
#define PX4IO_P_SETUP_PWM_DEFAULTRATE		3	/* 'low' PWM frame output rate in Hz */
#define PX4IO_P_SETUP_PWM_ALTRATE		4	/* 'high' PWM frame output rate in Hz */
#define PX4IO_P_SETUP_RELAYS			5	/* bitmask of relay/switch outputs, 0 = off, 1 = on */
#define PX4IO_P_SETUP_VBATT_SCALE		6	/* battery voltage correction factor (float) */
#define PX4IO_P_SETUP_SET_DEBUG			9	/* debug level for IO board */

#endif // __MAVSTATION_FIRMWARE_PROTOCOL_H__
