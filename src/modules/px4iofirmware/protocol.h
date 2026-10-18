/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

#include <inttypes.h>

/**
 * @file protocol.h
 *
 * PX4IO interface protocol.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 *
 * Communication is performed via writes to and reads from 16-bit virtual
 * registers organised into pages of 255 registers each.
 *
 * The first two bytes of each write select a page and offset address
 * respectively. Subsequent reads and writes increment the offset within
 * the page.
 *
 * Some pages are read- or write-only.
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
 * the signed version.
 *
 * Note that the implementation of readable pages prefers registers within
 * readable pages to be densely packed. Page numbers do not need to be
 * packed.
 *
 * Definitions marked [1] are only valid on PX4IOv1 boards. Likewise,
 * [2] denotes definitions specific to the PX4IOv2 board.
 */

/* Per C, this is safe for all 2's complement systems */
#define REG_TO_SIGNED(_reg)	((int16_t)(_reg))
#define SIGNED_TO_REG(_signed)	((uint16_t)(_signed))

#define REG_TO_BOOL(_reg) 	((bool)(_reg))

#define PX4IO_PROTOCOL_VERSION		5

/* maximum allowable sizes on this protocol version */
#define PX4IO_PROTOCOL_MAX_CONTROL_COUNT	8	/**< The protocol does not support more than set here, individual units might support less - see PX4IO_P_CONFIG_CONTROL_COUNT */

/* static configuration page */
#define PX4IO_PAGE_CONFIG			0
#define PX4IO_P_CONFIG_PROTOCOL_VERSION		0	/* PX4IO_PROTOCOL_VERSION */
#define PX4IO_P_CONFIG_HARDWARE_VERSION		1	/* magic numbers TBD */
#define PX4IO_P_CONFIG_BOOTLOADER_VERSION	2	/* get this how? */
#define PX4IO_P_CONFIG_MAX_TRANSFER		3	/* maximum I2C transfer size */
#define PX4IO_P_CONFIG_CONTROL_COUNT		4	/* hardcoded max control count supported */
#define PX4IO_P_CONFIG_ACTUATOR_COUNT		5	/* hardcoded max actuator output count */
#define PX4IO_P_CONFIG_RC_INPUT_COUNT		6	/* hardcoded max R/C input count supported */
#define PX4IO_P_CONFIG_ADC_INPUT_COUNT		7	/* hardcoded max ADC inputs */
#define PX4IO_MAX_TRANSFER_LEN			64

/* dynamic status page */
#define PX4IO_PAGE_STATUS			1
#define PX4IO_P_STATUS_FREEMEM			0
#define PX4IO_P_STATUS_CPULOAD			1

#define PX4IO_P_STATUS_FLAGS			2	 /* monitoring flags */
#define PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED	(1 << 0) /* arm-ok and locally armed */
#define PX4IO_P_STATUS_FLAGS_RC_OK		(1 << 1) /* RC input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_PPM		(1 << 2) /* PPM input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_DSM		(1 << 3) /* DSM input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_SBUS		(1 << 4) /* SBUS input is valid */
#define PX4IO_P_STATUS_FLAGS_FMU_OK		(1 << 5) /* controls from FMU are valid */
#define PX4IO_P_STATUS_FLAGS_RAW_PWM		(1 << 6) /* raw PWM from FMU */
#define PX4IO_P_STATUS_FLAGS_ARM_SYNC		(1 << 7) /* the arming state between IO and FMU is in sync */
#define PX4IO_P_STATUS_FLAGS_INIT_OK		(1 << 8) /* initialisation of the IO completed without error */
#define PX4IO_P_STATUS_FLAGS_FAILSAFE		(1 << 9) /* failsafe is active */
#define PX4IO_P_STATUS_FLAGS_SAFETY_OFF         (1 << 10) /* safety is off */
#define PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED	(1 << 11) /* FMU was initialized and OK once */
#define PX4IO_P_STATUS_FLAGS_RC_ST24		(1 << 12) /* ST24 input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_SUMD		(1 << 13) /* SUMD input is valid */
#define PX4IO_P_STATUS_FLAGS_SAFETY_BUTTON_EVENT	(1 << 14) /* px4io safety button was pressed for longer than 1 second */

#define PX4IO_P_STATUS_ALARMS			3	 /* alarm flags - alarms latch, write 1 to a bit to clear it */
#define PX4IO_P_STATUS_ALARMS_RC_LOST           (1 << 0) /* timed out waiting for RC input */
#define PX4IO_P_STATUS_ALARMS_PWM_ERROR         (1 << 1) /* PWM configuration or output was bad */

#define PX4IO_P_STATUS_VSERVO			6	/* [2] servo rail voltage in mV */
#define PX4IO_P_STATUS_VRSSI			7	/* [2] RSSI voltage */

/* array of PWM servo output values, microseconds */
#define PX4IO_PAGE_SERVOS			3	/* 0..CONFIG_ACTUATOR_COUNT-1 */

/* array of raw RC input values, microseconds */
#define PX4IO_PAGE_RAW_RC_INPUT			4
#define PX4IO_P_RAW_RC_COUNT			0	/* number of valid channels */
#define PX4IO_P_RAW_RC_FLAGS			1	/* RC detail status flags */
#define PX4IO_P_RAW_RC_FLAGS_FRAME_DROP		(1 << 0) /* single frame drop */
#define PX4IO_P_RAW_RC_FLAGS_FAILSAFE		(1 << 1) /* receiver is in failsafe mode */
#define PX4IO_P_RAW_RC_FLAGS_RC_DSM11		(1 << 2) /* DSM decoding is 11 bit mode */
#define PX4IO_P_RAW_RC_FLAGS_MAPPING_OK		(1 << 3) /* Channel mapping is ok */
#define PX4IO_P_RAW_RC_FLAGS_RC_OK		(1 << 4) /* RC reception ok */

#define PX4IO_P_RAW_RC_NRSSI			2	/* [2] Normalized RSSI value, 0: no reception, 255: perfect reception */
#define PX4IO_P_RAW_RC_DATA			3	/* [1] + [2] Details about the RC source (PPM frame length, Spektrum protocol type) */
#define PX4IO_P_RAW_FRAME_COUNT			4	/* Number of total received frames (wrapping counter) */
#define PX4IO_P_RAW_LOST_FRAME_COUNT		5	/* Number of total dropped frames (wrapping counter) */
#define PX4IO_P_RAW_RC_BASE			6	/* CONFIG_RC_INPUT_COUNT channels from here */

/* array of raw ADC values */
#define PX4IO_PAGE_RAW_ADC_INPUT		6	/* 0..CONFIG_ADC_INPUT_COUNT-1 */

/* PWM servo information */
#define PX4IO_PAGE_PWM_INFO			7
#define PX4IO_RATE_MAP_BASE			0	/* 0..CONFIG_ACTUATOR_COUNT bitmaps of PWM rate groups */

/* setup page */
#define PX4IO_PAGE_SETUP			50
#define PX4IO_P_SETUP_FEATURES			0
#define PX4IO_P_SETUP_FEATURES_SBUS1_OUT	(1 << 0) /**< enable S.Bus v1 output */
#define PX4IO_P_SETUP_FEATURES_SBUS2_OUT	(1 << 1) /**< enable S.Bus v2 output */
#define PX4IO_P_SETUP_FEATURES_ADC_RSSI		(1 << 2) /**< enable ADC RSSI parsing */

#define PX4IO_P_SETUP_ARMING			1	 /* arming controls */
#define PX4IO_P_SETUP_ARMING_IO_ARM_OK            (1 << 0) /* OK to arm the IO side */
#define PX4IO_P_SETUP_ARMING_FMU_ARMED            (1 << 1) /* FMU is already armed */
#define PX4IO_P_SETUP_ARMING_FMU_PREARMED         (1 << 2) /* FMU is already prearmed */
#define PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM      (1 << 3) /* use custom failsafe values */
#define PX4IO_P_SETUP_ARMING_LOCKDOWN             (1 << 4) /* If set, the system operates normally, but won't actuate any servos */
#define PX4IO_P_SETUP_ARMING_TERMINATION          (1 << 5) /* If set, the system will always output the failsafe values */
#define PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE (1 << 6) /* If set, the system will never return from a failsafe, but remain in failsafe once triggered. */

#define PX4IO_P_SETUP_PWM_RATES                 2	/* bitmask, 0 = low rate, 1 = high rate */
#define PX4IO_P_SETUP_PWM_DEFAULTRATE           3	/* 'low' PWM frame output rate in Hz */
#define PX4IO_P_SETUP_PWM_ALTRATE               4	/* 'high' PWM frame output rate in Hz */
#define PX4IO_P_SETUP_VSERVO_SCALE              5	/* hardware rev [2] servo voltage correction factor (float) */
#define PX4IO_P_SETUP_DSM                       6	/* DSM bind state */
enum {							/* DSM bind states */
	dsm_bind_power_down = 0,
	dsm_bind_power_up,
	dsm_bind_set_rx_out,
	dsm_bind_send_pulses,
	dsm_bind_reinit_uart
};
/* 8 */
#define PX4IO_P_SETUP_SET_DEBUG			9	/* debug level for IO board */

#define PX4IO_P_SETUP_REBOOT_BL			10	/* reboot IO into bootloader */
#define PX4IO_REBOOT_BL_MAGIC			14662	/* required argument for reboot (random) */

#define PX4IO_P_SETUP_CRC			11	/* get CRC of IO firmware */
/* storage space of 12 occupied by CRC */
#define PX4IO_P_SETUP_SAFETY_BUTTON_ACK	14	/**< ACK from FMU when it gets safety button pressed status */
#define PX4IO_P_SETUP_SAFETY_OFF		15	/**< FMU inform PX4IO about safety_off for LED indication*/
#define PX4IO_P_SETUP_SBUS_RATE			16	/**< frame rate of SBUS1 output in Hz */
#define PX4IO_P_SETUP_THERMAL			17	/**< thermal management */
#define PX4IO_P_SETUP_ENABLE_FLIGHTTERMINATION	18	/**< flight termination; false if the circuit breaker (CBRK_FLIGHTTERM) is set */
#define PX4IO_P_SETUP_PWM_RATE_GROUP0            19	/* Configure timer group 0 update rate in Hz */
#define PX4IO_P_SETUP_PWM_RATE_GROUP1            20	/* Configure timer group 1 update rate in Hz */
#define PX4IO_P_SETUP_PWM_RATE_GROUP2            21	/* Configure timer group 2 update rate in Hz */
#define PX4IO_P_SETUP_PWM_RATE_GROUP3            22	/* Configure timer group 3 update rate in Hz */

#define PX4IO_THERMAL_IGNORE			UINT16_MAX
#define PX4IO_THERMAL_OFF			0
#define PX4IO_THERMAL_FULL			10000

/* PWM output */
#define PX4IO_PAGE_DIRECT_PWM			54		/**< 0..CONFIG_ACTUATOR_COUNT-1 */

/* PWM failsafe values - zero disables the output */
#define PX4IO_PAGE_FAILSAFE_PWM			55		/**< 0..CONFIG_ACTUATOR_COUNT-1 */

/* Debug and test page - not used in normal operation */
#define PX4IO_PAGE_TEST				127
#define PX4IO_P_TEST_LED			0		/**< set the amber LED on/off */

/* PWM disarmed values that are active */
#define PX4IO_PAGE_DISARMED_PWM			109		/* 0..CONFIG_ACTUATOR_COUNT-1 */

/**
 * Serial protocol encapsulation.
 */
#define PKT_MAX_REGS	32 // by agreement w/FMU

#pragma pack(push, 1)
struct IOPacket {
	uint8_t 	count_code;
	uint8_t 	crc;
	uint8_t 	page;
	uint8_t 	offset;
	uint16_t	regs[PKT_MAX_REGS];
};
#pragma pack(pop)

#if (PX4IO_MAX_TRANSFER_LEN > PKT_MAX_REGS * 2)
#error The max transfer length of the IO protocol must not be larger than the IO packet size
#endif

#define PKT_CODE_READ		0x00	/* FMU->IO read transaction */
#define PKT_CODE_WRITE		0x40	/* FMU->IO write transaction */
#define PKT_CODE_SUCCESS	0x00	/* IO->FMU success reply */
#define PKT_CODE_CORRUPT	0x40	/* IO->FMU bad packet reply */
#define PKT_CODE_ERROR		0x80	/* IO->FMU register op error reply */

#define PKT_CODE_MASK		0xc0
#define PKT_COUNT_MASK		0x3f

#define PKT_COUNT(_p)	((_p).count_code & PKT_COUNT_MASK)
#define PKT_CODE(_p)	((_p).count_code & PKT_CODE_MASK)
#define PKT_SIZE(_p)	((size_t)((uint8_t *)&((_p).regs[PKT_COUNT(_p)]) - ((uint8_t *)&(_p))))

#ifdef __cplusplus
extern "C" {
#endif

extern const uint8_t crc8_tab[256];
uint8_t crc_packet(struct IOPacket *pkt);

#ifdef __cplusplus
}
#endif
