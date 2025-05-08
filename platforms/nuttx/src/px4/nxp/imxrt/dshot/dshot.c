/****************************************************************************
 *
 * Copyright (C) 2023 PX4 Development Team. All rights reserved.
 * Author: Peter van der Perk <peter.vanderperk@nxp.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/log.h>
#include <imxrt_flexio.h>
#include <hardware/imxrt_flexio.h>
#include <imxrt_periphclks.h>
#include <px4_arch/dshot.h>
#include <px4_arch/io_timer.h>
#include <drivers/drv_dshot.h>
#include <stdio.h>
#include "barriers.h"

#include "arm_internal.h"

#define FLEXIO_BASE			IMXRT_FLEXIO1_BASE
#define DSHOT_TIMERS			FLEXIO_SHIFTBUFNIS_COUNT
#define DSHOT_THROTTLE_POSITION		5u
#define DSHOT_TELEMETRY_POSITION	4u
#define NIBBLES_SIZE 			4u
#define DSHOT_NUMBER_OF_NIBBLES		3u

#if defined(IOMUX_PULL_UP_47K)
#define IOMUX_PULL_UP IOMUX_PULL_UP_47K
#endif

static const uint32_t gcr_decode[32] = {
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x9, 0xA, 0xB, 0x0, 0xD, 0xE, 0xF,
	0x0, 0x0, 0x2, 0x3, 0x0, 0x5, 0x6, 0x7,
	0x0, 0x0, 0x8, 0x1, 0x0, 0x4, 0xC, 0x0
};

uint32_t erpms[DSHOT_TIMERS];

typedef enum {
	DSHOT_START = 0,
	DSHOT_12BIT_FIFO,
	DSHOT_12BIT_TRANSFERRED,
	DSHOT_TRANSMIT_COMPLETE,
	BDSHOT_RECEIVE,
	BDSHOT_RECEIVE_COMPLETE,
} dshot_state;

typedef struct dshot_handler_t {
	bool			init;
	uint32_t 		data_seg1;
	uint32_t 		irq_data;
	dshot_state             state;
	bool			bdshot;
	uint32_t                raw_response;
	uint16_t                erpm;
	uint32_t		crc_error_cnt;
	uint32_t		frame_error_cnt;
	uint32_t		no_response_cnt;
	uint32_t		last_no_response_cnt;
} dshot_handler_t;

#define BDSHOT_OFFLINE_COUNT 400 // If there are no responses for 400 setpoints ESC is offline

static dshot_handler_t dshot_inst[DSHOT_TIMERS] = {};

static uint32_t dshot_tcmp;
static uint32_t bdshot_tcmp;
static uint32_t dshot_mask;
static uint32_t bdshot_recv_mask;
static uint32_t bdshot_parsed_recv_mask;

static inline uint32_t flexio_getreg32(uint32_t offset)
{
	return getreg32(FLEXIO_BASE + offset);
}

static inline void flexio_modifyreg32(unsigned int offset,
				      uint32_t clearbits,
				      uint32_t setbits)
{
	modifyreg32(FLEXIO_BASE + offset, clearbits, setbits);
}

static inline void flexio_putreg32(uint32_t value, uint32_t offset)
{
	putreg32(value, FLEXIO_BASE + offset);
}

static inline void enable_shifter_status_interrupts(uint32_t mask)
{
	flexio_modifyreg32(IMXRT_FLEXIO_SHIFTSIEN_OFFSET, 0, mask);
}

static inline void disable_shifter_status_interrupts(uint32_t mask)
{
	flexio_modifyreg32(IMXRT_FLEXIO_SHIFTSIEN_OFFSET, mask, 0);
}

static inline uint32_t get_shifter_status_flags(void)
{
	return flexio_getreg32(IMXRT_FLEXIO_SHIFTSTAT_OFFSET);
}

static inline void clear_shifter_status_flags(uint32_t mask)
{
	flexio_putreg32(mask, IMXRT_FLEXIO_SHIFTSTAT_OFFSET);
}

static inline void enable_timer_status_interrupts(uint32_t mask)
{
	flexio_modifyreg32(IMXRT_FLEXIO_TIMIEN_OFFSET, 0, mask);
}

static inline void disable_timer_status_interrupts(uint32_t mask)
{
	flexio_modifyreg32(IMXRT_FLEXIO_TIMIEN_OFFSET, mask, 0);
}

static inline uint32_t get_timer_status_flags(void)
{
	return flexio_getreg32(IMXRT_FLEXIO_TIMSTAT_OFFSET);
}

static inline void clear_timer_status_flags(uint32_t mask)
{
	flexio_putreg32(mask, IMXRT_FLEXIO_TIMSTAT_OFFSET);
}

static void flexio_dshot_output(uint32_t channel, uint32_t pin, uint32_t timcmp, bool inverted)
{
	/* Disable Shifter */
	flexio_putreg32(0, IMXRT_FLEXIO_SHIFTCTL0_OFFSET + channel * 0x4);

	/* No start bit, stop bit low */
	flexio_putreg32(FLEXIO_SHIFTCFG_INSRC(FLEXIO_SHIFTER_INPUT_FROM_PIN) |
			FLEXIO_SHIFTCFG_PWIDTH(0) |
			FLEXIO_SHIFTCFG_SSTOP(FLEXIO_SHIFTER_STOP_BIT_LOW) |
			FLEXIO_SHIFTCFG_SSTART(FLEXIO_SHIFTER_START_BIT_DISABLED_LOAD_DATA_ON_ENABLE),
			IMXRT_FLEXIO_SHIFTCFG0_OFFSET + channel * 0x4);

	/* Transmit mode, output to FXIO pin, inverted output for bdshot */
	flexio_putreg32(FLEXIO_SHIFTCTL_TIMSEL(channel) |
			FLEXIO_SHIFTCTL_TIMPOL(FLEXIO_SHIFTER_TIMER_POLARITY_ON_POSITIVE) |
			FLEXIO_SHIFTCTL_PINCFG(FLEXIO_PIN_CONFIG_OUTPUT) |
			FLEXIO_SHIFTCTL_PINSEL(pin) |
			FLEXIO_SHIFTCTL_PINPOL(inverted) |
			FLEXIO_SHIFTCTL_SMOD(FLEXIO_SHIFTER_MODE_TRANSMIT),
			IMXRT_FLEXIO_SHIFTCTL0_OFFSET + channel * 0x4);

	/* Start transmitting on trigger, disable on compare */
	flexio_putreg32(FLEXIO_TIMCFG_TIMOUT(FLEXIO_TIMER_OUTPUT_ONE_NOT_AFFECTED_BY_RESET) |
			FLEXIO_TIMCFG_TIMDEC(FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_SHIFT_TIMER_OUTPUT) |
			FLEXIO_TIMCFG_TIMRST(FLEXIO_TIMER_RESET_NEVER) |
			FLEXIO_TIMCFG_TIMDIS(FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE) |
			FLEXIO_TIMCFG_TIMENA(FLEXIO_TIMER_ENABLE_ON_TRIGGER_HIGH) |
			FLEXIO_TIMCFG_TSTOP(FLEXIO_TIMER_STOP_BIT_DISABLED) |
			FLEXIO_TIMCFG_TSTART(FLEXIO_TIMER_START_BIT_DISABLED),
			IMXRT_FLEXIO_TIMCFG0_OFFSET + channel * 0x4);

	flexio_putreg32(timcmp, IMXRT_FLEXIO_TIMCMP0_OFFSET + channel * 0x4);

	/* Baud mode, Trigger on shifter write */
	flexio_putreg32(FLEXIO_TIMCTL_TRGSEL((4 * channel) + 1) |
			FLEXIO_TIMCTL_TRGPOL(FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_LOW) |
			FLEXIO_TIMCTL_TRGSRC(FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL) |
			FLEXIO_TIMCTL_PINCFG(FLEXIO_PIN_CONFIG_OUTPUT_DISABLED) |
			FLEXIO_TIMCTL_PINSEL(0) |
			FLEXIO_TIMCTL_PINPOL(FLEXIO_PIN_ACTIVE_LOW) |
			FLEXIO_TIMCTL_TIMOD(FLEXIO_TIMER_MODE_DUAL8_BIT_BAUD_BIT),
			IMXRT_FLEXIO_TIMCTL0_OFFSET + channel * 0x4);

}

static int flexio_irq_handler(int irq, void *context, void *arg)
{
	uint32_t flags = get_shifter_status_flags();
	uint32_t channel;

	for (channel = 0; flags && channel < DSHOT_TIMERS; channel++) {
		if (flags & (1 << channel)) {
			disable_shifter_status_interrupts(1 << channel);

			if (dshot_inst[channel].state == DSHOT_START) {
				dshot_inst[channel].state = DSHOT_12BIT_FIFO;
				flexio_putreg32(dshot_inst[channel].irq_data, IMXRT_FLEXIO_SHIFTBUF0_OFFSET + channel * 0x4);

			} else if (dshot_inst[channel].state == BDSHOT_RECEIVE) {
				dshot_inst[channel].state = BDSHOT_RECEIVE_COMPLETE;
				dshot_inst[channel].raw_response = flexio_getreg32(IMXRT_FLEXIO_SHIFTBUFBIS0_OFFSET + channel * 0x4);

				bdshot_recv_mask |= (1 << channel);

				if (bdshot_recv_mask == dshot_mask) {
					// Received telemetry on all channels
					// Schedule workqueue?
				}
			}
		}
	}

	flags = get_timer_status_flags();

	for (channel = 0; flags; (channel = (channel + 1) % DSHOT_TIMERS)) {
		flags = get_timer_status_flags();

		if (flags & (1 << channel)) {
			clear_timer_status_flags(1 << channel);

			if (dshot_inst[channel].state == DSHOT_12BIT_FIFO) {
				dshot_inst[channel].state = DSHOT_12BIT_TRANSFERRED;

			} else if (!dshot_inst[channel].bdshot && dshot_inst[channel].state == DSHOT_12BIT_TRANSFERRED) {
				dshot_inst[channel].state = DSHOT_TRANSMIT_COMPLETE;

			} else if (dshot_inst[channel].bdshot && dshot_inst[channel].state == DSHOT_12BIT_TRANSFERRED) {
				disable_shifter_status_interrupts(1 << channel);
				dshot_inst[channel].state = BDSHOT_RECEIVE;

				/* Transmit done, disable timer and reconfigure to receive*/
				flexio_putreg32(0x0, IMXRT_FLEXIO_TIMCTL0_OFFSET + channel * 0x4);

				/* Input data from pin, no start/stop bit*/
				flexio_putreg32(FLEXIO_SHIFTCFG_INSRC(FLEXIO_SHIFTER_INPUT_FROM_PIN) |
						FLEXIO_SHIFTCFG_PWIDTH(0) |
						FLEXIO_SHIFTCFG_SSTOP(FLEXIO_SHIFTER_STOP_BIT_DISABLE) |
						FLEXIO_SHIFTCFG_SSTART(FLEXIO_SHIFTER_START_BIT_DISABLED_LOAD_DATA_ON_SHIFT),
						IMXRT_FLEXIO_SHIFTCFG0_OFFSET + channel * 0x4);

				/* Shifter receive mdoe, on FXIO pin input */
				flexio_putreg32(FLEXIO_SHIFTCTL_TIMSEL(channel) |
						FLEXIO_SHIFTCTL_TIMPOL(FLEXIO_SHIFTER_TIMER_POLARITY_ON_POSITIVE) |
						FLEXIO_SHIFTCTL_PINCFG(FLEXIO_PIN_CONFIG_OUTPUT_DISABLED) |
						FLEXIO_SHIFTCTL_PINSEL(timer_io_channels[channel].dshot.flexio_pin) |
						FLEXIO_SHIFTCTL_PINPOL(FLEXIO_PIN_ACTIVE_LOW) |
						FLEXIO_SHIFTCTL_SMOD(FLEXIO_SHIFTER_MODE_RECEIVE),
						IMXRT_FLEXIO_SHIFTCTL0_OFFSET + channel * 0x4);

				/* Make sure there no shifter flags high from transmission */
				clear_shifter_status_flags(1 << channel);

				/* Enable on pin transition, resychronize through reset on rising edge */
				flexio_putreg32(FLEXIO_TIMCFG_TIMOUT(FLEXIO_TIMER_OUTPUT_ONE_AFFECTED_BY_RESET) |
						FLEXIO_TIMCFG_TIMDEC(FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_SHIFT_TIMER_OUTPUT) |
						FLEXIO_TIMCFG_TIMRST(FLEXIO_TIMER_RESET_ON_TIMER_PIN_RISING_EDGE) |
						FLEXIO_TIMCFG_TIMDIS(FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE) |
						FLEXIO_TIMCFG_TIMENA(FLEXIO_TIMER_ENABLE_ON_TRIGGER_BOTH_EDGE) |
						FLEXIO_TIMCFG_TSTOP(FLEXIO_TIMER_STOP_BIT_ENABLE_ON_TIMER_DISABLE) |
						FLEXIO_TIMCFG_TSTART(FLEXIO_TIMER_START_BIT_ENABLED),
						IMXRT_FLEXIO_TIMCFG0_OFFSET + channel * 0x4);

				/* Enable on pin transition, resychronize through reset on rising edge */
				flexio_putreg32(bdshot_tcmp, IMXRT_FLEXIO_TIMCMP0_OFFSET + channel * 0x4);

				/* Trigger on FXIO pin transition, Baud mode */
				flexio_putreg32(FLEXIO_TIMCTL_TRGSEL(2 * timer_io_channels[channel].dshot.flexio_pin) |
						FLEXIO_TIMCTL_TRGPOL(FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_HIGH) |
						FLEXIO_TIMCTL_TRGSRC(FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL) |
						FLEXIO_TIMCTL_PINCFG(FLEXIO_PIN_CONFIG_OUTPUT_DISABLED) |
						FLEXIO_TIMCTL_PINSEL(0) |
						FLEXIO_TIMCTL_PINPOL(FLEXIO_PIN_ACTIVE_LOW) |
						FLEXIO_TIMCTL_TIMOD(FLEXIO_TIMER_MODE_DUAL8_BIT_BAUD_BIT),
						IMXRT_FLEXIO_TIMCTL0_OFFSET + channel * 0x4);

				/* Enable shifter interrupt for receiving data */
				enable_shifter_status_interrupts(1 << channel);
			}
		}

	}

	return OK;
}


int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq, bool enable_bidirectional_dshot)
{
	/* Calculate dshot timings based on dshot_pwm_freq */
	dshot_tcmp = 0x2F00 | (((BOARD_FLEXIO_PREQ / (dshot_pwm_freq * 3) / 2) - 1) & 0xFF);
	bdshot_tcmp = 0x2900 | (((BOARD_FLEXIO_PREQ / (dshot_pwm_freq * 5 / 4) / 2) - 3) & 0xFF);

	/* Clock FlexIO peripheral */
	imxrt_clockall_flexio1();

	/* Reset FlexIO peripheral */
	flexio_modifyreg32(IMXRT_FLEXIO_CTRL_OFFSET, 0,
			   FLEXIO_CTRL_SWRST_MASK);
	flexio_putreg32(0, IMXRT_FLEXIO_CTRL_OFFSET);

	/* Initialize FlexIO peripheral */
	flexio_modifyreg32(IMXRT_FLEXIO_CTRL_OFFSET,
			   (FLEXIO_CTRL_DOZEN_MASK |
			    FLEXIO_CTRL_DBGE_MASK |
			    FLEXIO_CTRL_FASTACC_MASK |
			    FLEXIO_CTRL_FLEXEN_MASK),
			   (FLEXIO_CTRL_DBGE(1) |
			    FLEXIO_CTRL_FASTACC(1) |
			    FLEXIO_CTRL_FLEXEN(0)));

	/* FlexIO IRQ handling */
	up_enable_irq(IMXRT_IRQ_FLEXIO1);
	irq_attach(IMXRT_IRQ_FLEXIO1, flexio_irq_handler, 0);

	dshot_mask = 0x0;

	for (unsigned channel = 0; (channel_mask != 0) && (channel < DSHOT_TIMERS); channel++) {
		if (channel_mask & (1 << channel)) {

			if (timer_io_channels[channel].dshot.pinmux == 0) { // board does not configure dshot on this pin
				continue;
			}

			imxrt_config_gpio(timer_io_channels[channel].dshot.pinmux | IOMUX_PULL_UP);

			dshot_inst[channel].bdshot = enable_bidirectional_dshot;

			flexio_dshot_output(channel, timer_io_channels[channel].dshot.flexio_pin, dshot_tcmp, dshot_inst[channel].bdshot);

			dshot_inst[channel].init = true;

			// Mask channel to be active on dshot
			dshot_mask |= (1 << channel);
		}
	}

	flexio_modifyreg32(IMXRT_FLEXIO_CTRL_OFFSET, 0,
			   FLEXIO_CTRL_FLEXEN_MASK);

	return channel_mask;
}

void up_bdshot_erpm(void)
{
	uint32_t value;
	uint32_t data;
	uint32_t csum_data;
	uint8_t exponent;
	uint16_t period;
	uint16_t erpm;

	bdshot_parsed_recv_mask = 0;

	// Decode each individual channel
	for (uint8_t channel = 0; (channel < DSHOT_TIMERS); channel++) {
		if (bdshot_recv_mask & (1 << channel)) {
			value = ~dshot_inst[channel].raw_response & 0xFFFFF;

			/* if lowest significant isn't 1 we've got a framing error */
			if (value & 0x1) {
				/* Decode RLL */
				value = (value ^ (value >> 1));

				/* Decode GCR */
				data = gcr_decode[value & 0x1fU];
				data |= gcr_decode[(value >> 5U) & 0x1fU] << 4U;
				data |= gcr_decode[(value >> 10U) & 0x1fU] << 8U;
				data |= gcr_decode[(value >> 15U) & 0x1fU] << 12U;

				/* Calculate checksum */
				csum_data = data;
				csum_data = csum_data ^ (csum_data >> 8U);
				csum_data = csum_data ^ (csum_data >> NIBBLES_SIZE);

				if ((csum_data & 0xFU) != 0xFU) {
					dshot_inst[channel].crc_error_cnt++;

				} else {
					data = (data >> 4) & 0xFFF;

					if (data == 0xFFF) {
						erpm = 0;

					} else {
						exponent = ((data >> 9U) & 0x7U); /* 3 bit: exponent */
						period = (data & 0x1ffU); /* 9 bit: period base */
						period = period << exponent; /* Period in usec */
						erpm = ((1000000U * 60U / 100U + period / 2U) / period);
					}

					dshot_inst[channel].erpm = erpm;
					bdshot_parsed_recv_mask |= (1 << channel);
					dshot_inst[channel].last_no_response_cnt = dshot_inst[channel].no_response_cnt;
				}

			} else {
				dshot_inst[channel].frame_error_cnt++;
			}
		}
	}
}


int up_bdshot_num_erpm_ready(void)
{
	int num_ready = 0;

	for (unsigned i = 0; i < DSHOT_TIMERS; ++i) {
		// We only check that data has been received, rather than if it's valid.
		// This ensures data is published even if one channel has bit errors.
		if (bdshot_recv_mask & (1 << i)) {
			++num_ready;
		}
	}

	return num_ready;
}


int up_bdshot_get_erpm(uint8_t channel, int *erpm)
{
	if (bdshot_parsed_recv_mask & (1 << channel)) {
		*erpm = (int)dshot_inst[channel].erpm;
		return 0;
	}

	return -1;
}

int up_bdshot_channel_status(uint8_t channel)
{
	if (channel < DSHOT_TIMERS) {
		return ((dshot_inst[channel].no_response_cnt - dshot_inst[channel].last_no_response_cnt) < BDSHOT_OFFLINE_COUNT);
	}

	return -1;
}

void up_bdshot_status(void)
{

	for (uint8_t channel = 0; (channel < DSHOT_TIMERS); channel++) {

		if (dshot_inst[channel].init) {
			PX4_INFO("Channel %i %s Last erpm %i value", channel, up_bdshot_channel_status(channel) ? "online" : "offline",
				 dshot_inst[channel].erpm);
			PX4_INFO("CRC errors Frame error No response");
			PX4_INFO("%10lu %11lu %11lu", dshot_inst[channel].crc_error_cnt, dshot_inst[channel].frame_error_cnt,
				 dshot_inst[channel].no_response_cnt);
		}
	}
}

void up_dshot_trigger(void)
{
	// Calc data now since we're not event driven
	if (bdshot_recv_mask != 0x0) {
		up_bdshot_erpm();
	}

	clear_timer_status_flags(0xFF);

	for (uint8_t channel = 0; (channel < DSHOT_TIMERS); channel++) {
		if (dshot_inst[channel].bdshot && (bdshot_recv_mask & (1 << channel)) == 0) {
			dshot_inst[channel].no_response_cnt++;
		}

		if (dshot_inst[channel].init && dshot_inst[channel].data_seg1 != 0) {
			flexio_putreg32(dshot_inst[channel].data_seg1, IMXRT_FLEXIO_SHIFTBUF0_OFFSET + channel * 0x4);
		}
	}

	bdshot_recv_mask = 0x0;

	clear_timer_status_flags(0xFF);
	enable_shifter_status_interrupts(0xFF);
	enable_timer_status_interrupts(0xFF);
}

/* Expand packet from 16 bits 48 to get T0H and T1H timing */
uint64_t dshot_expand_data(uint16_t packet)
{
	unsigned int mask;
	unsigned int index = 0;
	uint64_t expanded = 0x0;

	for (mask = 0x8000; mask != 0; mask >>= 1) {
		if (packet & mask) {
			expanded = expanded | ((uint64_t)0x3 << index);

		} else {
			expanded = expanded | ((uint64_t)0x1 << index);
		}

		index = index + 3;
	}

	return expanded;
}

/**
* bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit 	12		- dshot telemetry enable/disable
* bits 	13-16	- XOR checksum
**/
void dshot_motor_data_set(unsigned channel, uint16_t throttle, bool telemetry)
{
	if (channel < DSHOT_TIMERS && dshot_inst[channel].init) {
		uint16_t csum_data;
		uint16_t packet = 0;
		uint16_t checksum = 0;

		packet |= throttle << DSHOT_THROTTLE_POSITION;
		packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

		if (dshot_inst[channel].bdshot) {
			csum_data = ~packet;

		} else {
			csum_data = packet;
		}

		/* XOR checksum calculation */
		csum_data >>= NIBBLES_SIZE;

		for (unsigned i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
			checksum ^= (csum_data & 0x0F); // XOR data by nibbles
			csum_data >>= NIBBLES_SIZE;
		}

		packet |= (checksum & 0x0F);

		uint64_t dshot_expanded = dshot_expand_data(packet);

		dshot_inst[channel].data_seg1 = (uint32_t)(dshot_expanded & 0xFFFFFF);
		dshot_inst[channel].irq_data = (uint32_t)(dshot_expanded >> 24);
		dshot_inst[channel].state = DSHOT_START;

		if (dshot_inst[channel].bdshot) {

			flexio_putreg32(0x0, IMXRT_FLEXIO_TIMCTL0_OFFSET + channel * 0x4);
			disable_shifter_status_interrupts(1 << channel);

			flexio_dshot_output(channel, timer_io_channels[channel].dshot.flexio_pin, dshot_tcmp, dshot_inst[channel].bdshot);

			clear_timer_status_flags(0xFF);
		}
	}
}

int up_dshot_arm(bool armed)
{
	return io_timer_set_enable(armed, IOTimerChanMode_Dshot, IO_TIMER_ALL_MODES_CHANNELS);
}
