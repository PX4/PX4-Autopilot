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
#include <drivers/drv_hrt.h>
#include <nuttx/irq.h>
#include <string.h>

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

typedef enum {
	DSHOT_START = 0,
	DSHOT_12BIT_FIFO,
	DSHOT_12BIT_TRANSFERRED,
	DSHOT_TRANSMIT_COMPLETE,
	BDSHOT_RECEIVE,
	BDSHOT_RECEIVE_COMPLETE,
} dshot_state;

typedef struct {
	uint8_t value;
	bool ready;
} edt_sample_t;

typedef struct dshot_channel_t {
	bool			init;
	bool			bdshot;
	dshot_state		state;
	uint32_t		data_seg1;
	uint32_t		irq_data;
	uint32_t		raw_response;
	hrt_abstime		tx_started;	// 0 while nothing is in flight
	int32_t			erpm;
	bool			erpm_valid;
	edt_sample_t		edt_temp;
	edt_sample_t		edt_volt;
	edt_sample_t		edt_curr;
	uint32_t		crc_error_cnt;
	uint32_t		frame_error_cnt;
	uint32_t		no_response_cnt;
	uint16_t		consecutive_successes;
	uint16_t		consecutive_failures;
	bool			online;
	uint32_t		bdshot_tcmp;
	uint32_t		bdshot_training_mask;
	uint8_t			bdshot_training_count;
	uint8_t			bdshot_training_success;
	bool			bdshot_training_done;
	int8_t			bdshot_tcmp_offset;
} dshot_channel_t;

#define BDSHOT_OFFLINE_COUNT 200 // If there are no responses for 200 setpoints ESC is offline
#define BDSHOT_RETRAIN_COUNT (2 * BDSHOT_OFFLINE_COUNT)

#define BDSHOT_TCMP_MIN_OFFSET -16
#define BDSHOT_TCMP_MAX_OFFSET 15
#define BDSHOT_TCMP_TO_MASK(x) ((x) - BDSHOT_TCMP_MIN_OFFSET)
#define BDSHOT_TRAINING_TRIES 25
#define BDSHOT_TRAINING_SUCCESS 24

static volatile dshot_channel_t dshot_inst[DSHOT_TIMERS] = {};

static uint32_t dshot_tcmp;
static unsigned dshot_speed;
static uint32_t bdshot_ready_mask;
static hrt_abstime _bdshot_busy_us;
static bool _edt_enabled = false;
static bool _dshot_armed = false;

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

static inline void flexio_dshot_set_tcmp(uint32_t channel)
{
	dshot_inst[channel].bdshot_tcmp = 0x2900 | (((BOARD_FLEXIO_PREQ / (dshot_speed * 5 / 4) / 2) +
					  dshot_inst[channel].bdshot_tcmp_offset) & 0xFF);
}

// Shifter transmits on the FXIO pin. The timer runs in dual 8-bit baud mode and is triggered by the
// shifter status flag (active low = buffer full), so writing SHIFTBUF starts the frame.
static void flexio_dshot_output(uint32_t channel)
{
	uint32_t pin = timer_io_channels[channel].dshot.flexio_pin;
	bool inverted = dshot_inst[channel].bdshot;

	/* Disable timer, TIMCFG and TIMCMP may only be written while it is disabled */
	flexio_putreg32(0, IMXRT_FLEXIO_TIMCTL0_OFFSET + channel * 0x4);

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

	flexio_putreg32(dshot_tcmp, IMXRT_FLEXIO_TIMCMP0_OFFSET + channel * 0x4);

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

// Shifter receives from the FXIO pin. The timer is enabled by the first edge of the response and
// clocks 21 bits at the trained baud.
static void flexio_dshot_receive(uint32_t channel)
{
	uint32_t pin = timer_io_channels[channel].dshot.flexio_pin;

	/* Transmit done, disable timer and reconfigure to receive */
	flexio_putreg32(0, IMXRT_FLEXIO_TIMCTL0_OFFSET + channel * 0x4);

	/* Input data from pin, no start/stop bit */
	flexio_putreg32(FLEXIO_SHIFTCFG_INSRC(FLEXIO_SHIFTER_INPUT_FROM_PIN) |
			FLEXIO_SHIFTCFG_PWIDTH(0) |
			FLEXIO_SHIFTCFG_SSTOP(FLEXIO_SHIFTER_STOP_BIT_DISABLE) |
			FLEXIO_SHIFTCFG_SSTART(FLEXIO_SHIFTER_START_BIT_DISABLED_LOAD_DATA_ON_SHIFT),
			IMXRT_FLEXIO_SHIFTCFG0_OFFSET + channel * 0x4);

	/* Shifter receive mode, on FXIO pin input */
	flexio_putreg32(FLEXIO_SHIFTCTL_TIMSEL(channel) |
			FLEXIO_SHIFTCTL_TIMPOL(FLEXIO_SHIFTER_TIMER_POLARITY_ON_POSITIVE) |
			FLEXIO_SHIFTCTL_PINCFG(FLEXIO_PIN_CONFIG_OUTPUT_DISABLED) |
			FLEXIO_SHIFTCTL_PINSEL(pin) |
			FLEXIO_SHIFTCTL_PINPOL(FLEXIO_PIN_ACTIVE_LOW) |
			FLEXIO_SHIFTCTL_SMOD(FLEXIO_SHIFTER_MODE_RECEIVE),
			IMXRT_FLEXIO_SHIFTCTL0_OFFSET + channel * 0x4);

	/* Make sure there are no shifter flags high from transmission, they would read as a completed frame */
	clear_shifter_status_flags(1u << channel);

	/* Enable on pin transition, resynchronize through reset on rising edge */
	flexio_putreg32(FLEXIO_TIMCFG_TIMOUT(FLEXIO_TIMER_OUTPUT_ZERO_AFFECTED_BY_RESET) |
			FLEXIO_TIMCFG_TIMDEC(FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_SHIFT_TIMER_OUTPUT) |
			FLEXIO_TIMCFG_TIMRST(FLEXIO_TIMER_RESET_ON_TIMER_PIN_RISING_EDGE) |
			FLEXIO_TIMCFG_TIMDIS(FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE) |
			FLEXIO_TIMCFG_TIMENA(FLEXIO_TIMER_ENABLE_ON_TRIGGER_BOTH_EDGE) |
			FLEXIO_TIMCFG_TSTOP(FLEXIO_TIMER_STOP_BIT_ENABLE_ON_TIMER_DISABLE) |
			FLEXIO_TIMCFG_TSTART(FLEXIO_TIMER_START_BIT_ENABLED),
			IMXRT_FLEXIO_TIMCFG0_OFFSET + channel * 0x4);

	flexio_putreg32(dshot_inst[channel].bdshot_tcmp, IMXRT_FLEXIO_TIMCMP0_OFFSET + channel * 0x4);

	/* Trigger on FXIO pin transition, Baud mode. The timer pin is the channel pin so TIMRST
	 * resynchronizes the baud counter on every falling edge of the response; a baud-mode reset
	 * leaves the 21-bit count alone. */
	flexio_putreg32(FLEXIO_TIMCTL_TRGSEL(2 * pin) |
			FLEXIO_TIMCTL_TRGPOL(FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_HIGH) |
			FLEXIO_TIMCTL_TRGSRC(FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL) |
			FLEXIO_TIMCTL_PINCFG(FLEXIO_PIN_CONFIG_OUTPUT_DISABLED) |
			FLEXIO_TIMCTL_PINSEL(pin) |
			FLEXIO_TIMCTL_PINPOL(FLEXIO_PIN_ACTIVE_LOW) |
			FLEXIO_TIMCTL_TIMOD(FLEXIO_TIMER_MODE_DUAL8_BIT_BAUD_BIT),
			IMXRT_FLEXIO_TIMCTL0_OFFSET + channel * 0x4);
}

// The shifter has loaded a full 21-bit frame into its buffer. Reading it clears the status flag.
static void bdshot_latch_response(uint32_t channel)
{
	dshot_inst[channel].raw_response = flexio_getreg32(IMXRT_FLEXIO_SHIFTBUFBIS0_OFFSET + channel * 0x4);
	dshot_inst[channel].state = BDSHOT_RECEIVE_COMPLETE;
}

static int flexio_irq_handler(int irq, void *context, void *arg)
{
	// A status flag whose interrupt is masked belongs to a phase that already completed
	uint32_t pending = get_shifter_status_flags() & flexio_getreg32(IMXRT_FLEXIO_SHIFTSIEN_OFFSET);

	for (uint32_t channel = 0; pending != 0 && channel < DSHOT_TIMERS; channel++) {
		uint32_t bit = 1u << channel;

		if ((pending & bit) == 0) {
			continue;
		}

		pending &= ~bit;
		volatile dshot_channel_t *ch = &dshot_inst[channel];

		// One event per phase: the first buffer load while transmitting, the frame while receiving
		disable_shifter_status_interrupts(bit);

		if (ch->state == DSHOT_START) {
			ch->state = DSHOT_12BIT_FIFO;
			flexio_putreg32(ch->irq_data, IMXRT_FLEXIO_SHIFTBUF0_OFFSET + channel * 0x4);

		} else if (ch->state == BDSHOT_RECEIVE) {
			bdshot_latch_response(channel);
		}
	}

	pending = get_timer_status_flags() & flexio_getreg32(IMXRT_FLEXIO_TIMIEN_OFFSET);

	for (uint32_t channel = 0; pending != 0 && channel < DSHOT_TIMERS; channel++) {
		uint32_t bit = 1u << channel;

		if ((pending & bit) == 0) {
			continue;
		}

		pending &= ~bit;
		clear_timer_status_flags(bit);
		volatile dshot_channel_t *ch = &dshot_inst[channel];

		if (ch->state == DSHOT_12BIT_FIFO) {
			ch->state = DSHOT_12BIT_TRANSFERRED;

		} else if (ch->state == DSHOT_12BIT_TRANSFERRED) {
			if (ch->bdshot) {
				// The frame is out. Only the shifter flag matters until the next transmit.
				disable_timer_status_interrupts(bit);
				ch->state = BDSHOT_RECEIVE;
				flexio_dshot_receive(channel);

				/* Enable shifter interrupt for receiving data */
				enable_shifter_status_interrupts(bit);

			} else {
				ch->state = DSHOT_TRANSMIT_COMPLETE;
			}
		}
	}

	return OK;
}

static inline bool decode_gcr_payload(uint32_t value, uint16_t *payload)
{
	uint32_t data;
	uint32_t csum_data;

	/* if lowest significant bit isn't 1 we've got a framing error */
	if ((value & 0x1) == 0) {
		return false;
	}

	/* Decode RLL */
	value = value ^ (value >> 1);

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
		return false;
	}

	*payload = (data >> 4) & 0xFFF;
	return true;
}

static void decode_dshot_telemetry(uint32_t payload, struct BDShotTelemetry *packet)
{
	// MSB of the 9-bit mantissa clear means extended telemetry, not eRPM
	uint32_t mantissa = payload & 0x01FF;
	bool is_telemetry = !(mantissa & 0x0100);

	if (_edt_enabled && is_telemetry) {
		packet->type = (payload & 0x0F00) >> 8;
		packet->value = payload & 0x00FF;

	} else {
		uint8_t exponent = ((payload >> 9) & 0x7); /* 3 bit: exponent */
		uint16_t period = (payload & 0x1FF); /* 9 bit: period base */
		period = period << exponent; /* Period in usec */

		packet->type = DSHOT_EDT_ERPM;

		if (period == 65408 || period == 0) {
			packet->value = 0;

		} else {
			packet->value = (1000000U * 60U / 100U + period / 2U) / period;
		}
	}
}

static void bdshot_note_success(volatile dshot_channel_t *ch)
{
	ch->consecutive_failures = 0;

	if (ch->consecutive_successes < BDSHOT_OFFLINE_COUNT) {
		ch->consecutive_successes++;
	}

	if (ch->consecutive_successes >= BDSHOT_OFFLINE_COUNT) {
		ch->online = true;
	}
}

static void bdshot_restart_training(volatile dshot_channel_t *ch, uint32_t channel)
{
	ch->bdshot_training_done = false;
	ch->bdshot_training_mask = 0;
	ch->bdshot_training_count = 0;
	ch->bdshot_training_success = 0;
	ch->bdshot_tcmp_offset = BDSHOT_TCMP_MIN_OFFSET;
	ch->consecutive_successes = 0;
	ch->consecutive_failures = 0;
	ch->online = false;
	flexio_dshot_set_tcmp(channel);
}

// A missing response means the ESC is gone, not that the baud is wrong: it takes the channel offline
// but never restarts the sweep. Only frames that arrive and fail to decode can, after a second period.
static void bdshot_note_failure(volatile dshot_channel_t *ch, uint32_t channel, bool decoded_wrong)
{
	if (!ch->bdshot_training_done) {
		return;
	}

	ch->consecutive_successes = 0;
	uint16_t limit = decoded_wrong ? BDSHOT_RETRAIN_COUNT : BDSHOT_OFFLINE_COUNT;

	if (ch->consecutive_failures < limit) {
		ch->consecutive_failures++;
	}

	if (ch->consecutive_failures >= BDSHOT_OFFLINE_COUNT) {
		ch->online = false;
	}

	if (decoded_wrong && ch->consecutive_failures >= BDSHOT_RETRAIN_COUNT) {
		bdshot_restart_training(ch, channel);
	}
}

// Sweep the RX baud offset and settle on the centre of the range that decodes cleanly
static void bdshot_train(volatile dshot_channel_t *ch, uint32_t channel, uint32_t value)
{
	uint16_t payload;

	if (decode_gcr_payload(value, &payload)) {
		// Count successful responses
		ch->bdshot_training_success++;

	} else if ((value & 0x1) == 0) {
		// Invalidate frame error immediately
		ch->bdshot_training_count = BDSHOT_TRAINING_TRIES - 1;
	}

	// Keep count and check if a training round finished
	ch->bdshot_training_count++;

	if (ch->bdshot_training_count < BDSHOT_TRAINING_TRIES) {
		return;
	}

	if (ch->bdshot_training_success >= BDSHOT_TRAINING_SUCCESS) {
		ch->bdshot_training_mask |= (1u << BDSHOT_TCMP_TO_MASK(ch->bdshot_tcmp_offset));
	}

	ch->bdshot_training_count = 0;
	ch->bdshot_training_success = 0;
	ch->bdshot_tcmp_offset++;

	if (ch->bdshot_tcmp_offset > BDSHOT_TCMP_MAX_OFFSET) {
		if (ch->bdshot_training_mask == 0) {
			// No candidates retry
			ch->bdshot_tcmp_offset = BDSHOT_TCMP_MIN_OFFSET;

		} else {
			// Training done, use mask to find best offset
			int low  = __builtin_ctz(ch->bdshot_training_mask);
			int high = 31 - __builtin_clz(ch->bdshot_training_mask);
			ch->bdshot_tcmp_offset = ((low + high) / 2) + BDSHOT_TCMP_MIN_OFFSET;
			ch->bdshot_training_done = true;
			ch->consecutive_failures = 0;
			ch->consecutive_successes = BDSHOT_OFFLINE_COUNT;
			ch->online = true;
		}
	}

	// Update TCMP
	flexio_dshot_set_tcmp(channel);
}

// Called once per transmit cycle for every capturing channel, response or not
static void bdshot_process_response(uint32_t channel)
{
	volatile dshot_channel_t *ch = &dshot_inst[channel];

	// DShot.cpp waits until every BDShot channel is ready, so a silent ESC must not stall the others
	bdshot_ready_mask |= (1u << channel);

	if (ch->state != BDSHOT_RECEIVE_COMPLETE) {
		ch->no_response_cnt++;
		bdshot_note_failure(ch, channel, false);
		return;
	}

	uint32_t value = ~ch->raw_response & 0xFFFFF;

	// BDSHOT ESC hardware varies and timings differ between units.
	// Run training to estimate the correct baudrate to lock onto.
	if (!ch->bdshot_training_done) {
		bdshot_train(ch, channel, value);
		return;
	}

	uint16_t payload = 0;

	if (!decode_gcr_payload(value, &payload)) {
		if ((value & 0x1) == 0) {
			ch->frame_error_cnt++;

		} else {
			ch->crc_error_cnt++;
		}

		bdshot_note_failure(ch, channel, true);
		return;
	}

	struct BDShotTelemetry packet = {};

	decode_dshot_telemetry(payload, &packet);

	switch (packet.type) {
	case DSHOT_EDT_ERPM:
		ch->erpm = packet.value;
		ch->erpm_valid = true;
		break;

	case DSHOT_EDT_TEMPERATURE:
		ch->edt_temp.value = packet.value;
		ch->edt_temp.ready = true;
		break;

	case DSHOT_EDT_VOLTAGE:
		ch->edt_volt.value = packet.value;
		ch->edt_volt.ready = true;
		break;

	case DSHOT_EDT_CURRENT:
		ch->edt_curr.value = packet.value;
		ch->edt_curr.ready = true;
		break;

	default:
		break;
	}

	bdshot_note_success(ch);
}

int up_dshot_init(uint32_t channel_mask, uint32_t bdshot_channel_mask, unsigned dshot_pwm_freq, bool edt_enable)
{
	/* Clock FlexIO peripheral */
	imxrt_clockall_flexio1();

	/* Reset FlexIO peripheral, before the driver state so a stale IRQ has nothing to service */
	flexio_modifyreg32(IMXRT_FLEXIO_CTRL_OFFSET, 0, FLEXIO_CTRL_SWRST_MASK);
	flexio_putreg32(0, IMXRT_FLEXIO_CTRL_OFFSET);

	memset((void *)dshot_inst, 0, sizeof(dshot_inst));
	bdshot_ready_mask = 0;
	_dshot_armed = false;
	_edt_enabled = edt_enable;

	/* Calculate dshot timings based on dshot_pwm_freq */
	dshot_speed = dshot_pwm_freq;
	dshot_tcmp = 0x2F00 | (((BOARD_FLEXIO_PREQ / (dshot_pwm_freq * 3) / 2) - 1) & 0xFF);

	// 16-bit frame, ESC turnaround, 21-bit GCR response at 5/4 rate, margin for IRQ latency
	_bdshot_busy_us = (16u * 1000000u) / dshot_pwm_freq + 30u + (21u * 4u * 1000000u) / (5u * dshot_pwm_freq) + 50u;

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
	irq_attach(IMXRT_IRQ_FLEXIO1, flexio_irq_handler, 0);
	up_enable_irq(IMXRT_IRQ_FLEXIO1);

	uint32_t dshot_mask = 0;

	for (unsigned channel = 0; channel < MAX_TIMER_IO_CHANNELS; channel++) {
		uint32_t bit = 1u << channel;

		if (((channel_mask | bdshot_channel_mask) & bit) == 0) {
			continue;
		}

		// board does not configure dshot on this pin
		if (channel >= DSHOT_TIMERS || timer_io_channels[channel].dshot.pinmux == 0) {
			// Cannot capture, so it must never hold up DShot.cpp's all-channels-ready wait
			bdshot_ready_mask |= bit & bdshot_channel_mask;
			PX4_WARN("no FlexIO DShot on output %u", channel);
			continue;
		}

		volatile dshot_channel_t *ch = &dshot_inst[channel];

		imxrt_config_gpio(timer_io_channels[channel].dshot.pinmux | IOMUX_PULL_UP);

		ch->bdshot = (bdshot_channel_mask & bit) != 0;

		if (ch->bdshot) {
			ch->bdshot_tcmp_offset = BDSHOT_TCMP_MIN_OFFSET;
			flexio_dshot_set_tcmp(channel);
		}

		flexio_dshot_output(channel);
		ch->init = true;

		// Mask channel to be active on dshot
		dshot_mask |= bit;
	}

	flexio_modifyreg32(IMXRT_FLEXIO_CTRL_OFFSET, 0, FLEXIO_CTRL_FLEXEN_MASK);

	return dshot_mask;
}

uint16_t up_bdshot_get_ready_mask(void)
{
	return (uint16_t)bdshot_ready_mask;
}

int up_bdshot_num_errors(uint8_t channel)
{
	if (channel >= DSHOT_TIMERS) {
		return 0;
	}

	return dshot_inst[channel].crc_error_cnt + dshot_inst[channel].frame_error_cnt + dshot_inst[channel].no_response_cnt;
}

int up_bdshot_get_erpm(uint8_t channel, int *erpm)
{
	// Channels without capture keep their permanent ready bit, so return before clearing it
	if (channel >= DSHOT_TIMERS || !dshot_inst[channel].bdshot) {
		return -1;
	}

	int status = -1;

	if (dshot_inst[channel].erpm_valid) {
		*erpm = dshot_inst[channel].erpm;
		status = 0;
	}

	bdshot_ready_mask &= ~(1u << channel);
	return status;
}

int up_bdshot_get_extended_telemetry(uint8_t channel, int type, uint8_t *value)
{
	if (channel >= DSHOT_TIMERS) {
		return -1;
	}

	volatile edt_sample_t *sample = NULL;

	switch (type) {
	case DSHOT_EDT_TEMPERATURE:
		sample = &dshot_inst[channel].edt_temp;
		break;

	case DSHOT_EDT_VOLTAGE:
		sample = &dshot_inst[channel].edt_volt;
		break;

	case DSHOT_EDT_CURRENT:
		sample = &dshot_inst[channel].edt_curr;
		break;

	default:
		return -1;
	}

	if (!sample->ready) {
		return -1;
	}

	*value = sample->value;
	sample->ready = false;
	return 0;
}

int up_bdshot_channel_capture_supported(uint8_t channel)
{
	if (channel >= DSHOT_TIMERS) {
		return 0;
	}

	return dshot_inst[channel].init && dshot_inst[channel].bdshot;
}

int up_bdshot_channel_online(uint8_t channel)
{
	if (channel >= DSHOT_TIMERS) {
		return 0;
	}

	return dshot_inst[channel].online;
}

void up_bdshot_status(void)
{
	for (uint8_t channel = 0; channel < DSHOT_TIMERS; channel++) {
		volatile dshot_channel_t *ch = &dshot_inst[channel];

		if (!ch->init) {
			continue;
		}

		PX4_INFO("Channel %i %s Last erpm %li value", channel, ch->online ? "online" : "offline", ch->erpm);

		if (_edt_enabled) {
			PX4_INFO("EDT Temp %u C  Volt %.2f V  Curr %u A",
				 ch->edt_temp.value, (double)ch->edt_volt.value * 0.25, ch->edt_curr.value);
		}

		PX4_INFO("BDSHOT Training done: %s TCMP offset: %d mask 0x%08lx", ch->bdshot_training_done ? "YES" : "NO",
			 ch->bdshot_tcmp_offset, ch->bdshot_training_mask);
		PX4_INFO("CRC errors Frame error No response");
		PX4_INFO("%10lu %11lu %11lu", ch->crc_error_cnt, ch->frame_error_cnt, ch->no_response_cnt);
	}
}

void up_dshot_trigger(void)
{
	if (!_dshot_armed) {
		return;
	}

	hrt_abstime now = hrt_absolute_time();
	uint32_t tx_mask = 0;

	// Lock-free: the IRQ only acts on a channel whose interrupt is enabled, and a channel is only
	// touched here once its cycle is over, so the two contexts never own the same channel at once.
	for (uint32_t channel = 0; channel < DSHOT_TIMERS; channel++) {
		volatile dshot_channel_t *ch = &dshot_inst[channel];
		uint32_t bit = 1u << channel;

		if (!ch->init || ch->data_seg1 == 0) {
			continue;
		}

		if (ch->bdshot) {
			// A frame the IRQ has not consumed yet
			if (ch->state == BDSHOT_RECEIVE && (get_shifter_status_flags() & bit)) {
				bdshot_latch_response(channel);
			}

			// The ESC may still be driving the line
			if (ch->state != BDSHOT_RECEIVE_COMPLETE && (now - ch->tx_started) < _bdshot_busy_us) {
				continue;
			}

			// Calc data now since we're not event driven
			if (ch->tx_started != 0) {
				bdshot_process_response(channel);
			}

			flexio_dshot_output(channel);
		}

		// SSF is the timer trigger (active low): it reads "buffer empty" here and only the SHIFTBUF
		// write may clear it, or the timer starts on an empty shifter. TSF is just a stale flag.
		clear_timer_status_flags(bit);
		tx_mask |= bit;
	}

	// The IRQ must queue the second word before the first 24 sub-bits are out (20 us at DShot1200), so
	// preemption cannot be allowed between the buffer write and the interrupt enable. SHIFTSIEN/TIMIEN
	// have no set/clear aliases, so the read-modify-write also has to be atomic against the IRQ's own.
	irqstate_t irqflags = px4_enter_critical_section();

	for (uint32_t channel = 0; channel < DSHOT_TIMERS; channel++) {
		if (tx_mask & (1u << channel)) {
			dshot_inst[channel].state = DSHOT_START;
			dshot_inst[channel].tx_started = now;
			flexio_putreg32(dshot_inst[channel].data_seg1, IMXRT_FLEXIO_SHIFTBUF0_OFFSET + channel * 0x4);
		}
	}

	enable_shifter_status_interrupts(tx_mask);
	enable_timer_status_interrupts(tx_mask);
	px4_leave_critical_section(irqflags);
}

/* Expand packet from 16 bits to 48 to get T0H and T1H timing, each DShot bit becomes three
 * shifter bits: 110 for a one, 100 for a zero */
static uint64_t dshot_expand_data(uint16_t packet)
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
void dshot_motor_data_set(uint8_t channel, uint16_t throttle, bool telemetry)
{
	if (channel >= DSHOT_TIMERS || !dshot_inst[channel].init) {
		return;
	}

	uint16_t packet = 0;
	uint16_t checksum = 0;

	packet |= throttle << DSHOT_THROTTLE_POSITION;
	packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

	uint16_t csum_data = dshot_inst[channel].bdshot ? ~packet : packet;

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
}

int up_dshot_arm(bool armed)
{
	uint32_t mask = 0;

	for (uint32_t channel = 0; channel < DSHOT_TIMERS; channel++) {
		if (dshot_inst[channel].init) {
			mask |= 1u << channel;
		}
	}

	_dshot_armed = armed;

	if (!armed) {
		// Atomic against the IRQ's own SHIFTSIEN/TIMIEN updates; nothing fires once these are clear
		irqstate_t irqflags = px4_enter_critical_section();
		disable_shifter_status_interrupts(mask);
		disable_timer_status_interrupts(mask);
		px4_leave_critical_section(irqflags);

		for (uint32_t channel = 0; channel < DSHOT_TIMERS; channel++) {
			if (mask & (1u << channel)) {
				flexio_putreg32(0, IMXRT_FLEXIO_TIMCTL0_OFFSET + channel * 0x4);
				flexio_putreg32(0, IMXRT_FLEXIO_SHIFTCTL0_OFFSET + channel * 0x4);
			}
		}

		return 0;
	}

	// Interrupts are off after init or disarm, so the IRQ cannot see this reconfiguration
	for (uint32_t channel = 0; channel < DSHOT_TIMERS; channel++) {
		if (mask & (1u << channel)) {
			// Nothing is in flight; the first trigger must not count a missing response
			dshot_inst[channel].tx_started = 0;
			flexio_dshot_output(channel);
		}
	}

	return 0;
}
