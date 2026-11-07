/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file flexio_uart.c
 *
 * Half-duplex 8N1 UART on the ESC signal pins (drv_bitbang_uart.h) using FlexIO1, for the ESC
 * bootloader at 19200 baud. One channel is active at a time. Every other channel keeps a transmit
 * shifter that holds its stop bit, so all ESC lines are driven high: the AM32 bootloader boot
 * check pulls the line down and leaves the bootloader if it reads low.
 *
 * Timers: UART_TIM clocks the active shifter, HELPER_TIM is the RT106x baud/prescaler timer and
 * IDLE_TIM is never enabled, so idle shifters parked on it never shift.
 *
 * Desk check at 19200 baud (dual 8-bit baud mode: bit = 2 * (TIMCMP[7:0] + 1) decrements):
 *
 *   part     FlexIO clock  decrement source               divider               baud      error
 *   RT1176   108 MHz       FlexIO clock / 16 (TIMDEC 100b) TIMCMP[7:0] = 175    19176.1   -0.12 %
 *   i.MX95   133.33 MHz    FlexIO clock / 16 (TIMDEC 100b) TIMCMP[7:0] = 216    19201.2   +0.01 %
 *   RT1064   108 MHz  TX   timer 1, 16-bit, half bit       TIMCMP1 = 2812       19196.6   -0.02 %
 *            108 MHz  RX   timer 1 output, every 11 clocks TIMCMP[7:0] = 255    19176.1   -0.12 %
 *
 * RT106x has no decrement prescaler. TX follows the reference manual's "UART transmit
 * configuration for slow baud rate": timer 1 in 16-bit mode is enabled by the shifter flag,
 * toggles every half bit and clocks timer 0, which counts the 16 half bits and stops timer 1 on
 * disable. RX decrements timer 0 on both edges of a free-running timer 1 that toggles every 11
 * FlexIO clocks; RX needs no flag-gated enable because the start bit enables it.
 *
 * Transmit is interrupt driven: the shifter interrupt feeds the bytes, the timer interrupt marks
 * the end of the last byte's data bits, and the interrupt switches the line to receive after the
 * stop bit, because a bootloader answers right after it. Limits: the interrupt spins for that stop
 * bit (one bit time, 52 us at 19200 baud) because no timer flags its end; a timer that also counts
 * the stop bit would remove the spin. A frame written in several calls stays contiguous only if
 * the caller writes again within about 260 us of the previous call's return.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_bitbang_uart.h>
#include <drivers/drv_hrt.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <errno.h>
#include <stdlib.h>

#include "../include/flexio.h"

#define UART_TIM		0u
#define HELPER_TIM		1u	/* N-1 enable/disable is reserved for timers 0 and 4 */
#define IDLE_TIM		7u
#define RX_PRESCALE		11u
#define RX_IDLE_US		200000	/* longer than any bootloader reply, well inside the ESC's 2 s signal loss reset */
/* One shifter per output channel */
#define MAX_CHANNELS		(MAX_TIMER_IO_CHANNELS < FLEXIO_SHIFTBUFNIS_COUNT ? MAX_TIMER_IO_CHANNELS : FLEXIO_SHIFTBUFNIS_COUNT)

/* Session state, allocated by bitbang_uart_init() and freed by bitbang_uart_deinit() */
struct flexio_uart_s {
	volatile int8_t channel;
	volatile bool rx;
	volatile bool rx_bytes;		/* a byte was received since the last switch to receive */
	volatile bool tx_done;		/* the last frame's stop bit is out, the line is receiving */
	uint32_t channel_mask;
	uint32_t tx_cmp;
	uint32_t rx_cmp;
	uint32_t bit_us;
	hrt_abstime last_activity;
	uint8_t ring[256];
	volatile uint8_t head;
	volatile uint8_t tail;
	const uint8_t *tx_data;
	size_t tx_len;
	volatile size_t tx_pos;
};

static struct flexio_uart_s *_s;

static void configure_rx(uint8_t ch);

static int flexio_uart_isr(int irq, void *context, void *arg)
{
	struct flexio_uart_s *s = _s;

	if (!s || s->channel < 0) {
		return OK;
	}

	const int8_t ch = s->channel;
	const bool shifter_flag = flexio_getreg32(FLEXIO_OFFSET(SHIFTSTAT)) & (1u << ch);

	if (s->rx) {
		if (shifter_flag) {
			/* The receiver shifts in from bit 31, so the byte lands in SHIFTBUF[31:24], which reads as
			 * SHIFTBUFBYS[7:0] (byte swapped alias). Reading it clears the flag. */
			uint8_t byte = flexio_getreg32(FLEXIO_OFFSET(SHIFTBUFBYS0) + ch * 0x4);
			s->rx_bytes = true;

			if ((uint8_t)(s->head + 1) != s->tail) {
				s->ring[s->head++] = byte;
			}
		}

	} else if (flexio_getreg32(FLEXIO_OFFSET(TIMIEN)) & (1u << UART_TIM)) {
		/* The last byte's data bits ended on the timer compare: wait out the stop bit, then listen */
		if (flexio_getreg32(FLEXIO_OFFSET(TIMSTAT)) & (1u << UART_TIM)) {
			up_udelay(s->bit_us);
			configure_rx(ch);
			s->tx_done = true;
		}

	} else if (shifter_flag) {
		if (s->tx_pos < s->tx_len) {
			/* Refill the emptied shifter buffer, so a thread hold-off cannot open an inter-byte gap
			 * (the ESC bootloader drops a frame on a gap of more than about 260 us) */
			flexio_putreg32(s->tx_data[s->tx_pos++], FLEXIO_OFFSET(SHIFTBUF0) + ch * 0x4);

		} else {
			/* The last byte moved into the shifter: its data bits end on the next timer compare */
			flexio_modifyreg32(FLEXIO_OFFSET(SHIFTSIEN), 1u << ch, 0);
			flexio_putreg32(1u << UART_TIM, FLEXIO_OFFSET(TIMSTAT));
			flexio_modifyreg32(FLEXIO_OFFSET(TIMIEN), 0, 1u << UART_TIM);
		}
	}

	return OK;
}

/* Divider values for the FlexIO clock; -EINVAL if the baud rate is out of reach */
static int baud_config(struct flexio_uart_s *s, uint32_t baud)
{
	if (baud == 0) {
		return -EINVAL;
	}

#if FLEXIO_HAS_CLOCK_PRESCALER
	uint32_t div = (BOARD_FLEXIO_PREQ + 16 * baud) / (2 * 16 * baud) - 1;
	s->tx_cmp = ((8 * 2 - 1) << 8) | div;
#else
	uint32_t div = (BOARD_FLEXIO_PREQ + RX_PRESCALE * baud) / (2 * RX_PRESCALE * baud) - 1;
	s->tx_cmp = (BOARD_FLEXIO_PREQ + baud) / (2 * baud) - 1;	/* helper timer, half a bit */
#endif

	if (div > 0xFF || s->tx_cmp > 0xFFFF) {
		return -EINVAL;
	}

	s->rx_cmp = ((8 * 2 - 1) << 8) | div;
	s->bit_us = (1000000 + baud - 1) / baud;
	return 0;
}

static void shifter(uint32_t ch, uint32_t timer, bool rx)
{
	flexio_putreg32(0, FLEXIO_OFFSET(SHIFTCTL0) + ch * 0x4);

	/* Start bit 0, stop bit 1. A transmit shifter outputs its stop bit as soon as it is configured. */
	flexio_putreg32(FLEXIO_SHIFTCFG_INSRC(FLEXIO_SHIFTER_INPUT_FROM_PIN) |
			FLEXIO_SHIFTCFG_SSTOP(FLEXIO_SHIFTER_STOP_BIT_HIGH) |
			FLEXIO_SHIFTCFG_SSTART(FLEXIO_SHIFTER_START_BIT_LOW),
			FLEXIO_OFFSET(SHIFTCFG0) + ch * 0x4);

	/* Transmit shifts on the rising shift clock edge (bit start), receive samples on the falling one (mid bit) */
	flexio_putreg32(FLEXIO_SHIFTCTL_TIMSEL(timer) |
			FLEXIO_SHIFTCTL_TIMPOL(rx ? FLEXIO_SHIFTER_TIMER_POLARITY_ON_NEGATIVE : FLEXIO_SHIFTER_TIMER_POLARITY_ON_POSITIVE) |
			FLEXIO_SHIFTCTL_PINCFG(rx ? FLEXIO_PIN_CONFIG_OUTPUT_DISABLED : FLEXIO_PIN_CONFIG_OUTPUT) |
			FLEXIO_SHIFTCTL_PINSEL(timer_io_channels[ch].dshot.flexio_pin) |
			FLEXIO_SHIFTCTL_PINPOL(FLEXIO_PIN_ACTIVE_HIGH) |
			FLEXIO_SHIFTCTL_SMOD(rx ? FLEXIO_SHIFTER_MODE_RECEIVE : FLEXIO_SHIFTER_MODE_TRANSMIT),
			FLEXIO_OFFSET(SHIFTCTL0) + ch * 0x4);
}

static void timers_off(void)
{
	flexio_modifyreg32(FLEXIO_OFFSET(SHIFTSIEN), _s->channel_mask, 0);
	flexio_modifyreg32(FLEXIO_OFFSET(TIMIEN), 1u << UART_TIM, 0);
	_s->rx = false;
	flexio_putreg32(0, FLEXIO_OFFSET(TIMCTL0) + UART_TIM * 0x4);
	flexio_putreg32(0, FLEXIO_OFFSET(TIMCTL0) + HELPER_TIM * 0x4);
}

/* Line driven high by a transmit shifter whose timer never runs */
static void idle_high(uint32_t ch)
{
	shifter(ch, IDLE_TIM, false);
}

/* Park the previously active channel and make ch the active one, timers stopped */
static void activate(uint8_t ch)
{
	timers_off();

	if (_s->channel >= 0 && _s->channel != ch) {
		idle_high(_s->channel);
	}

	_s->channel = ch;
	_s->last_activity = hrt_absolute_time();
}

/* Writing SHIFTBUF starts a frame: the timer is gated by the shifter flag (active low = buffer full) */
static void configure_tx(uint8_t ch)
{
	const uint32_t flag_trigger = FLEXIO_TIMCTL_TRGSEL((4 * ch) + 1) |
				      FLEXIO_TIMCTL_TRGPOL(FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_LOW) |
				      FLEXIO_TIMCTL_TRGSRC(FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL);

	activate(ch);
	shifter(ch, UART_TIM, false);

#if FLEXIO_HAS_CLOCK_PRESCALER
	flexio_putreg32(FLEXIO_TIMCFG_TIMOUT(FLEXIO_TIMER_OUTPUT_ONE_NOT_AFFECTED_BY_RESET) |
			FLEXIO_TIMCFG_TIMDEC3(FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_DIV16) |
			FLEXIO_TIMCFG_TIMRST(FLEXIO_TIMER_RESET_NEVER) |
			FLEXIO_TIMCFG_TIMDIS(FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE) |
			FLEXIO_TIMCFG_TIMENA(FLEXIO_TIMER_ENABLE_ON_TRIGGER_HIGH) |
			FLEXIO_TIMCFG_TSTOP(FLEXIO_TIMER_STOP_BIT_ENABLE_ON_TIMER_DISABLE) |
			FLEXIO_TIMCFG_TSTART(FLEXIO_TIMER_START_BIT_ENABLED),
			FLEXIO_OFFSET(TIMCFG0) + UART_TIM * 0x4);
	flexio_putreg32(_s->tx_cmp, FLEXIO_OFFSET(TIMCMP0) + UART_TIM * 0x4);
	flexio_putreg32(flag_trigger |
			FLEXIO_TIMCTL_PINCFG(FLEXIO_PIN_CONFIG_OUTPUT_DISABLED) |
			FLEXIO_TIMCTL_TIMOD(FLEXIO_TIMER_MODE_DUAL8_BIT_BAUD_BIT),
			FLEXIO_OFFSET(TIMCTL0) + UART_TIM * 0x4);
#else
	/* Helper timer runs while SHIFTBUF is full and stops with timer 0 */
	flexio_putreg32(FLEXIO_TIMCFG_TIMOUT(FLEXIO_TIMER_OUTPUT_ONE_NOT_AFFECTED_BY_RESET) |
			FLEXIO_TIMCFG_TIMDIS(FLEXIO_TIMER_DISABLE_ON_PRE_TIMER_DISABLE) |
			FLEXIO_TIMCFG_TIMENA(FLEXIO_TIMER_ENABLE_ON_TRIGGER_HIGH),
			FLEXIO_OFFSET(TIMCFG0) + HELPER_TIM * 0x4);
	flexio_putreg32(_s->tx_cmp, FLEXIO_OFFSET(TIMCMP0) + HELPER_TIM * 0x4);
	/* Timer 0 counts 16 helper edges (8 bits) and shifts on the helper output */
	flexio_putreg32(FLEXIO_TIMCFG_TIMOUT(FLEXIO_TIMER_OUTPUT_ONE_NOT_AFFECTED_BY_RESET) |
			FLEXIO_TIMCFG_TIMDEC(FLEXIO_TIMER_DEC_SRC_ON_TRIGGER_INPUT_SHIFT_TRIGGER_INPUT) |
			FLEXIO_TIMCFG_TIMRST(FLEXIO_TIMER_RESET_NEVER) |
			FLEXIO_TIMCFG_TIMDIS(FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE) |
			FLEXIO_TIMCFG_TIMENA(FLEXIO_TIMER_ENABLE_ON_TRIGGER_RISING_EDGE) |
			FLEXIO_TIMCFG_TSTOP(FLEXIO_TIMER_STOP_BIT_ENABLE_ON_TIMER_DISABLE) |
			FLEXIO_TIMCFG_TSTART(FLEXIO_TIMER_START_BIT_ENABLED),
			FLEXIO_OFFSET(TIMCFG0) + UART_TIM * 0x4);
	flexio_putreg32(8 * 2 - 1, FLEXIO_OFFSET(TIMCMP0) + UART_TIM * 0x4);
	flexio_putreg32(FLEXIO_TIMCTL_TRGSEL((4 * HELPER_TIM) + 3) |
			FLEXIO_TIMCTL_TRGSRC(FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL) |
			FLEXIO_TIMCTL_TIMOD(FLEXIO_TIMER_MODE_SINGLE16_BIT),
			FLEXIO_OFFSET(TIMCTL0) + UART_TIM * 0x4);
	flexio_putreg32(flag_trigger | FLEXIO_TIMCTL_TIMOD(FLEXIO_TIMER_MODE_SINGLE16_BIT),
			FLEXIO_OFFSET(TIMCTL0) + HELPER_TIM * 0x4);
#endif
}

/* The start bit's falling edge (inverted pin) enables the timer, every falling edge resynchronises
 * the baud counter, and the first shift clock falling edge lands mid start bit */
static void configure_rx(uint8_t ch)
{
	uint32_t timcfg = FLEXIO_TIMCFG_TIMOUT(FLEXIO_TIMER_OUTPUT_ONE_AFFECTED_BY_RESET) |
			  FLEXIO_TIMCFG_TIMRST(FLEXIO_TIMER_RESET_ON_TIMER_PIN_RISING_EDGE) |
			  FLEXIO_TIMCFG_TIMDIS(FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE) |
			  FLEXIO_TIMCFG_TIMENA(FLEXIO_TIMER_ENABLE_ON_PIN_RISING_EDGE) |
			  FLEXIO_TIMCFG_TSTOP(FLEXIO_TIMER_STOP_BIT_ENABLE_ON_TIMER_DISABLE) |
			  FLEXIO_TIMCFG_TSTART(FLEXIO_TIMER_START_BIT_ENABLED);
	uint32_t timctl = FLEXIO_TIMCTL_PINCFG(FLEXIO_PIN_CONFIG_OUTPUT_DISABLED) |
			  FLEXIO_TIMCTL_PINSEL(timer_io_channels[ch].dshot.flexio_pin) |
			  FLEXIO_TIMCTL_PINPOL(FLEXIO_PIN_ACTIVE_LOW) |
			  FLEXIO_TIMCTL_TIMOD(FLEXIO_TIMER_MODE_DUAL8_BIT_BAUD_BIT);

	activate(ch);
	shifter(ch, UART_TIM, true);
	flexio_putreg32(1u << ch, FLEXIO_OFFSET(SHIFTSTAT));
	flexio_putreg32(1u << ch, FLEXIO_OFFSET(SHIFTERR));
	_s->tail = _s->head;
	_s->rx_bytes = false;

#if FLEXIO_HAS_CLOCK_PRESCALER
	timcfg |= FLEXIO_TIMCFG_TIMDEC3(FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_DIV16);
#else
	/* Free-running helper: always enabled, output toggles every RX_PRESCALE FlexIO clocks */
	timcfg |= FLEXIO_TIMCFG_TIMDEC(FLEXIO_TIMER_DEC_SRC_ON_TRIGGER_INPUT_SHIFT_TIMER_OUTPUT);
	timctl |= FLEXIO_TIMCTL_TRGSEL((4 * HELPER_TIM) + 3) | FLEXIO_TIMCTL_TRGSRC(FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL);
	flexio_putreg32(0, FLEXIO_OFFSET(TIMCFG0) + HELPER_TIM * 0x4);
	flexio_putreg32(RX_PRESCALE - 1, FLEXIO_OFFSET(TIMCMP0) + HELPER_TIM * 0x4);
	flexio_putreg32(FLEXIO_TIMCTL_TIMOD(FLEXIO_TIMER_MODE_SINGLE16_BIT), FLEXIO_OFFSET(TIMCTL0) + HELPER_TIM * 0x4);
#endif

	flexio_putreg32(timcfg, FLEXIO_OFFSET(TIMCFG0) + UART_TIM * 0x4);
	flexio_putreg32(_s->rx_cmp, FLEXIO_OFFSET(TIMCMP0) + UART_TIM * 0x4);
	flexio_putreg32(timctl, FLEXIO_OFFSET(TIMCTL0) + UART_TIM * 0x4);

	_s->rx = true;
	flexio_modifyreg32(FLEXIO_OFFSET(SHIFTSIEN), 0, 1u << ch);
}

static bool channel_valid(uint8_t ch)
{
	return _s && ch < MAX_CHANNELS && (_s->channel_mask & (1u << ch));
}

/* Returns the claimed pins to the io_timer pool */
static void release(uint32_t mask)
{
	for (uint32_t ch = 0; ch < MAX_CHANNELS; ch++) {
		if (mask & (1u << ch)) {
			io_timer_unallocate_channel(ch);
		}
	}
}

int bitbang_uart_init(uint32_t baudrate, bool single_wire)
{
	struct flexio_uart_s *s = calloc(1, sizeof(*s));

	if (!s) {
		return -ENOMEM;
	}

	s->channel = -1;

	if (!single_wire || baud_config(s, baudrate) != 0) {
		free(s);
		return -EINVAL;
	}

	/* One session at a time */
	irqstate_t flags = enter_critical_section();
	const bool busy = _s != NULL;

	if (!busy) {
		_s = s;
	}

	leave_critical_section(flags);

	if (busy) {
		free(s);
		return -EBUSY;
	}

	/* Claim every pin with a FlexIO function and refuse pins another driver owns (dshot, pwm_out,
	 * capture); the claim keeps a later dshot or pwm_out start off these pins */
	uint32_t mask = 0;

	for (uint32_t ch = 0; ch < MAX_CHANNELS; ch++) {
		if (!flexio_pin_unused(&timer_io_channels[ch].dshot)) {
			if (io_timer_allocate_channel(ch, IOTimerChanMode_Other) != 0) {
				release(mask);
				_s = NULL;
				free(s);
				return -EBUSY;
			}

			mask |= 1u << ch;
		}
	}

	flexio_clock_enable();
	flexio_modifyreg32(FLEXIO_OFFSET(CTRL), 0, FLEXIO_CTRL_SWRST_MASK);
	flexio_putreg32(0, FLEXIO_OFFSET(CTRL));
	flexio_putreg32(FLEXIO_CTRL_DBGE(1) | FLEXIO_CTRL_FASTACC(1) | FLEXIO_CTRL_FLEXEN(1), FLEXIO_OFFSET(CTRL));

	s->channel_mask = mask;

	for (uint32_t ch = 0; ch < MAX_CHANNELS; ch++) {
		if (mask & (1u << ch)) {
			/* Shifter first, so the pad is driven high the moment it is muxed to FlexIO */
			idle_high(ch);
			flexio_pin_config(&timer_io_channels[ch].dshot);
		}
	}

	irq_attach(FLEXIO_IRQ, flexio_uart_isr, NULL);
	up_enable_irq(FLEXIO_IRQ);

	return 0;
}

/* Every line stays driven high by its parked shifter; FlexIO keeps running for that */
int bitbang_uart_deinit(void)
{
	struct flexio_uart_s *s = _s;

	if (!s) {
		return -ENODEV;
	}

	timers_off();

	if (s->channel >= 0) {
		idle_high(s->channel);
	}

	up_disable_irq(FLEXIO_IRQ);
	irq_detach(FLEXIO_IRQ);
	release(s->channel_mask);
	_s = NULL;
	free(s);
	return 0;
}

int bitbang_uart_idle(uint8_t channel)
{
	if (!channel_valid(channel)) {
		return -EINVAL;
	}

	timers_off();

	if (_s->channel >= 0) {
		idle_high(_s->channel);
	}

	idle_high(channel);
	return 0;
}

int bitbang_uart_write(uint8_t channel, const uint8_t *data, size_t length)
{
	if (!channel_valid(channel) || !data || length == 0) {
		return -EINVAL;
	}

	struct flexio_uart_s *s = _s;

	const hrt_abstime deadline = hrt_absolute_time() + (length + 2) * 20 * s->bit_us;

	/* After the far end answered, the receiving line is only held by the pad pull-up while the far
	 * end lets go of it. Drive it high for one bit time first, so the far end's receiver does not
	 * take that edge as part of the frame (an ESC bootloader then rejects the frame). A write that
	 * follows a write starts at once, so a frame written in several calls stays contiguous. */
	if (s->rx && s->rx_bytes && s->channel == (int8_t)channel) {
		timers_off();
		idle_high(channel);
		up_udelay(s->bit_us);
	}

	configure_tx(channel);
	s->tx_data = data;
	s->tx_len = length;
	s->tx_pos = 0;
	s->tx_done = false;
	flexio_modifyreg32(FLEXIO_OFFSET(SHIFTSIEN), 0, 1u << channel);

	/* The interrupt sends the frame and switches to receive. Sleep while more than two bytes are
	 * queued, then poll (at most about four byte times), so the call returns at the frame end. */
	while (!s->tx_done && hrt_absolute_time() < deadline) {
		if (length - s->tx_pos > 2) {
			px4_usleep(1000);
		}
	}

	int ret = (int)length;
	irqstate_t flags = enter_critical_section();

	if (!s->tx_done) {
		configure_rx(channel);
		ret = -ETIMEDOUT;
	}

	leave_critical_section(flags);
	return ret;
}

int bitbang_uart_read(uint8_t channel, uint8_t *data, size_t length, uint32_t timeout_us)
{
	if (!channel_valid(channel) || !data || length == 0) {
		return -EINVAL;
	}

	/* A channel switch listens on the new line. The same channel keeps its state, so a line that
	 * went back to idle after RX_IDLE_US stays driven high until the next write. */
	if (channel != _s->channel) {
		configure_rx(channel);
	}

	const hrt_abstime deadline = hrt_absolute_time() + timeout_us;
	size_t n = 0;

	while (n < length) {
		if (_s->tail != _s->head) {
			data[n++] = _s->ring[_s->tail++];
			_s->last_activity = hrt_absolute_time();
			continue;
		}

		const hrt_abstime now = hrt_absolute_time();

		if (timeout_us > 0 && now >= deadline) {
			if (_s->rx && now - _s->last_activity > RX_IDLE_US) {
				timers_off();
				idle_high(channel);
			}

			break;
		}

		/* Bytes arrive through the shifter interrupt, so sleeping a tick loses nothing */
		px4_usleep(10);
	}

	return n;
}
