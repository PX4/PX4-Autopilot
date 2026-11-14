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
 * @file bitbang_uart.cpp
 *
 * Half-duplex 8N1 UART on the ESC signal pins (drv_bitbang_uart.h) for the ESC bootloader at
 * 19200 baud: one shared timer polls for start bits at about 4 x baud interrupts per second while
 * listening, then samples or transmits once per bit. The timer stops while parked. Interrupt
 * latency limits reliable operation to 19200 baud.
 *
 * Same contract as the NXP FlexIO backend: init claims the given ESC pins through io_timer (dshot
 * and pwm_out cannot take them, and init fails with -EBUSY while they own one) and drives all of
 * them high. Only the active channel ever listens; every other line, and a line that has received
 * nothing for RX_IDLE_US, stays driven high, because the AM32 bootloader's boot check pulls the
 * line down and leaves the bootloader if it reads low.
 */

#include <drivers/drv_bitbang_uart.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>

#include <px4_arch/io_timer.h>
#include <board_config.h>

#include <stm32_tim.h>
#include <nuttx/irq.h>

#include <errno.h>
#include <limits.h>

#if !defined(CONFIG_ARCH_CHIP_STM32F7) && !defined(CONFIG_ARCH_CHIP_STM32H7)
# error "Not supported on this MCU, requires STM32 based MCU"
#endif

// stm32_tim_init() only knows the timers enabled in the board's NuttX defconfig
#if CONFIG_UART_BITBANG_TIMER == 13 && ((defined(CONFIG_ARCH_CHIP_STM32H7) && !defined(CONFIG_STM32H7_TIM13)) || \
					(defined(CONFIG_ARCH_CHIP_STM32F7) && !defined(CONFIG_STM32F7_TIM13)))
# error "CONFIG_UART_BITBANG_TIMER is 13: enable TIM13 in the board's NuttX defconfig (CONFIG_STM32H7_TIM13=y or CONFIG_STM32F7_TIM13=y)"
#endif

// Timer interrupt sources
#define GTIM_SR_UIF   (1 << 0)
#define GTIM_DIER_UIE (1 << 0)

// UART frame format: 8N1 (8 data bits, no parity, 1 stop bit)
#define MAX_UART_CHANNELS 8
#define NO_CHANNEL 0xFF
#define UART_TOTAL_BITS 10
#define TX_BUFFER_SIZE 512
#define RX_BUFFER_SIZE 512
#define RX_IDLE_US 200000 // Longer than a bootloader reply, shorter than its signal-loss reset
#define RX_OVERSAMPLE 4
#define RX_LATENCY_TICKS 3 // interrupt entry plus re-phase time, so the samples center on the bits
#define MAX_BAUDRATE 19200 // the start-bit poll runs at RX_OVERSAMPLE x baud
#define TIMER_CLOCK_HZ 1000000

typedef struct {
	uint8_t channel;
	uint32_t channel_mask;
	uint32_t gpio_pin;
	uint32_t bit_time_ticks;
	bool initialized;
	struct stm32_tim_dev_s *timer;

	px4::atomic_bool tx_busy;
	uint16_t tx_shift_reg;
	uint8_t tx_bit_count;
	hrt_abstime tx_deadline;

	uint8_t tx_buffer[TX_BUFFER_SIZE];
	// Wrapping counters allow all 512 slots to be queued before TX starts.
	uint16_t tx_head;
	uint16_t tx_tail;

	bool rx_enabled;
	bool rx_armed; // the line was seen high: a low poll is a start bit
	bool rx_in_progress;
	uint16_t rx_shift_reg;
	uint8_t rx_bit_count;
	hrt_abstime last_activity;

	uint8_t rx_buffer[RX_BUFFER_SIZE];
	uint16_t rx_head;
	uint16_t rx_tail;

} bitbang_uart_t;

// One session and one active channel. State transitions and ring access are protected against the timer ISR.
static bitbang_uart_t uart = {};

static bool channel_valid(uint8_t channel)
{
	return uart.initialized && channel < MAX_UART_CHANNELS && (uart.channel_mask & (1u << channel));
}

static void idle_high(uint8_t channel)
{
	// The AM32 boot check overpowers an input's pull-up and leaves the bootloader on a low line.
	const uint32_t pin = io_timer_channel_get_gpio_output(channel);
	px4_arch_gpiowrite(pin, true);
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_SET(pin));
}

static void stop_timer()
{
	STM32_TIM_DISABLEINT(uart.timer, GTIM_DIER_UIE);
	STM32_TIM_SETMODE(uart.timer, STM32_TIM_MODE_DISABLED);
	STM32_TIM_ACKINT(uart.timer, GTIM_SR_UIF);
}

static void start_timer(uint32_t period, uint32_t counter = 0)
{
	// SETMODE generates UG to load buffered ARR on F7/H7; set the phase and clear UIF afterwards.
	STM32_TIM_DISABLEINT(uart.timer, GTIM_DIER_UIE);
	STM32_TIM_SETPERIOD(uart.timer, period - 1);
	STM32_TIM_SETMODE(uart.timer, STM32_TIM_MODE_UP);
	STM32_TIM_SETCOUNTER(uart.timer, counter);
	STM32_TIM_ACKINT(uart.timer, GTIM_SR_UIF);
	STM32_TIM_ENABLEINT(uart.timer, GTIM_DIER_UIE);
}

static void stop_receiving()
{
	stop_timer();
	uart.rx_enabled = false;
	uart.rx_in_progress = false;
}

static void listen_for_start_bit(bool line_high)
{
	// A line held low (unpowered ESC, framing error) must not restart a byte on every poll
	uart.rx_armed = line_high;
	uart.rx_in_progress = false;
	start_timer(uart.bit_time_ticks / RX_OVERSAMPLE);
}

static void configure_rx()
{
	stop_timer();
	px4_arch_configgpio(PX4_MAKE_GPIO_INPUT(uart.gpio_pin));
	uart.rx_tail = uart.rx_head;
	uart.rx_enabled = true;
	uart.last_activity = hrt_absolute_time();
	// The line was driven high until now (stop bit or parked), so the first low poll is a start bit
	listen_for_start_bit(true);
}

static void park_active_channel()
{
	stop_receiving();

	if (uart.channel != NO_CHANNEL) {
		idle_high(uart.channel);
	}
}

static uint16_t tx_queued()
{
	return (uint16_t)(uart.tx_head - uart.tx_tail);
}

static void load_tx_byte()
{
	const uint8_t byte = uart.tx_buffer[uart.tx_tail % TX_BUFFER_SIZE];
	++uart.tx_tail;
	uart.tx_shift_reg = (byte << 1) | (1 << (UART_TOTAL_BITS - 1));
	uart.tx_bit_count = 0;
}

static void finish_tx()
{
	// Also discards any queued bytes when a timed-out transmission is cancelled.
	uart.tx_tail = uart.tx_head;
	configure_rx();
	uart.tx_busy.store(false);
}

static void start_tx(hrt_abstime deadline)
{
	// A partial RX byte must not suppress start-bit detection after this transmission.
	stop_receiving();
	idle_high(uart.channel);
	load_tx_byte();
	uart.tx_deadline = deadline;
	uart.tx_busy.store(true);

	// Starting from zero holds driven-high idle for one bit time before the first start bit.
	start_timer(uart.bit_time_ticks);
}

static int wait_for_tx()
{
	while (uart.tx_busy.load()) {
		irqstate_t flags = px4_enter_critical_section();

		if (uart.tx_busy.load() && hrt_absolute_time() >= uart.tx_deadline) {
			finish_tx();
			px4_leave_critical_section(flags);
			return -ETIMEDOUT;
		}

		const bool sleep = tx_queued() > 2;
		px4_leave_critical_section(flags);

		// Poll the final bytes so consecutive writes need not wait for another scheduler tick.
		if (sleep) {
			px4_usleep(1000);
		}
	}

	return 0;
}

// TIM update interrupt handler: quarter-bit polling while listening, full bits during RX/TX.
// The only interrupt that touches the UART state, so it needs no critical section of its own.
static int timer_update_isr(int, void *, void *)
{
	// A stopped/restarted timer can still have an interrupt pending in the NVIC.
	if (!uart.timer || !STM32_TIM_CHECKINT(uart.timer, GTIM_SR_UIF)) {
		return OK;
	}

	STM32_TIM_ACKINT(uart.timer, GTIM_SR_UIF);

	if (uart.tx_busy.load()) {
		// The final stop bit stays driven for a full tick before RX takes over.
		if (uart.tx_bit_count == UART_TOTAL_BITS) {
			if (tx_queued() > 0) {
				load_tx_byte();

			} else {
				// Turn around in the ISR: a bootloader can reply before the task runs again.
				finish_tx();
			}
		}

		if (uart.tx_busy.load()) {
			px4_arch_gpiowrite(uart.gpio_pin, (uart.tx_shift_reg >> uart.tx_bit_count) & 1);
			++uart.tx_bit_count;
		}

	} else if (uart.rx_enabled) {
		const bool high = px4_arch_gpioread(uart.gpio_pin);

		if (!uart.rx_in_progress) {
			if (high) {
				uart.rx_armed = true;

			} else if (uart.rx_armed) {
				uart.rx_in_progress = true;
				uart.rx_shift_reg = 0;
				uart.rx_bit_count = 0;
				// Center the samples over the preceding poll interval's possible start edges; every
				// sample is also read an interrupt latency after its update event.
				const uint32_t poll_ticks = uart.bit_time_ticks / RX_OVERSAMPLE;
				const uint32_t start_delay = uart.bit_time_ticks / 2 - poll_ticks / 2 - RX_LATENCY_TICKS;
				start_timer(uart.bit_time_ticks, uart.bit_time_ticks - start_delay);
			}

		} else if (uart.rx_bit_count == 0 && high) {
			// A high start-bit sample was a glitch.
			listen_for_start_bit(true);

		} else {
			if (high) {
				uart.rx_shift_reg |= (1 << uart.rx_bit_count);
			}

			if (++uart.rx_bit_count == UART_TOTAL_BITS) {
				listen_for_start_bit(high);
				const uint16_t next_head = (uart.rx_head + 1) % RX_BUFFER_SIZE;

				// Data is LSB first between start and stop; drop on framing error or overrun. Only a
				// valid byte counts as activity, so a line stuck low still parks after RX_IDLE_US.
				if (high && next_head != uart.rx_tail) {
					uart.rx_buffer[uart.rx_head] = (uart.rx_shift_reg >> 1) & 0xFF;
					uart.rx_head = next_head;
					uart.last_activity = hrt_absolute_time();
				}
			}
		}
	}

	return OK;
}

static int bitbang_uart_init_timer()
{
	uart.timer = stm32_tim_init(CONFIG_UART_BITBANG_TIMER);

	// Not -EBUSY: callers take that as pins owned by an output driver
	if (!uart.timer) {
		return -ENODEV;
	}

	int ret = STM32_TIM_SETMODE(uart.timer, STM32_TIM_MODE_DISABLED);

	if (ret == 0) {
		STM32_TIM_SETCLOCK(uart.timer, TIMER_CLOCK_HZ);
		stop_timer();
		ret = STM32_TIM_SETISR(uart.timer, timer_update_isr, nullptr, 0);
	}

	if (ret < 0) {
		stm32_tim_deinit(uart.timer);
		uart.timer = nullptr;
	}

	return ret;
}

static void release_channels()
{
	for (uint8_t ch = 0; ch < MAX_UART_CHANNELS; ch++) {
		if (uart.channel_mask & (1u << ch)) {
			io_timer_unallocate_channel(ch);
		}
	}

	uart.channel_mask = 0;
}

int bitbang_uart_init(uint32_t baudrate, bool single_wire, uint32_t channel_mask)
{
	irqstate_t flags = px4_enter_critical_section();

	if (uart.initialized) {
		px4_leave_critical_section(flags);
		return -EBUSY;
	}

	if (!single_wire) {
		px4_leave_critical_section(flags);
		return -ENOSYS;
	}

	// Faster rates would poll for start bits faster than the interrupt can run; the period must fit
	// even a 16-bit general-purpose timer.
	const uint32_t bit_time_ticks = baudrate ? TIMER_CLOCK_HZ / baudrate : 0;

	if (baudrate == 0 || baudrate > MAX_BAUDRATE || bit_time_ticks > UINT16_MAX + 1u) {
		px4_leave_critical_section(flags);
		return -EINVAL;
	}

	for (uint8_t ch = 0; ch < MAX_UART_CHANNELS; ch++) {
		if ((channel_mask & (1u << ch)) && io_timer_channel_get_gpio_output(ch) != 0) {
			if (io_timer_allocate_channel(ch, IOTimerChanMode_Other) != 0) {
				release_channels();
				px4_leave_critical_section(flags);
				return -EBUSY;
			}

			uart.channel_mask |= 1u << ch;
		}
	}

	if (uart.channel_mask == 0) {
		px4_leave_critical_section(flags);
		return -EINVAL;
	}

	// Claims must all succeed before touching the shared timer or any output pin.
	uart.channel = NO_CHANNEL;
	uart.gpio_pin = 0;
	uart.bit_time_ticks = bit_time_ticks;
	uart.tx_busy.store(false);
	uart.tx_head = 0;
	uart.tx_tail = 0;
	uart.rx_enabled = false;
	uart.rx_armed = false;
	uart.rx_in_progress = false;
	uart.rx_head = 0;
	uart.rx_tail = 0;

	const int ret = bitbang_uart_init_timer();

	if (ret != 0) {
		release_channels();
		px4_leave_critical_section(flags);
		return ret;
	}

	for (uint8_t ch = 0; ch < MAX_UART_CHANNELS; ch++) {
		if (uart.channel_mask & (1u << ch)) {
			idle_high(ch);
		}
	}

	uart.initialized = true;
	px4_leave_critical_section(flags);
	return 0;
}

// Caller holds the critical section and chooses RX or TX before leaving it.
static void select_channel(uint8_t channel)
{
	park_active_channel();
	uart.channel = channel;
	uart.gpio_pin = io_timer_channel_get_gpio_output(channel);
	uart.tx_head = 0;
	uart.tx_tail = 0;
}

static int bitbang_uart_switch_channel(uint8_t channel)
{
	if (!channel_valid(channel)) {
		return -EINVAL;
	}

	const int ret = wait_for_tx();

	if (ret != 0) {
		return ret;
	}

	irqstate_t flags = px4_enter_critical_section();

	// A read on the same channel preserves its state, including driven-high idle parking.
	if (uart.channel != channel) {
		select_channel(channel);
		configure_rx();
	}

	px4_leave_critical_section(flags);
	return 0;
}

int bitbang_uart_deinit()
{
	if (!uart.initialized) {
		return -EINVAL;
	}

	const int ret = wait_for_tx();
	irqstate_t flags = px4_enter_critical_section();
	park_active_channel();
	STM32_TIM_SETISR(uart.timer, nullptr, nullptr, 0);
	stm32_tim_deinit(uart.timer);
	uart.timer = nullptr;

	for (uint8_t ch = 0; ch < MAX_UART_CHANNELS; ch++) {
		if (uart.channel_mask & (1u << ch)) {
			idle_high(ch);
		}
	}

	release_channels();
	uart.channel = NO_CHANNEL;
	uart.initialized = false;
	px4_leave_critical_section(flags);
	return ret;
}

int bitbang_uart_idle(uint8_t channel)
{
	if (!channel_valid(channel)) {
		return -EINVAL;
	}

	const int ret = wait_for_tx();
	irqstate_t flags = px4_enter_critical_section();
	park_active_channel();
	idle_high(channel);
	px4_leave_critical_section(flags);
	return ret;
}

int bitbang_uart_write(uint8_t channel, const uint8_t *data, size_t length)
{
	if (!channel_valid(channel) || !data || length == 0 || length > INT_MAX) {
		return -EINVAL;
	}

	int ret = wait_for_tx();

	if (ret != 0) {
		return ret;
	}

	const hrt_abstime deadline = hrt_absolute_time() + (length + 2ULL) * UART_TOTAL_BITS * 2 * uart.bit_time_ticks;
	size_t queued = 0;

	while (queued < length) {
		irqstate_t flags = px4_enter_critical_section();

		if (hrt_absolute_time() >= deadline) {
			if (uart.tx_busy.load()) {
				finish_tx();
			}

			px4_leave_critical_section(flags);
			return -ETIMEDOUT;
		}

		if (uart.channel != channel) {
			select_channel(channel);
		}

		// ESC frames fit in this ring: queue them fully before starting, so task latency cannot split a frame.
		while (queued < length && tx_queued() < TX_BUFFER_SIZE) {
			uart.tx_buffer[uart.tx_head % TX_BUFFER_SIZE] = data[queued++];
			++uart.tx_head;
		}

		// Channel selection, queueing and TX startup must be atomic with respect to the timer ISR.
		if (!uart.tx_busy.load()) {
			start_tx(deadline);
		}

		px4_leave_critical_section(flags);

		if (queued < length) {
			// Serial passthrough may send more than one ring's worth of bytes.
			px4_usleep(1000);
		}
	}

	ret = wait_for_tx();
	return ret == 0 ? (int)length : ret;
}

int bitbang_uart_read(uint8_t channel, uint8_t *data, size_t length, uint32_t timeout_us)
{
	if (!channel_valid(channel) || !data || length == 0 || length > INT_MAX) {
		return -EINVAL;
	}

	const int ret = bitbang_uart_switch_channel(channel);

	if (ret != 0) {
		return ret;
	}

	const hrt_abstime deadline = hrt_absolute_time() + timeout_us;
	size_t bytes_read = 0;

	while (bytes_read < length) {
		irqstate_t flags = px4_enter_critical_section();

		if (uart.rx_tail != uart.rx_head) {
			data[bytes_read++] = uart.rx_buffer[uart.rx_tail];
			uart.rx_tail = (uart.rx_tail + 1) % RX_BUFFER_SIZE;
			px4_leave_critical_section(flags);
			continue;
		}

		const hrt_abstime now = hrt_absolute_time();

		if (timeout_us > 0 && now >= deadline) {
			if (uart.rx_enabled && now - uart.last_activity > RX_IDLE_US) {
				park_active_channel();
			}

			px4_leave_critical_section(flags);
			break;
		}

		px4_leave_critical_section(flags);
		// RX is buffered by the ISR; poll once per scheduler tick on these boards.
		px4_usleep(1000);
	}

	return (int)bytes_read;
}
