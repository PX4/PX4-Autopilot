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
 * Simple bit-bang UART using timer for periodic interrupts and direct GPIO control.
 * Note: due to interrupt latency and CPU overhead, reliable operation is only
 * guaranteed up to 19200 baud. Higher baud rates are not supported.
 */

#include <drivers/drv_bitbang_uart.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/atomic.h>
#include <drivers/drv_hrt.h>

#include <px4_arch/io_timer.h>
#include <board_config.h>

#include <stm32_tim.h>
#include <stm32_gpio.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <string.h>
#include <errno.h>

#if !defined(CONFIG_ARCH_CHIP_STM32F7) && !defined(CONFIG_ARCH_CHIP_STM32H7)
# error "Not supported on this MCU, requires STM32 based MCU"
#endif

// Timer interrupt sources
#define GTIM_SR_UIF   (1 << 0)
#define GTIM_DIER_UIE (1 << 0)

// UART frame format: 8N1 (8 data bits, no parity, 1 stop bit)
#define MAX_UART_CHANNELS 8
#define UART_TOTAL_BITS 10  // Start + 8 data + Stop
#define TX_BUFFER_SIZE 512  // Circular buffer for queued bytes
#define RX_BUFFER_SIZE 512  // Circular buffer for received bytes

typedef struct {
	// Configuration
	uint8_t channel;
	uint32_t gpio_pin;
	uint32_t bit_time_ticks;
	bool initialized;
	bool single_wire; // true = half-duplex (TX and RX share one GPIO); false = full-duplex (not yet implemented)
	struct stm32_tim_dev_s *timer;

	// TX state
	px4::atomic_bool tx_busy;
	uint16_t tx_shift_reg;
	uint8_t tx_bit_count;

	// TX buffer (circular queue)
	uint8_t tx_buffer[TX_BUFFER_SIZE];
	volatile uint16_t tx_head;  // Write position
	volatile uint16_t tx_tail;  // Read position

	// RX state
	volatile bool rx_in_progress;
	volatile uint16_t rx_shift_reg;
	volatile uint8_t rx_bit_count;

	// RX buffer (circular queue)
	uint8_t rx_buffer[RX_BUFFER_SIZE];
	volatile uint16_t rx_head;  // Write position (interrupt)
	volatile uint16_t rx_tail;  // Read position (user)

} bitbang_uart_t;

// Single UART instance (only one channel active at a time)
static bitbang_uart_t uart = {};

// Forward declarations
static int gpio_rx_interrupt_callback(int irq, void *context, void *arg);

// Helper function to enable/disable GPIO interrupt for RX
static int set_gpio_interrupt(bool enable)
{
	if (enable) {
		// Stop timer
		STM32_TIM_SETMODE(uart.timer, STM32_TIM_MODE_DISABLED);

		px4_arch_gpiowrite(uart.gpio_pin, true);
		// Configure new GPIO as input with pullup for RX
		px4_arch_configgpio(PX4_MAKE_GPIO_INPUT(uart.gpio_pin));

		// Enable falling edge interrupt for start bit detection
		return px4_arch_gpiosetevent(PX4_MAKE_GPIO_EXTI(uart.gpio_pin), false, true, true, &gpio_rx_interrupt_callback, &uart);

	} else {
		// Disable interrupt
		return px4_arch_gpiosetevent(PX4_MAKE_GPIO_EXTI(uart.gpio_pin), false, false, false, nullptr, nullptr);
	}
}

// TIM update interrupt handler - fires every bit period
static int timer_update_isr(int, void *, void *)
{
	// Clear update interrupt flag
	STM32_TIM_ACKINT(uart.timer, GTIM_SR_UIF);

	// Handle TX
	if (uart.tx_busy.load()) {
		// Get current bit value from shift register
		bool bit_value = (uart.tx_shift_reg >> uart.tx_bit_count) & 0x01;

		// Set GPIO directly
		px4_arch_gpiowrite(uart.gpio_pin, bit_value);

		uart.tx_bit_count++;

		// Check if transmission complete
		if (uart.tx_bit_count >= UART_TOTAL_BITS) {
			// Check if there's more data in the buffer
			if (uart.tx_tail != uart.tx_head) {
				// Start next byte immediately without returning to scheduler
				uint8_t next_byte = uart.tx_buffer[uart.tx_tail];
				uart.tx_tail = (uart.tx_tail + 1) % TX_BUFFER_SIZE;

				// Build frame for next byte
				uart.tx_shift_reg = 0;
				uart.tx_shift_reg |= (0 << 0);           // Start bit
				uart.tx_shift_reg |= (next_byte << 1);   // Data bits
				uart.tx_shift_reg |= (1 << 9);           // Stop bit
				uart.tx_bit_count = 0;
				// tx_busy stays true

			} else {
				// No more data, stop transmitting
				uart.tx_busy.store(false);

				// Clear RX buffer
				uart.rx_tail = uart.rx_head;

				// Re-enable RX interrupt after transmission
				set_gpio_interrupt(true);
			}
		}
	}

	// Handle RX
	else if (uart.rx_in_progress) {
		// Sample current pin state
		bool bit_value = px4_arch_gpioread(uart.gpio_pin);

		// Accumulate all bits as seen on the wire
		if (bit_value) {
			uart.rx_shift_reg |= (1 << uart.rx_bit_count);
		}

		uart.rx_bit_count++;

		// Check if we've received the complete frame (start + 8 data + stop)
		if (uart.rx_bit_count >= UART_TOTAL_BITS) {
			// Extract data bits (bits 1-8) and reverse them (LSB-first on wire -> MSB-first in byte)
			const uint8_t received_byte = (uart.rx_shift_reg >> 1) & 0xFF;  // Skip start bit, get bits 1-8

			uart.rx_in_progress = false;

			// Store byte in RX buffer
			uint16_t next_head = (uart.rx_head + 1) % RX_BUFFER_SIZE;

			if (next_head != uart.rx_tail) {
				// Buffer not full, store byte
				uart.rx_buffer[uart.rx_head] = received_byte;
				uart.rx_head = next_head;
			}

			// If buffer full, drop byte (overrun)

			// Re-enable start bit detection
			set_gpio_interrupt(true);
		}
	}

	return OK;
}

// GPIO interrupt handler for RX - detects start bit only
static int gpio_rx_interrupt_callback(int irq, void *context, void *arg)
{
	// Start bit detection (high to low transition)
	if (!uart.rx_in_progress) {
		// Disable further start bit detection until this byte completes
		set_gpio_interrupt(false);

		uart.rx_in_progress = true;
		uart.rx_shift_reg = 0;
		uart.rx_bit_count = 0;

		// Configure timer to sample in middle of each bit period
		STM32_TIM_SETCOUNTER(uart.timer, uart.bit_time_ticks / 2);

		// Clear any pending update interrupt
		STM32_TIM_ACKINT(uart.timer, GTIM_SR_UIF);

		// Start timer for RX sampling
		STM32_TIM_SETMODE(uart.timer, STM32_TIM_MODE_UP);
	}

	return OK;
}

static bool tim_initialized = false;
// Shared timer initialization (call once)
static int bitbang_uart_init_timer(uint32_t baudrate)
{
	if (tim_initialized) {
		return 0; // Already initialized
	}

	uart.bit_time_ticks = 1000000 / baudrate; // Assume 1MHz timer

	// Initialize timer using NuttX API
	uart.timer = stm32_tim_init(CONFIG_UART_BITBANG_TIMER);

	if (uart.timer == nullptr) {
		PX4_ERR("Failed to initialize timer %d", CONFIG_UART_BITBANG_TIMER);
		return -1;
	}

	// Set timer to 1MHz clock
	STM32_TIM_SETCLOCK(uart.timer, 1000000);

	// Set period for baud rate
	STM32_TIM_SETPERIOD(uart.timer, uart.bit_time_ticks - 1);

	// Attach interrupt handler for update event
	STM32_TIM_SETISR(uart.timer, timer_update_isr, nullptr, 0);

	// Enable update interrupt
	STM32_TIM_ENABLEINT(uart.timer, GTIM_DIER_UIE);

	tim_initialized = true;

	return 0;
}

int bitbang_uart_init(uint32_t baudrate, bool single_wire)
{
	if (!single_wire) {
		PX4_ERR("bitbang_uart: full-duplex (dual-wire) mode is not yet implemented");
		return -ENOSYS;
	}

	// Initialize timer once (shared across all channels)
	int ret = bitbang_uart_init_timer(baudrate);

	if (ret != 0) {
		return ret;
	}

	// Initialize structure (no channel yet)
	uart.channel = 0xFF; // Mark as no channel selected
	uart.initialized = true;
	uart.single_wire = single_wire;

	// Set all ESC signal pins (channels 0-3) to high (idle state)
	for (uint8_t ch = 0; ch < MAX_UART_CHANNELS; ch++) {
		uint32_t gpio_pin = io_timer_channel_get_gpio_output(ch);
		PX4_DEBUG("bitbang_uart_init: channel %u gpio_pin=0x%08lx", ch, (unsigned long)gpio_pin);

		if (gpio_pin == 0) {
			PX4_WARN("bitbang_uart_init: channel %u has no GPIO pin (not in timer_config?)", ch);
			continue;
		}

		px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_SET(gpio_pin));
	}

	return 0;
}

int bitbang_uart_switch_channel(uint8_t channel)
{
	if (channel >= MAX_UART_CHANNELS) {
		return -EINVAL;
	}

	if (!uart.initialized) {
		return -EINVAL;
	}

	// If already on this channel, nothing to do
	if (uart.channel == channel) {
		return 0;
	}

	// Disable previous channel's RX interrupt if a channel was active
	if (uart.channel != 0xFF) {
		set_gpio_interrupt(false);

		// Wait for any ongoing TX to complete
		while (uart.tx_busy.load()) {
			px4_usleep(10);
		}

		// Stop the timer and clear any pending interrupt so a remaining RX
		// transfer cannot fire and overwrite the buffer state
		STM32_TIM_SETMODE(uart.timer, STM32_TIM_MODE_DISABLED);
		STM32_TIM_ACKINT(uart.timer, GTIM_SR_UIF);
	}

	// Reset TX/RX buffers to avoid stale data from the previous channel
	uart.tx_head = 0;
	uart.tx_tail = 0;
	uart.rx_head = 0;
	uart.rx_tail = 0;
	uart.rx_in_progress = false;

	// Update to new channel
	uart.channel = channel;
	uart.gpio_pin = io_timer_channel_get_gpio_output(channel);
	PX4_DEBUG("bitbang_uart_switch_channel: channel %u gpio_pin=0x%08lx", channel, (unsigned long)uart.gpio_pin);

	if (uart.gpio_pin == 0) {
		PX4_ERR("bitbang_uart_switch_channel: channel %u has no GPIO pin", channel);
		return -EINVAL;
	}

	// Set up interrupt on falling edge for start bit detection
	int ret = set_gpio_interrupt(true);

	if (ret != PX4_OK) {
		PX4_ERR("Failed to setup GPIO interrupt for channel %d", channel);
		return ret;
	}

	PX4_DEBUG("Switched to channel %d", channel);

	return 0;
}

int bitbang_uart_deinit()
{
	if (!uart.initialized) {
		return -EINVAL;
	}

	// Disable RX interrupt if a channel is active
	if (uart.channel != 0xFF) {
		set_gpio_interrupt(false);
	}

	// Wait for TX to complete
	while (uart.tx_busy.load()) {
		px4_usleep(100);
	}

	// Disable timer interrupts
	STM32_TIM_DISABLEINT(uart.timer, GTIM_DIER_UIE);

	// Detach interrupt handler
	STM32_TIM_SETISR(uart.timer, nullptr, nullptr, 0);

	// Stop timer
	STM32_TIM_SETMODE(uart.timer, STM32_TIM_MODE_DISABLED);

	// Deinitialize timer
	stm32_tim_deinit(uart.timer);
	uart.timer = nullptr;

	uart.initialized = false;
	tim_initialized = false;
	uart.channel = 0xFF;

	return 0;
}

static int bitbang_uart_write_byte(uint8_t channel, uint8_t byte)
{
	if (!uart.initialized) {
		return -EINVAL;
	}

	// Switch to requested channel if needed
	int ret = bitbang_uart_switch_channel(channel);

	if (ret != 0) {
		return ret;
	}

	// Check if buffer is full
	uint16_t next_head = (uart.tx_head + 1) % TX_BUFFER_SIZE;

	while (next_head == uart.tx_tail) {
		// Buffer full, wait
		px4_usleep(10);
	}

	// Add byte to buffer
	uart.tx_buffer[uart.tx_head] = byte;
	uart.tx_head = next_head;

	// If not currently transmitting, start transmission
	if (!uart.tx_busy.load()) {
		// In single-wire mode, disable RX interrupt during TX (shared GPIO)
		if (uart.single_wire) {
			set_gpio_interrupt(false);
		}

		// Reconfigure GPIO as output for transmission
		px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_SET(uart.gpio_pin));

		// Get first byte from buffer
		uint8_t first_byte = uart.tx_buffer[uart.tx_tail];
		uart.tx_tail = (uart.tx_tail + 1) % TX_BUFFER_SIZE;

		// Build frame: START(0) + DATA(8 bits) + STOP(1)
		uart.tx_shift_reg = 0;
		uart.tx_shift_reg |= (0 << 0);           // Start bit
		uart.tx_shift_reg |= (first_byte << 1);  // Data bits
		uart.tx_shift_reg |= (1 << 9);           // Stop bit

		uart.tx_bit_count = 0;
		uart.tx_busy.store(true);

		// Reset timer counter
		STM32_TIM_SETCOUNTER(uart.timer, 0);

		// Clear any pending update interrupt
		STM32_TIM_ACKINT(uart.timer, GTIM_SR_UIF);

		// Start timer
		STM32_TIM_SETMODE(uart.timer, STM32_TIM_MODE_UP);
	}

	return 0;
}

int bitbang_uart_write(uint8_t channel, const uint8_t *data, size_t length)
{
	if (!uart.initialized) {
		return -EINVAL;
	}

	if (length == 0) {
		return -EINVAL;
	}

	// Switch to requested channel if needed
	int ret = bitbang_uart_switch_channel(channel);

	if (ret != 0) {
		return ret;
	}

	for (size_t i = 0; i < length; i++) {
		ret = bitbang_uart_write_byte(channel, data[i]);

		if (ret < 0) {
			return ret;
		}
	}

	// Wait for last byte to complete
	while (uart.tx_busy.load()) {
		px4_usleep(10);
	}

	return 0;
}

int bitbang_uart_read(uint8_t channel, uint8_t *data, size_t length, uint32_t timeout_us)
{
	if (!uart.initialized || !data || length == 0) {
		return -EINVAL;
	}

	// Switch to requested channel if needed
	int ret = bitbang_uart_switch_channel(channel);

	if (ret != 0) {
		return ret;
	}

	hrt_abstime deadline = hrt_absolute_time() + timeout_us;
	size_t bytes_read = 0;

	while (bytes_read < length) {
		// Check for timeout
		if (timeout_us > 0 && hrt_absolute_time() >= deadline) {
			break; // Return partial read on timeout
		}

		// Check if data available
		if (uart.rx_tail != uart.rx_head) {
			// Read byte from buffer
			data[bytes_read++] = uart.rx_buffer[uart.rx_tail];
			uart.rx_tail = (uart.rx_tail + 1) % RX_BUFFER_SIZE;

		} else {
			// No data available, yield briefly
			px4_usleep(10);
		}
	}

	return bytes_read;
}
