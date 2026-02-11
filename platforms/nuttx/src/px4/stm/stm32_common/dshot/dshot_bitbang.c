/****************************************************************************
 *
 * Copyright (C) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in
 *  the documentation and/or other materials provided with the
 *  distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *  used to endorse or promote products derived from this software
 *  without specific prior written permission.
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
 * Bitbang DShot implementation.
 *
 * Uses a pacer timer to trigger DMA to GPIO BSRR/IDR registers instead of
 * timer DMA burst to CCR registers. This captures all motors on a GPIO port
 * simultaneously for bidirectional DShot, giving every motor RPM data every
 * cycle instead of round-robin.
 */

#if (CONFIG_STM32_HAVE_IP_DMA_V1)
// Do nothing. IP DMA V1 MCUs are not supported.
#else

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <stm32_dma.h>
#include <stm32_tim.h>
#include <stm32_gpio.h>
#include <px4_arch/dshot.h>
#include <px4_arch/io_timer.h>
#include <drivers/drv_dshot.h>
#include <px4_platform_common/log.h>
#include <stdio.h>
#include <string.h>

// DShot protocol definitions
#define ONE_MOTOR_DATA_SIZE         16u
#define MOTOR_PWM_BIT_1             14u
#define MOTOR_PWM_BIT_0             7u
#define DSHOT_THROTTLE_POSITION     5u
#define DSHOT_TELEMETRY_POSITION    4u
#define NIBBLES_SIZE                4u
#define DSHOT_NUMBER_OF_NIBBLES     3u
#define SUBPERIODS_PER_BIT          DSHOT_MOTOR_PWM_BIT_WIDTH  // 20

// Bitbang-specific
#define MAX_PORT_GROUPS             4u
#define MAX_PINS_PER_GROUP          8u
#define BSRR_BUFFER_SIZE            (ONE_MOTOR_DATA_SIZE * SUBPERIODS_PER_BIT + 1)  // 321 words
#define CAPTURE_SAMPLE_COUNT        512u

// DMA stream configuration
#define DSHOT_DMA_SCR_TX (DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | \
			  DMA_SCR_MINC | DMA_SCR_DIR_M2P | DMA_SCR_TCIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

#define DSHOT_DMA_SCR_RX (DMA_SCR_PRIHI | DMA_SCR_MSIZE_16BITS | DMA_SCR_PSIZE_16BITS | \
			  DMA_SCR_MINC | DMA_SCR_DIR_P2M | DMA_SCR_TCIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

#if defined(CONFIG_ARMV7M_DCACHE)
#  define DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
#  define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)
#else
#  define DMA_ALIGN_UP(n) (n)
#endif

// Timer register macros (from io_timer.c, local to this file)
#define REG(_tmr, _reg)     _REG32(io_timers[_tmr].base, _reg)
#define rCR1(_tmr)          REG(_tmr, STM32_GTIM_CR1_OFFSET)
#define rCR2(_tmr)          REG(_tmr, STM32_GTIM_CR2_OFFSET)
#define rDIER(_tmr)         REG(_tmr, STM32_GTIM_DIER_OFFSET)
#define rSR(_tmr)           REG(_tmr, STM32_GTIM_SR_OFFSET)
#define rEGR(_tmr)          REG(_tmr, STM32_GTIM_EGR_OFFSET)
#define rCNT(_tmr)          REG(_tmr, STM32_GTIM_CNT_OFFSET)
#define rPSC(_tmr)          REG(_tmr, STM32_GTIM_PSC_OFFSET)
#define rARR(_tmr)          REG(_tmr, STM32_GTIM_ARR_OFFSET)

// Port group: one per GPIO port that has motor pins
typedef struct port_group_t {
	uint32_t gpio_base;                          // GPIO port base address
	uint8_t  port_index;                         // NuttX port index (0=A, 7=H, 8=I)
	uint32_t pin_mask;                           // Bitmask of motor pins on this port
	uint32_t moder_mask;                         // MODER register mask
	uint32_t moder_output;                       // MODER value for output mode (01 per pin)
	uint8_t  pin_count;                          // Number of motor pins on this port
	uint8_t  pin_numbers[MAX_PINS_PER_GROUP];    // Pin numbers (0-15)
	uint8_t  output_channels[MAX_PINS_PER_GROUP]; // Corresponding output channel indices
	uint8_t  timer_index;                        // Pacer timer index (into io_timers[])
	DMA_HANDLE dma_handle;
	bool     bidirectional;
	bool     cycle_complete;
	uint8_t  pg_index;                           // Self-index for callback arg
} port_group_t;

static port_group_t _port_groups[MAX_PORT_GROUPS] = {};
static uint8_t _num_port_groups = 0;

// Per-channel lookup tables
static uint8_t _channel_to_port_group[MAX_TIMER_IO_CHANNELS];
static uint8_t _channel_to_pin[MAX_TIMER_IO_CHANNELS];
static bool _channel_initialized[MAX_TIMER_IO_CHANNELS] = {};

// Stored DShot packets (set by dshot_motor_data_set, consumed by up_dshot_trigger)
static uint16_t _channel_packet[MAX_TIMER_IO_CHANNELS] = {};

// DMA buffers (cache-aligned)
static uint32_t _bsrr_buffer[MAX_PORT_GROUPS][BSRR_BUFFER_SIZE]
	px4_cache_aligned_data() = {};
static uint16_t _idr_buffer[MAX_PORT_GROUPS][CAPTURE_SAMPLE_COUNT]
	px4_cache_aligned_data() = {};

// GCR decode table (same as dshot.c)
static const uint32_t gcr_decode[32] = {
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x9, 0xA, 0xB, 0x0, 0xD, 0xE, 0xF,
	0x0, 0x0, 0x2, 0x3, 0x0, 0x5, 0x6, 0x7,
	0x0, 0x0, 0x8, 0x1, 0x0, 0x4, 0xC, 0x0
};

// Global state
static uint32_t _bdshot_channel_mask = 0xFFFFFFFF;
static uint32_t _dshot_frequency = 0;
static bool     _edt_enabled = false;
static bool     _armed = false;

// ESC boot delay
static const uint64_t ESC_BOOT_DELAY_US = 3000000;

// Online flags
#define BDSHOT_OFFLINE_COUNT 200
static bool _bdshot_online[MAX_TIMER_IO_CHANNELS] = {};
static bool _bdshot_capture_supported[MAX_TIMER_IO_CHANNELS] = {};
static int _consecutive_failures[MAX_TIMER_IO_CHANNELS] = {};
static int _consecutive_successes[MAX_TIMER_IO_CHANNELS] = {};

typedef struct {
	int32_t erpm;
	bool ready;
	float rate_hz;
	uint64_t last_timestamp;
} erpm_data_t;

typedef struct {
	uint8_t value;
	bool ready;
	float rate_hz;
	uint64_t last_timestamp;
} edt_data_t;

// Adaptive base interval per channel (samples per GCR bit)
// At 20 sub-periods per DShot bit and GCR rate = DShot * 5/4, we get 16 samples per GCR bit
static float _base_interval[MAX_TIMER_IO_CHANNELS] = { [0 ...(MAX_TIMER_IO_CHANNELS - 1)] = 16.0f };

static erpm_data_t _erpms[MAX_TIMER_IO_CHANNELS] = {};
static edt_data_t _edt_temp[MAX_TIMER_IO_CHANNELS] = {};
static edt_data_t _edt_volt[MAX_TIMER_IO_CHANNELS] = {};
static edt_data_t _edt_curr[MAX_TIMER_IO_CHANNELS] = {};

// hrt callback handles for capture scheduling, one per port group
static struct hrt_call _capture_calls[MAX_PORT_GROUPS];

// Decoding status for each channel
static uint32_t read_ok[MAX_TIMER_IO_CHANNELS] = {};
static uint32_t read_fail_crc[MAX_TIMER_IO_CHANNELS] = {};

// Forward declarations
static void tx_dma_complete_callback(DMA_HANDLE handle, uint8_t status, void *arg);
static void capture_start_callback(void *arg);
static void capture_complete_callback(void *arg);
static void process_capture_results(uint8_t pg_index);
static uint32_t convert_idr_to_bitstream(uint8_t pg_index, uint8_t pin_num, uint8_t output_channel);
static void decode_dshot_telemetry(uint32_t payload, struct BDShotTelemetry *packet);
static float calculate_rate_hz(uint64_t last_timestamp, float last_rate_hz, uint64_t timestamp);

static void gpio_set_mode_output(port_group_t *pg)
{
	modifyreg32(pg->gpio_base + STM32_GPIO_MODER_OFFSET, pg->moder_mask, pg->moder_output);
}

static void gpio_set_mode_input(port_group_t *pg)
{
	modifyreg32(pg->gpio_base + STM32_GPIO_MODER_OFFSET, pg->moder_mask, 0);
}

static void gpio_set_idle_state(port_group_t *pg)
{
	if (pg->bidirectional) {
		// BDShot idles HIGH - set all motor pins
		putreg32(pg->pin_mask, pg->gpio_base + STM32_GPIO_BSRR_OFFSET);

	} else {
		// Normal DShot idles LOW - reset all motor pins
		putreg32(pg->pin_mask << 16, pg->gpio_base + STM32_GPIO_BSRR_OFFSET);
	}
}

static void timer_configure_pacer(uint8_t timer_index)
{
	// Enable timer clock
	modifyreg32(io_timers[timer_index].clock_register, 0, io_timers[timer_index].clock_bit);

	// Disable timer before configuring
	rCR1(timer_index) = 0;

	// Configure prescaler: timer clock / (dshot_freq * SUBPERIODS_PER_BIT)
	uint32_t timer_freq = io_timers[timer_index].clock_freq;
	uint32_t target_freq = _dshot_frequency * SUBPERIODS_PER_BIT;
	rPSC(timer_index) = (timer_freq / target_freq) - 1;

	// Auto-reload: one DMA transfer per timer update
	rARR(timer_index) = SUBPERIODS_PER_BIT - 1;

	test

	// Generate update event to load prescaler
	rEGR(timer_index) = GTIM_EGR_UG;

	// Clear any pending update interrupt/DMA flags
	rSR(timer_index) = 0;
}

static void timer_enable(uint8_t timer_index)
{
	rCR1(timer_index) |= GTIM_CR1_CEN;
}

static void timer_disable(uint8_t timer_index)
{
	rCR1(timer_index) &= ~GTIM_CR1_CEN;
	rCNT(timer_index) = 0;
}

static void timer_set_ude(uint8_t timer_index, bool enable)
{
	if (enable) {
		rDIER(timer_index) |= ATIM_DIER_UDE;

	} else {
		rDIER(timer_index) &= ~ATIM_DIER_UDE;
	}
}

static port_group_t *find_port_group(uint8_t port_index)
{
	for (uint8_t i = 0; i < _num_port_groups; i++) {
		if (_port_groups[i].port_index == port_index) {
			return &_port_groups[i];
		}
	}

	return NULL;
}

static port_group_t *find_or_create_port_group(uint8_t port_index)
{
	port_group_t *pg = find_port_group(port_index);

	if (pg) {
		return pg;
	}

	if (_num_port_groups >= MAX_PORT_GROUPS) {
		return NULL;
	}

	pg = &_port_groups[_num_port_groups];
	memset(pg, 0, sizeof(*pg));
	pg->port_index = port_index;
	pg->gpio_base = g_gpiobase[port_index];
	pg->timer_index = UINT8_MAX;
	pg->cycle_complete = true;
	pg->pg_index = _num_port_groups;
	_num_port_groups++;

	return pg;
}

int up_dshot_init(uint32_t channel_mask, uint32_t bdshot_channel_mask, unsigned dshot_pwm_freq, bool edt_enable)
{
	_dshot_frequency = dshot_pwm_freq;
	_bdshot_channel_mask = bdshot_channel_mask;
	_edt_enabled = edt_enable;
	_num_port_groups = 0;

	if (bdshot_channel_mask) {
		PX4_INFO("BDShot bitbang enabled");
	}

	int32_t channels_init_mask = 0;

	// Phase 1: Discover port groups from timer_io_channels[]
	// Collect available timer indices per port group for assignment
	uint8_t pg_available_timers[MAX_PORT_GROUPS][MAX_IO_TIMERS];
	uint8_t pg_available_timer_count[MAX_PORT_GROUPS] = {};

	for (uint8_t ch = 0; ch < MAX_TIMER_IO_CHANNELS; ch++) {
		if (!(channel_mask & (1 << ch))) {
			continue;
		}

		uint32_t gpio_out = timer_io_channels[ch].gpio_out;

		if (gpio_out == 0) {
			continue;
		}

		uint8_t timer_index = timer_io_channels[ch].timer_index;

		if (timer_index >= MAX_IO_TIMERS || io_timers[timer_index].dshot.dma_map_up == 0) {
			// No DMA UP mapping for this timer - skip
			continue;
		}

		uint8_t port_index = (gpio_out & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
		uint8_t pin_num = (gpio_out & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

		port_group_t *pg = find_or_create_port_group(port_index);

		if (!pg) {
			PX4_WARN("Too many port groups, skipping channel %u", ch);
			continue;
		}

		if (pg->pin_count >= MAX_PINS_PER_GROUP) {
			PX4_WARN("Too many pins on port group %u, skipping channel %u", pg->pg_index, ch);
			continue;
		}

		// Add pin to port group
		pg->pin_mask |= (1u << pin_num);
		pg->moder_mask |= (3u << (pin_num * 2));
		pg->moder_output |= (1u << (pin_num * 2));  // 01 = output mode
		pg->pin_numbers[pg->pin_count] = pin_num;
		pg->output_channels[pg->pin_count] = ch;
		pg->pin_count++;

		// Track channel-to-port-group mapping
		_channel_to_port_group[ch] = pg->pg_index;
		_channel_to_pin[ch] = pin_num;

		// Mark BDShot
		if (bdshot_channel_mask & (1 << ch)) {
			pg->bidirectional = true;
		}

		// Track available timers for this port group (deduplicated)
		bool timer_already_listed = false;

		for (uint8_t t = 0; t < pg_available_timer_count[pg->pg_index]; t++) {
			if (pg_available_timers[pg->pg_index][t] == timer_index) {
				timer_already_listed = true;
				break;
			}
		}

		if (!timer_already_listed && pg_available_timer_count[pg->pg_index] < MAX_IO_TIMERS) {
			pg_available_timers[pg->pg_index][pg_available_timer_count[pg->pg_index]] = timer_index;
			pg_available_timer_count[pg->pg_index]++;
		}
	}

	// Phase 2: Assign pacer timers (greedy, fewest-alternatives-first)
	// Sort port groups by number of available timers (ascending) - simple selection sort
	uint8_t pg_order[MAX_PORT_GROUPS];

	for (uint8_t i = 0; i < _num_port_groups; i++) {
		pg_order[i] = i;
	}

	for (uint8_t i = 0; i < _num_port_groups; i++) {
		for (uint8_t j = i + 1; j < _num_port_groups; j++) {
			if (pg_available_timer_count[pg_order[j]] < pg_available_timer_count[pg_order[i]]) {
				uint8_t tmp = pg_order[i];
				pg_order[i] = pg_order[j];
				pg_order[j] = tmp;
			}
		}
	}

	bool timer_assigned[MAX_IO_TIMERS] = {};

	for (uint8_t i = 0; i < _num_port_groups; i++) {
		uint8_t pg_idx = pg_order[i];
		port_group_t *pg = &_port_groups[pg_idx];

		for (uint8_t t = 0; t < pg_available_timer_count[pg_idx]; t++) {
			uint8_t ti = pg_available_timers[pg_idx][t];

			if (!timer_assigned[ti]) {
				pg->timer_index = ti;
				timer_assigned[ti] = true;
				break;
			}
		}

		if (pg->timer_index == UINT8_MAX) {
			PX4_ERR("No timer available for port group %u (port %c)", pg_idx, 'A' + pg->port_index);
			continue;
		}
	}

	// Phase 3: Initialize GPIO pins and pacer timers
	for (uint8_t pg_idx = 0; pg_idx < _num_port_groups; pg_idx++) {
		port_group_t *pg = &_port_groups[pg_idx];

		if (pg->timer_index == UINT8_MAX) {
			continue;
		}

		io_timer_channel_mode_t mode = pg->bidirectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot;

		// Initialize each channel through io_timer to claim resources
		for (uint8_t p = 0; p < pg->pin_count; p++) {
			uint8_t ch = pg->output_channels[p];
			int ret = io_timer_channel_init(ch, mode, NULL, NULL);

			if (ret != OK) {
				PX4_WARN("io_timer_channel_init %u failed (%d)", ch, ret);
				continue;
			}

			_channel_initialized[ch] = true;
			channels_init_mask |= (1 << ch);

			// Mark BDShot capture supported (bitbang captures all simultaneously)
			if (bdshot_channel_mask & (1 << ch)) {
				_bdshot_capture_supported[ch] = true;
			}
		}

		// Override GPIO MODER from AF (10) to GP output (01)
		gpio_set_mode_output(pg);
		gpio_set_idle_state(pg);

		// Configure pacer timer
		timer_configure_pacer(pg->timer_index);

		PX4_INFO("Port group %u: GPIO port %c, %u pins, pin_mask=0x%04lx, pacer=TIM%u (idx %u)",
			 pg_idx, 'A' + pg->port_index, pg->pin_count,
			 (unsigned long)pg->pin_mask, pg->timer_index + 1, pg->timer_index);
	}

	return channels_init_mask;
}

void dshot_motor_data_set(uint8_t channel, uint16_t data, bool telemetry)
{
	if (channel >= MAX_TIMER_IO_CHANNELS || !_channel_initialized[channel]) {
		return;
	}

	// Build the 16-bit packet and store for later BSRR buffer construction in trigger()
	uint16_t packet = 0;
	uint16_t checksum = 0;

	packet |= data << DSHOT_THROTTLE_POSITION;
	packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

	uint16_t csum_data = packet;
	csum_data >>= NIBBLES_SIZE;

	for (uint8_t i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
		checksum ^= (csum_data & 0x0F);
		csum_data >>= NIBBLES_SIZE;
	}

	if (_bdshot_channel_mask & (1 << channel)) {
		packet |= ((~checksum) & 0x0F);

	} else {
		packet |= ((checksum) & 0x0F);
	}

	_channel_packet[channel] = packet;
}

// Build BSRR words for a single motor into the port group's shared buffer
static void build_bsrr_for_channel(uint8_t channel)
{
	uint8_t pg_index = _channel_to_port_group[channel];
	uint8_t pin_num = _channel_to_pin[channel];
	port_group_t *pg = &_port_groups[pg_index];
	uint32_t *buffer = _bsrr_buffer[pg_index];
	uint16_t packet = _channel_packet[channel];

	uint32_t set_mask, reset_mask;

	if (pg->bidirectional) {
		// BDShot (inverted): idle HIGH, pulse LOW
		set_mask = (1u << (pin_num + 16));  // BSRR reset (drive LOW)
		reset_mask = (1u << pin_num);       // BSRR set (drive HIGH)

	} else {
		// Normal DShot: idle LOW, pulse HIGH
		set_mask = (1u << pin_num);         // BSRR set (drive HIGH)
		reset_mask = (1u << (pin_num + 16)); // BSRR reset (drive LOW)
	}

	for (uint8_t bit = 0; bit < ONE_MOTOR_DATA_SIZE; bit++) {
		unsigned base = bit * SUBPERIODS_PER_BIT;
		bool is_one = (packet >> (15 - bit)) & 1;  // MSB first
		unsigned duty_point = is_one ? MOTOR_PWM_BIT_1 : MOTOR_PWM_BIT_0;

		buffer[base] |= set_mask;              // Start of bit: begin pulse
		buffer[base + duty_point] |= reset_mask; // End of duty cycle: end pulse
	}
}

void up_dshot_trigger(void)
{
	// Check all port groups are ready
	for (uint8_t i = 0; i < _num_port_groups; i++) {
		if (!_port_groups[i].cycle_complete) {
			PX4_WARN("Bitbang cycle not complete for port group %u", i);
			return;
		}
	}

	for (uint8_t pg_idx = 0; pg_idx < _num_port_groups; pg_idx++) {
		port_group_t *pg = &_port_groups[pg_idx];

		if (pg->timer_index == UINT8_MAX || pg->pin_count == 0) {
			continue;
		}

		// Zero BSRR buffer, then rebuild from stored packets
		memset(_bsrr_buffer[pg_idx], 0, sizeof(_bsrr_buffer[pg_idx]));

		for (uint8_t p = 0; p < pg->pin_count; p++) {
			uint8_t ch = pg->output_channels[p];

			if (_channel_initialized[ch]) {
				build_bsrr_for_channel(ch);
			}
		}

		// Add trailing word to return pins to idle state
		if (pg->bidirectional) {
			_bsrr_buffer[pg_idx][BSRR_BUFFER_SIZE - 1] = pg->pin_mask;  // SET = idle HIGH

		} else {
			_bsrr_buffer[pg_idx][BSRR_BUFFER_SIZE - 1] = pg->pin_mask << 16;  // RESET = idle LOW
		}

		if (pg->bidirectional) {
			pg->cycle_complete = false;
		}

		// Ensure GPIO is in output mode
		gpio_set_mode_output(pg);

		// Flush dcache
		up_clean_dcache((uintptr_t)_bsrr_buffer[pg_idx],
				(uintptr_t)_bsrr_buffer[pg_idx] + DMA_ALIGN_UP(sizeof(_bsrr_buffer[pg_idx])));

		// Free any previous DMA handle
		if (pg->dma_handle != NULL) {
			stm32_dmastop(pg->dma_handle);
			stm32_dmafree(pg->dma_handle);
			pg->dma_handle = NULL;
		}

		// Allocate DMA
		pg->dma_handle = stm32_dmachannel(io_timers[pg->timer_index].dshot.dma_map_up);

		if (pg->dma_handle == NULL) {
			PX4_WARN("DMA allocation failed for port group %u", pg_idx);
			pg->cycle_complete = true;
			continue;
		}

		// Setup DMA: memory -> GPIO BSRR
		px4_stm32_dmasetup(pg->dma_handle,
				   pg->gpio_base + STM32_GPIO_BSRR_OFFSET,
				   (uint32_t)_bsrr_buffer[pg_idx],
				   BSRR_BUFFER_SIZE,
				   DSHOT_DMA_SCR_TX);

		// Clear UDE before starting
		timer_set_ude(pg->timer_index, false);

		// Reset counter
		rCNT(pg->timer_index) = 0;
		rSR(pg->timer_index) = 0;

		// Start DMA with callback
		stm32_dmastart(pg->dma_handle, tx_dma_complete_callback, &pg->pg_index, false);

		// Enable UDE then start timer
		timer_set_ude(pg->timer_index, true);
		timer_enable(pg->timer_index);
	}
}

static void tx_dma_complete_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	uint8_t pg_index = *((uint8_t *)arg);
	port_group_t *pg = &_port_groups[pg_index];

	// Stop DMA and timer
	stm32_dmastop(pg->dma_handle);
	stm32_dmafree(pg->dma_handle);
	pg->dma_handle = NULL;
	timer_set_ude(pg->timer_index, false);
	timer_disable(pg->timer_index);

	// For non-BDShot or during ESC boot: done
	if (!pg->bidirectional || (hrt_absolute_time() <= ESC_BOOT_DELAY_US)) {
		if (pg->bidirectional) {
			pg->cycle_complete = true;
		}

		return;
	}

	// Switch GPIO to input mode for capture
	gpio_set_mode_input(pg);

	// Schedule capture start after ESC turnaround time (~30us)
	hrt_call_after(&_capture_calls[pg_index], 30, capture_start_callback, &pg->pg_index);
}

static void capture_start_callback(void *arg)
{
	uint8_t pg_index = *((uint8_t *)arg);
	port_group_t *pg = &_port_groups[pg_index];

	// Zero capture buffer
	memset(_idr_buffer[pg_index], 0, sizeof(_idr_buffer[pg_index]));
	up_clean_dcache((uintptr_t)_idr_buffer[pg_index],
			(uintptr_t)_idr_buffer[pg_index] + DMA_ALIGN_UP(sizeof(_idr_buffer[pg_index])));

	// Allocate DMA (reuse same DMAMUX route)
	pg->dma_handle = stm32_dmachannel(io_timers[pg->timer_index].dshot.dma_map_up);

	if (pg->dma_handle == NULL) {
		PX4_WARN("DMA allocation failed for capture, port group %u", pg_index);
		gpio_set_mode_output(pg);
		gpio_set_idle_state(pg);
		pg->cycle_complete = true;
		return;
	}

	// Setup DMA: GPIO IDR -> memory
	px4_stm32_dmasetup(pg->dma_handle,
			   pg->gpio_base + STM32_GPIO_IDR_OFFSET,
			   (uint32_t)_idr_buffer[pg_index],
			   CAPTURE_SAMPLE_COUNT,
			   DSHOT_DMA_SCR_RX);

	// Clear UDE before starting
	timer_set_ude(pg->timer_index, false);
	rCNT(pg->timer_index) = 0;
	rSR(pg->timer_index) = 0;

	// Start DMA (no callback - we use hrt timer for completion)
	stm32_dmastart(pg->dma_handle, NULL, NULL, false);

	// Enable UDE then start timer
	timer_set_ude(pg->timer_index, true);
	timer_enable(pg->timer_index);

	// Schedule capture complete after GCR frame time + margin
	// GCR frame = 21 bits at DShot rate * 4/5
	// Frame time in us = 21 * (1e6 / (dshot_freq * 5/4)) = 21 * 4e6 / (5 * dshot_freq)
	hrt_abstime frame_us = (21 * 4 * 1000000ULL) / (5 * _dshot_frequency);
	hrt_abstime delay = frame_us + 20; // +20us margin
	hrt_call_after(&_capture_calls[pg_index], delay, capture_complete_callback, &pg->pg_index);
}

static void capture_complete_callback(void *arg)
{
	uint8_t pg_index = *((uint8_t *)arg);
	port_group_t *pg = &_port_groups[pg_index];

	// Stop DMA and timer
	if (pg->dma_handle != NULL) {
		stm32_dmastop(pg->dma_handle);
		stm32_dmafree(pg->dma_handle);
		pg->dma_handle = NULL;
	}

	timer_set_ude(pg->timer_index, false);
	timer_disable(pg->timer_index);

	// Invalidate dcache
	up_invalidate_dcache((uintptr_t)_idr_buffer[pg_index],
			     (uintptr_t)_idr_buffer[pg_index] + DMA_ALIGN_UP(sizeof(_idr_buffer[pg_index])));

	// Process capture for ALL motors on this port (not round-robin)
	process_capture_results(pg_index);

	// Switch GPIO back to output mode and set idle state
	gpio_set_mode_output(pg);
	gpio_set_idle_state(pg);

	pg->cycle_complete = true;
}

static void process_capture_results(uint8_t pg_index)
{
	port_group_t *pg = &_port_groups[pg_index];

	for (uint8_t p = 0; p < pg->pin_count; p++) {
		uint8_t output_channel = pg->output_channels[p];
		uint8_t pin_num = pg->pin_numbers[p];

		if (!(_bdshot_channel_mask & (1 << output_channel))) {
			continue;
		}

		uint32_t value = convert_idr_to_bitstream(pg_index, pin_num, output_channel);

		// Decode RLL
		value = (value ^ (value >> 1));

		// Decode GCR
		uint32_t payload = gcr_decode[value & 0x1f];
		payload |= gcr_decode[(value >> 5) & 0x1f] << 4;
		payload |= gcr_decode[(value >> 10) & 0x1f] << 8;
		payload |= gcr_decode[(value >> 15) & 0x1f] << 12;

		// Calculate checksum
		uint32_t checksum = payload;
		checksum = checksum ^ (checksum >> 8);
		checksum = checksum ^ (checksum >> NIBBLES_SIZE);

		if ((checksum & 0xF) != 0xF) {
			++read_fail_crc[output_channel];

			if (_consecutive_failures[output_channel]++ > BDSHOT_OFFLINE_COUNT) {
				_consecutive_failures[output_channel] = BDSHOT_OFFLINE_COUNT;
				_consecutive_successes[output_channel] = 0;
				_bdshot_online[output_channel] = false;
			}

			continue;
		}

		++read_ok[output_channel];

		if (_consecutive_successes[output_channel]++ > BDSHOT_OFFLINE_COUNT) {
			_consecutive_successes[output_channel] = BDSHOT_OFFLINE_COUNT;
			_consecutive_failures[output_channel] = 0;
			_bdshot_online[output_channel] = true;
		}

		// Convert payload into telemetry type/value
		struct BDShotTelemetry packet = {};
		payload = (payload >> 4) & 0xFFF;
		decode_dshot_telemetry(payload, &packet);

		hrt_abstime now = hrt_absolute_time();

		switch (packet.type) {
		case DSHOT_EDT_ERPM: {
				_erpms[output_channel].erpm = packet.value;
				_erpms[output_channel].ready = true;
				uint64_t last_timestamp = _erpms[output_channel].last_timestamp;
				float last_rate_hz = _erpms[output_channel].rate_hz;
				_erpms[output_channel].rate_hz = calculate_rate_hz(last_timestamp, last_rate_hz, now);
				_erpms[output_channel].last_timestamp = now;
				break;
			}

		case DSHOT_EDT_TEMPERATURE: {
				_edt_temp[output_channel].value = packet.value;
				_edt_temp[output_channel].ready = true;
				uint64_t last_timestamp = _edt_temp[output_channel].last_timestamp;
				float last_rate_hz = _edt_temp[output_channel].rate_hz;
				_edt_temp[output_channel].rate_hz = calculate_rate_hz(last_timestamp, last_rate_hz, now);
				_edt_temp[output_channel].last_timestamp = now;
				break;
			}

		case DSHOT_EDT_VOLTAGE: {
				_edt_volt[output_channel].value = packet.value;
				_edt_volt[output_channel].ready = true;
				uint64_t last_timestamp = _edt_volt[output_channel].last_timestamp;
				float last_rate_hz = _edt_volt[output_channel].rate_hz;
				_edt_volt[output_channel].rate_hz = calculate_rate_hz(last_timestamp, last_rate_hz, now);
				_edt_volt[output_channel].last_timestamp = now;
				break;
			}

		case DSHOT_EDT_CURRENT: {
				_edt_curr[output_channel].value = packet.value;
				_edt_curr[output_channel].ready = true;
				uint64_t last_timestamp = _edt_curr[output_channel].last_timestamp;
				float last_rate_hz = _edt_curr[output_channel].rate_hz;
				_edt_curr[output_channel].rate_hz = calculate_rate_hz(last_timestamp, last_rate_hz, now);
				_edt_curr[output_channel].last_timestamp = now;
				break;
			}

		case DSHOT_EDT_STATE_EVENT:
			break;

		default:
			break;
		}
	}
}

// Convert IDR capture samples to a GCR bitstream for a single pin
static uint32_t convert_idr_to_bitstream(uint8_t pg_index, uint8_t pin_num, uint8_t output_channel)
{
	uint16_t *buffer = _idr_buffer[pg_index];
	uint16_t pin_bit = (1u << pin_num);

	// First pass: extract edges and collect intervals
	uint32_t intervals[64];
	unsigned interval_count = 0;

	// BDShot response starts HIGH (idle), then the ESC pulls LOW to begin.
	// Skip initial idle samples, find the first falling edge.
	unsigned start = 0;

	for (unsigned i = 1; i < CAPTURE_SAMPLE_COUNT; i++) {
		bool prev = (buffer[i - 1] & pin_bit) != 0;
		bool curr = (buffer[i] & pin_bit) != 0;

		if (prev && !curr) {
			// Found first falling edge - start of GCR frame
			start = i;
			break;
		}
	}

	if (start == 0) {
		// No falling edge found
		return 0;
	}

	// Collect edge intervals from the start of the frame
	unsigned last_edge = start;
	bool prev_state = (buffer[start] & pin_bit) != 0;

	for (unsigned i = start + 1; i < CAPTURE_SAMPLE_COUNT; i++) {
		bool state = (buffer[i] & pin_bit) != 0;

		if (state != prev_state) {
			uint32_t interval = i - last_edge;

			if (interval_count < 64) {
				intervals[interval_count++] = interval;
			}

			last_edge = i;
			prev_state = state;
		}
	}

	if (interval_count == 0) {
		return 0;
	}

	// Calibrate base interval
	uint32_t min_interval = intervals[0];

	for (unsigned i = 1; i < interval_count; i++) {
		if (intervals[i] < min_interval) {
			min_interval = intervals[i];
		}
	}

	// Filter out obviously invalid minimum intervals
	if (min_interval < 4 || min_interval > 80) {
		return 0;
	}

	// Update base interval with low-pass filter
	_base_interval[output_channel] = 0.9f * _base_interval[output_channel] + 0.1f * (float)min_interval;

	// Second pass: decode bits using adaptive threshold
	uint32_t value = 0;
	uint32_t high = 1;  // First edge is falling (idle HIGH -> LOW), so first run is HIGH
	unsigned shifted = 0;

	float base = _base_interval[output_channel];

	for (unsigned i = 0; i < interval_count; i++) {
		float interval_f = (float)intervals[i];
		uint32_t bits = (uint32_t)((interval_f / base) + 0.5f);

		if (bits < 1) {
			bits = 1;

		} else if (bits > 4) {
			bits = 4;
		}

		for (unsigned bit = 0; bit < bits; ++bit) {
			value = (value << 1) | high;
			++shifted;
		}

		high = !high;
	}

	// Flexible frame validation
	if (shifted < 18 || shifted > 24) {
		return 0;
	}

	// Pad/trim to 21 bits
	if (shifted < 21) {
		value <<= (21 - shifted);

	} else if (shifted > 21) {
		value >>= (shifted - 21);
	}

	return value;
}

static void decode_dshot_telemetry(uint32_t payload, struct BDShotTelemetry *packet)
{
	bool edt_enabled = _edt_enabled;
	uint32_t mantissa = payload & 0x01FF;
	bool is_telemetry = !(mantissa & 0x0100);

	if (edt_enabled && is_telemetry) {
		packet->type = (payload & 0x0F00) >> 8;
		packet->value = payload & 0x00FF;

	} else {
		uint8_t exponent = ((payload >> 9) & 0x7);
		uint16_t period = (payload & 0x1FF);
		period = period << exponent;

		packet->type = DSHOT_EDT_ERPM;

		if (period == 65408 || period == 0) {
			packet->value = 0;

		} else {
			packet->value = (1000000 * 60 / 100 + period / 2) / period;
		}
	}
}

static float calculate_rate_hz(uint64_t last_timestamp, float last_rate_hz, uint64_t timestamp)
{
	if (last_timestamp == 0 || timestamp <= last_timestamp) {
		return last_rate_hz;
	}

	uint64_t dt_us = timestamp - last_timestamp;
	float instant_rate = 1000000.0f / dt_us;
	float rate_hz = instant_rate * 0.125f + last_rate_hz * 0.875f;
	return rate_hz;
}

int up_dshot_arm(bool armed)
{
	_armed = armed;

	for (uint8_t pg_idx = 0; pg_idx < _num_port_groups; pg_idx++) {
		port_group_t *pg = &_port_groups[pg_idx];

		if (armed) {
			gpio_set_mode_output(pg);
			gpio_set_idle_state(pg);

		} else {
			// Reset all motor pins LOW when disarmed
			gpio_set_mode_output(pg);
			putreg32(pg->pin_mask << 16, pg->gpio_base + STM32_GPIO_BSRR_OFFSET);
		}
	}

	return PX4_OK;
}

int up_bdshot_num_errors(uint8_t channel)
{
	return read_fail_crc[channel];
}

int up_bdshot_get_erpm(uint8_t channel, int *erpm)
{
	if (channel >= MAX_TIMER_IO_CHANNELS || !_channel_initialized[channel]) {
		return PX4_ERROR;
	}

	if (_erpms[channel].ready) {
		*erpm = _erpms[channel].erpm;
		return PX4_OK;
	}

	return PX4_ERROR;
}

int up_bdshot_get_extended_telemetry(uint8_t channel, int type, uint8_t *value)
{
	int result = PX4_ERROR;

	switch (type) {
	case DSHOT_EDT_TEMPERATURE:
		if (_edt_temp[channel].ready) {
			*value = _edt_temp[channel].value;
			_edt_temp[channel].ready = false;
			result = PX4_OK;
		}

		break;

	case DSHOT_EDT_VOLTAGE:
		if (_edt_volt[channel].ready) {
			*value = _edt_volt[channel].value;
			_edt_volt[channel].ready = false;
			result = PX4_OK;
		}

		break;

	case DSHOT_EDT_CURRENT:
		if (_edt_curr[channel].ready) {
			*value = _edt_curr[channel].value;
			_edt_curr[channel].ready = false;
			result = PX4_OK;
		}

		break;

	default:
		break;
	}

	return result;
}

int up_bdshot_channel_online(uint8_t channel)
{
	if (channel >= MAX_TIMER_IO_CHANNELS) {
		return 0;
	}

	return _bdshot_online[channel];
}

int up_bdshot_channel_capture_supported(uint8_t channel)
{
	if (channel >= MAX_TIMER_IO_CHANNELS) {
		return 0;
	}

	return _bdshot_capture_supported[channel];
}

void up_bdshot_status(void)
{
	PX4_INFO("dshot bitbang driver stats:");

	for (uint8_t pg_idx = 0; pg_idx < _num_port_groups; pg_idx++) {
		port_group_t *pg = &_port_groups[pg_idx];
		PX4_INFO("  Port group %u: GPIO port %c, %u pins, pin_mask=0x%04lx, pacer timer idx %u",
			 pg_idx, 'A' + pg->port_index, pg->pin_count,
			 (unsigned long)pg->pin_mask, pg->timer_index);
	}

	if (_bdshot_channel_mask) {
		PX4_INFO("BDShot channel mask: 0x%02x", (unsigned)_bdshot_channel_mask);

		for (int i = 0; i < MAX_TIMER_IO_CHANNELS; i++) {
			if (_bdshot_channel_mask & (1 << i)) {
				if (_bdshot_capture_supported[i]) {
					PX4_INFO("Ch%u: read_ok %lu, fail_crc %lu",
						 i, read_ok[i], read_fail_crc[i]);

				} else {
					PX4_INFO("Ch%u: capture not supported", i);
				}
			}
		}
	}

	if (_edt_enabled) {
		PX4_INFO("BDShot EDT");

		for (int i = 0; i < MAX_TIMER_IO_CHANNELS; i++) {
			if (_bdshot_online[i]) {
				PX4_INFO("Ch%d:  eRPM: %dHz  Temp: %dC (%.1fHz)  Volt: %.2fV (%.1fHz)  Curr: %.1fA (%.1fHz)",
					 i,
					 (int)_erpms[i].rate_hz,
					 (int)_edt_temp[i].value,
					 (double)_edt_temp[i].rate_hz,
					 (double)_edt_volt[i].value * 0.25,
					 (double)_edt_volt[i].rate_hz,
					 (double)_edt_curr[i].value * 0.5,
					 (double)_edt_curr[i].rate_hz);
			}
		}
	}
}

#endif
