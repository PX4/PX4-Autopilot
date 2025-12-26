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

#if (CONFIG_STM32_HAVE_IP_DMA_V1)
//Do nothing. IP DMA V1 MCUs are not supported.
#else

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <stm32_dma.h>
#include <stm32_tim.h>
#include <px4_arch/dshot.h>
#include <px4_arch/io_timer.h>
#include <drivers/drv_dshot.h>

#include <px4_platform_common/log.h>
#include <stdio.h>
#include <drivers/drv_input_capture.h>

#include <lib/perf/perf_counter.h>

// This can be overriden for a specific board.
#ifndef BOARD_DMA_NUM_DSHOT_CHANNELS
#define BOARD_DMA_NUM_DSHOT_CHANNELS 1
#endif

// DShot protocol definitions
#define ONE_MOTOR_DATA_SIZE         16u
#define MOTOR_PWM_BIT_1             14u
#define MOTOR_PWM_BIT_0             7u
#define DSHOT_THROTTLE_POSITION     5u
#define DSHOT_TELEMETRY_POSITION    4u
#define NIBBLES_SIZE                4u
#define DSHOT_NUMBER_OF_NIBBLES     3u
#define MAX_NUM_CHANNELS_PER_TIMER  4u // CCR1-CCR4

// DMA stream configuration registers
#define DSHOT_DMA_SCR (DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | \
		       DMA_SCR_DIR_M2P | DMA_SCR_TCIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

// 16-bit because not all of the General Purpose Timers support 32-bit
#define DSHOT_BIDIRECTIONAL_DMA_SCR (DMA_SCR_PRIHI | DMA_SCR_MSIZE_16BITS | DMA_SCR_PSIZE_16BITS | DMA_SCR_MINC | \
				     DMA_SCR_DIR_P2M | DMA_SCR_TCIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

#if defined(CONFIG_ARMV7M_DCACHE)
#  define DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
#  define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)
#else
#define DMA_ALIGN_UP(n) (n)
#endif

#define CHANNEL_OUTPUT_BUFF_SIZE    17u
#define CHANNEL_CAPTURE_BUFF_SIZE   32u

#define DSHOT_OUTPUT_BUFFER_SIZE(channel_count) (DMA_ALIGN_UP(sizeof(uint32_t) * CHANNEL_OUTPUT_BUFF_SIZE * channel_count))
#define DSHOT_CAPTURE_BUFFER_SIZE(channel_count) (DMA_ALIGN_UP(sizeof(uint16_t)* CHANNEL_CAPTURE_BUFF_SIZE * channel_count))

static void init_timer_config(uint32_t channel_mask);
static void init_timers_dma_up(void);
static int32_t init_timer_channels(uint8_t timer_index);

static void select_next_capture_channel(uint8_t timer_index);
static uint8_t output_channel_from_timer_channel(uint8_t timer_index, uint8_t timer_channel);

static void dma_burst_finished_callback(DMA_HANDLE handle, uint8_t status, void *arg);
static void capture_complete_callback(void *arg);

static void process_capture_results(uint8_t timer_index, uint8_t channel_index);
static uint32_t convert_edge_intervals_to_bitstream(uint8_t timer_index, uint8_t channel_index);
static void decode_dshot_telemetry(uint32_t payload, struct BDShotTelemetry *packet);

// Timer configuration struct
typedef struct timer_config_t {
	DMA_HANDLE dma_handle;          // DMA stream for DMA update and eRPM Capture Compare
	bool enabled;                   // Timer enabled
	bool enabled_channels[4];       // Timer Channels enabled (requested)
	bool initialized;               // Timer initialized
	bool initialized_channels[4];   // Timer channels initialized (successfully started)
	bool bidirectional;             // Timer in bidir (inverted) mode
	int capture_channel;            // Timer channel currently being catured in bidirectional mode
	uint8_t timer_index;            // Timer index. Necessary to have memory for passing pointer to hrt callback
} timer_config_t;

static timer_config_t timer_configs[MAX_IO_TIMERS] = {};

// Output buffer of interleaved motor output bytes
static uint8_t dshot_burst_buffer_array[MAX_IO_TIMERS * DSHOT_OUTPUT_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER)]
px4_cache_aligned_data() = {};
static uint32_t *dshot_output_buffer[MAX_IO_TIMERS] = {};

// Buffer containing channel capture data for a single timer
static uint16_t dshot_capture_buffer[MAX_IO_TIMERS][MAX_NUM_CHANNELS_PER_TIMER][CHANNEL_CAPTURE_BUFF_SIZE]
px4_cache_aligned_data() = {};

static const uint32_t gcr_decode[32] = {
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x9, 0xA, 0xB, 0x0, 0xD, 0xE, 0xF,
	0x0, 0x0, 0x2, 0x3, 0x0, 0x5, 0x6, 0x7,
	0x0, 0x0, 0x8, 0x1, 0x0, 0x4, 0xC, 0x0
};

// Indicates when the bdshot capture cycle is finished. This is necessary since the captured data is
// processed after a fixed delay in an hrt callback. System jitter can delay the firing of the hrt callback
// and thus delay the processing of the data. This should never happen in a properly working system, as the
// jitter would have to be longer than the control allocator update interval. A warning is issued if this
// ever does occur.
static uint32_t _bdshot_channel_mask = 0xFFFFFFFF;
static uint32_t _dshot_frequency = 0;
static bool     _edt_enabled = false; // Extended DShot Telemetry

static bool _bdshot_cycle_complete[MAX_IO_TIMERS] = { [0 ...(MAX_IO_TIMERS - 1)] = true };

// Online flags, set if ESC is reponding with valid BDShot frames
#define BDSHOT_OFFLINE_COUNT 200
static bool _bdshot_online[MAX_TIMER_IO_CHANNELS] = {};
static bool _bdshot_processed[MAX_TIMER_IO_CHANNELS] = {};
static int _consecutive_failures[MAX_TIMER_IO_CHANNELS] = {};
static int _consecutive_successes[MAX_TIMER_IO_CHANNELS] = {};

typedef struct erpm_data_t {
	int32_t erpm;
	bool ready;
	float rate_hz;
	uint64_t last_timestamp;
} erpm_data_t;

typedef struct edt_data_t {
	uint8_t value;
	bool ready;
	float rate_hz;
	uint64_t last_timestamp;
} edt_data_t;

// Adaptive base interval per channel (ticks per bit)
static float _base_interval[MAX_TIMER_IO_CHANNELS] = { [0 ...(MAX_TIMER_IO_CHANNELS - 1)] = 21.0f };

erpm_data_t _erpms[MAX_TIMER_IO_CHANNELS] = {};

edt_data_t _edt_temp[MAX_TIMER_IO_CHANNELS] = {};
edt_data_t _edt_volt[MAX_TIMER_IO_CHANNELS] = {};
edt_data_t _edt_curr[MAX_TIMER_IO_CHANNELS] = {};

static float calculate_rate_hz(uint64_t last_timestamp, float last_rate_hz, uint64_t timestamp);

// hrt callback handle for captcomp post dma processing per timer
static struct hrt_call _cc_calls[MAX_IO_TIMERS];

// decoding status for each channel
static uint32_t read_ok[MAX_TIMER_IO_CHANNELS] = {};
static uint32_t read_fail_crc[MAX_TIMER_IO_CHANNELS] = {};

static perf_counter_t hrt_callback_perf = NULL;
static perf_counter_t capture_cycle_perf = NULL;
static perf_counter_t capture_cycle_perf2 = NULL;

static void init_timer_config(uint32_t channel_mask)
{
	// Mark timers in use, channels in use, and timers for bdshot
	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {
		if (channel_mask & (1 << output_channel)) {
			uint8_t timer_index = timer_io_channels[output_channel].timer_index;

			uint8_t timer_channel = timer_io_channels[output_channel].timer_channel;

			if ((timer_channel <= 0) || (timer_channel >= 5)) {
				// invalid channel, only 1 - 4
				continue;
			}

			uint8_t timer_channel_index = timer_channel - 1;

			if (io_timers[timer_index].dshot.dma_base == 0) {
				// board does not configure dshot on this timer
				continue;
			}

			timer_configs[timer_index].timer_index = timer_index;
			timer_configs[timer_index].enabled = true;
			timer_configs[timer_index].enabled_channels[timer_channel_index] = true;

			// TODO: bdshot per timer not on all timers
			// Mark timer as bidirectional
			if (_bdshot_channel_mask & (1 << output_channel)) {
				timer_configs[timer_index].bidirectional = true;
			}
		}
	}
}

// Initializes dshot on configured timers if DShot mode is enabled and DMA is available.
static void init_timers_dma_up(void)
{
	for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {

		if (!timer_configs[timer_index].enabled) {
			// timer is not enabled for dshot
			continue;
		}

		if (io_timers[timer_index].dshot.dma_map_up == 0) {
			// timer does not have DMA UP mapped
			continue;
		}

		// Attempt to allocate DMA for Burst
		timer_configs[timer_index].dma_handle = stm32_dmachannel(io_timers[timer_index].dshot.dma_map_up);

		if (timer_configs[timer_index].dma_handle == NULL) {
			PX4_WARN("Failed to allocate Timer %u DMA UP", timer_index);
			continue;
		}

		PX4_INFO("Allocated DMA UP Timer Index %u", timer_index);
		timer_configs[timer_index].initialized = true;
	}

	// Bidirectional DShot will free/allocate DMA stream on every update event. This is required
	// in order to reconfigure the DMA stream between Timer Burst and CaptureCompare.
	for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {

		if (!timer_configs[timer_index].bidirectional) {
			continue;
		}

		if (timer_configs[timer_index].dma_handle != NULL) {
			stm32_dmafree(timer_configs[timer_index].dma_handle);
			timer_configs[timer_index].dma_handle = NULL;
			PX4_INFO("Freed DMA UP Timer Index %u", timer_index);
		}
	}
}

static int32_t init_timer_channels(uint8_t timer_index)
{
	int32_t channels_init_mask = 0;

	if (!timer_configs[timer_index].enabled || !timer_configs[timer_index].initialized) {
		// timer is not enabled or could not be initialized
		return 0;
	}

	io_timer_channel_mode_t mode = timer_configs[timer_index].bidirectional ? IOTimerChanMode_DshotInverted :
				       IOTimerChanMode_Dshot;

	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {

		uint8_t timer_channel = timer_io_channels[output_channel].timer_channel;

		if ((timer_channel <= 0) || (timer_channel >= 5)) {
			// invalid channel, only 1 - 4
			continue;
		}

		uint8_t timer_channel_index = timer_channel - 1;
		bool this_timer = timer_index == timer_io_channels[output_channel].timer_index;
		bool channel_enabled = timer_configs[timer_index].enabled_channels[timer_channel_index];

		if (this_timer && channel_enabled) {
			int ret = io_timer_channel_init(output_channel, mode, NULL, NULL);

			if (ret != OK) {
				PX4_WARN("io_timer_channel_init %u failed", output_channel);
				continue;
			}

			timer_configs[timer_index].initialized_channels[timer_channel_index] = true;
			channels_init_mask |= (1 << output_channel);

			PX4_DEBUG("%sDShot initialized OutputChannel %u", timer_configs[timer_index].bidirectional ? "B" : "", output_channel);
		}
	}

	return channels_init_mask;
}

int up_dshot_init(uint32_t channel_mask, uint32_t bdshot_channel_mask, unsigned dshot_pwm_freq, bool edt_enable)
{
	_dshot_frequency = dshot_pwm_freq;
	_bdshot_channel_mask = bdshot_channel_mask;
	_edt_enabled = edt_enable;

	if (bdshot_channel_mask) {
		// TODO: show which timers/channels it's enabled on
		PX4_INFO("BDShot enabled");
		hrt_callback_perf = perf_alloc(PC_ELAPSED, "dshot: callback perf");
		capture_cycle_perf = perf_alloc(PC_INTERVAL, "dshot: cycle perf");
		capture_cycle_perf2 = perf_alloc(PC_INTERVAL, "dshot: cycle perf2");
	}

	// NOTE: Bidirectional DShot uses round-robin capture (1 channel per timer per cycle).
	// Each timer needs 1 DMA channel, re-used for burst transmit and then capture.
	// Multiple timers are processed in parallel for low latency.

	// Initialize timer_config data based on enabled channels
	init_timer_config(channel_mask);

	// Initializes dshot on each timer if DShot mode is enabled and DMA is available
	init_timers_dma_up();

	int32_t channels_init_mask = 0;

	for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {
		channels_init_mask |= init_timer_channels(timer_index);
	}

	unsigned output_buffer_offset = 0;

	for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {
		if (timer_configs[timer_index].initialized) {
			if (io_timers[timer_index].base == 0) { // no more timers configured
				break;
			}

			// we know the uint8_t* cast to uint32_t* is fine, since we're aligned to cache line size
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
			dshot_output_buffer[timer_index] = (uint32_t *) &dshot_burst_buffer_array[output_buffer_offset];

#pragma GCC diagnostic pop
			uint32_t channel_count = io_timers_channel_mapping.element[timer_index].channel_count_including_gaps;
			output_buffer_offset += DSHOT_OUTPUT_BUFFER_SIZE(channel_count);

			if (output_buffer_offset > sizeof(dshot_burst_buffer_array)) {
				return -EINVAL; // something is wrong with the board configuration or some other logic
			}
		}
	}

	return channels_init_mask;
}

// Kicks off a DMA transmit for each configured timer and the associated channels
void up_dshot_trigger()
{
	if (_bdshot_channel_mask) {
		bool all_complete = true;

		for (int i = 0; i < MAX_IO_TIMERS; i++) {
			if (!_bdshot_cycle_complete[i]) {
				all_complete = false;
			}
		}

		if (!all_complete) {
			PX4_WARN("BDShot cycle not complete! Check system jitter");
			return;
		}
	}

	// For each timer, begin DMA transmit
	for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {

		timer_config_t *timer = &timer_configs[timer_index];

		io_timer_channel_mode_t mode = timer->bidirectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot;

		io_timer_set_enable(true, mode, io_timer_get_group(timer_index));

		if (timer->enabled && timer->initialized) {

			uint32_t channel_count = io_timers_channel_mapping.element[timer_index].channel_count_including_gaps;

			io_timer_set_dshot_burst_mode(timer_index, _dshot_frequency, channel_count);

			if (timer->bidirectional) {
				// Deallocate DMA from previous transaction
				if (timer->dma_handle != NULL) {
					stm32_dmastop(timer->dma_handle);
					stm32_dmafree(timer->dma_handle);
					timer->dma_handle = NULL;
				}

				// Allocate DMA
				timer->dma_handle = stm32_dmachannel(io_timers[timer_index].dshot.dma_map_up);

				if (timer->dma_handle == NULL) {
					PX4_WARN("DMA allocation for timer %u failed", timer_index);
					continue;
				}
			}

			// Flush cache so DMA sees the data
			up_clean_dcache((uintptr_t) dshot_output_buffer[timer_index],
					(uintptr_t) dshot_output_buffer[timer_index] +
					DSHOT_OUTPUT_BUFFER_SIZE(channel_count));

			px4_stm32_dmasetup(timer->dma_handle,
					   io_timers[timer_index].base + STM32_GTIM_DMAR_OFFSET,
					   (uint32_t)(dshot_output_buffer[timer_index]),
					   channel_count * CHANNEL_OUTPUT_BUFF_SIZE,
					   DSHOT_DMA_SCR);

			// Clean UDE flag before DMA is started
			io_timer_update_dma_req(timer_index, false);

			// Trigger DMA (DShot Outputs). Only capture compare after the system has had time to boot.
			if (timer->bidirectional && (hrt_absolute_time() > 3000000)) {

				perf_begin(capture_cycle_perf);
				stm32_dmastart(timer->dma_handle, dma_burst_finished_callback, &timer->timer_index, false);

			} else {
				stm32_dmastart(timer->dma_handle,  NULL, NULL, false);

				if (timer->bidirectional) {
					_bdshot_cycle_complete[timer_index] = true;
				}
			}

			// Enable DMA update request
			io_timer_update_dma_req(timer_index, true);
		}
	}
}

static void select_next_capture_channel(uint8_t timer_index)
{
	int current = timer_configs[timer_index].capture_channel;

	// Try each of the 4 possible channels, starting from current+1
	for (int i = 1; i <= 4; i++) {
		int next = (current + i) % 4;

		if (timer_configs[timer_index].initialized_channels[next]) {
			uint8_t output_channel = output_channel_from_timer_channel(timer_index, next);

			if (_bdshot_channel_mask & (1 << output_channel)) {
				timer_configs[timer_index].capture_channel = next;
				return;
			}
		}
	}
}

static uint8_t output_channel_from_timer_channel(uint8_t timer_index, uint8_t timer_channel_index)
{
	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {

		bool is_this_timer = timer_index == timer_io_channels[output_channel].timer_index;
		uint8_t channel_index = timer_io_channels[output_channel].timer_channel - 1;

		if (is_this_timer && (channel_index == timer_channel_index)) {
			// We found the output channel associated with this timer channel
			return output_channel;
		}
	}

	// TODO: error handling?
	return 0;
}

void dma_burst_finished_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	uint8_t timer_index = *((uint8_t *)arg);

	// Clean DMA UP configuration
	if (timer_configs[timer_index].dma_handle != NULL) {
		stm32_dmastop(timer_configs[timer_index].dma_handle);
		stm32_dmafree(timer_configs[timer_index].dma_handle);
		timer_configs[timer_index].dma_handle = NULL;
	}

	// Disable DMA update request
	io_timer_update_dma_req(timer_index, false);

	// De-allocate timer
	io_timer_unallocate_timer(timer_index);

	// Flush cache so DMA sees the data
	memset(dshot_capture_buffer[timer_index], 0, sizeof(dshot_capture_buffer[timer_index]));
	up_clean_dcache((uintptr_t) dshot_capture_buffer[timer_index],
			(uintptr_t) dshot_capture_buffer[timer_index] + DSHOT_CAPTURE_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER));

	// Re-initialize all output channels on this timer as CaptureDMA to ensure all lines idle high
	for (uint8_t channel = 0; channel < MAX_TIMER_IO_CHANNELS; channel++) {

		bool is_this_timer = timer_index == timer_io_channels[channel].timer_index;
		uint8_t timer_channel_index = timer_io_channels[channel].timer_channel - 1;
		bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];

		if (is_this_timer && channel_initialized) {

			io_timer_unallocate_channel(channel);
			// Initialize back to DShotInverted to bring IO back to the expected idle state
			io_timer_channel_init(channel, IOTimerChanMode_CaptureDMA, NULL, NULL);
		}
	}

	// Select the next capture channel
	select_next_capture_channel(timer_index);

	// Allocate DMA for currently selected capture_channel
	uint8_t capture_channel = timer_configs[timer_index].capture_channel;
	timer_configs[timer_index].dma_handle = stm32_dmachannel(io_timers[timer_index].dshot.dma_map_ch[capture_channel]);

	// If DMA handler is valid, start DMA
	if (timer_configs[timer_index].dma_handle == NULL) {
		PX4_WARN("failed to allocate dma for timer %u channel %u", timer_index, capture_channel);
		return;
	}

	// Set Capture mode for this channel
	io_timer_set_dshot_capture_mode(timer_index, capture_channel, _dshot_frequency);
	io_timer_capture_dma_req(timer_index, capture_channel, true);

	// Choose which CC register for this DMA stream
	uint32_t periph_addr = io_timers[timer_index].base + STM32_GTIM_CCR1_OFFSET + (4 * capture_channel);

	// Setup DMA for this channel
	px4_stm32_dmasetup(timer_configs[timer_index].dma_handle,
			   periph_addr,
			   (uint32_t) dshot_capture_buffer[timer_index][capture_channel],
			   CHANNEL_CAPTURE_BUFF_SIZE,
			   DSHOT_BIDIRECTIONAL_DMA_SCR);

	// NOTE: we can't use DMA callback since GCR encoding creates a variable length pulse train. Instead
	// we use an hrt callback to schedule the processing of the received and DMAd eRPM frames.
	stm32_dmastart(timer_configs[timer_index].dma_handle, NULL, NULL, false);

	// Enable CaptureDMA on this timer's channels only
	io_timer_set_enable(true, IOTimerChanMode_CaptureDMA, io_timer_get_group(timer_index));

	// Measuring the time it takes from when we start the DMA to when we enable CaptureDMA
	perf_end(capture_cycle_perf);
	perf_begin(capture_cycle_perf2);

	// 30us to switch regardless of DShot frequency + eRPM frame time + 20us for good measure
	hrt_abstime frame_us = (16 * 1000000) / _dshot_frequency; // 16 bits * us_per_s / bits_per_s
	hrt_abstime delay = 30 + frame_us + 20;
	hrt_call_after(&_cc_calls[timer_index], delay, capture_complete_callback, arg);
}

static void capture_complete_callback(void *arg)
{
	perf_end(capture_cycle_perf2);
	perf_begin(hrt_callback_perf);

	uint8_t timer_index = *((uint8_t *)arg);

	// Unallocate the timer as CaptureDMA
	io_timer_unallocate_timer(timer_index);

	uint8_t capture_channel = timer_configs[timer_index].capture_channel;

	// Disable capture DMA
	io_timer_capture_dma_req(timer_index, capture_channel, false);

	// Stop DMA (should already be finished)
	stm32_dmastop(timer_configs[timer_index].dma_handle);

	// Re-initialize all output channels on this timer
	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {

		bool is_this_timer = timer_index == timer_io_channels[output_channel].timer_index;
		uint8_t timer_channel_index = timer_io_channels[output_channel].timer_channel - 1;
		bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];

		if (is_this_timer && channel_initialized) {
			io_timer_unallocate_channel(output_channel);
			// Initialize back to DShotInverted to bring IO back to the expected idle state
			io_timer_channel_init(output_channel, IOTimerChanMode_DshotInverted, NULL, NULL);
		}
	}

	// Invalidate the dcache to ensure most recent data is available
	up_invalidate_dcache((uintptr_t) dshot_capture_buffer[timer_index],
			     (uintptr_t) dshot_capture_buffer[timer_index] + DSHOT_CAPTURE_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER));

	// Process eRPM frames from all channels on this timer
	process_capture_results(timer_index, capture_channel);

	// Enable this timer's channels as DShotInverted
	io_timer_set_enable(true, IOTimerChanMode_DshotInverted, io_timer_get_group(timer_index));

	perf_end(hrt_callback_perf);

	_bdshot_cycle_complete[timer_index] = true;
}

void process_capture_results(uint8_t timer_index, uint8_t channel_index)
{
	uint8_t output_channel = output_channel_from_timer_channel(timer_index, channel_index);
	uint32_t value = convert_edge_intervals_to_bitstream(timer_index, channel_index);

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

		_bdshot_processed[output_channel] = true;
		return;
	}

	++read_ok[output_channel];

	if (_consecutive_successes[output_channel]++ > BDSHOT_OFFLINE_COUNT) {
		_consecutive_successes[output_channel] = BDSHOT_OFFLINE_COUNT;
		_consecutive_failures[output_channel] = 0;
		_bdshot_online[output_channel] = true;
	}

	// Convert payload into telem type/value
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
		// TODO: Handle these?
		break;

	default:
		PX4_WARN("unknown EDT type %d", packet.type);
		break;
	}

	_bdshot_processed[output_channel] = true;
}

float calculate_rate_hz(uint64_t last_timestamp, float last_rate_hz, uint64_t timestamp)
{
	if (last_timestamp == 0 || timestamp <= last_timestamp) {
		return last_rate_hz;
	}

	uint64_t dt_us = timestamp - last_timestamp;

	float instant_rate = 1000000.0f / dt_us;

	// Simple exponential moving average with fixed alpha
	// Alpha = 0.125 (1/8) works well across all rates
	float rate_hz = instant_rate * 0.125f + last_rate_hz * 0.875f;

	return rate_hz;
}

// Converts captured edge timestamps into a raw bit stream.
// Measures the time intervals between signal edges to determine how many consecutive
// 1s or 0s to shift in, alternating the bit value with each edge transition.
// Uses adaptive per-channel timing calibration to handle ESC oscillator variation.
// Returns a 20 bit raw value that still needs RLL and GCR decoding.
uint32_t convert_edge_intervals_to_bitstream(uint8_t timer_index, uint8_t channel_index)
{
	// First pass: collect all intervals
	uint32_t intervals[CHANNEL_CAPTURE_BUFF_SIZE];
	unsigned interval_count = 0;

	// We can ignore the very first data point as it's the pulse before it starts.
	unsigned previous = dshot_capture_buffer[timer_index][channel_index][1];

	for (unsigned i = 2; i < CHANNEL_CAPTURE_BUFF_SIZE; ++i) {
		if (dshot_capture_buffer[timer_index][channel_index][i] == 0) {
			// Once we get zeros we're through
			break;
		}

		uint32_t interval = dshot_capture_buffer[timer_index][channel_index][i] - previous;

		// Filter out noise: reject intervals <10 or >100 ticks
		if (interval >= 10 && interval <= 100) {
			intervals[interval_count++] = interval;
		}

		previous = dshot_capture_buffer[timer_index][channel_index][i];
	}

	if (interval_count == 0) {
		return 0;
	}

	// Calibrate base interval using minimum interval (always 1 bit in GCR)
	// The shortest valid interval in the frame represents a single bit period
	uint32_t min_interval = intervals[0];

	for (unsigned i = 1; i < interval_count; i++) {
		if (intervals[i] < min_interval) {
			min_interval = intervals[i];
		}
	}

	uint8_t output_channel = output_channel_from_timer_channel(timer_index, channel_index);

	// Update base interval with low-pass filter (10% weight on new measurement)
	// This adapts to each ESC's actual timing while filtering transient noise
	_base_interval[output_channel] = 0.9f * _base_interval[output_channel] + 0.1f * (float)min_interval;

	// Second pass: decode bits using adaptive threshold
	uint32_t value = 0;
	uint32_t high = 1;
	unsigned shifted = 0;

	float base = _base_interval[output_channel];

	for (unsigned i = 0; i < interval_count; i++) {
		// Convert interval to bit count using relative threshold
		// Round to nearest: 0.5-1.5 → 1, 1.5-2.5 → 2, 2.5-3.5 → 3, etc.
		float interval_f = (float)intervals[i];
		uint32_t bits = (uint32_t)((interval_f / base) + 0.5f);

		// Clamp to valid range (1-4 bits typical in GCR encoding)
		if (bits < 1) {
			bits = 1;

		} else if (bits > 4) {
			bits = 4;
		}

		// Shift in the bits
		for (unsigned bit = 0; bit < bits; ++bit) {
			value = (value << 1) | high;
			++shifted;
		}

		high = !high;
	}

	// Flexible frame validation: accept 18-24 bits instead of forcing exactly 21
	// This handles ESC timing variation and incomplete frames
	if (shifted < 18 || shifted > 24) {
		// Frame too short or too long - likely corrupted
		return 0;
	}

	// Pad to 21 bits for GCR decoder (which expects exactly 21 bits)
	if (shifted < 21) {
		value <<= (21 - shifted);

	} else if (shifted > 21) {
		// Trim excess bits (shouldn't happen often with proper decoding)
		value >>= (shifted - 21);
	}

	return value;
}

void decode_dshot_telemetry(uint32_t payload, struct BDShotTelemetry *packet)
{
	// Extended DShot Telemetry
	bool edt_enabled = _edt_enabled;
	uint32_t mantissa = payload & 0x01FF;
	bool is_telemetry = !(mantissa & 0x0100); // if the msb of the mantissa is zero, then this is an extended telemetry packet

	if (edt_enabled && is_telemetry) {
		packet->type = (payload & 0x0F00) >> 8;
		packet->value = payload & 0x00FF; // extended telemetry value is 8 bits wide

	} else {
		// otherwise it's an eRPM frame
		uint8_t exponent = ((payload >> 9) & 0x7); // 3 bit: exponent
		uint16_t period = (payload & 0x1FF); // 9 bit: period base
		period = period << exponent; // Period in usec

		packet->type = DSHOT_EDT_ERPM;

		if (period == 65408 || period == 0) {
			// 65408 is a special case for zero motion (e.g., stationary motor)
			packet->value = 0;

		} else {
			packet->value = (1000000 * 60 / 100 + period / 2) / period;
		}
	}
}

// bits  1-11  - throttle value (0-47 are reserved for commands, 48-2047 give 2000 steps of throttle resolution)
// bit   12    - dshot telemetry enable/disable
// bits  13-16 - XOR checksum
void dshot_motor_data_set(uint8_t channel, uint16_t data, bool telemetry)
{
	uint8_t timer_index = timer_io_channels[channel].timer_index;
	uint8_t timer_channel_index = timer_io_channels[channel].timer_channel - 1;
	bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];

	if (!channel_initialized) {
		return;
	}

	uint16_t packet = 0;
	uint16_t checksum = 0;

	packet |= data << DSHOT_THROTTLE_POSITION;
	packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

	uint16_t csum_data = packet;

	// XOR checksum calculation
	csum_data >>= NIBBLES_SIZE;

	for (uint8_t i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
		checksum ^= (csum_data & 0x0F); // XOR data by nibbles
		csum_data >>= NIBBLES_SIZE;
	}

	if (_bdshot_channel_mask & (1 << channel)) {
		packet |= ((~checksum) & 0x0F);

	} else {
		packet |= ((checksum) & 0x0F);
	}

	const io_timers_channel_mapping_element_t *mapping = &io_timers_channel_mapping.element[timer_index];
	uint8_t num_motors = mapping->channel_count_including_gaps;
	uint8_t timer_channel = timer_io_channels[channel].timer_channel - mapping->lowest_timer_channel;

	for (uint8_t motor_data_index = 0; motor_data_index < ONE_MOTOR_DATA_SIZE; motor_data_index++) {
		dshot_output_buffer[timer_index][motor_data_index * num_motors + timer_channel] =
			(packet & 0x8000) ? MOTOR_PWM_BIT_1 : MOTOR_PWM_BIT_0;  // MSB first
		packet <<= 1;
	}
}

int up_dshot_arm(bool armed)
{
	int ret = PX4_OK;

	for (int timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {
		io_timer_channel_mode_t mode = timer_configs[timer_index].bidirectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot;
		ret |= io_timer_set_enable(armed, mode, io_timer_get_group(timer_index));
	}

	return ret;
}

int up_bdshot_num_channels_ready(void)
{
	int num_ready = 0;

	for (uint8_t i = 0; i < MAX_TIMER_IO_CHANNELS; ++i) {
		if (_bdshot_processed[i]) {
			++num_ready;
		}
	}

	return num_ready;
}

int up_bdshot_num_errors(uint8_t channel)
{
	return read_fail_crc[channel];
}

int up_bdshot_get_erpm(uint8_t channel, int *erpm)
{
	uint8_t timer_index = timer_io_channels[channel].timer_index;
	uint8_t timer_channel_index = timer_io_channels[channel].timer_channel - 1;
	bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];

	int status = PX4_ERROR;

	if (channel_initialized && _erpms[channel].ready) {
		*erpm = _erpms[channel].erpm;
		status = PX4_OK;
	}

	// Mark sample read
	_bdshot_processed[channel] = false;

	return status;
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

void up_bdshot_status(void)
{
	PX4_INFO("dshot driver stats:");

	if (_bdshot_channel_mask) {
		PX4_INFO("BDShot channel mask: 0x%2x", (unsigned)_bdshot_channel_mask);
	}

	if (_edt_enabled) {
		PX4_INFO("BDShot EDT rates");

		for (int i = 0; i < MAX_TIMER_IO_CHANNELS; i++) {
			if (_bdshot_online[i]) {
				PX4_INFO("Ch%d:  eRPM: %dHz  Temp: %.2fHz  Volt: %.2fHz  Curr: %.2fHz",
					 i,
					 (int)_erpms[i].rate_hz,
					 (double)_edt_temp[i].rate_hz,
					 (double)_edt_volt[i].rate_hz,
					 (double)_edt_curr[i].rate_hz);
			}

			PX4_INFO("Output %u: read %lu, failed CRC %lu", i, read_ok[i], read_fail_crc[i]);
		}
	}
}

#endif
