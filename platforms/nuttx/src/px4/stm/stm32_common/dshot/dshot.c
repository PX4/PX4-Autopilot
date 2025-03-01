/****************************************************************************
 *
 * Copyright (C) 2024 PX4 Development Team. All rights reserved.
 * Author: Igor Misic <igy1000mb@gmail.com>
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
static void init_timers_dma_capt_comp(uint8_t timer_index);
static int32_t init_timer_channels(uint8_t timer_index);

static void configure_channels_round_robin(uint8_t timer_index);

static void dma_burst_finished_callback(DMA_HANDLE handle, uint8_t status, void *arg);
static void capture_complete_callback(void *arg);

static void process_capture_results(uint8_t timer_index);
static unsigned calculate_period(uint8_t timer_index, uint8_t channel_index);

// Timer configuration struct
typedef struct timer_config_t {
	DMA_HANDLE dma_up_handle;       // DMA stream for DMA update
	DMA_HANDLE dma_ch_handle[4];    // DMA streams for bidi CaptComp
	bool enabled;                   // Timer enabled
	bool enabled_channels[4];       // Timer Channels enabled (requested)
	bool initialized;               // Timer initialized
	bool initialized_channels[4];   // Timer channels initialized (successfully started)
	bool bidirectional;             // Timer in bidi (inverted) mode
	bool captcomp_channels[4];      // Channels configured for CaptComp
	bool round_robin_enabled;
	uint8_t timer_index;            // Timer index. Necessary to have memory for passing pointer to hrt callback
} timer_config_t;

static uint8_t _num_dma_available = 0;

static timer_config_t timer_configs[MAX_IO_TIMERS] = {};

// Output buffer of interleaved motor output bytes
static uint8_t dshot_burst_buffer_array[MAX_IO_TIMERS * DSHOT_OUTPUT_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER)]
px4_cache_aligned_data() = {};
static uint32_t *dshot_output_buffer[MAX_IO_TIMERS] = {};

// Buffer containing channel capture data for a single timer
static uint16_t dshot_capture_buffer[MAX_NUM_CHANNELS_PER_TIMER][CHANNEL_CAPTURE_BUFF_SIZE]
px4_cache_aligned_data() = {};

static bool     _bidirectional = false;
static uint8_t  _bidi_timer_index = 0; // TODO: BDSHOT_TIM param to select timer index?
static uint32_t _dshot_frequency = 0;

// eRPM data for channels on the singular timer
static int32_t _erpms[MAX_NUM_CHANNELS_PER_TIMER] = {};
static bool _erpms_ready[MAX_NUM_CHANNELS_PER_TIMER] = {};

// hrt callback handle for captcomp post dma processing
static struct hrt_call _cc_call;

// decoding status for each channel
static uint32_t read_ok[MAX_NUM_CHANNELS_PER_TIMER] = {};
static uint32_t read_fail_nibble[MAX_NUM_CHANNELS_PER_TIMER] = {};
static uint32_t read_fail_crc[MAX_NUM_CHANNELS_PER_TIMER] = {};
static uint32_t read_fail_zero[MAX_NUM_CHANNELS_PER_TIMER] = {};

static void init_timer_config(uint32_t channel_mask)
{
	// Mark timers in use, channels in use, and timers for bidi dshot
	for (unsigned output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {
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

			// NOTE: only 1 timer can be used if Bidirectional DShot is enabled
			if (_bidirectional && (timer_index != _bidi_timer_index)) {
				continue;
			}

			// NOTE: this is necessary to pass timer_index to DMA callback
			timer_configs[timer_index].timer_index = timer_index;
			// Mark timer as enabled
			timer_configs[timer_index].enabled = true;
			// Mark channel as enabled
			timer_configs[timer_index].enabled_channels[timer_channel_index] = true;

			// Mark timer as bidirectional
			if (_bidirectional && timer_index == _bidi_timer_index) {
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

		// NOTE: only 1 timer can be used if Bidirectional DShot is enabled
		if (_bidirectional && (timer_index != _bidi_timer_index)) {
			continue;
		}

		// Attempt to allocate DMA for Burst
		timer_configs[timer_index].dma_up_handle = stm32_dmachannel(io_timers[timer_index].dshot.dma_map_up);

		if (timer_configs[timer_index].dma_up_handle == NULL) {
			PX4_DEBUG("Failed to allocate Timer %u DMA UP", timer_index);
			continue;
		}

		PX4_DEBUG("Allocated DMA UP Timer Index %u", timer_index);
		timer_configs[timer_index].initialized = true;
	}

	// Free the allocated DMA channels
	for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {
		if (timer_configs[timer_index].dma_up_handle != NULL) {
			stm32_dmafree(timer_configs[timer_index].dma_up_handle);
			timer_configs[timer_index].dma_up_handle = NULL;
			PX4_DEBUG("Freed DMA UP Timer Index %u", timer_index);
		}
	}
}

static void init_timers_dma_capt_comp(uint8_t timer_index)
{
	if (timer_configs[timer_index].enabled && timer_configs[timer_index].initialized) {

		// Allocate DMA for each enabled channel
		for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {

			uint8_t timer_channel = timer_io_channels[output_channel].timer_channel;

			if ((timer_channel <= 0) || (timer_channel >= 5)) {
				// invalid channel, only 1 - 4
				continue;
			}

			// For each enabled channel on this timer, try allocating DMA
			uint8_t timer_channel_index = timer_channel - 1;
			bool this_timer = timer_index == timer_io_channels[output_channel].timer_index;
			bool channel_enabled = timer_configs[timer_index].enabled_channels[timer_channel_index];

			if (this_timer && channel_enabled) {
				uint32_t dma_map_ch = io_timers[timer_index].dshot.dma_map_ch[timer_channel_index];

				if (dma_map_ch == 0) {
					// Timer channel is not mapped
					PX4_WARN("Error! Timer %u Channel %u DMA is unmapped", timer_index, timer_channel_index);
					continue;
				}

				PX4_DEBUG("Allocating DMA CH for Timer Index %u Channel %u", timer_index, timer_channel_index);
				// Allocate DMA
				timer_configs[timer_index].dma_ch_handle[timer_channel_index] = stm32_dmachannel(dma_map_ch);

				if (timer_configs[timer_index].dma_ch_handle[timer_channel_index] == NULL) {
					PX4_WARN("Failed to allocate Timer %u Channel DMA CH %u, output_channel %u", timer_index, timer_channel_index,
						 output_channel);
					continue;
				}

				PX4_DEBUG("Allocated DMA CH Timer Index %u Channel %u", timer_index, timer_channel_index);
				// Mark this timer channel as bidirectional
				timer_configs[timer_index].captcomp_channels[timer_channel_index] = true;
				_num_dma_available++;

				if (_num_dma_available >= BOARD_DMA_NUM_DSHOT_CHANNELS) {
					PX4_INFO("Limiting DMA channels to %u", _num_dma_available);
					break;
				}
			}
		}

		// De-allocate DMA for each channel
		for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {
			if (timer_index == timer_io_channels[output_channel].timer_index) {

				uint8_t timer_channel = timer_io_channels[output_channel].timer_channel;

				if ((timer_channel <= 0) || (timer_channel >= 5)) {
					// invalid channel, only 1 - 4
					continue;
				}

				uint8_t timer_channel_index = timer_channel - 1;

				if (timer_configs[timer_index].dma_ch_handle[timer_channel_index]) {
					stm32_dmafree(timer_configs[timer_index].dma_ch_handle[timer_channel_index]);
					timer_configs[timer_index].dma_ch_handle[timer_channel_index] = NULL;
					PX4_DEBUG("Freed DMA CH Timer Index %u Channel %u", timer_index, timer_channel_index);
				}
			}
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

			if (timer_configs[timer_index].bidirectional) {
				PX4_DEBUG("DShot initialized OutputChannel %u (bidirectional)", output_channel);

			} else {
				PX4_DEBUG("DShot initialized OutputChannel %u", output_channel);
			}
		}
	}

	return channels_init_mask;
}

int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq, bool enable_bidirectional_dshot)
{
	_dshot_frequency = dshot_pwm_freq;
	_bidirectional = enable_bidirectional_dshot;

	if (_bidirectional) {
		PX4_INFO("Bidirectional DShot enabled, only one timer will be used");
	}

	// NOTE: if bidirectional is enabled only 1 timer can be used. This is because Burst mode uses 1 DMA channel per timer
	// and CaptureCompare (for reading ESC eRPM) uses 4 DMA. Even if we only used CaptureCompare on a single timer
	// we would still need 5 DMA; 1 DMA for the second timer burst, and 4 DMA for the first timer CaptureCompare. The only
	// way to support more than 1 timer is to burst/captcomp sequentially for each timer enabled for dshot. The code is
	// structured in a way to allow extending support for this in the future.

	// Initialize timer_config data based on enabled channels
	init_timer_config(channel_mask);

	// Initializes dshot on each timer if DShot mode is enabled and DMA is available
	init_timers_dma_up();

	// Initializes a single timer in Bidirectional DShot mode
	if (_bidirectional) {
		// Use first configured DShot timer (Timer index 0)
		// TODO: BDSHOT_TIM param to select timer index?
		init_timers_dma_capt_comp(_bidi_timer_index);

		// Enable round robin if we have 1 - 3 DMA
		if ((_num_dma_available < 4) && _num_dma_available > 0) {
			timer_configs[_bidi_timer_index].round_robin_enabled = true;
		}
	}

	int32_t channels_init_mask = 0;

	for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {
		channels_init_mask |= init_timer_channels(timer_index);
	}

	unsigned output_buffer_offset = 0;

	for (unsigned timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {
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
	// Enable DShot inverted on all channels
	io_timer_set_enable(true, _bidirectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot,
			    IO_TIMER_ALL_MODES_CHANNELS);

	// For each timer, begin DMA transmit
	for (uint8_t timer_index = 0; timer_index < MAX_IO_TIMERS; timer_index++) {
		if (timer_configs[timer_index].enabled && timer_configs[timer_index].initialized) {

			uint32_t channel_count = io_timers_channel_mapping.element[timer_index].channel_count_including_gaps;

			io_timer_set_dshot_burst_mode(timer_index, _dshot_frequency, channel_count);

			// Allocate DMA
			if (timer_configs[timer_index].dma_up_handle == NULL) {
				timer_configs[timer_index].dma_up_handle = stm32_dmachannel(io_timers[timer_index].dshot.dma_map_up);

				if (timer_configs[timer_index].dma_up_handle == NULL) {
					PX4_WARN("DMA allocation for timer %u failed", timer_index);
					continue;
				}
			}

			// Flush cache so DMA sees the data
			up_clean_dcache((uintptr_t) dshot_output_buffer[timer_index],
					(uintptr_t) dshot_output_buffer[timer_index] +
					DSHOT_OUTPUT_BUFFER_SIZE(channel_count));

			px4_stm32_dmasetup(timer_configs[timer_index].dma_up_handle,
					   io_timers[timer_index].base + STM32_GTIM_DMAR_OFFSET,
					   (uint32_t)(dshot_output_buffer[timer_index]),
					   channel_count * CHANNEL_OUTPUT_BUFF_SIZE,
					   DSHOT_DMA_SCR);

			// Clean UDE flag before DMA is started
			io_timer_update_dma_req(timer_index, false);

			// Trigger DMA (DShot Outputs)
			if (timer_configs[timer_index].bidirectional) {
				stm32_dmastart(timer_configs[timer_index].dma_up_handle, dma_burst_finished_callback,
					       &timer_configs[timer_index].timer_index,
					       false);

			} else {
				stm32_dmastart(timer_configs[timer_index].dma_up_handle,  NULL, NULL, false);
			}

			// Enable DMA update request
			io_timer_update_dma_req(timer_index, true);
		}
	}
}

static void configure_channels_round_robin(uint8_t timer_index)
{
	switch (_num_dma_available) {
	case 1: {
			for (uint8_t i = 0; i < 4; i++) {
				if (timer_configs[timer_index].captcomp_channels[i]) {
					timer_configs[timer_index].captcomp_channels[i] = false;

					if (i == 3) {
						timer_configs[timer_index].captcomp_channels[0] = true;

					} else {
						timer_configs[timer_index].captcomp_channels[i + 1] = true;
					}

					break;
				}
			}

			break;
		}

	case 2: {
			if (timer_configs[timer_index].captcomp_channels[0]) {
				timer_configs[timer_index].captcomp_channels[0] = false;
				timer_configs[timer_index].captcomp_channels[1] = true;
				timer_configs[timer_index].captcomp_channels[2] = false;
				timer_configs[timer_index].captcomp_channels[3] = true;

			} else {
				timer_configs[timer_index].captcomp_channels[0] = true;
				timer_configs[timer_index].captcomp_channels[1] = false;
				timer_configs[timer_index].captcomp_channels[2] = true;
				timer_configs[timer_index].captcomp_channels[3] = false;
			}

			break;
		}

	case 3: {
			for (uint8_t i = 0; i < 4; i++) {
				if (!timer_configs[timer_index].captcomp_channels[i]) {
					timer_configs[timer_index].captcomp_channels[i] = true;

					if (i == 3) {
						timer_configs[timer_index].captcomp_channels[0] = false;

					} else {
						timer_configs[timer_index].captcomp_channels[i + 1] = false;
					}

					break;
				}
			}

			break;
		}

	default:
		break;
	}
}

void dma_burst_finished_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	uint8_t timer_index = *((uint8_t *)arg);

	// Clean DMA UP configuration
	if (timer_configs[timer_index].dma_up_handle != NULL) {
		stm32_dmastop(timer_configs[timer_index].dma_up_handle);
		stm32_dmafree(timer_configs[timer_index].dma_up_handle);
		timer_configs[timer_index].dma_up_handle = NULL;
	}

	// Disable DMA update request
	io_timer_update_dma_req(timer_index, false);

	// De-allocate timer
	io_timer_unallocate_timer(timer_index);

	// Flush cache so DMA sees the data
	memset(dshot_capture_buffer, 0, sizeof(dshot_capture_buffer));
	up_clean_dcache((uintptr_t) dshot_capture_buffer,
			(uintptr_t) dshot_capture_buffer + DSHOT_CAPTURE_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER));

	// If round robin is enabled reconfigure which channels we capture on
	if (timer_configs[timer_index].round_robin_enabled) {
		configure_channels_round_robin(timer_index);
	}

	// Allocate DMA for all enabled channels on this timer
	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {

		bool is_this_timer = timer_index == timer_io_channels[output_channel].timer_index;
		uint8_t timer_channel = timer_io_channels[output_channel].timer_channel;

		if ((timer_channel <= 0) || (timer_channel >= 5)) {
			// invalid channel, only 1 - 4
			continue;
		}

		uint8_t timer_channel_index = timer_channel - 1;
		bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];
		bool captcomp_enabled = timer_configs[timer_index].captcomp_channels[timer_channel_index];

		if (is_this_timer && channel_initialized && captcomp_enabled) {

			// Re-initialize output for CaptureDMA
			io_timer_unallocate_channel(output_channel);
			io_timer_channel_init(output_channel, IOTimerChanMode_CaptureDMA, NULL, NULL);

			// Allocate DMA
			if (timer_configs[timer_index].dma_ch_handle[timer_channel_index] == NULL) {
				timer_configs[timer_index].dma_ch_handle[timer_channel_index] = stm32_dmachannel(
							io_timers[timer_index].dshot.dma_map_ch[timer_channel_index]);
			}

			// If DMA handler is valid, start DMA
			if (timer_configs[timer_index].dma_ch_handle[timer_channel_index] == NULL) {
				PX4_WARN("failed to allocate dma for timer %u channel %u", timer_index, timer_channel_index);
				return;
			}

			// Set Capture mode for this channel
			io_timer_set_dshot_capture_mode(timer_index, timer_channel_index, _dshot_frequency);
			io_timer_capture_dma_req(timer_index, timer_channel_index, true);

			// Choose which CC register for this DMA stream
			uint32_t periph_addr = io_timers[timer_index].base + STM32_GTIM_CCR1_OFFSET + (4 * timer_channel_index);

			// Setup DMA for this channel
			px4_stm32_dmasetup(timer_configs[timer_index].dma_ch_handle[timer_channel_index],
					   periph_addr,
					   (uint32_t) dshot_capture_buffer[timer_channel_index],
					   CHANNEL_CAPTURE_BUFF_SIZE,
					   DSHOT_BIDIRECTIONAL_DMA_SCR);

			// NOTE: we can't use DMA callback since GCR encoding creates a variable length pulse train. Instead
			// we use an hrt callback to schedule the processing of the received and DMAd eRPM frames.
			stm32_dmastart(timer_configs[timer_index].dma_ch_handle[timer_channel_index], NULL, NULL, false);
		}
	}

	// Enable CaptureDMA and on all configured channels
	io_timer_set_enable(true, IOTimerChanMode_CaptureDMA, IO_TIMER_ALL_MODES_CHANNELS);

	// 30us to switch regardless of DShot frequency + eRPM frame time + 10us for good measure
	hrt_abstime frame_us = (16 * 1000000) / _dshot_frequency; // 16 bits * us_per_s / bits_per_s
	hrt_abstime delay = 30 + frame_us + 10;
	hrt_call_after(&_cc_call, delay, capture_complete_callback, arg);
}

static void capture_complete_callback(void *arg)
{
	uint8_t timer_index = *((uint8_t *)arg);

	// Unallocate the timer as CaptureDMA
	io_timer_unallocate_timer(timer_index);

	for (uint8_t output_channel = 0; output_channel < MAX_TIMER_IO_CHANNELS; output_channel++) {

		bool is_this_timer = timer_index == timer_io_channels[output_channel].timer_index;
		uint8_t timer_channel = timer_io_channels[output_channel].timer_channel;

		if ((timer_channel <= 0) || (timer_channel >= 5)) {
			// invalid channel, only 1 - 4
			continue;
		}

		uint8_t timer_channel_index = timer_channel - 1;
		bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];
		bool captcomp_enabled = timer_configs[timer_index].captcomp_channels[timer_channel_index];

		if (is_this_timer && channel_initialized) {

			if (captcomp_enabled) {
				// Disable capture DMA
				io_timer_capture_dma_req(timer_index, timer_channel_index, false);

				if (timer_configs[timer_index].dma_ch_handle[timer_channel_index] != NULL) {
					stm32_dmastop(timer_configs[timer_index].dma_ch_handle[timer_channel_index]);
					stm32_dmafree(timer_configs[timer_index].dma_ch_handle[timer_channel_index]);
					timer_configs[timer_index].dma_ch_handle[timer_channel_index] = NULL;
				}
			}

			io_timer_unallocate_channel(output_channel);
			// Initialize back to DShotInverted to bring IO back to the expected idle state
			io_timer_channel_init(output_channel, IOTimerChanMode_DshotInverted, NULL, NULL);
		}
	}

	// Invalidate the dcache to ensure most recent data is available
	up_invalidate_dcache((uintptr_t) dshot_capture_buffer,
			     (uintptr_t) dshot_capture_buffer + DSHOT_CAPTURE_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER));

	// Process eRPM frames from all channels on this timer
	process_capture_results(timer_index);

	// Enable all channels configured as DShotInverted
	io_timer_set_enable(true, IOTimerChanMode_DshotInverted, IO_TIMER_ALL_MODES_CHANNELS);
}

void process_capture_results(uint8_t timer_index)
{
	for (uint8_t channel_index = 0; channel_index < MAX_NUM_CHANNELS_PER_TIMER; channel_index++) {

		if (!timer_configs[timer_index].captcomp_channels[channel_index]) {
			continue;
		}

		// Calculate the period for each channel
		const unsigned period = calculate_period(timer_index, channel_index);

		if (period == 0) {
			// If the parsing failed, set the eRPM to 0
			_erpms[channel_index] = 0;

		} else if (period == 65408) {
			// Special case for zero motion (e.g., stationary motor)
			_erpms[channel_index] = 0;

		} else {
			// Convert the period to eRPM
			_erpms[channel_index] = (1000000 * 60) / period;
		}

		// We set it ready anyway, not to hold up other channels when used in round robin.
		_erpms_ready[channel_index] = true;
	}
}

/**
* bits  1-11    - throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit   12      - dshot telemetry enable/disable
* bits  13-16   - XOR checksum
**/
void dshot_motor_data_set(unsigned channel, uint16_t throttle, bool telemetry)
{
	uint8_t timer_index = timer_io_channels[channel].timer_index;
	uint8_t timer_channel_index = timer_io_channels[channel].timer_channel - 1;
	bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];

	if (!channel_initialized) {
		return;
	}

	uint16_t packet = 0;
	uint16_t checksum = 0;

	packet |= throttle << DSHOT_THROTTLE_POSITION;
	packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

	uint16_t csum_data = packet;

	/* XOR checksum calculation */
	csum_data >>= NIBBLES_SIZE;

	for (uint8_t i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
		checksum ^= (csum_data & 0x0F); // XOR data by nibbles
		csum_data >>= NIBBLES_SIZE;
	}

	if (_bidirectional) {
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
	return io_timer_set_enable(armed, _bidirectional ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot,
				   IO_TIMER_ALL_MODES_CHANNELS);
}

int up_bdshot_num_erpm_ready(void)
{
	int num_ready = 0;

	for (unsigned i = 0; i < MAX_NUM_CHANNELS_PER_TIMER; ++i) {
		if (_erpms_ready[i]) {
			++num_ready;
		}
	}

	return num_ready;
}

int up_bdshot_get_erpm(uint8_t output_channel, int *erpm)
{
	uint8_t timer_index = timer_io_channels[output_channel].timer_index;
	uint8_t timer_channel_index = timer_io_channels[output_channel].timer_channel - 1;
	bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];

	if (channel_initialized) {
		*erpm = _erpms[timer_channel_index];
		_erpms_ready[timer_channel_index] = false;
		return PX4_OK;
	}

	// this channel is not configured for dshot
	return PX4_ERROR;
}

int up_bdshot_channel_status(uint8_t channel)
{
	uint8_t timer_index = timer_io_channels[channel].timer_index;
	uint8_t timer_channel_index = timer_io_channels[channel].timer_channel - 1;
	bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];

	// TODO: track that each channel is communicating using the decode stats
	if (channel_initialized) {
		return 1;
	}

	return 0;
}

void up_bdshot_status(void)
{
	PX4_INFO("dshot driver stats:");

	if (_bidirectional) {
		PX4_INFO("Bidirectional DShot enabled");
		PX4_INFO("Available DMA: %u", _num_dma_available);

		if (_num_dma_available < 4) {
			PX4_INFO("Round robin enabled");
		}
	}

	uint8_t timer_index = _bidi_timer_index;

	for (uint8_t timer_channel_index = 0; timer_channel_index < MAX_NUM_CHANNELS_PER_TIMER; timer_channel_index++) {
		bool channel_initialized = timer_configs[timer_index].initialized_channels[timer_channel_index];

		if (channel_initialized) {
			PX4_INFO("Timer %u, Channel %u: read %lu, failed nibble %lu, failed CRC %lu, invalid/zero %lu",
				 timer_index, timer_channel_index,
				 read_ok[timer_channel_index],
				 read_fail_nibble[timer_channel_index],
				 read_fail_crc[timer_channel_index],
				 read_fail_zero[timer_channel_index]);
		}
	}
}

uint8_t nibbles_from_mapped(uint8_t mapped)
{
	switch (mapped) {
	case 0x19:
		return 0x00;

	case 0x1B:
		return 0x01;

	case 0x12:
		return 0x02;

	case 0x13:
		return 0x03;

	case 0x1D:
		return 0x04;

	case 0x15:
		return 0x05;

	case 0x16:
		return 0x06;

	case 0x17:
		return 0x07;

	case 0x1a:
		return 0x08;

	case 0x09:
		return 0x09;

	case 0x0A:
		return 0x0A;

	case 0x0B:
		return 0x0B;

	case 0x1E:
		return 0x0C;

	case 0x0D:
		return 0x0D;

	case 0x0E:
		return 0x0E;

	case 0x0F:
		return 0x0F;

	default:
		// Unknown mapped
		return 0xFF;
	}
}

unsigned calculate_period(uint8_t timer_index, uint8_t channel_index)
{
	uint32_t value = 0;
	uint32_t high = 1; // We start off with high
	unsigned shifted = 0;

	// We can ignore the very first data point as it's the pulse before it starts.
	unsigned previous = dshot_capture_buffer[channel_index][1];

	// Loop through the capture buffer for the specified channel
	for (unsigned i = 2; i < CHANNEL_CAPTURE_BUFF_SIZE; ++i) {

		if (dshot_capture_buffer[channel_index][i] == 0) {
			// Once we get zeros we're through
			break;
		}

		// This seemss to work with dshot 150, 300, 600, 1200
		// The values were found by trial and error to get the quantization just right.
		const uint32_t bits = (dshot_capture_buffer[channel_index][i] - previous + 5) / 20;

		// Convert GCR encoded pulse train into value
		for (unsigned bit = 0; bit < bits; ++bit) {
			value = (value << 1) | high;
			++shifted;
		}

		// The next edge toggles.
		high = !high;

		previous = dshot_capture_buffer[channel_index][i];
	}

	if (shifted == 0) {
		// no data yet, or this time
		++read_fail_zero[channel_index];
		return 0;
	}

	// We need to make sure we shifted 21 times. We might have missed some low "pulses" at the very end.
	value <<= (21 - shifted);

	// From GCR to eRPM according to:
	// https://brushlesswhoop.com/dshot-and-bidirectional-dshot/#erpm-transmission
	unsigned gcr = (value ^ (value >> 1));

	uint32_t data = 0;

	// 20bits -> 5 mapped -> 4 nibbles
	for (unsigned i = 0; i < 4; ++i) {
		uint32_t nibble = nibbles_from_mapped(gcr & 0x1F) << (4 * i);

		if (nibble == 0xFF) {
			++read_fail_nibble[channel_index];;
			return 0;
		}

		data |= nibble;
		gcr >>= 5;
	}

	unsigned shift = (data & 0xE000) >> 13;
	unsigned period = ((data & 0x1FF0) >> 4) << shift;
	unsigned crc = data & 0xF;

	unsigned payload = (data & 0xFFF0) >> 4;
	unsigned calculated_crc = (~(payload ^ (payload >> 4) ^ (payload >> 8))) & 0x0F;

	if (crc != calculated_crc) {
		++read_fail_crc[channel_index];;
		return 0;
	}

	++read_ok[channel_index];;
	return period;
}

#endif
