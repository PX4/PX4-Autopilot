/****************************************************************************
 *
 * Copyright (C) 2019-2023 PX4 Development Team. All rights reserved.
 * Author: Igor Misic <igy1000mb@gmail.com>
 * Author: Julian Oes <julian@oes.ch>
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


#if (CONFIG_STM32_HAVE_IP_DMA_V1)
//Do nothing. IP DMA V1 MCUs are not supported.
#else

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/log.h>
#include <stm32_dma.h>
#include <stm32_tim.h>
#include <px4_arch/dshot.h>
#include <px4_arch/io_timer.h>
#include <drivers/drv_dshot.h>

#include <stdio.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_hrt.h>


#define MOTOR_PWM_BIT_1				14u
#define MOTOR_PWM_BIT_0				7u
#define DSHOT_TIMERS				MAX_IO_TIMERS
#define MOTORS_NUMBER				DIRECT_PWM_OUTPUT_CHANNELS
#define ONE_MOTOR_DATA_SIZE			16u
#define ONE_MOTOR_BUFF_SIZE			17u
#define ALL_MOTORS_BUF_SIZE			(MOTORS_NUMBER * ONE_MOTOR_BUFF_SIZE)
#define DSHOT_THROTTLE_POSITION		5u
#define DSHOT_TELEMETRY_POSITION	4u
#define NIBBLES_SIZE 				4u
#define DSHOT_NUMBER_OF_NIBBLES		3u
#define DSHOT_END_OF_STREAM 		16u
#define MAX_NUM_CHANNELS_PER_TIMER	4u // CCR1-CCR4

#define DSHOT_DMA_SCR (DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | \
		       DMA_SCR_DIR_M2P | DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

#define DSHOT_BIDIRECTIONAL_DMA_SCR (DMA_SCR_PRIHI | DMA_SCR_MSIZE_16BITS | DMA_SCR_PSIZE_16BITS | DMA_SCR_MINC | \
				     DMA_SCR_DIR_P2M | DMA_SCR_TCIE | DMA_SCR_TEIE | DMA_SCR_DMEIE)

typedef struct dshot_handler_t {
	DMA_HANDLE		dma_handle;
	uint32_t		dma_size;
} dshot_handler_t;

#if defined(CONFIG_ARMV7M_DCACHE)
#  define DMA_BUFFER_MASK    (ARMV7M_DCACHE_LINESIZE - 1)
#  define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)
#else
#define DMA_ALIGN_UP(n) (n)
#endif
#define DSHOT_BURST_BUFFER_SIZE(motors_number) (DMA_ALIGN_UP(sizeof(uint32_t)*ONE_MOTOR_BUFF_SIZE*motors_number))

static dshot_handler_t dshot_handler[DSHOT_TIMERS] = {};
static uint8_t dshot_burst_buffer_array[DSHOT_TIMERS * DSHOT_BURST_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER)]
px4_cache_aligned_data() = {};
static uint32_t *dshot_burst_buffer[DSHOT_TIMERS] = {};

static uint16_t dshot_capture_buffer[32] px4_cache_aligned_data() = {};

static struct hrt_call _call;

static void do_capture(DMA_HANDLE handle, uint8_t status, void *arg);
static void process_capture_results(void *arg);
static unsigned calculate_period(void);
static int dshot_output_timer_init(unsigned channel);
static int dshot_output_timer_deinit(unsigned channel);

static uint32_t read_ok = 0;
static uint32_t read_fail_nibble = 0;
static uint32_t read_fail_crc = 0;
static uint32_t read_fail_zero = 0;

static bool bidirectional_dshot_enabled = true;

static uint32_t _dshot_frequency = 0;
static int _timers_init_mask = 0;
static int _channels_init_mask = 0;

// We only support capture on the first timer (usually 4 channels) for now.
static uint32_t _motor_to_capture = 0;
static int32_t _erpms[4] = {};

static void(*_erpm_callback)(int32_t[], size_t, void *) = NULL;
static void *_erpm_callback_context = NULL;

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

unsigned calculate_period(void)
{
	uint32_t value = 0;

	// We start off with high
	uint32_t high = 1;

	unsigned shifted = 0;
	unsigned previous = 0;

	for (unsigned i = 1; i < (32); ++i) {

		// We can ignore the very first data point as it's the pulse before it starts.
		if (i > 1) {

			if (dshot_capture_buffer[i] == 0) {
				// Once we get zeros we're through.
				break;
			}

			// This seemss to work with dshot 150, 300, 600, 1200
			// The values were found by trial and error to get the quantization just right.
			const uint32_t bits = (dshot_capture_buffer[i] - previous + 5) / 20;

			for (unsigned bit = 0; bit < bits; ++bit) {
				value = value << 1;
				value |= high;
				++shifted;
			}

			// The next edge toggles.
			high = !high;
		}

		previous = dshot_capture_buffer[i];
	}

	if (shifted == 0) {
		// no data yet, or this time
		++read_fail_zero;
		return 0;
	}

	// We need to make sure we shifted 21 times. We might have missed some low "pulses" at the very end.
	value = value << (21 - shifted);

	// Note: At 0 throttle, the value is 0x1AD6AE, so 0b110101101011010101110

	// From GCR to eRPM according to:
	// https://brushlesswhoop.com/dshot-and-bidirectional-dshot/#erpm-transmission
	unsigned gcr = (value ^ (value >> 1));

	uint32_t data = 0;

	// 20bits -> 5 mapped -> 4 nibbles
	for (unsigned i = 0; i < 4; ++i) {
		uint32_t nibble = nibbles_from_mapped(gcr & (0x1F)) << (4 * i);

		if (nibble == 0xff) {
			++read_fail_nibble;
			return 0;
		}

		data |= nibble;
		gcr = gcr >> 5;
	}

	unsigned shift = (data & 0xE000) >> 13;
	unsigned period = ((data & 0x1FF0) >> 4) << shift;
	unsigned crc = (data & 0xf);

	unsigned payload = (data & 0xFFF0) >> 4;
	unsigned calculated_crc = (~(payload ^ (payload >> 4) ^ (payload >> 8))) & 0x0F;

	if (crc != calculated_crc) {
		++read_fail_crc;
		return 0;
	}

	++read_ok;
	return period;
}

int dshot_output_timer_deinit(unsigned channel)
{
	return io_timer_unallocate_channel(channel);
}

int dshot_output_timer_init(unsigned channel)
{
	int ret = io_timer_channel_init(channel,
					bidirectional_dshot_enabled ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot, NULL, NULL);

	if (ret == -EBUSY) {
		// either timer or channel already used - this is not fatal
		return OK;

	} else {
		return ret;
	}
}

int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq, bool enable_bidirectional_dshot)
{
	bidirectional_dshot_enabled = enable_bidirectional_dshot;
	_dshot_frequency = dshot_pwm_freq;

	for (unsigned channel = 0; channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (channel_mask & (1 << channel)) {
			uint8_t timer = timer_io_channels[channel].timer_index;

			if (io_timers[timer].dshot.dma_base == 0) { // board does not configure dshot on this timer
				continue;
			}

			int ret = dshot_output_timer_init(channel);

			if (ret != OK) {
				return ret;
			}

			_channels_init_mask |= (1 << channel);
			_timers_init_mask |= (1 << timer);
		}
	}

	for (uint8_t timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		if (_timers_init_mask & (1 << timer)) {
			if (dshot_handler[timer].dma_handle == NULL) {
				dshot_handler[timer].dma_size = io_timers_channel_mapping.element[timer].channel_count_including_gaps *
								ONE_MOTOR_BUFF_SIZE;

				io_timer_set_dshot_mode(timer, _dshot_frequency,
							io_timers_channel_mapping.element[timer].channel_count_including_gaps);

				dshot_handler[timer].dma_handle = stm32_dmachannel(io_timers[timer].dshot.dmamap);

				if (NULL == dshot_handler[timer].dma_handle) {
					// TODO: how to log this?
					return -ENOSR;
				}

				// We have tested this now but will anyway initialize it again during a trigger, so we can free it again.
				stm32_dmafree(dshot_handler[timer].dma_handle);
				dshot_handler[timer].dma_handle = NULL;
			}
		}
	}

	unsigned buffer_offset = 0;

	for (unsigned timer = 0; timer < DSHOT_TIMERS; ++timer) {
		if (_timers_init_mask & (1 << timer)) {
			if (io_timers[timer].base == 0) { // no more timers configured
				break;
			}

			// we know the uint8_t* cast to uint32_t* is fine, since we're aligned to cache line size
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
			dshot_burst_buffer[timer] = (uint32_t *) &dshot_burst_buffer_array[buffer_offset];
#pragma GCC diagnostic pop
			buffer_offset += DSHOT_BURST_BUFFER_SIZE(
						 io_timers_channel_mapping.element[timer].channel_count_including_gaps);

			if (buffer_offset > sizeof(dshot_burst_buffer_array)) {
				return -EINVAL; // something is wrong with the board configuration or some other logic
			}
		}
	}

	return _channels_init_mask;
}

void up_dshot_trigger(void)
{

	for (unsigned channel = 0; channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (_channels_init_mask & (1 << channel)) {

			// For bidirectional dshot we need to re-initialize the timers every time here.
			// In normal mode, we just do it once.
			int ret = OK;

			if (bidirectional_dshot_enabled) {
				dshot_output_timer_deinit(channel);
				ret = dshot_output_timer_init(channel);
			}

			if (ret != OK) {
				// TODO: what to do here?
				return;
			}
		}
	}

	io_timer_set_enable(true, bidirectional_dshot_enabled ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot,
			    IO_TIMER_ALL_MODES_CHANNELS);

	for (unsigned timer = 0; timer < DSHOT_TIMERS; ++timer) {
		if (_timers_init_mask & (1 << timer)) {

			if (dshot_handler[timer].dma_handle == NULL) {
				dshot_handler[timer].dma_size = io_timers_channel_mapping.element[timer].channel_count_including_gaps *
								ONE_MOTOR_BUFF_SIZE;

				io_timer_set_dshot_mode(timer, _dshot_frequency,
							io_timers_channel_mapping.element[timer].channel_count_including_gaps);

				dshot_handler[timer].dma_handle = stm32_dmachannel(io_timers[timer].dshot.dmamap);

				if (NULL == dshot_handler[timer].dma_handle) {
					// TODO: how to log this?
					return;
				}
			}

			// Flush cache so DMA sees the data
			up_clean_dcache((uintptr_t) dshot_burst_buffer[timer],
					(uintptr_t) dshot_burst_buffer[timer] +
					DSHOT_BURST_BUFFER_SIZE(
						io_timers_channel_mapping.element[timer].channel_count_including_gaps));

			px4_stm32_dmasetup(dshot_handler[timer].dma_handle,
					   io_timers[timer].base + STM32_GTIM_DMAR_OFFSET,
					   (uint32_t)(dshot_burst_buffer[timer]),
					   dshot_handler[timer].dma_size,
					   DSHOT_DMA_SCR);

			// Clean UDE flag before DMA is started
			io_timer_update_dma_req(timer, false);
			// Trigger DMA (DShot Outputs)
			stm32_dmastart(dshot_handler[timer].dma_handle, bidirectional_dshot_enabled ? do_capture : NULL, NULL, false);
			io_timer_update_dma_req(timer, true);
		}
	}
}

void do_capture(DMA_HANDLE handle, uint8_t status, void *arg)
{
	(void)handle;
	(void)status;
	(void)arg;

	for (unsigned timer = 0; timer < DSHOT_TIMERS; ++timer) {
		if (_timers_init_mask & (1 << timer)) {
			if (dshot_handler[timer].dma_handle != NULL) {
				stm32_dmastop(dshot_handler[timer].dma_handle);
				stm32_dmafree(dshot_handler[timer].dma_handle);
				dshot_handler[timer].dma_handle = NULL;
			}

			// TODO: this doesn't scale to more than 4 motors yet
			unsigned capture_channel = _motor_to_capture;

			dshot_handler[timer].dma_size = sizeof(dshot_capture_buffer);

			// Instead of using the UP DMA channel, we need to use the CH1-4 channels.
			// It turns out that we can infer the DMA channel index by starting from the UP channel -5.
			unsigned timer_channel = timer_io_channels[capture_channel].timer_channel;

			switch (timer_channel) {
			case 1:
				dshot_handler[timer].dma_handle = stm32_dmachannel(io_timers[timer].dshot.dmamap - 5 + 1);
				break;

			case 2:
				dshot_handler[timer].dma_handle = stm32_dmachannel(io_timers[timer].dshot.dmamap - 5 + 2);
				break;

			case 3:
				dshot_handler[timer].dma_handle = stm32_dmachannel(io_timers[timer].dshot.dmamap - 5 + 3);
				break;

			case 4:
				dshot_handler[timer].dma_handle = stm32_dmachannel(io_timers[timer].dshot.dmamap - 5 + 4);
				break;
			}

			memset(dshot_capture_buffer, 0, sizeof(dshot_capture_buffer));
			up_clean_dcache((uintptr_t) dshot_capture_buffer,
					(uintptr_t) dshot_capture_buffer +
					sizeof(dshot_capture_buffer));

			px4_stm32_dmasetup(dshot_handler[timer].dma_handle,
					   io_timers[timer].base + STM32_GTIM_DMAR_OFFSET,
					   (uint32_t) dshot_capture_buffer,
					   sizeof(dshot_capture_buffer),
					   DSHOT_BIDIRECTIONAL_DMA_SCR);

			io_timer_unallocate_channel(capture_channel);
			io_timer_channel_init(capture_channel, IOTimerChanMode_CaptureDMA, NULL, NULL);
			io_timer_set_enable(true, IOTimerChanMode_CaptureDMA, 1 << capture_channel);

			up_input_capture_set(capture_channel, Both, 0, NULL, NULL);

			io_timer_capture_update_dma_req(timer, false);
			io_timer_set_capture_mode(timer, _dshot_frequency, capture_channel);
			stm32_dmastart(dshot_handler[timer].dma_handle, NULL, NULL, false);
			io_timer_capture_update_dma_req(timer, true);
		}
	}

	// It takes around 85 us for the ESC to respond, so we should have a result after 150 us, surely.
	hrt_call_after(&_call, 150, process_capture_results, NULL);
}

void process_capture_results(void *arg)
{
	(void)arg;

	// In case DMA is still set up from the last capture, we clear that.
	for (unsigned timer = 0; timer < DSHOT_TIMERS; ++timer) {
		if (_timers_init_mask & (1 << timer)) {
			if (dshot_handler[timer].dma_handle != NULL) {
				stm32_dmastop(dshot_handler[timer].dma_handle);
				stm32_dmafree(dshot_handler[timer].dma_handle);
				dshot_handler[timer].dma_handle = NULL;
			}
		}
	}

	up_invalidate_dcache((uintptr_t)dshot_capture_buffer,
			     (uintptr_t)dshot_capture_buffer +
			     sizeof(dshot_capture_buffer));

	const unsigned period = calculate_period();

	if (period == 0) {
		// If the parsing failed, we get 0.
		_erpms[_motor_to_capture] = 0;

	} else if (period == 65408) {
		// For still, we get this magic 65408 value.
		_erpms[_motor_to_capture] = 0;

	} else {
		// from period in us to eRPM
		_erpms[_motor_to_capture] = 1000000 * 60 / period;
	}

	for (unsigned channel = 0; channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (_channels_init_mask & (1 << channel)) {
			io_timer_unallocate_channel(channel);
			io_timer_channel_init(channel, IOTimerChanMode_DshotInverted, NULL, NULL);
		}
	}

	if (_erpm_callback != NULL) {
		// Only publish every 4th time once all measurements have come in.
		if (_motor_to_capture == 3) {
			_erpm_callback(_erpms, 4, _erpm_callback_context);
		}
	}

	_motor_to_capture = (_motor_to_capture + 1) % 4;
}

/**
* bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit 	12		- dshot telemetry enable/disable
* bits 	13-16	- XOR checksum
**/
void dshot_motor_data_set(unsigned motor_number, uint16_t throttle, bool telemetry)
{
	uint16_t packet = 0;
	uint16_t checksum = 0;

	packet |= throttle << DSHOT_THROTTLE_POSITION;
	packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

	uint16_t csum_data = packet;

	/* XOR checksum calculation */
	csum_data >>= NIBBLES_SIZE;

	for (unsigned i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
		checksum ^= (csum_data & 0x0F); // XOR data by nibbles
		csum_data >>= NIBBLES_SIZE;
	}

	if (bidirectional_dshot_enabled) {
		packet |= ((~checksum) & 0x0F);

	} else {
		packet |= ((checksum) & 0x0F);
	}

	unsigned timer = timer_io_channels[motor_number].timer_index;

	// If this timer is not initialized, we give up here.
	if (!(_timers_init_mask & (1 << timer))) {
		return;
	}

	uint32_t *buffer = dshot_burst_buffer[timer];
	const io_timers_channel_mapping_element_t *mapping = &io_timers_channel_mapping.element[timer];
	unsigned num_motors = mapping->channel_count_including_gaps;
	unsigned timer_channel_index = timer_io_channels[motor_number].timer_channel - mapping->lowest_timer_channel;

	for (unsigned motor_data_index = 0; motor_data_index < ONE_MOTOR_DATA_SIZE; motor_data_index++) {
		buffer[motor_data_index * num_motors + timer_channel_index] =
			(packet & 0x8000) ? MOTOR_PWM_BIT_1 : MOTOR_PWM_BIT_0;  // MSB first
		packet <<= 1;
	}
}

int up_dshot_arm(bool armed)
{
	return io_timer_set_enable(armed, bidirectional_dshot_enabled ? IOTimerChanMode_DshotInverted : IOTimerChanMode_Dshot,
				   IO_TIMER_ALL_MODES_CHANNELS);
}

void up_dshot_set_erpm_callback(void(*callback)(int32_t[], size_t, void *), void *context)
{
	_erpm_callback = callback;
	_erpm_callback_context = context;
}

void print_driver_stats()
{
	PX4_INFO("dshot driver stats: %lu read, %lu failed nibble, %lu failed CRC, %lu invalid/zero",
		 read_ok, read_fail_nibble, read_fail_crc, read_fail_zero);
}

#endif
