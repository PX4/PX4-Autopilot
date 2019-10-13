/****************************************************************************
 *
 * Copyright (C) 2019 PX4 Development Team. All rights reserved.
 * Author: Igor Misic <igy1000mb@gmail.com>
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

#include <px4_config.h>
#include <px4_micro_hal.h>
#include <stm32_dma.h>
#include <stm32_tim.h>
#include <px4_arch/dshot.h>
#include <px4_arch/io_timer.h>
#include <drivers/drv_pwm_output.h>


#define REG(_tmr, _reg)			(*(volatile uint32_t *)(io_timers[_tmr].dshot.dma_base + _reg))

/* DMA registers */
#define rLIFCR(_tmr)		REG(_tmr, STM32_DMA_LIFCR_OFFSET)
#define rHIFCR(_tmr)		REG(_tmr, STM32_DMA_HIFCR_OFFSET)

#define rS0CR(_tmr)			REG(_tmr, STM32_DMA_S0CR_OFFSET)
#define rS0NDTR(_tmr)		REG(_tmr, STM32_DMA_S0NDTR_OFFSET)
#define rS0PAR(_tmr)		REG(_tmr, STM32_DMA_S0PAR_OFFSET)
#define rS0M0AR(_tmr)		REG(_tmr, STM32_DMA_S0M0AR_OFFSET)
#define rS0FCR(_tmr)		REG(_tmr, STM32_DMA_S0FCR_OFFSET)

#define rS1CR(_tmr)			REG(_tmr, STM32_DMA_S1CR_OFFSET)
#define rS1NDTR(_tmr)		REG(_tmr, STM32_DMA_S1NDTR_OFFSET)
#define rS1PAR(_tmr)		REG(_tmr, STM32_DMA_S1PAR_OFFSET)
#define rS1M0AR(_tmr)		REG(_tmr, STM32_DMA_S1M0AR_OFFSET)
#define rS1FCR(_tmr)		REG(_tmr, STM32_DMA_S1FCR_OFFSET)

#define rS2CR(_tmr)			REG(_tmr, STM32_DMA_S2CR_OFFSET)
#define rS2NDTR(_tmr)		REG(_tmr, STM32_DMA_S2NDTR_OFFSET)
#define rS2PAR(_tmr)		REG(_tmr, STM32_DMA_S2PAR_OFFSET)
#define rS2M0AR(_tmr)		REG(_tmr, STM32_DMA_S2M0AR_OFFSET)
#define rS2FCR(_tmr)		REG(_tmr, STM32_DMA_S2FCR_OFFSET)

#define rS3CR(_tmr)			REG(_tmr, STM32_DMA_S3CR_OFFSET)
#define rS3NDTR(_tmr)		REG(_tmr, STM32_DMA_S3NDTR_OFFSET)
#define rS3PAR(_tmr)		REG(_tmr, STM32_DMA_S3PAR_OFFSET)
#define rS3M0AR(_tmr)		REG(_tmr, STM32_DMA_S3M0AR_OFFSET)
#define rS3FCR(_tmr)		REG(_tmr, STM32_DMA_S3FCR_OFFSET)

#define rS4CR(_tmr)			REG(_tmr, STM32_DMA_S4CR_OFFSET)
#define rS4NDTR(_tmr)		REG(_tmr, STM32_DMA_S4NDTR_OFFSET)
#define rS4PAR(_tmr)		REG(_tmr, STM32_DMA_S4PAR_OFFSET)
#define rS4M0AR(_tmr)		REG(_tmr, STM32_DMA_S4M0AR_OFFSET)
#define rS4FCR(_tmr)		REG(_tmr, STM32_DMA_S4FCR_OFFSET)

#define rS5CR(_tmr)			REG(_tmr, STM32_DMA_S5CR_OFFSET)
#define rS5NDTR(_tmr)		REG(_tmr, STM32_DMA_S5NDTR_OFFSET)
#define rS5PAR(_tmr)		REG(_tmr, STM32_DMA_S5PAR_OFFSET)
#define rS5M0AR(_tmr)		REG(_tmr, STM32_DMA_S5M0AR_OFFSET)
#define rS5FCR(_tmr)		REG(_tmr, STM32_DMA_S5FCR_OFFSET)

#define rS6CR(_tmr)			REG(_tmr, STM32_DMA_S6CR_OFFSET)
#define rS6NDTR(_tmr)		REG(_tmr, STM32_DMA_S6NDTR_OFFSET)
#define rS6PAR(_tmr)		REG(_tmr, STM32_DMA_S6PAR_OFFSET)
#define rS6M0AR(_tmr)		REG(_tmr, STM32_DMA_S6M0AR_OFFSET)
#define rS6FCR(_tmr)		REG(_tmr, STM32_DMA_S6FCR_OFFSET)

#define rS7CR(_tmr)			REG(_tmr, STM32_DMA_S7CR_OFFSET)
#define rS7NDTR(_tmr)		REG(_tmr, STM32_DMA_S7NDTR_OFFSET)
#define rS7PAR(_tmr)		REG(_tmr, STM32_DMA_S7PAR_OFFSET)
#define rS7M0AR(_tmr)		REG(_tmr, STM32_DMA_S7M0AR_OFFSET)
#define rS7FCR(_tmr)		REG(_tmr, STM32_DMA_S7FCR_OFFSET)

#define MOTOR_PWM_BIT_1				14u
#define MOTOR_PWM_BIT_0				7u
#define DSHOT_TIMERS				MAX_IO_TIMERS
#define MOTORS_NUMBER				DIRECT_PWM_OUTPUT_CHANNELS
#define ONE_MOTOR_DATA_SIZE			16u
#define ONE_MOTOR_BUFF_SIZE			18u
#define ALL_MOTORS_BUF_SIZE			(MOTORS_NUMBER * ONE_MOTOR_BUFF_SIZE)
#define DSHOT_THROTTLE_POSITION		5u
#define DSHOT_TELEMETRY_POSITION	4u
#define NIBBLES_SIZE 				4u
#define DSHOT_NUMBER_OF_NIBBLES		3u
#define MAX_NUM_CHANNELS_PER_TIMER	4u // CCR1-CCR4

typedef struct dshot_handler_t {
	bool			init;
	uint8_t			motors_number;
} dshot_handler_t;

#define DMA_BUFFER_MASK    (PX4_ARCH_DCACHE_LINESIZE - 1)
#define DMA_ALIGN_UP(n)    (((n) + DMA_BUFFER_MASK) & ~DMA_BUFFER_MASK)
#define DSHOT_BURST_BUFFER_SIZE(motors_number) (DMA_ALIGN_UP(sizeof(uint32_t)*ONE_MOTOR_BUFF_SIZE*motors_number))

static dshot_handler_t dshot_handler[DSHOT_TIMERS] = {};
static uint16_t *motor_buffer = NULL;
static uint8_t dshot_burst_buffer_array[DSHOT_TIMERS * DSHOT_BURST_BUFFER_SIZE(MAX_NUM_CHANNELS_PER_TIMER)]
__attribute__((aligned(PX4_ARCH_DCACHE_LINESIZE))); // DMA buffer
static uint32_t *dshot_burst_buffer[DSHOT_TIMERS] = {};

#ifdef BOARD_DSHOT_MOTOR_ASSIGNMENT
static const uint8_t motor_assignment[MOTORS_NUMBER] = BOARD_DSHOT_MOTOR_ASSIGNMENT;
#endif /* BOARD_DSHOT_MOTOR_ASSIGNMENT */

void dshot_dmar_data_prepare(uint8_t timer, uint8_t first_motor, uint8_t motors_number);
int dshot_setup_stream_registers(uint32_t timer);

int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq)
{
	// Alloc buffers if they do not exist. We don't use channel_mask so that potential future re-init calls can
	// use the same buffer.
	if (!motor_buffer) {
		motor_buffer = (uint16_t *)malloc(sizeof(uint16_t) * ALL_MOTORS_BUF_SIZE);

		if (!motor_buffer) {
			return -ENOMEM;
		}
	}

	unsigned buffer_offset = 0;

	for (unsigned timer = 0; timer < DSHOT_TIMERS; ++timer) {
		if (io_timers[timer].base == 0) { // no more timers configured
			break;
		}

		// we know the uint8_t* cast to uint32_t* is fine, since we're aligned to cache line size
#pragma GCC diagnostic ignored "-Wcast-align"
		dshot_burst_buffer[timer] = (uint32_t *)&dshot_burst_buffer_array[buffer_offset];
#pragma GCC diagnostic pop
		buffer_offset += DSHOT_BURST_BUFFER_SIZE(io_timers[timer].dshot.channels_number);

		if (buffer_offset > sizeof(dshot_burst_buffer_array)) {
			return -EINVAL; // something is wrong with the board configuration or some other logic
		}
	}

	/* Init channels */
	int ret_val = OK;

	for (unsigned channel = 0; (channel_mask != 0) && (channel < MAX_TIMER_IO_CHANNELS) && (OK == ret_val); channel++) {
		if (channel_mask & (1 << channel)) {
			uint8_t timer = timer_io_channels[channel].timer_index;

			if (io_timers[timer].dshot.dma_base == 0) { // board does not configure dshot on this timer
				continue;
			}

			// First free any that were not DShot mode before
			if (-EBUSY == io_timer_is_channel_free(channel)) {
				io_timer_free_channel(channel);
			}

			ret_val = io_timer_channel_init(channel, IOTimerChanMode_Dshot, NULL, NULL);

			if (OK == ret_val) {
				channel_mask &= ~(1 << channel);
				dshot_handler[timer].init = true;
			}
		}
	}

	for (uint8_t timer_index = 0; (timer_index < DSHOT_TIMERS) && (OK == ret_val); timer_index++) {

		if (true == dshot_handler[timer_index].init) {
			dshot_handler[timer_index].motors_number = io_timers[timer_index].dshot.channels_number;
			io_timer_set_dshot_mode(timer_index, dshot_pwm_freq, dshot_handler[timer_index].motors_number);
			ret_val = dshot_setup_stream_registers(timer_index);
		}
	}

	return ret_val;
}

int dshot_setup_stream_registers(uint32_t timer)
{
	int ret_val = OK;

	switch (io_timers[timer].dshot.stream) {
	case DShot_Stream0:
		rS0CR(timer) |= DMA_SCR_CHSEL(io_timers[timer].dshot.channel);
		rS0CR(timer) |= DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | DMA_SCR_DIR_M2P;
		rS0CR(timer) |= DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE;
		rS0PAR(timer)  = io_timers[timer].base + STM32_GTIM_DMAR_OFFSET;
		rS0M0AR(timer) = (uint32_t)(dshot_burst_buffer[timer]);
		rS0FCR(timer) &= 0x0;  /* Disable FIFO */
		break;

	case DShot_Stream1:
		rS1CR(timer) |= DMA_SCR_CHSEL(io_timers[timer].dshot.channel);
		rS1CR(timer) |= DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | DMA_SCR_DIR_M2P;
		rS1CR(timer) |= DMA_SCR_DIR_M2P;
		rS1CR(timer) |= DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE;
		rS1PAR(timer)  = io_timers[timer].base + STM32_GTIM_DMAR_OFFSET;
		rS1M0AR(timer) = (uint32_t)(dshot_burst_buffer[timer]);
		rS1FCR(timer) &= 0x0;  /* Disable FIFO */
		break;

	case DShot_Stream2:
		rS2CR(timer) |= DMA_SCR_CHSEL(io_timers[timer].dshot.channel);
		rS2CR(timer) |= DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | DMA_SCR_DIR_M2P;
		rS2CR(timer) |= DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE;
		rS2PAR(timer)  = io_timers[timer].base + STM32_GTIM_DMAR_OFFSET;
		rS2M0AR(timer) = (uint32_t)(dshot_burst_buffer[timer]);
		rS2FCR(timer) &= 0x0;  /* Disable FIFO */
		break;

	case DShot_Stream3:
		rS3CR(timer) |= DMA_SCR_CHSEL(io_timers[timer].dshot.channel);
		rS3CR(timer) |= DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | DMA_SCR_DIR_M2P;
		rS3CR(timer) |= DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE;
		rS3PAR(timer)  = io_timers[timer].base + STM32_GTIM_DMAR_OFFSET;
		rS3M0AR(timer) = (uint32_t)(dshot_burst_buffer[timer]);
		rS3FCR(timer) &= 0x0;  /* Disable FIFO */
		break;

	case DShot_Stream4:
		rS4CR(timer) |= DMA_SCR_CHSEL(io_timers[timer].dshot.channel);
		rS4CR(timer) |= DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | DMA_SCR_DIR_M2P;
		rS4CR(timer) |= DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE;
		rS4PAR(timer)  = io_timers[timer].base + STM32_GTIM_DMAR_OFFSET;
		rS4M0AR(timer) = (uint32_t)(dshot_burst_buffer[timer]);
		rS4FCR(timer) &= 0x0;  /* Disable FIFO */
		break;

	case DShot_Stream5:
		rS5CR(timer) |= DMA_SCR_CHSEL(io_timers[timer].dshot.channel);
		rS5CR(timer) |= DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | DMA_SCR_DIR_M2P;
		rS5CR(timer) |= DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE;
		rS5PAR(timer)  =  io_timers[timer].base + STM32_GTIM_DMAR_OFFSET;
		rS5M0AR(timer) = (uint32_t)(dshot_burst_buffer[timer]);
		rS5FCR(timer) &= 0x0;  /* Disable FIFO */
		break;

	case DShot_Stream6:
		rS6CR(timer) |= DMA_SCR_CHSEL(io_timers[timer].dshot.channel);
		rS6CR(timer) |= DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | DMA_SCR_DIR_M2P;
		rS6CR(timer) |= DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE;
		rS6PAR(timer)  =  io_timers[timer].base + STM32_GTIM_DMAR_OFFSET;
		rS6M0AR(timer) = (uint32_t)(dshot_burst_buffer[timer]);
		rS6FCR(timer) &= 0x0;  /* Disable FIFO */
		break;

	case DShot_Stream7:
		rS7CR(timer) |= DMA_SCR_CHSEL(io_timers[timer].dshot.channel);
		rS7CR(timer) |= DMA_SCR_PRIHI | DMA_SCR_MSIZE_32BITS | DMA_SCR_PSIZE_32BITS | DMA_SCR_MINC | DMA_SCR_DIR_M2P;
		rS7CR(timer) |= DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE;
		rS7PAR(timer)  =  io_timers[timer].base + STM32_GTIM_DMAR_OFFSET;
		rS7M0AR(timer) = (uint32_t)(dshot_burst_buffer[timer]);
		rS7FCR(timer) &= 0x0;  /* Disable FIFO */
		break;

	default:
		ret_val = ERROR;
		break;
	}

	return ret_val;
}

void up_dshot_trigger(void)
{
	uint8_t first_motor = 0;

	for (uint8_t timer = 0; (timer < DSHOT_TIMERS); timer++) {

		if (true == dshot_handler[timer].init) {

			uint32_t dma_int_streamx_mask;
			volatile uint32_t *rSxNDTR;
			volatile uint32_t *rSxCR;
			volatile uint32_t *rxIFCR;

			switch (io_timers[timer].dshot.stream) {
			case DShot_Stream0:
				dma_int_streamx_mask = DMA_INT_STREAM0_MASK;
				rxIFCR = &rLIFCR(timer);
				rSxNDTR = &rS0NDTR(timer);
				rSxCR = &rS0CR(timer);
				break;

			case DShot_Stream1:
				dma_int_streamx_mask = DMA_INT_STREAM1_MASK;
				rxIFCR = &rLIFCR(timer);
				rSxNDTR = &rS1NDTR(timer);
				rSxCR = &rS1CR(timer);
				break;

			case DShot_Stream2:
				dma_int_streamx_mask = DMA_INT_STREAM2_MASK;
				rxIFCR = &rLIFCR(timer);
				rSxNDTR = &rS2NDTR(timer);
				rSxCR = &rS2CR(timer);
				break;

			case DShot_Stream3:
				dma_int_streamx_mask = DMA_INT_STREAM3_MASK;
				rxIFCR = &rLIFCR(timer);
				rSxNDTR = &rS3NDTR(timer);
				rSxCR = &rS3CR(timer);
				break;

			case DShot_Stream4:
				dma_int_streamx_mask = DMA_INT_STREAM4_MASK;
				rxIFCR = &rHIFCR(timer);
				rSxNDTR = &rS4NDTR(timer);
				rSxCR = &rS4CR(timer);
				break;

			case DShot_Stream5:
				dma_int_streamx_mask = DMA_INT_STREAM5_MASK;
				rxIFCR = &rHIFCR(timer);
				rSxNDTR = &rS5NDTR(timer);
				rSxCR = &rS5CR(timer);
				break;

			case DShot_Stream6:
				dma_int_streamx_mask = DMA_INT_STREAM6_MASK;
				rxIFCR = &rHIFCR(timer);
				rSxNDTR = &rS6NDTR(timer);
				rSxCR = &rS6CR(timer);
				break;

			case DShot_Stream7:
			default:
				dma_int_streamx_mask = DMA_INT_STREAM7_MASK;
				rxIFCR = &rHIFCR(timer);
				rSxNDTR = &rS7NDTR(timer);
				rSxCR = &rS7CR(timer);
				break;
			}

			// Check if DMA is still in progress (just to be safe, not expected to happen)
			if (*rSxCR & DMA_SCR_EN) {
				continue;
			}

			uint8_t motors_number = dshot_handler[timer].motors_number;
			dshot_dmar_data_prepare(timer, first_motor, motors_number);

			// Flush cache so DMA sees the data
			up_clean_dcache((uintptr_t)dshot_burst_buffer[timer],
					(uintptr_t)dshot_burst_buffer[timer] + DSHOT_BURST_BUFFER_SIZE(motors_number));

			first_motor += motors_number;

			uint32_t dshot_data_size = motors_number * ONE_MOTOR_BUFF_SIZE;
			*rxIFCR |= dma_int_streamx_mask; //clear interrupt flags
			*rSxNDTR = dshot_data_size;
			io_timer_update_generation(timer);
			*rSxCR |= DMA_SCR_EN; // Trigger DMA
		}
	}
}

/**
* bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit 	12		- dshot telemetry enable/disable
* bits 	13-16	- XOR checksum
**/
static void dshot_motor_data_set(uint32_t motor_number, uint16_t throttle, bool telemetry)
{
	uint16_t packet = 0;
	uint16_t checksum = 0;

#ifdef BOARD_DSHOT_MOTOR_ASSIGNMENT
	motor_number = motor_assignment[motor_number];
#endif /* BOARD_DSHOT_MOTOR_ASSIGNMENT */

	packet |= throttle << DSHOT_THROTTLE_POSITION;
	packet |= ((uint16_t)telemetry & 0x01) << DSHOT_TELEMETRY_POSITION;

	uint32_t i;
	uint16_t csum_data = packet;

	/* XOR checksum calculation */
	csum_data >>= NIBBLES_SIZE;

	for (i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
		checksum ^= (csum_data & 0x0F); // XOR data by nibbles
		csum_data >>= NIBBLES_SIZE;
	}

	packet |= (checksum & 0x0F);

	for (i = 0; i < ONE_MOTOR_DATA_SIZE; i++) {
		motor_buffer[motor_number * ONE_MOTOR_BUFF_SIZE + i] = (packet & 0x8000) ? MOTOR_PWM_BIT_1 :
				MOTOR_PWM_BIT_0;  // MSB first
		packet <<= 1;
	}

	motor_buffer[motor_number * ONE_MOTOR_BUFF_SIZE + 16] = 0;
	motor_buffer[motor_number * ONE_MOTOR_BUFF_SIZE + 17] = 0;
}

void up_dshot_motor_data_set(uint32_t motor_number, uint16_t throttle, bool telemetry)
{
	dshot_motor_data_set(motor_number, throttle + DShot_cmd_MIN_throttle, telemetry);
}

void up_dshot_motor_command(unsigned channel, uint16_t command, bool telemetry)
{
	dshot_motor_data_set(channel, command, telemetry);
}

void dshot_dmar_data_prepare(uint8_t timer, uint8_t first_motor, uint8_t motors_number)
{
	uint32_t *buffer = dshot_burst_buffer[timer];

	for (uint32_t motor_data_index = 0; motor_data_index < ONE_MOTOR_BUFF_SIZE ; motor_data_index++) {
		for (uint32_t motor_index = 0; motor_index < motors_number; motor_index++) {
			buffer[motor_data_index * motors_number + motor_index] = motor_buffer[(motor_index +
					first_motor) * ONE_MOTOR_BUFF_SIZE + motor_data_index];
		}
	}
}

int up_dshot_arm(bool armed)
{
	return io_timer_set_enable(armed, IOTimerChanMode_Dshot, IO_TIMER_ALL_MODES_CHANNELS);
}


#endif
