/****************************************************************************
 *
 * Copyright (C) 2019 PX4 Development Team. All rights reserved.
 * Author: Igor Mišić <igy1000mb@gmail.com>
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

/*
 * @file drv_dshot.c
*/

#include <px4_config.h>
#include <stm32_dma.h>
#include <stm32_tim.h>
#include "drv_dshot.h"
#include "drv_io_timer.h"
#include <drivers/drv_pwm_output.h>

#define DSHOT_DMA_BASE		STM32_DMA2_BASE
#define REG(_reg)			(*(volatile uint32_t *)(DSHOT_DMA_BASE + _reg))

/* DMA registers */
#define rHIFCR			REG(STM32_DMA_HIFCR_OFFSET)
#define rS5CR			REG(STM32_DMA_S5CR_OFFSET)
#define rS5NDTR			REG(STM32_DMA_S5NDTR_OFFSET)
#define rS5PAR			REG(STM32_DMA_S5PAR_OFFSET)
#define rS5M0AR			REG(STM32_DMA_S5M0AR_OFFSET)
#define rS5FCR			REG(STM32_DMA_S5FCR_OFFSET)

#define MOTOR_PWM_BIT_1				14u
#define MOTOR_PWM_BIT_0				7u
#define MOTORS_NUMBER				4u
#define ONE_MOTOR_DATA_SIZE			16u
#define ONE_MOTOR_BUFF_SIZE			18u
#define ALL_MOTORS_BUF_SIZE			(MOTORS_NUMBER * ONE_MOTOR_BUFF_SIZE)
#define DSHOT_THROTTLE_POSITION		5u
#define DSHOT_TELEMETRY_POSITION	4u
#define NIBBLES_SIZE 				4u
#define DSHOT_NUMBER_OF_NIBBLES		3u
#define ARMING_REPETITION			1000u

uint32_t motorBuffer[MOTORS_NUMBER][ONE_MOTOR_BUFF_SIZE] = {0};
uint32_t dshotBurstBuffer[ALL_MOTORS_BUF_SIZE] = {0};

void dshot_dmar_data_prepare(void);

int up_dshot_init(uint32_t channel_mask, unsigned dshot_pwm_freq)
{
	int retVal = ERROR;
	/* Init channels */
	unsigned used_channel;
	for (unsigned channel = 0; channel_mask != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (channel_mask & (1 << channel)) {

			// First free any that were not dshot mode before
			if (-EBUSY == io_timer_is_channel_free(channel)) {
				io_timer_free_channel(channel);
			}

			io_timer_channel_init(channel, IOTimerChanMode_Dshot, NULL, NULL);
			channel_mask &= ~(1 << channel);
			used_channel = channel;
			retVal = OK;
		}
	}

	if (OK == retVal) {
		//Pass one channel to get a timer. In this iteration, only one timer can work as Dshot.
		//TODO: add support for multiple timer Dshot capabilities.
		io_timer_set_dshot_mode(used_channel, dshot_pwm_freq);


		/* DMA setup stream 5*/
		rS5CR |= DMA_SCR_CHSEL(0x6); /* Channel 6 */
		rS5CR |= DMA_SCR_PRIHI;
		rS5CR |= DMA_SCR_MSIZE_32BITS;
		rS5CR |= DMA_SCR_PSIZE_32BITS;
		rS5CR |= DMA_SCR_MINC;
		rS5CR |= DMA_SCR_DIR_M2P;
		rS5CR |= DMA_SCR_TCIE | DMA_SCR_HTIE | DMA_SCR_TEIE | DMA_SCR_DMEIE;

		rS5PAR  = STM32_TIM1_DMAR;
		rS5M0AR = (uint32_t)dshotBurstBuffer;

		rS5FCR &= 0x0;  /* Disable FIFO */
	}

	return retVal;
}

void up_dshot_trigger(void)
{
	dshot_dmar_data_prepare();
	rHIFCR |= 0x3F << 6; //clear DMA stream 5 interrupt flags
	rS5NDTR = ALL_MOTORS_BUF_SIZE;
	rS5CR |= DMA_SCR_EN;
}

/**
* bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit 	12		- dshot telemetry enable/disable
* bits 	13-16	- XOR checksum
**/
void up_dshot_motor_data_prepare(uint32_t motorNumber, uint16_t throttle)
{
	uint16_t packet = 0;
	uint16_t telemetry = 0;
	uint16_t checksum = 0;

	packet |= throttle << DSHOT_THROTTLE_POSITION;
	packet |= telemetry << DSHOT_TELEMETRY_POSITION;

	uint32_t i;
	uint16_t csum_data = packet;

	/* XOR checksum calculation */
	csum_data >>= NIBBLES_SIZE;
	for (i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
		checksum ^= (csum_data & 0x0F); // XOR data by nibbles
		csum_data >>= NIBBLES_SIZE;
	}

	packet |= (checksum & 0x0F);

	for(i = 0; i < ONE_MOTOR_DATA_SIZE; i++) {
		motorBuffer[motorNumber][i] = (packet & 0x8000) ? MOTOR_PWM_BIT_1 : MOTOR_PWM_BIT_0;  // MSB first
		packet <<= 1;
	}

	motorBuffer[motorNumber][16] = 0;
	motorBuffer[motorNumber][17] = 0;
}

void dshot_dmar_data_prepare(void)
{
    for(uint32_t i = 0; i < ONE_MOTOR_BUFF_SIZE ; i++)
    {
        for(uint32_t j = 0; j < MOTORS_NUMBER; j++)
        {
        	dshotBurstBuffer[i*MOTORS_NUMBER+j] = motorBuffer[j][i];
        }
    }
}

int up_dshot_arm(bool armed)
{
	int retVal = ERROR;

	if (true == armed) {

		int success = io_timer_set_enable(true, IOTimerChanMode_Dshot, IO_TIMER_ALL_MODES_CHANNELS);

		if(OK == success) {
			// Arming for dshot is repeating any throttle value less than 47.
			for(uint32_t motorNumber = 0; motorNumber < MOTORS_NUMBER; motorNumber++) {
				up_dshot_motor_data_prepare(motorNumber, 0);
			}

			for(uint32_t i = 0; i < ARMING_REPETITION; i++) {
				up_dshot_trigger();
				usleep(1000);
			}
			retVal = OK;
		}
	} else {
		//disarm by disabling timer
		int success = io_timer_set_enable(false, IOTimerChanMode_Dshot, IO_TIMER_ALL_MODES_CHANNELS);

		if(OK == success) {
			retVal = OK;
		}
	}

	return retVal;
}

