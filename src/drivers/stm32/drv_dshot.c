/****************************************************************************
 *
 *   Copyright (C) 2012, 2019 PX4 Development Team. All rights reserved.
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

#define DSHOT_DMA_BASE	STM32_DMA2_BASE

#define REG(_reg)	(*(volatile uint32_t *)(DSHOT_DMA_BASE + _reg))

/* DMA registers */
#define rS5CR			REG(STM32_DMA_S5CR_OFFSET)
#define rS5FCR			REG(STM32_DMA_S5FCR_OFFSET)
#define rS5M0AR			REG(STM32_DMA_S5M0AR_OFFSET)
#define rS5PAR			REG(STM32_DMA_S5PAR_OFFSET)

#define MOTOR_PWM_BIT_1			14u
#define MOTOR_PWM_BIT_0			7u
#define MOTORS_NUMBER			MAX_TIMER_IO_CHANNELS
#define ONE_MOTOR_BUFF_SIZE		18u
#define ALL_MOTORS_BUF_SIZE		(MAX_TIMER_IO_CHANNELS * ONE_MOTOR_BUFF_SIZE)

uint32_t dshotBurstBuffer[ALL_MOTORS_BUF_SIZE] = {0};

static int dshot_dma_isr(int irq, void *context, void *arg);

void dshot_dma_init(void)
{
	/* claim our interrupt vector */
	irq_attach(STM32_IRQ_DMA2S5, dshot_dma_isr, NULL);
	/* enable interrupts */
	up_enable_irq(STM32_IRQ_DMA2S5);

	/* DMA setup stream 5*/
	rS5CR |= DMA_SCR_CHSEL(0x6); /* Channel 6 */
	rS5CR |= DMA_SCR_PRIHI;
	rS5CR |= DMA_SCR_MSIZE_32BITS;
	rS5CR |= DMA_SCR_PSIZE_32BITS;
	rS5CR |= DMA_SCR_MINC;
	rS5CR |= DMA_SCR_DIR_M2P;

	rS5FCR &= 0x0;  /* Disable FIFO */

	rS5M0AR = (uint32_t)dshotBurstBuffer;
	rS5PAR  = STM32_TIM1_DMAR;
}
static int
dshot_dma_isr(int irq, void *context, void *arg)
{
	return OK;
}

/**
* bits 	1-11	- throttle value (0-47 are reserved, 48-2047 give 2000 steps of throttle resolution)
* bit 	12		- dshot telemetry enable/disable
* bits 	13-16	- XOR checksum
**/
void dshot_data_prepare(uint16_t throttle, uint32_t* dmaBuffer)
{
	uint16_t packet = 0;
	uint16_t telemetry = 0;
	uint16_t checksum = 0;

	packet |= throttle << 5;
	packet |= telemetry << 4;

	uint32_t i;
	uint16_t csum_data = packet;

	csum_data >>= 4;
	for (i = 0; i < 3; i++) {
		checksum ^= (csum_data & 0x0F); // xor data by nibbles
		csum_data >>= 4;
	}

	packet |= (checksum & 0x0F);

	for(i = 0; i < 16; i++) {
		dmaBuffer[i] = (packet & 0x8000) ? MOTOR_PWM_BIT_1 : MOTOR_PWM_BIT_0;  // MSB first
		packet <<= 1;
	}

	dmaBuffer[16] = 0;
	dmaBuffer[17] = 0;
}

void dshot_dmar_data_prepare(uint32_t bufferIn[MOTORS_NUMBER][ONE_MOTOR_BUFF_SIZE], uint32_t* bufferOut)
{
    for(uint32_t i = 0; i < ONE_MOTOR_BUFF_SIZE ; i++)
    {
        for(uint32_t j = 0; j < MOTORS_NUMBER; j++)
        {
            bufferOut[i*MOTORS_NUMBER+j] = bufferIn[j][i];
        }
    }
}

