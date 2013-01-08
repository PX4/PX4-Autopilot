/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file i2c.c
 *
 * I2C communication for the PX4IO module.
 */

#include <stdint.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <stm32_i2c.h>

#define DEBUG
#include "px4io.h"

/*
 * I2C register definitions.
 */
#define I2C_BASE	STM32_I2C1_BASE

#define REG(_reg)	(*(volatile uint32_t *)(I2C_BASE + _reg))

#define rCR1		REG(STM32_I2C_CR1_OFFSET)
#define rCR2		REG(STM32_I2C_CR2_OFFSET)
#define rOAR1		REG(STM32_I2C_OAR1_OFFSET)
#define rOAR2		REG(STM32_I2C_OAR2_OFFSET)
#define rDR		REG(STM32_I2C_DR_OFFSET)
#define rSR1		REG(STM32_I2C_SR1_OFFSET)
#define rSR2		REG(STM32_I2C_SR2_OFFSET)
#define rCCR		REG(STM32_I2C_CCR_OFFSET)
#define rTRISE		REG(STM32_I2C_TRISE_OFFSET)

static int	i2c_interrupt(int irq, void *context);
static void	i2c_dump(void);

void
i2c_init(void)
{
	// enable the i2c block clock and reset it
	modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_I2C1EN);
	modifyreg32(STM32_RCC_APB1RSTR, 0, RCC_APB1RSTR_I2C1RST);
	modifyreg32(STM32_RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST, 0);

	// configure the i2c GPIOs
	stm32_configgpio(GPIO_I2C1_SCL);
	stm32_configgpio(GPIO_I2C1_SDA);

	// soft-reset the block
	rCR1 |= I2C_CR1_SWRST;
	rCR1 = 0;

	// set the frequency value in CR2
	rCR2 &= ~I2C_CR2_FREQ_MASK;
	rCR2 |= STM32_PCLK1_FREQUENCY / 1000000;

	// set divisor and risetime for fast mode
	uint16_t result = STM32_PCLK1_FREQUENCY / (400000 * 25);
	if (result < 1)
		result = 1;
	result = 3;
	rCCR &= ~I2C_CCR_CCR_MASK;
	rCCR |= I2C_CCR_DUTY | I2C_CCR_FS | result;
	rTRISE = (uint16_t)((((STM32_PCLK1_FREQUENCY / 1000000) * 300) / 1000) + 1);

	// set our device address
	rOAR1 = 0x1a << 1;

	// enable event interrupts
	irq_attach(STM32_IRQ_I2C1EV, i2c_interrupt);
	up_enable_irq(STM32_IRQ_I2C1EV);
	rCR2 |= I2C_CR2_ITEVFEN;

	// and enable the I2C port
	rCR1 |= I2C_CR1_ACK | I2C_CR1_PE;

	i2c_dump();
}

static int
i2c_interrupt(int irq, FAR void *context)
{
	uint16_t sr1 = rSR1;

	// XXX not sure what else we need to do here...
	if (sr1 & I2C_SR1_ERRORMASK) {
		//debug("errors 0x%04x", sr1 & I2C_SR1_ERRORMASK);
		//i2c_dump();
		rSR1 = 0;
	}

	if (sr1 & I2C_SR1_ADDR) {

		/* XXX we have been addressed, set up to receive */

		/* clear ADDR */
		(void)rSR2;

		debug("addr");
	}

	if (sr1 & I2C_SR1_RXNE) {

		debug("data 0x%02x", rDR);
	}

	if (sr1 & I2C_SR1_STOPF) {
		/* write to CR1 to clear STOPF */
		rCR1 |= I2C_CR1_PE;

		debug("stop");

		/* XXX handle txn */
	}

	return 0;
}

static void
i2c_dump(void)
{
	debug("CR1   0x%08x  CR2   0x%08x", rCR1,  rCR2);
	debug("OAR1  0x%08x  OAR2  0x%08x", rOAR1, rOAR2);
	debug("CCR   0x%08x  TRISE 0x%08x", rCCR,  rTRISE);
	debug("SR1   0x%08x  SR2   0x%08x", rSR1,  rSR2);
}

