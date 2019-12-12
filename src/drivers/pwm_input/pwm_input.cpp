/****************************************************************************
 *
 *   Copyright (c) 2015 Andrew Tridgell. All rights reserved.
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

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/pwm_input.h>

#include <drivers/device/device.h>


#if HRT_TIMER == PWMIN_TIMER
#error cannot share timer between HRT and PWMIN
#endif

#if !defined(GPIO_PWM_IN) || !defined(PWMIN_TIMER) || !defined(PWMIN_TIMER_CHANNEL)
#error PWMIN defines are needed in board_config.h for this board
#endif

/* Get the timer defines */
#define INPUT_TIMER PWMIN_TIMER
#include "timer_registers.h"
#define PWMIN_TIMER_BASE	TIMER_BASE
#define PWMIN_TIMER_CLOCK	TIMER_CLOCK
#define PWMIN_TIMER_POWER_REG	TIMER_CLOCK_POWER_REG
#define PWMIN_TIMER_POWER_BIT	TIMER_CLOCK_POWER_BIT
#define PWMIN_TIMER_VECTOR	TIMER_IRQ_REG

/*
 * HRT clock must be at least 1MHz
 */
#if PWMIN_TIMER_CLOCK <= 1000000
# error PWMIN_TIMER_CLOCK must be greater than 1MHz
#endif

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(PWMIN_TIMER_BASE + _reg))

#define rCR1		REG(STM32_GTIM_CR1_OFFSET)
#define rCR2		REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR		REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER		REG(STM32_GTIM_DIER_OFFSET)
#define rSR		REG(STM32_GTIM_SR_OFFSET)
#define rEGR		REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1		REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2		REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER		REG(STM32_GTIM_CCER_OFFSET)
#define rCNT		REG(STM32_GTIM_CNT_OFFSET)
#define rPSC		REG(STM32_GTIM_PSC_OFFSET)
#define rARR		REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1		REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2		REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3		REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4		REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR		REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR		REG(STM32_GTIM_DMAR_OFFSET)

/*
 * Specific registers and bits used by HRT sub-functions
 */
#if PWMIN_TIMER_CHANNEL == 1
#define rCCR_PWMIN_A		rCCR1			/* compare register for PWMIN */
#define DIER_PWMIN_A		(GTIM_DIER_CC1IE) 	/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_A		GTIM_SR_CC1IF		/* interrupt status for PWMIN */
#define rCCR_PWMIN_B		rCCR2 			/* compare register for PWMIN */
#define SR_INT_PWMIN_B		GTIM_SR_CC2IF		/* interrupt status for PWMIN */
#define CCMR1_PWMIN		((0x02 << GTIM_CCMR1_CC2S_SHIFT) | (0x01 << GTIM_CCMR1_CC1S_SHIFT))
#define CCMR2_PWMIN		0
#define CCER_PWMIN		(GTIM_CCER_CC2P | GTIM_CCER_CC1E | GTIM_CCER_CC2E)
#define SR_OVF_PWMIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)
#define SMCR_PWMIN_1		(0x05 << GTIM_SMCR_TS_SHIFT)
#define SMCR_PWMIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#elif PWMIN_TIMER_CHANNEL == 2
#define rCCR_PWMIN_A		rCCR2			/* compare register for PWMIN */
#define DIER_PWMIN_A		(GTIM_DIER_CC2IE)	/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_A		GTIM_SR_CC2IF		/* interrupt status for PWMIN */
#define rCCR_PWMIN_B		rCCR1			/* compare register for PWMIN */
#define DIER_PWMIN_B		GTIM_DIER_CC1IE		/* interrupt enable for PWMIN */
#define SR_INT_PWMIN_B		GTIM_SR_CC1IF		/* interrupt status for PWMIN */
#define CCMR1_PWMIN		((0x01 << GTIM_CCMR1_CC2S_SHIFT) | (0x02 << GTIM_CCMR1_CC1S_SHIFT))
#define CCMR2_PWMIN		0
#define CCER_PWMIN		(GTIM_CCER_CC1P | GTIM_CCER_CC1E | GTIM_CCER_CC2E)
#define SR_OVF_PWMIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)
#define SMCR_PWMIN_1		(0x06 << GTIM_SMCR_TS_SHIFT)
#define SMCR_PWMIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#else
#error PWMIN_TIMER_CHANNEL must be either 1 and 2.
#endif

class PWMIN
{
public:
	int init();

	void publish(uint16_t status, uint32_t period, uint32_t pulse_width);
	void print_info(void);

private:
	void _timer_init(void);

	uint32_t _error_count{};
	uint32_t _pulses_captured{};
	uint32_t _last_period{};
	uint32_t _last_width{};

	bool _timer_started{};

	pwm_input_s _pwm{};

	uORB::PublicationData<pwm_input_s>	_pwm_input_pub{ORB_ID(pwm_input)};

};

static int pwmin_tim_isr(int irq, void *context, void *arg);
static void pwmin_start();
static void pwmin_info(void);
static void pwmin_test(void);
static void pwmin_usage(void);

static PWMIN *g_dev;

int
PWMIN::init()
{
	// TODO: why does update fail if it is not first called here?
	_pwm_input_pub.update();

	// Initialize the timer for measuring pulse widths
	g_dev->_timer_init();

	return OK;
}

void
PWMIN::_timer_init(void)
{
	/* run with interrupts disabled in case the timer is already
	 * setup. We don't want it firing while we are doing the setup */
	irqstate_t flags = px4_enter_critical_section();

	/* configure input pin */
	px4_arch_configgpio(GPIO_PWM_IN);

	/* claim our interrupt vector */
	irq_attach(PWMIN_TIMER_VECTOR, pwmin_tim_isr, NULL);

	/* Clear no bits, set timer enable bit.*/
	modifyreg32(PWMIN_TIMER_POWER_REG, 0, PWMIN_TIMER_POWER_BIT);

	/* disable and configure the timer */
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = DIER_PWMIN_A;
	rCCER = 0;		/* unlock CCMR* registers */
	rCCMR1 = CCMR1_PWMIN;
	rCCMR2 = CCMR2_PWMIN;
	rSMCR = SMCR_PWMIN_1;	/* Set up mode */
	rSMCR = SMCR_PWMIN_2;	/* Enable slave mode controller */
	rCCER = CCER_PWMIN;
	rDCR = 0;

	/* for simplicity scale by the clock in MHz. This gives us
	 * readings in microseconds which is typically what is needed
	 * for a PWM input driver */
	uint32_t prescaler = PWMIN_TIMER_CLOCK / 1000000UL;

	/*
	 * define the clock speed. We want the highest possible clock
	 * speed that avoids overflows.
	 */
	rPSC = prescaler - 1;

	/* run the full span of the counter. All timers can handle
	 * uint16 */
	rARR = UINT16_MAX;

	/* generate an update event; reloads the counter, all registers */
	rEGR = GTIM_EGR_UG;

	/* enable the timer */
	rCR1 = GTIM_CR1_CEN;

	px4_leave_critical_section(flags);

	/* enable interrupts */
	up_enable_irq(PWMIN_TIMER_VECTOR);
}

void
PWMIN::publish(uint16_t status, uint32_t period, uint32_t pulse_width)
{
	/* if we missed an edge, we have to give up */
	if (status & SR_OVF_PWMIN) {
		_error_count++;
		return;
	}

	_pwm.timestamp = hrt_absolute_time();
	_pwm.error_count = _error_count;
	_pwm.period = period;
	_pwm.pulse_width = pulse_width;

	_pwm_input_pub.publish(_pwm);

	// update statistics
	_last_period = period;
	_last_width = pulse_width;
	_pulses_captured++;
}

void
PWMIN::print_info(void)
{
	PX4_INFO("count=%u period=%u width=%u\n",
		 (unsigned)_pulses_captured,
		 (unsigned)_last_period,
		 (unsigned)_last_width);
}

static int pwmin_tim_isr(int irq, void *context, void *arg)
{
	uint16_t status = rSR;
	uint32_t period = rCCR_PWMIN_A;
	uint32_t pulse_width = rCCR_PWMIN_B;

	/* ack the interrupts we just read */
	rSR = 0;

	if (g_dev != nullptr) {
		g_dev->publish(status, period, pulse_width);
	}

	return OK;
}

static void pwmin_start()
{
	if (g_dev != nullptr) {
		errx(1, "driver already started");
	}

	g_dev = new PWMIN();

	if (g_dev == nullptr) {
		errx(1, "driver allocation failed");
	}

	if (g_dev->init() != OK) {
		errx(1, "driver init failed");
	}

	exit(0);
}

static void pwmin_test(void)
{
	uint64_t start_time = hrt_absolute_time();

	PX4_INFO("Showing samples for 5 seconds\n");

	while (hrt_absolute_time() < start_time + 5U * 1000UL * 1000UL) {

		g_dev->print_info();

		// sleep for PWM period (50Hz)
		px4_usleep(20);
	}

	exit(0);
}

static void pwmin_info(void)
{
	if (g_dev == nullptr) {
		PX4_INFO("driver not started\n");
		exit(1);
	}

	g_dev->print_info();
	exit(0);
}

static void pwmin_usage()
{
	PX4_ERR("unrecognized command, try 'start', 'info', 'reset' or 'test'");
}

extern "C" __EXPORT int pwm_input_main(int argc, char *argv[])
{
	if (argc < 2) {
		pwmin_usage();
		return -1;
	}

	const char *verb = argv[1];

	if (!strcmp(verb, "start")) {
		pwmin_start();
	}

	if (!strcmp(verb, "info")) {
		pwmin_info();
	}

	if (!strcmp(verb, "test")) {
		pwmin_test();
	}

	pwmin_usage();
	return -1;
}
