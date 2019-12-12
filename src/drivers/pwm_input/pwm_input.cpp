/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "pwm_input.h"

static PWMIN *g_dev;

int
PWMIN::init()
{
	// NOTE: must first publish here, first publication cannot be in interrupt context
	_pwm_input_pub.update();

	// Initialize the timer for measuring pulse widths
	g_dev->timer_init();

	return PX4_OK;
}

void
PWMIN::timer_init(void)
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

	return PX4_OK;
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
		 static_cast<unsigned>(_pulses_captured),
		 static_cast<unsigned>(_last_period),
		 static_cast<unsigned>(_last_width));
}

namespace pwm_input
{

static int start()
{
	if (g_dev != nullptr) {
		PX4_ERR("driver already started");
		return 1;
	}

	g_dev = new PWMIN();

	if (g_dev == nullptr) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;

	}

	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver init failed");
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int test(void)
{
	uint64_t start_time = hrt_absolute_time();

	PX4_INFO("Showing samples for 5 seconds\n");

	while (hrt_absolute_time() < start_time + 5U * 1000UL * 1000UL) {

		g_dev->print_info();

		// sleep for PWM period (50Hz)
		px4_usleep(20);
	}

	return PX4_OK;
}

static int info(void)
{
	if (g_dev == nullptr) {
		PX4_INFO("driver not started\n");
		return 1;
	}

	g_dev->print_info();
	return PX4_OK;
}

static int usage()
{
	PX4_ERR("unrecognized command, try 'start', 'info', 'reset' or 'test'");

	return PX4_ERROR;
}

} // end namespace pwm_input

extern "C" __EXPORT int pwm_input_main(int argc, char *argv[])
{
	if (argc < 2) {
		return pwm_input::usage();
	}

	const char *verb = argv[1];

	if (!strcmp(verb, "start")) {
		return pwm_input::start();
	}

	if (!strcmp(verb, "info")) {
		return pwm_input::info();
	}

	if (!strcmp(verb, "test")) {
		return pwm_input::test();
	}

	return pwm_input::usage();
}
