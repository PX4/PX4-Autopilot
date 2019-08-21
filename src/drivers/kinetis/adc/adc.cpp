/****************************************************************************
 *
 *   Copyright (C) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file adc.cpp
 *
 * TODO:This is stubbed out leving the code intact to document the needed
 * mechinsm for porting.
 *
 * Driver for the Kinetis ADC.
 *
 * This is a low-rate driver, designed for sampling things like voltages
 * and so forth. It avoids the gross complexity of the NuttX ADC driver.
 */

#include <nuttx/analog/adc.h>
#include <kinetis.h>
#include <chip/kinetis_sim.h>
#include <chip/kinetis_adc.h>

#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_config.h>
#include <px4_log.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/system_power.h>

using namespace time_literals;

#if defined(ADC_CHANNELS)

#define ADC_TOTAL_CHANNELS 		32

#define _REG(_addr)	(*(volatile uint32_t *)(_addr))

/* ADC register accessors */

#define REG(a, _reg)	_REG(KINETIS_ADC##a##_BASE + (_reg))

#define rSC1A(adc)  REG(adc, KINETIS_ADC_SC1A_OFFSET) /* ADC status and control registers 1 */
#define rSC1B(adc)  REG(adc, KINETIS_ADC_SC1B_OFFSET) /* ADC status and control registers 1 */
#define rCFG1(adc)  REG(adc, KINETIS_ADC_CFG1_OFFSET) /* ADC configuration register 1 */
#define rCFG2(adc)  REG(adc, KINETIS_ADC_CFG2_OFFSET) /* Configuration register 2 */
#define rRA(adc)    REG(adc, KINETIS_ADC_RA_OFFSET)   /* ADC data result register */
#define rRB(adc)    REG(adc, KINETIS_ADC_RB_OFFSET)   /* ADC data result register */
#define rCV1(adc)   REG(adc, KINETIS_ADC_CV1_OFFSET)  /* Compare value registers */
#define rCV2(adc)   REG(adc, KINETIS_ADC_CV2_OFFSET)  /* Compare value registers */
#define rSC2(adc)   REG(adc, KINETIS_ADC_SC2_OFFSET)  /* Status and control register 2 */
#define rSC3(adc)   REG(adc, KINETIS_ADC_SC3_OFFSET)  /* Status and control register 3 */
#define rOFS(adc)   REG(adc, KINETIS_ADC_OFS_OFFSET)  /* ADC offset correction register */
#define rPG(adc)    REG(adc, KINETIS_ADC_PG_OFFSET)   /* ADC plus-side gain register */
#define rMG(adc)    REG(adc, KINETIS_ADC_MG_OFFSET)   /* ADC minus-side gain register */
#define rCLPD(adc)  REG(adc, KINETIS_ADC_CLPD_OFFSET) /* ADC plus-side general calibration value register */
#define rCLPS(adc)  REG(adc, KINETIS_ADC_CLPS_OFFSET) /* ADC plus-side general calibration value register */
#define rCLP4(adc)  REG(adc, KINETIS_ADC_CLP4_OFFSET) /* ADC plus-side general calibration value register */
#define rCLP3(adc)  REG(adc, KINETIS_ADC_CLP3_OFFSET) /* ADC plus-side general calibration value register */
#define rCLP2(adc)  REG(adc, KINETIS_ADC_CLP2_OFFSET) /* ADC plus-side general calibration value register */
#define rCLP1(adc)  REG(adc, KINETIS_ADC_CLP1_OFFSET) /* ADC plus-side general calibration value register */
#define rCLP0(adc)  REG(adc, KINETIS_ADC_CLP0_OFFSET) /* ADC plus-side general calibration value register */
#define rCLMD(adc)  REG(adc, KINETIS_ADC_CLMD_OFFSET) /* ADC minus-side general calibration value register */
#define rCLMS(adc)  REG(adc, KINETIS_ADC_CLMS_OFFSET) /* ADC minus-side general calibration value register */
#define rCLM4(adc)  REG(adc, KINETIS_ADC_CLM4_OFFSET) /* ADC minus-side general calibration value register */
#define rCLM3(adc)  REG(adc, KINETIS_ADC_CLM3_OFFSET) /* ADC minus-side general calibration value register */
#define rCLM2(adc)  REG(adc, KINETIS_ADC_CLM2_OFFSET) /* ADC minus-side general calibration value register */
#define rCLM1(adc)  REG(adc, KINETIS_ADC_CLM1_OFFSET) /* ADC minus-side general calibration value register */
#define rCLM0(adc)  REG(adc, KINETIS_ADC_CLM0_OFFSET) /* ADC minus-side general calibration value register */

class ADC : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	ADC(uint32_t channels);
	~ADC();

	virtual int		init();

	virtual int		ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t		read(file *filp, char *buffer, size_t len);

protected:
	virtual int		open_first(struct file *filp);
	virtual int		close_last(struct file *filp);

private:
	void			Run() override;

	/**
	 * Sample a single channel and return the measured value.
	 *
	 * @param channel		The channel to sample.
	 * @return			The sampled value, or 0xffff if
	 *				sampling failed.
	 */
	uint16_t		sample(unsigned channel);

	void			update_adc_report(hrt_abstime now);
	void			update_system_power(hrt_abstime now);


	static const hrt_abstime	kINTERVAL{10_ms};	/**< 100Hz base rate */

	perf_counter_t			_sample_perf;

	unsigned			_channel_count{0};
	px4_adc_msg_t			*_samples{nullptr};	/**< sample buffer */

	uORB::Publication<adc_report_s>		_to_adc_report{ORB_ID(adc_report)};
	uORB::Publication<system_power_s>	_to_system_power{ORB_ID(system_power)};
};

ADC::ADC(uint32_t channels) :
	CDev(ADC0_DEVICE_PATH),
	ScheduledWorkItem(px4::wq_configurations::hp_default),
	_sample_perf(perf_alloc(PC_ELAPSED, "adc_samples"))
{
	/* always enable the temperature sensor */
	channels |= 1 << (ADC_SC1_ADCH_TEMP >> ADC_SC1_ADCH_SHIFT);

	/* allocate the sample array */
	for (unsigned i = 0; i < ADC_TOTAL_CHANNELS; i++) {
		if (channels & (1 << i)) {
			_channel_count++;
		}
	}

	if (_channel_count > PX4_MAX_ADC_CHANNELS) {
		PX4_ERR("PX4_MAX_ADC_CHANNELS is too small:is %d needed:%d", PX4_MAX_ADC_CHANNELS, _channel_count);
	}

	_samples = new px4_adc_msg_t[_channel_count];

	/* prefill the channel numbers in the sample array */
	if (_samples != nullptr) {
		unsigned index = 0;

		for (unsigned i = 0; i < ADC_TOTAL_CHANNELS; i++) {
			if (channels & (1 << i)) {
				_samples[index].am_channel = i;
				_samples[index].am_data = 0;
				index++;
			}
		}
	}
}

ADC::~ADC()
{
	if (_samples != nullptr) {
		delete _samples;
	}

	irqstate_t flags = px4_enter_critical_section();
	_REG(KINETIS_SIM_SCGC3) &= ~SIM_SCGC3_ADC1;
	px4_leave_critical_section(flags);

	perf_free(_sample_perf);
}

int board_adc_init()
{
	/* Input is Buss Clock 56 Mhz We will use /8 for 7 Mhz */

	irqstate_t flags = px4_enter_critical_section();

	_REG(KINETIS_SIM_SCGC3) |= SIM_SCGC3_ADC1;
	rCFG1(1) = ADC_CFG1_ADICLK_BUSCLK | ADC_CFG1_MODE_1213BIT | ADC_CFG1_ADIV_DIV8;
	rCFG2(1) = 0;
	rSC2(1) = ADC_SC2_REFSEL_DEFAULT;

	px4_leave_critical_section(flags);

	/* Clear the CALF and begin the calibration */

	rSC3(1) = ADC_SC3_CAL | ADC_SC3_CALF;

	while ((rSC1A(1) & ADC_SC1_COCO) == 0) {
		usleep(100);

		if (rSC3(1) & ADC_SC3_CALF) {
			return -1;
		}
	}

	/* dummy read to clear COCO of calibration */

	int32_t r = rRA(1);

	/* Check the state of CALF at the end of calibration */

	if (rSC3(1) & ADC_SC3_CALF) {
		return -1;
	}

	/* Calculate the calibration values for single ended positive */

	r = rCLP0(1) + rCLP1(1)  + rCLP2(1)  + rCLP3(1)  + rCLP4(1)  + rCLPS(1) ;
	r = 0x8000U | (r >> 1U);
	rPG(1) = r;

	/* Calculate the calibration values for double ended Negitive */

	r = rCLM0(1) + rCLM1(1)  + rCLM2(1)  + rCLM3(1)  + rCLM4(1)  + rCLMS(1) ;
	r = 0x8000U | (r >> 1U);
	rMG(1) = r;

	/* kick off a sample and wait for it to complete */
	hrt_abstime now = hrt_absolute_time();

	rSC1A(1) =  ADC_SC1_ADCH(ADC_SC1_ADCH_TEMP);

	while (!(rSC1A(1) & ADC_SC1_COCO)) {

		/* don't wait for more than 500us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 500) {
			return -1;
		}
	}

	return OK;
}

int
ADC::init()
{
	int rv = board_adc_init();

	if (rv < 0) {
		PX4_DEBUG("sample timeout");
		return rv;
	}

	/* create the device node */
	return CDev::init();
}

int
ADC::ioctl(file *filp, int cmd, unsigned long arg)
{
	return -ENOTTY;
}

ssize_t
ADC::read(file *filp, char *buffer, size_t len)
{
	const size_t maxsize = sizeof(px4_adc_msg_t) * _channel_count;

	if (len > maxsize) {
		len = maxsize;
	}

	/* block interrupts while copying samples to avoid racing with an update */
	irqstate_t flags = px4_enter_critical_section();
	memcpy(buffer, _samples, len);
	px4_leave_critical_section(flags);

	return len;
}

int
ADC::open_first(struct file *filp)
{
	/* get fresh data */
	Run();

	/* and schedule regular updates */
	ScheduleOnInterval(kINTERVAL, kINTERVAL);

	return 0;
}

int
ADC::close_last(struct file *filp)
{
	ScheduleClear();

	return 0;
}

void
ADC::Run()
{
	hrt_abstime now = hrt_absolute_time();

	/* scan the channel set and sample each */
	for (unsigned i = 0; i < _channel_count; i++) {
		_samples[i].am_data = sample(_samples[i].am_channel);
	}

	update_adc_report(now);
	update_system_power(now);
}

void
ADC::update_adc_report(hrt_abstime now)
{
	adc_report_s adc = {};
	adc.timestamp = now;

	unsigned max_num = _channel_count;

	if (max_num > (sizeof(adc.channel_id) / sizeof(adc.channel_id[0]))) {
		max_num = (sizeof(adc.channel_id) / sizeof(adc.channel_id[0]));
	}

	for (unsigned i = 0; i < max_num; i++) {
		adc.channel_id[i] = _samples[i].am_channel;
		adc.channel_value[i] = _samples[i].am_data * 3.3f / 4096.0f;
	}

	_to_adc_report.publish(adc);
}

void
ADC::update_system_power(hrt_abstime now)
{
#if defined (BOARD_ADC_USB_CONNECTED)
	system_power_s system_power {};
	system_power.timestamp = now;

#if defined(ADC_5V_RAIL_SENSE)

	for (unsigned i = 0; i < _channel_count; i++) {

		if (_samples[i].am_channel == ADC_5V_RAIL_SENSE) {
			// it is 2:1 scaled
			system_power.voltage5v_v = _samples[i].am_data * (6.6f / 4096.0f);
		}
	}

#endif

	/* Note once the board_config.h provides BOARD_ADC_USB_CONNECTED,
	 * It must provide the true logic GPIO BOARD_ADC_xxxx macros.
	 */
	// these are not ADC related, but it is convenient to
	// publish these to the same topic

	system_power.usb_connected = BOARD_ADC_USB_CONNECTED;
	/* If provided used the Valid signal from HW*/
#if defined(BOARD_ADC_USB_VALID)
	system_power.usb_valid = BOARD_ADC_USB_VALID;
#else
	/* If not provided then use connected */
	system_power.usb_valid  = system_power.usb_connected;
#endif

	system_power.brick_valid   = BOARD_ADC_BRICK_VALID;
	system_power.servo_valid   = BOARD_ADC_SERVO_VALID;

	// OC pins are active low
	system_power.periph_5v_oc  = BOARD_ADC_PERIPH_5V_OC;
	system_power.hipower_5v_oc = BOARD_ADC_HIPOWER_5V_OC;

	/* lazily publish */
	_to_system_power.publish(system_power);

#endif // BOARD_ADC_USB_CONNECTED
}

uint16_t board_adc_sample(unsigned channel)
{
	irqstate_t flags = px4_enter_critical_section();

	/* clear any previous COCC */
	rRA(1);

	/* run a single conversion right now - should take about 35 cycles (5 microseconds) max */
	rSC1A(1) = ADC_SC1_ADCH(channel);

	/* wait for the conversion to complete */
	const hrt_abstime now = hrt_absolute_time();

	while (!(rSC1A(1) & ADC_SC1_COCO)) {

		/* don't wait for more than 10us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 10) {
			px4_leave_critical_section(flags);
			return 0xffff;
		}
	}

	/* read the result and clear EOC */
	uint16_t result = rRA(1);

	px4_leave_critical_section(flags);

	return result;
}

uint16_t
ADC::sample(unsigned channel)
{
	perf_begin(_sample_perf);
	uint16_t result = board_adc_sample(channel);

	if (result == 0xffff) {
		PX4_ERR("sample timeout");
	}

	perf_end(_sample_perf);
	return result;
}

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int adc_main(int argc, char *argv[]);

namespace
{
ADC	*g_adc{nullptr};

int
test(void)
{

	int fd = open(ADC0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("can't open ADC device %d", errno);
		return 1;
	}

	for (unsigned i = 0; i < 50; i++) {
		px4_adc_msg_t data[ADC_TOTAL_CHANNELS];
		ssize_t count = read(fd, data, sizeof(data));

		if (count < 0) {
			PX4_ERR("read error");
			return 1;
		}

		unsigned channels = count / sizeof(data[0]);

		for (unsigned j = 0; j < channels; j++) {
			printf("%d: %u  ", data[j].am_channel, data[j].am_data);
		}

		printf("\n");
		px4_usleep(500000);
	}

	return 0;
}
}

int
adc_main(int argc, char *argv[])
{
	if (g_adc == nullptr) {
		/* XXX this hardcodes the default channel set for the board in board_config.h - should be configurable */
		g_adc = new ADC(ADC_CHANNELS);

		if (g_adc == nullptr) {
			PX4_ERR("couldn't allocate the ADC driver");
			return 1;
		}

		if (g_adc->init() != OK) {
			delete g_adc;
			PX4_ERR("ADC init failed");
			return 1;
		}
	}

	if (argc > 1) {
		if (!strcmp(argv[1], "test")) {
			return test();
		}
	}

	return 0;
}
#endif
