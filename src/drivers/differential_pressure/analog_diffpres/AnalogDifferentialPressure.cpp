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

#include "MS4525.hpp"

/* Measurement rate is 100Hz */
static constexpr int MEAS_RATE{100};
static constexpr uint32_t CONVERSION_INTERVAL{1000000 / MEAS_RATE};

AnalogDifferentialPressure::AnalogDifferentialPressure(uint8_t channel) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_diff_press(0 /* device id */)
{
	//_px4_diff_press.set_device_type(DRV_DIFF_PRESS_DEVTYPE_MS4525);

	updateParams();
}

void
AnalogDifferentialPressure::start()
{
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	DevMgr::getHandle(ADC0_DEVICE_PATH, _h_adc);

	if (!_h_adc.isValid()) {
		PX4_ERR("no ADC found: %s (%d)", ADC0_DEVICE_PATH, _h_adc.getError());
		return PX4_ERROR;
	}


#endif // ADC_AIRSPEED_VOLTAGE_CHANNEL

	ScheduleOnInterval(10000); // 100 Hz
}

void
AnalogDifferentialPressure::stop()
{
	ScheduleClear();
}

void
AnalogDifferentialPressure::Run()
{
	// TODO: check for param update

	perf_begin(_sample_perf);

	if (_parameters.diff_pres_analog_scale > 0.0f) {

		hrt_abstime t = hrt_absolute_time();
		/* make space for a maximum of twelve channels (to ensure reading all channels at once) */
		px4_adc_msg_t buf_adc[PX4_MAX_ADC_CHANNELS];
		/* read all channels available */
		int ret = _h_adc.read(&buf_adc, sizeof(buf_adc));

		if (ret >= (int)sizeof(buf_adc[0])) {

			/* Read add channels we got */
			for (unsigned i = 0; i < ret / sizeof(buf_adc[0]); i++) {
				if (ADC_AIRSPEED_VOLTAGE_CHANNEL == buf_adc[i].am_channel) {

					/* calculate airspeed, raw is the difference from */
					const float voltage = (float)(buf_adc[i].am_data) * 3.3f / 4096.0f * 2.0f;  // V_ref/4096 * (voltage divider factor)

					/**
					 * The voltage divider pulls the signal down, only act on
					 * a valid voltage from a connected sensor. Also assume a non-
					 * zero offset from the sensor if its connected.
					 */
					if (voltage > 0.4f) {
						const float diff_pres_pa_raw = voltage * _param_sens_dpres_ansc;

						_diff_pres.update(t, diff_pres_pa_raw);
					}
				}
			}
		}
	}

	perf_end(_sample_perf);

	return ret;
}
