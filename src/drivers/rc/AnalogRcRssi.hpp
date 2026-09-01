/****************************************************************************
 *
 *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include <board_config.h>
#include <drivers/drv_adc.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/adc_report.h>

#if defined(ADC_RC_RSSI_CHANNEL)
# define RC_ANALOG_RSSI_CHANNEL ADC_RC_RSSI_CHANNEL
#elif defined(ADC_RSSI_IN_CHANNEL)
# define RC_ANALOG_RSSI_CHANNEL ADC_RSSI_IN_CHANNEL
#endif

/**
 * Analog RSSI from the board RSSI ADC pin, used when the RC protocol
 * does not provide a value (SBUS, PPM).
 */
class AnalogRcRssi
{
public:
	void update()
	{
#if defined(RC_ANALOG_RSSI_CHANNEL)

		if (_adc_report_sub.updated()) {
			adc_report_s adc;

			if (_adc_report_sub.copy(&adc)) {
				for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
					if (adc.channel_id[i] == RC_ANALOG_RSSI_CHANNEL) {
						const float adc_volt = adc.raw_data[i] *
								       adc.v_ref /
								       adc.resolution;

						if (_volt < 0.0f) {
							_volt = adc_volt;
						}

						_volt = _volt * 0.995f + adc_volt * 0.005f;

						/* only allow this to be used if we see a high RSSI once */
						if (_volt > 2.5f) {
							_stable = true;
						}

						break;
					}
				}
			}
		}

#endif
	}

	void fill_missing(int32_t &rssi) const
	{
#if defined(RC_ANALOG_RSSI_CHANNEL)

		if ((rssi < 0) && _stable) {
			const float rssi_analog = ((_volt - 0.2f) / 3.0f) * 100.0f;
			rssi = math::constrain((int)rssi_analog, 0, 100);
		}

#endif
	}

	void print_status() const
	{
#if defined(RC_ANALOG_RSSI_CHANNEL)

		if (_stable) {
			PX4_INFO("vrssi: %dmV", (int)(_volt * 1000.0f));
		}

#endif
	}

private:
#if defined(RC_ANALOG_RSSI_CHANNEL)
	uORB::Subscription _adc_report_sub {ORB_ID(adc_report)};
	float _volt{-1.0f};
	bool _stable{false};
#endif
};
