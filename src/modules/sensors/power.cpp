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

/**
 * @file power.cpp
 *
 * @author Timothy Scott <timothy@auterion.com>
 */
#include "power.h"

Power::Power() {}

void Power::update(px4_adc_msg_t buf_adc[PX4_MAX_ADC_CHANNELS], int nchannels, float throttle, bool armed)
{

#if BOARD_NUMBER_BRICKS > 0
	/* For legacy support we publish the battery_status for the Battery that is
	 * associated with the Brick that is the selected source for VDD_5V_IN
	 * Selection is done in HW ala a LTC4417 or similar, or may be hard coded
	 * Like in the FMUv4
	 */

	/* Per Brick readings with default unread channels at 0 */
	int32_t bat_current_cnt[BOARD_NUMBER_BRICKS];
	int32_t bat_voltage_cnt[BOARD_NUMBER_BRICKS];

	// The channel readings are not necessarily in a nice order, so we iterate through
	// to find every relevant channel.
	for (int i = 0; i < nchannels; i++) {
		for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {
			/* look for specific channels and process the raw voltage to measurement data */
			if (_analogBatteries[b]->vChannel == buf_adc[i].am_channel) {
				/* Voltage in ADC counts */
				bat_voltage_cnt[b] = buf_adc[i].am_data;

			} else if (_analogBatteries[b]->iChannel == buf_adc[i].am_channel) {
				/* Voltage at current sense resistor in ADC counts */
				bat_current_cnt[b] = buf_adc[i].am_data;
			}
		}
	}

	/* Based on the valid_chan, used to indicate the selected the lowest index
	 * (highest priority) supply that is the source for the VDD_5V_IN
	 * When < 0 none selected
	 */

	int selected_source = -1;

	for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {
		if (_analogBatteries[b]->is_valid() && selected_source < 0) {
			selected_source = b;
		}

		_analogBatteries[b]->updateBatteryStatus(bat_voltage_cnt[b], bat_current_cnt[b], hrt_absolute_time(),
				selected_source == b, b, throttle, armed);
	}

#endif /* BOARD_NUMBER_BRICKS > 0 */

}