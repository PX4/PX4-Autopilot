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
 * @file power.h
 *
 * @author Timothy Scott <timothy@auterion.com>
 */

#pragma once

#include <board_config.h>
#include <battery/battery.h>

#ifdef BOARD_NUMBER_DIGITAL_BRICKS
#define TOTAL_BRICKS (BOARD_NUMBER_BRICKS + BOARD_NUMBER_DIGITAL_BRICKS)
#else
#define TOTAL_BRICKS BOARD_NUMBER_BRICKS
#endif

/**
 * Measures voltage, current, etc. of all batteries connected to the vehicle, both
 * analog and digital.
 */
class Power
{
public:
	Power();

	/**
	 * Updates the measurements of each battery.
	 *
	 * If the parameter `BAT_SOURCE` == 0, this function will also publish an instance of the uORB topic
	 * `battery_status` for each battery. For reasons of backwards compability, instance 0 will always be the
	 * primary battery. However, this may change in the future! In the future, please use orb_priority() to find
	 * the primary battery.
	 * @param buf_adc Buffer of ADC readings
	 * @param nchannels Number of valid ADC readings in `buf_adc`
	 * @param throttle Normalized throttle (between 0 and 1, or maybe between -1 and 1 in the future)
	 * @param armed True if the vehicle is armed
	 */
	void update(px4_adc_msg_t buf_adc[PX4_MAX_ADC_CHANNELS], int nchannels, float throttle, bool armed);

private:

	/*
	 * All of these #if's are doing one thing: Building an array of `BatteryBase` objects, one
	 * for each possible connected battery. A `BatteryBase` object does not mean that a battery IS connected,
	 * it just means that one CAN be connected.
	 *
	 * For an example of what this looks like after preprocessing, assume that BOARD_NUMBER_BRICKS = 2:
	 * ```
	 * Battery1 _battery0;
	 * Battery2 _battery1;
	 *
	 * BatteryBase *_analogBatteries[2] {
	 *     &_battery0,
	 *     &_battery1,
	 * }
	 */

	// TODO: Add digital batteries

#if BOARD_NUMBER_BRICKS > 0
	Battery1 _battery0;
#endif
#if BOARD_NUMBER_BRICKS > 1
	Battery2 _battery1;
#endif

	BatteryBase *_analogBatteries[BOARD_NUMBER_BRICKS] {
#if BOARD_NUMBER_BRICKS > 0
		&_battery0,
#endif
#if BOARD_NUMBER_BRICKS > 1
		&_battery1,
#endif
	}; // End _analogBatteries

};
