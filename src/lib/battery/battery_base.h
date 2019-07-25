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
 * @file battery_base.h
 *
 * Library calls for battery functionality.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Timothy Scott <timothy@auterion.com>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <drivers/drv_hrt.h>
#include <px4_module_params.h>
#include <drivers/drv_adc.h>
#include <board_config.h>
#include <drivers/boards/common/board_common.h>

/**
 * BatteryBase is a base class for any type of battery.
 *
 * Each of the virtual _get_*() functions corresponds to a parameter used for calibration. However, depending
 * on the type of battery, these values may not come from parameters.
 *
 * Most implementations of BatteryBase will also inherit ModuleParams, and most of the virtual function
 * implementations will look something like:
 * 		float _get_bat_v_empty() override {return _param_bat_v_empty().get();}
 *
 * For a full implementation, see src/modules/sensors/analog_battery.{cpp, h}
 * For a minimal implementation, see src/modules/simulator/simulator.h
 */
class BatteryBase
{
public:
	BatteryBase();

	/**
	 * Reset all battery stats and report invalid/nothing.
	 */
	void reset();

	/**
	 * Get the battery cell count
	 */
	int cell_count() { return _get_bat_n_cells(); }

	/**
	 * Get the empty voltage per cell
	 */
	float empty_cell_voltage() { return _get_bat_v_empty(); }

	/**
	 * Get the full voltage per cell
	 */
	float full_cell_voltage() { return _get_bat_v_charged(); }

	/**
	 * Update current battery status message.
	 *
	 * @param voltage_raw Battery voltage read from ADC, in Volts
	 * @param current_raw Voltage of current sense resistor, in Amps
	 * @param timestamp Time at which the ADC was read (use hrt_absolute_time())
	 * @param selected_source This battery is on the brick that the selected source for selected_source
	 * @param priority: The brick number -1. The term priority refers to the Vn connection on the LTC4417
	 * @param throttle_normalized Throttle of the vehicle, between 0 and 1
	 * @param armed Arming state of the vehicle
	 */
	void updateBatteryStatus(float voltage_v, float current_a, hrt_abstime timestamp,
				 bool selected_source, int priority, float throttle_normalized, bool armed, bool connected);

protected:
	/**
	 * @return Value, in volts, at which a single cell of the battery would be considered empty.
	 */
	virtual float _get_bat_v_empty() = 0;
	/**
	 * @return Value, in volts, at which a single cell of the battery would be considered full.
	 */
	virtual float _get_bat_v_charged() = 0;
	/**
	 * @return Number of cells in series in the battery.
	 */
	virtual int _get_bat_n_cells() = 0;
	/**
	 * @return Total capacity of the battery, in mAh
	 */
	virtual float _get_bat_capacity() = 0;
	/**
	 * @return Voltage drop per cell at full throttle. This implicitly defines the internal resistance of the
	 *     battery. If _get_bat_r_internal() >= to 0, then _get_bat_v_load_drop() is ignored.
	 */
	virtual float _get_bat_v_load_drop() = 0;
	/**
	 * @return Internal resistance of the battery, in Ohms. If non-negative, then this is used instead of
	 *     _get_bat_v_load_drop().
	 */
	virtual float _get_bat_r_internal() = 0;
	/**
	 * @return Low battery threshold, as fraction between 0 and 1.
	 */
	virtual float _get_bat_low_thr() = 0;
	/**
	 * @return Critical battery threshold, as fraction between 0 and 1.
	 */
	virtual float _get_bat_crit_thr() = 0;
	/**
	 * @return Emergency battery threshold, as fraction between 0 and 1.
	 */
	virtual float _get_bat_emergen_thr() = 0;
	/**
	 * @return Source of battery data. 0 = internal power module. 1 = external, via Mavlink
	 */
	virtual int _get_source() = 0;


private:
	void filterVoltage(float voltage_v);
	void filterThrottle(float throttle);
	void filterCurrent(float current_a);
	void sumDischarged(hrt_abstime timestamp, float current_a);
	void estimateRemaining(float voltage_v, float current_a, float throttle, bool armed);
	void determineWarning(bool connected);
	void computeScale();

	bool _battery_initialized = false;
	float _voltage_filtered_v = -1.f;
	float _throttle_filtered = -1.f;
	float _current_filtered_a = -1.f;
	float _discharged_mah = 0.f;
	float _discharged_mah_loop = 0.f;
	float _remaining_voltage = -1.f;		///< normalized battery charge level remaining based on voltage
	float _remaining = -1.f;			///< normalized battery charge level, selected based on config param
	float _scale = 1.f;
	uint8_t _warning;
	hrt_abstime _last_timestamp;

	orb_advert_t _orbAdvert{nullptr};
	int _orbInstance;
	battery_status_s _battery_status;
};
