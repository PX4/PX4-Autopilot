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
 * @file vl53l1x.cpp
 *
 * Driver for the vl53l1x ToF Sensor from ST Microelectronics connected via I2C.
 */

#include "VL53L1X.hpp"

#define _BSD_SOURCE
#include <endian.h>

//#define VL53L1X_DEBUG 1

using namespace time_literals;

VL53L1X::VL53L1X(uint8_t rotation, int bus, int address) :
	I2C("VL53L1X", nullptr, bus, address, 400000),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_rotation(rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "vl53l1x_read")),
	_comms_errors(perf_alloc(PC_COUNT, "vl53l1x_com_err"))
{
	//_debug_enabled = true;

	// up the retries since the device misses the first measure attempts
	I2C::_retries = 3;
}

VL53L1X::~VL53L1X()
{
	/* make sure we are truly inactive */
	stop();

	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

// check sensor ID registers
bool
VL53L1X::check_id()
{
	uint8_t v1, v2;

	if (!(read_register(0x010F, v1) &&
	      read_register(0x0110, v2))) {

		PX4_ERR("check_id failed");

		return false;
	}

	if ((v1 != 0xEA) ||
	    (v2 != 0xCC)) {

		PX4_ERR("check_id failed (v1, v2)");

		return false;
	}

	PX4_INFO("Detected VL53L1X on bus");

	return true;
}

bool
VL53L1X::initSensor()
{
	uint8_t pad_i2c_hv_extsup_config = 0;
	uint16_t mm_config_outer_offset_mm = 0;

	// setup for 2.8V operation
	if (!(read_register(PAD_I2C_HV__EXTSUP_CONFIG, pad_i2c_hv_extsup_config) &&
	      write_register(PAD_I2C_HV__EXTSUP_CONFIG, pad_i2c_hv_extsup_config | 0x01))) {

		PX4_ERR("PAD_I2C_HV__EXTSUP_CONFIG fail");
	}

	// store oscillator info for later use
	if (!(read_register16(OSC_MEASURED__FAST_OSC__FREQUENCY, fast_osc_frequency) &&
	      read_register16(RESULT__OSC_CALIBRATE_VAL, osc_calibrate_val))) {

		PX4_ERR("OSC_MEASURED__FAST_OSC__FREQUENCY fail");
	}

	// static config
	if (!(write_register16(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate) && // should already be this value after reset
	      write_register(GPIO__TIO_HV_STATUS, 0x02) &&
	      write_register(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8) && // tuning parm default
	      write_register(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16) && // tuning parm default
	      write_register(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01) &&
	      write_register(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF) &&
	      write_register(ALGO__RANGE_MIN_CLIP, 0) && // tuning parm default
	      write_register(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2) // tuning parm default
	     )) {

		PX4_ERR("DSS_CONFIG__TARGET_TOTAL_RATE_MCPS fail");
	}

	// general config
	if (!(write_register16(SYSTEM__THRESH_RATE_HIGH, 0x0000) &&
	      write_register16(SYSTEM__THRESH_RATE_LOW, 0x0000) &&
	      write_register(DSS_CONFIG__APERTURE_ATTENUATION, 0x38))) {

		PX4_ERR("SYSTEM__THRESH_RATE_HIGH fail");
	}

	// timing config
	if (!(write_register16(RANGE_CONFIG__SIGMA_THRESH, 360) && // tuning parm default
	      write_register16(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192) // tuning parm default
	     )) {
		PX4_ERR("RANGE_CONFIG__SIGMA_THRESH fail");
	}

	// dynamic config
	if (!(write_register(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01) &&
	      write_register(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01) &&
	      write_register(SD_CONFIG__QUANTIFIER, 2) // tuning parm default
	     )) {
		PX4_ERR("SYSTEM__GROUPED_PARAMETER_HOLD_0 fail");
	}


	// from VL53L1_preset_mode_timed_ranging_*
	// GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
	// and things don't seem to work if we don't set GPH back to 0 (which the API
	// does here).
	if (!(
		    write_register(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00) &&
		    write_register(SYSTEM__SEED_CONFIG, 1) // tuning parm default
	    )) {
		PX4_ERR("SYSTEM__GROUPED_PARAMETER_HOLD fail");
	}


	// from VL53L1_config_low_power_auto_mode
	if (!(write_register(SYSTEM__SEQUENCE_CONFIG, 0x8B) && // VHV, PHASECAL, DSS1, RANGE
	      write_register16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8) &&
	      write_register(DSS_CONFIG__ROI_MODE_CONTROL, 2) && // REQUESTED_EFFFECTIVE_SPADS
	      read_register16(MM_CONFIG__OUTER_OFFSET_MM, mm_config_outer_offset_mm) &&
	      setDistanceMode(Long) &&
	      setMeasurementTimingBudget(40000))) {

		PX4_ERR("SYSTEM__SEQUENCE_CONFIG fail");
	}

	// the API triggers this change in VL53L1_init_and_start_range() once a
	// measurement is started; assumes MM1 and MM2 are disabled
	if (!(write_register16(ALGO__PART_TO_PART_RANGE_OFFSET_MM, mm_config_outer_offset_mm * 4))) {

		PX4_ERR("ALGO__PART_TO_PART_RANGE_OFFSET_MM fail");
		return false;
	}

	// set continuous mode
	startContinuous(50);

	return true;
}

// set distance mode to Short, Medium, or Long
// based on VL53L1_SetDistanceMode()
bool
VL53L1X::setDistanceMode(DistanceMode distance_mode)
{
	// save existing timing budget
	uint32_t budget_us = 0;

	if (!getMeasurementTimingBudget(budget_us)) {
		return false;
	}

	switch (distance_mode) {
	case Short:

		// from VL53L1_preset_mode_standard_ranging_short_range()

		if (!(// timing config
			    write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07) &&
			    write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05) &&
			    write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38) &&

			    // dynamic config
			    write_register(SD_CONFIG__WOI_SD0, 0x07) &&
			    write_register(SD_CONFIG__WOI_SD1, 0x05) &&
			    write_register(SD_CONFIG__INITIAL_PHASE_SD0, 6) && // tuning parm default
			    write_register(SD_CONFIG__INITIAL_PHASE_SD1, 6))) { // tuning parm default

			return false;
		}

		break;

	case Medium:

		// from VL53L1_preset_mode_standard_ranging()

		if (!(// timing config
			    write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B) &&
			    write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09) &&
			    write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78) &&

			    // dynamic config
			    write_register(SD_CONFIG__WOI_SD0, 0x0B) &&
			    write_register(SD_CONFIG__WOI_SD1, 0x09) &&
			    write_register(SD_CONFIG__INITIAL_PHASE_SD0, 10) && // tuning parm default
			    write_register(SD_CONFIG__INITIAL_PHASE_SD1, 10))) { // tuning parm default

			return false;
		}

		break;

	case Long:

		// from VL53L1_preset_mode_standard_ranging_long_range()

		if (!(// timing config
			    write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F) &&
			    write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D) &&
			    write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8) &&

			    // dynamic config
			    write_register(SD_CONFIG__WOI_SD0, 0x0F) &&
			    write_register(SD_CONFIG__WOI_SD1, 0x0D) &&
			    write_register(SD_CONFIG__INITIAL_PHASE_SD0, 14) && // tuning parm default
			    write_register(SD_CONFIG__INITIAL_PHASE_SD1, 14))) { // tuning parm default

			return false;
		}

		break;

	default:
		// unrecognized mode - do nothing
		return false;
	}

	// reapply timing budget
	return setMeasurementTimingBudget(budget_us);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement. A longer timing budget allows for more accurate measurements.
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool
VL53L1X::setMeasurementTimingBudget(uint32_t budget_us)
{
	// assumes PresetMode is LOWPOWER_AUTONOMOUS
	if (budget_us <= TimingGuard) {
		return false;
	}

	uint32_t range_config_timeout_us = budget_us - TimingGuard;

	if (range_config_timeout_us > 1100000) {
		return false; // FDA_MAX_TIMING_BUDGET_US * 2
	}

	range_config_timeout_us /= 2;

	// VL53L1_calc_timeout_register_values() begin

	uint8_t range_config_vcsel_period = 0;

	if (!read_register(RANGE_CONFIG__VCSEL_PERIOD_A, range_config_vcsel_period)) {
		return false;
	}

	// "Update Macro Period for Range A VCSEL Period"
	uint32_t macro_period_us = calcMacroPeriod(range_config_vcsel_period);

	// "Update Phase timeout - uses Timing A"
	// Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
	// via VL53L1_get_preset_mode_timing_cfg().
	uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);

	if (phasecal_timeout_mclks > 0xFF) {
		phasecal_timeout_mclks = 0xFF;
	}

	if (!(write_register(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks) &&

	      // "Update MM Timing A timeout"
	      // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
	      // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
	      // actually ends up with a slightly different value because it gets assigned,
	      // retrieved, recalculated with a different macro period, and reassigned,
	      // but it probably doesn't matter because it seems like the MM ("mode
	      // mitigation"?) sequence steps are disabled in low power auto mode anyway.
	      write_register16(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
				       timeoutMicrosecondsToMclks(1, macro_period_us))) &&

	      // "Update Range Timing A timeout"
	      write_register16(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
				       timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us))) &&

	      // "Update Macro Period for Range B VCSEL Period"
	      read_register(RANGE_CONFIG__VCSEL_PERIOD_B, range_config_vcsel_period)
	     )) {
		return false;
	}

	// "Update Macro Period for Range B VCSEL Period"
	macro_period_us = calcMacroPeriod(range_config_vcsel_period);

	// "Update MM Timing B timeout"
	// (See earlier comment about MM Timing A timeout.)
	return write_register16(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
					timeoutMicrosecondsToMclks(1, macro_period_us))) &&

	       // "Update Range Timing B timeout"
	       write_register16(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
					timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));
}

// Get the measurement timing budget in microseconds
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool
VL53L1X::getMeasurementTimingBudget(uint32_t &budget)
{
	// assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
	// enabled: VHV, PHASECAL, DSS1, RANGE

	// "Update Macro Period for Range A VCSEL Period"
	uint8_t range_config_vcsel_period_a = 0;

	if (!read_register(RANGE_CONFIG__VCSEL_PERIOD_A, range_config_vcsel_period_a)) {
		return false;
	}

	uint32_t macro_period_us = calcMacroPeriod(range_config_vcsel_period_a);

	uint16_t timeout_macrop_a = 0;

	if (!read_register16(RANGE_CONFIG__TIMEOUT_MACROP_A, timeout_macrop_a)) {
		return false;
	}

	// "Get Range Timing A timeout"
	uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(timeout_macrop_a), macro_period_us);

	budget = 2 * range_config_timeout_us + TimingGuard;
	return true;
}

// Start continuous ranging measurements, with the given inter-measurement
// period in milliseconds determining how often the sensor takes a measurement.
bool
VL53L1X::startContinuous(uint32_t period_ms)
{
	// fix for actual measurement period shorter than set
	uint32_t adjusted_period_ms = period_ms + (period_ms * 64 / 1000);

	// from VL53L1_set_inter_measurement_period_ms()
	return write_register32(SYSTEM__INTERMEASUREMENT_PERIOD, adjusted_period_ms * osc_calibrate_val) &&
	       write_register(SYSTEM__INTERRUPT_CLEAR, 0x01) && // sys_interrupt_clear_range
	       write_register(SYSTEM__MODE_START, 0x40); // mode_range__timed
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L1_decode_timeout()
uint32_t
VL53L1X::decodeTimeout(uint16_t reg_val)
{
	return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L1_encode_timeout()
uint16_t
VL53L1X::encodeTimeout(uint32_t timeout_mclks)
{
	// encoded format: "(LSByte * 2^MSByte) + 1"
	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (timeout_mclks > 0) {
		ls_byte = timeout_mclks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0) {
			ls_byte >>= 1;
			ms_byte++;
		}

		return (ms_byte << 8) | (ls_byte & 0xFF);

	} else {
		return 0;
	}
}

// Convert sequence step timeout from macro periods to microseconds with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_us()
uint32_t
VL53L1X::timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
	return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// Convert sequence step timeout from microseconds to macro periods with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_mclks()
uint32_t
VL53L1X::timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
	return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// Calculate macro period in microseconds (12.12 format) with given VCSEL period
// assumes fast_osc_frequency has been read and stored
// based on VL53L1_calc_macro_period_us()
uint32_t
VL53L1X::calcMacroPeriod(uint8_t vcsel_period)
{
	// from VL53L1_calc_pll_period_us()
	// fast osc frequency in 4.12 format; PLL period in 0.24 format
	uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

	// from VL53L1_decode_vcsel_period()
	uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

	// VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
	uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
	macro_period_us >>= 6;
	macro_period_us *= vcsel_period_pclks;
	macro_period_us >>= 6;

	return macro_period_us;
}

// "Setup ranges after the first one in low power auto mode by turning off
// FW calibration steps and programming static values"
// based on VL53L1_low_power_auto_setup_manual_calibration()
bool
VL53L1X::setupManualCalibration(void)
{
	uint8_t saved_vhv_init = 0;
	uint8_t saved_vhv_timeout = 0;
	uint8_t phasecal_result_vcsel_start = 0;

	// "save original vhv configs"
	if (!(read_register(VHV_CONFIG__INIT, saved_vhv_init) &&
	      read_register(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout))) {

		PX4_ERR("VHV_CONFIG__INIT failed");
		return false;
	}

	// "disable VHV init"
	if (!write_register(VHV_CONFIG__INIT, saved_vhv_init & 0x7F)) {
		PX4_ERR("VHV_CONFIG__INIT failed");
		return false;
	}

	// "set loop bound to tuning param"
	// tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)
	if (!write_register(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, (saved_vhv_timeout & 0x03) + (3 << 2))) {

		PX4_ERR("VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND failed");
		return false;
	}

	// "override phasecal"
	if (!(write_register(PHASECAL_CONFIG__OVERRIDE, 0x01) &&
	      read_register(PHASECAL_RESULT__VCSEL_START, phasecal_result_vcsel_start) &&
	      write_register(CAL_CONFIG__VCSEL_START, phasecal_result_vcsel_start))) {

		PX4_ERR("PHASECAL_CONFIG__OVERRIDE failed");
		return false;

	}

	_calibrated = true;
	PX4_INFO("setupManualCalibration success");

	return true;
}

// check if sensor has new reading available
// assumes interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1)
bool
VL53L1X::dataReady()
{
	uint8_t gpio_tio_hv_status = 0;

	return read_register(GPIO__TIO_HV_STATUS, gpio_tio_hv_status) && ((gpio_tio_hv_status & 0x01) == 0);
}

// read - return last value measured by sensor
bool
VL53L1X::get_reading(float &reading_m)
{
	if (!_calibrated) {
		_calibrated = setupManualCalibration();
	}

	uint8_t tries = 10;

	while (!dataReady()) {
		tries--;
		usleep(1000);

		if (tries == 0) {
			PX4_ERR("all retries failed");
			return false;
		}
	}

	uint16_t reading_mm = 0;
	uint8_t range_status = 0;

	if (!(read_register(RESULT__RANGE_STATUS, range_status) &&
	      read_register16(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, reading_mm))) {

		PX4_ERR("RESULT__RANGE_STATUS failed");

		return false;
	}

	// "apply correction gain"
	// gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
	// Basically, this appears to scale the result by 2011/2048, or about 98%
	// (with the 1024 added for proper rounding).
	reading_mm = ((uint32_t)reading_mm * 2011 + 0x0400) / 0x0800;

	// sys_interrupt_clear_range
	if (!write_register(SYSTEM__INTERRUPT_CLEAR, 0x01)) {
		PX4_ERR("SYSTEM__INTERRUPT_CLEAR failed");
		return false;
	}

	switch ((DeviceError)range_status) {
	case RANGECOMPLETE:
#ifdef VL53L1X_DEBUG
		PX4_INFO("RANGECOMPLETE %d", reading_mm);
#endif // VL53L1X_DEBUG
		break;

	default:
#ifdef VL53L1X_DEBUG
		PX4_INFO("status %d", (int)range_status);
#endif // VL53L1X_DEBUG
		return false;
	}

	reading_m = (float)(reading_mm) / 1000.0f;

	return true;
}

bool
VL53L1X::read_register(uint16_t reg, uint8_t &value)
{
	uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
	return (transfer(b, 2, &value, 1) == PX4_OK);
}

bool
VL53L1X::read_register16(uint16_t reg, uint16_t &value)
{
	uint16_t v = 0;
	uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };

	if (transfer(b, 2, (uint8_t *)&v, 2) != PX4_OK) {
		return false;
	}

	value = be16toh(v);
	return true;
}

bool
VL53L1X::write_register(uint16_t reg, uint8_t value)
{
	uint8_t b[3] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), value };
	return (transfer(b, 3, nullptr, 0) == PX4_OK);
}

bool
VL53L1X::write_register16(uint16_t reg, uint16_t value)
{
	uint8_t b[4] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), uint8_t(value >> 8), uint8_t(value & 0xFF) };
	return (transfer(b, 4, nullptr, 0) == PX4_OK);
}

bool
VL53L1X::write_register32(uint16_t reg, uint32_t value)
{
	uint8_t b[6] = { uint8_t(reg >> 8),
			 uint8_t(reg & 0xFF),
			 uint8_t((value >> 24) & 0xFF),
			 uint8_t((value >> 16) & 0xFF),
			 uint8_t((value >>  8) & 0xFF),
			 uint8_t((value)       & 0xFF)
		       };

	return (transfer(b, 6, nullptr, 0) == PX4_OK);
}

int
VL53L1X::init()
{
	set_device_address(VL53L1X_BASEADDR);

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	// call timer() every 50ms. We expect new data to be available every 50ms
	ScheduleOnInterval(50_ms);

	return PX4_OK;
}

int
VL53L1X::probe()
{
	if (check_id() && initSensor()) {
		return OK;
	}

	// not found on any address
	return -EIO;
}

void
VL53L1X::start()
{
	/* schedule a cycle to start things */
	ScheduleDelayed(50_ms);
}

void
VL53L1X::stop()
{
	ScheduleClear();
}

void
VL53L1X::Run()
{
	perf_begin(_sample_perf);
	float range = 0.0f;

	if (get_reading(range)) {
		distance_sensor_s report{};
		report.timestamp = hrt_absolute_time();
		report.id = get_device_id(); /* TODO: set proper ID */
		report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
		report.orientation = _rotation;
		report.current_distance = range;
		report.min_distance = VL53L1X_MIN_RANGING_DISTANCE;
		report.max_distance = VL53L1X_MAX_RANGING_DISTANCE;
		report.variance = 0.0f;
		report.signal_quality = -1;

		orb_publish_auto(ORB_ID(distance_sensor), &_distance_sensor_topic, &report, &_orb_class_instance, ORB_PRIO_DEFAULT);
	}

	perf_end(_sample_perf);
}

void
VL53L1X::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
