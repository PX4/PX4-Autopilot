/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "TFMINI_I2C.hpp"

#include <lib/parameters/param.h>

static uint8_t crc8(uint8_t *p, uint8_t len)
{
	uint16_t i;
	uint16_t crc = 0x0;

        for(i=0;i<len;i++)
            crc += p[i];

	return crc & 0xFF;
}

TFMINI_I2C::TFMINI_I2C(const int bus, const int address, const uint8_t rotation) :
        I2C("TFMINI_I2C", nullptr, bus, address, 100000),
        ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
        _px4_rangefinder(0 /* device id not yet used */, ORB_PRIO_DEFAULT, rotation)
{
	// up the retries since the device misses the first measure attempts
	I2C::_retries = 3;
        orientation = rotation;
        Address = address;
}

TFMINI_I2C::~TFMINI_I2C()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TFMINI_I2C::collect()
{
	if (!_collect_phase) {
		return measure();
	}

	perf_begin(_sample_perf);

	// Transfer data from the bus.
        uint8_t val[9] {};
	const hrt_abstime timestamp_sample = hrt_absolute_time();
        int ret_val = transfer(nullptr, 0, &val[0], 9);

	if (ret_val < 0) {
		PX4_ERR("error reading from sensor: %d", ret_val);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret_val;
	}
        while (val[0] != 0x59 || val[1] != 0x59){
                PX4_ERR("error reading from sensor: 0x%02X 0x%02X", val[0], val[1]);
                perf_count(_comms_errors);
                perf_end(_sample_perf);
                return measure();
        }
        uint16_t distance_cm = (val[3] << 8) | val[2];
        float distance_m = static_cast<float>(distance_cm) * 1e-2f;
        int strength = (val[5] << 8) | val[4];
//        uint16_t temperature = ((val[7] << 8) | val[6]) / 8 - 256;

        // Final data quality evaluation. This is based on the datasheet and simple heuristics retrieved from experiments
        // Step 1: Normalize signal strength to 0...100 percent using the absolute signal peak strength.
        uint8_t signal_quality = 100 * strength / 65535.0f;

        // Step 2: Filter physically impossible measurements, which removes some crazy outliers that appear on LL40LS.
        if (distance_m < TFMINI_S_I2C_MIN_DISTANCE)
            signal_quality = 0;
        if((strength == -1) || (strength == -2))
            signal_quality = strength;

        if (crc8(val, 8) == val[8]) {
                _px4_rangefinder.update(timestamp_sample, distance_m,signal_quality);
                PX4_INFO("Tested data as %f m",double(distance_m));
	}

	// Next phase is measurement.
	_collect_phase = false;

	perf_count(_sample_perf);
	perf_end(_sample_perf);

	return PX4_OK;
}

int TFMINI_I2C::init()
{
	int32_t hw_model = 0;
        param_get(param_find("SENS_EN_TFMINI2C"), &hw_model);

	switch (hw_model) {
	case 0: // Disabled
		PX4_WARN("Disabled");
		return PX4_ERROR;

        case 1: // Autodetect - assume default TFmini-S (12m, 100 Hz)
                set_device_address(Address);

                if (I2C::init() != OK) {
                    PX4_ERR("ERROR the TFmini-S at 0x%02x",Address);
                    return PX4_ERROR;
                } else {
                    PX4_INFO("Initialized the TFmini-S at 0x%02x",Address);
                        _px4_rangefinder.set_min_distance(TFMINI_S_I2C_MIN_DISTANCE);
                        _px4_rangefinder.set_max_distance(TFMINI_S_I2C_MAX_DISTANCE);
                        _px4_rangefinder.set_orientation(orientation);
                        _px4_rangefinder.set_fov(math::degrees(2.0f));
                }
                break;

        case 2: // TFmini (12m, 100 Hz)
                set_device_address(Address);

		if (I2C::init() != OK) {
			return PX4_ERROR;
		}

                _px4_rangefinder.set_min_distance(TFMINI_I2C_MIN_DISTANCE);
                _px4_rangefinder.set_max_distance(TFMINI_I2C_MAX_DISTANCE);
                _px4_rangefinder.set_orientation(orientation);
                _px4_rangefinder.set_fov(math::degrees(2.3f));
		break;

        case 3: // TFmini_Plus.
                set_device_address(Address);

		if (I2C::init() != OK) {
			return PX4_ERROR;
		}

                _px4_rangefinder.set_min_distance(TFMINI_PLUS_I2C_MIN_DISTANCE);
                _px4_rangefinder.set_max_distance(TFMINI_PLUS_I2C_MAX_DISTANCE);
                _px4_rangefinder.set_orientation(orientation);
		break;

	default:
		PX4_ERR("invalid HW model %d.", hw_model);
		return PX4_ERROR;
	}
for(int k=0;k<5;k++){
        measure();
        uint8_t val[9] {};
        PX4_INFO("TFMINI_I2C::collect: i2c::transfer returned %d", transfer(nullptr, 0, &val[0], 9));
        PX4_INFO("Tested data as 0x%02x 0x%02x %f m",val[0],val[1],double((val[3] << 8) | val[2]));
}
	start();

	return PX4_OK;
}

int TFMINI_I2C::measure()
{
	// Send the command to begin a measurement.
        const uint8_t cmd[5] = {MEASURE_REG1,MEASURE_REG2,MEASURE_REG3,MEASURE_REG4,MEASURE_REG5};
        int ret_val = transfer(&cmd[0], sizeof(cmd), nullptr, 0);

	if (ret_val != PX4_OK) {
		perf_count(_comms_errors);
                PX4_INFO("TFMINI_I2C::measure: i2c::transfer returned %d", ret_val);
		return ret_val;
	}

	_collect_phase = true;
	return PX4_OK;
}

int TFMINI_I2C::probe()
{
    const uint8_t cmd[5] = {MEASURE_REG1,MEASURE_REG2,MEASURE_REG3,MEASURE_REG4,MEASURE_REG5};
    int ret_val = transfer(&cmd[0], sizeof(cmd), nullptr, 0);
    if (ret_val != PX4_OK) {
            perf_count(_comms_errors);
            PX4_INFO("TFMINI_I2C::probe: i2c::transfer returned %d", ret_val);
            return ret_val;
    }
    return PX4_OK;
}

void TFMINI_I2C::Run()
{
    // Collect the sensor data.
    if (collect() != PX4_OK) {
            PX4_INFO("collection error");
            // If an error occurred, restart the measurement state machine.
            start();
            return;
    }
}

void TFMINI_I2C::start()
{
	_collect_phase = false;

	// Schedule the driver to run on a set interval
        ScheduleOnInterval(TFMINI_MEASUREMENT_INTERVAL); // 100ms recommanded
}

void TFMINI_I2C::stop()
{
	ScheduleClear();
}

void TFMINI_I2C::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	_px4_rangefinder.print_status();
}
