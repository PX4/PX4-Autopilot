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

#include "tfmini_s_down.hpp"

#include <lib/parameters/param.h>


static uint8_t crc8(uint8_t *p, uint8_t len)
{
        uint16_t i;
        uint16_t crc = 0x0;

        for(i=0;i<len;i++)
            crc += p[i];

        return crc & 0xFF;
}

TFmini_s_down::TFmini_s_down(const int bus, const int address, const uint8_t rotation) :
        I2C("tfmini_s", nullptr, bus, address, 100000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_rangefinder(0 /* device id not yet used */, ORB_PRIO_DEFAULT, rotation)
{
	// up the retries since the device misses the first measure attempts
	I2C::_retries = 3;
}

TFmini_s_down::~TFmini_s_down()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int TFmini_s_down::collect()
{
    if (!_collect_phase) {
            return measure();
    }

    perf_begin(_sample_perf);

    // Transfer data from the bus.
    uint8_t val[9] {};
    const hrt_abstime timestamp_sample = hrt_absolute_time();
    set_device_address(TFMINI_S_D_ADDR);
    int ret_val = transfer(nullptr, 0, &val[0], 9);

    if (ret_val < 0) {
            PX4_ERR("error reading from sensor: 0x%02x", TFMINI_S_D_ADDR);
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
//  uint16_t temperature = ((val[7] << 8) | val[6]) / 8 - 256;

    PX4_ERR("distance %d cm", distance_cm);
    // Final data quality evaluation. This is based on the datasheet and simple heuristics retrieved from experiments
    // Step 1: Normalize signal strength to 0...100 percent using the absolute signal peak strength.
    uint8_t signal_quality = 100 * strength / 65535.0f;

    // Step 2: Filter physically impossible measurements, which removes some crazy outliers that appear on LL40LS.
    if (distance_m < TFMINI_S_MIN_DISTANCE)
             signal_quality = 0;
    if((strength == -1) || (strength == -2))
             signal_quality = strength;

    if (crc8(val, 8) == val[8]) {
             _px4_rangefinder.update(timestamp_sample, distance_m,signal_quality);
             PX4_ERR("Tested data as %f m",double(distance_m));
    }

    // Next phase is measurement.
    _collect_phase = false;

    perf_count(_sample_perf);
    perf_end(_sample_perf);

    return PX4_OK;
}

int TFmini_s_down::init()
{
	int32_t hw_model = 0;
        param_get(param_find("SENS_EN_TFMINI_D"), &hw_model);

        switch (hw_model) {
	case 0: // Disabled
		PX4_WARN("Disabled");
		return PX4_ERROR;

        case 1: // Autodetect - assume default TFmini-s
                set_device_address(TFMINI_S_D_ADDR);

                if (I2C::init() != OK) {
                    PX4_ERR("initialisation failed at i2c address 0x%02x",TFMINI_S_D_ADDR);
                    return PX4_ERROR;

                } else {
                    // Assume minimum and maximum possible distances acros Evo family
                    _px4_rangefinder.set_min_distance(TFMINI_S_MIN_DISTANCE);
                    _px4_rangefinder.set_max_distance(TFMINI_S_MAX_DISTANCE);
                }

                break;

	default:
		PX4_ERR("invalid HW model %d.", hw_model);
		return PX4_ERROR;
	}

	start();

	return PX4_OK;
}

int TFmini_s_down::measure()
{
    // Send the command to begin a measurement.
    const uint8_t cmd[] = {0x5A,0x05,0x00,0x01,0x60}; // cm output
    // const uint8_t cmd[] = {0x5A,0x05,0x00,0x06,0x65}; // mm output
    int ret_val = transfer(&cmd[0], 5, nullptr, 0);


	if (ret_val != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret_val);
		return ret_val;
	}

	_collect_phase = true;
	return PX4_OK;
}

int TFmini_s_down::probe()
{
        return measure();
}

void TFmini_s_down::Run()
{
	// Perform data collection.
	collect();
}

void TFmini_s_down::start()
{
	_collect_phase = false;

	// Schedule the driver to run on a set interval
        ScheduleOnInterval(TFMINI_S_MEASUREMENT_INTERVAL);
}

void TFmini_s_down::stop()
{
	ScheduleClear();
}

void TFmini_s_down::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	_px4_rangefinder.print_status();
}
