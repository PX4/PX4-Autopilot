/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *   Modified for Keller LD sensor by Blue Robotics Inc.
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

#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_baro.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/err.h>

#include "bar100_registers.h"

/* helper macro for handling report buffer indices */
// #define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
// #define POW2(_x)		((_x) * (_x))

class Bar100 : public device::I2C, public I2CSPIDriver<Bar100>
{
public:
    Bar100(const I2CSPIDriverConfig &config);
    ~Bar100() override;

    static void print_usage();

    int init();

    /**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
    void RunImpl();
    void print_status() override;
    int read(unsigned offset, void *data, unsigned count) override;

private:
    int probe() override;

    uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};

    bool _collect_phase{false};
    unsigned _measure_phase{false};

    float _last_temperature{NAN};
    float _last_pressure{NAN};

    perf_counter_t _sample_perf;
    perf_counter_t _measure_perf;
    perf_counter_t _comms_errors;

    // Bar100 sensor state
    bool _initialized{false};
    float _fluid_density{1029.0f}; // Default seawater

    // Sensor info
    uint16_t _equipment{0};
    uint16_t _place{0};
    uint16_t _file{0};
    uint8_t _mode{0};
    uint16_t _year{0};
    uint8_t _month{0};
    uint8_t _day{0};
    uint32_t _code{0};
    uint16_t _P{0};
    float _P_bar{0.0f};
    float _P_mode{0.0f};
    float _P_min{0.0f};
    float _P_max{0.0f};

    // Sensor read helpers
    int _reset();
    int _read_memory_map();
    int _read_sensor();

    void _start();

    // Conversion helpers
    float _pressure();
    float _temperature();
    float _depth();
    float _altitude();

    bool _status();
    bool _is_initialized();
};
