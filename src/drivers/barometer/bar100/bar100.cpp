/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file BAR100.cpp
 * Driver for the BAR100 barometric pressure sensor connected via I2C.
 */

#include "bar100.hpp"
#include "bar100_registers.h"

#define BAR100_CONVERSION_INTERVAL (20_ms) // Keep this for PX4's time literal

Bar100::Bar100(const I2CSPIDriverConfig &config) :
    I2C(config),
    I2CSPIDriver(config),
    _sample_perf(perf_alloc(PC_ELAPSED, "bar100_read")),
    _measure_perf(perf_alloc(PC_ELAPSED, "bar100_measure")),
    _comms_errors(perf_alloc(PC_COUNT, "bar100_com_err"))
{
}

Bar100::~Bar100()
{
    perf_free(_sample_perf);
    perf_free(_measure_perf);
    perf_free(_comms_errors);
}

int Bar100::init()
{
    int ret = I2C::init();
    if (ret != PX4_OK) {
	DEVICE_DEBUG("I2C::init failed (%i)", ret);
        return ret;
    }

    // Read memory map to initialize sensor info
    if (_read_memory_map() != PX4_OK) {
        return -EIO;
    }

    _initialized = true;
    _start();
    return PX4_OK;
}

int Bar100::probe()
{
    set_device_address(BAR100_I2C_ADDR);
    // Try to read memory map as probe
    return (_read_memory_map() == PX4_OK) ? PX4_OK : -EIO;
}

int Bar100::read(unsigned offset, void *data, unsigned count)
{
    // Not used in PX4 barometer drivers, but required by base class
    return PX4_OK;
}

void Bar100::RunImpl()
{
    perf_begin(_sample_perf);

    if (!_initialized) {
        perf_end(_sample_perf);
        return;
    }

    if (_read_sensor() == PX4_OK) {
        sensor_baro_s sensor_baro{};
        sensor_baro.timestamp_sample = hrt_absolute_time();
        sensor_baro.device_id = get_device_id();
        sensor_baro.pressure = _pressure();      // Pa
        sensor_baro.temperature = _temperature(); // deg C
        sensor_baro.error_count = perf_event_count(_comms_errors);
        sensor_baro.timestamp = hrt_absolute_time();
        _sensor_baro_pub.publish(sensor_baro);
    }

    perf_end(_sample_perf);

    // Schedule next measurement
    ScheduleDelayed(BAR100_CONVERSION_INTERVAL); // 20ms interval, adjust as needed
}

void Bar100::_start()
{
    ScheduleDelayed(BAR100_CONVERSION_INTERVAL);
}

int Bar100::_reset()
{
    // No reset command for KellerLD, just re-init memory map
    return _read_memory_map();
}

int Bar100::_read_memory_map()
{
    // Use register constants from header
    uint8_t regs[BAR100_NUM_REGISTERS] = {
        BAR100_REG_CUST_ID0,
        BAR100_REG_CUST_ID1,
        BAR100_REG_SCALING0,
        BAR100_REG_SCALING1,
        BAR100_REG_SCALING2,
        BAR100_REG_SCALING3,
        BAR100_REG_SCALING4
    };
    uint16_t values[BAR100_NUM_REGISTERS] = {};

    for (int i = 0; i < BAR100_NUM_REGISTERS; ++i) {
        uint8_t cmd = regs[i];
        uint8_t buf[3] = {};
        if (transfer(&cmd, 1, buf, 3) != PX4_OK) {
            return -EIO;
        }
        values[i] = (buf[1] << 8) | buf[2];
    }

    // Assign values as in KellerLD.cpp
    uint16_t cust_id0 = values[0];
    uint16_t cust_id1 = values[1];
    _code = (uint32_t(cust_id1) << 16) | cust_id0;
    _equipment = cust_id0 >> 10;
    _place = cust_id0 & 0b000000111111111;
    _file = cust_id1;

    uint16_t scaling0 = values[2];
    _mode = scaling0 & 0b00000011;
    _year = scaling0 >> 11;
    _month = (scaling0 & 0b0000011110000000) >> 7;
    _day = (scaling0 & 0b0000000001111100) >> 2;

    if (_mode == 0) {
        _P_mode = 1.01325f;
    } else if (_mode == 1) {
        _P_mode = 1.0f;
    } else {
        _P_mode = 0.0f;
    }

    uint32_t scaling12 = (uint32_t(values[3]) << 16) | values[4];
    _P_min = *reinterpret_cast<float*>(&scaling12);

    uint32_t scaling34 = (uint32_t(values[5]) << 16) | values[6];
    _P_max = *reinterpret_cast<float*>(&scaling34);

    return PX4_OK;
}

int Bar100::_read_sensor()
{
    // Request measurement
    uint8_t cmd = BAR100_REQUEST;
    if (transfer(&cmd, 1, nullptr, 0) != PX4_OK) {
        perf_count(_comms_errors);
        return -EIO;
    }

    px4_usleep(9000); // 9ms conversion time

    uint8_t buf[5] = {};
    if (transfer(nullptr, 0, buf, 5) != PX4_OK) {
        perf_count(_comms_errors);
        return -EIO;
    }

    uint8_t status = buf[0];
    _P = (buf[1] << 8) | buf[2];
    uint16_t T = (buf[3] << 8) | buf[4];

    _P_bar = (float(_P) - 16384.0f) * (_P_max - _P_min) / 32768.0f + _P_min + _P_mode;
    _last_temperature = ((T >> 4) - 24) * 0.05f - 50.0f;

    return PX4_OK;
}

float Bar100::_pressure()
{
    return _P_bar * 1000.0f; // Pa
}

float Bar100::_temperature()
{
    return _last_temperature;
}

float Bar100::_depth()
{
    return (_pressure() - 101325.0f) / (_fluid_density * 9.80665f);
}

float Bar100::_altitude()
{
    return (1.0f - pow((_pressure() / 1013.25f), 0.190284f)) * 145366.45f * 0.3048f;
}

bool Bar100::_status()
{
    return _equipment <= 62;
}

bool Bar100::_is_initialized()
{
    return _initialized;
}

void Bar100::print_status()
{
    I2CSPIDriverBase::print_status();
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
}
