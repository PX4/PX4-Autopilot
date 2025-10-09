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
#include <inttypes.h>

#define BAR100_CONVERSION_INTERVAL (20_ms) // Keep this for PX4's time literal

Bar100::Bar100(const I2CSPIDriverConfig &config) :
    I2C(config),
    I2CSPIDriver(config),
    _sample_perf(perf_alloc(PC_ELAPSED, "bar100_read")),
    _measure_perf(perf_alloc(PC_ELAPSED, "bar100_measure")),
    _comms_errors(perf_alloc(PC_COUNT, "bar100_com_err"))
{
    PX4_ERR("Bar100 constructor called");
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
        PX4_ERR("I2C init failed (%d)", ret);
        return ret;
    }

    if (_reset() != PX4_OK) {
        PX4_ERR("BAR100 memory map read failed");
        return PX4_ERROR;
    }

    _initialized = true;
    _start();
    return PX4_OK;
}

int Bar100::probe()
{
    PX4_INFO("Probing BAR100 at I2C addr 0x%02X", BAR100_I2C_ADDR);

    uint8_t cmd = BAR100_REQUEST_CMD ; // usually 0xAC
    if (transfer(&cmd, 1, nullptr, 0) != PX4_OK) {
        PX4_ERR("BAR100 did not ACK request command");
        return -EIO;
    }

    px4_usleep(10000); // wait 10ms
    uint8_t buf[5] {};
    if (transfer(nullptr, 0, buf, 5) != PX4_OK) {
        PX4_ERR("BAR100 read failed");
        return -EIO;
    }

    PX4_INFO("BAR100 probe successful: %02x %02x %02x %02x %02x",
             buf[0], buf[1], buf[2], buf[3], buf[4]);
    return PX4_OK;
}


int Bar100::read(unsigned offset, void *data, unsigned count)
{
    // Not used in PX4 barometer drivers, but required by base class
    return PX4_OK;
}

void Bar100::RunImpl()
{
    perf_begin(_sample_perf);

    if (_read_sensor() == PX4_OK) {
        sensor_baro_s report{};
        report.timestamp = hrt_absolute_time();
        report.device_id = get_device_id();
        report.pressure = _last_pressure;
        report.temperature = _last_temperature;

        _sensor_baro_pub.publish(report);
    }

    perf_end(_sample_perf);
    ScheduleDelayed(BAR100_CONVERSION_INTERVAL_US);
}

void Bar100::_start()
{
    ScheduleDelayed(BAR100_CONVERSION_INTERVAL_US);
}

int Bar100::_reset()
{
    // No reset command for KellerLD, just re-init memory map
    return _read_memory_map();
}

bool Bar100::_read_registers(uint8_t start_reg, uint8_t *data, unsigned len)
{
    // Send the register address, then read `len` bytes from it
    if (transfer(&start_reg, 1, data, len) != PX4_OK) {
        PX4_ERR("BAR100: failed to read registers starting at 0x%02X", start_reg);
        perf_count(_comms_errors);
        return false;
    }

    return true;
}

// Helper: read a single MTP 16-bit value (write mtp address, read 3 bytes: status, hi, lo)
bool Bar100::_read_mtp16(uint8_t mtp_addr, uint16_t &val)
{
    uint8_t cmd = mtp_addr;
    uint8_t buf[3] = {0};

    // write MTP address, then read 3 bytes (status + msb + lsb)
    if (transfer(&cmd, 1, buf, sizeof(buf)) != PX4_OK) {
        PX4_ERR("BAR100: failed mtp read 0x%02X", mtp_addr);
        perf_count(_comms_errors);
        return false;
    }

    // buf[0] = status (ignore), buf[1]=msb, buf[2]=lsb
    val = (static_cast<uint16_t>(buf[1]) << 8) | static_cast<uint16_t>(buf[2]);
    return true;
}

bool Bar100::_read_memory_map()
{
    // Read CUST_ID registers (each as 16-bit via mtp)
    uint16_t cust_id0 = 0;
    uint16_t cust_id1 = 0;

    if (!_read_mtp16(BAR100_REG_CUST_ID0, cust_id0) ||
        !_read_mtp16(BAR100_REG_CUST_ID1, cust_id1)) {
        PX4_ERR("BAR100: failed to read CUST_IDs");
        return false;
    }

    _equipment = cust_id0 >> 10;
    _code = (static_cast<uint32_t>(cust_id1) << 16) | cust_id0;

    // Read scaling registers individually (each mtp returns 16-bit)
    uint16_t scaling0 = 0, scaling1 = 0, scaling2 = 0, scaling3 = 0, scaling4 = 0;

    if (!_read_mtp16(BAR100_REG_SCALING0, scaling0) ||
        !_read_mtp16(BAR100_REG_SCALING1, scaling1) ||
        !_read_mtp16(BAR100_REG_SCALING2, scaling2) ||
        !_read_mtp16(BAR100_REG_SCALING3, scaling3) ||
        !_read_mtp16(BAR100_REG_SCALING4, scaling4)) {
        PX4_ERR("BAR100: failed to read SCALING registers");
        return false;
    }

    // Decode mode & date from scaling0 (same bit fields as Arduino)
    _mode  = scaling0 & 0b00000011;
    uint16_t year  = (scaling0 >> 11) + 2000; // assume offset from 2000
    uint8_t month  = (scaling0 & 0b0000011110000000) >> 7;
    uint8_t day    = (scaling0 & 0b0000000001111100) >> 2;

    // P_mode same as Arduino
    if (_mode == 0) {
        _P_mode = 1.01325f; // PA mode (vented)
    } else if (_mode == 1) {
        _P_mode = 1.0f; // PR mode (sealed)
    } else {
        _P_mode = 0.0f; // absolute or unspecified
    }

    // Combine scaling1+2 and scaling3+4 into 32-bit words (Arduino does this)
    uint32_t scaling12 = (static_cast<uint32_t>(scaling1) << 16) | scaling2;
    uint32_t scaling34 = (static_cast<uint32_t>(scaling3) << 16) | scaling4;

    // Reinterpret as IEEE-754 floats — exactly like Arduino code
    memcpy(&_P_min, &scaling12, sizeof(float));
    memcpy(&_P_max, &scaling34, sizeof(float));

    PX4_INFO("BAR100 MTP: code=0x%08" PRIx32 " equip=%u mode=%u date=%04u-%02u-%02u",
             _code, _equipment, _mode, year, month, day);
    PX4_INFO("BAR100 calib: Pmin=%.6f bar Pmax=%.6f bar Pmode=%.6f",
             (double)_P_min, (double)_P_max, (double)_P_mode);

    // Basic sanity check
    if (!PX4_ISFINITE(_P_min) || !PX4_ISFINITE(_P_max) || _P_min >= _P_max) {
        PX4_ERR("BAR100: bad calibration data (Pmin=%.6f Pmax=%.6f)", (double)_P_min, (double)_P_max);
        return false;
    }

    return true;
}

int Bar100::_read_sensor()
{
    uint8_t cmd = BAR100_REQUEST_CMD;
    uint8_t buf[BAR100_MEASUREMENT_BYTES] = {0};

    // send measurement request
    if (transfer(&cmd, 1, nullptr, 0) != PX4_OK) {
        perf_count(_comms_errors);
        PX4_ERR("BAR100: failed to send measure cmd");
        return PX4_ERROR;
    }

    // wait per datasheet (Arduino uses 9 ms)
    px4_usleep(9000);

    if (transfer(nullptr, 0, buf, sizeof(buf)) != PX4_OK) {
        perf_count(_comms_errors);
        PX4_ERR("BAR100: failed to read measurement");
        return PX4_ERROR;
    }

    // Debug raw bytes (enable PX4_DEBUG to see)
    PX4_DEBUG("BAR100 raw: %02x %02x %02x %02x %02x", buf[0], buf[1], buf[2], buf[3], buf[4]);

    // Extract raw values exactly like Arduino
    uint16_t P_raw = (static_cast<uint16_t>(buf[1]) << 8) | static_cast<uint16_t>(buf[2]);
    uint16_t T_raw = (static_cast<uint16_t>(buf[3]) << 8) | static_cast<uint16_t>(buf[4]);

    // Apply Arduino (KellerLD) formula for pressure (bar)
    // P_bar = (P_raw - 16384) * (P_max - P_min) / 32768 + P_min + P_mode
    _P_bar = (static_cast<float>(P_raw) - 16384.0f) * (_P_max - _P_min) / 32768.0f + _P_min + _P_mode;

    // Apply Arduino formula for temperature (°C)
    // T_degc = ((T_raw >> 4) - 24) * 0.05 - 50
    _last_temperature = ((T_raw >> 4) - 24) * 0.05f - 50.0f;

    // Convert bar -> Pascal for PX4 (1 bar = 100000 Pa)
    _last_pressure = _P_bar * 100000.0f;

    PX4_DEBUG("BAR100 conv: P_raw=%u P_bar=%.6fbar P_Pa=%.2f Pa T_raw=%u T_C=%.2f",
              P_raw, (double)_P_bar, (double)_last_pressure, T_raw, (double)_last_temperature);

    return PX4_OK;
}

float Bar100::_pressure()
{
    return _last_pressure; // Already in Pascals
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
    return (1.0f - powf((_pressure() / 1013.25f), 0.190284f)) * 145366.45f * 0.3048f;
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
