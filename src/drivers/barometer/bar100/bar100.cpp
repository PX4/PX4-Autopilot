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

    // Try to read memory map; if it fails, fall back to safe defaults so driver can start
    if (!_read_memory_map()) {
        PX4_WARN("BAR100 memory map read failed - using safe defaults");
        // safe defaults: 0..10 bar, vented gauge offset ~1.01325 bar
        _P_min = 0.0f;
        _P_max = 10.0f;
        _P_mode = 1.01325f;
        // don't return error — allow driver to start with defaults
    }

    _initialized = true;
    _start();
    return PX4_OK;
}

int Bar100::probe()
{
    PX4_INFO("Probing BAR100 at I2C addr 0x%02X", BAR100_I2C_ADDR);

    // Ensure the I2C address is set for the base I2C helper functions
    set_device_address(BAR100_I2C_ADDR);

    const unsigned MAX_PROBE_ATTEMPTS = 4;
    const useconds_t SHORT_DELAY_US = 5000;
    const useconds_t CONVERSION_US = 10000; // 10 ms

    uint8_t buf[BAR100_MEASUREMENT_BYTES] = {0};
    int ret = PX4_ERROR;

    for (unsigned attempt = 0; attempt < MAX_PROBE_ATTEMPTS; ++attempt) {

        // Strategy A: Trigger a measurement (0xAC), wait, then read 5 bytes
        {
            uint8_t cmd = BAR100_REQUEST_CMD;
            ret = transfer(&cmd, 1, nullptr, 0);
            if (ret == PX4_OK) {
                px4_usleep(CONVERSION_US);
                ret = transfer(nullptr, 0, buf, sizeof(buf));
                if (ret == PX4_OK) {
                    PX4_INFO("BAR100 probe successful (trigger read) attempt %u: %02x %02x %02x %02x %02x",
                             attempt + 1, buf[0], buf[1], buf[2], buf[3], buf[4]);

                    // validate minimally: not both pressure and temp zero
                    uint16_t P_raw = (static_cast<uint16_t>(buf[1]) << 8) | buf[2];
                    uint16_t T_raw = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];
                    if (P_raw != 0 || T_raw != 0) {
                        return PX4_OK;
                    } else {
                        PX4_WARN("BAR100 probe got zeroed data (P_raw=%u T_raw=%u), trying alternate probe", P_raw, T_raw);
                    }
                } else {
                    PX4_DEBUG("BAR100 read after trigger failed (ret=%d) attempt %u", ret, attempt + 1);
                }
            } else {
                PX4_DEBUG("BAR100 trigger write failed (ret=%d) attempt %u", ret, attempt + 1);
            }
        }

        px4_usleep(SHORT_DELAY_US);

        // Strategy B: Read-only probe (try to read 5 bytes without writing)
        {
            memset(buf, 0, sizeof(buf));
            ret = transfer(nullptr, 0, buf, sizeof(buf));
            if (ret == PX4_OK) {
                PX4_INFO("BAR100 probe successful (read-only) attempt %u: %02x %02x %02x %02x %02x",
                         attempt + 1, buf[0], buf[1], buf[2], buf[3], buf[4]);

                uint16_t P_raw = (static_cast<uint16_t>(buf[1]) << 8) | buf[2];
                uint16_t T_raw = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];
                if (P_raw != 0 || T_raw != 0) {
                    return PX4_OK;
                } else {
                    PX4_WARN("BAR100 read-only returned zeros, trying write-then-read");
                }
            } else {
                PX4_DEBUG("BAR100 read-only failed (ret=%d) attempt %u", ret, attempt + 1);
            }
        }

        px4_usleep(SHORT_DELAY_US);

        // Strategy C: Write 0x00 (some units respond to address 0x00) then read
        {
            uint8_t addr0 = 0x00;
            ret = transfer(&addr0, 1, nullptr, 0);
            if (ret == PX4_OK) {
                px4_usleep(SHORT_DELAY_US);
                memset(buf, 0, sizeof(buf));
                ret = transfer(nullptr, 0, buf, sizeof(buf));
                if (ret == PX4_OK) {
                    PX4_INFO("BAR100 probe successful (0x00 addr) attempt %u: %02x %02x %02x %02x %02x",
                             attempt + 1, buf[0], buf[1], buf[2], buf[3], buf[4]);
                    uint16_t P_raw = (static_cast<uint16_t>(buf[1]) << 8) | buf[2];
                    uint16_t T_raw = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];
                    if (P_raw != 0 || T_raw != 0) {
                        return PX4_OK;
                    } else {
                        PX4_WARN("BAR100 0x00-read returned zeros");
                    }
                }
            }
        }

        // small delay and retry
        px4_usleep(10000);
    }

    PX4_ERR("BAR100 did not respond to probe after %u attempts", MAX_PROBE_ATTEMPTS);
    return -EIO;
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

bool Bar100::_read_registers(uint8_t reg, uint8_t *data, unsigned len)
{
    uint8_t cmd = reg;

    // Select register
    if (transfer(&cmd, 1, nullptr, 0) != PX4_OK) {
        PX4_ERR("BAR100: write reg 0x%02X failed", reg);
        return false;
    }

    // Wait for sensor to prepare MTP data
    px4_usleep(2000);  // 2 ms minimum

    // Read status + 2 bytes (Keller always sends 3 bytes)
    uint8_t buf[3] = {};
    if (transfer(nullptr, 0, buf, sizeof(buf)) != PX4_OK) {
        PX4_ERR("BAR100: read reg 0x%02X failed", reg);
        return false;
    }

    // Skip status byte, copy MSB/LSB into data buffer
    if (len >= 2) {
        data[0] = buf[1];
        data[1] = buf[2];
    } else if (len == 1) {
        data[0] = buf[2];
    }

    return true;
}

// Helper: read a single MTP 16-bit value (write mtp address, read 3 bytes: status, hi, lo)
// Robust helper: read a single MTP 16-bit value (write mtp address, read 3 bytes: status, hi, lo)
bool Bar100::_read_mtp16(uint8_t mtp_addr, uint16_t &val)
{
    const unsigned MAX_ATTEMPTS = 3;
    const useconds_t T_SETUP_US = 1500;   // small setup delay
    const useconds_t T_INTER_US = 2000;   // inter-read gap
    uint8_t buf[3] = {0};
    int ret = PX4_ERROR;

    for (unsigned attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {

        // Attempt A: combined transaction (write addr then read 3 bytes)
        {
            uint8_t addr = mtp_addr;
            ret = transfer(&addr, 1, buf, sizeof(buf));
            if (ret == PX4_OK) {
                PX4_DEBUG("BAR100 MTP[0x%02X] combined attempt %u raw: %02X %02X %02X",
                         mtp_addr, attempt+1, buf[0], buf[1], buf[2]);
                if (!(buf[1] == 0x00 && buf[2] == 0x00)) {
                    val = (static_cast<uint16_t>(buf[1]) << 8) | static_cast<uint16_t>(buf[2]);
                    px4_usleep(T_INTER_US);
                    return true;
                }
                // if data bytes are zero, fall through to separate flow
            } else {
                PX4_DEBUG("BAR100 MTP combined transfer failed for 0x%02X attempt %u (ret=%d)",
                         mtp_addr, attempt+1, ret);
            }
        }

        // small pause then try separate write/read
        px4_usleep(T_SETUP_US);

        // Attempt B: write addr, then read 3 bytes
        {
            uint8_t cmd = mtp_addr;
            ret = transfer(&cmd, 1, nullptr, 0);
            if (ret != PX4_OK) {
                PX4_DEBUG("BAR100 MTP write-only failed for 0x%02X attempt %u (ret=%d)",
                         mtp_addr, attempt+1, ret);
                px4_usleep(T_INTER_US);
                continue;
            }

            // give the device time to prepare response
            px4_usleep(T_SETUP_US);

            memset(buf, 0, sizeof(buf));
            ret = transfer(nullptr, 0, buf, sizeof(buf));
            if (ret == PX4_OK) {
                PX4_DEBUG("BAR100 MTP[0x%02X] separate attempt %u raw: %02X %02X %02X",
                         mtp_addr, attempt+1, buf[0], buf[1], buf[2]);
                if (!(buf[1] == 0x00 && buf[2] == 0x00)) {
                    val = (static_cast<uint16_t>(buf[1]) << 8) | static_cast<uint16_t>(buf[2]);
                    px4_usleep(T_INTER_US);
                    return true;
                }
            } else {
                PX4_DEBUG("BAR100 MTP read-only failed for 0x%02X attempt %u (ret=%d)",
                         mtp_addr, attempt+1, ret);
            }
        }

        px4_usleep(5000);
    }

    PX4_ERR("BAR100: failed to read MTP[0x%02X] after %u attempts (last ret=%d) - raw: %02X %02X %02X",
           mtp_addr, MAX_ATTEMPTS, ret, buf[0], buf[1], buf[2]);

    perf_count(_comms_errors);
    return false;
}

// Try to read memory map; if it fails we fall back to defaults (but return true if we set defaults)
bool Bar100::_read_memory_map()
{
    // Trigger a measurement to ensure device awake
    uint8_t trigger = BAR100_REQUEST_CMD;
    transfer(&trigger, 1, nullptr, 0);
    px4_usleep(10000);

    uint16_t cust0 = 0, cust1 = 0;

    if (!_read_mtp16(BAR100_REG_CUST_ID0, cust0) ||
        !_read_mtp16(BAR100_REG_CUST_ID1, cust1)) {
        PX4_WARN("BAR100: CUST_ID MTP reads failed");
        // fall back to defaults so that driver can start
        _equipment = 0;
        _code = 0;
        _P_min = 0.0f;
        _P_max = 10.0f;
        _P_mode = 1.01325f;
        return true;
    }

    _equipment = cust0 >> 10;
    _code = (static_cast<uint32_t>(cust1) << 16) | cust0;

    // Read scaling registers 0x12..0x16 (each returns a 16-bit word)
    uint16_t scaling0 = 0, scaling1 = 0, scaling2 = 0, scaling3 = 0, scaling4 = 0;
    if (!_read_mtp16(BAR100_REG_SCALING0, scaling0) ||
        !_read_mtp16(BAR100_REG_SCALING1, scaling1) ||
        !_read_mtp16(BAR100_REG_SCALING2, scaling2) ||
        !_read_mtp16(BAR100_REG_SCALING3, scaling3) ||
        !_read_mtp16(BAR100_REG_SCALING4, scaling4)) {
        PX4_WARN("BAR100: scaling MTP reads failed - using defaults");
        _P_min = 0.0f;
        _P_max = 10.0f;
        _P_mode = 1.01325f;
        return true;
    }

    PX4_INFO("BAR100 raw scaling words: %04x %04x %04x %04x %04x",
             scaling0, scaling1, scaling2, scaling3, scaling4);

    // decode bits as in KellerLD
    _mode  = scaling0 & 0b00000011;
    uint16_t year  = (scaling0 >> 11) + 2000;
    uint8_t month  = (scaling0 & 0b0000011110000000) >> 7;
    uint8_t day    = (scaling0 & 0b0000000001111100) >> 2;

    if (_mode == 0) {
        _P_mode = 1.01325f;
    } else if (_mode == 1) {
        _P_mode = 1.0f;
    } else {
        _P_mode = 0.0f;
    }

    // combine scaling words into 32-bit floats (Keller/Arduino format)
    uint32_t scaling12 = (static_cast<uint32_t>(scaling1) << 16) | scaling2;
    uint32_t scaling34 = (static_cast<uint32_t>(scaling3) << 16) | scaling4;
    memcpy(&_P_min, &scaling12, sizeof(float));
    memcpy(&_P_max, &scaling34, sizeof(float));

    PX4_INFO("BAR100 MTP: code=0x%08" PRIx32 " equip=%u mode=%u date=%04u-%02u-%02u",
             _code, _equipment, _mode, year, month, day);
    PX4_INFO("BAR100 calib: Pmin=%.6f bar Pmax=%.6f bar Pmode=%.6f",
             (double)_P_min, (double)_P_max, (double)_P_mode);

    // Validate: if calibration looks invalid, fall back to safe defaults
    const float EPS = 1e-6f;
    if (!PX4_ISFINITE(_P_min) || !PX4_ISFINITE(_P_max) || _P_min >= _P_max || fabsf(_P_min) < EPS || fabsf(_P_max) < EPS) {
        PX4_WARN("BAR100: bad calibration (Pmin=%.6f Pmax=%.6f) - using defaults", (double)_P_min, (double)_P_max);
        _P_min = 0.0f;
        _P_max = 10.0f;
        _P_mode = 1.01325f;
    }

    return true;
}

int Bar100::_read_sensor()
{
    uint8_t cmd = BAR100_REQUEST_CMD;
    uint8_t buf[BAR100_MEASUREMENT_BYTES] = {0};

    // Trigger measurement
    if (transfer(&cmd, 1, nullptr, 0) != PX4_OK) {
        perf_count(_comms_errors);
        PX4_ERR("BAR100: failed to send measure cmd");
        return PX4_ERROR;
    }

    // Wait conversion time; Keller LD uses ~9ms, use 15ms for margin
    px4_usleep(15000);

    if (transfer(nullptr, 0, buf, sizeof(buf)) != PX4_OK) {
        perf_count(_comms_errors);
        PX4_ERR("BAR100: failed to read measurement");
        return PX4_ERROR;
    }

    PX4_DEBUG("BAR100 raw measurement: %02x %02x %02x %02x %02x",
              buf[0], buf[1], buf[2], buf[3], buf[4]);

    uint16_t P_raw = (static_cast<uint16_t>(buf[1]) << 8) | static_cast<uint16_t>(buf[2]);
    uint16_t T_raw = (static_cast<uint16_t>(buf[3]) << 8) | static_cast<uint16_t>(buf[4]);

    if (P_raw == 0 || T_raw == 0) {
        PX4_WARN("BAR100 invalid measurement: P_raw=%u T_raw=%u", P_raw, T_raw);
        return PX4_ERROR;
    }

    // KellerLD formulas
    _P_bar = (static_cast<float>(P_raw) - 16384.0f) * (_P_max - _P_min) / 32768.0f + _P_min + _P_mode;
    _last_temperature = ((T_raw >> 4) - 24) * 0.05f - 50.0f;
    _last_pressure = _P_bar * 100000.0f; // bar -> Pa

    PX4_DEBUG("BAR100 conv: P_raw=%u P_bar=%.6f bar P_Pa=%.2f Pa T_raw=%u T_C=%.2f",
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
