/****************************************************************************

 * Copyright (c) 2026, PX4 Development Team.
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ****************************************************************************/
#include "af9838.hpp"

using namespace AF9838;
using namespace time_literals;

AF9838_Driver::AF9838_Driver(const I2CSPIDriverConfig &config)
    : device::I2C(config),
      I2CSPIDriver(config),
      _px4_mag(get_device_id(), config.rotation)
{
    _px4_mag.set_device_type(DRV_MAG_DEVTYPE_AF9838);
    _px4_mag.set_device_id(get_device_id());
}

I2CSPIDriverBase *AF9838_Driver::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
    PX4_INFO("AF9838: instantiate bus=%d addr=0x%02X freq=%u",
         config.bus, config.i2c_address, config.bus_frequency);

    AF9838_Driver *dev = new AF9838_Driver(config);

    if (!dev) { return nullptr; }

    const int ret = dev->init();
    PX4_INFO("AF9838: I2C::init ret=%d", ret);

    if (ret != PX4_OK) {
        delete dev;
        return nullptr;
    }

    return dev;
}


int AF9838_Driver::probe()
{
    constexpr int MAX_TRIES = 3;
    uint8_t id = 0xff;

    PX4_INFO("AF9838: probe() on bus=%d addr=0x%02X", get_device_bus(), get_device_address());

    for (int i = 0; i < MAX_TRIES; i++) {
        if (transfer(&REG_PCODE, 1, &id, 1) == PX4_OK) {
            PX4_INFO("AF9838: REG_PCODE=0x%02X (try %d)", id, i + 1);
            return PX4_OK;
        }

        px4_usleep(1_ms);
    }

    PX4_ERR("AF9838: probe() failed (no response) bus=%d addr=0x%02X",
        get_device_bus(), get_device_address());
    return PX4_ERROR;
}


int AF9838_Driver::init()
{
    PX4_INFO("AF9838: init() enter");

    int ret = device::I2C::init();
    PX4_INFO("AF9838: device::I2C::init ret=%d", ret);

    if (ret != PX4_OK) { return ret; }

    PX4_INFO("AF9838: soft_reset...");
    (void)soft_reset();

    PX4_INFO("AF9838: configure BISC...");

    if (configure() != PX4_OK) {
        PX4_ERR("AF9838: configure failed");
        return PX4_ERROR;
    }

    float hz = AF9838::DEFAULT_RATE_HZ;

    if (hz < 1.f) { hz = 1.f; }

    if (hz > 1000.f) { hz = 1000.f; }

    _measure_interval_us = (uint32_t)(1000000.0f / hz);

    _running = true;
    ScheduleOnInterval(_measure_interval_us);
    PX4_INFO("AF9838: scheduled every %u us (%.1f Hz)",
         (unsigned)_measure_interval_us,
         1e6 / static_cast<double>(_measure_interval_us));

    const hrt_abstime now = hrt_absolute_time();
    _px4_mag.update(now, 0.f, 0.f, 0.f);
    PX4_INFO("AF9838: init() done");
    return PX4_OK;
}


int AF9838_Driver::soft_reset()
{
    if (write_reg(REG_SWR, 0x81) != PX4_OK) {
        PX4_ERR("soft reset write failed");
        return PX4_ERROR;
    }

    px4_usleep(5_ms);

    return PX4_OK;
}

int AF9838_Driver::configure()
{
    PX4_INFO("AF9838: Triggering BISC in configure...");

    if (write_reg(REG_STATE, STATE_SELF_TEST) != PX4_OK) {
        return PX4_ERROR;
    }

    px4_usleep(40_ms);

    if (write_reg(REG_STATE, STATE_SINGLE) != PX4_OK) {
        return PX4_ERROR;
    }

    px4_usleep(6_ms);

    return PX4_OK;
}

uint8_t AF9838_Driver::read_reg(uint8_t reg)
{
    uint8_t v{0};
    transfer(&reg, 1, &v, 1);
    return v;
}

int AF9838_Driver::read_block(uint8_t reg, uint8_t *buf, size_t len)
{
    return transfer(&reg, 1, buf, len);
}

int AF9838_Driver::write_reg(uint8_t reg, uint8_t val)
{
    const uint8_t cmd[2] = {reg, val};
    return transfer(cmd, sizeof(cmd), nullptr, 0);
}


void AF9838_Driver::RunImpl()
{
    if (!_running) {
        return;
    }

    static bool in_run = false;

    if (in_run) {
        return;
    }

    in_run = true;

    const hrt_abstime now = hrt_absolute_time();

    if (_last_run != 0 && (now - _last_run) < _measure_interval_us) {
        in_run = false;
        return;
    }

    _last_run = now;

    if (_waiting_data) {
        if (now - _last_trigger >= 5000) {
            uint8_t b[6] {};
            int16_t rx = 0, ry = 0, rz = 0;
            float mx_uT = 0.f, my_uT = 0.f, mz_uT = 0.f;
            float mx_G = 0.f, my_G = 0.f, mz_G = 0.f;

            if (read_block(REG_CH1_LSB, b, sizeof(b)) == PX4_OK) {
                rx = (int16_t)((b[1] << 8) | b[0]);
                ry = (int16_t)((b[3] << 8) | b[2]);
                rz = (int16_t)((b[5] << 8) | b[4]);

                mx_uT = rx * AF9838::LSB_TO_uT;
                my_uT = ry * AF9838::LSB_TO_uT;
                mz_uT = rz * AF9838::LSB_TO_uT;

                mx_G = mx_uT * AF9838::uT_TO_G;
                my_G = my_uT * AF9838::uT_TO_G;
                mz_G = mz_uT * AF9838::uT_TO_G;

                _px4_mag.update(now, mx_G, my_G, mz_G);

            }

            _waiting_data = false;
        }

    } else {
        if (write_reg(REG_STATE, STATE_SINGLE) == PX4_OK) {
            _last_trigger = now;
            _waiting_data = true;
        }
    }

#if defined(STATUS_HOFL)
    uint8_t st = read_reg(REG_STATUS);

    if (st & STATUS_HOFL) {
        PX4_WARN("AF9838: overflow");
    }

#endif

    in_run = false;
}


void AF9838_Driver::start()
{
    _running = true;
    ScheduleOnInterval(_measure_interval_us);
    PX4_INFO("AF9838: start() with interval %u us (%.1f Hz)",
         (unsigned)_measure_interval_us, 1e6 / static_cast<double>(_measure_interval_us));

}
void AF9838_Driver::stop()
{
    _running = false;
    ScheduleClear();
}

void AF9838_Driver::print_status()
{
    PX4_INFO("AF9838 @ I2C addr 0x%02x, running=%d", get_device_address(), _running ? 1 : 0);
}
