/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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

#include <drivers/device/i2c.h>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

enum MS5837_TYPE {
	MS5837_30BA = 0,
	MS5837_02BA,
};


#pragma pack(push,1)

/**
 * Calibration PROM as reported by the device.
 */
typedef struct prom_s {
	uint16_t factory_setup;
	uint16_t c1_pressure_sens;
	uint16_t c2_pressure_offset;
	uint16_t c3_temp_coeff_pres_sens;
	uint16_t c4_temp_coeff_pres_offset;
	uint16_t c5_reference_temp;
	uint16_t c6_temp_coeff_temp;
	uint16_t dummy0;
} prom_s;

typedef union prom_u {
	uint16_t c[8];
	prom_s s;
} prom_u;

#pragma pack(pop)


class MS5837 : public device::I2C, public I2CSPIDriver<MS5837> {
public:
    MS5837(I2CSPIBusOption bus_option, const int bus, int bus_frequency, MS5837_TYPE ms5837_type);

    ~MS5837() override;

    static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
                                         int runtime_instance);
    static void print_usage();

    int init() override;

    int probe() override;

    void RunImpl();

protected:
    void print_status() override;

    void start();

    int reset();

    int measure();

    int collect();

    int read_prom();

    int read_raw(uint32_t *data);

    int measure(unsigned addr);

    bool crc4(uint16_t *n_prom);

    PX4Barometer _px4_barometer;

    enum MS5837_TYPE _ms5837_type;

    prom_u _prom;

    bool _collect_phase{false};
    unsigned _measure_phase{false};

    /* intermediate temperature values per MS5837 datasheet */
    int64_t _OFF{0};
    int64_t _SENS{0};

    perf_counter_t _sample_perf;
    perf_counter_t _measure_perf;
    perf_counter_t _comms_errors;

    float temperature;
    float cal_pressure;
    float pressure;
    float depth;
    float altitude;
};
