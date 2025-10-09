/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team.
 *   Modified for Keller LD sensor by Blue Robotics Inc.
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

class Bar100 : public device::I2C, public I2CSPIDriver<Bar100>
{
public:
    Bar100(const I2CSPIDriverConfig &config);
    ~Bar100() override;

    static void print_usage();
    int init();
    void RunImpl();
    void print_status() override;
    int read(unsigned offset, void *data, unsigned count) override;

private:
    int probe() override;
    int _read_sensor();
    int _reset();
    bool _read_memory_map();
    void _start();
    bool _read_mtp16(uint8_t mtp_addr, uint16_t &val);
    bool _read_registers(uint8_t reg, uint8_t *buf, size_t len);

    // Conversion helpers
    float _pressure();
    float _temperature();
    float _depth();
    float _altitude();

    uint16_t _equipment{0};
    uint16_t _place{0};
    uint16_t _file{0};
    uint32_t _code{0};

    uint16_t _P{0};
    uint8_t _mode{0};
    float _P_mode{0.0f};
    float _P_min{0.0f};
    float _P_max{0.0f};
    bool _status();
    bool _is_initialized();

    // Internal state
    uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};

    bool _collect_phase{false};
    bool _measure_phase{false};
    bool _initialized{false};

    float _last_temperature{NAN};
    float _last_pressure{NAN};
    float _P_bar{0.0f};
    float _fluid_density{1029.0f}; // Default seawater density

    // Performance counters
    perf_counter_t _sample_perf;
    perf_counter_t _measure_perf;
    perf_counter_t _comms_errors;
};
