#pragma once
#define DEBUG_BUILD

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/device/i2c.h>
#include <uORB/Publication.hpp>

// Other device IDs are defined in drv_sensor.h
#define DRV_TEMP_DEVTYPE_THERMISTOR 0xC0

// External Bus Number
#define EXTERNAL_I2C_BUS 3
// This is the analog to digital convertor connected to the thermistor
#define MCP3221_ADDR 0x4d
// Barometric pressure sensor
#define MS5611_ADDR 0x77
// Humidity and temperature sensor
#define SHT41_ADDR 0x51


class ToddSensor : public ModuleBase<ToddSensor>, public ModuleParams, public device::I2C, public px4::ScheduledWorkItem {
public:
    ToddSensor();
    ~ToddSensor() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    int print_status() override;

    int init() override;
    void start();
    void stop();

private:
    int probe() override;
    void Run() override;

    float read_thermistor();
    float convert_to_temperature(uint16_t adc_value);

    static constexpr float VOLTAGE_REFERENCE = 3.3f;  // Adjust to your reference voltage
    static constexpr float THERMISTOR_R1 = 10000.0f;  // Your voltage divider resistor value

    bool _running{false};
    static constexpr uint32_t MEASUREMENT_INTERVAL{1000000}; // 1Hz

    // Performance counters
    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
    perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
