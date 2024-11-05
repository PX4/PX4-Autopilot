#include "ToddSensor.hpp"
#include <math.h>

using namespace time_literals;

ToddSensor *g_dev{nullptr};

ToddSensor::ToddSensor() :
    ModuleParams(nullptr),
    I2C(DRV_TEMP_DEVTYPE_THERMISTOR, MODULE_NAME, EXTERNAL_I2C_BUS, MCP3221_ADDR, 100000),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
    _retries = 3;
}

ToddSensor::~ToddSensor()
{
    stop();
}

int ToddSensor::probe()
{
    PX4_INFO("Probing ToddSensor on bus %d, address 0x%02X...", get_device_bus(), get_device_address());

    uint8_t data[2] = {0};
    const uint8_t cmd[] = {0}; // Empty command to trigger read

    // Use I2C::transfer() instead of ambiguous read()
    int ret = transfer(cmd, 0, data, sizeof(data));

    if (ret != PX4_OK) {
        PX4_DEBUG("probe failed on bus %d (error: %d)", get_device_bus(), ret);
        return ret;
    }

    PX4_DEBUG("probe success on bus %d, address 0x%02X", get_device_bus(), get_device_address());
    return PX4_OK;
}

int ToddSensor::init()
{
    PX4_DEBUG("Initializing ToddSensor");
    int ret = I2C::init();
    if (ret != PX4_OK) {
        PX4_ERR("I2C init failed: %d", ret);
        return ret;
    }

    ret = probe();
    if (ret != PX4_OK) {
        PX4_ERR("Probe failed: %d", ret);
        return ret;
    }

    PX4_INFO("Init successful, starting measurements");
    _running = true;
    ScheduleOnInterval(1_s);
    return PX4_OK;
}

void ToddSensor::stop()
{
    if (_running) {
        _running = false;
        ScheduleClear();
    }
}

float ToddSensor::read_thermistor()
{
    uint8_t data[2] = {0};
    const uint8_t cmd[] = {0}; // Empty command to trigger read

    // Add more detailed error checking
    int ret = I2C::transfer(cmd, 0, data, sizeof(data));
    if (ret != PX4_OK) {
        PX4_ERR("I2C transfer failed: %d", ret);
        return NAN;
    }

    // Print raw bytes for debugging
    PX4_DEBUG("Raw bytes: 0x%02X 0x%02X", data[0], data[1]);

    uint16_t adc_value = ((data[0] & 0x0F) << 8) | data[1];

    // Validate ADC value
    if (adc_value == 0x0000 || adc_value == 0x0FFF) {
        PX4_ERR("Invalid ADC value: 0x%03X", adc_value);
        return NAN;
    }

    PX4_DEBUG("Raw ADC: 0x%03X (%d)", adc_value, adc_value);
    return convert_to_temperature(adc_value);
}

float ToddSensor::convert_to_temperature(uint16_t adc_value)
{
    float voltage = (adc_value * VOLTAGE_REFERENCE) / 4095.0f;
    float r = 10000.0f * ((VOLTAGE_REFERENCE / voltage) - 1);  // Using 10kΩ thermistor

    // Steinhart-Hart coefficients from calibration
    const float c1 = 0.8237883661e-3f;
    const float c2 = 2.642550066e-4f;
    const float c3 = 1.24004296e-7f;

    float log_r = logf(r);
    float temp_k = 1.0f / (c1 + c2 * log_r + c3 * powf(log_r, 3));
    return temp_k - 273.15f;  // Convert to Celsius
}

int ToddSensor::task_spawn(int argc, char *argv[])
{
    PX4_INFO("ToddSensor task_spawn called with argc: %d", argc);

    // Default to external I2C bus 3
    int bus = 3;
    int bus_frequency = 50; // 50 kHz

    // Parse arguments - expect "-X -b <number> -f <freq>"
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-b") == 0 && (i + 1) < argc) {
            bus = atoi(argv[i + 1]);
            i++;
        } else if (strcmp(argv[i], "-f") == 0 && (i + 1) < argc) {
            bus_frequency = atoi(argv[i + 1]);
            i++;
        }
    }

    PX4_INFO("Starting on I2C bus %d at %d kHz", bus, bus_frequency);

    ToddSensor *instance = new ToddSensor();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init() == PX4_OK) {
            return PX4_OK;
        }
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

void ToddSensor::Run()
{
    PX4_INFO("Running ToddSensor");
    if (!_running) {
        return;
    }

    float temp = read_thermistor();
    if (!std::isnan(temp)) {
        PX4_INFO("Temperature: %.2f°C", (double)temp);
    } else {
        PX4_ERR("Failed to get valid temperature reading");
    }
}

// Main entry point
extern "C" __EXPORT int todd_sensor_main(int argc, char *argv[])
{
    PX4_INFO("Todd Sensor main called with argc: %d", argc);

    if (argc < 2) {
        PX4_ERR("missing command");
        return -1;
    }

    if (!strcmp(argv[1], "start")) {
        PX4_INFO("Start command received");
        if (g_dev != nullptr) {
            PX4_ERR("already started");
            return -1;
        }

        g_dev = new ToddSensor(); // I2C bus 1
        if (g_dev == nullptr) {
            PX4_ERR("alloc failed");
            return -1;
        }

        if (g_dev->init() != OK) {
            delete g_dev;
            g_dev = nullptr;
            return -1;
        }
        return OK;
    }

    if (!strcmp(argv[1], "stop")) {
        PX4_INFO("Stop command received");
        if (g_dev == nullptr) {
            PX4_ERR("not running");
            return -1;
        }
        delete g_dev;
        g_dev = nullptr;
        return OK;
    }

    PX4_ERR("unknown command");
    return -1;
}

int ToddSensor::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}


int ToddSensor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
			Reads the temperature from a thermistor connected to an MCP3221 ADC.
		)DESCR_STR"
	);
	return 0;
}
