/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file sfm.cpp
 *
 * @author WANG Ze <ze.wang@upmc.etu.fr>
 *
 * Driver for the SENSIRION SFM Low Pressure DropDigital Flow Meter (i2c).
 * Default I2C address 0x40 is used.
 */

#include <string.h>

#include <containers/Array.hpp>
#include <drivers/device/i2c.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/topics/distance_sensor.h>

/* Configuration Constants */
#define TFMINI_S_I2C_BASEADDR                   0x10 // 7-bit address.
#define TFMINI_I2C_BASEADDR                     0x10 // 7-bit address.
#define TFMINI_PLUS_I2C_BASEADDR                0x16 // 7-bit address.
#define TFMINI_BUS_DEFAULT			PX4_I2C_BUS_ONBOARD

/* TFMINI-S Registers addresses */
#define MEASURE_REG1                            0x5A
#define MEASURE_REG2                            0x05
#define MEASURE_REG3                            0x00
#define MEASURE_REG4                            0x01
#define MEASURE_REG5                            0x60 // Measure range register.
#define TERARANGER_WHO_AM_I_REG                 0x01 // Who am I test register.
#define TERARANGER_WHO_AM_I_REG_VAL             0xA1

/* Device limits */
#define TFMINI_S_I2C_MAX_DISTANCE             (12.00f)
#define TFMINI_S_I2C_MIN_DISTANCE             (0.01f)

#define TFMINI_I2C_MAX_DISTANCE                 (3.0f)
#define TFMINI_I2C_MIN_DISTANCE                 (0.10f)

#define TFMINI_PLUS_I2C_MAX_DISTANCE         (60.0f)
#define TFMINI_PLUS_I2C_MIN_DISTANCE         (0.50f)

#define TFMINI_MEASUREMENT_INTERVAL         10_ms
#define NUM_TFMINI_MAX                      5	// Maximum number of sensors on bus
#define TFMINI_BUS_CLOCK                    100000 // 100kHz bus speed

using namespace time_literals;

class TFMINI_I2C : public device::I2C, public px4::ScheduledWorkItem
{
public:
        TFMINI_I2C(const int bus = TFMINI_BUS_DEFAULT);
        virtual ~TFMINI_I2C();

	/**
	 * Initializes the sensors, advertises uORB topic,
	 * sets device addresses
	 */
	virtual int init() override;

	/**
	 * Prints basic diagnostic information about the driver.
	 */
	void print_info();

	/**
	 * Initializes the automatic measurement state machine and starts the driver.
	 */
	void start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop();

protected:

private:

	/**
	 * Sends an i2c measure command to check for presence of a sensor.
	 */
	int probe();

	/**
	 * Collects the most recent sensor measurement data from the i2c bus.
	 */
        int collect();

	/**
	* Performs a poll cycle; collect from the previous measurement and start a new one.
	*/
	void Run() override;

        orb_advert_t _tfmini_topic={};

        perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "tfmini_comms_err")};
        perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "tfmini_sample_perf")};

        int Address[NUM_TFMINI_MAX] = {};
        size_t	_sensor_count{0};
};


static uint8_t crc8(uint8_t *p, uint8_t len)
{
        uint16_t i;
        uint16_t crc = 0x0;

        for(i=0;i<len;i++)
            crc += p[i];

        return crc & 0xFF;
}


TFMINI_I2C::TFMINI_I2C(const int bus) :
        I2C("TFMINI_I2C", nullptr, bus, TFMINI_S_I2C_BASEADDR, TFMINI_BUS_CLOCK),
        ScheduledWorkItem(MODULE_NAME,px4::device_bus_to_wq(get_device_id()))
{
        // up the retries since the device misses the first measure attempts
        I2C::_retries = 3;
}

TFMINI_I2C::~TFMINI_I2C()
{
	// Ensure we are truly inactive.
	stop();

	// Unadvertise the distance sensor topic.

        if (_tfmini_topic != nullptr) {
                orb_unadvertise(_tfmini_topic);
        }

	// Free perf counters.
	perf_free(_comms_errors);
	perf_free(_sample_perf);
}

int
TFMINI_I2C::init()
{
        int32_t hw_model = 0;
        param_get(param_find("SENS_EN_TFMINI2C"), &hw_model);

        switch (hw_model) {
        case 0: // Disabled
            PX4_WARN("Disabled");
            return PX4_ERROR;
            break;
        case 1: // Autodetect - assume default TFmini-S (12m, 100 Hz)
                // Check for connected tfmini-s on each i2c port,
                // starting from the base address 0x10 and incrementing
                for (uint8_t i = 0; i <= NUM_TFMINI_MAX; i++) {
                    // Check if a sensor is present.
                    int ADD = TFMINI_S_I2C_BASEADDR + i;
                    set_device_address(ADD);

                    if (I2C::init() != OK) {
                        PX4_ERR("failed to init I2C, the TFmini-S at 0x%02x",ADD);
                        continue;
                    } else {
                        Address[_sensor_count] = ADD;
                        _sensor_count++;
                        PX4_INFO("Initialized one TFmini-S at 0x%02x",ADD);
                    }
                }

                if (_sensor_count == 0){
                    PX4_ERR("no sensor is searched");
                    return PX4_ERROR;
                } else
                    PX4_INFO("%i sensor(s) connected", _sensor_count);

                break;

        case 2: // TFmini (12m, 100 Hz)
                // Check for connected tfmini-s on each i2c port,
                // starting from the base address 0x10 and incrementing
                for (uint8_t i = 0; i <= NUM_TFMINI_MAX; i++) {
                    // Check if a sensor is present.
                    int ADD = TFMINI_I2C_BASEADDR + i;
                    set_device_address(ADD);

                    if (I2C::init() != OK) {
                        PX4_DEBUG("failed to init I2C, the TFmini-S at 0x%02x",ADD);
                        continue;
                    } else {
                        Address[_sensor_count] = ADD;
                        _sensor_count++;
                        PX4_INFO("Initialized one TFmini-S at 0x%02x",ADD);
                    }
                }

                if (_sensor_count == 0){
                    PX4_ERR("no sensor is searched");
                    return PX4_ERROR;
                } else
                    PX4_INFO("%i sensor(s) connected", _sensor_count);

                break;

        case 3: // TFmini_Plus.
            // Check for connected tfmini-s on each i2c port,
            // starting from the base address 0x10 and incrementing
            for (uint8_t i = 0; i <= NUM_TFMINI_MAX; i++) {
                // Check if a sensor is present.
                int ADD = TFMINI_PLUS_I2C_BASEADDR + i;
                set_device_address(ADD);

                if (I2C::init() != OK) {
                    PX4_DEBUG("failed to init I2C, the TFmini-S at 0x%02x",ADD);
                    continue;
                } else {
                    Address[_sensor_count] = ADD;
                    _sensor_count++;
                    PX4_INFO("Initialized one TFmini-S at 0x%02x",ADD);
                }
            }

            if (_sensor_count == 0){
                PX4_ERR("no sensor is searched");
                return PX4_ERROR;
            } else
                PX4_INFO("%i sensor(s) connected", _sensor_count);

                break;

        default:
                PX4_ERR("invalid HW model %d.", hw_model);
                return PX4_ERROR;
	}

	// Allow for sensor auto-addressing time
	px4_usleep(1_s);

        for(uint8_t i=0;i<_sensor_count;i++){
            PX4_INFO("address: 0x%2x",Address[i]);
            uint8_t val[9] = {};

            // Set address of the current sensor to collect data from.
            set_device_address(Address[i]);
            I2C::init();
            // Transfer data from the bus.
            int ret_val = probe();
            if (ret_val != PX4_OK) {
                    perf_count(_comms_errors);
                    PX4_ERR("TFMINI_I2C::probe: i2c::transfer returned %d", ret_val);
                    continue;
            }
            ret_val = transfer(nullptr, 0, &val[0], 9);

            if (ret_val != PX4_OK) {
                    PX4_ERR("sensor read failed");
                    perf_count(_comms_errors);
                    perf_end(_sample_perf);
                    continue;
            }
            PX4_INFO("distance_cm: %u",(val[3] << 8) | val[2]);
        }

	return PX4_OK;
}

int
TFMINI_I2C::probe()
{
        const uint8_t cmd[5] = {MEASURE_REG1,MEASURE_REG2,MEASURE_REG3,MEASURE_REG4,MEASURE_REG5};
        int ret_val = transfer(&cmd[0], sizeof(cmd), nullptr, 0);
	return ret_val;
}

int
TFMINI_I2C::collect()
{
    int ret = PX4_OK;
    for(uint8_t i=0;i<_sensor_count;i++){

        uint8_t val[9] = {};
        perf_begin(_sample_perf);

        // Set address of the current sensor to collect data from.
        set_device_address(Address[i]);
        I2C::init();
        // Transfer data from the bus.
        int ret_val = probe();
        if (ret_val != PX4_OK) {
                perf_count(_comms_errors);
                PX4_ERR("TFMINI_I2C::probe: i2c::transfer returned %d", ret_val);
                ret = ret_val;
                continue;
        }
        ret_val = transfer(nullptr, 0, &val[0], 9);

        if (ret_val != PX4_OK) {
                PX4_ERR("sensor read failed");
                perf_count(_comms_errors);
                perf_end(_sample_perf);
                ret = ret_val;
                continue;
        }

        if (val[0] != 0x59 || val[1] != 0x59)
            PX4_ERR("reading from sensor: 0x%02X 0x%02X", val[0], val[1]);

        if (crc8(val, 8) == val[8]) {

            uint16_t distance_cm = (val[3] << 8) | val[2];
            PX4_INFO("distance_cm: %u",distance_cm);
            float distance_m = static_cast<float>(distance_cm) * 1e-2f;
            int strength = (val[5] << 8) | val[4];

            // Final data quality evaluation. This is based on the datasheet and simple heuristics retrieved from experiments
            // Step 1: Normalize signal strength to 0...100 percent using the absolute signal peak strength.
            uint8_t signal_quality = 100 * strength / 65535.0f;

            // Step 2: Filter physically impossible measurements, which removes some crazy outliers that appear on LL40LS.
            if (distance_m < TFMINI_S_I2C_MIN_DISTANCE)
                signal_quality = 0;
            if((strength == -1) || (strength == -2))
                signal_quality = strength;

            distance_sensor_s report {};
            report.current_distance = distance_m;
            report.signal_quality = signal_quality;
            report.min_distance = TFMINI_S_I2C_MIN_DISTANCE;
            report.max_distance = TFMINI_S_I2C_MAX_DISTANCE;
            report.h_fov = 2;
            report.v_fov = 2;
            report.id = Address[i] - TFMINI_S_I2C_BASEADDR;
//            switch (Address[i]) {
//            case TFMINI_S_I2C_BASEADDR:
//                report.id = 0; // downward faceing
//                break;

//            case 0x11:
//                report.id = 1; // forward faceing
//                break;

//            case 0x12:
//                report.id = 2; // left faceing
//                break;

//            case 0x13:
//                report.id = 3; // right faceing
//                break;.

//            case 0x14:
//                report.id = 4; // upward faceing
//                break;

//            case 0x15:
//                report.id = 5; // backward faceing
//                break;
//            default:
//                break;
//            }


            int	instance_id = i;
    /*		if(_anemometer_topic[index]==nullptr)
                    _anemometer_topic[index] = orb_advertise_multi(ORB_ID(anemometer), &report, &instance_id, ORB_PRIO_DEFAULT);
            report.id               = instance_id;
            orb_publish(ORB_ID(anemometer), &_anemometer_topic[index], &report);
    */
            orb_publish_auto(ORB_ID(distance_sensor), &_tfmini_topic, &report, &instance_id, ORB_PRIO_DEFAULT);

            PX4_INFO("Tested data as %f m",double(distance_m));
        }
        else
            PX4_INFO("reading from sensor: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", val[0], val[1], val[2], val[3], val[4]
                    , val[5], val[6], val[7], val[8]);

        perf_count(_sample_perf);
        perf_end(_sample_perf);
    }

        return ret;
}

void
TFMINI_I2C::print_info()
{
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
}

void
TFMINI_I2C::Run()
{
    // Collect the sensor data.

    if ( collect() != PX4_OK) {
        PX4_INFO("collection error");
        // If an error occurred, restart the measurement state machine.
        start();
        return;
    }
}

void
TFMINI_I2C::start()
{
	// Schedule the driver to run on a set interval
        ScheduleOnInterval(TFMINI_MEASUREMENT_INTERVAL);
}

void
TFMINI_I2C::stop()
{
	ScheduleClear();
}


/**
 * Local functions in support of the shell command.
 */
namespace tfmini_i2c
{

TFMINI_I2C *g_dev;

int start();
int start_bus(int i2c_bus);
int status();
int stop();
int usage();

/**
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 */
int
start()
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (size_t i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i]) == PX4_OK) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(int i2c_bus)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
        g_dev = new TFMINI_I2C(i2c_bus);

	if (g_dev == nullptr) {
		delete g_dev;
		return PX4_ERROR;
	}

	// Initialize the sensor.
	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	// Start the driver.
	g_dev->start();

	PX4_INFO("driver started");
	return PX4_OK;
}

/**
 * Print the driver status.
 */
int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

/**
 * Stop the driver.
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	}

	PX4_INFO("driver stopped");
	return PX4_OK;
}

/**
 * Print usage information about the driver.
 */
int
usage()
{
	PX4_INFO("Usage: mappydot <command> [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-a --all");
        PX4_INFO("\t-b --bus i2cbus (%i)", TFMINI_BUS_DEFAULT);
	PX4_INFO("command:");
	PX4_INFO("\tstart|start_bus|status|stop");
	return PX4_OK;
}

} // namespace mappydot


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int tfmini_i2c_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	int ch;
        int i2c_bus = TFMINI_BUS_DEFAULT;
	int myoptind = 1;

	bool start_all = false;

	while ((ch = px4_getopt(argc, argv, "ab:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			start_all = true;
			break;

		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option!");
                        return tfmini_i2c::usage();
		}
	}

	if (myoptind >= argc) {
                return tfmini_i2c::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
                        return tfmini_i2c::start();

		} else {
                        return tfmini_i2c::start_bus(i2c_bus);
		}
	}

	// Print the driver status.
	if (!strcmp(argv[myoptind], "status")) {
                return tfmini_i2c::status();
	}

	// Stop the driver.
	if (!strcmp(argv[myoptind], "stop")) {
                return tfmini_i2c::stop();
	}

	// Print driver usage information.
        return tfmini_i2c::usage();
}
