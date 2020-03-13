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
#include <drivers/drv_anemometer.h> 
#include <perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/anemometer.h>

/* MappyDot Registers */
#define SET_CMD 					0x10

/* Device limits */
#define SFM_BUS_CLOCK					400000 // 400kHz bus speed
//#define SFM_DEVICE_PATH					"/dev/sfm"

/* Configuration Constants */
#define SFM_BASE_ADDR					0x40
#define TCA_ADDR					0x70
#define SFM_BUS_DEFAULT					PX4_I2C_BUS_ONBOARD
#define SFM_MEASUREMENT_INTERVAL_USEC			5000  // 5ms measurement interval, 200Hz.
#define ANEMOMETER_MAX_SENSORS                          3	// Maximum number of sensors on bus

using namespace time_literals;

class SFM : public device::I2C, public px4::ScheduledWorkItem
{
public:
	SFM(const int bus = SFM_BUS_DEFAULT);
	virtual ~SFM();

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

	/**
	 * Gets the current sensor rotation value.
	 */
	int get_sensor_rotation(const size_t index);

	px4::Array<uint8_t, ANEMOMETER_MAX_SENSORS> _sensor_port {};
	px4::Array<uint8_t, ANEMOMETER_MAX_SENSORS> _sensor_rotations {};

	size_t	_sensor_count{0};

	orb_advert_t _anemometer_topic[3]={};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "sfm_comms_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "sfm_sample_perf")};

	float offset{32000.0f};
	float scale{140.0f};
	float slm2ms{0.0546808f};
};


SFM::SFM(const int bus) :
        I2C("SFM", nullptr, bus, SFM_BASE_ADDR, SFM_BUS_CLOCK), // should SFM_BASE_ADDR -> TCA_ADDR
        ScheduledWorkItem(MODULE_NAME,px4::device_bus_to_wq(get_device_id()))
{
        // up the retries since the device misses the first measure attempts
        I2C::_retries = 3;
}

SFM::~SFM()
{
	// Ensure we are truly inactive.
	stop();

	// Unadvertise the distance sensor topic.
	for(int i=0;i<3;i++){
		if (_anemometer_topic[i] != nullptr) {
			orb_unadvertise(_anemometer_topic[i]);
		}
	}

	// Free perf counters.
	perf_free(_comms_errors);
	perf_free(_sample_perf);
}

int
SFM::init()
{
        int32_t hw_model = 0;
        param_get(param_find("SENS_EN_SFM"), &hw_model);

        if (hw_model == 0) {
		PX4_WARN("disabled");
		return PX4_ERROR;
	}

	if (I2C::init() != PX4_OK) {
		PX4_ERR("failed to init I2C");
		return PX4_ERROR;
	}

	// Allow for sensor auto-addressing time
	px4_usleep(1_s);

        // Check for connected anemometer on each i2c port,
	// starting from the base address 0x40 and incrementing
	int j=0;
	for (uint8_t i = 0; i <= ANEMOMETER_MAX_SENSORS; i++) {
//		// set TCA9578A chanal
//		set_device_address(TCA_ADDR);
//		uint8_t cmd = 1 << i; // for port #index in tca9548a
//		transfer(&cmd, 1, nullptr, 0);

		// Check if a sensor is present.
		set_device_address(SFM_BASE_ADDR);
		if (probe() != PX4_OK) {
			continue;
		}

		// Store I2C address
		_sensor_port[j] = i;
		_sensor_rotations[j] = get_sensor_rotation(i);
		_sensor_count++;
		j++;

		PX4_INFO("sensor at port #0%d at address 0x%02X added", i, get_device_address());
	}

	if (_sensor_count == 0) {
		return PX4_ERROR;
	}

	PX4_INFO("%i sensors connected", _sensor_count);

	return PX4_OK;
}

int
SFM::probe()
{
	uint8_t cmd[2] = {SET_CMD,0x00}; 
	int ret_val = transfer(&cmd[0], 2, nullptr, 0);

	return ret_val;
}

int
SFM::collect()
{        
	uint8_t val[2] = {};
	perf_begin(_sample_perf);

	// Increment the sensor index, (limited to the number of sensors connected).
	for (size_t index = 0; index < _sensor_count; index++) {
//		set_device_address(TCA_ADDR);
//		uint8_t cmd = 1 << _sensor_port[index]; // for port #_sensor_port[index] in tca9548a
//		transfer(&cmd, 1, nullptr, 0);

		// Set address of the current sensor to collect data from.
		set_device_address(SFM_BASE_ADDR);

		// Transfer data from the bus.
		probe();
		int ret_val = transfer(nullptr, 0, &val[0], 2);

		if (ret_val < 0) {
			PX4_ERR("sensor %i read failed, port: # %d", index, _sensor_port[index]);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret_val;
		}

		uint16_t speed = uint16_t(val[0]) << 8 | val[1];
		float speed_ms = static_cast<float>(speed - offset) / scale * slm2ms;

		anemometer_s report {};
		report.current_speed 	= speed_ms * 7.0f;
		report.id               = _sensor_port[index];
		report.orientation      = _sensor_rotations[index];
		report.signal_quality   = -1;
		report.timestamp        = hrt_absolute_time();
		report.variance         = 0;
		report.error_count = perf_event_count(_comms_errors);
		
		int	instance_id;
/*		if(_anemometer_topic[index]==nullptr) 
			_anemometer_topic[index] = orb_advertise_multi(ORB_ID(anemometer), &report, &instance_id, ORB_PRIO_DEFAULT);
		report.id               = instance_id;
		orb_publish(ORB_ID(anemometer), &_anemometer_topic[index], &report);		
*/
		orb_publish_auto(ORB_ID(anemometer), &_anemometer_topic[index], &report, &instance_id, ORB_PRIO_DEFAULT);
	}

        perf_count(_sample_perf);
	perf_end(_sample_perf);
	return PX4_OK;
}

int
SFM::get_sensor_rotation(const size_t index)
{
        int32_t _p_sensor1_rot = 1; // x
        int32_t _p_sensor2_rot = 2; // y
        int32_t _p_sensor3_rot = 3; // z
        param_get(param_find("SENS_SFM1_ROT"),&_p_sensor1_rot);
        param_get(param_find("SENS_SFM2_ROT"),&_p_sensor2_rot);
        param_get(param_find("SENS_SFM3_ROT"),&_p_sensor3_rot);
	switch (index) {
            case 0: return _p_sensor1_rot;

            case 1: return _p_sensor2_rot;

            case 2: return _p_sensor3_rot;

            default: return PX4_ERROR;
	}
}

void
SFM::print_info()
{
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
}

void
SFM::Run()
{
	// Collect the sensor data.
	if (collect() != PX4_OK) {
		PX4_INFO("collection error");
		// If an error occurred, restart the measurement state machine.
		start();
		return;
	}
}

void
SFM::start()
{
	// Schedule the driver to run on a set interval
        ScheduleOnInterval(SFM_MEASUREMENT_INTERVAL_USEC);
}

void
SFM::stop()
{
	ScheduleClear();
}


/**
 * Local functions in support of the shell command.
 */
namespace sfm
{

SFM *g_dev;

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
	g_dev = new SFM(i2c_bus);

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
	PX4_INFO("\t-b --bus i2cbus (%i)", SFM_BUS_DEFAULT);
	PX4_INFO("command:");
	PX4_INFO("\tstart|start_bus|status|stop");
	return PX4_OK;
}

} // namespace mappydot


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int sfm_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	int ch;
	int i2c_bus = SFM_BUS_DEFAULT;
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
			return sfm::usage();
		}
	}

	if (myoptind >= argc) {
		return sfm::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return sfm::start();

		} else {
			return sfm::start_bus(i2c_bus);
		}
	}

	// Print the driver status.
	if (!strcmp(argv[myoptind], "status")) {
		return sfm::status();
	}

	// Stop the driver.
	if (!strcmp(argv[myoptind], "stop")) {
		return sfm::stop();
	}

	// Print driver usage information.
	return sfm::usage();
}
