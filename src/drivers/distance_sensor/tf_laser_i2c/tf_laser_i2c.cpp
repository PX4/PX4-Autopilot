/****************************************************************************
 *
 *   Copyright (c) 2012-2023 PX4 Development Team. All rights reserved.
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
 * @file tf_laser_i2c.cpp
 *
 * Driver for the Benewake range finders Sensor connected via I2C.
 */
#include "tf_laser_i2c.hpp"

using namespace time_literals;

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))

/* Configuration Constants */
#define TF_LASER_I2C_BUS_SPEED                        100000 // 100kHz bus speed.
#define TF_LASER_I2C_BASE_ADDR                        0x10


#define TF_LASER_I2C_MEASURE_INTERVAL                 30_ms // 60ms minimum for one sonar.
#define TF_LASER_I2C_INTERVAL_BETWEEN_SUCCESIVE_FIRES 20_ms // 30ms minimum between each sonar measurement (watch out for interference!).
#define TFMINI_HEAD1                 0x59
#define TFMINI_HEAD2                 0x59


TF_LASER_I2C::TF_LASER_I2C(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{
	set_device_type(DRV_DIST_DEVTYPE_TF_LASER_I2C);
	_measure_interval = TF_LASER_I2C_MEASURE_INTERVAL;
}

TF_LASER_I2C::~TF_LASER_I2C()
{
	// Unadvertise the distance sensor topic.
	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	// Free perf counters.
	perf_free(_comms_error);
	perf_free(_sample_perf);
}

/**
 * @brief Get the parameter value, according to index and type
 */
int32_t
TF_LASER_I2C::get_param(uint8_t index, uint8_t type)
{
	const char *pname_format_tfi2c[3] = {"SENS_TF_ADDR%d", "SENS_TF_ROT%d", "SENS_TF_TYPE%d"};
	char p_name[16];
	int32_t value;
	sprintf(p_name, pname_format_tfi2c[type], index);
	param_t param_h = param_find(p_name);

	if (param_h != PARAM_INVALID) {
		param_get(param_h, &value);

	} else {
		PX4_DEBUG("PARAM_INVALID: %s", p_name);
	}

	return value;
}

/**
 * @brief Get the parameters of the sensor, according to the hw_type
 *
 * @input hw_type: Sensor type
 * @output dist_min dist_max fov
 * @return true if success
 * */
bool
TF_LASER_I2C::get_hw_setting(int32_t hw_type, int32_t *dist_min, int32_t *dist_max, int32_t *fov)
{
	switch (hw_type) {
	case HW_TYPE_TFmini: // TFmini (12m, 100 Hz)
		*dist_min = 40;
		*dist_max = 1200;
		*fov = 1.15f;
		break;

	case HW_TYPE_TFmini_plus: // TFminiPlus (10cm-1200cm, 3.6°)
		*dist_min = 10;
		*dist_max = 1200;
		*fov = 3.6f;
		break;

	case HW_TYPE_TFminiS: // TFminiS (10cm-1200cm, 2°)
		*dist_min = 10;
		*dist_max = 1200;
		*fov = 2.0f;
		break;

	case HW_TYPE_TF02Pro: // TF02Pro (10cm-4000cm, 3°)
		*dist_min = 10;
		*dist_max = 4000;
		*fov = 3.0f;
		break;

	case HW_TYPE_TF03: // TF03 (10-10000cm, 0.5°)
		*dist_min = 10;
		*dist_max = 10000;
		*fov = 0.5f;
		break;

	case HW_TYPE_TFLuna: // TFLuna (20cm-800cm, 2°)
		*dist_min = 20;
		*dist_max = 800;
		*fov = 2.0f;
		break;

	default:
		PX4_ERR("invalid HW model %" PRId32 ".", hw_type);
		return false;
	}

	return true;
}

int
TF_LASER_I2C::init()
{
	uint8_t number_of_sensors = _p_sensor_enabled.get();

	if (number_of_sensors == 0) {
		PX4_WARN("disabled");
		return PX4_ERROR;
	}

	// Initialize the I2C device
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	// Check for connected rangefinders on each i2c port by decrementing from the base address,
	uint8_t address;

	for (uint8_t index = 0; index < number_of_sensors ; index++) {
		address = (uint8_t)get_param(index, TF_PARAM_ADDR);
		set_device_address(address);
		px4_usleep(_measure_interval);

		if (measure() == PX4_OK) {
			px4_usleep(_measure_interval);
			// Store I2C address、rotation、range
			int32_t type, dist_min, dist_max, fov;
			type = get_param(index, (uint8_t)TF_PARAM_TYPE);
			bool ret = get_hw_setting(type, &dist_min, &dist_max, &fov);

			if (!ret) {
				PX4_ERR("get setting err,index:%d,address:%d", index, address);
				continue;
			}

			sensor_param_value[_sensor_count].addresses = address;
			sensor_param_value[_sensor_count].rotations = (uint8_t)get_param(index, TF_PARAM_ROT);
			sensor_param_value[_sensor_count].max_range = dist_max;
			sensor_param_value[_sensor_count].min_range = dist_min;
			sensor_param_value[_sensor_count].fov = fov;
			set_param();
			printf("tf[%i]: address 0x%02x ,rot %d, range:%.2f - %.2f m, tpye:%d\n", _sensor_count, \
			       get_device_address(), \
			       sensor_param_value[_sensor_count].rotations, \
			       (double)(sensor_param_value[_sensor_count].min_range / 100.0f), \
			       (double)(sensor_param_value[_sensor_count].max_range / 100.0f), \
			       (int)type);

			_sensor_count++;

			if (_sensor_count >= RANGE_FINDER_MAX_SENSORS) {
				break;
			}
		}

		px4_usleep(_measure_interval);
	}

	// Return an error if no sensors were detected.
	if (_sensor_count == 0) {
		PX4_ERR("no sensors discovered");
		return PX4_ERROR;
	}

	// If more than one sonar is detected, adjust the meaure interval to avoid sensor interference.
	if (_sensor_count > 1) {
		_measure_interval = TF_LASER_I2C_INTERVAL_BETWEEN_SUCCESIVE_FIRES;
	}

	PX4_INFO("Total sensors connected: %i", _sensor_count);
	start();
	return PX4_OK;
}

int
TF_LASER_I2C::set_param()
{
	int ret;
	const uint8_t CMD_OUTPUT_FORMAT_CM[] =   { 0x5A, 0x05, 0x05, 0x01, 0x65 };
	const uint8_t CMD_FRAME_RATE_100HZ[] =   { 0x5A, 0x06, 0x03, 0x64, 0x00, 0xC7 };
	const uint8_t CMD_ENABLE_DATA_OUTPUT[] = { 0x5A, 0x05, 0x07, 0x01, 0x67 };
	const uint8_t CMD_SAVE_SETTINGS[] =      { 0x5A, 0x04, 0x11, 0x6F };
	const uint8_t CMD_SYSTEM_RESET[] =       { 0x5A, 0x04, 0x04, 0x62 };
	const uint8_t *cmds[] = {
		CMD_OUTPUT_FORMAT_CM,
		CMD_FRAME_RATE_100HZ,
		CMD_ENABLE_DATA_OUTPUT,
		CMD_SAVE_SETTINGS,
	};

	for (uint8_t i = 0; i < ARRAY_SIZE(cmds); i++) {
		ret = tfi2c_transfer(cmds[i], cmds[i][1], nullptr, 0);

		if (!ret) {
			PX4_INFO(": Unable to set configuration register %u\n", cmds[i][2]);
			return ret;
		}

		px4_usleep(100_ms);
	}

	tfi2c_transfer(CMD_SYSTEM_RESET, sizeof(CMD_SYSTEM_RESET), nullptr, 0);

	return PX4_OK;
}

int
TF_LASER_I2C::measure()
{
	// Send the command to take a measurement.
	const uint8_t CMD_READ_MEASUREMENT[] = {0x5A, 0x05, 0x00, 0x01, 0x60 };
	int ret_val = transfer(CMD_READ_MEASUREMENT, 5, nullptr, 0);
	return ret_val;
}

int
TF_LASER_I2C::collect()
{
	uint16_t distance_cm = 0;
	const uint8_t Data_Len = 9;
	static  uint8_t raw_data[Data_Len];
	uint8_t CMD_READ_MEASUREMENT[] = { 0x5A, 0x05, 0x00, 0x01, 0x60 };

	perf_begin(_sample_perf);

	// Increment i2c adress to next sensor.
	_sensor_index++;
	_sensor_index %= _sensor_count;

	// Set the sensor i2c adress for the active cycle.
	set_device_address(sensor_param_value[_sensor_index].addresses);

	// Transfer data from the bus.
	int ret_val = tfi2c_transfer(CMD_READ_MEASUREMENT, 5, raw_data, Data_Len);

	if (ret_val < 0) {
		perf_count(_comms_error);
		perf_end(_sample_perf);
		return ret_val;
	}

	/**  header1, header2,distanceL,distanceH,timestampL,timestampH,checksum; **/
	if (raw_data[0] == TFMINI_HEAD1 && raw_data[1] == TFMINI_HEAD2) {
		if (check_checksum(raw_data, Data_Len)) {
			distance_cm = ((uint16_t)raw_data[3] << 8) | raw_data[2];
		}

	} else {
		return -EAGAIN;
	}

	if (distance_cm > sensor_param_value[_sensor_index].max_range) {
		distance_cm = sensor_param_value[_sensor_index].max_range ;
	}

	if (distance_cm < sensor_param_value[_sensor_index].min_range) {
		distance_cm = sensor_param_value[_sensor_index].min_range ;
	}


	float distance_m = static_cast<float>(distance_cm) * 1e-2f;

	distance_sensor_s report;
	report.current_distance = distance_m;
	report.device_id        = get_device_id();
	report.max_distance     = sensor_param_value[_sensor_index].max_range / 100.0f;
	report.min_distance     = sensor_param_value[_sensor_index].min_range / 100.0f;
	report.orientation      = sensor_param_value[_sensor_index].rotations;
	report.signal_quality   = -1;
	report.timestamp        = hrt_absolute_time();
	report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.variance         = 0.0f;
	report.h_fov = math::radians(sensor_param_value[_sensor_index].fov);
	report.v_fov = math::radians(sensor_param_value[_sensor_index].fov);

	int instance_id;
	orb_publish_auto(ORB_ID(distance_sensor), &_distance_sensor_topic, &report, &instance_id);

	perf_end(_sample_perf);
	return PX4_OK;
}


int
TF_LASER_I2C::check_checksum(uint8_t *arr, int pkt_len)
{
	uint8_t checksum = 0;
	int i;

	/* sum them all except the last (the checksum) */
	for (i = 0; i < pkt_len - 1; i++) {
		checksum += arr[i];
	}

	return checksum == arr[pkt_len - 1];
}

int
TF_LASER_I2C::tfi2c_transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
{
	bool send_and_receive = false;

	if (send != nullptr && send_len > 0) {

		int ret = transfer(send, send_len, nullptr, 0);
		send_and_receive = true;

		if (ret != PX4_OK) {
			return ret;
		}
	}

	if (recv != nullptr && recv_len > 0) {
		if (send_and_receive) {
			px4_usleep(500_us);
		}

		return transfer(nullptr, 0, recv, recv_len);
	}

	return PX4_ERROR;
}


void
TF_LASER_I2C::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_error);
	PX4_INFO("poll interval:  %ums", _measure_interval / 1000);

	for (int i = 0; i < _sensor_count; i++) {
		PX4_INFO("sensor: %i, address %u, rotation:%u, range:%.2f - %.2f m", i, sensor_param_value[i].addresses,
			 sensor_param_value[i].rotations,
			 (double)(sensor_param_value[i].min_range / 100.0f), (double)(sensor_param_value[i].max_range / 100.0f));
	}

}

void
TF_LASER_I2C::RunImpl()
{
	// Collect the sensor data.
	if (collect() != PX4_OK) {
		PX4_INFO("collection error");
		px4_usleep(1_s);

	}
}

int
TF_LASER_I2C::set_address(const uint8_t address)
{
	if (_sensor_count > 1) {
		PX4_INFO("multiple sensors are connected");
		return PX4_ERROR;
	}

	PX4_INFO("requested address: %u", address);

	uint8_t shifted_address = address ;
	uint8_t cmd[5] = {0x5A, 0x05, 0x0B, shifted_address, 0};
	cmd[4] = 0x5A + 0x05 + 0x0B + shifted_address;	//checksum

	if (transfer(cmd, sizeof(cmd), nullptr, 0) != PX4_OK) {
		PX4_INFO("could not set the address");
	}

	set_device_address(address);
	PX4_INFO("device address: %u", get_device_address());
	return PX4_OK;
}

void
TF_LASER_I2C::start()
{
	// Fetch parameter values.
	ModuleParams::updateParams();
	// Schedule the driver cycle at regular intervals.
	ScheduleOnInterval(_measure_interval);
}

void
TF_LASER_I2C::custom_method(const BusCLIArguments &cli)
{
	set_address(cli.i2c_address);
}

void
TF_LASER_I2C::print_usage()
{
	PRINT_MODULE_USAGE_NAME("tf_laser_i2c", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	// PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x16);
	PRINT_MODULE_USAGE_COMMAND("set_address");
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x16);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int tf_laser_i2c_main(int argc, char *argv[])
{
	using ThisDriver = TF_LASER_I2C;
	BusCLIArguments cli{true, false};
	cli.i2c_address = TF_LASER_I2C_BASE_ADDR;
	cli.default_i2c_frequency = TF_LASER_I2C_BUS_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_TF_LASER_I2C);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "set_address")) {
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
