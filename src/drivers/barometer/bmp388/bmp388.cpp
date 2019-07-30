/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file bmp388.cpp
 * Driver for the BMP388 barometric pressure sensor connected via I2C
 * (SPI still TODO/test).
 */

#include "bmp388.h"

#include <lib/drivers/barometer/PX4Barometer.hpp>

enum BMP388_BUS {
	BMP388_BUS_ALL = 0,
	BMP388_BUS_I2C_INTERNAL,
	BMP388_BUS_I2C_INTERNAL1,
	BMP388_BUS_I2C_EXTERNAL,
	BMP388_BUS_SPI_INTERNAL,
	BMP388_BUS_SPI_EXTERNAL
};

/*
 * BMP388 internal constants and data structures.
 */

class BMP388 : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	BMP388(bmp388::IBMP388 *interface, const char *path);
	virtual ~BMP388();

	virtual int		init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

private:
	PX4Barometer		_px4_baro;

	bmp388::IBMP388		*_interface;

	bool               	_running{false};

	uint8_t				_curr_ctrl{0};

	unsigned			_report_interval{0}; // 0 - no cycling, otherwise period of sending a report
	unsigned			_max_measure_interval{0}; // interval in microseconds needed to measure


	bool			_collect_phase{false};

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	struct bmp388::calibration_s *_cal; // stored calibration constants
	struct bmp388::fcalibration_s _fcal; // pre processed calibration constants

	float			_P; /* in Pa */
	float			_T; /* in K */


	/* periodic execution helpers */
	void			start_cycle();
	void			stop_cycle();

	void			Run() override;

	int		measure(); //start measure
	int		collect(); //get results and publish

	// from BMP3 library...
	bool			soft_reset();
	bool			get_calib_data();
	bool			validate_trimming_param();
	bool 			set_sensor_settings();
	bool			set_op_mode(uint8_t op_mode);

	bool 			get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data);
	bool			compensate_data(uint8_t sensor_comp, const struct bmp3_uncomp_data *uncomp_data, struct bmp3_data *comp_data);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int bmp388_main(int argc, char *argv[]);

BMP388::BMP388(bmp388::IBMP388 *interface, const char *path) :
	CDev(path),
	ScheduledWorkItem(px4::device_bus_to_wq(interface->get_device_id())),
	_px4_baro(interface->get_device_id()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmp388: read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "bmp388: measure")),
	_comms_errors(perf_alloc(PC_COUNT, "bmp388: comms errors"))
{
	_px4_baro.set_device_type(DRV_DEVTYPE_BMP388);
}

BMP388::~BMP388()
{
	// make sure we are truly inactive
	stop_cycle();

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

bool
BMP388::soft_reset()
{
	bool    result = false;
	uint8_t status;
	int     ret;

	status = _interface->get_reg(BMP3_SENS_STATUS_REG_ADDR);

	if (status & BMP3_CMD_RDY) {
		ret = _interface->set_reg(BPM3_CMD_SOFT_RESET, BMP3_CMD_ADDR);

		if (ret == OK) {
			usleep(BMP3_POST_RESET_WAIT_TIME);
			status = _interface->get_reg(BMP3_ERR_REG_ADDR);

			if ((status & BMP3_CMD_ERR) == 0) {
				result = true;
			}
		}
	}

	return result;
}

/*
 * @brief function to calculate CRC for the trimming parameters
 *
 * See https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/self-test/bmp3_selftest.c
 * */
static int8_t cal_crc(uint8_t seed, uint8_t data)
{
	int8_t poly = 0x1D;
	int8_t var2;
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if ((seed & 0x80) ^ (data & 0x80)) {
			var2 = 1;

		} else {
			var2 = 0;
		}

		seed = (seed & 0x7F) << 1;
		data = (data & 0x7F) << 1;
		seed = seed ^ (uint8_t)(poly * var2);
	}

	return (int8_t)seed;
}

/*
 * @brief Function to verify the trimming parameters
 *
 * See https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/self-test/bmp3_selftest.c
 * */
bool
BMP388::validate_trimming_param()
{
	bool    result = false;
	uint8_t crc = 0xFF;
	uint8_t stored_crc;
	uint8_t trim_param[21];
	uint8_t i;

	memcpy(&trim_param, _cal, 21);

	for (i = 0; i < 21; i++) {
		crc = (uint8_t)cal_crc(crc, trim_param[i]);
	}

	crc = (crc ^ 0xFF);

	stored_crc = _interface->get_reg(BMP3_TRIM_CRC_DATA_ADDR);

	if (stored_crc == crc) {
		result = true;
	}

	return result;
}

bool
BMP388::set_sensor_settings()
{
	int ret;

	uint8_t press_os = BMP3_NO_OVERSAMPLING;
	uint8_t temp_os = BMP3_NO_OVERSAMPLING;

	/*
	  Pressure      Temperature   Measurement
	  Oversample    Oversample    Time Max
	  x1            1x            6.4 ms
	  x2            1x            8.7 ms
	  x4            1x            13.3 ms
	  x8            1x            22.5 ms
	  x16           2x            43.2 ms
	 */
	_max_measure_interval = 6400;

	/* Select the pressure and temperature sensor to be enabled */
	uint8_t pwc_ctl_reg = 0;
	pwc_ctl_reg = BMP3_SET_BITS_POS_0(pwc_ctl_reg, BMP3_PRESS_EN, BMP3_ENABLE);
	pwc_ctl_reg = BMP3_SET_BITS(pwc_ctl_reg, BMP3_TEMP_EN, BMP3_ENABLE);

	ret = _interface->set_reg(pwc_ctl_reg, BMP3_PWR_CTRL_ADDR);

	if (ret != OK) {
		PX4_WARN("failed to set settings BMP3_PWR_CTRL_ADDR");
		return false;
	}

	/* Select the output data rate and over sampling settings for pressure and temperature */
	uint8_t osr_ctl_reg = 0;
	osr_ctl_reg = BMP3_SET_BITS_POS_0(osr_ctl_reg, BMP3_PRESS_OS, press_os);
	osr_ctl_reg = BMP3_SET_BITS(osr_ctl_reg, BMP3_TEMP_OS, temp_os);

	/* 0x1C is the register address of over sampling register */
	ret = _interface->set_reg(osr_ctl_reg, BMP3_OSR_ADDR);

	if (ret != OK) {
		PX4_WARN("failed to set settings BMP3_OSR_ADDR");
		return false;
	}

	uint8_t odr_ctl_reg = 0;
	odr_ctl_reg = BMP3_SET_BITS_POS_0(odr_ctl_reg, BMP3_ODR, BMP3_ODR_25_HZ);

	/* 0x1D is the register address of output data rate register */
	ret = _interface->set_reg(odr_ctl_reg, 0x1D);

	if (ret != OK) {
		PX4_WARN("failed to set settingsoutput data rate register");
		return false;
	}

	return true;
}


/*!
 * @brief This API sets the power mode of the sensor.
 */
bool
BMP388::set_op_mode(uint8_t op_mode)
{
	bool    result = false;
	uint8_t last_set_mode;
	uint8_t op_mode_reg_val;
	int     ret = OK;

	op_mode_reg_val = _interface->get_reg(BMP3_PWR_CTRL_ADDR);
	last_set_mode = BMP3_GET_BITS(op_mode_reg_val, BMP3_OP_MODE);

	/* If the sensor is not in sleep mode put the device to sleep
	 * mode */
	if (last_set_mode != BMP3_SLEEP_MODE) {
		/* Device should be put to sleep before transiting to
		* forced mode or normal mode */
		op_mode_reg_val = op_mode_reg_val & (~(BMP3_OP_MODE_MSK));
		ret = _interface->set_reg(op_mode_reg_val, BMP3_PWR_CTRL_ADDR);

		if (ret != OK) {
			return false;
		}

		px4_usleep(BMP3_POST_SLEEP_WAIT_TIME);
	}

	if (ret == OK) {
		op_mode_reg_val = _interface->get_reg(BMP3_PWR_CTRL_ADDR);
		op_mode_reg_val = BMP3_SET_BITS(op_mode_reg_val, BMP3_OP_MODE, op_mode);
		ret = _interface->set_reg(op_mode_reg_val, BMP3_PWR_CTRL_ADDR);

		if (ret != OK) {
			return false;
		}

		result = true;
	}

	return result;
}

/*!
 *  @brief This internal API is used to parse the pressure or temperature or
 *  both the data and store it in the bmp3_uncomp_data structure instance.
 */
static void parse_sensor_data(const uint8_t *reg_data, struct bmp3_uncomp_data *uncomp_data)
{
	/* Temporary variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	/* Store the parsed register values for pressure data */
	data_xlsb = (uint32_t)reg_data[0];
	data_lsb = (uint32_t)reg_data[1] << 8;
	data_msb = (uint32_t)reg_data[2] << 16;
	uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for temperature data */
	data_xlsb = (uint32_t)reg_data[3];
	data_lsb = (uint32_t)reg_data[4] << 8;
	data_msb = (uint32_t)reg_data[5] << 16;
	uncomp_data->temperature = data_msb | data_lsb | data_xlsb;
}


/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 * For eg if returned temperature is 2426 then it is 2426/100 = 24.26 deg Celsius
 */
static int64_t compensate_temperature(const struct bmp3_uncomp_data *uncomp_data, struct bmp3_calib_data *calib_data)
{
	int64_t partial_data1;
	int64_t partial_data2;
	int64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;
	int64_t partial_data6;
	int64_t comp_temp;

	partial_data1 = ((int64_t)uncomp_data->temperature - (256 * calib_data->reg_calib_data.par_t1));
	partial_data2 = calib_data->reg_calib_data.par_t2 * partial_data1;
	partial_data3 = (partial_data1 * partial_data1);
	partial_data4 = (int64_t)partial_data3 * calib_data->reg_calib_data.par_t3;
	partial_data5 = ((int64_t)(partial_data2 * 262144) + partial_data4);
	partial_data6 = partial_data5 / 4294967296;

	/* Store t_lin in dev. structure for pressure calculation */
	calib_data->reg_calib_data.t_lin = partial_data6;
	comp_temp = (int64_t)((partial_data6 * 25) / 16384);

	return comp_temp;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 * for eg return if pressure is 9528709 which is 9528709/100 = 95287.09 Pascal or 952.8709 hecto Pascal
 */
static uint64_t compensate_pressure(const struct bmp3_uncomp_data *uncomp_data,
				    const struct bmp3_calib_data *calib_data)
{
	const struct bmp3_reg_calib_data *reg_calib_data = &calib_data->reg_calib_data;
	int64_t partial_data1;
	int64_t partial_data2;
	int64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;
	int64_t partial_data6;
	int64_t offset;
	int64_t sensitivity;
	uint64_t comp_press;

	partial_data1 = reg_calib_data->t_lin * reg_calib_data->t_lin;
	partial_data2 = partial_data1 / 64;
	partial_data3 = (partial_data2 * reg_calib_data->t_lin) / 256;
	partial_data4 = (reg_calib_data->par_p8 * partial_data3) / 32;
	partial_data5 = (reg_calib_data->par_p7 * partial_data1) * 16;
	partial_data6 = (reg_calib_data->par_p6 * reg_calib_data->t_lin) * 4194304;
	offset = (reg_calib_data->par_p5 * 140737488355328) + partial_data4 + partial_data5 + partial_data6;
	partial_data2 = (reg_calib_data->par_p4 * partial_data3) / 32;
	partial_data4 = (reg_calib_data->par_p3 * partial_data1) * 4;
	partial_data5 = (reg_calib_data->par_p2 - 16384) * reg_calib_data->t_lin * 2097152;
	sensitivity = ((reg_calib_data->par_p1 - 16384) * 70368744177664) + partial_data2 + partial_data4 + partial_data5;
	partial_data1 = (sensitivity / 16777216) * uncomp_data->pressure;
	partial_data2 = reg_calib_data->par_p10 * reg_calib_data->t_lin;
	partial_data3 = partial_data2 + (65536 * reg_calib_data->par_p9);
	partial_data4 = (partial_data3 * uncomp_data->pressure) / 8192;
	/*dividing by 10 followed by multiplying by 10 to avoid overflow caused by (uncomp_data->pressure * partial_data4) */
	partial_data5 = (uncomp_data->pressure * (partial_data4 / 10)) / 512;
	partial_data5 = partial_data5 * 10;
	partial_data6 = (int64_t)((uint64_t)uncomp_data->pressure * (uint64_t)uncomp_data->pressure);
	partial_data2 = (reg_calib_data->par_p11 * partial_data6) / 65536;
	partial_data3 = (partial_data2 * uncomp_data->pressure) / 128;
	partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
	comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776);

	return comp_press;
}

/*!
 * @brief This internal API is used to compensate the pressure or temperature
 * or both the data according to the component selected by the user.
 */
bool
BMP388::compensate_data(uint8_t sensor_comp,
			const struct bmp3_uncomp_data *uncomp_data,
			struct bmp3_data *comp_data)
{
	int8_t rslt = OK;
	struct bmp3_calib_data calib_data = {0};
	struct bmp3_reg_calib_data *reg_calib_data = &calib_data.reg_calib_data;
	memcpy(reg_calib_data, _cal, 21);

	if ((uncomp_data != NULL) && (comp_data != NULL)) {
		if (sensor_comp & (BMP3_PRESS | BMP3_TEMP)) {
			comp_data->temperature = compensate_temperature(uncomp_data, &calib_data);
		}

		if (sensor_comp & BMP3_PRESS) {
			/* Compensate the pressure data */
			comp_data->pressure = compensate_pressure(uncomp_data, &calib_data);
		}

	} else {
		rslt = -1;
	}

	return (rslt == 0);
}

/*!
 * @brief This API reads the pressure, temperature or both data from the
 * sensor, compensates the data and store it in the bmp3_data structure
 * instance passed by the user.
 */
bool
BMP388::get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data)
{
	bool result = false;
	int8_t rslt;

	/* Array to store the pressure and temperature data read from
	 * the sensor */
	uint8_t reg_data[BMP3_P_T_DATA_LEN] = { 0 };
	struct bmp3_uncomp_data uncomp_data = { 0 };

	/* Read the pressure and temperature data from the sensor */
	rslt = _interface->get_reg_buf(BMP3_DATA_ADDR, reg_data, BMP3_P_T_DATA_LEN);

	if (rslt == OK) {
		/* Parse the read data from the sensor */
		parse_sensor_data(reg_data, &uncomp_data);

		/* Compensate the pressure/temperature/both data read
			* from the sensor */
		result = compensate_data(sensor_comp, &uncomp_data, comp_data);
	}

	return result;
}

int
BMP388::init()
{
	int ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("CDev init failed");
		return ret;
	}

	if (!soft_reset()) {
		PX4_WARN("failed to reset baro during init");
		return -EIO;
	}

	if (_interface->get_reg(BMP3_CHIP_ID_ADDR) != BMP3_CHIP_ID) {
		PX4_WARN("id of your baro is not: 0x%02x", BMP3_CHIP_ID);
		return -EIO;
	}

	_cal = _interface->get_calibration(BMP3_CALIB_DATA_ADDR);

	if (!_cal) {
		PX4_WARN("failed to get baro cal init");
		return -EIO;
	}

	if (!validate_trimming_param()) {
		PX4_WARN("failed to validate trim param");
		return -EIO;
	}

	if (!set_sensor_settings()) {
		PX4_WARN("failed to set sensor settings");
		return -EIO;
	}

	/* do a first measurement cycle to populate reports with valid data */
	if (measure()) {
		return -EIO;
	}

	usleep(_max_measure_interval);

	if (collect()) {
		return -EIO;
	}

	return OK;
}

void
BMP388::start_cycle()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_running = true;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
BMP388::stop_cycle()
{
	_running = false;

	ScheduleClear();
}

void
BMP388::Run()
{
	if (_collect_phase) {
		collect();
		unsigned wait_gap = _report_interval - _max_measure_interval;

		if ((wait_gap != 0) && (_running)) {
			//need to wait some time before new measurement
			ScheduleDelayed(wait_gap);

			return;
		}

	}

	measure();

	if (_running) {
		ScheduleDelayed(_max_measure_interval);
	}
}

int
BMP388::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);

	/* start measure */
	if (!set_op_mode(BMP3_FORCED_MODE)) {
		PX4_WARN("failed to set operating mode");
		perf_count(_comms_errors);
		perf_cancel(_measure_perf);
		return -EIO;
	}

	perf_end(_measure_perf);

	return OK;
}

int
BMP388::collect()
{
	uint8_t sensor_comp = BMP3_PRESS | BMP3_TEMP;

	_collect_phase = false;

	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	/* Variable used to store the compensated data */
	bmp3_data data{};

	if (!get_sensor_data(sensor_comp, &data)) {
		perf_count(_comms_errors);
		perf_cancel(_sample_perf);
		return -EIO;
	}

	_px4_baro.set_error_count(perf_event_count(_comms_errors));

	// Temperature
	_T = (float)(data.temperature / 100.0f);

	// Pressure
	_P = (float)(data.pressure / 100.0f); // to hecto Pascal

	float temperature = _T;
	float pressure = _P / 100.0f; // to mbar

	_px4_baro.set_temperature(temperature);
	_px4_baro.update(timestamp_sample, pressure); // to mbar

	perf_end(_sample_perf);

	return OK;
}

void
BMP388::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u us \n", _report_interval);

	_px4_baro.print_status();
}

/**
 * Local functions in support of the shell command.
 */
namespace bmp388
{

/*
  list of supported bus configurations
 */
struct bmp388_bus_option {
	enum BMP388_BUS busid;
	const char *devpath;
	BMP388_constructor interface_constructor;
	uint8_t busnum;
	uint32_t device;
	bool external;
	BMP388 *dev;
} bus_options[] = {
#if defined(PX4_SPIDEV_EXT_BARO) && defined(PX4_SPI_BUS_EXT)
	{ BMP388_BUS_SPI_EXTERNAL, "/dev/bmp388_spi_ext", &bmp388_spi_interface, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_BARO, true, NULL },
#endif
#if defined(PX4_SPIDEV_BARO)
#  if defined(PX4_SPIDEV_BARO_BUS)
	{ BMP388_BUS_SPI_INTERNAL, "/dev/bmp388_spi_int", &bmp388_spi_interface, PX4_SPIDEV_BARO_BUS, PX4_SPIDEV_BARO, false, NULL },
#  else
	{ BMP388_BUS_SPI_INTERNAL, "/dev/bmp388_spi_int", &bmp388_spi_interface, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_BARO, false, NULL },
#  endif
#endif
#if defined(PX4_I2C_BUS_ONBOARD) && defined(PX4_I2C_OBDEV_BMP388)
	{ BMP388_BUS_I2C_INTERNAL, "/dev/bmp388_i2c_int", &bmp388_i2c_interface, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_BMP388, false, NULL },
#endif
#if defined(PX4_I2C_BUS_ONBOARD) && defined(PX4_I2C_OBDEV1_BMP388)
	{ BMP388_BUS_I2C_INTERNAL1, "/dev/bmp388_i2c_int1", &bmp388_i2c_interface, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV1_BMP388, false, NULL },
#endif
#if defined(PX4_I2C_BUS_EXPANSION) && defined(PX4_I2C_OBDEV_BMP388)
	{ BMP388_BUS_I2C_EXTERNAL, "/dev/bmp388_i2c_ext", &bmp388_i2c_interface, PX4_I2C_BUS_EXPANSION, PX4_I2C_OBDEV_BMP388, true, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

bool	start_bus(struct bmp388_bus_option &bus);
struct bmp388_bus_option &find_bus(enum BMP388_BUS busid);
void	start(enum BMP388_BUS busid);
void	test(enum BMP388_BUS busid);
void	reset(enum BMP388_BUS busid);
void	info();
void	usage();


/**
 * Start the driver.
 */
bool
start_bus(struct bmp388_bus_option &bus)
{
	if (bus.dev != nullptr) {
		PX4_ERR("bus option already started");
		exit(1);
	}

	bmp388::IBMP388 *interface = bus.interface_constructor(bus.busnum, bus.device, bus.external);

	if (interface->init() != OK) {
		delete interface;
		PX4_WARN("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new BMP388(interface, bus.devpath);

	if (bus.dev == nullptr) {
		return false;
	}

	if (OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = nullptr;
		return false;
	}

	int fd = open(bus.devpath, O_RDONLY);

	/* set the poll rate to default, starts automatic data collection */
	if (fd == -1) {
		PX4_ERR("can't open baro device");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		PX4_ERR("failed setting default poll rate");
		exit(1);
	}

	close(fd);
	return true;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum BMP388_BUS busid)
{
	uint8_t i;
	bool started = false;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == BMP388_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != BMP388_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		started |= start_bus(bus_options[i]);
	}

	if (!started) {
		PX4_WARN("bus option number is %d", i);
		PX4_ERR("driver start failed");
		exit(1);
	}

	// one or more drivers started OK
	exit(0);
}


/**
 * find a bus structure for a busid
 */
struct bmp388_bus_option &find_bus(enum BMP388_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == BMP388_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	PX4_ERR("bus %u not started", (unsigned)busid);
	exit(1);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		struct bmp388_bus_option &bus = bus_options[i];

		if (bus.dev != nullptr) {
			PX4_WARN("%s", bus.devpath);
			bus.dev->print_info();
		}
	}

	exit(0);
}

void
usage()
{
	PX4_WARN("missing command: try 'start', 'info'");
	PX4_WARN("options:");
	PX4_WARN("    -X    (external I2C bus TODO)");
	PX4_WARN("    -I    (internal I2C bus TODO)");
	PX4_WARN("    -S    (external SPI bus)");
	PX4_WARN("    -s    (internal SPI bus)");
}

} // namespace

int
bmp388_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	enum BMP388_BUS busid = BMP388_BUS_ALL;

	while ((ch = px4_getopt(argc, argv, "XIJSs", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = BMP388_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = BMP388_BUS_I2C_INTERNAL;
			break;

		case 'J':
			busid = BMP388_BUS_I2C_INTERNAL1;
			break;

		case 'S':
			busid = BMP388_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = BMP388_BUS_SPI_INTERNAL;
			break;

		default:
			bmp388::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		bmp388::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		bmp388::start(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		bmp388::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return -1;
}
