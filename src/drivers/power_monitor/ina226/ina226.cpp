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
 * @file ina226.cpp
 * @author David Sidrane <david_s5@usa.net>
 *
 * Driver for the I2C attached INA226
 */
#define INA226_RAW // remove this

#include <string.h>

#include <px4_config.h>
#include <px4_getopt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/power_monitor.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

/* Configuration Constants */
#define INA226_BUS_DEFAULT		                PX4_I2C_BUS_EXPANSION
#define INA226_BASEADDR 	                    0x41 /* 7-bit address. 8-bit address is 0x41 */

/* INA226 Registers addresses */
#define INA226_REG_CONFIGURATION             (0x00)
#define INA226_REG_SHUNTVOLTAGE              (0x01)
#define INA226_REG_BUSVOLTAGE                (0x02)
#define INA226_REG_POWER                     (0x03)
#define INA226_REG_CURRENT                   (0x04)
#define INA226_REG_CALIBRATION               (0x05)
#define INA226_REG_MASKENABLE                (0x06)
#define INA226_REG_ALERTLIMIT                (0x07)
#define INA226_MFG_ID                        (0xfe)
#define INA226_MFG_DIEID                     (0xff)

#define INA226_MFG_ID_TI                     (0x5449) // TI
#define INA226_MFG_DIE                       (0x2260) // INA2260

/* INA226 Configuration Register */
#define INA226_MODE_SHIFTS                   (0)
#define INA226_MODE_MASK                     (7 << INA226_MODE_SHIFTS)
#define INA226_MODE_SHUTDOWN                 (0 << INA226_MODE_SHIFTS)
#define INA226_MODE_SHUNT_TRIG               (1 << INA226_MODE_SHIFTS)
#define INA226_MODE_BUS_TRIG                 (2 << INA226_MODE_SHIFTS)
#define INA226_MODE_SHUNT_BUS_TRIG           (3 << INA226_MODE_SHIFTS)
#define INA226_MODE_ADC_OFF                  (4 << INA226_MODE_SHIFTS)
#define INA226_MODE_SHUNT_CONT               (5 << INA226_MODE_SHIFTS)
#define INA226_MODE_BUS_CONT                 (6 << INA226_MODE_SHIFTS)
#define INA226_MODE_SHUNT_BUS_CONT           (7 << INA226_MODE_SHIFTS)

#define INA226_VSHCT_SHIFTS                  (3)
#define INA226_VSHCT_MASK                    (7 << INA226_VSHCT_SHIFTS)
#define INA226_VSHCT_140US                   (0 << INA226_VSHCT_SHIFTS)
#define INA226_VSHCT_204US                   (1 << INA226_VSHCT_SHIFTS)
#define INA226_VSHCT_332US                   (2 << INA226_VSHCT_SHIFTS)
#define INA226_VSHCT_588US                   (3 << INA226_VSHCT_SHIFTS)
#define INA226_VSHCT_1100US                  (4 << INA226_VSHCT_SHIFTS)
#define INA226_VSHCT_2116US                  (5 << INA226_VSHCT_SHIFTS)
#define INA226_VSHCT_4156US                  (6 << INA226_VSHCT_SHIFTS)
#define INA226_VSHCT_8244US                  (7 << INA226_VSHCT_SHIFTS)

#define INA226_VBUSCT_SHIFTS                 (6)
#define INA226_VBUSCT_MASK                   (7 << INA226_VBUSCT_SHIFTS)
#define INA226_VBUSCT_140US                  (0 << INA226_VBUSCT_SHIFTS)
#define INA226_VBUSCT_204US                  (1 << INA226_VBUSCT_SHIFTS)
#define INA226_VBUSCT_332US                  (2 << INA226_VBUSCT_SHIFTS)
#define INA226_VBUSCT_588US                  (3 << INA226_VBUSCT_SHIFTS)
#define INA226_VBUSCT_1100US                 (4 << INA226_VBUSCT_SHIFTS)
#define INA226_VBUSCT_2116US                 (5 << INA226_VBUSCT_SHIFTS)
#define INA226_VBUSCT_4156US                 (6 << INA226_VBUSCT_SHIFTS)
#define INA226_VBUSCT_8244US                 (7 << INA226_VBUSCT_SHIFTS)

#define INA226_AVERAGES_SHIFTS                (9)
#define INA226_AVERAGES_MASK                  (7 << INA226_AVERAGES_SHIFTS)
#define INA226_AVERAGES_1                     (0 << INA226_AVERAGES_SHIFTS)
#define INA226_AVERAGES_4                     (1 << INA226_AVERAGES_SHIFTS)
#define INA226_AVERAGES_16                    (2 << INA226_AVERAGES_SHIFTS)
#define INA226_AVERAGES_64                    (3 << INA226_AVERAGES_SHIFTS)
#define INA226_AVERAGES_128                   (4 << INA226_AVERAGES_SHIFTS)
#define INA226_AVERAGES_256                   (5 << INA226_AVERAGES_SHIFTS)
#define INA226_AVERAGES_512                   (6 << INA226_AVERAGES_SHIFTS)
#define INA226_AVERAGES_1024                  (7 << INA226_AVERAGES_SHIFTS)

#define INA226_CONFIG (INA226_MODE_SHUNT_BUS_CONT | INA226_VSHCT_588US | INA226_VBUSCT_588US | INA226_AVERAGES_64)

#define INA226_RST                            (1 << 15)

/* INA226 Enable / Mask Register */

#define INA226_LEN                           (1 << 0)
#define INA226_APOL                          (1 << 1)
#define INA226_OVF                           (1 << 2)
#define INA226_CVRF                          (1 << 3)
#define INA226_AFF                           (1 << 4)

#define INA226_CNVR                          (1 << 10)
#define INA226_POL                           (1 << 11)
#define INA226_BUL                           (1 << 12)
#define INA226_BOL                           (1 << 13)
#define INA226_SUL                           (1 << 14)
#define INA226_SOL                           (1 << 15)

#define INA226_CONVERSION_INTERVAL 	          (100000-7) /* 100 ms / 10 Hz */
#define MAX_CURRENT                           164.0f    /* 164 Amps */
#define DN_MAX                                32768.0f  /* 2^15 */
#define INA226_CONST                          0.00512f  /* is an internal fixed value used to ensure scaling is maintained properly  */
#define INA226_SHUNT                          0.0005f   /* Shunt is 500 uOhm */
#define INA226_VSCALE                         0.00125f  /* LSB of voltage is 1.25 mV  */

#define swap16(w)                       __builtin_bswap16((w))

class INA226 : public device::I2C, px4::ScheduledWorkItem
{
public:
	INA226(int bus = INA226_BUS_DEFAULT, int address = INA226_BASEADDR);
	virtual ~INA226();

	virtual int 		  init();

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				      print_info();

protected:
	virtual int	  		probe();

private:
	bool			        _sensor_ok{false};
	int				        _measure_interval{0};
	bool			        _collect_phase{false};

	orb_advert_t		  _power_monitor_topic{nullptr};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	int16_t           _bus_volatage{0};
	int16_t           _power{-1};
	int16_t           _current{-1};
	int16_t           _shunt{0};
	int16_t           _cal{0};
	bool              _mode_trigged{false};

	float             _max_current{MAX_CURRENT};
	float             _rshunt{INA226_SHUNT};
	uint16_t          _config{INA226_CONFIG};
	float             _current_lsb{_max_current / DN_MAX};
	float             _power_lsb{25.0f * _current_lsb};

	/**
	* Test whetpower_monitorhe device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to read or write.
	* @return			.
	*/
	int               read(uint8_t address);
	int               write(uint8_t address, uint16_t data);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				      start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				      stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				     Run() override;

	int					     measure();
	int					     collect();

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ina226_main(int argc, char *argv[]);

INA226::INA226(int bus, int address) :
	I2C("INA226", nullptr, bus, address, 100000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_sample_perf(perf_alloc(PC_ELAPSED, "ina226_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ina226_com_err"))
{
	float fvalue = MAX_CURRENT;
	_max_current = fvalue;
	param_t ph = param_find("INA226_CURRENT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_max_current = fvalue;
	}

	fvalue = INA226_SHUNT;
	_rshunt = fvalue;
	ph = param_find("INA226_SHUNT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_rshunt = fvalue;
	}

	ph = param_find("INA226_CONFIG");
	int32_t value = INA226_CONFIG;
	_config = (uint16_t)value;

	if (ph != PARAM_INVALID && param_get(ph, &value) == PX4_OK) {
		_config = (uint16_t)value;
	}

	_mode_trigged = ((_config & INA226_MODE_MASK) >> INA226_MODE_SHIFTS) <=
			((INA226_MODE_SHUNT_BUS_TRIG & INA226_MODE_MASK) >>
			 INA226_MODE_SHIFTS);

	_current_lsb = _max_current / DN_MAX;
	_power_lsb = 25 * _current_lsb;
}

INA226::~INA226()
{
	/* make sure we are truly inactive */
	stop();

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int INA226::read(uint8_t address)
{
	union {
		uint16_t reg;
		uint8_t  b[2] = {};
	} data;

	int ret = transfer(&address, 1, &data.b[0], sizeof(data.b));

	if (OK != ret) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return -1;
	}

	return swap16(data.reg);
}

int INA226::write(uint8_t address, uint16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

int
INA226::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	write(INA226_REG_CONFIGURATION, INA226_RST);

	_cal = INA226_CONST / (_current_lsb * INA226_SHUNT);

	if (write(INA226_REG_CALIBRATION, _cal) < 0) {
		return -3;
	}

	// If we run in continuous mode then start it here

	if (!_mode_trigged) {
		ret = write(INA226_REG_CONFIGURATION, _config);

	} else {
		ret = OK;
	}

	set_device_address(INA226_BASEADDR);	/* set I2c Address */

	start();
	_sensor_ok = true;

	return ret;
}

int
INA226::probe()
{
	int value = read(INA226_MFG_ID);

	if (value < 0) {
		perf_count(_comms_errors);
	}

	if (value != INA226_MFG_ID_TI) {
		PX4_DEBUG("probe mfgid %d", value);
		return -1;
	}

	value = read(INA226_MFG_DIEID);

	if (value < 0) {
		perf_count(_comms_errors);
	}

	if (value != INA226_MFG_DIE) {
		PX4_DEBUG("probe die id %d", value);
		return -1;
	}

	return OK;
}

int
INA226::measure()
{
	int ret = OK;

	if (_mode_trigged) {
		ret = write(INA226_REG_CONFIGURATION, _config);

		if (ret < 0) {
			perf_count(_comms_errors);
			PX4_DEBUG("i2c::transfer returned %d", ret);
		}
	}

	return ret;
}

int
INA226::collect()
{
	int ret = -EIO;

	/* read from the sensor */
	perf_begin(_sample_perf);

	ret = _bus_volatage = read(INA226_REG_BUSVOLTAGE);

	if (_bus_volatage >= 0) {
		ret = _power = read(INA226_REG_POWER);

		if (_power >= 0) {
			ret = _current = read(INA226_REG_CURRENT);

			if (_current >= 0) {
				ret = _shunt = read(INA226_REG_SHUNTVOLTAGE);

				if (_shunt >= 0) {

					struct power_monitor_s report;
					report.timestamp = hrt_absolute_time();
					report.voltage_v = (float) _bus_volatage * INA226_VSCALE;
					report.current_a = (float) _current * _current_lsb;
					report.power_w   = (float) _power * _power_lsb;
#if defined(INA226_RAW)
					report.rconf  = read(INA226_REG_CONFIGURATION);
					report.rsv    = read(INA226_REG_SHUNTVOLTAGE);
					report.rbv    = read(INA226_REG_BUSVOLTAGE);
					report.rp     = read(INA226_REG_POWER);
					report.rc     = read(INA226_REG_CURRENT);
					report.rcal   = read(INA226_REG_CALIBRATION);
					report.me     = read(INA226_REG_MASKENABLE);
					report.al     = read(INA226_REG_ALERTLIMIT);
#endif

					/* publish it */
					int instance;
					orb_publish_auto(ORB_ID(power_monitor), &_power_monitor_topic, &report, &instance, ORB_PRIO_DEFAULT);

					ret = OK;
					perf_end(_sample_perf);
					return ret;
				}
			}
		}
	}

	PX4_DEBUG("error reading from sensor: %d", ret);
	perf_count(_comms_errors);
	perf_end(_sample_perf);
	return ret;
}

void
INA226::start()
{
	ScheduleClear();

	/* reset the report ring and state machine */
	_collect_phase = false;

	_measure_interval = INA226_CONVERSION_INTERVAL;

	/* schedule a cycle to start things */
	ScheduleDelayed(5);
}

void
INA226::stop()
{
	ScheduleClear();
}

void
INA226::Run()
{
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			/* if error restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = !_mode_trigged;

		if (_measure_interval > INA226_CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - INA226_CONVERSION_INTERVAL);
			return;
		}
	}

	/* Measurement  phase */

	/* Perform measurement */
	if (OK != measure()) {
		PX4_DEBUG("measure error ina226");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(INA226_CONVERSION_INTERVAL);
}

void
INA226::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	printf("poll interval:  %u \n", _measure_interval);
}

/**
 * Local functions in support of the shell command.
 */
namespace ina226
{

INA226	*g_dev;

int 	start();
int 	start_bus(int i2c_bus);
int 	stop();
int 	info();

/**
 *
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 *
 */
int
start()
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
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
		return PX4_ERROR;
	}

	/* create the driver */
	g_dev = new INA226(i2c_bus);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	return PX4_OK;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

/**
 * Stop the driver
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

} /* namespace */


static void
ina2262_usage()
{
	PX4_INFO("usage: ina226 command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", INA226_BUS_DEFAULT);
	PX4_INFO("\t-a --all");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|test|info");
}

int
ina226_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	bool start_all = false;

	int i2c_bus = INA226_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "ab:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 'a':
			start_all = true;
			break;

		default:
			PX4_WARN("Unknown option!");
			goto out_error;
		}
	}

	if (myoptind >= argc) {
		goto out_error;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return ina226::start();

		} else {
			return ina226::start_bus(i2c_bus);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return ina226::stop();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return ina226::info();
	}

out_error:
	ina2262_usage();
	return PX4_ERROR;
}
