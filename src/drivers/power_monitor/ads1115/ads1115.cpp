/**
 * @file ads1115.cpp
 *
 * Driver for the ADS1115-based power monitor board for Leap Aeronautics.
 *
 * TO-DO-
 *
 * 1) Finish constructor: Add param checks, and other first run initializations
 * 2) Finish Nuttx Shell functions: start(), start_bus(), stop(), info() etc
 * 3) Fix uORB publication process
 * 4) Add computation functions: Reverse 2's compliment, Averaging, Steinhart
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <nuttx/clock.h>

#include <board_config.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>
#include <uORB/topics/power_monitor.h> // power_monitor struct msg

#include <float.h>
#include <px4_platform_common/getopt.h> // Reading inputs from NuttX shell
#include <lib/conversion/rotation.h>

// #include "ads1115.h"

#define ADS1115_BUS_DEFAULT	PX4_I2C_BUS_EXPANSION
#define ADS1115_BASEADDR 	(0x48)

#define ADS1115_CONVERSION_INTERVAL	(0.004f) // 1/Data Rate

#define ADS1115_WRITE_ADDR	(0x90) // 1001 000(0)
#define ADS1115_READ_ADDR	(0x91) // 1001 000(1)

#define ADS1115_CONFIG_REG	(0x01)
#define ADS1115_CONVERSION_REG 	(0x00)

#define ADS1115_CHANNEL_ONE	(0x42A3) // 0 100 001 0 101 0 0 0 11
#define ADS1115_CHANNEL_TWO	(0x52A3) // 0 101 001 0 101 0 0 0 11
#define ADS1115_CHANNEL_THREE 	(0x62A3) // 0 110 001 0 101 0 0 0 11
#define ADS1115_CHANNEL_FOUR	(0x72A3) // 0 111 001 0 101 0 0 0 11


class ADS1115 : public device::I2C, px4::ScheduledWorkItem
{
public:
	ADS1115(int bus = ADS1115_BUS_DEFAULT, int address = ADS1115_BASEADDR);
	virtual  ~ADS1115();

	virtual int  init();

	void  print_info();

protected:
	virtual int  probe();

private:
	orb_advert_t _power_monitor_topic{nullptr};

	float _measure_interval{ADS1115_CONVERSION_INTERVAL};

	bool writeToConfigRegister(uint16_t value);
	bool readFromConfigRegister(uint32_t *configRegValue);
	bool readFromConversionRegister(uint16_t *conversionRegValue);

	double getVoltage();
	double getCurrent();
	double getWireTemp();
	double getBattTemp();

	void start();
	void stop();
	void Run() override;
};

extern "C" __EXPORT int ads1115_main(int argc, char *argv[]);

ADS1115::ADS1115(int bus, int address) :
	I2C("ADS1115", nullptr, bus, address, 100000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id()))
{
	PX4_DEBUG("constructor\n");
}

ADS1115::~ADS1115()
{
	stop();
}

bool ADS1115::writeToConfigRegister(uint16_t value)
{
	PX4_INFO("ADS1115 write to config in progress");
	uint8_t data[3] = {ADS1115_CONFIG_REG,
		((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

bool ADS1115::readFromConfigRegister(uint32_t *configRegValue) // add a print for the output obtained from final transfer and change fn arg to uint32_t and check for hard reset error
{
	// write adrptr reg operation : involves a write process only
	PX4_INFO("ADS1115 set ptr to config in progress");
	uint8_t setptr = ADS1115_CONFIG_REG;
	transfer(&setptr, sizeof(setptr), nullptr, 0);
	// read config reg operation : involves a write and read process
	// PX4_INFO("ADS1115 send read in progress");
	// uint8_t address = ADS1115_READ_ADDR;
	// transfer(&address, sizeof(address), nullptr, 0);
	PX4_INFO("ADS1115 read from config reg in progress");
	transfer(nullptr, 0, (uint8_t *)configRegValue, sizeof(configRegValue));
	if (*configRegValue == 0x42C2FFFF)
	{
		PX4_INFO("configRegisterValue obtained");
	}
	// PX4_INFO("value sent %d of %d size", (unsigned int)configRegValue, sizeof(configRegValue));
	return PX4_OK;
}

bool ADS1115::readFromConversionRegister(uint16_t *conversionRegValue)
{
	PX4_INFO("ADS1115 read from conv in progress");
	uint8_t address = ADS1115_READ_ADDR;
	return transfer(&address, sizeof(address), (uint8_t *)conversionRegValue, sizeof(conversionRegValue));
}

double ADS1115::getVoltage()
{
	double voltage = 0;

	writeToConfigRegister(ADS1115_CHANNEL_ONE);

	// Read from conversion register
	uint16_t conversionRegValue = 0;
	if (readFromConversionRegister(&conversionRegValue))
	{
		// Perform reverse 2's compliment, perform binary to decimal conversion
		// other scaling etc., math fns as per circuit
		// double voltage = final value
		voltage = 10;
	}
	else
	{
		PX4_DEBUG("Reading from Voltage Conversion Register failed");
		voltage = 0.0;
	}

	ScheduleDelayed(ADS1115_CONVERSION_INTERVAL);

	return voltage;
}

double ADS1115::getCurrent()
{
	double current = 0;

	writeToConfigRegister(ADS1115_CHANNEL_TWO);

	// Read from conversion register
	uint16_t conversionRegValue = 0;
	if (readFromConversionRegister(&conversionRegValue))
	{
		// Perform reverse 2's compliment, perform binary to decimal conversion
		// other scaling etc., math fns as per circuit
		// double current = final value
		current = 10;
	}
	else
	{
		PX4_DEBUG("Reading from Current Conversion Register failed");
		current = 0.0;
	}

	ScheduleDelayed(ADS1115_CONVERSION_INTERVAL);

	return current;
}

double ADS1115::getWireTemp()
{
	double wireTemp = 0;

	writeToConfigRegister(ADS1115_CHANNEL_THREE);

	// Read from conversion register
	uint16_t conversionRegValue = 0;
	if (readFromConversionRegister(&conversionRegValue))
	{
		// Perform reverse 2's compliment, perform binary to decimal conversion
		// other scaling etc., math fns as per circuit
		// double wireTemp = final value
		wireTemp = 10;
	}
	else
	{
		PX4_DEBUG("Reading from Wire Temperature Conversion Register failed");
		wireTemp = 0.0;
	}

	ScheduleDelayed(ADS1115_CONVERSION_INTERVAL);

	return wireTemp;
}

double ADS1115::getBattTemp()
{
	double battTemp = 0;
	writeToConfigRegister(ADS1115_CHANNEL_FOUR);

	// Read from conversion register
	uint16_t conversionRegValue = 0;
	if (readFromConversionRegister(&conversionRegValue))
	{
		// Perform reverse 2's compliment, perform binary to decimal conversion
		// other scaling etc., math fns as per circuit
		// double battTemp = final value
		battTemp = 10;
	}
	else
	{
		PX4_DEBUG("Reading from Battery Temperature Conversion Register failed");
		battTemp = 0.0;
	}

	ScheduleDelayed(ADS1115_CONVERSION_INTERVAL);

	return battTemp;
}

void ADS1115::start()
{
	ScheduleClear();

	PX4_DEBUG("start done.\n");

	/* schedule a cycle to start things */
	ScheduleDelayed(5);
}

void ADS1115::stop()
{
	ScheduleClear();
}

void ADS1115::Run()
{
	struct power_monitor_s report;
	report.timestamp = hrt_absolute_time();
	report.voltage_v = getVoltage();
	report.current_a = getCurrent();
	report.wiretemp_c = getWireTemp();
	report.batttemp_c = getBattTemp();

	int instance;
	orb_publish_auto(ORB_ID(power_monitor), &_power_monitor_topic, &report, &instance, ORB_PRIO_DEFAULT);

	ScheduleDelayed(1000);
}

int ADS1115::init()
{
	int ret = PX4_ERROR;
	PX4_INFO("init() begun");

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
	{
		PX4_INFO("i2c init() failed");
		return ret;
	}

	if (probe() == PX4_ERROR)
	{
		PX4_INFO("probe() failed");
		return ret;
	}

	//set_device_address(ADS1115_BASEADDR);	/* set I2c Address */

	PX4_INFO("init done.\n");

	start();

	return OK;
}

int  ADS1115::probe()
{
	uint32_t configRegisterValue = 0;

	PX4_INFO("ADS1115 probe attempt");

	writeToConfigRegister(ADS1115_CHANNEL_ONE);

	if (readFromConfigRegister(&configRegisterValue))
	{
		PX4_INFO("ADS1115 probe reading");
		if (ADS1115_CHANNEL_ONE != configRegisterValue)
		{
			PX4_INFO("ADS1115 probe failed");
			return PX4_ERROR;
		}
		else
		{
			PX4_INFO("ADS1115 probe success");
			return OK;
		}
	}

	return PX4_ERROR;
}

void ADS1115::print_info()
{
	printf("ADS1115 Power monitor");
}

/*-----------------------------------------------------------------------*/

namespace ads1115
{
	ADS1115	*g_dev;

	int sh_start();
	int sh_start_bus(int i2c_bus);
	int sh_stop();
	int info();

	int sh_start()
	{
		if (g_dev != nullptr)
		{
			PX4_ERR("Driver already started");
			return PX4_ERROR;
		} else
		{
			PX4_INFO("Driver started");
			return OK;
		}
	}

	int sh_start_bus(int i2c_bus)
	{
		if (g_dev != nullptr)
		{
			PX4_ERR("Driver already started");
			return PX4_ERROR;
		}

		/* create the driver */
		g_dev = new ADS1115(i2c_bus);

		if (g_dev == nullptr)
		{
			goto faila;
		}

		if (OK != g_dev->init())
		{
			goto failb;
		}

		PX4_INFO("start_bus() success");
		return PX4_OK;

	faila:
		if (g_dev != nullptr)
		{
			delete g_dev;
			g_dev = nullptr;
			PX4_INFO("start_bus() failed because g_dev");
		}
	failb:
		if (g_dev != nullptr)
		{
			delete g_dev;
			g_dev = nullptr;
			PX4_INFO("start_bus() failed because init()");
		}
		return PX4_ERROR;
	}

	int sh_stop()
	{
		if (g_dev != nullptr)
		{
			PX4_INFO("stop() success");
			delete g_dev;
			g_dev = nullptr;
			return PX4_OK;
		} else
		{
			PX4_ERR("Driver not running yet");
			return PX4_ERROR;
		}
	}

	int sh_info()
	{
		if (g_dev == nullptr)
		{
			PX4_INFO("driver poll restart failed");
			return PX4_ERROR;
		}

		printf("state @ %p\n", g_dev);
		g_dev->print_info();
		return PX4_OK;
	}
}

static void ads1115_usage()
{
	PX4_INFO("usage: ads1115 command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", ADS1115_BUS_DEFAULT);
	PX4_INFO("\t-a --all");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|test|info");
}


int ads1115_main(int argc, char *argv[])
{
	// printf("hello world");
	// return OK;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	bool start_all = false;

	int i2c_bus = ADS1115_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "ab:R:", &myoptind, &myoptarg)) != EOF)
	{
		switch (ch)
		{
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

	if (!strcmp(argv[myoptind], "start"))
	{
		if (start_all)
		{
			return ads1115::sh_start();

		}
		else
		{
			return ads1115::sh_start_bus(i2c_bus);
		}
	}
	out_error:
		ads1115_usage();
		return PX4_ERROR;
	}
	return OK;
}
