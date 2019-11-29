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
#include <uORB/topics/battery_status.h>

#include <float.h>
#include <px4_platform_common/getopt.h> // Reading inputs from NuttX shell
#include <lib/conversion/rotation.h>

// #include "ads1115.h"

#define ADS1115_BUS_DEFAULT     PX4_I2C_BUS_EXPANSION
#define ADS1115_BASEADDR        (0x48)

#define ADS1115_CONVERSION_INTERVAL     (8000) // 1/Data Rate in microseconds

#define ADS1115_WRITE_ADDR      (0x90) // 1001 000(0)
#define ADS1115_READ_ADDR       (0x91) // 1001 000(1)

#define ADS1115_CONFIG_REG      (0x01)
#define ADS1115_CONVERSION_REG  (0x00)

#define ADS1115_CHANNEL_ONE     (0xC383) // 1 100 001 1 100 0 0 0 11
#define ADS1115_CHANNEL_TWO     (0xD383) // 1 101 001 1 100 0 0 0 11
#define ADS1115_CHANNEL_THREE 	(0xE383) // 1 110 001 1 100 0 0 0 11
#define ADS1115_CHANNEL_FOUR	(0xF383) // 1 111 001 1 100 0 0 0 11

#define voltage_gain            (0.004) //
#define current_gain            (0.025) //

#define swap16(w)       __builtin_bswap16((w))

class ADS1115 : public device::I2C, px4::ScheduledWorkItem
{
public:
	ADS1115(int bus = ADS1115_BUS_DEFAULT, int address = ADS1115_BASEADDR);
	virtual ~ADS1115();

	virtual int init();

	void print_info();

    	void start();

    	void stop();

protected:
	virtual int probe();

private:
	orb_advert_t _power_monitor_topic{nullptr};
    	orb_advert_t _battery_status_topic{nullptr};

	int _measure_interval{0};
    	bool _collect_phase{false};

	int writeConfigReg(uint16_t value);
	int readConfigReg(uint16_t *configRegValue);
	int readConvReg(uint16_t *convRegValue);

	int measureCurrent();
	int measureVoltage();
	int measureWiretemp();
	int measureBatttemp();

	// double getVoltage();
	// double getCurrent();
	// double getWireTemp();
	// double getBattTemp();

    	// double v_avging();
    	// double i_avging();

   	void Run() override;
};

extern "C" __EXPORT int ads1115_main(int argc, char *argv[]);

ADS1115::ADS1115(int bus, int address) : // FILL CONSTRUCTOR
	I2C("ADS1115", nullptr, bus, address, 400000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())) // px4::wq_configurations::hp_default)
{
	PX4_DEBUG("constructor\n");
}

ADS1115::~ADS1115() //FILL DESTRUCTOR
{
    stop();
}

int ADS1115::readConvReg(uint16_t *convRegVal)
{
	uint8_t adrRegPtr = ADS1115_CONVERSION_REG;

	transfer(&adrRegPtr, sizeof(adrRegPtr), nullptr, 0);

    	return transfer(nullptr, 0, (uint8_t *)convRegVal, sizeof(convRegVal));
}

int ADS1115::readConfigReg(uint16_t *configRegVal)
{
    	uint8_t adrRegPtr = ADS1115_CONFIG_REG;

	transfer(&adrRegPtr, sizeof(adrRegPtr), nullptr, 0);

    	return transfer(nullptr, 0, (uint8_t *)configRegVal, sizeof(configRegVal));

	//return PX4_OK;
}

int ADS1115::writeConfigReg(uint16_t value)
{
    	uint8_t data[3] = {ADS1115_CONFIG_REG,
		((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};

	return transfer(data, sizeof(data), nullptr, 0);
}

int ADS1115::measureCurrent()
{

	return writeConfigReg(ADS1115_CHANNEL_ONE);
}

int ADS1115::measureVoltage()
{
	return writeConfigReg(ADS1115_CHANNEL_TWO);
}

int ADS1115::measureWiretemp()
{
	return writeConfigReg(ADS1115_CHANNEL_THREE);
}

int ADS1115::measureBatttemp()
{
	return writeConfigReg(ADS1115_CHANNEL_FOUR);
}

void ADS1115::Run()
{
	static int measure_count, read_count;
	uint16_t convRegVal = 0;
	static double current = 0;
	static double voltage = 0;
	static double wiretemp = 0;
	static double batttemp = 0;
	static double discharged = 0;

	struct power_monitor_s report;
	struct battery_status_s mainrep;

	if (_measure_interval == 0) {
		return;
	}

	if (_collect_phase)
	{
		switch(read_count)
		{
			case 0:
				if (!readConvReg(&convRegVal))
				{
					current = swap16(convRegVal);
                    			discharged += 0.001;
					read_count = 1;
					break;
				} else
				{
					PX4_ERR("current collect error");
					break;
				}

			case 1:
				if (!readConvReg(&convRegVal))
				{
					voltage = swap16(convRegVal);
					read_count = 2;
					break;
				} else
				{
					PX4_ERR("voltage collect error");
					break;
				}

			case 2:
				if (!readConvReg(&convRegVal))
				{
					wiretemp++;//swap16(convRegVal);
					read_count = 3;
					break;
				} else
				{
					PX4_ERR("wire temp collect error");
					break;
				}

			case 3:
				if (!readConvReg(&convRegVal))
				{
					batttemp++;//swap16(convRegVal);
					read_count = 0;
					break;
				} else
				{
					PX4_ERR("batt temp collect error");
					break;
				}
		}
		_collect_phase = false;

		if (_measure_interval > ADS1115_CONVERSION_INTERVAL)
		{
			ScheduleDelayed(_measure_interval - ADS1115_CONVERSION_INTERVAL);
			return;
		}
	}

	switch(measure_count)
	{
		case 0:
			if (!measureCurrent())
			{
				measure_count = 1;
				break;
			} else
			{
				PX4_ERR("current measure error");
				break;
			}
		case 1:
			if (!measureVoltage())
			{
				measure_count = 2;
				break;
			} else
			{
				PX4_ERR("voltage measure error");
				break;
			}
		case 2:
			if (!measureWiretemp())
			{
				measure_count = 3;
				break;
			} else
			{
				PX4_ERR("wire temp measure error");
				break;
			}

		case 3:
			if (!measureBatttemp())
			{
				measure_count = 0;
				break;
			} else
			{
				PX4_ERR("batt temp measure error");
				break;
			}
	}

	_collect_phase = true;

	if (_measure_interval > 0)
	{
		ScheduleDelayed(ADS1115_CONVERSION_INTERVAL);
	}

	report.timestamp = hrt_absolute_time();
	report.current_a = current*current_gain;
	report.voltage_v = voltage*voltage_gain;
	report.wiretemp_c = wiretemp;
	report.batttemp_c = batttemp;

	int pminstance;
	orb_publish_auto(ORB_ID(power_monitor), &_power_monitor_topic,
        	&report, &pminstance, ORB_PRIO_DEFAULT);

	mainrep.timestamp = hrt_absolute_time();
	mainrep.current_a = current*current_gain;
	mainrep.voltage_v = voltage*voltage_gain;
	mainrep.voltage_filtered_v = voltage*voltage_gain;
	mainrep.discharged_mah = discharged;

	int bsinstance;
	orb_publish_auto(ORB_ID(battery_status), &_battery_status_topic,
	        &mainrep, &bsinstance, ORB_PRIO_DEFAULT);
}

void ADS1115::start()
{
	ScheduleClear();

	_measure_interval = ADS1115_CONVERSION_INTERVAL;

	ScheduleNow();
}

void ADS1115::stop()
{
	if (_measure_interval > 0)
	{
		_measure_interval = 0;
	}
	ScheduleClear();
}

int ADS1115::probe()
{
	uint16_t configRegVal = 0;
	uint16_t convRegVal = 0;

	writeConfigReg(ADS1115_CHANNEL_ONE);

	if (!readConfigReg(&configRegVal))
	{
		if (0x8343 != configRegVal)
		{
			PX4_WARN("ADS1115::readConfigReg() failed");
			return PX4_ERROR;
		} else {
			return PX4_OK;
		}
	}

	if (!readConvReg(&convRegVal))
	{
		PX4_INFO("%u", convRegVal);
	}

	PX4_INFO("ADS1115::probe() success");
	return PX4_OK;
}

int ADS1115::init()
{
	if (I2C::init() != PX4_OK)
	{
		PX4_WARN("I2C::init() failed, in ADS1115::init() if.1");
		return PX4_ERROR;
	}
	PX4_INFO("ADS1115::init() success");
	start();
	return PX4_OK;
}

void ADS1115::print_info()
{
	PX4_INFO("Driver information: ADS1115 Power Monitor Breakout Board");
}

/* ------------------------------------------------------------------------- */

namespace ads1115
{
    ADS1115 *g_dev;

    int sh_init(int i2c_bus);
    int sh_stop();
    int sh_info();

    int sh_init(int i2c_bus)
    {
        if (g_dev != nullptr)
        {
            PX4_ERR("ADS1115 Driver already started, g_dev != nullptr in sh_init if.1");
            return PX4_ERROR;
        }

        g_dev = new ADS1115(i2c_bus);

        if (g_dev == nullptr)
        {
            PX4_ERR("ADS1115 Driver not started, g_dev = nullptr in sh_init if.2");
            return PX4_ERROR;
        }

        if (OK != g_dev->init())
        {
            PX4_ERR("ADS1115::init() failed in sh_init if.3");
            goto fail;
        }

        return PX4_OK;

        fail:
            if (g_dev != nullptr)
            {
                delete g_dev;
                g_dev = nullptr;
                return PX4_ERROR;
            }
        return PX4_ERROR;
    }

    int sh_stop()
	{
		if (g_dev != nullptr)
		{
			g_dev->stop();
            PX4_INFO("ADS1115 Driver sh_stop() success");
			// delete g_dev;
			g_dev = nullptr;
			return PX4_OK;
		} else
		{
			PX4_ERR("ADS1115 Driver not running yet");
			return PX4_ERROR;
		}
	}

	int sh_info()
	{
		if (g_dev == nullptr)
		{
			PX4_ERR("ADS1115 Driver not started, g_dev == nullptr in sh_info");
			return PX4_ERROR;
		}

		PX4_INFO("state @ %p\n", g_dev);
		g_dev->print_info();
		return PX4_OK;
	}
}

static void ads1115_usage()
{
    PX4_INFO("ads1115 [command]");
    PX4_INFO("commands:");
	PX4_INFO("\tinit|stop|info");
}

int ads1115_main(int argc, char *argv[])
{
    int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
    int i2c_bus = ADS1115_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "s:R:", &myoptind, &myoptarg)) != EOF)
    {
        switch(ch)
        {
            case 's':
                i2c_bus = atoi(myoptarg);
                break;
            default:
                PX4_WARN("unknown option!");
                goto fail;
        }

        if (!strcmp(argv[myoptind], "init"))
        {
            return ads1115::sh_init(i2c_bus);
        }

        if (!strcmp(argv[myoptind], "stop"))
        {
            return ads1115::sh_stop();
        }

	    if (!strcmp(argv[myoptind], "info"))
        {
            ads1115_usage();
	        return ads1115::sh_info();
        }

        fail:
            ads1115_usage();
            return PX4_ERROR;
    }
    return PX4_OK;
}
