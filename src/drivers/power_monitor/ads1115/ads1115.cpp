/**
 * @file ads1115.cpp
 *
 * Driver for the ADS1115-based power monitor board for Leap Aeronautics.
 *
 * TO-DO-
 *
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
#include <uORB/topics/battery_status.h>

#include <float.h>
#include <px4_platform_common/getopt.h> // Reading inputs from NuttX shell
#include <lib/conversion/rotation.h>

// #include "ads1115.h"

#define ADS1115_BUS_DEFAULT     PX4_I2C_BUS_EXPANSION
#define ADS1115_BASEADDR        (0x48)

#define ADS1115_CONVERSION_INTERVAL     (1200) // 1/Data Rate in microseconds

#define ADS1115_WRITE_ADDR      (0x90) // 1001 000(0)
#define ADS1115_READ_ADDR       (0x91) // 1001 000(1)

#define ADS1115_CONFIG_REG      (0x01)
#define ADS1115_CONVERSION_REG  (0x00)

#define ADS1115_CHANNEL_ONE     (0xC3E3) // 1 100 001 1 111 0 0 0 11
#define ADS1115_CHANNEL_TWO     (0xD3E3) // 1 101 001 1 111 0 0 0 11
#define ADS1115_CHANNEL_THREE 	(0xE3E3) // 1 110 001 1 111 0 0 0 11
#define ADS1115_CHANNEL_FOUR	(0xF3E3) // 1 111 001 1 111 0 0 0 11

#define batt_capacity		    (22000)
#define voltage_gain            (0.004f) //
#define voltage_offset		    (0.370f)
#define current_gain            (0.025f) //
#define current_offset		    (2.275f)

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

    struct power_monitor_s report;
    struct battery_status_s mainrep;

    hrt_abstime _now_timestamp = 0;
	hrt_abstime _last_timestamp = 0;

    bool batt_connected{false};

    float current = 0;
    float voltage = 0;
    float wiretemp = 0;
    float batttemp = 0;
    float discharged = 0;

	int writeConfigReg(uint16_t value);
	int readConfigReg(uint16_t *configRegValue);
	int readConvReg(uint16_t *convRegValue);

	float getVoltage();
	float getCurrent();
	double getWireTemp();
	double getBattTemp();
    float getResistance(double temp_val);
    float getSteinhart(float reisistance);

    void Run() override;
};

extern "C" __EXPORT int ads1115_main(int argc, char *argv[]);

ADS1115::ADS1115(int bus, int address) :
	I2C("ADS1115", nullptr, bus, address, 400000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())) // px4::wq_configurations::hp_default)
{
	PX4_DEBUG("constructor\n");
}

ADS1115::~ADS1115()
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
}

int ADS1115::writeConfigReg(uint16_t value)
{
    uint8_t data[3] = {ADS1115_CONFIG_REG,
		((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};

    return transfer(data, sizeof(data), nullptr, 0);
}

int ADS1115::probe()
{
    uint16_t configRegVal = 0;
    uint16_t convRegVal = 0;

    writeConfigReg(ADS1115_CHANNEL_ONE);

    if (!readConfigReg(&configRegVal))
    {
        if (0xE343 != configRegVal)
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

float ADS1115::getCurrent()
{
    float current_raw = 0;
    float sum_i = 0;
    float average_i = 0;
    uint16_t convRegVal = 0;

    writeConfigReg(ADS1115_CHANNEL_ONE);

    usleep(ADS1115_CONVERSION_INTERVAL);

    for(int i = 0; i < 5; i++)
    {
        if (!readConvReg(&convRegVal))
        {
            current_raw = swap16(convRegVal);
            sum_i += current_raw;
        } else
        {
            PX4_ERR("Current Reading Failed");
        }
        usleep(ADS1115_CONVERSION_INTERVAL);
    }
    average_i = sum_i / 5;
    return average_i;
}

float ADS1115::getVoltage()
{
    float voltage_raw = 0;
    float sum_v = 0;
    float average_v = 0;
    uint16_t convRegVal = 0;

    writeConfigReg(ADS1115_CHANNEL_TWO);

    usleep(ADS1115_CONVERSION_INTERVAL);

    for (int v = 0; v < 5; v++)
    {
        if (!readConvReg(&convRegVal))
        {
            voltage_raw = swap16(convRegVal);
            sum_v += voltage_raw;
        } else
        {
            PX4_ERR("Voltage Reading Failed");
        }
        usleep(ADS1115_CONVERSION_INTERVAL);
    }
    average_v = sum_v / 5;
    return average_v;
}

float ADS1115::getResistance(double temp_val)
{
    float adc_value = temp_val * (8.192/5);
    float resistance = 10000 / (65535 / adc_value - 1);
    return resistance;
}

float ADS1115::getSteinhart(float resistance)
{
    float Rref = 10000.0;
    float A = 0.003354016;
    float B = 0.0002569850;
    float C = 0.000002620131;
    float D = 0.00000006383091;
    float E = log(resistance/Rref);

    float temp_converted_k = 1/(A + (B*E) + (C*(E*E)) + (D*(E*E*E)));
    float temp_converted_c = temp_converted_k - 273.15f;
    return temp_converted_c;
}

double ADS1115::getWireTemp()
{
    float wiretemp_regval = 0;
    float wiretemp_raw = 0;
    double wiretemp_resistance = 0;
    float sum_wt = 0;
    double average_wt = 0;
    uint16_t convRegVal = 0;

    writeConfigReg(ADS1115_CHANNEL_THREE);

    usleep(ADS1115_CONVERSION_INTERVAL);

    for (int wt = 0; wt < 5; wt++)
    {
        if (!readConvReg(&convRegVal))
        {
            wiretemp_regval = swap16(convRegVal);
            sum_wt += wiretemp_regval;
        } else
        {
            PX4_ERR("Wire Temperature Reading Failed");
        }
        usleep(ADS1115_CONVERSION_INTERVAL);
    }
    average_wt = sum_wt / 5;

    wiretemp_resistance = getResistance(average_wt);

    wiretemp_raw = getSteinhart(wiretemp_resistance);

    return wiretemp_raw;
}

double ADS1115::getBattTemp()
{
    float batttemp_regval = 0;
    float batttemp_raw = 0;
    double batttemp_resistance = 0;
    float sum_bt = 0;
    double average_bt = 0;
    uint16_t convRegVal = 0;

    writeConfigReg(ADS1115_CHANNEL_FOUR);

    usleep(ADS1115_CONVERSION_INTERVAL);

    for (int bt = 0; bt < 5; bt++)
    {
        if (!readConvReg(&convRegVal))
        {
            batttemp_regval = swap16(convRegVal);
            sum_bt += batttemp_regval;
        } else
        {
            PX4_ERR("Wire Temperature Reading Failed");
        }
        usleep(ADS1115_CONVERSION_INTERVAL);
    }
    average_bt = sum_bt / 5;

    batttemp_resistance = getResistance(average_bt);

    batttemp_raw = getSteinhart(batttemp_resistance);

    return batttemp_raw;
}

void ADS1115::start()
{
    ScheduleClear();
    ScheduleNow();
}

void ADS1115::stop()
{
    ScheduleClear();
}

void ADS1115::Run()
{
    _now_timestamp = hrt_absolute_time();
    report.timestamp = hrt_absolute_time();
    mainrep.timestamp = hrt_absolute_time();

    current = (getCurrent() * current_gain) - current_offset;

    usleep(ADS1115_CONVERSION_INTERVAL);

    voltage = (getVoltage() * voltage_gain) - voltage_offset;

    usleep(ADS1115_CONVERSION_INTERVAL);

    wiretemp = getWireTemp();

    usleep(ADS1115_CONVERSION_INTERVAL);

    batttemp = getBattTemp();


	if (voltage > 10)
	{
		batt_connected = true;
	} else
    {
        batt_connected = false;
    }


	if ((current > 3) & (_last_timestamp != 0))
	{
		const float dt = (_now_timestamp - _last_timestamp) / 1e6;
		float _discharged_temp = (current * 1e3f) * (dt / 3600.f);
		discharged += _discharged_temp;
	}
    _last_timestamp = _now_timestamp;

    report.current_a = current;
    report.voltage_v = voltage;
    report.wiretemp_c = wiretemp;
    report.batttemp_c = batttemp;

    mainrep.current_a = current;
	mainrep.current_filtered_a = current;
	/*------*/
	mainrep.voltage_v = voltage;
	mainrep.voltage_filtered_v = voltage;
	/*------*/
	mainrep.discharged_mah = discharged;
	/*------*/
	mainrep.temperature = batttemp;
	/*------*/
	mainrep.cell_count = 4;
	/*------*/
	mainrep.connected = batt_connected;
	/*------*/
	mainrep.system_source = false;
	/*------*/
	mainrep.capacity = batt_capacity;

    int instance;
    orb_publish_auto(ORB_ID(power_monitor), &_power_monitor_topic,
        &report, &instance, ORB_PRIO_DEFAULT);

    int instance1;
    orb_publish_auto(ORB_ID(battery_status), &_battery_status_topic,
        &mainrep, &instance1, ORB_PRIO_DEFAULT);

    ScheduleDelayed(ADS1115_CONVERSION_INTERVAL);
}
/* ------------------------------------------------------------------------- */

namespace ads1115 // COMPLETE SH_MEASURE()
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

        // PX4_INFO("ADS1115 Driver sh_init() success");
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
