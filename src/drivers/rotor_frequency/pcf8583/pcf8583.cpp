/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file pcf8583.cpp
 *
 * @author ThunderFly s.r.o., Vít Hanousek <hanousekvit@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace/TFRPM01
 *
 * Driver for Main Rotor speed sensor using MLAB.CZ RTC003A I2C counter.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/workqueue.h>

#include <lib/parameters/param.h>

#include <board_config.h>

#include <px4_log.h>
#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/rotor_frequency.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>


/* Configuration Constants */
#define PCF8583_BUS_DEFAULT 		      PX4_I2C_BUS_EXPANSION
#define PCF8583_DEVICE_PATH	         "/dev/pcf8583"


//loaded parameters defaluts
#define PCF8583_BASEADDR_DEFAULT 	            0x52     //ROTROFREQ_ADDR param default value
#define PCF8583_POOL_INTERVAL_DAFAULT	         1000000  //ROTORFREQ_POOL param defalut value
#define PCF8583_MAGNET_COUNT_DEFAULT           3        //ROTORFREQ_MAGNET param defalut value
#define PCF8583_RESET_COUNT_DEFAULT            0 //0 - reset after every measurement

#define PCF8583_POOL_INTERVAL_MAX              10000000 //10s max - limiter for setting by ioctl


#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


class PCF8583 : public device::I2C
{
public:
	PCF8583(int bus = PCF8583_BUS_DEFAULT,
	      int address = PCF8583_BASEADDR_DEFAULT);
	virtual ~PCF8583();

	virtual int 		init();

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			   ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:
	int            _pool_interval; //Interval of reading counter and publishing frequency
   int            _pool_interval_default; //Interval of reading counter and publishing frequency
   float          _indicated_frequency;
   float          _estimated_accurancy;
   int            _count;
   int            _reset_count;
   int            _magnet_count;
   uint64_t       _lastmeasurement_time;
	work_s			_work{};

	orb_advert_t		_rotor_frequency_topic;

	//perf_counter_t		_sample_perf;
	//perf_counter_t		_comms_errors;


	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void stop();

   /**
      set cycle trampoline in workqueue
   */
   void schedule_measurement();

	/**
	* Perform a poll cycle; collect from the previous measurement.
	*/
	void cycle();

   /**
    Get data from sensor
   */
   void readSensorAndComputeFreqency();


   /**
   *Publis rotor_frequency uORB mesage
   */
   void publish();

   int getCounter();
   void resetCounter();

   uint8_t readRegister(uint8_t reg);
   void setRegister(uint8_t reg, uint8_t value);

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int pcf8583_main(int argc, char *argv[]);

PCF8583::PCF8583(int bus, int address) :
	I2C("PCF8583", PCF8583_DEVICE_PATH, bus, address, 400000),
	_pool_interval(0),//us-0=disabled
   _pool_interval_default(PCF8583_POOL_INTERVAL_DAFAULT),//us
	_indicated_frequency(0.0),
   _estimated_accurancy(0.0),
   _count(0),//couter last count
   _reset_count(PCF8583_RESET_COUNT_DEFAULT ),
   _magnet_count(PCF8583_MAGNET_COUNT_DEFAULT),
	_rotor_frequency_topic(nullptr)

{
}

PCF8583::~PCF8583()
{
	/* make sure we are truly inactive */
	stop();

	if (_rotor_frequency_topic != nullptr) {
		orb_unadvertise(_rotor_frequency_topic);
	}

	/* free perf counters */
	//perf_free(_sample_perf);
	//perf_free(_comms_errors);
}

int
PCF8583::init()
{
	int ret = PX4_ERROR;

   //load parameters
	int address=PCF8583_BASEADDR_DEFAULT;
   if(param_find("PCF8583_ADDR")!=PARAM_INVALID)
   	param_get(param_find("PCF8583_ADDR"), &address);

   set_device_address(address);

   if(param_find("PCF8583_POOL")!=PARAM_INVALID)
      param_get(param_find("PCF8583_POOL"),&_pool_interval_default);

   if(param_find("PCF8583_RESET")!=PARAM_INVALID)
      param_get(param_find("PCF8583_RESET"),&_reset_count);

   if(param_find("PCF8583_MAGNET")!=PARAM_INVALID)
      param_get(param_find("PCF8583_MAGNET"),&_magnet_count);

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

   //set counter mode
   setRegister(0x00,0b00100000);

	/* get a publish handle on the range finder topic */
	struct rotor_frequency_s rf_report = {};
	_rotor_frequency_topic=orb_advertise(ORB_ID(rotor_frequency), &rf_report);

	if (_rotor_frequency_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object");
	}

	return PX4_OK;
}

int
PCF8583::getCounter()
{
   uint8_t a = readRegister(0x01);
   uint8_t b = readRegister(0x02);
   uint8_t c = readRegister(0x03);

   return int((a&0x0f)*1 + ((a&0xf0)>>4)*10 + (b&0x0f)*100 + ((b&0xf0)>>4)*1000+ (c&0x0f)*10000 + ((c&0xf0)>>4)*1000000);
}

void
PCF8583::resetCounter()
{
        setRegister(0x01,0x00);
        setRegister(0x02,0x00);
        setRegister(0x03,0x00);
}

void
PCF8583::setRegister(uint8_t reg, uint8_t value)
{
      uint8_t buff[2];
      buff[0]=reg;
      buff[1]=value;
      int ret=transfer(buff, 2, nullptr, 0);

      if (OK != ret) {
		      PX4_DEBUG("PCF8583::setRegister : i2c::transfer returned %d", ret);
	      }
}

uint8_t
PCF8583::readRegister(uint8_t reg)
{
      uint8_t rcv;
      int ret=transfer(&reg, 1, &rcv, 1);

      if (OK != ret) {
	      PX4_DEBUG("PCF8583::readRegister : i2c::transfer returned %d", ret);
      }

      return rcv;
}

int
PCF8583::probe()
{
	//return measure();??
   return PX4_OK;
}

int
PCF8583::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = ( _pool_interval == 0);

					/* set interval for next measurement to minimum legal value */

					 _pool_interval = _pool_interval_default;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
                  resetCounter();
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_pool_interval == 0);

					/* convert hz to interval in microseconds */
					int interval = 1000000 / arg;

					/* check against maximum rate */
					if (interval > 0 && interval < PCF8583_POOL_INTERVAL_MAX) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_pool_interval = interval;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
                  resetCounter();
						start();
					}

					return OK;
				}
			}
		}

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
PCF8583::read(device::file_t *filp, char *buffer, size_t buflen)
{
	int ret = 0;

	/* buffer must be large enough */
   /*if(buflen < sizeof(float))
   {
		return -ENOSPC;
   }*/

	/* if automatic measurement is enabled */
	/*if (_pool_interval> 0) {
      *((float*)buffer)=_indicated_frequency;
      ret+=sizeof(float);*/
		/* if there was no data, warn the caller */
		/*return ret;
	}*/

	/* manual measurement - run one conversion */
   //TODO:ResetCounter
	/* wait for it to complete */
	//usleep(PCF8583_POOL_INTERVAL);
   //TODO: getCounter

	return ret;
}

void
PCF8583::readSensorAndComputeFreqency()
{

   int oldcount=_count;
   uint64_t oldtime=_lastmeasurement_time;

   _count=getCounter();
   _lastmeasurement_time=hrt_absolute_time();

   int diffCount=_count-oldcount;
   uint64_t diffTime=_lastmeasurement_time-oldtime;
   if(_reset_count<_count+diffCount)
   {
      resetCounter();
      _lastmeasurement_time=hrt_absolute_time();
      _count=0;
   }

   _indicated_frequency=(float)diffCount/_magnet_count/((float)diffTime/1000000);
   _estimated_accurancy=1/(float)_magnet_count/((float)diffTime/1000000);
}

void PCF8583::publish()
{
   //PX4_INFO("Measurement: freq:%.2f, acc: %.2f, relacc: %.0f\%, count: %d", (double)_indicated_frequency, (double) _estimated_accurancy,
    //      (double)(_estimated_accurancy/_indicated_frequency*100), _count);

   //PX4_ERR("Measurement: freq:%f, count: %d", (double)_indicated_frequency, _count);

	struct rotor_frequency_s msg;
	msg.timestamp = hrt_absolute_time();
   msg.indicated_frequency_hz = _indicated_frequency;
   msg.indicated_frequency_rpm = _indicated_frequency*60;
   msg.estimated_accurancy_hz=_estimated_accurancy;
   msg.estimated_accurancy_rpm=_estimated_accurancy*60;
   msg.count=_count;

	// publish it, if we are the primary
	if (_rotor_frequency_topic != nullptr) {
		orb_publish(ORB_ID(rotor_frequency), _rotor_frequency_topic, &msg);
	}

	// notify anyone waiting for data
	poll_notify(POLLIN);

}


void
PCF8583::start()
{
	/* schedule a cycle to start things */
   resetCounter();
   _lastmeasurement_time=hrt_absolute_time();
	schedule_measurement();
}

void PCF8583::schedule_measurement()
{
   work_queue(HPWORK, &_work, (worker_t)&PCF8583::cycle_trampoline, this, USEC2TICK(_pool_interval));
}

void
PCF8583::stop()
{
	work_cancel(HPWORK, &_work);
}

void
PCF8583::cycle_trampoline(void *arg)
{
	PCF8583 *dev = (PCF8583 *)arg;
	dev->cycle();
   //start new cycle
   dev-> schedule_measurement();
}

void
PCF8583::cycle()
{
	/*Collect results */
   readSensorAndComputeFreqency();
   publish();
}


void
PCF8583::print_info()
{
	//perf_print_counter(_sample_perf);
	//perf_print_counter(_comms_errors);
	printf("poll interval:  %d us\n", _pool_interval);
}

/**
 * Local functions in support of the shell command.
 */
namespace pcf8583
{

PCF8583	*g_dev;

int 	start();
int 	start_bus( int i2c_bus);
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
	int fd = -1;

	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	/* create the driver */
	g_dev = new PCF8583( i2c_bus);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(PCF8583_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
      PX4_INFO("Cannot open device %s",PCF8583_DEVICE_PATH);
		goto fail;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
      PX4_INFO("error setting PollRate...");
		px4_close(fd);
		goto fail;
	}

	px4_close(fd);
   PX4_INFO("pcf8583 for bus: %d started.",i2c_bus );
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
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

} /* namespace */


static void
pcf8583_usage()
{
	PX4_INFO("usage: pcf8583 command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", PCF8583_BUS_DEFAULT);
	PX4_INFO("\t-a --all");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|info");
}

int
pcf8583_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	bool start_all = false;

	int i2c_bus = PCF8583_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "ab:", &myoptind, &myoptarg)) != EOF) {
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
			return pcf8583::start();

		} else {
			return pcf8583::start_bus(i2c_bus);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return pcf8583::stop();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return pcf8583::info();
	}

out_error:
	pcf8583_usage();
	return PX4_ERROR;
}
