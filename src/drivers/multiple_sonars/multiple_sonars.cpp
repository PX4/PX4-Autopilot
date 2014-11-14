/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file multiple_sonars.cpp
 * @author Jonas Vantilt and Jon Verbeke
 *
 * Driver for multiple Maxbotix sonar range finders connected via I2C.
 */

#include <nuttx/config.h>

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
#include <vector>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_multiple_range_finders.h> //******
#include <drivers/drv_range_finder.h> //******
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

#include <board_config.h>

/* Configuration Constants */
#define MULTIPLE_SONARS_BUS 			PX4_I2C_BUS_EXPANSION

//********
#define SONAR_ADDRESS1 	0x70 /* 7-bit address. 8-bit address is 0xE0 */
#define SONAR_ADDRESS2  0x68 /* 7-bit address. 8-bit address is  */
#define SONAR_ADDRESS3  0x67 /* 7-bit address. 8-bit address is  */
#define SONAR_ADDRESS4  0x66 /* 7-bit address. 8-bit address is  */
#define SONAR_ADDRESS5  0x65 /* 7-bit address. 8-bit address is  */
#define SONAR_ADDRESS6  0x64 /* 7-bit address. 8-bit address is  */
//********
#define MULTIPLE_SONARS_DEVICE_PATH	"/dev/multiple_sonars"

/* MB12xx Registers addresses */

#define MULTIPLE_SONARS_TAKE_RANGE_REG	0x51		/* Measure range Register */
#define MULTIPLE_SONARS_SET_ADDRESS_1	0xAA		/* Change address 1 Register */
#define MULTIPLE_SONARS_SET_ADDRESS_2	0xA5		/* Change address 2 Register */

/* Device limits */
#define MULTIPLE_SONARS_MIN_DISTANCE (0.20f)
#define MULTIPLE_SONARS_MAX_DISTANCE (7.65f)

#define MULTIPLE_SONARS_CONVERSION_INTERVAL 400000 //60000 /* 60ms */
#define TICKS_BETWEEN_SUCCESIVE_FIRES 300000 // 30ms

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class MULTIPLE_SONARS : public device::I2C
{
public:
	MULTIPLE_SONARS(int bus = MULTIPLE_SONARS_BUS, int address = SONAR_ADDRESS1);
	virtual ~MULTIPLE_SONARS();

	virtual int 		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();
//****
	void				changeAddress(uint8_t address_index);
//*****

protected:
	virtual int			probe();

private:
	float				_min_distance;
	float				_max_distance;
	work_s				_work;
	RingBuffer		*_reports;
	bool				_sensor_ok;
	int					_measure_ticks;
	bool				_collect_phase;
	int					_class_instance;

	orb_advert_t		_range_finder_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;
//******
	int			_index_counter;

	uint8_t			_addr [6];
	std::vector<uint8_t>	addr_ind;
	struct multiple_range_finders_report report;
//******
	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return		True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults MULTIPLE_SONARS_MIN_DISTANCE
	* and MULTIPLE_SONARS_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					measure();
	int					collect();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void		cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int multiple_sonars_main(int argc, char *argv[]);

MULTIPLE_SONARS::MULTIPLE_SONARS(int bus, int address) :
	I2C("Multiple_sonars", MULTIPLE_SONARS_DEVICE_PATH, bus, address, 100000),
	_min_distance(MULTIPLE_SONARS_MIN_DISTANCE),
	_max_distance(MULTIPLE_SONARS_MAX_DISTANCE),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_class_instance(-1),
	_range_finder_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "multiple_sonars_read")),
	_comms_errors(perf_alloc(PC_COUNT, "multiple_sonars_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "multiple_sonars_buffer_overflows")),
	_index_counter(0)//******,
{
	// enable debug() calls
	_debug_enabled = false;
	//********
	_addr[0] = SONAR_ADDRESS1;
	_addr[1] = SONAR_ADDRESS2;
	_addr[2] = SONAR_ADDRESS3;
	_addr[3] = SONAR_ADDRESS4;
	_addr[4] = SONAR_ADDRESS5;
	_addr[5] = SONAR_ADDRESS6;
	//********

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

MULTIPLE_SONARS::~MULTIPLE_SONARS()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
}

int
MULTIPLE_SONARS::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		goto out;
	}

	/* allocate basic report buffers */
	//_reports = new RingBuffer(2, sizeof(range_finder_report));
	_reports = new RingBuffer(2, sizeof(multiple_range_finders_report));
//******
	changeAddress(0);
///******
	if (_reports == nullptr) {
		goto out;
	}

	_class_instance = register_class_devname(RANGE_FINDER_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* get a publish handle on the range finder topic */
		//struct range_finder_report rf_report;
		struct multiple_range_finders_report rf_report;
		int ret2 = measure();
		if(ret2 == 0){}
		/*{
			addr_ind.push_back(0);
			log("sonar on address 0x70");
		}*/
		addr_ind.push_back(0);

		_reports->get(&rf_report);
		//_range_finder_topic = orb_advertise(ORB_ID(sensor_range_finder), &rf_report);
		_range_finder_topic = orb_advertise(ORB_ID(sensor_multiple_range_finders), &rf_report);

		if (_range_finder_topic < 0) {
			debug("failed to create sensor_range_finder object. Did you start uOrb?");
		}
	}
//*****
	usleep(1000000);

	// check for connected rangefinders:
	for(int count = 1; count < 6; count++) {
		changeAddress(count);
		int ret3 = measure();
		
		if(ret3 == 0) { //sonar is present -> store address_index in array
			addr_ind.push_back(count);
			log("sonar added");
		}
		usleep(1000000);
	}
	changeAddress(0);

	for(int i=0; i < addr_ind.size(); i++) {
		log("sonar met address %d toegevoegd", _addr[addr_ind[i]]);
	}
	log("size if addr_ind is: %d",addr_ind.size());	
//*******
	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;
out:
	return ret;
}

int
MULTIPLE_SONARS::probe()
{
	return measure();
}

void
MULTIPLE_SONARS::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
MULTIPLE_SONARS::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
MULTIPLE_SONARS::get_minimum_distance()
{
	return _min_distance;
}

float
MULTIPLE_SONARS::get_maximum_distance()
{
	return _max_distance;
}

int
MULTIPLE_SONARS::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(MULTIPLE_SONARS_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(MULTIPLE_SONARS_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = irqsave();

			if (!_reports->resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	case RANGEFINDERIOCSETMINIUMDISTANCE: {
			set_minimum_distance(*(float *)arg);
			return 0;
		}
		break;

	case RANGEFINDERIOCSETMAXIUMDISTANCE: {
			set_maximum_distance(*(float *)arg);
			return 0;
		}
		break;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
MULTIPLE_SONARS::read(struct file *filp, char *buffer, size_t buflen)
{
	//unsigned count = buflen / sizeof(struct range_finder_report);
	unsigned count = buflen / sizeof(struct multiple_range_finders_report);
	//struct range_finder_report *rbuf = reinterpret_cast<struct range_finder_report *>(buffer);
	struct multiple_range_finders_report *rbuf = reinterpret_cast<struct multiple_range_finders_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(MULTIPLE_SONARS_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
MULTIPLE_SONARS::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = MULTIPLE_SONARS_TAKE_RANGE_REG;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		log("i2c::transfer returned %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}

int
MULTIPLE_SONARS::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[2] = {0, 0};

	perf_begin(_sample_perf);

	ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		log("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t distance = val[0] << 8 | val[1];
	float si_units = (distance * 1.0f) / 100.0f; /* cm to m */
	//struct range_finder_report report;
	//struct multiple_range_finders_report report;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	//report.distance = si_units;
	//report.valid = si_units > get_minimum_distance() && si_units < get_maximum_distance() ? 1 : 0;
	report.distance[addr_ind[_index_counter]] = si_units;
	report.just_updated = addr_ind[_index_counter];

	/* publish it, if we are the primary */
	if (_range_finder_topic >= 0) {
		//orb_publish(ORB_ID(sensor_range_finder), _range_finder_topic, &report);
		orb_publish(ORB_ID(sensor_multiple_range_finders), _range_finder_topic, &report);
	}

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
MULTIPLE_SONARS::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&MULTIPLE_SONARS::cycle_trampoline, this, 1);

	/* notify about state change */
	struct subsystem_info_s info = {
		true,
		true,
		true,
		SUBSYSTEM_TYPE_RANGEFINDER
	};
	static orb_advert_t pub = -1;

	if (pub > 0) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);

	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}

void
MULTIPLE_SONARS::stop()
{
	work_cancel(HPWORK, &_work);
}

void
MULTIPLE_SONARS::cycle_trampoline(void *arg)
{
	MULTIPLE_SONARS *dev = (MULTIPLE_SONARS *)arg;

	dev->cycle();
}
//**************
void
MULTIPLE_SONARS::changeAddress(uint8_t address_index )
{
	uint8_t ADDR;
	if(address_index <= sizeof(_addr))
	{
		ADDR = _addr[address_index];
	}
	else{ ADDR = SONAR_ADDRESS1;}

	set_address(ADDR);

	/*if(get_address() == SONAR_ADDRESS1){
                set_address(0x68);
          //      warnx("address changed");
        }
        else{   if(get_address() == 0x68) {
                        set_address(SONAR_ADDRESS1);
            //            warnx("address changed");
                } else{
        //                warnx("address NOT changed");
                }
        }*/	
}
//******************

/*void
MULTIPLE_SONARS::cycle()
{
	
	// collection phase? 
	if (_collect_phase) {
		// ********
		changeAddress(_index_counter);
		_index_counter++;
		if(_index_counter >= addr_ind.size())
		{_index_counter = 0;}
		// **************
		// perform collection 
		if (OK != collect()) {
			log("collection error");
			// restart the measurement state machine 
			start();
			return;
		}

		// next phase is measurement 
		_collect_phase = false;
		
		//
		// Is there a collect->measure gap?
		//
		if (_measure_ticks > USEC2TICK(MULTIPLE_SONARS_CONVERSION_INTERVAL)) {
			log("here");
			// schedule a fresh cycle call when we are ready to measure again 
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&MULTIPLE_SONARS::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(MULTIPLE_SONARS_CONVERSION_INTERVAL));
			return;
		}
	}

	// measurement phase
	if (OK != measure()) {
		log("measure error");
	}

	// next phase is collection 
	_collect_phase = true;

	// schedule a fresh cycle call when the measurement is done 
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&MULTIPLE_SONARS::cycle_trampoline,
		   this,
		   USEC2TICK(MULTIPLE_SONARS_CONVERSION_INTERVAL));
}*/

//*************** new cycle
void MULTIPLE_SONARS::cycle()
{
    switch (addr_ind.size()) {
    
    case 1:
	if (_collect_phase) {
                /* perform collection */
                if (OK != collect()) {
                        log("collection error");
                        /* restart the measurement state machine */
                        start();
                        return;
                }
                /* next phase is measurement */
                _collect_phase = false;
                /*
                 * Is there a collect->measure gap?
                 */
                if (_measure_ticks > USEC2TICK(MULTIPLE_SONARS_CONVERSION_INTERVAL)) {
                        log("here");
                        /* schedule a fresh cycle call when we are ready to measure again */
                        work_queue(HPWORK,
                                   &_work,
                                   (worker_t)&MULTIPLE_SONARS::cycle_trampoline,
                                   this,
                                   _measure_ticks - USEC2TICK(MULTIPLE_SONARS_CONVERSION_INTERVAL));
                        return;
                }
        }
        /* measurement phase */
        if (OK != measure()) {
                log("measure error");
        }
        /* next phase is collection */
        _collect_phase = true;
         /* schedule a fresh cycle call when the measurement is done */
        work_queue(HPWORK,
                   &_work,
                   (worker_t)&MULTIPLE_SONARS::cycle_trampoline,
                   this,
                   USEC2TICK(MULTIPLE_SONARS_CONVERSION_INTERVAL));
	break;

    case 2:
	if (_collect_phase) {
		changeAddress(addr_ind[_index_counter]);
                _index_counter++;
                if(_index_counter >= addr_ind.size())
                {_index_counter = 0;}
                /* perform collection */
                if (OK != collect()) {
                        log("collection error");
                        /* restart the measurement state machine */
                        start();
                        return;
                }
                /* next phase is measurement */
                _collect_phase = false;
                /*
                 * Is there a collect->measure gap?
                 */
                //if (_measure_ticks > USEC2TICK(TICKS_BETWEEN_SUCCESIVE_FIRES)) {
                //        log("here");
                        /* schedule a fresh cycle call when we are ready to measure again */
                //        work_queue(HPWORK,
                //                   &_work,
                //                   (worker_t)&MULTIPLE_SONARS::cycle_trampoline,
                //                   this,
                //                   _measure_ticks - USEC2TICK(TICKS_BETWEEN_SUCCESIVE_FIRES));
                //        return;
                //}
        }
        /* measurement phase */
        if (OK != measure()) {
                log("measure error");
        }
        /* next phase is collection */
        _collect_phase = true;
         /* schedule a fresh cycle call when the measurement is done */
        work_queue(HPWORK,
                   &_work,
                   (worker_t)&MULTIPLE_SONARS::cycle_trampoline,
                   this,
                   USEC2TICK(TICKS_BETWEEN_SUCCESIVE_FIRES));
        break;

    case 3:
        if (_collect_phase) {
                changeAddress(addr_ind[_index_counter]);
                _index_counter++;
                if(_index_counter >= addr_ind.size())
                {_index_counter = 0;}
                /* perform collection */
                if (OK != collect()) {
                        log("collection error");
                        /* restart the measurement state machine */
                        start();
                        return;
                }
                /* next phase is measurement */
                _collect_phase = false;
                /*
                 * Is there a collect->measure gap?
                 */
                        /* schedule a fresh cycle call when we are ready to measure again */
                work_queue(HPWORK,
                           &_work,
                           (worker_t)&MULTIPLE_SONARS::cycle_trampoline,
                           this,
                           USEC2TICK(TICKS_BETWEEN_SUCCESIVE_FIRES));
                return;
        }
        /* measurement phase */
        if (OK != measure()) {
                log("measure error");
        }
        /* next phase is collection */
        _collect_phase = true;
         /* schedule a fresh cycle call when the measurement is done */
        /*work_queue(HPWORK,
                   &_work,
                   (worker_t)&MULTIPLE_SONARS::cycle_trampoline,
                   this,
                   USEC2TICK(TICKS_BETWEEN_SUCCESIVE_FIRES));*/
        break;

    case 4:
        if (_collect_phase) {
                changeAddress(addr_ind[_index_counter]);
                _index_counter--;
                if(_index_counter == -1)
                {_index_counter = 3;}
                /* perform collection */
                if (OK != collect()) {
                        log("collection error");
                        /* restart the measurement state machine */
                        start();
                        return;
                }
                /* next phase is measurement */
                _collect_phase = false;
                /*
                 * Is there a collect->measure gap?
                 */
                        /* schedule a fresh cycle call when we are ready to measure again */
                work_queue(HPWORK,
                           &_work,
                           (worker_t)&MULTIPLE_SONARS::cycle_trampoline,
                           this,
                           USEC2TICK(TICKS_BETWEEN_SUCCESIVE_FIRES));
                return;
        }
        /* measurement phase */
	changeAddress(addr_ind[_index_counter]);
        _index_counter = _index_counter + 2;
        if(_index_counter == 4)
        {_index_counter = 0;}
	if(_index_counter == 5)
	{_index_counter = 1;}
        if (OK != measure()) {
                log("measure error");
        }
        /* next phase is collection */
        _collect_phase = true;
         /* schedule a fresh cycle call when the measurement is done */
        /*work_queue(HPWORK,
                   &_work,
                   (worker_t)&MULTIPLE_SONARS::cycle_trampoline,
                   this,
                   USEC2TICK(TICKS_BETWEEN_SUCCESIVE_FIRES));*/
        break;

     case 5:
        if (_collect_phase) {
                changeAddress(addr_ind[_index_counter]);
                _index_counter-2;
                if(_index_counter == -1)
                {_index_counter = 4;}
		if(_index_counter <= -2)
		{_index_counter = 3;}
                /* perform collection */
                if (OK != collect()) {
                        log("collection error");
                        /* restart the measurement state machine */
                        start();
                        return;
                }
                /* next phase is measurement */
                _collect_phase = false;
                /*
                 * Is there a collect->measure gap?
                 */
                        /* schedule a fresh cycle call when we are ready to measure again */
                work_queue(HPWORK,
                           &_work,
                           (worker_t)&MULTIPLE_SONARS::cycle_trampoline,
                           this,
                           USEC2TICK(TICKS_BETWEEN_SUCCESIVE_FIRES));
                return;
        }
        /* measurement phase */
        changeAddress(addr_ind[_index_counter]);
        _index_counter = _index_counter + 3;
        if(_index_counter == 5)
        {_index_counter = 0;}
        if(_index_counter == 6)
        {_index_counter = 1;}
	if(_index_counter >= 7)
        {_index_counter = 2;}
        if (OK != measure()) {
                log("measure error");
        }
        /* next phase is collection */
        _collect_phase = true;
        break;

    case 6:
        if (_collect_phase) {
                changeAddress(addr_ind[_index_counter]);
                _index_counter-3;
                if(_index_counter == -1)
                {_index_counter = 5;}
                if(_index_counter == -2)
                {_index_counter = 4;}
		if(_index_counter <= -3)
                {_index_counter = 3;}
                /* perform collection */
                if (OK != collect()) {
                        log("collection error");
                        /* restart the measurement state machine */
                        start();
                        return;
                }
                /* next phase is measurement */
                _collect_phase = false;
                /*
                 * Is there a collect->measure gap?
                 */
                        /* schedule a fresh cycle call when we are ready to measure again */
                work_queue(HPWORK,
                           &_work,
                           (worker_t)&MULTIPLE_SONARS::cycle_trampoline,
                           this,
                           USEC2TICK(TICKS_BETWEEN_SUCCESIVE_FIRES));
                return;
        }
        /* measurement phase */
        changeAddress(addr_ind[_index_counter]);
        _index_counter = _index_counter + 4;
        if(_index_counter == 6)
        {_index_counter = 0;}
        if(_index_counter == 7)
        {_index_counter = 1;}
        if(_index_counter == 8)
        {_index_counter = 2;}
	if(_index_counter >= 9)
        {_index_counter = 3;}
        if (OK != measure()) {
                log("measure error");
        }
        /* next phase is collection */
        _collect_phase = true;
        break;

    }
}
//************* new cycle ends




void
MULTIPLE_SONARS::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace multiple_sonars
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

MULTIPLE_SONARS	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();
void	change_address();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new MULTIPLE_SONARS(MULTIPLE_SONARS_BUS);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(MULTIPLE_SONARS_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Change the driver address
 */
void change_address()
{
	if (g_dev != nullptr) {
		g_dev->changeAddress(0);
        }
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	//struct range_finder_report report;
	struct multiple_range_finders_report report;
	ssize_t sz;
	int ret;

	int fd = open(MULTIPLE_SONARS_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'multiple_sonars start' if the driver is not running", MULTIPLE_SONARS_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.distance[1]);
	warnx("time:        %lld", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("measurement sensor1: %0.3f", (double)report.distance[0]);
		warnx("measurement sensor2: %0.3f", (double)report.distance[1]);
		warnx("measurement sensor3: %0.3f", (double)report.distance[2]);
		warnx("measurement sensor4: %0.3f", (double)report.distance[3]);
		warnx("measurement sensor5: %0.3f", (double)report.distance[4]);
		warnx("measurement sensor6: %0.3f", (double)report.distance[5]);
		warnx("time:        %lld", report.timestamp);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(MULTIPLE_SONARS_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace

int
multiple_sonars_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		multiple_sonars::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		multiple_sonars::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		multiple_sonars::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		multiple_sonars::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		multiple_sonars::info();
	}

	/*
         * Change address.
         */
        if (!strcmp(argv[1], "change_ address") || !strcmp(argv[1], "change")) {
                multiple_sonars::change_address();
        }


	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
