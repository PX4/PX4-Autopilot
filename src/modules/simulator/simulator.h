/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file simulator.h
 * A device simulator
 */

#pragma once

#include <semaphore.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_attitude.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <v1.0/mavlink_types.h>
#include <v1.0/common/mavlink.h>
#ifndef __PX4_QURT
#include <sys/socket.h>
#include <netinet/in.h>
#endif

namespace simulator {

// FIXME - what is the endianness of these on actual device?
#pragma pack(push, 1)
struct RawAccelData {
        int16_t x;
        int16_t y;
        int16_t z;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RawMPUData {
	uint8_t	accel_x[2];
	uint8_t	accel_y[2];
	uint8_t	accel_z[2];
	uint8_t	temp[2];
	uint8_t	gyro_x[2];
	uint8_t	gyro_y[2];
	uint8_t	gyro_z[2];
};
#pragma pack(pop)

struct RawBaroData {
	uint8_t		d[3];
};

template <typename RType> class Report {
public:
	Report(int readers) :
        	_max_readers(readers),
        	_report_len(sizeof(RType))
	{
        	sem_init(&_lock, 0, _max_readers);
	}

	~Report() {};

	bool copyData(void *outbuf, int len)
	{
		if (len != _report_len) {
			return false;
		}
		read_lock();
		memcpy(outbuf, &_buf[_readidx], _report_len);
		read_unlock();
		return true;
	}
	void writeData(void *inbuf)
	{
		write_lock();
		memcpy(&_buf[!_readidx], inbuf, _report_len);
		_readidx = !_readidx;
		write_unlock();
	}

protected:
	void read_lock() { sem_wait(&_lock); }
	void read_unlock() { sem_post(&_lock); }
	void write_lock() 
	{
		for (int i=0; i<_max_readers; i++) {
                	sem_wait(&_lock);
        	}
	}
	void write_unlock()
	{
		for (int i=0; i<_max_readers; i++) {
			sem_post(&_lock);
		}
	}

	int _readidx;
	sem_t _lock;
	const int _max_readers;
	const int _report_len;
	RType _buf[2];
};

};

class Simulator {
public:
	static Simulator *getInstance();

	enum sim_dev_t {
		SIM_GYRO,
		SIM_ACCEL,
		SIM_MAG
	};

	struct sample {
		float x;
		float y;
		float z;
		sample() : x(0), y(0), z(0) {}
		sample(float a, float b, float c) : x(a), y(b), z(c) {}
	};

	static int start(int argc, char *argv[]);

	bool getRawAccelReport(uint8_t *buf, int len);
	bool getMPUReport(uint8_t *buf, int len);
	bool getBaroSample(uint8_t *buf, int len);
private:
	Simulator() :
	_accel(1),
	_mpu(1),
	_baro(1),
	_sensor_combined_pub(nullptr)
#ifndef __PX4_QURT
	,
	_manual_control_sp_pub(nullptr),
	_actuator_outputs_sub(-1),
	_vehicle_attitude_sub(-1),
	_sensor{},
	_manual_control_sp{},
	_actuators{},
	_attitude{}
#endif
	{}
	~Simulator() { _instance=NULL; }

#ifndef __PX4_QURT
	void updateSamples();
#endif

	static Simulator *_instance;

	// simulated sensor instances
	simulator::Report<simulator::RawAccelData> 	_accel;
	simulator::Report<simulator::RawMPUData>	_mpu;
	simulator::Report<simulator::RawBaroData>	_baro;

	// uORB publisher handlers
	orb_advert_t _accel_pub;
	orb_advert_t _baro_pub;
	orb_advert_t _gyro_pub;
	orb_advert_t _mag_pub;
	orb_advert_t _sensor_combined_pub;

	// class methods
	void publishSensorsCombined();

#ifndef __PX4_QURT
	// uORB publisher handlers
	orb_advert_t _manual_control_sp_pub;

	// uORB subscription handlers
	int _actuator_outputs_sub;
	int _vehicle_attitude_sub;

	// uORB data containers
	struct sensor_combined_s _sensor;
	struct manual_control_setpoint_s _manual_control_sp;
	struct actuator_outputs_s _actuators;
	struct vehicle_attitude_s _attitude;

	int _fd;
	unsigned char _buf[200];
	hrt_abstime _time_last;
	struct sockaddr_in _srcaddr;
	socklen_t _addrlen = sizeof(_srcaddr);

	void poll_topics();
	void handle_message(mavlink_message_t *msg);
	void send_data();
	void pack_actuator_message(mavlink_hil_controls_t &actuator_msg);
	void send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID);
	static void *sending_trampoline(void *);
	void send();
#endif
};
