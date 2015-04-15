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

class SimulatorReport {
public:
	SimulatorReport(int readers, int reportLen);
	~SimulatorReport() {};

	int getReadIdx() { return _readidx; }
	int getWriteIdx() { return !_readidx; }

	bool copyData(void *inbuf, void *outbuf, int len);
	bool writeData(void *inbuf, void *outbuf, int len);

protected:
	void read_lock();
	void read_unlock();
	void write_lock();
	void write_unlock();

	void swapBuffers() 
	{
		write_lock(); 
        	_readidx = !_readidx;
		write_unlock(); 
	}

	int _readidx;
	sem_t _lock;
	const int _max_readers;
	const int _report_len;
};

class RawAccelReport : public SimulatorReport {
public:
	RawAccelReport() : SimulatorReport(1, sizeof(RawAccelData)) {}
	~RawAccelReport() {}

// FIXME - what is the endianness of these on actual device?
#pragma pack(push, 1)
        struct RawAccelData {
                int16_t         x;
                int16_t         y;
                int16_t         z;
        };
#pragma pack(pop)

	RawAccelData _data[2];
};

class MPUReport : public SimulatorReport {
public:
	MPUReport() : SimulatorReport(1, sizeof(RawMPUData)) {}
	~MPUReport() {}

#pragma pack(push, 1)
	struct RawMPUData {
		uint8_t		accel_x[2];
		uint8_t		accel_y[2];
		uint8_t		accel_z[2];
		uint8_t		temp[2];
		uint8_t		gyro_x[2];
		uint8_t		gyro_y[2];
		uint8_t		gyro_z[2];
	};
#pragma pack(pop)
	
	RawMPUData _data[2];
};

class BaroReport : public SimulatorReport {
public:
	BaroReport() : SimulatorReport(1, sizeof(RawBaroData)) {}
	~BaroReport() {}

	struct RawBaroData {
		uint8_t		d[3];
	};
	
	RawBaroData _data[2];
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
	Simulator() {}
	~Simulator() { _instance=NULL; }

	void updateSamples();

	static Simulator *_instance;
	RawAccelReport 	_accel;
	MPUReport	_mpu;
	BaroReport	_baro;
};

