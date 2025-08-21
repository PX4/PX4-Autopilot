/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/spi.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

enum reg_scha63t {
	RATE_XZ     = 0x01,
	RATE_Y      = 0x03,
	ACC_X       = 0x04,
	ACC_Y       = 0x05,
	ACC_Z       = 0x06,
	TEMP        = 0x07,
	S_SUM       = 0x0E,
	R_S1        = 0x10,
	A_S1        = 0x12,
	C_S1        = 0x14,
	C_S2        = 0x15,
	G_FILT_DYN  = 0x16,
	RESCTRL     = 0x18,
	MODE        = 0x19,
	A_FILT_DYN  = 0x1A,
	T_ID2       = 0x1C,
	T_ID0       = 0x1D,
	T_ID1       = 0x1E,
	SEL_BANK    = 0x1F,
};

template <typename T>
class Vector3
{
public:
	T x, y, z;
	Vector3<T>() : x(0), y(0), z(0) {}
	Vector3<T>(const T x0, const T y0, const T z0) : x(x0), y(y0), z(z0) {}
	Vector3<T> &operator *=(const T num);
};
template <typename T>
inline Vector3<T> &Vector3<T>::operator *=(const T num)
{
	x *= num; y *= num; z *= num;
	return *this;
}
typedef Vector3<float> Vector3f;

class Math_Crc
{
public:
	// CRC8-SAE J1850 (X8+X4+X3+X2+1) left move
	uint8_t crc8_sae(const uint8_t *data, uint16_t length)
	{
		uint8_t crc = 0xFF;

		while (length--) {
			crc ^= *data++;
			for ( int i = 0 ; i < 8 ; i++ ){
            	if ( crc & 0x80 ){
					crc <<= 1;
					crc ^= 0x1D;
            	}
				else{
					crc <<= 1;
				}
        	}
		}

		crc ^= 0xFF;
		return	crc;
	}
};

class AP_SCHA63T
{
public:
	AP_SCHA63T() {};

	int start();
	int init();

private:
	void ScheduleDelayed(uint16_t msec) { usleep(msec * 1000); }
	bool check_startup();
	bool read_register(uint8_t uno_due, reg_scha63t reg_addr, uint8_t *val);
	bool write_register(uint8_t uno_due, reg_scha63t reg_addr, uint16_t val);
};

class SCHA63T : public device::SPI, public I2CSPIDriver<SCHA63T>
{
public:
	SCHA63T(const I2CSPIDriverConfig &config);
	virtual ~SCHA63T() = default;

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	virtual void RunImpl() = 0;
	virtual void print_status() = 0;

	virtual int32_t get_rate_hz() {return 0;}
	virtual void ConfigureAccel() {};
	virtual void ConfigureGyro() {};
	virtual void read_accel();
	virtual void read_gyro();
	virtual bool read_register(uint8_t uno_due, reg_scha63t reg_addr, uint8_t *val);
	virtual bool write_register(uint8_t uno_due, reg_scha63t reg_addr, uint16_t val);

	virtual void update_temper(uint16_t temper);
	virtual void set_temperature(float temp_degc) {}
	virtual void update() {}

	Vector3f _accel;
	Vector3f _gyro;

protected:
	int init();
	bool Reset();

	perf_counter_t _bad_register_perf;
	perf_counter_t _bad_transfer_perf;
	perf_counter_t _fifo_empty_perf;
	perf_counter_t _fifo_overflow_perf;

	Math_Crc crc;
};

class SCHA63T_Accelerometer : public SCHA63T
{
public:
	SCHA63T_Accelerometer(const I2CSPIDriverConfig &config);
	~SCHA63T_Accelerometer() override;

	void RunImpl() override;
	void print_status() override;
	void exit_and_cleanup() override;

	void ConfigureAccel() override;

	void set_temperature(float temp_degc) override;
	void update() override;

private:
	PX4Accelerometer _px4_accel;
};

class SCHA63T_Gyroscope : public SCHA63T
{
public:
	SCHA63T_Gyroscope(const I2CSPIDriverConfig &config);
	~SCHA63T_Gyroscope() override;

	void RunImpl() override;
	void print_status() override;
	void exit_and_cleanup() override;

	int32_t get_rate_hz() override;
	void ConfigureGyro() override;

	void set_temperature(float temp_degc) override;
	void update() override;

private:
	PX4Gyroscope _px4_gyro;
};
