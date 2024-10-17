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
private:
	// CRC8-SAE J1850 (X8+X4+X3+X2+1) left move table
	const uint8_t crc8_table_sae[256] = {
		0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
		0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
		0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
		0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
		0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
		0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
		0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
		0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
		0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
		0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
		0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
		0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
		0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
		0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
		0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
		0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4,
	};

public:
	uint8_t crc8_sae(const uint8_t *data, uint16_t length)
	{
		uint8_t crc = 0xFF;

		while (length--) {
			crc = crc8_table_sae[crc ^ (*data & 0xFF)];
			data++;
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
