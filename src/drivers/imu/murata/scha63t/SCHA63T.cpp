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

#include "SCHA63T.hpp"

using namespace time_literals;

#define SCHA63T_UNO 0
#define SCHA63T_DUE 1
#define G_FILT     0x2424    // Ry/Ry2 filter 300Hz 3rd order filter
#define HW_RES     0x0001    // HardReset
#define RES_EOI    0x0002    // End Of Initialization
#define MODE_NORM  0x0000    // Mode
#define A_FILT     0x0444    // Ax/Ay/Az filter 300Hz 3rd order filter

#define DEG_TO_RAD  (M_PI / 180.0)
#define GRAVITY_MSS (9.80665f)
static constexpr int16_t combine(uint8_t msb, uint8_t lsb) { return (msb << 8u) | lsb; }
static constexpr float radians(double deg) { return deg * DEG_TO_RAD; }

static SCHA63T *dev_uno = nullptr;
static SCHA63T *dev_due = nullptr;

SCHA63T::SCHA63T(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config)
{
}

I2CSPIDriverBase *SCHA63T::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	SCHA63T *instance = nullptr;

	if (config.devid_driver_index == DRV_ACC_DEVTYPE_SCHA63T) {
		instance = new SCHA63T_Accelerometer(config);
		dev_uno = instance;

	} else if (config.devid_driver_index == DRV_GYR_DEVTYPE_SCHA63T) {
		instance = new SCHA63T_Gyroscope(config);
		dev_due = instance;
	}

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->init()) {
		delete instance;
		return nullptr;
	}

	if (dev_uno && dev_due)  {
		AP_SCHA63T *sensor = new AP_SCHA63T();

		if (OK != sensor->start()) {
			delete sensor;
			return nullptr;
		}
	}

	return instance;
}

int SCHA63T::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool SCHA63T::Reset()
{
	ScheduleClear();
	ScheduleNow();
	return true;
}

void SCHA63T::read_accel()
{
	uint8_t rsp_accl_x[4] {};
	uint8_t rsp_accl_y[4] {};
	uint8_t rsp_accl_z[4] {};
	uint8_t rsp_temper[4] {};

	int16_t accel_x = 0;
	int16_t accel_y = 0;
	int16_t accel_z = 0;
	int16_t uno_temp = 0;

	// ACCL_X Cmd Send (first response is undefined data)
	if (!read_register(SCHA63T_UNO, ACC_X, rsp_accl_x)) {
		return;
	}

	// ACCL_Y Cmd Send + ACCL_X Response Receive
	if (!read_register(SCHA63T_UNO, ACC_Y, rsp_accl_x)) {
		return;
	}

	// ACCL_Z Cmd Send + ACCL_Y Response Receive
	if (!read_register(SCHA63T_UNO, ACC_Z, rsp_accl_y)) {
		return;
	}

	// TEMPER Cmd Send + RATE_X Response Receive
	if (!read_register(SCHA63T_UNO, TEMP, rsp_accl_z)) {
		return;
	}

	// TEMPER Cmd Send + TEMPRE Response Receive
	if (!read_register(SCHA63T_UNO, TEMP, rsp_temper)) {
		return;
	}

	// response data address check
	if (((rsp_accl_x[0] & 0x7C) >> 2) != ACC_X) {
		return;
	}

	accel_x = combine(rsp_accl_x[1], rsp_accl_x[2]);

	if (((rsp_accl_y[0] & 0x7C) >> 2) != ACC_Y) {
		return;
	}

	accel_y = combine(rsp_accl_y[1], rsp_accl_y[2]);

	if (((rsp_accl_z[0] & 0x7C) >> 2) != ACC_Z) {
		return;
	}

	accel_z = combine(rsp_accl_z[1], rsp_accl_z[2]);

	if (((rsp_temper[0] & 0x7C) >> 2) != TEMP) {
		return;
	}

	uno_temp = combine(rsp_temper[1], rsp_temper[2]);
	update_temper(uno_temp);

	// change coordinate system from left hand too right hand
	accel_z = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

	_accel = Vector3f((float)accel_x, (float)accel_y, (float)accel_z);

	update();
}

void SCHA63T::read_gyro()
{
	uint8_t rsp_rate_x[4];
	uint8_t rsp_rate_y[4];
	uint8_t rsp_rate_z[4];
	uint8_t rsp_uno_temper[4];
	uint8_t rsp_due_temper[4];

	int16_t gyro_x = 0;
	int16_t gyro_y = 0;
	int16_t gyro_z = 0;
	int16_t uno_temp = 0;
	int16_t due_temp = 0;

	// RATE_Y Cmd Send (first response is undefined data)
	if (!read_register(SCHA63T_DUE, RATE_Y, rsp_rate_y)) {
		return;
	}

	// RATE_Z Cmd Send + RATE_Y Response Receive
	if (!read_register(SCHA63T_DUE, RATE_XZ, rsp_rate_y)) {
		return;
	}

	// TEMPER Cmd Send + RATE_Z Response Receive
	if (!read_register(SCHA63T_DUE, TEMP, rsp_rate_z)) {
		return;
	}

	// TEMPER Cmd Send + TEMPRE Response Receive
	if (!read_register(SCHA63T_DUE, TEMP, rsp_due_temper)) {
		return;
	}

	// RATE_X Cmd Send + ACCL_Z Response Receive
	if (!read_register(SCHA63T_UNO, RATE_XZ, rsp_rate_x)) {
		return;
	}

	// TEMPER Cmd Send + TEMPRE Response Receive
	if (!read_register(SCHA63T_UNO, TEMP, rsp_rate_x)) {
		return;
	}

	// TEMPER Cmd Send + TEMPRE Response Receive
	if (!read_register(SCHA63T_UNO, TEMP, rsp_uno_temper)) {
		return;
	}

	// response data address check
	if (((rsp_rate_x[0] & 0x7C) >> 2) != RATE_XZ) {
		return;
	}

	gyro_x = combine(rsp_rate_x[1], rsp_rate_x[2]);

	if (((rsp_rate_y[0] & 0x7C) >> 2) != RATE_Y) {
		return;
	}

	gyro_y = combine(rsp_rate_y[1], rsp_rate_y[2]);

	if (((rsp_rate_z[0] & 0x7C) >> 2) != RATE_XZ) {
		return;
	}

	gyro_z = combine(rsp_rate_z[1], rsp_rate_z[2]);

	if (((rsp_uno_temper[0] & 0x7C) >> 2) != TEMP) {
		return;
	}

	uno_temp = combine(rsp_uno_temper[1], rsp_uno_temper[2]);

	if (((rsp_due_temper[0] & 0x7C) >> 2) != TEMP) {
		return;
	}

	due_temp = combine(rsp_due_temper[1], rsp_due_temper[2]);
	update_temper((uno_temp + due_temp) * 0.5);

	// change coordinate system from left hand too right hand
	gyro_z = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

	_gyro = Vector3f((float)gyro_x, (float)gyro_y, (float)gyro_z);

	update();
}

void SCHA63T::update_temper(uint16_t temper)
{
	const float temperature = 25.0f + (temper / 30);
	const float temp_degc = (0.5f * temperature) + 23.0f;
	set_temperature(temp_degc);
}

bool SCHA63T::read_register(uint8_t uno_due, reg_scha63t reg_addr, uint8_t *val)
{
	int ret_px4 = PX4_ERROR;
	bool ret = false;
	uint8_t cmd[4];
	uint8_t bCrc;

	cmd[1] = cmd[2] = 0;
	cmd[0] = reg_addr << 2;
	cmd[0] &= 0x7f;
	cmd[3] = crc.crc8_sae(cmd, 3);

	uint8_t buf[4];

	switch (uno_due) {
	case SCHA63T_UNO:
		memcpy(buf, cmd, 4);
		ret_px4 = dev_uno->transfer(cmd, buf, 4);
		memcpy(val, buf, 4);
		break;

	case SCHA63T_DUE:
		memcpy(buf, cmd, 4);
		ret_px4 = dev_due->transfer(cmd, buf, 4);
		memcpy(val, buf, 4);
		break;

	default:
		break;
	}

	if (ret_px4 == PX4_OK) {
		bCrc = crc.crc8_sae(val, 3);

		if (bCrc != val[3]) {
			ret_px4 = PX4_ERROR;
		}
	}

	// true:OK. false:FAILED
	if (ret_px4 == PX4_OK) { ret = true; }

	return ret;
}

bool SCHA63T::write_register(uint8_t uno_due, reg_scha63t reg_addr, uint16_t val)
{
	int ret_px4 = PX4_ERROR;
	bool ret = false;
	uint8_t res[4];
	uint8_t cmd[4];

	cmd[0] = reg_addr << 2;
	cmd[0] |= 0x80;
	cmd[1] = (val >> 8);
	cmd[2] = val;
	cmd[3] = crc.crc8_sae(cmd, 3);

	uint8_t buf[4];

	switch (uno_due) {
	case SCHA63T_UNO:
		memcpy(buf, cmd, 4);
		ret_px4 = dev_uno->transfer(cmd, buf, 4);
		memcpy(res, buf, 4);
		break;

	case SCHA63T_DUE:
		memcpy(buf, cmd, 4);
		ret_px4 = dev_due->transfer(cmd, buf, 4);
		memcpy(res, buf, 4);
		break;

	default:
		break;
	}

	// true:OK. false:FAILED
	if (ret_px4 == PX4_OK) { ret = true; }

	return ret;
}

/* class AP_SCHA63T */
int AP_SCHA63T::start()
{
#if defined(GPIO_SCHA63T_RESET)
	stm32_gpiowrite(GPIO_SCHA63T_RESET, true);
#endif

	// setting config
	dev_uno->ConfigureAccel();
	dev_due->ConfigureGyro();

	int ret = init();

	if (ret != OK) { return ret; }

	float rate = 2000.f;
	dev_uno->ScheduleOnInterval(rate);
	dev_due->ScheduleOnInterval(rate);

	return ret;
}

int AP_SCHA63T::init()
{
	// wait 25ms for non-volatile memory (NVM) read
	ScheduleDelayed(25);

	// set DUE operation mode on (must be less than 1ms)
	write_register(SCHA63T_DUE, MODE, MODE_NORM);
	write_register(SCHA63T_DUE, MODE, MODE_NORM);

	// set UNO operation mode on
	write_register(SCHA63T_UNO, MODE, MODE_NORM);

	// wait 70ms initial startup
	ScheduleDelayed(70);

	// set UNO configuration (data filter, flag filter)
	write_register(SCHA63T_UNO, G_FILT_DYN, G_FILT);
	write_register(SCHA63T_UNO, A_FILT_DYN, A_FILT);

	// reset DUE write (0001h) to register 18h
	write_register(SCHA63T_DUE, RESCTRL, HW_RES);

	// wait 25ms for non-volatile memory (NVM) read
	ScheduleDelayed(25);

	// set DUE operation mode on (must be less than 1ms)
	write_register(SCHA63T_DUE, MODE, MODE_NORM);
	write_register(SCHA63T_DUE, MODE, MODE_NORM);

	// wait 1ms (50ms has already passed)
	ScheduleDelayed(1);

	// set DUE configuration (data filter, flag filter)
	write_register(SCHA63T_DUE, G_FILT_DYN, G_FILT);

	// startup clear (startup_attempt = 0)
	if (!check_startup()) {
		// system in FAILURE mode (startup_attempt not equal 0 startup_attempt = 1)
		// reset UNO write (0001h) to register 18h
		write_register(SCHA63T_UNO, RESCTRL, HW_RES);
		// reset DUE write (0001h) to register 18h
		write_register(SCHA63T_DUE, RESCTRL, HW_RES);
		// wait 25ms for non-volatile memory (NVM) read
		ScheduleDelayed(25);

		// set DUE operation mode on (must be less than 1ms)
		write_register(SCHA63T_DUE, MODE, MODE_NORM);
		write_register(SCHA63T_DUE, MODE, MODE_NORM);
		// set UNO operation mode on
		write_register(SCHA63T_UNO, MODE, MODE_NORM);
		// wait 70ms initial startup
		ScheduleDelayed(50);

		// set UNO configuration (data filter, flag filter)
		write_register(SCHA63T_UNO, G_FILT_DYN, G_FILT);
		write_register(SCHA63T_UNO, A_FILT_DYN, A_FILT);
		// set DUE configuration (data filter, flag filter)
		write_register(SCHA63T_DUE, G_FILT_DYN, G_FILT);

		// wait 45ms (adjust restart duration to 500ms)
		ScheduleDelayed(45);

		if (!check_startup()) {
			// check FAILED
			return ERROR;
		}
	}

	// check ok
	return OK;
}

bool AP_SCHA63T::check_startup()
{
	uint8_t val[4] {};

	// wait 405ms (300Hz filter)
	ScheduleDelayed(405);

	// start EOI = 1
	if (!write_register(SCHA63T_UNO, RESCTRL, RES_EOI)) {
		return false;
	}

	if (!write_register(SCHA63T_DUE, RESCTRL, RES_EOI)) {
		return false;
	}

	// ready summary status twice
	for (uint8_t i = 0; i < 2; i++) {
		if (!read_register(SCHA63T_UNO, S_SUM, val)) {
			return false;
		}

		if (!read_register(SCHA63T_DUE, S_SUM, val)) {
			return false;
		}

		// wait at least 2.5ms
		ScheduleDelayed(3);
	}

	// read summary status
	if (!read_register(SCHA63T_UNO, S_SUM, val)) {
		return false;
	}

	// check UNO summary status
	if (!((val[1] & 0x9e) && (val[2] & 0xda))) {
		return false;
	}

	if (!read_register(SCHA63T_DUE, S_SUM, val)) {
		return false;
	}

	// check DUE summary status
	if (!((val[1] & 0xf8) && (val[2] & 0x03))) {
		return false;
	}

	// success if we got this far
	return true;
}

bool AP_SCHA63T::read_register(uint8_t uno_due, reg_scha63t reg_addr, uint8_t *val)
{
	bool ret = false;

	switch (uno_due) {
	case SCHA63T_UNO:
		ret = dev_uno->read_register(uno_due, reg_addr, val);
		break;

	case SCHA63T_DUE:
		ret = dev_due->read_register(uno_due, reg_addr, val);
		break;

	default:
		break;
	}

	return ret;
}

bool AP_SCHA63T::write_register(uint8_t uno_due, reg_scha63t reg_addr, uint16_t val)
{
	bool ret = false;

	switch (uno_due) {
	case SCHA63T_UNO:
		ret = dev_uno->write_register(uno_due, reg_addr, val);
		break;

	case SCHA63T_DUE:
		ret = dev_due->write_register(uno_due, reg_addr, val);
		break;

	default:
		break;
	}

	return ret;
}

/* class SCHA63T_Accelerometer */
SCHA63T_Accelerometer::SCHA63T_Accelerometer(const I2CSPIDriverConfig &config) :
	SCHA63T(config),
	_px4_accel(get_device_id(), config.rotation)
{
	_bad_register_perf = perf_alloc(PC_COUNT, MODULE_NAME"_accel: bad register");
	_bad_transfer_perf = perf_alloc(PC_COUNT, MODULE_NAME"_accel: bad transfer");
	_fifo_empty_perf = perf_alloc(PC_COUNT, MODULE_NAME"_accel: FIFO empty");
	_fifo_overflow_perf = perf_alloc(PC_COUNT, MODULE_NAME"_accel: FIFO overflow");
}

SCHA63T_Accelerometer::~SCHA63T_Accelerometer()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
}

void SCHA63T_Accelerometer::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void SCHA63T_Accelerometer::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
}

void SCHA63T_Accelerometer::RunImpl()
{
	read_accel();
}

void SCHA63T_Accelerometer::ConfigureAccel()
{
	_px4_accel.set_scale(CONSTANTS_ONE_G / 4905.f); // 4905 LSB/g, 0.204mg/LSB
	_px4_accel.set_range(6.f * CONSTANTS_ONE_G);
}

void SCHA63T_Accelerometer::set_temperature(float temp_degc)
{
	if (PX4_ISFINITE(temp_degc)) {
		_px4_accel.set_temperature(temp_degc);

	} else {
		perf_count(_bad_transfer_perf);
	}
}

void SCHA63T_Accelerometer::update()
{
	// error count setting
	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));
	// PX4 Controller ACCL-DATA update
	_px4_accel.update(hrt_absolute_time(), _accel.x, _accel.y, _accel.z);
}

/* class SCHA63T_Gyroscope */
SCHA63T_Gyroscope::SCHA63T_Gyroscope(const I2CSPIDriverConfig &config) :
	SCHA63T(config),
	_px4_gyro(get_device_id(), config.rotation)
{
	_bad_register_perf = perf_alloc(PC_COUNT, MODULE_NAME"_gyro: bad register");
	_bad_transfer_perf = perf_alloc(PC_COUNT, MODULE_NAME"_gyro: bad transfer");
	_fifo_empty_perf = perf_alloc(PC_COUNT, MODULE_NAME"_gyro: FIFO empty");
	_fifo_overflow_perf = perf_alloc(PC_COUNT, MODULE_NAME"_gyro: FIFO overflow");
}

SCHA63T_Gyroscope::~SCHA63T_Gyroscope()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
}

void SCHA63T_Gyroscope::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void SCHA63T_Gyroscope::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
}

void SCHA63T_Gyroscope::RunImpl()
{
	read_gyro();
}

int32_t SCHA63T_Gyroscope::get_rate_hz()
{
	return _px4_gyro.get_max_rate_hz();
}

void SCHA63T_Gyroscope::ConfigureGyro()
{
	_px4_gyro.set_scale(math::radians(1.f / 80.f));
	_px4_gyro.set_range(math::radians(300.f));
}

void SCHA63T_Gyroscope::set_temperature(float temp_degc)
{
	if (PX4_ISFINITE(temp_degc)) {
		_px4_gyro.set_temperature(temp_degc);

	} else {
		perf_count(_bad_transfer_perf);
	}
}

void SCHA63T_Gyroscope::update()
{
	// error count setting
	_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));
	// PX4 Controller ACCL-DATA update
	_px4_gyro.update(hrt_absolute_time(), _gyro.x, _gyro.y, _gyro.z);
}
