/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "BMI088.hpp"

BMI088::BMI088(int bus, enum Rotation rotation) :
	_accel{bus, PX4_SPIDEV_BMI088_ACC, rotation},
	_gyro{bus, PX4_SPIDEV_BMI088_GYR, rotation}
{
}

int
BMI088::init()
{
	int ret_accel = _accel.init();

	if (ret_accel != PX4_OK) {
		return ret_accel;
	}

	int ret_gyro = _gyro.init();

	if (ret_gyro != PX4_OK) {
		return ret_gyro;
	}

	return PX4_OK;
}

static int acc_data_ready_interrupt(int irq, void *context, void *arg)
{
	static volatile int acc_count = 0;
	acc_count++;
	return PX4_OK;
}

static int gyro_data_ready_interrupt(int irq, void *context, void *arg)
{
	static volatile int gyro_count = 0;
	gyro_count++;
	return PX4_OK;
}

bool
BMI088::start()
{

	// make sure we are stopped first
	stop();

#ifdef GPIO_SPI3_DRDY1_BMI088_INT1_ACCEL
	// Setup data ready Interrupt on Falling edge for better noise immunity
	px4_arch_gpiosetevent(GPIO_SPI3_DRDY1_BMI088_INT1_ACCEL, false, true, false, &acc_data_ready_interrupt, this);
#endif // GPIO_SPI3_DRDY1_BMI088_INT1_ACCEL

#ifdef GPIO_SPI3_DRDY2_BMI088_INT3_GYRO
	// Setup data ready Interrupt on Falling edge for better noise immunity
	px4_arch_gpiosetevent(GPIO_SPI3_DRDY2_BMI088_INT3_GYRO, false, true, false, &gyro_data_ready_interrupt, this);
#endif // GPIO_SPI3_DRDY2_BMI088_INT3_GYRO

	// start polling at the specified rate
	//ScheduleOnInterval(BMI088_GYRO_DEFAULT_RATE - BMI088_TIMER_REDUCTION, 1000);
	// make another measurement
	Run(false);
	Run(true);

	return true;
}

bool
BMI088::stop()
{

#ifdef GPIO_SPI3_DRDY1_BMI088_INT1_ACCEL
	// Disable data ready callback
	px4_arch_gpiosetevent(GPIO_SPI3_DRDY1_BMI088_INT1_ACCEL, false, false, false, nullptr, nullptr);
#endif // GPIO_SPI3_DRDY1_BMI088_INT1_ACCEL

#ifdef GPIO_SPI3_DRDY2_BMI088_INT3_GYRO
	// Disable data ready callback
	px4_arch_gpiosetevent(GPIO_SPI3_DRDY2_BMI088_INT3_GYRO, false, false, false, nullptr, nullptr);
#endif // GPIO_SPI3_DRDY2_BMI088_INT3_GYRO

	//ScheduleClear();

	return true;
}

void
BMI088::print_info()
{
	_accel.print_info();
	_gyro.print_info();
}

void
BMI088::print_registers()
{
	_accel.print_registers();
	_gyro.print_registers();
}

void
BMI088::Run(bool a)
{


	a ? _accel.Run() :   _gyro.Run();

	// grab temperature from accel and set in gyro

	// TODO: at 1 Hz check temperature, health, etc?




}
