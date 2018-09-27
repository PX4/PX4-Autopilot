/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file mpu9250.cpp
 * @author Jacob Dahl <jacob.dahl@tealdrones.com>
 *
 * Driver for the Invensense MPU9250 connected via I2C or SPI.
 */

#include "mpu9250.h"


extern "C" __EXPORT int mpu9250_main(int argc, char *argv[]);

MPU9250::MPU9250(mpu9250::mpu9250_bus_option &bus, enum Rotation rotation) :

	// Create the communication interface.
	_interface(bus.interface_constructor(bus.busnum, bus.address)),

	// Construct the gyro and accel objects.
	_accel(bus.accel_path, _interface, DRV_ACC_DEVTYPE_MPU9250),
	_gyro(bus.gyro_path, _interface, DRV_GYR_DEVTYPE_MPU9250),

	_spi_transfer(perf_alloc(PC_ELAPSED, "mpu9250_spi_transfer")),
	_cycle(perf_alloc(PC_ELAPSED, "mpu9250_cycle")),
	_fifo_maxed(perf_alloc(PC_COUNT, "mpu9250_fifo_maxed")),

	_rotation(rotation)

{
	if (_interface->init() != OK) {
		delete _interface;
		warnx("No device on bus %u.", (unsigned)bus.busid);
	}

	probe();

	if (_whoami == WHOAMI_9250) {
		_mag = new Mag(bus.mag_path, _interface, DRV_MAG_DEVTYPE_MPU9250);
	}

	// Allocate 512 bytes for dma FIFO transfer.
	_dma_data_buffer = (uint8_t *)fat_dma_alloc(_dma_buffer_size);
}

MPU9250::~MPU9250()
{
	if (_interface) {
		delete _interface;
	}

	if (_mag) {
		delete _mag;
	}

	fat_dma_free(_dma_data_buffer, _dma_buffer_size);

	hrt_cancel(&_hrt_call);
	px4_sem_destroy(&_data_semaphore);

	perf_free(_spi_transfer);
	perf_free(_cycle);
	perf_free(_fifo_maxed);
}

mpu9250::mpu9250_bus_option &MPU9250::initialize_bus(mpu9250::MPU9250_BUS bus_id)
{
	uint8_t i = 0;

	for (i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (bus_id == mpu9250::g_bus_options[i].busid) {
			return mpu9250::g_bus_options[i];
		}
	}

	PX4_ERR("Could not find bus.");
	return mpu9250::g_bus_options[i];
}

MPU9250 *MPU9250::instantiate(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	enum mpu9250::MPU9250_BUS busid = mpu9250::MPU9250_BUS_SPI_INTERNAL;

	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "XISstR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = mpu9250::MPU9250_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = mpu9250::MPU9250_BUS_I2C_INTERNAL;
			break;

		case 'S':
			busid = mpu9250::MPU9250_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = mpu9250::MPU9250_BUS_SPI_INTERNAL;
			break;

		case 't':
			busid = mpu9250::MPU9250_BUS_SPI_INTERNAL2;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			print_usage();
			return 0;
		}
	}

	// Get the bus options.
	struct mpu9250::mpu9250_bus_option bus = initialize_bus(busid);

	// Create the parent object with the correct interface and options.
	MPU9250 *mpu9250 = new MPU9250(bus, rotation);

	return mpu9250;
}


int MPU9250::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("mpu9250", SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 1, 2000,
					 (px4_main_t)&run_trampoline, (char *const *)argv);

	return task_id;
}

int MPU9250::custom_command(int argc, char *argv[])
{

	return 0;
}

void MPU9250::init()
{
	_accel.init();
	_accel.configure_filter(ACCEL_SAMPLE_RATE, ACCEL_FILTER_FREQ);

	_gyro.init();
	_gyro.configure_filter(GYRO_SAMPLE_RATE, GYRO_FILTER_FREQ);

	// Configure all the chip settings.
	// SEE DATASHEET FOR REGISTER DEFINTIONS -- https://media.digikey.com/pdf/Data%20Sheets/InvenSense%20PDFs/MPU-9250.pdf

	// Reset all fields to default.
	write_reg(PWR_MGMT_1_ADDR, PWR_MGMT_1_RESET);

	// Allow time to power up.
	usleep(100000);

	write_reg(PWR_MGMT_2_ADDR, PWR_MGMT_2_ENABLE_GYRO_ACCEL);

	write_reg(SMPLRT_DIV, 0);
	write_reg(CONFIG_ADDR, CONFIG_NO_OVERFLOW);

	write_reg(GYRO_CONFIG_ADDR, GYRO_CONFIG_32KHZ);

	write_reg(ACCEL_CONFIG1_ADDR, ACCEL_CONFIG1_16G_FS);
	write_reg(ACCEL_CONFIG2_ADDR, ACCEL_CONFIG2_4KHZ);

	write_reg(INT_PIN_CFG_ADDR, INT_PIN_CFG_VALUE);
	write_reg(INT_ENABLE_ADDR, INT_ENABLE_NONE);

	// Using the FIFO at 32khz for gyro and accel only.
	write_reg(FIFO_EN_ADDR, FIFO_EN_GYRO_ACCEL);

	// Must write this last. Clears the FIFO, resets sensors, resets data path and enables FIFO.
	clean_reset();

	// Set up the mag for SPI mode. NOTE: enabling the i2c master interface brings the FIFO output rate down to 4KHz.
	if (_whoami == WHOAMI_9250) {

		_mag->init();
		_mag->configure_filter(MAG_SAMPLE_RATE, MAG_FILTER_FREQ);

		// Enable the i2c master.
		write_reg(USER_CTRL_ADDR, USER_CTRL_FIFO_EN | USER_CTRL_BIT_I2C_MST_EN);

		// Configure the i2c master.
		write_reg(I2C_MST_CTRL_ADDR, I2C_MST_CTRL_VALUE);

		// Turn on continuous mode.
		write_reg(I2C_SLV0_CTRL, 0);
		write_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x00);
		write_reg(I2C_SLV0_REG,  AK8963REG_CNTL1_ADDR);
		write_reg(I2C_SLV0_DATA_OUT, AK8963_CONTINUOUS_16BIT);
		write_reg(I2C_SLV0_CTRL, 0x01 | BIT_I2C_SLV0_EN);

		// Turn off slave.
		write_reg(I2C_SLV0_CTRL, 0);
		// Set the ADDR and READ BIT.
		write_reg(I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
		// Specify the register to READ at.
		write_reg(I2C_SLV0_REG,  AK8963REG_ST1);
		// Turn the thing on.
		write_reg(I2C_SLV0_CTRL, 0x08 | BIT_I2C_SLV0_EN);
	}
}

void MPU9250::clean_reset()
{
	// Disable new data from entering FIFO.
	write_reg(FIFO_EN_ADDR, FIFO_EN_DISABLE_ALL);

	// Reset the digital signal path for all the sensors.
	write_reg(SIGNAL_PATH_RESET_ADDR, SIGNAL_PATH_RESET_ALL);

	// Clear the sensors registers -- turn off FIFO.
	write_reg(USER_CTRL_ADDR, USER_CTRL_SENSORS_CLEAR);

	// Reset the FIFO -- turn off FIFO.
	write_reg(USER_CTRL_ADDR, USER_CTRL_FIFO_RESET);

	// Re-enable FIFO.
	write_reg(FIFO_EN_ADDR, FIFO_EN_GYRO_ACCEL);

	// Turn on FIFO.
	write_reg(USER_CTRL_ADDR, USER_CTRL_FIFO_EN);
}



int MPU9250::probe()
{
	_whoami = read_reg(WHOAMI_ADDR);

	switch (_whoami) {
	case WHOAMI_9250:
		PX4_INFO("Driver started successfully: 9250.");
		return PX4_OK;

	case WHOAMI_6500:
		PX4_INFO("Driver started successfully: 6500.");
		return PX4_OK;
	}

	return PX4_ERROR;
}


int MPU9250::self_test()
{
	// if (perf_event_count(_sample_perf) == 0) {
	// 	measure();
	// }

	// // return 0 on success, 1 else
	// return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
	return 0;
}

uint8_t MPU9250::read_reg(unsigned reg)
{
	uint8_t buf[2] = {0};

	_interface->read(reg, buf, 2);

	return buf[1];
}

void MPU9250::write_reg(unsigned reg, uint8_t value)
{
	uint8_t buf[2] = {0};

	buf[0] = value;

	_interface->write(reg, buf, 2);
}

void MPU9250::post_semaphore(void *sem)
{
	px4_sem_post((px4_sem_t *)sem);
}

void MPU9250::run()
{
	// Find device and set type (9250/6500).
	if (probe() != PX4_OK) {
		PX4_ERR("Device not found.");
		return;
	}

	// Initialize ASIC configuration settings.
	init();

	px4_sem_init(&_data_semaphore, 0, 0);

	hrt_call_every(&_hrt_call, 100000, 1000, (hrt_callout)&MPU9250::post_semaphore, &_data_semaphore);

	while (!should_exit()) {

		perf_begin(_cycle);

		// Clear buffer
		memset(_dma_data_buffer, 0, _dma_buffer_size);

		// Wait until semaphore has been posted via the timer interrupt.
		px4_sem_wait(&_data_semaphore);
		perf_begin(_spi_transfer);

		// Check the FIFO data count.
		uint8_t fifo_count_high = read_reg(FIFO_COUNTH) & 0x1F;
		uint8_t fifo_count_low = read_reg(FIFO_COUNTL);
		uint16_t fifo_count = 0;
		fifo_count = ((fifo_count_high << 8) & 0xFF00) | (fifo_count_low & 0x00FF);

		if (fifo_count >= 512) {
			fifo_count = 512;
			perf_count(_fifo_maxed);
		}

		// We only want to read FULL (12B) sets of data (6 gyro / 6 accel).
		uint16_t remainder = fifo_count % 12;
		fifo_count -= remainder;

		// Collect the FIFO. This is done via DMA.
		if (OK != _interface->read(FIFO_R_W, _dma_data_buffer, fifo_count + 1)) {
			PX4_INFO("SPI read failed");
			return;
		}

		perf_end(_spi_transfer);

		unsigned samples = fifo_count / 12;

		float accel_x = 0;
		float accel_y = 0;
		float accel_z = 0;

		float gyro_x = 0;
		float gyro_y = 0;
		float gyro_z = 0;

		for (unsigned i = 0; i < samples; i++) {
			accel_x += int16_t(_dma_data_buffer[_offset + 0 + i * 12] << 8 | _dma_data_buffer[_offset + 1 + i * 12]);
			accel_y += int16_t(_dma_data_buffer[_offset + 2 + i * 12] << 8 | _dma_data_buffer[_offset + 3 + i * 12]);
			accel_z += int16_t(_dma_data_buffer[_offset + 4 + i * 12] << 8 | _dma_data_buffer[_offset + 5 + i * 12]);

			gyro_x 	+= int16_t(_dma_data_buffer[_offset + 6 + i * 12] << 8 | _dma_data_buffer[_offset + 7 + i * 12]);
			gyro_y 	+= int16_t(_dma_data_buffer[_offset + 8 + i * 12] << 8 | _dma_data_buffer[_offset + 9 + i * 12]);
			gyro_z 	+= int16_t(_dma_data_buffer[_offset + 10 + i * 12] << 8 | _dma_data_buffer[_offset + 11 + i * 12]);
		}

		// Always reset if the fifo has filled.
		if (fifo_count >= 504) {
			clean_reset();
		}

		accel_x = (accel_x / (samples));
		accel_y = (accel_y / (samples));
		accel_z = (accel_z / (samples));

		gyro_x = (gyro_x / (samples));
		gyro_y = (gyro_y / (samples));
		gyro_z = (gyro_z / (samples));

		_accel.publish(accel_x, accel_y, accel_z, ACCEL_FS_RANGE_M_S2 / 65535.0f, _rotation);
		_gyro.publish(gyro_x, gyro_y, gyro_z, GYRO_FS_RANGE_RADS / 65535.0f, _rotation);

		// Read mag at 100hz.
		if (_whoami == WHOAMI_9250) {
			if (hrt_elapsed_time(&_mag_poll_time) > _mag_interval) {

				_interface->read(EXT_SENS_DATA_00, _dma_data_buffer, 8 + _offset);

				float x 		= (uint16_t)(_dma_data_buffer[3] << 8 | _dma_data_buffer[2]);
				float y 		= (uint16_t)(_dma_data_buffer[5] << 8 | _dma_data_buffer[4]);
				float z 		= (uint16_t)(_dma_data_buffer[7] << 8 | _dma_data_buffer[6]);

				// Now bring the mag into same reference frame as the accel and gyro.
				int16_t temp = x;
				x = y;
				y = temp;

				_mag->publish(x, y, z, MAG_FS_RANGE_UT / 65535.0f, _rotation);

				_mag_poll_time = hrt_absolute_time();
			}
		}

		// Read temperature at 10hz

		perf_end(_cycle);
	}

}



int MPU9250::print_status()
{

	return PX4_OK;
}


int MPU9250::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		Bla Blah blaaaa.
		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mpu9250", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("controller_period",
					 "Reports the heater driver cycle period value, (us), and sets it if supplied an argument.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("duty_cycle", "Reports the heater duty cycle (%).");
	PRINT_MODULE_USAGE_COMMAND_DESCR("feed_forward",
					 "Sets the feedforward value if supplied an argument and reports the current value.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("integrator",
					 "Sets the integrator gain value if supplied an argument and reports the current value.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("proportional",
					 "Sets the proportional gain value if supplied an argument and reports the current value.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("sensor_id", "Reports the current IMU the heater is temperature controlling.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("setpoint", "Reports the current IMU temperature.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Starts the IMU heater driver as a background task");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status",
					 "Reports the current IMU temperature, temperature setpoint, and heater on/off status.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stops the IMU heater driver.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("temp", "Reports the current IMU temperature.");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int mpu9250_main(int argc, char *argv[])
{
	return MPU9250::main(argc, argv);
}