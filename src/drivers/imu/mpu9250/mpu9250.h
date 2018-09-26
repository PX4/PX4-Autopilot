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

#pragma once

#include <drivers/drv_hrt.h>
#include <drivers/accelerometer/Accel.hpp>
#include <drivers/gyroscope/Gyro.hpp>
#include <drivers/magnetometer/Mag.hpp>

#include <ecl/geo/geo.h>

#include <nuttx/fs/fat.h>

#include <perf/perf_counter.h>

#include <px4_module_multi.h>
#include <px4_getopt.h>


// MPU9250 registers
#define WHOAMI_ADDR					0x75

#define PWR_MGMT_1_ADDR				0x6B
#define PWR_MGMT_1_AUTO_SELECT		0x01
#define PWR_MGMT_1_20MHZ_OSC		0x00
#define PWR_MGMT_1_RESET			0x80

#define PWR_MGMT_2_ADDR				0x6C
#define PWR_MGMT_2_ENABLE_GYRO_ACCEL 0x00
#define PWR_MGMT_2_DISABLE_GYRO_ACCEL 0x3F

#define SMPLRT_DIV					0x19

#define SIGNAL_PATH_RESET_ADDR		0x68
#define SIGNAL_PATH_RESET_GYRO_ACCEL 0x06
#define SIGNAL_PATH_RESET_ALL		0x07

#define CONFIG_ADDR					0x1A
#define CONFIG_VALUE				0x07
#define CONFIG_NO_OVERFLOW 			0x47

#define GYRO_CONFIG_ADDR			0x1B
#define GYRO_CONFIG_8KHZ			0x18
#define GYRO_CONFIG_32KHZ			0x1B

#define ACCEL_CONFIG1_ADDR			0x1C
#define ACCEL_CONFIG1_16G_FS		0x18
#define ACCEL_CONFIG2_ADDR 			0x1D
#define ACCEL_CONFIG2_4KHZ			0x0F

#define INT_PIN_CFG_ADDR			0x37
#define INT_PIN_CFG_VALUE			0x10

#define INT_ENABLE_ADDR				0x38
#define INT_ENABLE_ANY_READ			0x10
#define INT_ENABLE_NONE				0x00

#define FIFO_EN_ADDR				0x23
#define FIFO_EN_GYRO_ACCEL			0x78
#define FIFO_EN_ACCEL 				0x08
#define FIFO_EN_DISABLE_ALL 		0x00

#define USER_CTRL_ADDR				0x6A
#define USER_CTRL_FIFO_EN			0x60
#define USER_CTRL_SENSORS_CLEAR		0x21
#define USER_CTRL_FIFO_RESET		0x24
#define USER_CTRL_BIT_I2C_MST_EN    0x20
#define USER_CTRL_I2C_MASTER_EN		0x5D

#define INT_STATUS					0x3A

// Supports 20MHz reads
#define FIFO_COUNTH					0x72
#define FIFO_COUNTL					0x73
#define FIFO_R_W					0x74

#define WHOAMI_9250					0x71
#define WHOAMI_6500					0x70

#define I2C_MST_CTRL_ADDR			0x24
#define I2C_MST_CTRL_VALUE			0x5D

#define I2C_SLV0_ADDR				0x25
#define I2C_SLV0_REG				0x26
#define I2C_SLV0_CTRL				0x27
#define I2C_SLV0_DATA_OUT		 	0x63
#define I2C_SLV0_EN_8BYTES          0x88
#define I2C_SLV0_EN_1BYTES          0x81
#define BIT_I2C_SLV0_EN             0x80

#define EXT_SENS_DATA_00			0x49


// ak8963 register address and bit definitions
#define AK8963_I2C_ADDR         0x0C
#define AK8963_DEVICE_ID        0x48
#define AK8963REG_WIA           0x00
#define AK8963REG_ST1           0x02
#define AK8963REG_CNTL1_ADDR	0x0A
#define AK8963REG_CNTL2_ADDR    0x0B
#define AK8963_CONTINUOUS_16BIT 0x16
#define AK8963_SELFTEST_MODE    0x08
#define AK8963_RESET            0x01


#define ACCEL_SAMPLE_RATE	1000
#define ACCEL_FILTER_FREQ 30
#define ACCEL_FS_RANGE_M_S2	32.0f * CONSTANTS_ONE_G

#define GYRO_SAMPLE_RATE	1000
#define GYRO_FILTER_FREQ 30
#define GYRO_FS_RANGE_RADS	(4000.0f * M_PI_F) / 180

#define MAG_SAMPLE_RATE 100
#define MAG_FILTER_FREQ 10
#define MAG_FS_RANGE_UT 9600.0f


#define MPU_DEVICE_PATH_ACCEL		"/dev/mpu9250_accel"
#define MPU_DEVICE_PATH_GYRO		"/dev/mpu9250_gyro"
#define MPU_DEVICE_PATH_MAG			"/dev/mpu9250_mag"

#define MPU_DEVICE_PATH_ACCEL_1		"/dev/mpu9250_accel1"
#define MPU_DEVICE_PATH_GYRO_1		"/dev/mpu9250_gyro1"
#define MPU_DEVICE_PATH_MAG_1		"/dev/mpu9250_mag1"

#define MPU_DEVICE_PATH_ACCEL_EXT	"/dev/mpu9250_accel_ext"
#define MPU_DEVICE_PATH_GYRO_EXT	"/dev/mpu9250_gyro_ext"
#define MPU_DEVICE_PATH_MAG_EXT 	"/dev/mpu9250_mag_ext"

#define MPU_DEVICE_PATH_ACCEL_EXT1	"/dev/mpu9250_accel_ext1"
#define MPU_DEVICE_PATH_GYRO_EXT1	"/dev/mpu9250_gyro_ext1"
#define MPU_DEVICE_PATH_MAG_EXT1 	"/dev/mpu9250_mag_ext1"

#define MPU_DEVICE_PATH_ACCEL_EXT2	"/dev/mpu9250_accel_ext2"
#define MPU_DEVICE_PATH_GYRO_EXT2	"/dev/mpu9250_gyro_ext2"
#define MPU_DEVICE_PATH_MAG_EXT2	"/dev/mpu9250_mag_ext2"

#define NUM_BUS_OPTIONS (sizeof(mpu9250::g_bus_options)/sizeof(mpu9250::g_bus_options[0]))


typedef device::Device *(*MPU9250_constructor)(int, uint32_t);

// Interface factories.
extern device::Device *MPU9250_SPI_interface(int bus, uint32_t cs);
extern device::Device *MPU9250_I2C_interface(int bus, uint32_t address);
extern int MPU9250_probe(device::Device *dev, int device_type);

namespace mpu9250
{
enum MPU9250_BUS {
	MPU9250_BUS_ALL = 0,
	MPU9250_BUS_I2C_INTERNAL,
	MPU9250_BUS_I2C_EXTERNAL,
	MPU9250_BUS_SPI_INTERNAL,
	MPU9250_BUS_SPI_INTERNAL2,
	MPU9250_BUS_SPI_EXTERNAL
};
// shitty list of bus configurations -- lets find a better way to do this in the future?

struct mpu9250_bus_option {
	enum MPU9250_BUS busid;
	const char *accel_path;
	const char *gyro_path;
	const char *mag_path;
	MPU9250_constructor interface_constructor;
	uint8_t busnum;
	uint32_t address;
} g_bus_options[] = {

#  if defined(PX4_I2C_BUS_ONBOARD)
	{ MPU9250_BUS_I2C_INTERNAL, MPU_DEVICE_PATH_ACCEL, MPU_DEVICE_PATH_GYRO, MPU_DEVICE_PATH_MAG,  &MPU9250_I2C_interface, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_MPU9250},
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION)
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT, MPU_DEVICE_PATH_GYRO_EXT, MPU_DEVICE_PATH_MAG_EXT, &MPU9250_I2C_interface, PX4_I2C_BUS_EXPANSION, PX4_I2C_OBDEV_MPU9250},
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION1)
	{
		MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT1, MPU_DEVICE_PATH_GYRO_EXT1, MPU_DEVICE_PATH_MAG_EXT1, &MPU9250_I2C_interface, PX4_I2C_BUS_EXPANSION1, PX4_I2C_OBDEV_MPU9250,
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION2)
		{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT2, MPU_DEVICE_PATH_GYRO_EXT2, MPU_DEVICE_PATH_MAG_EXT2, &MPU9250_I2C_interface, PX4_I2C_BUS_EXPANSION2, PX4_I2C_OBDEV_MPU9250},
#  endif

#ifdef PX4_SPIDEV_MPU
		{ MPU9250_BUS_SPI_INTERNAL, MPU_DEVICE_PATH_ACCEL, MPU_DEVICE_PATH_GYRO, MPU_DEVICE_PATH_MAG, &MPU9250_SPI_interface, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_MPU},
#endif
#ifdef PX4_SPIDEV_MPU2
		{ MPU9250_BUS_SPI_INTERNAL2, MPU_DEVICE_PATH_ACCEL_1, MPU_DEVICE_PATH_GYRO_1, MPU_DEVICE_PATH_MAG_1, &MPU9250_SPI_interface, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_MPU2},
#endif
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_MPU)
		{ MPU9250_BUS_SPI_EXTERNAL, MPU_DEVICE_PATH_ACCEL_EXT, MPU_DEVICE_PATH_GYRO_EXT, MPU_DEVICE_PATH_MAG_EXT, &MPU9250_SPI_interface, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_MPU},
#endif
	};
}///end namespace


class MPU9250 : public ModuleBaseMulti<MPU9250>
{
public:
	MPU9250(mpu9250::mpu9250_bus_option &options, enum Rotation rotation);
	virtual ~MPU9250();

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static MPU9250 *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief Finds bus options structure associated with the given ID.
	 * @return Returns the bus options structure.
	 */
	static mpu9250::mpu9250_bus_option &initialize_bus(mpu9250::MPU9250_BUS bus_id);

	/** @brief Posts the semaphore to signal the main loop to continue. */
	static void post_semaphore(void *sem);

	/** @brief Resets the mpu9250 such that the FIFO does NOT become corrupt. */
	void clean_reset();

	/** @brief Initializes the driver and ASIC settings. */
	void init();

	/** @brief Starts the driver and kicks off the main loop. */
	void run();

	/**
	 * @brief Prints the status of the driver.
	 * @return Returns PX4_OK.
	 */
	int print_status() override;

	/**
	 * @brief Reads the device ID from the ASIC.
	 * @return Returns PX4_OK if device is found, PX4_ERROR otherwise.
	 */
	int	probe();


private:

	device::Device *_interface = nullptr;

	Accel _accel;
	Gyro _gyro;
	Mag _mag;

	hrt_abstime _mag_poll_time = 0;
	hrt_abstime _mag_interval = 12000;

	uint8_t	_whoami = 0;

	struct hrt_call _hrt_call;

	perf_counter_t		_spi_transfer;
	perf_counter_t		_cycle;
	perf_counter_t		_fifo_maxed;

	unsigned _offset = 1;

	uint8_t	*_dma_data_buffer;

	static constexpr unsigned _dma_buffer_size = 544; // 32B block size

	px4_sem_t _data_semaphore;

	enum Rotation _rotation = ROTATION_NONE;

	/**
	 * @brief 		Read a register.
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);


	/**
	 * @brief 		Write a register.
	 * @param		The register to write.
	 * @param		The value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);


	/**
	 * @brief 		Measurement self test
	 * @return 		PX4_OK on success, PX4_ERROR on failure
	 */
	int 			self_test();
};
