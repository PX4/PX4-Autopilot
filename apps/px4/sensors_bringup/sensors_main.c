/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/spi.h>
#include <nuttx/i2c.h>

#include "sensors.h"

__EXPORT int sensors_bringup_main(int argc, char *argv[]);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: user_start/adc_main
 ****************************************************************************/

int sensors_bringup_main(int argc, char *argv[])
{
	struct spi_dev_s *spi;
	int result = -1;

	spi = up_spiinitialize(1);
	if (!spi) {
		message("Failed to initialize SPI port 1\n");
		goto out;
	}

	struct i2c_dev_s *i2c;
	i2c = up_i2cinitialize(2);
	if (!i2c) {
		message("Failed to initialize I2C bus 2\n");
		goto out;
	}

	int ret;

#define EEPROM_ADDRESS		0x50
#define HMC5883L_ADDRESS	0x1E

	//uint8_t devaddr = EEPROM_ADDRESS;

	I2C_SETFREQUENCY(i2c, 100000);
//
//	uint8_t subaddr = 0x00;
//	int ret = 0;
//
//	// ATTEMPT HMC5883L CONFIG
//	I2C_SETADDRESS(i2c, HMC5883L_ADDRESS, 7);
//	subaddr = 0x02; // mode register
//	ret = I2C_WRITE(i2c, &subaddr, 0);
//	if (ret < 0)
//	{
//		message("I2C_WRITE failed: %d\n", ret);
//	}
//	else
//	{
//		message("I2C_WRITE SUCCEEDED: %d\n", ret);
//	}

	//fflush(stdout);
//
//
//


#define STATUS_REGISTER		0x09 // Of HMC5883L

	// ATTEMPT HMC5883L WRITE
	I2C_SETADDRESS(i2c, HMC5883L_ADDRESS, 7);
	uint8_t cmd = 0x09;
	uint8_t status_id[4] = {0, 0, 0, 0};


	ret = I2C_WRITEREAD(i2c, &cmd, 1, status_id, 4);

	if (ret >= 0 && status_id[1] == 'H' && status_id[2] == '4' && status_id[3] == '3')
	{
		message("HMC5883L identified, device status: %d\n", status_id[0]);
	} else {
		message("HMC5883L identification failed: %d\n", ret);
	}

#define HMC5883L_ADDR_CONF_A				0x00
#define HMC5883L_ADDR_CONF_B				0x01
#define HMC5883L_ADDR_MODE					0x02

#define HMC5883L_AVERAGING_1		(0 << 5) /* conf a register */
#define HMC5883L_AVERAGING_2		(1 << 5)
#define HMC5883L_AVERAGING_4		(2 << 5)
#define HMC5883L_AVERAGING_8		(3 << 5)

#define HMC5883L_RATE_75HZ			(6 << 2) /* 75 Hz */

#define HMC5883L_RANGE_0_88GA			(0 << 5)

	uint8_t rate_cmd[] = {HMC5883L_ADDR_CONF_A, HMC5883L_RATE_75HZ | HMC5883L_AVERAGING_8};
	ret = I2C_WRITE(i2c, rate_cmd, sizeof(rate_cmd));
	message("Wrote %d into register 0x00 of HMC, result: %d (0 = success)\n", HMC5883L_RATE_75HZ | HMC5883L_AVERAGING_8, ret);

	uint8_t range_cmd[] = {HMC5883L_ADDR_CONF_B, HMC5883L_RANGE_0_88GA};
	ret = I2C_WRITE(i2c, range_cmd, sizeof(range_cmd));
	message("Wrote %d into register 0x01 of HMC, result: %d (0 = success)\n", HMC5883L_RANGE_0_88GA, ret);

	// Set HMC into continous mode
	// First write address, then value
	uint8_t cont_address[] = {HMC5883L_ADDR_MODE, 0x00};
	ret = I2C_WRITE(i2c, cont_address, sizeof(cont_address));

	message("Wrote 0x00 into register 0x02 of HMC, result: %d (0 = success)\n", ret);


	// ATTEMPT HMC5883L READ
	int h = 0;

	I2C_SETADDRESS(i2c, HMC5883L_ADDRESS, 7);
	for (h = 0; h < 5; h++)
	{

		cont_address[0] = HMC5883L_ADDR_MODE;
		cont_address[1] = 0x01;
		ret = I2C_WRITE(i2c, cont_address, sizeof(cont_address));

		message("Wrote 0x01 into register 0x02 of HMC, result: %d (0 = success)\n", ret);

		usleep(100000);

		cont_address[1] = 0x00;
		uint8_t dummy;
		ret = I2C_WRITEREAD(i2c, cont_address, sizeof(cont_address), &dummy, 1);

		message("Wrote 0x00 into register 0x02 of HMC, result: %d (0 = success)\n", ret);

		usleep(100000);


		int16_t hmc5883l_data[3] = {0, 0, 0};
		uint8_t data_address = 0x03;
		uint8_t* data_ptr = (uint8_t*)hmc5883l_data;
		ret = I2C_WRITEREAD(i2c, &data_address, 1, data_ptr, 6);
		if (ret < 0)
		{
			message("HMC5883L READ failed: %d\n", ret);
		}
		else
		{
			// mask out top four bits as only 12 bits are valid
			hmc5883l_data[0] &= 0xFFF;
			hmc5883l_data[1] &= 0xFFF;
			hmc5883l_data[2] &= 0xFFF;

			message("HMC5883L READ SUCCEEDED: %d, val: %d %d %d\n", ret, hmc5883l_data[0], hmc5883l_data[1], hmc5883l_data[2]);
			uint8_t hmc_status;
			ret = I2C_WRITEREAD(i2c, &cmd, 1, &hmc_status, 1);

			message("\t status: %d\n", hmc_status);
		}
	}


	// Possible addresses: 0x77 or 0x76
#define MS5611_ADDRESS_1	0x76
#define MS5611_ADDRESS_2	0x77
	I2C_SETADDRESS(i2c, MS5611_ADDRESS_1, 7);
	// Reset cmd
	uint8_t ms5611_cmd[2] = {0x00, 0x1E};
	ret = I2C_WRITE(i2c, ms5611_cmd, 2);
	if (ret < 0)
	{
		message("MS5611 #1 WRITE failed: %d\n", ret);
	}
	else
	{
		message("MS5611 #1 WRITE SUCCEEDED: %d\n", ret);
	}

	fflush(stdout);

	I2C_SETADDRESS(i2c, MS5611_ADDRESS_2, 7);
	ret = I2C_WRITE(i2c, ms5611_cmd, 2);
	if (ret < 0)
	{
		message("MS5611 #2 WRITE failed: %d\n", ret);
	}
	else
	{
		message("MS5611 #2 WRITE SUCCEEDED: %d\n", ret);
	}

	fflush(stdout);


	// Wait for reset to complete (10 ms nominal, wait: 100 ms)
	usleep(100000);

	// Read PROM data
	uint8_t prom_buf[2] = {0,1};

	uint16_t calibration[6];

	int i = 0;

	prom_buf[0] = 0xA2 + (i*2);

	struct i2c_msg_s msgv[2] = {
	        {
	            .addr   = MS5611_ADDRESS_2,
	            .flags  = 0,
	            .buffer = prom_buf,
	            .length = 1
	        },
	        {
	            .addr   = MS5611_ADDRESS_2,
	            .flags  = I2C_M_READ,
	            .buffer = prom_buf,
	            .length = 1
	        }
	    };

	calibration[i] = prom_buf[0]*256;
	calibration[i]+= prom_buf[1];

	int retval;

	if ( (retval = I2C_TRANSFER(i2c, msgv, 2)) == OK )
	{
		printf("SUCCESS ACCESSING PROM OF MS5611: %d, value C1: %d\n", retval, (int)calibration[0]);
	}
	else
	{
		printf("FAIL ACCESSING PROM OF MS5611\n");
	}




	// TESTING CODE, EEPROM READ/WRITE
	uint8_t val[1] = {10};
	int retval_eeprom;
	uint8_t eeprom_subaddr[2] = {0, 0};

	struct i2c_msg_s msgv_eeprom[2] = {
	        {
	            .addr   = EEPROM_ADDRESS,
	            .flags  = 0,
	            .buffer = eeprom_subaddr,
	            .length = 2
	        },
	        {
	            .addr   = EEPROM_ADDRESS,
	            .flags  = I2C_M_READ,
	            .buffer = val,
	            .length = 1
	        }
	    };

	val[0] = 5;

	if ( (retval_eeprom = I2C_TRANSFER(i2c, msgv_eeprom, 2)) == OK )
	{
		printf("SUCCESS READING EEPROM: %d, value: %d\n", retval_eeprom, (int)val[0]);
	}
	else
	{
		printf("FAIL READING EEPROM: %d, value: %d\n", retval_eeprom, (int)val[0]);
	}

	// Increment val
	val[0] = val[0] + 1;

	struct i2c_msg_s msgv_eeprom_write[2] = {
		        {
		            .addr   = EEPROM_ADDRESS,
		            .flags  = I2C_M_NORESTART,
		            .buffer = eeprom_subaddr,
		            .length = 2
		        },
		        {
		            .addr   = EEPROM_ADDRESS,
		            .flags  = I2C_M_NORESTART,
		            .buffer = val,
		            .length = 1
		        }
	};


	if ( (retval_eeprom = I2C_TRANSFER(i2c, msgv_eeprom_write, 2)) == OK )
	{
		printf("SUCCESS WRITING EEPROM: %d\n", retval_eeprom);
	}

	usleep(10000);

	struct i2c_msg_s msgv_eeprom2[2] = {
	        {
	            .addr   = EEPROM_ADDRESS,
	            .flags  = 0,
	            .buffer = eeprom_subaddr,
	            .length = 2
	        },
	        {
	            .addr   = EEPROM_ADDRESS,
	            .flags  = I2C_M_READ,
	            .buffer = val,
	            .length = 1
	        }
	    };

	val[0] = 5;


	if ( (retval_eeprom = I2C_TRANSFER(i2c, msgv_eeprom2, 2)) == OK )
	{
		printf("SUCCESS READING WRITE RESULT EEPROM: %d, value: %d\n", retval_eeprom, (int)val[0]);
	}
	else
	{
		printf("FAIL READING WRITE RESULT EEPROM: %d, value: %d\n", retval_eeprom, (int)val[0]);
	}

	// Configure sensors
	l3gd20_test_configure(spi);
	bma180_test_configure(spi);

	for (int i = 0; i < 3; i++)
	{
		l3gd20_test_read(spi);
		bma180_test_read(spi);
		printf("# %d of 10\n", i+1);
		usleep(50000);
	}


	out:
   	msgflush();
	return result;
}
