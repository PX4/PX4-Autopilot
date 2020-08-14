/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bmi08x_defs.h"

/******************************************************************************/
/*!                       Macro definitions                                   */
#define BMI08X_READ_WRITE_LEN  UINT8_C(46)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection for accel */
uint8_t acc_dev_add;

/*! Variable that holds the I2C device address or SPI chip selection for gyro */
uint8_t gyro_dev_add;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * @brief Function for initialization of I2C bus.
 */
int8_t user_i2c_init(void)
{

    /* Implement I2C bus initialization according to the target machine. */
    return 0;
}

/*!
 * @brief Function for initialization of SPI bus.
 */
int8_t user_spi_init(void)
{

    /* Implement SPI bus initialization according to the target machine. */
    return 0;
}

/*!
 * @brief Function for reading the sensor's registers through SPI bus.
 */
int8_t user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Implement the SPI read routine according to the target machine. */
    return 0;
}

/*!
 * @brief Function for reading the sensor's registers through I2C bus.
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Implement the I2C read routine according to the target machine. */
    return 0;
}

/*!
 * @brief Function for writing the sensor's registers through SPI bus.
 */
int8_t user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Implement the SPI write routine according to the target machine. */
    return 0;
}

/*!
 * @brief Function for writing the sensor's registers through I2C bus.
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Implement the I2C write routine according to the target machine. */
    return 0;
}

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs.
 */
void user_delay(uint32_t period_us, void *intf_ptr)
{
    /* Implement the delay routine according to the target machine. */
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bmi08x_interface_init(struct bmi08x_dev *bmi08x, uint8_t intf, uint8_t variant)
{
    int8_t rslt = BMI08X_OK;

    if (bmi08x != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMI08X_I2C_INTF)
        {
            printf("I2C Interface \n");

            /* To initialize the user I2C function */
            user_i2c_init();

            /* To initialize the user I2C function */
            acc_dev_add = BMI08X_ACCEL_I2C_ADDR_PRIMARY;
            gyro_dev_add = BMI08X_GYRO_I2C_ADDR_PRIMARY;
            bmi08x->intf = BMI08X_I2C_INTF;
            bmi08x->read = user_i2c_read;
            bmi08x->write = user_i2c_write;
        }

        /* Bus configuration : SPI */
        else if (intf == BMI08X_SPI_INTF)
        {
            printf("SPI Interface \n");

            /* To initialize the user SPI function */
            user_spi_init();

            /* To initialize the user SPI function */
            bmi08x->intf = BMI08X_SPI_INTF;
            bmi08x->read = user_spi_read;
            bmi08x->write = user_spi_write;

            /* SPI chip select pin for Accel */
            acc_dev_add = 0;

            /* SPI chip select pin for Gyro */
            gyro_dev_add = 1;
        }

        /* Selection of bmi085 or bmi088 sensor variant */
        bmi08x->variant = variant;

        /* Assign accel device address to accel interface pointer */
        bmi08x->intf_ptr_accel = &acc_dev_add;

        /* Assign gyro device address to gyro interface pointer */
        bmi08x->intf_ptr_gyro = &gyro_dev_add;

        /* Configure delay in microseconds */
        bmi08x->delay_us = user_delay;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bmi08x->read_write_len = BMI08X_READ_WRITE_LEN;

    }
    else
    {
        rslt = BMI08X_E_NULL_PTR;
    }

    return rslt;

}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi08x_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMI08X_OK)
    {
        printf("%s\t", api_name);
        if (rslt & BMI08X_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt & BMI08X_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt & BMI08X_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt & BMI08X_E_OUT_OF_RANGE)
        {
            printf("Error [%d] : Out of Range\r\n", rslt);
        }
        else if (rslt & BMI08X_E_INVALID_INPUT)
        {
            printf("Error [%d] : Invalid input\r\n", rslt);
        }
        else if (rslt & BMI08X_E_CONFIG_STREAM_ERROR)
        {
            printf("Error [%d] : Config stream error\r\n", rslt);
        }
        else if (rslt & BMI08X_E_RD_WR_LENGTH_INVALID)
        {
            printf("Error [%d] : Invalid Read write length\r\n", rslt);
        }
        else if (rslt & BMI08X_E_INVALID_CONFIG)
        {
            printf("Error [%d] : Invalid config\r\n", rslt);
        }
        else if (rslt & BMI08X_E_FEATURE_NOT_SUPPORTED)
        {
            printf("Error [%d] : Feature not supported\r\n", rslt);
        }
        else
        {
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}
