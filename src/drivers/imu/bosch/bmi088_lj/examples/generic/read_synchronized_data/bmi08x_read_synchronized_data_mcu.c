/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file    read_synchronized_data_mcu.c
 * @brief   Sample file how to read bmi08x synchronized sensor data
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "bmi08x.h"
#include "bmi08x_common.h"

/*********************************************************************/
/* macro definitions */
/*********************************************************************/

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*********************************************************************/
/* global variables */
/*********************************************************************/
unsigned char data_sync_int = false;

/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;

/*! bmi08x int config */
struct bmi08x_int_cfg int_config;

/*Data Sync configuration object*/
struct bmi08x_data_sync_cfg sync_cfg;

/*! @brief variable to hold the bmi08x accel data */
struct bmi08x_sensor_data bmi08x_accel;

/*! @brief variable to hold the bmi08x gyro data */
struct bmi08x_sensor_data bmi08x_gyro;

/*********************************************************************/
/* function declarations */
/*********************************************************************/

/*!
 *  @brief This internal function converts lsb to meter per second squared for 16 bit accelerometer for
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Gravity.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/*!
 * @brief    This internal API is used to initialize the bmi08x sensor
 */
static int8_t init_bmi08x(void);

/*!
 * @brief    BMI08x data sync. interrupt callback
 */
void bmi08x_data_sync_int();

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi08x sensor with default.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t init_bmi08x(void)
{
    int8_t rslt;

    /* Initialize bmi08a */
    rslt = bmi08a_init(&bmi08xdev);
    bmi08x_error_codes_print_result("bmi08a_init", rslt);

    /* Initialize bmi08g */
    rslt = bmi08g_init(&bmi08xdev);
    bmi08x_error_codes_print_result("bmi08g_init", rslt);

    if (rslt == BMI08X_OK)
    {
        printf("BMI08x initialization success!\n");
        printf("Accel chip ID - 0x%x\n", bmi08xdev.accel_chip_id);
        printf("Gyro chip ID - 0x%x\n", bmi08xdev.gyro_chip_id);

        /* Reset the accelerometer */
        rslt = bmi08a_soft_reset(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_soft_reset", rslt);

        /* Wait for 1 ms - delay taken care inside the function*/

        /*! Max read/write length (maximum supported length is 32).
         To be set by the user */
        bmi08xdev.read_write_len = 32;

        /*set accel power mode */
        bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
        rslt = bmi08a_set_power_mode(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_power_mode", rslt);

        if (rslt == BMI08X_OK)
        {
            bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
            rslt = bmi08g_set_power_mode(&bmi08xdev);
            bmi08x_error_codes_print_result("bmi08g_set_power_mode", rslt);
        }

        if ((bmi08xdev.accel_cfg.power == BMI08X_ACCEL_PM_ACTIVE) &&
            (bmi08xdev.gyro_cfg.power == BMI08X_GYRO_PM_NORMAL))
        {
            printf("Uploading BMI08X data synchronization feature config !\n");

            /*API uploads the bmi08x config file onto the device*/
            if (rslt == BMI08X_OK)
            {
                rslt = bmi08a_load_config_file(&bmi08xdev);
                bmi08x_error_codes_print_result("bmi08a_load_config_file", rslt);

                /* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
                if (rslt == BMI08X_OK)
                {
                    /*for data synchronization bandwidth and odr will be configured based on the selected sync mode  */
                    if (bmi08xdev.variant == BMI085_VARIANT)
                    {
                        /*assign accel range setting*/
                        bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_4G;
                    }
                    else if (bmi08xdev.variant == BMI088_VARIANT)
                    {
                        bmi08xdev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
                    }

                    /*assign gyro range setting*/
                    bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;

                    /*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
                    sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_400HZ;

                    rslt = bmi08a_configure_data_synchronization(sync_cfg, &bmi08xdev);
                    bmi08x_error_codes_print_result("bmi08a_configure_data_synchronization", rslt);
                }
            }

            if (rslt == BMI08X_OK)
            {
                printf("BMI08x data synchronization feature configured !\n");
            }
            else
            {
                printf("BMI08x data synchronization feature configuration failure!\n");
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This internal API is used to enable data synchronization interrupt.
 *
 *  @return int8_t
 *
 */
static int8_t enable_bmi08x_data_synchronization_interrupt()
{
    int8_t rslt = BMI08X_OK;

    /*set accel interrupt pin configuration*/
    /*configure host data ready interrupt */
    int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
    int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
    int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    /*configure Accel syncronization input interrupt pin */
    int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
    int_config.accel_int_config_2.int_type = BMI08X_ACCEL_SYNC_DATA_RDY_INT;
    int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    /*set gyro interrupt pin configuration*/
    int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
    int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;
    int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
    int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

    int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
    int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;
    int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
    int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

    /* Enable synchronization interrupt pin */
    rslt = bmi08a_set_data_sync_int_config(&int_config, &bmi08xdev);
    bmi08x_error_codes_print_result("bmi08a_set_data_sync_int_config", rslt);

    return rslt;
}

/*!
 *  @brief This internal API is used to disable data synchronization interrupt.
 *
 *  @param[in] void
 *
 *  @return int8_t
 *
 */
static int8_t disable_bmi08x_data_synchronization_interrupt()
{
    int8_t rslt;

    /*turn off the sync feature*/
    sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_OFF;

    rslt = bmi08a_configure_data_synchronization(sync_cfg, &bmi08xdev);
    bmi08x_error_codes_print_result("bmi08a_configure_data_synchronization", rslt);

    /* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
    /* configure synchronization interrupt pins */
    if (rslt == BMI08X_OK)
    {
        /*set accel interrupt pin configuration*/
        /*configure host data ready interrupt */
        int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
        int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
        int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        /*configure Accel synchronization input interrupt pin */
        int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
        int_config.accel_int_config_2.int_type = BMI08X_ACCEL_SYNC_DATA_RDY_INT;
        int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        /*set gyro interrupt pin configuration*/
        int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
        int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;
        int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
        int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;
        int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
        int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

        /* Disable synchronization interrupt pin */
        rslt = bmi08a_set_data_sync_int_config(&int_config, &bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_data_sync_int_config", rslt);
    }

    return rslt;
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @return status
 *
 */
int main(int argc, char *argv[])
{
    int8_t rslt;
    float x, y, z;

    /* Interface given as parameter
     *           For I2C : BMI08X_I2C_INTF
     *           For SPI : BMI08X_SPI_INTF
     * Sensor variant given as parameter
     *          For BMI085 : BMI085_VARIANT
     *          For BMI088 : BMI088_VARIANT
     */
    rslt = bmi08x_interface_init(&bmi08xdev, BMI08X_I2C_INTF, BMI085_VARIANT);
    bmi08x_error_codes_print_result("bmi08x_interface_init", rslt);

    /*initialize the sensors*/
    init_bmi08x();

    /*Enable data ready interrupts*/
    enable_bmi08x_data_synchronization_interrupt();

    if (data_sync_int == true)
    {
        data_sync_int = false;

        rslt = bmi08a_get_synchronized_data(&bmi08x_accel, &bmi08x_gyro, &bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_get_synchronized_data", rslt);

        printf("\nACCEL  X : %d\t Y : %d\t Z : %d\t;\t GYRO  X : %d\t Y : %d\t Z : %d\t Timestamp : %lu\n",
               bmi08x_accel.x,
               bmi08x_accel.y,
               bmi08x_accel.z,
               bmi08x_gyro.x,
               bmi08x_gyro.y,
               bmi08x_gyro.z);

        /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
        x = lsb_to_mps2(bmi08x_accel.x, 2, 16);
        y = lsb_to_mps2(bmi08x_accel.y, 2, 16);
        z = lsb_to_mps2(bmi08x_accel.z, 2, 16);

        /* Print the data in m/s2. */
        printf("Gravity-x = %4.2f, Gravity-y = %4.2f, Gravity-z = %4.2f\t", x, y, z);

        /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
        x = lsb_to_dps(bmi08x_gyro.x, 2000, 16);
        y = lsb_to_dps(bmi08x_gyro.y, 2000, 16);
        z = lsb_to_dps(bmi08x_gyro.z, 2000, 16);

        /* Print the data in dps. */
        printf("DPS-x = %4.2f, DPS-y = %4.2f, DPS-z = %4.2f\n", x, y, z);
    }

    /*disable data ready interrupts*/
    disable_bmi08x_data_synchronization_interrupt();

    return rslt;
}

/* BMI08x data sync. interrupt callback */
void bmi08x_data_sync_int()
{
    data_sync_int = true;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    float gravity;

    uint32_t half_scale = ((1 << bit_width) / 2);

    gravity = (float)((GRAVITY_EARTH * val * g_range) / half_scale);

    return gravity;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI08X_GYRO_RANGE_2000_DPS)) * (val);
}
