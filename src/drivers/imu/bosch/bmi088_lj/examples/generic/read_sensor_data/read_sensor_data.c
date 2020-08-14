/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi08x_read_sensor_data.c
 * @brief   Sample file how to read bmi08x sensor data
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

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
/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;

/*! @brief variable to hold the bmi08x accel data */
struct bmi08x_sensor_data bmi08x_accel;

/*! @brief variable to hold the bmi08x gyro data */
struct bmi08x_sensor_data bmi08x_gyro;

/*! bmi08x accel int config */
struct bmi08x_accel_int_channel_cfg accel_int_config;

/*! bmi08x gyro int config */
struct bmi08x_gyro_int_channel_cfg gyro_int_config;

/*********************************************************************/
/* static function declarations */
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
 * @brief    This internal API is used to initialize the bmi08x sensor with default
 */
static int8_t init_bmi08x(void);

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi08x sensor
 *  settings like power mode and OSRS settings.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t init_bmi08x(void)
{
    int8_t rslt;

    rslt = bmi08a_init(&bmi08xdev);
    bmi08x_error_codes_print_result("bmi08a_init", rslt);

    if (rslt == BMI08X_OK)
    {
        rslt = bmi08g_init(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_init", rslt);
    }

    if (rslt == BMI08X_OK)
    {
        bmi08xdev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;

        if (bmi08xdev.variant == BMI085_VARIANT)
        {
            bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
        }
        else if (bmi08xdev.variant == BMI088_VARIANT)
        {
            bmi08xdev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
        }

        bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
        bmi08xdev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_power_mode", rslt);
        bmi08xdev.delay_us(10, bmi08xdev.intf_ptr_accel);
        rslt = bmi08a_set_meas_conf(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_meas_conf", rslt);
        bmi08xdev.delay_us(10, bmi08xdev.intf_ptr_accel);

        bmi08xdev.gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
        bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_250_DPS;
        bmi08xdev.gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
        bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

        rslt = bmi08g_set_power_mode(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_set_power_mode", rslt);
        bmi08xdev.delay_us(10, bmi08xdev.intf_ptr_gyro);

        rslt = bmi08g_set_meas_conf(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_set_meas_conf", rslt);
        bmi08xdev.delay_us(10, bmi08xdev.intf_ptr_gyro);
    }

    return rslt;

}

/*!
 *  @brief This API is used to enable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t enable_bmi08x_interrupt()
{
    int8_t rslt;

    /*set accel interrupt pin configuration*/
    accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08X_ACCEL_DATA_RDY_INT;
    accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    /*Enable accel data ready interrupt channel*/
    rslt = bmi08a_set_int_config((const struct bmi08x_accel_int_channel_cfg*)&accel_int_config, &bmi08xdev);
    bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

    if (rslt == BMI08X_OK)
    {
        /*set gyro interrupt pin configuration*/
        gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08X_GYRO_DATA_RDY_INT;
        gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

        /*Enable gyro data ready interrupt channel*/
        rslt = bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config, &bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);
    }

    return rslt;
}

/*!
 *  @brief This API is used to disable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static int8_t disable_bmi08x_interrupt()
{
    int8_t rslt;

    /*set accel interrupt pin configuration*/
    accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08X_ACCEL_DATA_RDY_INT;
    accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

    /*Disable accel data ready interrupt channel*/
    rslt = bmi08a_set_int_config((const struct bmi08x_accel_int_channel_cfg*)&accel_int_config, &bmi08xdev);
    bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

    if (rslt == BMI08X_OK)
    {
        /*set gyro interrupt pin configuration*/
        gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08X_GYRO_DATA_RDY_INT;
        gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        /*Disable gyro data ready interrupt channel*/
        rslt = bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config, &bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);
    }

    return rslt;
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @return status
 *
 */
int main(void)
{
    int8_t rslt;

    uint8_t times_to_read = 0;
    float x, y, z;

    /* Interface given as parameter
     *           For I2C : BMI08X_I2C_INTF
     *           For SPI : BMI08X_SPI_INTF
     * Sensor variant given as parameter
     *          For BMI085 : BMI085_VARIANT
     *          For BMI088 : BMI088_VARIANT
     */
    rslt = bmi08x_interface_init(&bmi08xdev, BMI08X_SPI_INTF, BMI085_VARIANT);

    if (rslt == BMI08X_OK)
    {
        rslt = init_bmi08x();

        /*Enable data ready interrupts*/
        rslt = enable_bmi08x_interrupt();

        if (rslt == BMI08X_OK)
        {
            printf("\nACCEL DATA\n");
            printf("Accel data in LSB units and Gravity data in m/s^2\n");
            while (times_to_read < 10)
            {
                rslt = bmi08a_get_data(&bmi08x_accel, &bmi08xdev);
                printf("ACCEL[%d] X : %d raw LSB\t Y : %d raw LSB\t Z : %d raw LSB\n",
                       times_to_read,
                       bmi08x_accel.x,
                       bmi08x_accel.y,
                       bmi08x_accel.z);

                /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                x = lsb_to_mps2(bmi08x_accel.x, 2, 16);
                y = lsb_to_mps2(bmi08x_accel.y, 2, 16);
                z = lsb_to_mps2(bmi08x_accel.z, 2, 16);

                /* Print the data in m/s2. */
                printf("\t  Gravity-x = %4.2f, Gravity-y = %4.2f, Gravity-z = %4.2f\n", x, y, z);

                bmi08xdev.delay_us(10, bmi08xdev.intf_ptr_accel);
                times_to_read = times_to_read + 1;
            }

            times_to_read = 0;

            printf("\n\nGYRO DATA\n");
            printf("Gyro data in LSB units and degrees per second\n");
            while (times_to_read < 10)
            {
                rslt = bmi08g_get_data(&bmi08x_gyro, &bmi08xdev);
                printf("GYRO[%d] X : %d raw LSB\t Y : %d raw LSB\t Z : %d raw LSB\n",
                       times_to_read,
                       bmi08x_gyro.x,
                       bmi08x_gyro.y,
                       bmi08x_gyro.z);

                /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                x = lsb_to_dps(bmi08x_gyro.x, 2000, 16);
                y = lsb_to_dps(bmi08x_gyro.y, 2000, 16);
                z = lsb_to_dps(bmi08x_gyro.z, 2000, 16);

                /* Print the data in dps. */
                printf("\t  DPS-x = %4.2f, DPS-y = %4.2f, DPS-z = %4.2f\n", x, y, z);

                bmi08xdev.delay_us(10, bmi08xdev.intf_ptr_gyro);
                times_to_read = times_to_read + 1;
            }
        }

        /*disable data ready interrupts*/
        rslt = disable_bmi08x_interrupt();
    }

    return rslt;
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
