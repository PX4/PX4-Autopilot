/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi08x_read_from_fifo.c
 * @brief   Sample code to read BMI08x sensor data from FIFO
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
/*                       Macro definitions                           */
/*********************************************************************/

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*********************************************************************/
/*                          Global variables                         */
/*********************************************************************/

/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;

/*! @brief variable to hold the bmi08x accel data */
struct bmi08x_sensor_data bmi08x_accel;

/*! @brief variable to hold the bmi08x gyro data */
struct bmi08x_sensor_data bmi08x_gyro;

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
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @return status
 *
 */
int main(void)
{
    int8_t rslt;

    /* To configure the FIFO accel configurations */
    struct bmi08x_accel_fifo_config config;

    /* Read the x,y,z axis data */
    struct bmi08x_sensor_data bmi08x_accel[100] = { { 0 } };

    /* Initialize FIFO frame structure */
    struct bmi08x_fifo_frame fifo_frame = { 0 };

    /* Initialize interrupt configurations */
    struct bmi08x_accel_int_channel_cfg int_config;

    /* Number of accelerometer frames */
    uint16_t accel_length = 100;

    /* Number of bytes of FIFO data */
    uint8_t fifo_data[1024] = { 0 };

    /* Variable to set water mark level */
    uint16_t wml = 0;

    /* Variable to index bytes */
    uint16_t idx = 0;

    uint8_t pwr_data;

    float x, y, z;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated */

    /* Interface given as parameter
     *           For I2C : BMI08X_I2C_INTF
     *           For SPI : BMI08X_SPI_INTF
     * Sensor variant given as parameter
     *          For BMI085 : BMI085_VARIANT
     *          For BMI088 : BMI088_VARIANT
     */
    rslt = bmi08x_interface_init(&bmi08xdev, BMI08X_I2C_INTF, BMI085_VARIANT);
    bmi08x_error_codes_print_result("bmi08x_interface_init", rslt);

    if (rslt == BMI08X_OK)
    {
        /* Initialize bmi08a */
        rslt = bmi08a_init(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_init", rslt);

        /* Initialize bmi08g */
        rslt = bmi08g_init(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08g_init", rslt);

        /* To get power mode */
        rslt = bmi08a_get_power_mode(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_get_power_mode", rslt);

        /* Switch on accelerometer */
        bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
        rslt = bmi08a_set_power_mode(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_power_mode", rslt);

        /* To uploads the bmi08a config file */
        rslt = bmi08a_load_config_file(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_load_config_file", rslt);

        rslt = bmi08a_get_regs(BMI08X_REG_ACCEL_PWR_CONF, &pwr_data, 1, &bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_get_regs", rslt);

        bmi08xdev.delay_us(BMI08X_MS_TO_US(50), bmi08xdev.intf_ptr_accel);

        bmi08xdev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
        bmi08xdev.accel_cfg.odr = BMI08X_ACCEL_ODR_200_HZ;

        /* set odr,bandwidth and range */
        rslt = bmi08a_set_meas_conf(&bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_meas_conf", rslt);

        bmi08xdev.delay_us(BMI08X_MS_TO_US(100), bmi08xdev.intf_ptr_accel);

        /* Configure the Interrupt configurations (int_channel,int_type, int_pin_config_level,
         * output and enable int pin) */
        int_config.int_channel = BMI08X_INT_CHANNEL_1;
        int_config.int_type = BMI08X_FIFO_WM_INT;
        int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

        /* Set the interrupt configuration */
        rslt = bmi08a_set_int_config(&int_config, &bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

        /* Set water mark(aka 6 frames, each 7 bytes: 1 byte header + 6 bytes accel data) */
        wml = 1000;
        rslt = bmi08a_set_fifo_wm(wml, &bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_fifo_wm", rslt);

        printf("watermark_data: %d\n", wml);

        /* Update FIFO structure */
        fifo_frame.data = fifo_data;
        fifo_frame.length = 1024;

        config.accel_en = BMI08X_ENABLE;

        /* Set FIFO configuration by enabling accelerometer */
        rslt = bmi08a_set_fifo_config(&config, &bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_set_fifo_config", rslt);

        bmi08xdev.delay_us(BMI08X_MS_TO_US(1000), bmi08xdev.intf_ptr_accel);

        /* Read FIFO data */
        rslt = bmi08a_read_fifo_data(&fifo_frame, &bmi08xdev);
        bmi08x_error_codes_print_result("bmi08a_read_fifo_data", rslt);

        if (rslt == BMI08X_OK)
        {
            printf("Requested data frames before parsing: %d\t\n", accel_length);

            /* Parse the FIFO data to extract accelerometer data from the FIFO buffer */
            rslt = bmi08a_extract_accel(bmi08x_accel, &accel_length, &fifo_frame, &bmi08xdev);
            bmi08x_error_codes_print_result("bmi08a_extract_accel", rslt);

            printf("Parsed accelerometer frames: %d\r\n", accel_length);

            printf("Accel data in LSB units and Gravity data in m/s^2\n");

            /* Print the parsed accelerometer data from the FIFO buffer */
            for (idx = 0; idx < accel_length; idx++)
            {
                printf("ACCEL[%d] X : %d raw LSB\t Y : %d raw LSB\t Z : %d raw LSB\n",
                       idx,
                       bmi08x_accel[idx].x,
                       bmi08x_accel[idx].y,
                       bmi08x_accel[idx].z);

                /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                x = lsb_to_mps2(bmi08x_accel[idx].x, 2, 16);
                y = lsb_to_mps2(bmi08x_accel[idx].y, 2, 16);
                z = lsb_to_mps2(bmi08x_accel[idx].z, 2, 16);

                /* Print the data in m/s2. */
                printf("\t  Gravity-x = %4.2f, Gravity-y = %4.2f, Gravity-z = %4.2f\n", x, y, z);
            }
        }
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
