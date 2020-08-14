/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file       bmi08g.c
 * @date       2020-06-26
 * @version    v1.5.3
 *
 */

/*! \file bmi08g.c
 * \brief Sensor Driver for BMI08X family of sensors */

/****************************************************************************/

/**\name        Header files
 ****************************************************************************/
#include "bmi08x.h"

/****************************************************************************/

/**\name        Local structures
 ****************************************************************************/

/****************************************************************************/

/*! Static Function Declarations
 ****************************************************************************/

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t null_ptr_check(const struct bmi08x_dev *dev);

/*!
 *  @brief This API reads the data from the given register address of gyro sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out]reg_data  : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *data, uint32_t len, struct bmi08x_dev *dev);

/*!
 *  @brief This API writes the given data to the register address
 *  of gyro sensor.
 *
 *  @param[in] reg_addr  : Register address to where the data to be written.
 *  @param[in] reg_data  : Pointer to data buffer which is to be written
 *  in the sensor.
 *  @param[in] len       : No. of bytes of data to write.
 *  @param[in] dev       : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, struct bmi08x_dev *dev);

/*!
 * @brief This API sets the data ready interrupt for gyro sensor.
 *
 * @param[in] int_config  : Structure instance of bmi08x_gyro_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_gyro_data_ready_int(const struct bmi08x_gyro_int_channel_cfg *int_config, struct bmi08x_dev *dev);

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 *
 * @param[in] int_config  : Structure instance of bmi08x_gyro_int_channel_cfg.
 * @param[in] dev         : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_int_pin_config(const struct bmi08x_gyro_int_channel_cfg *int_config, struct bmi08x_dev *dev);

/*!
 *  @brief This API enables or disables the Gyro Self test feature in the
 *  sensor.
 *
 *  @param[in] selftest : Variable used to enable or disable
 *  the Gyro self test feature
 *  Value   |  Description
 *  --------|---------------
 *  0x00    | BMI08X_DISABLE
 *  0x01    | BMI08X_ENABLE
 *
 *  @param[in] dev : Structure instance of bmi08x_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 *
 */
static int8_t set_gyro_selftest(uint8_t selftest, struct bmi08x_dev *dev);

/****************************************************************************/

/**\name        Extern Declarations
 ****************************************************************************/

/****************************************************************************/

/**\name        Globals
 ****************************************************************************/

/****************************************************************************/

/**\name        Function definitions
 ****************************************************************************/

/*!
 *  @brief This API is the entry point for gyro sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of gyro sensor.
 */
int8_t bmi08g_init(struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08X_OK)
    {
        /* Read gyro chip id */
        rslt = get_regs(BMI08X_REG_GYRO_CHIP_ID, &chip_id, 1, dev);

        if (rslt == BMI08X_OK)
        {
            if (chip_id == BMI08X_GYRO_CHIP_ID)
            {
                /* Store the chip ID in dev structure */
                dev->gyro_chip_id = chip_id;
            }
            else
            {
                rslt = BMI08X_E_DEV_NOT_FOUND;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address
 * of gyro sensor.
 */
int8_t bmi08g_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmi08x_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI08X_OK) && (reg_data != NULL))
    {
        if (len > 0)
        {
            /* Reading from the register */
            rslt = get_regs(reg_addr, reg_data, len, dev);
        }
        else
        {
            rslt = BMI08X_E_RD_WR_LENGTH_INVALID;
        }
    }
    else
    {
        rslt = BMI08X_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of gyro sensor.
 */
int8_t bmi08g_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, struct bmi08x_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI08X_OK) && (reg_data != NULL))
    {
        if (len > 0)
        {
            /* Writing to the register */
            rslt = set_regs(reg_addr, reg_data, len, dev);
        }
        else
        {
            rslt = BMI08X_E_RD_WR_LENGTH_INVALID;
        }
    }
    else
    {
        rslt = BMI08X_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API resets the gyro sensor.
 */
int8_t bmi08g_soft_reset(struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08X_OK)
    {
        /* Reset gyro device */
        data = BMI08X_SOFT_RESET_CMD;
        rslt = set_regs(BMI08X_REG_GYRO_SOFTRESET, &data, 1, dev);

        if (rslt == BMI08X_OK)
        {
            /* delay 30 ms after writing reset value to its register */
            dev->delay_us(BMI08X_MS_TO_US(BMI08X_GYRO_SOFTRESET_DELAY), dev->intf_ptr_gyro);
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the gyro odr and range from the sensor, store it in the bmi08x_dev
 * structure instance passed by the user.
 */
int8_t bmi08g_get_meas_conf(struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint8_t data[2];

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08X_OK)
    {
        rslt = get_regs(BMI08X_REG_GYRO_RANGE, data, 2, dev);

        if (rslt == BMI08X_OK)
        {
            dev->gyro_cfg.range = data[0];
            dev->gyro_cfg.odr = (data[1] & BMI08X_GYRO_BW_MASK);
            dev->gyro_cfg.bw = dev->gyro_cfg.odr;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the output data rate, range and bandwidth
 * of gyro sensor.
 */
int8_t bmi08g_set_meas_conf(struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint8_t data;
    uint8_t odr, range;
    uint8_t is_range_invalid = FALSE, is_odr_invalid = FALSE;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08X_OK)
    {
        odr = dev->gyro_cfg.odr;
        range = dev->gyro_cfg.range;

        if (odr > BMI08X_GYRO_BW_32_ODR_100_HZ)
        {
            /* Updating the status */
            is_odr_invalid = TRUE;
        }

        if (range > BMI08X_GYRO_RANGE_125_DPS)
        {
            /* Updating the status */
            is_range_invalid = TRUE;
        }

        /* If ODR and Range is valid, write it to gyro config. registers */
        if ((!is_odr_invalid) && (!is_range_invalid))
        {
            /* Read range value from the range register */
            rslt = get_regs(BMI08X_REG_GYRO_BANDWIDTH, &data, 1, dev);

            if (rslt == BMI08X_OK)
            {
                data = BMI08X_SET_BITS_POS_0(data, BMI08X_GYRO_BW, odr);

                /* Write odr value to odr register */
                rslt = set_regs(BMI08X_REG_GYRO_BANDWIDTH, &data, 1, dev);

                if (rslt == BMI08X_OK)
                {
                    /* Read range value from the range register */
                    rslt = get_regs(BMI08X_REG_GYRO_RANGE, &data, 1, dev);

                    if (rslt == BMI08X_OK)
                    {
                        data = BMI08X_SET_BITS_POS_0(data, BMI08X_GYRO_RANGE, range);

                        /* Write range value to range register */
                        rslt = set_regs(BMI08X_REG_GYRO_RANGE, &data, 1, dev);
                    }
                }
            }
        }
        else
        {
            /* Invalid configuration present in ODR, Range */
            rslt = BMI08X_E_INVALID_CONFIG;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the gyro power mode from the sensor,
 * store it in the bmi08x_dev structure instance
 * passed by the user.
 *
 */
int8_t bmi08g_get_power_mode(struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08X_OK)
    {
        rslt = get_regs(BMI08X_REG_GYRO_LPM1, &data, 1, dev);

        if (rslt == BMI08X_OK)
        {
            /* Updating the power mode in the dev structure */
            dev->gyro_cfg.power = data;
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the gyro sensor.
 */
int8_t bmi08g_set_power_mode(struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint8_t power_mode, data;
    uint8_t is_power_switching_mode_valid = TRUE;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08X_OK)
    {
        /*read the previous power state*/
        rslt = get_regs(BMI08X_REG_GYRO_LPM1, &data, 1, dev);

        if (rslt == BMI08X_OK)
        {
            power_mode = dev->gyro_cfg.power;

            /*switching between normal mode and the suspend modes is allowed, it is not possible to switch
             * between suspend and deep suspend and vice versa. Check for invalid power switching (i.e)
             * deep suspend to suspend */
            if ((power_mode == BMI08X_GYRO_PM_SUSPEND) && (data == BMI08X_GYRO_PM_DEEP_SUSPEND))
            {
                /* Updating the status */
                is_power_switching_mode_valid = FALSE;
            }

            /* Check for invalid power switching (i.e) from suspend to deep suspend */
            if ((power_mode == BMI08X_GYRO_PM_DEEP_SUSPEND) && (data == BMI08X_GYRO_PM_SUSPEND))
            {
                /* Updating the status */
                is_power_switching_mode_valid = FALSE;
            }

            /* Check if power switching mode is valid*/
            if (is_power_switching_mode_valid)
            {
                /* Write power to power register */
                rslt = set_regs(BMI08X_REG_GYRO_LPM1, &power_mode, 1, dev);

                if (rslt == BMI08X_OK)
                {
                    /* Time required to switch the power mode */
                    dev->delay_us(BMI08X_MS_TO_US(BMI08X_GYRO_POWER_MODE_CONFIG_DELAY), dev->intf_ptr_gyro);
                }
            }
            else
            {
                /* Updating the error */
                rslt = BMI08X_E_INVALID_INPUT;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the gyro data from the sensor,
 * store it in the bmi08x_sensor_data structure instance
 * passed by the user.
 */
int8_t bmi08g_get_data(struct bmi08x_sensor_data *gyro, struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint8_t data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI08X_OK) && (gyro != NULL))
    {
        /* read gyro sensor data */
        rslt = get_regs(BMI08X_REG_GYRO_X_LSB, data, 6, dev);

        if (rslt == BMI08X_OK)
        {
            lsb = data[0];
            msb = data[1];
            msblsb = (msb << 8) | lsb;
            gyro->x = (int16_t)msblsb; /* Data in X axis */

            lsb = data[2];
            msb = data[3];
            msblsb = (msb << 8) | lsb;
            gyro->y = (int16_t)msblsb; /* Data in Y axis */

            lsb = data[4];
            msb = data[5];
            msblsb = (msb << 8) | lsb;
            gyro->z = (int16_t)msblsb; /* Data in Z axis */
        }
    }
    else
    {
        rslt = BMI08X_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API configures the necessary gyro interrupt
 * based on the user settings in the bmi08x_int_cfg
 * structure instance.
 */
int8_t bmi08g_set_int_config(const struct bmi08x_gyro_int_channel_cfg *int_config, struct bmi08x_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMI08X_OK) && (int_config != NULL))
    {

        switch (int_config->int_type)
        {
            case BMI08X_GYRO_DATA_RDY_INT:
            {
                /* Data ready interrupt */
                rslt = set_gyro_data_ready_int(int_config, dev);
            }
            break;

            default:
                rslt = BMI08X_E_INVALID_CONFIG;
                break;
        }
    }
    else
    {
        rslt = BMI08X_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the self test functionality of the
 *  gyro sensor is working or not.
 */
int8_t bmi08g_perform_selftest(struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, loop_break = 1;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMI08X_OK)
    {
        /* Enable the gyro self-test */
        rslt = set_gyro_selftest(BMI08X_ENABLE, dev);

        if (rslt == BMI08X_OK)
        {
            /* Loop till self-test ready bit is set */
            while (loop_break)
            {
                /* Read self-test register to check if self-test ready bit is set */
                rslt = get_regs(BMI08X_REG_GYRO_SELF_TEST, &data, 1, dev);

                if (rslt == BMI08X_OK)
                {
                    data = BMI08X_GET_BITS(data, BMI08X_GYRO_SELF_TEST_RDY);

                    if (data)
                    {
                        /* If self-test ready bit is set, exit the loop */
                        loop_break = 0;
                    }
                }
                else
                {
                    /* Exit the loop in case of communication failure */
                    loop_break = 0;
                }
            }

            if (rslt == BMI08X_OK)
            {
                /* Read self-test register to check for self-test Ok bit */
                rslt = get_regs(BMI08X_REG_GYRO_SELF_TEST, &data, 1, dev);

                if (rslt == BMI08X_OK)
                {
                    data = BMI08X_GET_BITS(data, BMI08X_GYRO_SELF_TEST_RESULT);

                    rslt = bmi08g_soft_reset(dev);

                    if (rslt == BMI08X_OK)
                    {
                        /* Updating the self test result */
                        rslt = (int8_t) data;
                    }
                }
            }
        }
    }

    return rslt;
}

/*****************************************************************************/
/* Static function definition */

/*! @cond DOXYGEN_SUPRESS */

/* Suppressing doxygen warnings triggered for same static function names present across various sensor variant
 * directories */

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bmi08x_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL) ||
        (dev->intf_ptr_accel == NULL) || (dev->intf_ptr_gyro == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BMI08X_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BMI08X_OK;
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address of gyro sensor.
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmi08x_dev *dev)
{
    int8_t rslt = BMI08X_OK;

    if (dev->intf == BMI08X_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = (reg_addr | BMI08X_SPI_RD_MASK);
    }

    /* read a gyro register */
    dev->intf_rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr_gyro);

    if (dev->intf_rslt != BMI08X_INTF_RET_SUCCESS)
    {
        /* Updating the error */
        rslt = BMI08X_E_COM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address of gyro sensor.
 */
static int8_t set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, struct bmi08x_dev *dev)
{
    int8_t rslt = BMI08X_OK;

    if (dev->intf == BMI08X_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = (reg_addr & BMI08X_SPI_WR_MASK);
    }

    /* write to a gyro register */
    dev->intf_rslt = dev->write(reg_addr, reg_data, len, dev->intf_ptr_gyro);

    if (dev->intf_rslt != BMI08X_INTF_RET_SUCCESS)
    {
        /* Updating the error */
        rslt = BMI08X_E_COM_FAIL;
    }

    return rslt;
}

/*!
 * @brief This API sets the data ready interrupt for gyro sensor.
 */
static int8_t set_gyro_data_ready_int(const struct bmi08x_gyro_int_channel_cfg *int_config, struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint8_t conf, data[2] = { 0 };

    /* read interrupt map register */
    rslt = get_regs(BMI08X_REG_GYRO_INT3_INT4_IO_MAP, &data[0], 1, dev);

    if (rslt == BMI08X_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case BMI08X_INT_CHANNEL_3:

                /* Data to enable new data ready interrupt */
                data[0] = BMI08X_SET_BITS_POS_0(data[0], BMI08X_GYRO_INT3_MAP, conf);
                break;

            case BMI08X_INT_CHANNEL_4:

                /* Data to enable new data ready interrupt */
                data[0] = BMI08X_SET_BITS(data[0], BMI08X_GYRO_INT4_MAP, conf);
                break;

            default:
                rslt = BMI08X_E_INVALID_INPUT;
                break;
        }

        if (rslt == BMI08X_OK)
        {
            /*condition to check disabling the interrupt in single channel when both
             * interrupts channels are enabled*/
            if (data[0] & BMI08X_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4)
            {
                /* Updating the data */
                /* Data to enable new data ready interrupt */
                data[1] = BMI08X_GYRO_DRDY_INT_ENABLE_VAL;
            }
            else
            {
                data[1] = BMI08X_GYRO_DRDY_INT_DISABLE_VAL;
            }

            /* write data to interrupt map register */
            rslt = set_regs(BMI08X_REG_GYRO_INT3_INT4_IO_MAP, &data[0], 1, dev);

            if (rslt == BMI08X_OK)
            {
                /* Configure interrupt pin */
                rslt = set_int_pin_config(int_config, dev);

                if (rslt == BMI08X_OK)
                {
                    /* write data to interrupt control register */
                    rslt = set_regs(BMI08X_REG_GYRO_INT_CTRL, &data[1], 1, dev);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 */
static int8_t set_int_pin_config(const struct bmi08x_gyro_int_channel_cfg *int_config, struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    /* Read interrupt configuration register */
    rslt = get_regs(BMI08X_REG_GYRO_INT3_INT4_IO_CONF, &data, 1, dev);

    if (rslt == BMI08X_OK)
    {
        switch (int_config->int_channel)
        {
            /* Interrupt pin or channel 3 */
            case BMI08X_INT_CHANNEL_3:

                /* Update data with user configured bmi08x_int_cfg structure */
                data = BMI08X_SET_BITS_POS_0(data, BMI08X_GYRO_INT3_LVL, int_config->int_pin_cfg.lvl);
                data = BMI08X_SET_BITS(data, BMI08X_GYRO_INT3_OD, int_config->int_pin_cfg.output_mode);
                break;

            case BMI08X_INT_CHANNEL_4:

                /* Update data with user configured bmi08x_int_cfg structure */
                data = BMI08X_SET_BITS(data, BMI08X_GYRO_INT4_LVL, int_config->int_pin_cfg.lvl);
                data = BMI08X_SET_BITS(data, BMI08X_GYRO_INT4_OD, int_config->int_pin_cfg.output_mode);
                break;

            default:
                break;
        }

        /* write to interrupt configuration register */
        rslt = set_regs(BMI08X_REG_GYRO_INT3_INT4_IO_CONF, &data, 1, dev);
    }

    return rslt;
}

/*!
 *  @brief This API enables or disables the Gyro Self test feature in the
 *  sensor.
 */
static int8_t set_gyro_selftest(uint8_t selftest, struct bmi08x_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check for valid selftest input */
    if ((selftest == BMI08X_ENABLE) || (selftest == BMI08X_DISABLE))
    {
        /* Read self test register */
        rslt = get_regs(BMI08X_REG_GYRO_SELF_TEST, &data, 1, dev);

        if (rslt == BMI08X_OK)
        {
            /* Enable self-test */
            data = BMI08X_SET_BITS_POS_0(data, BMI08X_GYRO_SELF_TEST_EN, selftest);

            /* write self test input value to self-test register */
            rslt = set_regs(BMI08X_REG_GYRO_SELF_TEST, &data, 1, dev);
        }
    }
    else
    {
        rslt = BMI08X_E_INVALID_INPUT;
    }

    return rslt;
}

/*! @endcond */
